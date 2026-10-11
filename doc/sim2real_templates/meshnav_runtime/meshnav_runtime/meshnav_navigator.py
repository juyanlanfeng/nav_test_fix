"""接收 RViz 目标，协调 MBF 规划、执行和当前路线的可通行性检查。

节点使用一个执行器处理回调。旧路可通行时继续执行；确认阻断后先停车再申请新路。
短暂无路时在时限内保持停止并重试，不继续执行已知受阻的旧路线。
epoch 区分前后两次导航任务，sequence 区分同一任务内前后两条执行路径。
"""

# 路线裁剪会构造新的 ROS 消息，避免修改仍被控制器使用的原始路径。
from copy import deepcopy

import json  # 将状态及附加字段编码成 JSON，发布到 ~/status。
import math  # 检查数值有效性，并计算路径段的投影和距离。
import time  # 使用墙钟衡量异步请求和位姿反馈的超时。

import rclpy  # 初始化、运行并关闭 ROS 2 节点。
from action_msgs.msg import GoalStatus  # 判断 action 最终是否成功。
from geometry_msgs.msg import PoseStamped  # 接收目标与控制器反馈的位姿。
from mbf_msgs.action import ExePath, GetPath  # MBF 的路径执行和全局规划 action。
from mbf_msgs.srv import CheckPath  # 检查当前路径是否仍可通行的服务。
from rclpy.action import ActionClient  # 向 MBF 发送异步 action 目标。
from rclpy.clock import Clock, ClockType  # 定时器使用不会受仿真时间暂停影响的稳态时钟。
from rclpy.node import Node  # ROS 2 节点基类。
from rclpy.qos import DurabilityPolicy, QoSProfile  # 让迟加入的订阅者读到最近一条状态。
from std_msgs.msg import String  # 状态消息载体。
from std_srvs.srv import Trigger  # 取消当前导航的服务类型。


def remaining_path(path, current, progress, window):
    """截取从车辆当前位置到终点的路段，供 CheckPath 检查。

    path 是控制器正在执行的整条路线；current 是最新反馈位姿；progress 是上次
    已走到的路径段索引；window 限制本次向前搜索的距离，避免在路线交叉处跳段。
    返回新的 Path 和更新后的路径段索引，不修改传入的 path。
    """
    # 没有路径点，或反馈位姿与路径不在同一坐标系时，不能直接比较坐标。
    if not path.poses or current.header.frame_id != path.header.frame_id:
        raise ValueError("Cannot check route: empty path or mismatched feedback frame")

    def xyz(pose):
        """把 PoseStamped 的位置取为便于计算的三维元组。"""
        p = pose.pose.position
        return (p.x, p.y, p.z)

    # 在当前车辆位置附近寻找旧路径上的连接点，初值放在已确认的进度处。
    position = xyz(current)
    # best 依次保存：最小距离平方、对应路径段索引、连接点位姿。
    best = (math.inf, progress, deepcopy(path.poses[progress]))
    # 只累计本次搜索的路径长度，不从每次检查时的整条路径起点重新搜索。
    distance = 0.0
    # 从上次进度起向前搜索，因此进度不会倒退到已经走过的路径段。
    for i in range(progress, len(path.poses) - 1):
        # a、b 是当前路径段两端；delta 是由 a 指向 b 的向量。
        a, b = xyz(path.poses[i]), xyz(path.poses[i + 1])
        delta = tuple(y - x for x, y in zip(a, b))
        # 使用长度平方计算车辆在路径段上的投影比例；零长度段取起点。
        length2 = sum(v * v for v in delta)
        t = max(0.0, min(1.0, sum((p - x) * d for p, x, d in
                                  zip(position, a, delta)) / length2)) if length2 else 0.0
        # 按限制在 [0, 1] 内的比例求路径段上离车辆最近的点。
        point = tuple(x + t * d for x, d in zip(a, delta))
        # 比较车辆与该连接点的三维距离平方，避免不必要的平方根运算。
        error = sum((p - x) ** 2 for p, x in zip(position, point))
        if error < best[0]:
            # 复制原路径点的消息头和姿态，只把位置改成段内连接点。
            join = deepcopy(path.poses[i])
            join.pose.position.x, join.pose.position.y, join.pose.position.z = point
            best = (error, i, join)
        # 超出前视窗口就停止，防止交叉路径附近误选远处的后续路段。
        distance += math.sqrt(length2)
        if distance >= window:
            break
    # 从完整路径复制消息头；下方再替换为未走过的路径点。
    result = deepcopy(path)
    # 把“车辆当前位置 → 连接点”也送去检查，防止偏离旧路时漏掉中间障碍。
    result.poses = [deepcopy(current), best[2]] + list(path.poses[best[1] + 1:])
    # 同时返回进度索引，下一次从这里继续向前搜索。
    return result, best[1]


class MeshnavNavigator(Node):
    """只在当前剩余路线不可通行时更新 MBF 执行路径的协调节点。"""

    def __init__(self):
        # 节点名决定相对话题 ~/status 和服务 ~/cancel 的最终名称。
        super().__init__("meshnav_navigator")
        # YAML 可覆盖这里的默认值；注释中的频率只控制检查，不强制换路。
        defaults = {
            "goal_topic": "/rviz/goal_pose",  # RViz 发送新目标的 PoseStamped 话题。
            "get_path_action": "/move_base_flex/get_path",  # 全局路径规划 action。
            "exe_path_action": "/move_base_flex/exe_path",  # 路径执行 action。
            "planner": "mesh_planner",  # 向 MBF 请求的规划器实例名。
            "controller": "mesh_controller",  # 向 MBF 请求的控制器实例名。
            "planner_frequency": 2.0,  # 旧路检查频率，Hz；0 表示只规划一次。
            "check_path_service": "/move_base_flex/check_path_cost",  # 剩余路线检查服务。
            "pose_timeout": 2.0,  # 控制器反馈位姿允许的最大墙钟间隔，秒。
            "path_progress_window": 2.0,  # 每次在旧路径上向前寻找连接点的长度，米。
            "action_timeout": 15.0,  # 检查、规划或目标接收允许的最长墙钟时间，秒。
            "replan_patience": 15.0,  # 受阻后停车等待有效新路线的总时限，秒（墙钟）。
            "replan_retry_period": 0.5,  # 短暂无路时两次请求的最小间隔，秒（墙钟）。
        }
        # 声明参数并读回最终值，统一存放在 cfg 中供回调使用。
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        # 频率不能为负；超时和搜索长度必须是有限的正数。
        if (not math.isfinite(self.cfg["planner_frequency"]) or
                self.cfg["planner_frequency"] < 0 or any(
                    not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0
                    for key in ("action_timeout", "pose_timeout", "path_progress_window",
                                "replan_patience", "replan_retry_period"))):
            raise ValueError("Frequency must be finite and >= 0; timeouts and progress window > 0")
        # 两个 action 客户端分别负责产生路径和让控制器执行路径。
        self.planner = ActionClient(self, GetPath, self.cfg["get_path_action"])
        self.controller = ActionClient(self, ExePath, self.cfg["exe_path_action"])
        # 服务读取 MeshNav 当前 final 代价，用来判断旧路是否可行。
        self.path_checker = self.create_client(CheckPath, self.cfg["check_path_service"])
        self.check_busy = False  # 是否有尚未返回的路线检查请求。
        self.active_path = None  # 当前交给控制器执行的完整路径；未规划时为空。
        self.current_pose = None  # 当前执行路径对应的最近一次控制器反馈位姿。
        self.pose_received = 0.0  # 上述位姿到达本节点时的墙钟时间。
        self.path_progress = 0  # 上次检查时找到的路径段索引，只向前推进。
        # 状态只保留最新一条，迟加入的监控端也能收到它。
        self.status = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # RViz 目标订阅和用户取消服务；二者只更新任务状态，具体推进由 tick 完成。
        self.create_subscription(PoseStamped, self.cfg["goal_topic"], self.new_goal, 1)
        self.create_service(Trigger, "~/cancel", self.cancel)
        self.epoch = 0  # 每次结束/替换任务时加一，用于丢弃上个任务的迟到回调。
        self.sequence = 0  # 每下发一次 ExePath 加一，区分同一任务的新旧执行路径。
        self.executions = {}  # sequence → 已接收的 ExePath goal handle。
        self.pending_sends = 0  # 已发送但还未得到接收结果的 ExePath 数量。
        self.plan_busy = False  # GetPath 是否仍在接收或计算中。
        self.plan_handle = None  # 已接收的 GetPath goal handle，用于取消。
        self.target = None  # 当前任务的导航目标；None 表示当前没有正在执行的任务。
        self.queued_target = None  # 取消旧任务后待启动的新目标。
        self.distance = math.inf  # 控制器反馈的剩余目标距离，尚未收到时为无穷大。
        self.plan_count = 0  # 此任务真正下发给控制器的路径条数。
        self.last_plan = 0.0  # 上次检查或规划时的 ROS 时间，用于控制检查频率。
        self.request_started = 0.0  # 最近一次异步请求开始的墙钟时间。
        self.stop_started = None  # 开始等待旧任务取消完成的墙钟时间。
        self.stop_state = None  # 最近一次 stop 的原因，用于发布取消完成状态。
        self.waiting_for_replan_stop = False
        self.replan_deadline = None
        self.retry_plan_at = 0.0
        # 20 Hz 轮询任务状态；稳态时钟可在仿真时间暂停时继续处理取消。
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        # 启动时向状态监控端公布空闲状态。
        self.publish_status("idle")

    def publish_status(self, state, **fields):
        """同时向状态话题和日志输出当前任务状态。"""
        # 固定包含 state、当前任务的执行路径数，fields 放具体事件信息。
        text = json.dumps(dict(state=state, plans=self.plan_count, **fields), ensure_ascii=False)
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def stop(self, state, **fields):
        """结束当前任务，并异步取消可能仍在运行的 MBF 目标。"""
        # 先换 epoch：此后旧任务返回的回调不能再更新新任务状态。
        self.epoch += 1
        # 删除旧任务目标、路径和位姿，等待已发出的 action 结束。
        self.target = None
        self.active_path = None
        self.current_pose = None
        self.waiting_for_replan_stop = False
        self.replan_deadline = None
        self.retry_plan_at = 0.0
        # 使用墙钟统计取消等待时间；state 记录这次停止的原因。
        self.stop_started = time.monotonic()
        self.stop_state = state
        # GetPath 已被 MBF 接收时请求取消；尚未被接收的请求由迟到回调处理。
        if self.plan_handle is not None:
            self.plan_handle.cancel_goal_async()
        # 逐条取消已接收的 ExePath；字典复制可避免回调变更迭代对象。
        for handle in list(self.executions.values()):
            handle.cancel_goal_async()
        # 当前状态先公布，实际取消完成由后续 tick 确认。
        self.publish_status(state, **fields)

    def new_goal(self, pose):
        """处理 RViz 新目标：校验后排队，先终止旧任务。"""
        # 把位置和四元数分量放在一起检查，避免 NaN/无穷值流入规划器。
        values = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                  pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
        # 目标必须明确坐标系，所有几何数值必须有限。
        if not pose.header.frame_id or not all(math.isfinite(v) for v in values):
            self.get_logger().error("Rejected goal: missing frame or non-finite pose")
            return
        # 全零四元数不能表示有效方向；这里仅检查长度没有接近零。
        if sum(v * v for v in values[3:]) < 1e-6:
            self.get_logger().error("Rejected goal: zero quaternion")
            return
        # 记下最新目标；tick 会等旧任务完全退出后再启动它。
        self.queued_target = pose
        self.stop("replacing_goal")

    def cancel(self, request, response):
        """响应 ~/cancel：清除待启动目标并请求停止当前任务。"""
        # 清除排队目标，避免取消旧任务后误启动先前收到的新目标。
        self.queued_target = None
        self.stop("cancel_requested")
        # 服务成功表示“取消请求已受理”；实际结束状态另见 ~/status。
        response.success = True
        response.message = "Cancellation requested; see /meshnav_navigator/status for completion"
        return response

    def tick(self):
        """定期推进任务状态机，并按配置频率检查当前剩余路线。"""
        # 判断异步检查、规划、执行目标接收/执行是否仍占用旧任务。
        busy = self.check_busy or self.plan_busy or self.pending_sends or bool(self.executions)
        # 没有正在导航的目标时，先处理停止状态，再考虑排队的新目标。
        if self.target is None:
            if busy:
                # 超时只报告等待取消，并重置报告时间，避免每 50 ms 刷屏。
                if self.stop_started and time.monotonic() - self.stop_started > self.cfg["action_timeout"]:
                    self.publish_status("cancel_timeout", message="Waiting for MBF to finish cancellation")
                    self.stop_started = time.monotonic()
                return
            # 所有旧 action 都结束后，才报告用户主动取消已经完成。
            if self.stop_started is not None:
                self.stop_started = None
                if self.stop_state == "cancel_requested":
                    self.publish_status("canceled")
            # 没有新目标就保持空闲；服务端未就绪时让目标继续排队。
            if self.queued_target is None:
                return
            if not self.planner.server_is_ready() or not self.controller.server_is_ready():
                return
            # 接管排队目标，并重置只属于本次导航的距离和路径计数。
            self.target, self.queued_target = self.queued_target, None
            self.distance = math.inf
            self.plan_count = 0
            self.publish_status("planning", x=self.target.pose.position.x, y=self.target.pose.position.y)
        if self.replan_deadline is not None and time.monotonic() >= self.replan_deadline:
            self.stop("failed", message="No feasible replacement route within replan_patience")
            return
        if self.waiting_for_replan_stop:
            # 旧 action 完成取消前不规划，否则起点会在规划过程中继续移动。
            if self.executions or self.pending_sends:
                return
            self.waiting_for_replan_stop = False
        if time.monotonic() < self.retry_plan_at:
            return
        # 已有检查、规划或新执行目标尚未被接收时，不重复发请求。
        if self.check_busy or self.plan_busy or self.pending_sends:
            # 这里用墙钟发现请求迟迟不返回；随后 stop 会取消仍在执行的旧目标。
            if time.monotonic() - self.request_started > self.cfg["action_timeout"]:
                self.stop("failed", message="Planning or goal acceptance timed out")
            return
        # 检查频率按 ROS 时间计算；定时器本身使用前述稳态时钟运行。
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.active_path is None:
            self.retry_plan_at = 0.0
            self.request_plan(now)
            return
        frequency = self.cfg["planner_frequency"]
        # 初次规划无需等待；已有路径时按频率间隔进行可行性检查。
        if self.plan_count:
            # 0 表示初次规划后不再做周期性路径检查。
            if frequency == 0:
                return
            if now - self.last_plan < 1.0 / frequency:
                return
        # 现有路线只检查，不因为有更短候选路线就强制切换。
        self.last_plan = now
        self.check_path()

    def check_path(self):
        """异步检查当前剩余路线。"""
        # 必须有足够新的控制器反馈位姿，才能裁掉已经走过的路段。
        if (self.current_pose is None or
                time.monotonic() - self.pose_received > self.cfg["pose_timeout"]):
            self.stop("failed", message="No fresh controller pose for route check")
            return
        # 服务不可用时不把旧路默认为安全通行。
        if not self.path_checker.service_is_ready():
            self.stop("failed", message="Path feasibility service unavailable")
            return
        try:
            # 剩余路径从车辆位置开始，包含车辆回到旧路的连接段。
            path, self.path_progress = remaining_path(
                self.active_path, self.current_pose, self.path_progress,
                self.cfg["path_progress_window"])
        except ValueError as exc:
            # 路径空或反馈位姿坐标系不符时中止导航。
            self.stop("failed", message=str(exc))
            return
        # 只检查路径本身经过的网格；车体占地余量已通过膨胀层配置。
        request = CheckPath.Request()
        request.path = path
        request.path_cells_only = True
        # 查询包含实时障碍的全局 final 网格，不使用局部 costmap。
        request.costmap = CheckPath.Request.GLOBAL_COSTMAP
        request.return_on = CheckPath.Response.LETHAL
        # 标记请求在途，并记录墙钟和发起时所属的任务版本。
        self.check_busy = True
        self.request_started = time.monotonic()
        epoch = self.epoch
        self.path_checker.call_async(request).add_done_callback(
            lambda future: self.path_checked(future, epoch))

    def path_checked(self, future, epoch):
        """可行则保留旧路；受阻则先取消执行，再从停止后的位置规划。"""
        # 请求已经结束；如果所属任务换过了，就忽略这个迟到响应。
        self.check_busy = False
        if epoch != self.epoch:
            return
        try:
            # state 是服务对剩余路径中最坏位置给出的通行状态。
            response = future.result()
            state = response.state
        except Exception as exc:
            # 服务调用出错时停止，避免在未知代价上继续导航。
            self.stop("failed", message="Route check failed: " + str(exc))
            return
        # 旧路仍然畅通：直接维持当前 ExePath。
        if state == CheckPath.Response.FREE:
            return
        # UNKNOWN 等状态不能证实“旧路被阻断”，也不能证实“安全可走”。
        if state not in (CheckPath.Response.LETHAL, CheckPath.Response.OUTSIDE):
            self.stop("failed", message="Route feasibility unknown; navigation stopped")
            return
        self.publish_status("replanning", message="Remaining route is blocked; stopping before replanning",
                            check_state=state, blocked_segment=response.last_checked)
        self.sequence += 1  # 旧执行的取消结果不能结束仍然有效的导航目标。
        self.active_path = None
        self.waiting_for_replan_stop = True
        self.replan_deadline = time.monotonic() + self.cfg["replan_patience"]
        self.retry_plan_at = 0.0
        for handle in list(self.executions.values()):
            handle.cancel_goal_async()

    def request_plan(self, now):
        """请求 MBF 从当前车位规划到本次任务目标。"""
        # 不指定 start_pose，让 MBF 自己通过当前 TF 取得车辆位置。
        goal = GetPath.Goal()
        goal.target_pose = self.target
        # 目标消息的时间戳使用本节点当前 ROS 时间。
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.use_start_pose = False
        # 规划器名称必须对应 MBF 配置中加载的实例名。
        goal.planner = self.cfg["planner"]
        # 标记本次规划在途，并记录频率控制时间和超时起点。
        self.plan_busy = True
        self.last_plan = now
        self.request_started = time.monotonic()
        # 保存所属任务版本，防止目标更换后误处理旧规划结果。
        epoch = self.epoch
        self.planner.send_goal_async(goal).add_done_callback(
            lambda future: self.plan_accepted(future, epoch))

    def plan_accepted(self, future, epoch):
        """处理 GetPath 目标是否被 MBF 接收的异步回调。"""
        try:
            # send_goal_async 的第一阶段只返回目标句柄，不返回实际路径。
            handle = future.result()
        except Exception as exc:
            # 请求发送失败；若已切换任务，错误不属于当前任务。
            self.plan_busy = False
            if epoch == self.epoch:
                self.stop("failed", message=str(exc))
            return
        # MBF 拒绝规划目标时结束当前任务。
        if not handle.accepted:
            self.plan_busy = False
            if epoch == self.epoch:
                self.stop("failed", message="Planner rejected goal")
            return
        # 保存句柄以供取消，并注册第二阶段的规划结果回调。
        self.plan_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.plan_done(f, epoch))
        # 如果在等待接收期间任务已更换，立刻取消迟到的旧目标。
        if epoch != self.epoch:
            handle.cancel_goal_async()

    def plan_done(self, future, epoch):
        """处理从停止位置请求的新路线。"""
        # 规划 action 已结束，清理忙标记和可取消句柄。
        self.plan_busy = False
        self.plan_handle = None
        # 被新目标或取消操作淘汰的规划结果一律忽略。
        if epoch != self.epoch:
            return
        try:
            # wrapped 同时包含 action 状态和具体 GetPath.Result。
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.stop("failed", message=str(exc))
            return
        self.adopt_plan(wrapped, epoch)

    def adopt_plan(self, wrapped, epoch):
        """验证候选规划并向同一个 MBF 控制器下发新执行路径。"""
        # 必须同时满足 action 成功、规划器返回成功和至少有一个路径点。
        result = wrapped.result
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                result.outcome != GetPath.Result.SUCCESS or not result.path.poses):
            if (self.replan_deadline is not None and result.outcome in (
                    GetPath.Result.NO_PATH_FOUND, GetPath.Result.BLOCKED_START,
                    GetPath.Result.BLOCKED_GOAL)):
                self.retry_plan_at = time.monotonic() + self.cfg["replan_retry_period"]
                self.publish_status("waiting_for_path", outcome=result.outcome,
                                    message="Stopped; waiting for a feasible replacement route")
                return
            self.stop("failed", outcome=result.outcome, message="Replan failed: " + result.message)
            return
        self.replan_deadline = None
        self.retry_plan_at = 0.0
        # 保存被采用的完整路径，后续周期检查都基于它。
        self.active_path = result.path
        # 从采用新路的时刻开始计算下次周期检查间隔。
        self.last_plan = self.get_clock().now().nanoseconds * 1e-9
        # 新路径需要重新从其首段累计行驶进度。
        self.path_progress = 0
        self.plan_count += 1
        # 每条下发的 ExePath 使用新序号，旧路径的迟到反馈不会覆盖新状态。
        self.sequence += 1
        sequence = self.sequence
        # ExePath 要使用 MBF 中加载的控制器实例，以及本次采用的路径。
        goal = ExePath.Goal()
        goal.controller = self.cfg["controller"]
        goal.path = result.path
        # 发送完成前先计数，防止 tick 在目标接收期间发送重复请求。
        self.pending_sends += 1
        self.request_started = time.monotonic()
        # 反馈和接收回调都携带任务版本及路径序号，以识别迟到结果。
        self.controller.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback(msg, epoch, sequence)
        ).add_done_callback(lambda f: self.execution_accepted(f, epoch, sequence))
        # 状态里的 plans 已包括刚采用的这条执行路径。
        self.publish_status("following", path_poses=len(result.path.poses), path_cost=result.cost)

    def feedback(self, msg, epoch, sequence):
        """保存当前执行路径的目标距离和车位，供剩余路线检查使用。"""
        # 只接受当前任务、当前 ExePath 的反馈；旧执行的反馈可能迟到。
        if epoch == self.epoch and sequence == self.sequence:
            self.distance = msg.feedback.dist_to_goal
            self.current_pose = msg.feedback.current_pose
            # 保存收到反馈时的墙钟时间，用来判断位姿是否过期。
            self.pose_received = time.monotonic()

    def execution_accepted(self, future, epoch, sequence):
        """处理 ExePath 目标是否被 MBF 接收的异步回调。"""
        # 目标接收请求结束，不再占用 pending_sends 计数。
        self.pending_sends -= 1
        try:
            # 接收阶段返回执行目标句柄，尚不是导航完成结果。
            handle = future.result()
        except Exception as exc:
            if epoch == self.epoch:
                self.stop("failed", message=str(exc))
            return
        # 当前任务的控制目标被拒绝时结束导航。
        if not handle.accepted:
            if epoch == self.epoch:
                self.stop("failed", message="Controller rejected path")
            return
        # 保存句柄以供取消，并等待执行完成的 action 结果。
        self.executions[sequence] = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.execution_done(f, epoch, sequence))
        # 等待接收期间若任务已更换或旧路线已受阻，取消迟到的执行目标。
        if epoch != self.epoch or sequence != self.sequence:
            handle.cancel_goal_async()

    def execution_done(self, future, epoch, sequence):
        """处理 ExePath 的最终结果，并忽略被新路径替换的旧结果。"""
        # 此序号的 action 已结束，从仍需等待的执行句柄中移除。
        self.executions.pop(sequence, None)
        # 旧任务或同一任务的旧路径结束，不应终止当前新路径。
        if epoch != self.epoch or sequence != self.sequence:
            return
        try:
            # MBF 的最终结果包含 action 状态和控制器给出的 outcome。
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.stop("failed", message=str(exc))
            return
        # 只有 action 完成且控制器 outcome=0 才算到点成功。
        succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.outcome == 0
        # 向状态话题报告结果，stop 同时处理本任务的剩余句柄。
        self.stop("succeeded" if succeeded else "failed", outcome=result.outcome,
                  message=result.message, dist_to_goal=result.dist_to_goal)


def main(args=None):
    """console_scripts 入口：启动节点并在退出时释放 ROS 资源。"""
    # 先初始化 rclpy，再创建使用参数、话题、服务和 action 的节点。
    rclpy.init(args=args)
    node = MeshnavNavigator()
    try:
        # 进入回调循环，直到收到退出信号。
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 是正常的手动退出路径。
        pass
    finally:
        # 退出时释放节点；仅在 rclpy 仍有效时调用 shutdown。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
