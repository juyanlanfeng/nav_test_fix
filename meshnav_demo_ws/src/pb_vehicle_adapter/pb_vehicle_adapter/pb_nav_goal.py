"""Unified goal/cancel client for the PB MeshNav / JIE / DDDMR frameworks.

    ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --case smoke
    ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --x X --y Y --z Z --yaw YAW
    ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --case smoke
    ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --cancel

Wraps each framework's native interface so the user does not have to hand-write
three different action/point-topic flows.  Always uses the map frame and sim time.
"""

import argparse
import math
import os
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import tf2_ros
import yaml


FRAMEWORKS = ("meshnav", "jie", "dddmr")
DEFAULT_RESULT_TIMEOUT = 180.0
# Arrival tolerance per framework: the controller's own goal tolerance, so that
# "arrived" means the controller stopped where it thinks it is done instead of
# the vehicle passing through the tolerance circle on its way.
#   MeshNav `mesh_controller` uses mbf's `dist_tolerance: 0.2`
#   d1_controller uses `goal_position_tolerance: 0.1`
DEFAULT_ARRIVAL_XY_TOLERANCE = 0.30
ARRIVAL_XY_TOLERANCE = {"meshnav": 0.20, "jie": 0.10, "dddmr": 0.20}

# --- cancellation contract (doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md R2) ----
# Distinguishable results instead of a single boolean.  NO_ACTIVE_GOAL is an
# idempotent CLI result but must never be counted as a verified cancellation.
CANCELED = "CANCELED"
CANCEL_ACCEPTED = "CANCEL_ACCEPTED"
ALREADY_TERMINAL = "ALREADY_TERMINAL"
NO_ACTIVE_GOAL = "NO_ACTIVE_GOAL"
REJECTED = "REJECTED"
TIMEOUT = "TIMEOUT"
SUCCEEDED = "SUCCEEDED"
ABORTED = "ABORTED"

CANCEL_ERROR_NONE = 0
CANCEL_ERROR_REJECTED = 1
CANCEL_ERROR_UNKNOWN_GOAL_ID = 2
CANCEL_ERROR_GOAL_TERMINATED = 3

GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6
GOAL_STATUS_TERMINAL = {GOAL_STATUS_SUCCEEDED, GOAL_STATUS_CANCELED, GOAL_STATUS_ABORTED}

# Action servers that own a navigation task, per entry.  Cancelling an entry never
# touches another framework's server, and a MoveBase-orchestrated task must be
# stopped at its top level first (it would otherwise re-dispatch its children).
FRAMEWORK_CANCEL_ACTIONS = {
    # move_base first: if the task is orchestrated by MBF, its top level must stop
    # before its children, otherwise it re-dispatches them right after they are
    # cancelled.  Direct GetPath->ExePath runs simply report NO_ACTIVE_GOAL there.
    "meshnav": ("/move_base_flex/move_base", "/move_base_flex/get_path",
                "/move_base_flex/exe_path"),
    "dddmr": ("/p2p_move_base",),
    "jie": (),  # topic-driven: no action UUID exists to observe
}
FRAMEWORK_COMMAND_TOPICS = {
    "meshnav": "/cmd_vel",
    "jie": "/cmd_vel_jie",
    "dddmr": "/pb/dddmr_cmd_vel_stamped",
}
# Exit codes: 0 verified cancellation (or already finished), 3 no active goal
# (idempotent, not a pass), 1 rejected/timed out/unverifiable.
CANCEL_EXIT_OK = 0
CANCEL_EXIT_FAILED = 1
CANCEL_EXIT_NO_ACTIVE_GOAL = 3

# jie_path_node subscribes to /start_point and /goal_point with
# `QoS(1).transient_local().reliable()`; a default (volatile) publisher is
# incompatible and every point is dropped silently, so the planner never answers.
JIE_POINT_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _load_smoke(framework):
    path = os.path.join(get_package_share_directory("pb_vehicle_adapter"), "config", "smoke_goal.yaml")
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data.get(framework, data["default"])


class NavGoal(Node):
    def __init__(self):
        super().__init__("pb_nav_goal")
        # Section 14: every interface works in map frame and simulation time, so
        # goal stamps match the rest of the stack instead of wall-clock time.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._jie_goal_pub = self.create_publisher(PointStamped, "/goal_point", JIE_POINT_QOS)
        self._jie_start_pub = self.create_publisher(PointStamped, "/start_point", JIE_POINT_QOS)
        self._jie_start_cmd = self.create_publisher(Bool, "/start_navigation", 10)
        self._jie_stop_cmd = self.create_publisher(Bool, "/stop_navigation", 10)
        self._paths = []
        self.result_timeout = DEFAULT_RESULT_TIMEOUT
        self.plan_only = False
        self.path_out = None
        # Name of the controller plugin the running server actually loaded; the
        # entry selects it with `controller_plugin` and must send the same name in
        # the ExePath goal, otherwise MBF rejects the goal.
        self.controller_name = "mesh_controller"

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_pose(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                tf = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
                return tf.transform.translation, tf.transform.rotation
            except Exception:  # noqa: BLE001 - keep waiting for the TF
                rclpy.spin_once(self, timeout_sec=0.1)
        return None, None

    def log_pose(self, label):
        """Log a map-frame pose as acceptance evidence."""
        translation, _rotation = self.robot_pose(timeout=5.0)
        if translation is None:
            self.get_logger().warn("%s unavailable (no map -> base_footprint TF)" % label)
            return
        try:
            tf = self._tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            yaw = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)
        except Exception:  # noqa: BLE001
            yaw = float("nan")
        self.get_logger().info("%s x=%.3f y=%.3f z=%.3f yaw=%.3f rad"
                               % (label, translation.x, translation.y, translation.z, yaw))

    # ---- cancellation (R2) -----------------------------------------------
    @staticmethod
    def _status_outcome(status):
        if status == GOAL_STATUS_CANCELED:
            return CANCELED
        if status == GOAL_STATUS_SUCCEEDED:
            return SUCCEEDED
        if status == GOAL_STATUS_ABORTED:
            return ABORTED
        return None

    def wait_for_terminal(self, action, uuids, timeout):
        """Wait until every given goal UUID reaches a terminal status.

        A CancelGoal response only means the request was accepted; the goal is not
        cancelled until its status says so.
        """
        from action_msgs.msg import GoalStatusArray

        seen = {}

        def callback(message):
            for entry in message.status_list:
                seen[tuple(entry.goal_info.goal_id.uuid)] = entry.status

        subscription = self.create_subscription(
            GoalStatusArray, action + "/_action/status", callback, 10
        )
        deadline = time.monotonic() + timeout
        pending = set(uuids)
        while pending and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            for key in list(pending):
                if seen.get(key) in GOAL_STATUS_TERMINAL:
                    pending.discard(key)
        self.destroy_subscription(subscription)
        return {key: seen.get(key) for key in uuids}

    def cancel_handle(self, handle, label, timeout=10.0):
        """Cancel a goal whose handle this process owns, then wait for its result."""
        future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return TIMEOUT, "%s: no cancel response within %.1f s" % (label, timeout)
        response = future.result()
        if response.return_code == CANCEL_ERROR_GOAL_TERMINATED:
            # A stopped, aborted task is not a successful arrival/cancel race.
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            if not result_future.done() or result_future.result() is None:
                return TIMEOUT, "%s: terminal result unavailable" % label
            wrapped = result_future.result()
            status = self._status_outcome(wrapped.status)
            result_code = getattr(wrapped.result, "outcome", 0)
            if status == ABORTED or (status == SUCCEEDED and result_code != 0):
                return ABORTED, "%s: terminal status=%s outcome=%s message=%s" % (
                    label, status, result_code, getattr(wrapped.result, "message", ""))
            if status not in (SUCCEEDED, CANCELED):
                return TIMEOUT, "%s: unexpected terminal status %s" % (label, wrapped.status)
            return ALREADY_TERMINAL, "%s: already terminal status=%s outcome=%s" % (label, status, result_code)
        if response.return_code == CANCEL_ERROR_UNKNOWN_GOAL_ID:
            return NO_ACTIVE_GOAL, "%s: unknown goal id" % label
        if response.return_code == CANCEL_ERROR_REJECTED:
            return REJECTED, "%s: cancel rejected by the server" % label
        if not response.goals_canceling:
            return NO_ACTIVE_GOAL, "%s: server reported no goals cancelling" % label
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done() or result_future.result() is None:
            return TIMEOUT, "%s: no terminal result within %.1f s" % (label, timeout)
        status = result_future.result().status
        outcome = self._status_outcome(status)
        if outcome is None or outcome == TIMEOUT:
            return TIMEOUT, "%s: unexpected terminal status %s" % (label, status)
        return outcome, "%s: terminal status %s" % (label, outcome)

    def cancel_actions(self, actions, goal_uuid=None, timeout=8.0):
        """Cancel goals through an action server's CancelGoal service.

        With ``goal_uuid`` the request targets that goal; without it the request
        carries a zero UUID and zero stamp, which the action interface defines as
        "all goals of this server".  The scope is therefore one navigation entry's
        action servers, never every robot.
        """
        from action_msgs.srv import CancelGoal

        results = {}
        for action in actions:
            client = self.create_client(CancelGoal, action + "/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
                # The server is not part of this run, so this entry has no task of
                # that kind: that is "no active goal", not a cancellation failure.
                # Only a server that exists but does not answer is a TIMEOUT.
                results[action] = (NO_ACTIVE_GOAL, "server not running in this entry")
                self.destroy_client(client)
                continue
            request = CancelGoal.Request()
            if goal_uuid is not None:
                request.goal_info.goal_id.uuid = list(goal_uuid)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                results[action] = (TIMEOUT, "no cancel response")
                self.destroy_client(client)
                continue
            response = future.result()
            uuids = [tuple(info.goal_id.uuid) for info in response.goals_canceling]
            if response.return_code == CANCEL_ERROR_REJECTED:
                results[action] = (REJECTED, "cancel rejected by the server")
            elif response.return_code == CANCEL_ERROR_GOAL_TERMINATED:
                results[action] = (ALREADY_TERMINAL, "goal already terminated")
            elif response.return_code == CANCEL_ERROR_UNKNOWN_GOAL_ID:
                results[action] = (NO_ACTIVE_GOAL, "unknown goal id")
            elif not uuids:
                results[action] = (NO_ACTIVE_GOAL, "no active goal on this server")
            else:
                statuses = self.wait_for_terminal(action, uuids, timeout)
                observed = [self._status_outcome(status) for status in statuses.values()]
                if not any(observed):
                    results[action] = (TIMEOUT, "terminal status not observed for %d goal(s)" % len(uuids))
                elif all(outcome == CANCELED for outcome in observed if outcome):
                    results[action] = (CANCELED, "%d goal(s) canceled" % len(uuids))
                else:
                    outcomes = sorted({outcome for outcome in observed if outcome})
                    results[action] = (outcomes[0], "terminal outcomes %s" % outcomes)
            self.destroy_client(client)
        return results

    def verify_stop(self, timeout=10.0, stop_threshold=0.05, hold=0.5):
        """Check the two stop contracts separately: zero command and real standstill."""
        from geometry_msgs.msg import Twist

        state = {"command": None, "twist": None}
        subscriptions = [
            self.create_subscription(
                Twist, "/pb/cmd_vel_safe", lambda message: state.__setitem__("command", message), 10
            ),
            self.create_subscription(
                Odometry, "/odom", lambda message: state.__setitem__("twist", message), 20
            ),
        ]
        zero_command = False
        stalled_since = None
        stopped = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not (zero_command and stopped):
            rclpy.spin_once(self, timeout_sec=0.1)
            command = state["command"]
            if command is not None:
                magnitude = (abs(command.linear.x) + abs(command.linear.y)
                             + abs(command.angular.z))
                if magnitude < 1.0e-6:
                    zero_command = True
            twist = state["twist"]
            if twist is not None:
                measured = twist.twist.twist
                speed = math.sqrt(measured.linear.x ** 2 + measured.linear.y ** 2
                                  + measured.angular.z ** 2)
                if speed <= stop_threshold:
                    if stalled_since is None:
                        stalled_since = time.monotonic()
                    elif time.monotonic() - stalled_since >= hold:
                        stopped = True
                else:
                    stalled_since = None
        for subscription in subscriptions:
            self.destroy_subscription(subscription)
        return zero_command, stopped

    def report_cancel(self, outcome, framework, uuid, zero_command, stopped, detail):
        """Emit one machine-readable line, separate from the ROS log."""
        if uuid is None:
            uuid_text = "-"
        else:
            uuid_text = "".join("%02x" % byte for byte in uuid)
        print("pb_nav_goal: cancel_result=%s framework=%s uuid=%s zero_command=%s "
              "stopped=%s detail=%s"
              % (outcome, framework, uuid_text,
                 "ok" if zero_command else "not-observed",
                 "ok" if stopped else "not-observed", detail))
        sys.stdout.flush()

    def cancel_entry(self, framework, goal_uuid, timeout):
        """Cancel this entry's active task. Never plans and never sends a goal."""
        self.get_logger().info(
            "cancel requested: framework=%s uuid=%s scope=%s"
            % (framework,
               "".join("%02x" % byte for byte in goal_uuid) if goal_uuid else "all active goals",
               "this navigation entry only"))
        if framework == "jie":
            self._jie_stop_cmd.publish(Bool(data=True))
            zero_command, stopped = self.verify_stop(timeout)
            self.report_cancel(CANCEL_ACCEPTED, framework, None, zero_command, stopped,
                               "topic driven entry: /stop_navigation published, no action UUID to observe")
            return CANCEL_EXIT_OK if (zero_command and stopped) else CANCEL_EXIT_FAILED

        results = self.cancel_actions(FRAMEWORK_CANCEL_ACTIONS[framework], goal_uuid, timeout)
        for action, (outcome, detail) in sorted(results.items()):
            self.get_logger().info("cancel %s -> %s (%s)" % (action, outcome, detail))
        outcomes = [outcome for outcome, _ in results.values()]
        if CANCELED in outcomes:
            aggregate = CANCELED
        elif ALREADY_TERMINAL in outcomes:
            aggregate = ALREADY_TERMINAL
        elif outcomes and all(outcome == NO_ACTIVE_GOAL for outcome in outcomes):
            aggregate = NO_ACTIVE_GOAL
        elif TIMEOUT in outcomes:
            aggregate = TIMEOUT
        elif REJECTED in outcomes:
            aggregate = REJECTED
        else:
            aggregate = outcomes[0] if outcomes else NO_ACTIVE_GOAL

        zero_command, stopped = self.verify_stop(timeout)
        detail = "; ".join("%s=%s" % (action, outcome)
                           for action, (outcome, _) in sorted(results.items()))
        self.report_cancel(aggregate, framework, goal_uuid, zero_command, stopped, detail)
        if aggregate == NO_ACTIVE_GOAL:
            return CANCEL_EXIT_NO_ACTIVE_GOAL
        if aggregate in (CANCELED, ALREADY_TERMINAL) and zero_command and stopped:
            return CANCEL_EXIT_OK
        return CANCEL_EXIT_FAILED

    def await_result(self, handle, timeout, label):
        """Return the action result, or None on timeout/abort.

        rclpy resolves the result future to None when the goal ends without a
        result, so callers must not assume a wrapped result object.
        """
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error("%s did not finish within %.0f s; cancelling" % (label, timeout))
            outcome, detail = self.cancel_handle(handle, label)
            self.get_logger().info("timeout cancel -> %s (%s)" % (outcome, detail))
            return None
        wrapped = future.result()
        if wrapped is None:
            self.get_logger().error("%s ended without a result (aborted or cancelled)" % label)
            return None
        return wrapped.result

    # ---- plan / path helpers --------------------------------------------
    def save_path(self, path):
        """Record a planned path (map frame) so its z profile can be checked.

        The tunnel acceptance has to prove the path stays on the tunnel floor
        (z < 0.10 m) instead of snapping to the roof top at z = 0.424 m, which
        only a recorded path can show.
        """
        poses = list(path.poses)
        if not poses:
            self.get_logger().error("plan has no poses")
            return None
        ys = [pose.pose.position.y for pose in poses]
        zs = [pose.pose.position.z for pose in poses]
        self.get_logger().info("plan: %d poses, z %.4f..%.4f, y %.3f..%.3f"
                               % (len(poses), min(zs), max(zs), min(ys), max(ys)))
        if not self.path_out:
            return (min(zs), max(zs))
        with open(self.path_out, "w", encoding="utf-8") as stream:
            stream.write("x,y,z,yaw\n")
            for pose in poses:
                q = pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                stream.write("%.5f,%.5f,%.5f,%.5f\n"
                             % (pose.pose.position.x, pose.pose.position.y,
                                pose.pose.position.z, yaw))
        self.get_logger().info("plan written to %s" % self.path_out)
        return (min(zs), max(zs))

    # ---- MeshNav: MBF GetPath then ExePath -------------------------------
    def run_meshnav(self, goal, cancel_after=0.0):
        from mbf_msgs.action import ExePath, GetPath
        from rclpy.action import ActionClient

        get_path = ActionClient(self, GetPath, "/move_base_flex/get_path")
        exe_path = ActionClient(self, ExePath, "/move_base_flex/exe_path")
        for name, client in (("get_path", get_path), ("exe_path", exe_path)):
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error("MeshNav %s action server unavailable" % name)
                return 1
        request = GetPath.Goal()
        request.use_start_pose = False
        request.target_pose = goal
        request.planner = "mesh_planner"
        future = get_path.send_goal_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MeshNav GetPath goal rejected")
            return 1
        result = self.await_result(handle, self.result_timeout, "MeshNav GetPath")
        if result is None:
            return 1
        self.get_logger().info("MeshNav GetPath outcome=%d message=%s points=%d"
                               % (result.outcome, result.message, len(result.path.poses)))
        if result.outcome != 0 or not result.path.poses:
            self.get_logger().error("MeshNav planning failed")
            return 1
        self.save_path(result.path)
        if self.plan_only:
            return 0
        execute = ExePath.Goal()
        execute.path = result.path
        execute.controller = self.controller_name
        future = exe_path.send_goal_async(execute)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("MeshNav ExePath goal rejected")
            return 1
        if cancel_after > 0.0:
            self.spin_for(cancel_after)
            uuid = tuple(handle.goal_id.uuid) if getattr(handle, "goal_id", None) else None
            outcome, detail = self.cancel_handle(handle, "MeshNav ExePath")
            zero_command, stopped = self.verify_stop()
            self.report_cancel(outcome, "meshnav", uuid, zero_command, stopped, detail)
            if outcome in (CANCELED, ALREADY_TERMINAL) and zero_command and stopped:
                return CANCEL_EXIT_OK
            return CANCEL_EXIT_FAILED
        result = self.await_result(handle, self.result_timeout, "MeshNav ExePath")
        if result is None:
            return 1
        self.get_logger().info("MeshNav ExePath outcome=%d message=%s" % (result.outcome, result.message))
        return 0 if result.outcome == 0 else 1

    # ---- JIE: /start_point + /goal_point, then the start command ---------
    def run_jie(self, goal, cancel_after=0.0):
        translation, _ = self.robot_pose()
        if translation is None:
            self.get_logger().error("JIE: no map -> base_footprint TF for the start point")
            return 1
        start = PointStamped()
        start.header.frame_id = "map"
        start.header.stamp = self.get_clock().now().to_msg()
        start.point.x, start.point.y, start.point.z = translation.x, translation.y, translation.z
        self._jie_start_pub.publish(start)
        target = PointStamped()
        target.header.frame_id = "map"
        target.header.stamp = self.get_clock().now().to_msg()
        target.point.x = goal.pose.position.x
        target.point.y = goal.pose.position.y
        target.point.z = goal.pose.position.z
        self._jie_goal_pub.publish(target)
        self.get_logger().info("JIE start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)"
                               % (start.point.x, start.point.y, start.point.z,
                                  target.point.x, target.point.y, target.point.z))
        paths = []
        self.create_subscription(Path, "/planned_path", lambda msg: paths.append(msg), 10)
        deadline = time.monotonic() + DEFAULT_RESULT_TIMEOUT
        while time.monotonic() < deadline and not paths:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not paths:
            self.get_logger().error("JIE: no /planned_path received")
            return 1
        self.get_logger().info("JIE planned path with %d poses" % len(paths[-1].poses))
        self.save_path(paths[-1])
        if self.plan_only:
            return 0
        self._jie_start_cmd.publish(Bool(data=True))
        self.get_logger().info("JIE start command sent")
        if cancel_after > 0.0:
            self.spin_for(cancel_after)
            self._jie_stop_cmd.publish(Bool(data=True))
            zero_command, stopped = self.verify_stop()
            self.report_cancel(CANCEL_ACCEPTED, "jie", None, zero_command, stopped,
                               "in-process cancel: /stop_navigation published")
            return CANCEL_EXIT_OK if (zero_command and stopped) else CANCEL_EXIT_FAILED
        # d1_controller has no arrival topic, so completion is judged from the
        # live map pose, exactly like the manual acceptance procedure, using the
        # controller's own goal tolerance.
        return 0 if self.wait_for_arrival(goal, ARRIVAL_XY_TOLERANCE["jie"]) else 1

    def wait_for_arrival(self, goal, tolerance_xy=DEFAULT_ARRIVAL_XY_TOLERANCE):
        """Wait until the vehicle really stands within `tolerance_xy` of the goal.

        A plain "distance below tolerance" test declares arrival while the vehicle
        is still driving: the first JIE tunnel run reported
        "arrived: 0.299 m from the goal (tolerance 0.30 m)" four seconds after the
        start command, with the vehicle still inside the tunnel.  The tolerance is
        therefore held over several consecutive samples before the run counts as
        arrived, so the reported final pose is the settled one.
        """
        deadline = time.monotonic() + self.result_timeout
        best = None
        settled = 0
        while time.monotonic() < deadline:
            translation, _rotation = self.robot_pose(timeout=1.0)
            if translation is not None:
                distance = math.hypot(translation.x - goal.pose.position.x,
                                      translation.y - goal.pose.position.y)
                best = distance if best is None else min(best, distance)
                settled = settled + 1 if distance <= tolerance_xy else 0
                if settled >= 3:
                    self.get_logger().info(
                        "arrived: %.3f m from the goal (tolerance %.2f m, settled)"
                        % (distance, tolerance_xy))
                    return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error("did not arrive within %.0f s (closest %.3f m, tolerance %.2f m)"
                                % (self.result_timeout,
                                   best if best is not None else float("nan"), tolerance_xy))
        return False

    # ---- DDDMR: /p2p_move_base PToPMoveBase action -----------------------
    def run_dddmr(self, goal, cancel_after=0.0):
        try:
            from dddmr_sys_core.action import PToPMoveBase
        except ImportError:
            self.get_logger().error(
                "dddmr_sys_core not found; source third_party/setup_dddmr_env.sh first")
            return 1
        from rclpy.action import ActionClient

        client = ActionClient(self, PToPMoveBase, "/p2p_move_base")
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("DDDMR /p2p_move_base action server unavailable")
            return 1
        request = PToPMoveBase.Goal()
        request.target_pose = goal
        request.target_value = 0.0
        future = client.send_goal_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("DDDMR goal rejected")
            return 1
        if cancel_after > 0.0:
            self.spin_for(cancel_after)
            uuid = tuple(handle.goal_id.uuid) if getattr(handle, "goal_id", None) else None
            outcome, detail = self.cancel_handle(handle, "DDDMR PToPMoveBase")
            zero_command, stopped = self.verify_stop()
            self.report_cancel(outcome, "dddmr", uuid, zero_command, stopped, detail)
            if outcome in (CANCELED, ALREADY_TERMINAL) and zero_command and stopped:
                return CANCEL_EXIT_OK
            return CANCEL_EXIT_FAILED
        result = self.await_result(handle, self.result_timeout, "DDDMR PToPMoveBase")
        if result is None:
            return 1
        # DDDMR: SUCCESS = 1, not 0.
        self.get_logger().info("DDDMR status=%d result=%s" % (result.status, result.result))
        return 0 if result.status == 1 else 1


def _parse_uuid(text):
    """Parse a goal UUID from 32 hex characters or 16 comma-separated bytes."""
    cleaned = text.strip().replace("-", "")
    if "," in cleaned:
        values = [int(part) for part in cleaned.split(",")]
    else:
        if len(cleaned) != 32:
            raise ValueError("UUID must be 32 hex characters or 16 comma-separated bytes")
        values = [int(cleaned[index:index + 2], 16) for index in range(0, 32, 2)]
    if len(values) != 16 or any(value < 0 or value > 255 for value in values):
        raise ValueError("UUID must contain exactly 16 bytes in range 0..255")
    return tuple(values)


def _build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--framework", required=True, choices=FRAMEWORKS)
    parser.add_argument("--case", choices=["smoke"], help="use the verified smoke goal")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0, help="degrees")
    parser.add_argument(
        "--cancel", action="store_true",
        help="cancel this navigation entry's active goal and exit; never plans "
             "and never sends a goal",
    )
    parser.add_argument(
        "--goal-uuid",
        help="cancel one specific goal: 32 hex characters or 16 comma-separated "
             "bytes; the default cancels all active goals of this entry only",
    )
    parser.add_argument(
        "--controller", default="mesh_controller",
        help="controller plugin name loaded by the server (must match the entry's "
             "controller_plugin choice)",
    )
    parser.add_argument(
        "--cancel-after", type=float, default=0.0,
        help="send the goal, then cancel that same UUID after N seconds",
    )
    parser.add_argument(
        "--plan-only", action="store_true",
        help="stop after planning: MeshNav skips ExePath, JIE skips /start_navigation",
    )
    parser.add_argument(
        "--path-out", help="write the planned path (map frame) to this CSV file",
    )
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_RESULT_TIMEOUT,
        help="wall-clock seconds to wait for the action result (default %.0f)" % DEFAULT_RESULT_TIMEOUT,
    )
    return parser


def parse_arguments(argv=None):
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.case == "smoke":
        smoke = _load_smoke(args.framework)
        goal_cfg = smoke["goal"]
        args.x, args.y = goal_cfg["x"], goal_cfg["y"]
        args.z = goal_cfg.get("z", 0.0)
        args.yaw = goal_cfg.get("yaw", 0.0)
    if not args.cancel and (args.x is None or args.y is None):
        parser.error("provide --x/--y (or --case smoke, or --cancel)")
    return args


def run_cli(args, node):
    """Execute one parsed invocation against `node`.

    Separated from `main()` so the cancel branch can be tested without a second
    `rclpy.init()` in the same process.  The cancel branch must stay above every
    goal construction and every action client: the old implementation planned
    first and then cancelled the goal it had just created, so `--cancel` could not
    affect a task started by an earlier process.
    """
    node.result_timeout = args.timeout
    node.plan_only = args.plan_only
    node.path_out = args.path_out
    node.controller_name = args.controller

    if args.cancel:
        goal_uuid = _parse_uuid(args.goal_uuid) if args.goal_uuid else None
        return node.cancel_entry(args.framework, goal_uuid, args.timeout)

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = node.get_clock().now().to_msg()
    if args.x is not None:
        goal.pose.position.x = args.x
        goal.pose.position.y = args.y
        goal.pose.position.z = args.z
    qx, qy, qz, qw = _yaw_quaternion(math.radians(args.yaw))
    goal.pose.orientation.x, goal.pose.orientation.y = qx, qy
    goal.pose.orientation.z, goal.pose.orientation.w = qz, qw

    node.log_pose("start pose")
    if args.framework == "meshnav":
        status = node.run_meshnav(goal, args.cancel_after)
    elif args.framework == "jie":
        status = node.run_jie(goal, args.cancel_after)
    else:
        status = node.run_dddmr(goal, args.cancel_after)
    node.log_pose("final pose")
    return status


def main(argv=None):
    args = parse_arguments(argv)
    rclpy.init()
    node = NavGoal()
    try:
        return run_cli(args, node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
