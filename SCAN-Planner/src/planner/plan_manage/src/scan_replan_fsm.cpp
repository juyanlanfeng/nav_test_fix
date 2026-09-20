// 重规划状态机，回答什么时候规划、用什么起点、失败了怎么办
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <stdexcept>

// 这里相当于是给declare_parameter()家了一道保险，先检查变量是否声明过了，没有声明再添加，声明过了则不再重复声明
// 不写没有任何影响
namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  // 普通初始化函数，不是 C++ 构造函数：由 main() 在创建 FSM 对象后显式调用。
  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    // 初始运行状态：还没有位姿、目标或可执行轨迹。
    node_ = node;
    current_wp_ = 0;  // 当前需要到达的航点索引，也就是当前的终点

    // *** 状态机 ***//
    exec_state_ = FSM_EXEC_STATE::INIT;  // 初始化状态机，置于INIT状态，等待里程计和目标点触发

    trigger_ = false;  // 是否已收到启动导航的触发，如目标/参考路径或预设航点触发。
    have_target_ = false;  // 是否已有可供规划使用的目标。

    // trigger_变量在接收到导航请求后就会置true,而have_target_只有在全局轨迹成功规划后才会置true

    have_odom_ = false;  // 是否收到过里程计；这个标志本身不代表数据仍然新鲜。
    have_new_target_ = false;  // 是否有新目标，即下次规划时是否需要更换目标点
    rviz_height_ready_ = false;  // 手动目标模式所用的 RViz 高度参考是否已准备好。
    go2_execution_frozen_ = false;  // 控制器是否因转向对齐等原因暂停轨迹时间推进。
    flag_escape_emergency_ = true;  // 急停分支的单次停止轨迹发布标志，避免反复发布。
    need_hover_stop_ = false;  // 连续失败后置 true：停止后等待新目标，而非立即自动重试。
    replan_fail_count_ = 0;  // FSM 的连续规划失败次数；成功规划时清零。
    last_freeze_update_time_ = node_->now();  // 冻结时间补偿的时间基准；启用仿真时间时使用 ROS 仿真时钟。

    // launch/YAML 可覆盖这些默认值；下列 -1 是缺少配置的占位值，不是推荐运行值。

    // 导航输入模式：1=手动目标，2=预设航点，3=外部参考路径；非法值会在函数末尾抛异常。
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);

    // 规划器里面实际上有两种轨迹，scan_planner从上层规划器获取路径，然后通过插值、平滑获得一个“全局”轨迹
    // 机器人会将“全局”轨迹划分为多个小的“局部”轨迹执行，那么多久更新一次局部轨迹就是由这个参数决定的
    // 机器人在当前这段轨迹上至少要走多少米，才允许再重新规划一次，单位m
    // 它不是重规划时间间隔；新路径或碰撞检查仍可触发重规划。
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);

    // 机器人快到终点了（最终终点），就没必要再重新规划了，单位m
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);

    // 沿长程参考曲线选取局部目标时使用的视距，比较与规划起点的空间距离，单位m
    // 也就是设置局部小段路径的长度
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);

    // 发现前方有障碍且重规划失败时，若距碰撞的参考时间小于此值则急停，单位s
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);

    // 是否允许急停后的自动状态转换：速度足够低时，按 need_hover_stop_ 选择重试或等待新目标。
    // false 并不禁用碰撞检查/停止轨迹，而是关闭这些自动退出急停的分支。
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);

    // 连续失败次数上限；达到后停止并等待新目标，不能直接换算为固定超时时长。
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);

    // 以下 self_* 成员用于 FSM 的车体包络可视化；GridMap 另读相同参数用于实际膨胀查询。
    // 单位 m：障碍体素分别向 +Z / -Z 膨胀的距离，不要直接当作车体上/下半高。
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);

    // 单位 m：双圆柱近似中每个圆柱的水平半径。
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);

    // 单位 m：两个圆柱中心相对车体参考点沿朝向分别偏移 +offset / -offset，中心间距为 2*offset。
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);

    // 单位 m：地表路径点到规划参考点的高度偏移；pathCallback 会给外部路径的 z 加上它。
    // 如果全局路径已经表达车体参考点高度，必须避免再次加高；它不是完整车体净空高度。
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);

    // 包络 Marker 的坐标系名称，应与其位置坐标一致；设置字符串不会自动执行 TF 坐标变换。
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    // 如果是预设航点模式
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {}); // 读取预设的航点
      // 必须至少一个航点且每组三个数；这里仅检查数组长度，不检查可达性。
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);  // 数值个数除以 3 得到航点个数。
      preset_waypoints_.resize(waypoint_num_);  // 分配三维航点数组。
      for (int i = 0; i < waypoint_num_; i++)
      {
        // 将扁平参数数组的第 i 组三个数转换为 Eigen 三维向量，此处不添加 body_height。
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    // 装配可视化和规划管理器：这三行创建的是普通 C++ 对象，不是 ROS 节点。
    // 彼此之间是直接函数调用，全部挂在 scan_planner_node 这一个节点上。
    // 为什么用"指针成员 + reset(new ...)"而不是直接写成成员对象：
    //   main 里是先构造状态机、之后才把 node 交给 init()（见 scan_planner_node.cpp)
    //       scan_planner::SCANReplanFSM planner;   // 此刻还没有 node
    //       planner.init(node.get());              // 现在才拿到 node
    //   而这两个对象都必须有 node 才能构造（一个要 create_publisher，一个要
    //   declare_parameter），所以只能推迟到这里创建。reset(p) = 让智能指针接管 p。
    //
    // 两个 Ptr 不是同一种：visualization_ 是 shared_ptr（要按值传给 initPlanModules
    // 一份），planner_manager_ 是 unique_ptr（没人共享它）。于是同一个可视化对象被
    // 状态机和 manager 共同持有，manager 里的 display* 用的是同一个对象，不是副本。
    visualization_.reset(new PlanningVisualization(node_));  // 只开 5 个 marker 话题的画图工具箱
    planner_manager_.reset(new SCANPlannerManager);          // 构造函数是空的，当前仍是空壳

    // 真正的初始化都在这里：读 manager.* 与 optimization.* 参数，建 GridMap 并 initMap，
    // 建 BsplineOptimizer 并 setParam/setEnvironment，建 A*（100^3 格 × 0.05 m 分辨率
    // = 5 m 见方的盒子，只够局部绕障），最后把 visualization_ 存下来。见 planner_manager.cpp。
    planner_manager_->initPlanModules(node_, visualization_);

    // ── 关于 visualization_：只画图、不影响规划结果，但也不是"随便能删" ──────────
    // 但有三个折扣，改这块代码时必须知道：
    //   a) 传 nullptr 会崩：reboundReplan() 里是无条件解引用 visualization_ 的，没有判空。
    //      所以写成 initPlanModules(node_) 省掉第二个参数 -> 段错误。想真正关掉可视化，
    //      必须先给 planner_manager.cpp 里那两处 display 调用加上判空。
    //   b) 有真实开销，且不计入日志里的规划耗时：那两行 display 位于 reboundReplan() 的
    //      t_init 计时结束之后、下一次 t_start 之前，而最后打印的是 t_init+t_opt+t_refine，
    //      所以报出来的 total time 是偏小的。
    //   c) 没有独立开关：shared_ptr 被状态机和 manager 共享，要关只能两边一起关。

    // 注册墙钟定时器：10 ms 是 FSM 请求调度周期（名义 100 Hz），不是规划计算耗时保证。
    // main 使用单线程执行器，耗时规划会推迟同一执行器中的其他回调。
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));

    // 50 ms（名义 20 Hz）检查当前轨迹未来部分是否碰撞；这不是独立的安全线程。
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));

    // 订阅车体里程计
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));

    // 控制器通知轨迹计时是否冻结；整数 10 是 QoS 历史队列深度，不是频率。
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    // 发布局部轨迹（控制点、节点向量、起始时间等），交给轨迹跟踪器执行。
    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);

    // 发布调试数据，队列深度 100
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);

    // 发布可视化包络；可靠传输并保留最新一条，便于具有兼容 QoS 的晚加入订阅者读取。
    // 仅用于可视化，不用管他
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    // 按导航模式选择目标来源。以下话题是相对名称，可以在 launch 中重映射。
    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)  // 模式 1：接收 RViz 等来源的单个目标位姿。
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1)); // 订阅目标点话题
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)  // 模式 3：接收 MeshNav/JIE 等外部全局路径。
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1)); // 订阅initial,原始路径需要发到这个话题上
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)  // 模式 2：不新建目标订阅，收到首次里程计后启动预设航点。
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else  // 缺少有效模式配置时终止初始化，交给 main 的异常处理报告错误。
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  /**
   * @brief 激活参数中的预设航点，从第一个航点开始生成内部参考轨迹。
   * @details 无形参，读取 preset_waypoints_ 和 odom_pos_，重置当前航点索引并置导航触发标志。
   *          首段参考生成成功后进入 GEN_NEW_TRAJ；失败仅记录错误，不代表车辆已启动。
   * @return 无（void）；结果通过 active_waypoints_、FSM 状态及日志体现。
   */
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_; // 存储预设的航点

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i); // 可视化
    }

    active_waypoints_ = wps; // 将预设的航点添加到激活航点
    // 这里区分一下preset_waypoints_和active_waypoints_，preset_waypoints_存储原始航点数据，active_waypoints_则是导航器实际操作的航点，
    // 这样可以避免操作过程中污染原始数据

    current_wp_ = 0; // 当前为第0个航点
    trigger_ = true; // 已经收到导航触发
    init_pt_ = odom_pos_; // 初始位置，直接来自里程计（在回调函数中只取了x,y,z）

    if (planNextWaypoint()) // 先尝试规划到第一个路径点
    {
      // 第一个路径点规划成功
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 状态机
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  /**
   * @brief 接收 RViz 单目标，将其包装成单点 Path 后交给 waypointCallback。
   * @param msg 目标位姿的只读共享指针；空消息或初始车体高度尚未就绪时忽略。
   *            后续实际使用目标 XY 和初始里程计高度，不使用目标朝向约束。
   * @return 无（void）；目标处理和状态切换由 waypointCallback 完成。
   */
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  /**
   * @brief 按手动单目标语义生成从当前里程计状态到目标的内部参考轨迹。
   * @param msg 航点消息；只使用第一个点的 XY，Z 使用 rviz_goal_height_。
   *            空消息、空路径或首点输入 Z 小于 -0.1 m 时忽略；不在此执行 TF 变换。
   * @return 无（void）；成功置目标标志，并从 WAIT_TARGET/EXEC_TRAJ 分别转入生成/重规划。
   * @note 与 pathCallback 不同，本函数不把整条 Path 当作外部参考路线。
   */
  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  /**
   * @brief 根据整组航点生成内部长程参考轨迹，检查/调整终点并发布可视化。
   * @param waypoints 按行进顺序排列的三维参考点，单位 m，须已处于规划坐标系和参考点高度。
   *                  此处不再增加 body_height_，以最后一个点作为初始终点。
   * @return true 表示参考生成与终点检查通过；false 表示数组为空、生成失败或终点调整失败。
   * @note 成功更新 global_data_ 和目标标志；不是全场地图搜索，也不保证整条参考无碰撞。
   */
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;

    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  /**
   * @brief 规划轨迹到 current_wp_ 这个目标点
   * @details 无形参，读取 active_waypoints_ 与当前运动状态；函数本身不会递增 current_wp_。
   *          更新起始状态、终点、参考轨迹和目标标志，并显示当前航点。
   * @return true 表示参考生成及终点调整成功；false 表示索引越界或上述过程失败。
   */
  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size()) // 越界检查
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_]; // 终点设为current_wp_
    setStartStateFromOdomOrCurrentTraj(); // 设置起始状态，此时刚刚发出第一个点，还没开始规划，初始状态来自于里程计

    // 尝试规划一条路径
    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    } // 规划不出来，报错

    if (!adjustGlobalTargetIfOccupied()) // 要么终点不会发生碰撞，要么能找到一个替代的点，否则规划失败
      return false;

    // 可视化
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero(); // 这个没用
    have_target_ = true; // 有目标点
    have_new_target_ = true; // 有新目标点

    // 可视化
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  /**
   * @brief 判断当前是否采用逐个预设航点推进的模式；无形参，不修改对象。
   * @return 仅 navi_mode_ 为 PRESET_TARGET 时返回 true；外部参考路径模式也返回 false。
   */
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  /**
   * @brief 终点被占据时，沿现有内部参考轨迹从后向前寻找空闲点并截短参考有效时长。
   * @details 无形参，使用 grid_map_、global_data_ 与 end_pt_；成功调整时同步限制参考进度时间。
   * @return false 表示终点被占据且采样找不到空闲替代点；其他分支返回 true。
   * @note true 还包含“无地图”“时长过短”以及终点查询值 <= 0 的跳过检查情况；
   *       地图外返回 -1 也被此处接受，因此 true 不等价于严格确认目标安全或可达。
   */
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_; // 栅格地图
    auto &global_data = planner_manager_->global_data_; // 全局路径
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3) // 没有地图或者全局路径长度直接为0,就没有重新寻找空闲点的必要了
      return true;

    constexpr double sample_dt = 0.05; // 取样间隔
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt))); // 切段数量
    // std::ceil(duration / sample_dt)除完以后向上取整
    // static_cast<int>（）将double转为int
    // std::max(1, ) 至少切一段
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration); // 终点位置
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num); // 去掉第一段
    // 对机器人做一次碰撞检测——查"如果机器人以 final_prev -> final_pt 为车头朝向、中心放在 final_pt，会不会撞到障碍"
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt)); // 也就是检查机器人在路径终点处是否会碰到障碍物
    if (final_occ <= 0)
      return true; // 终点不会碰到障碍物，或者终点不在观测范围内，无需截断，直接返回

    // 终点处会发生碰撞，开始向前截断
    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num; // 截短参考有效时长
      const double prev_t = duration * std::max(0, i - 1) / sample_num; // 截短后的有效时长的前一段
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t); // 截短后的终点坐标
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t); // 终点前一个点

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0) // 终点不会碰到障碍物
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt; // 更新终点到截断后的点
        global_data.global_duration_ = t; // 更新全局参考时间
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true; // 找到可用的替代点
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false; // 整条路径上都没有可用的空闲位置
  }

  /**
   * @brief 接收外部全局路径并建立 SCAN 内部参考，供后续局部轨迹规划使用。
   * @param msg initial_path 的只读路径消息；读取全部点的 XYZ，并给 Z 加上 body_height_。
   *            调用方须保证坐标系一致；本函数不做 TF 转换，也不读取路径点朝向。
   * @return 无（void）；空路径直接忽略，生成失败记录日志，成功设置参考与目标标志。
   * @note 成功后仅对 WAIT_TARGET/EXEC_TRAJ 显式切换状态，不能视为任意状态下的任务抢占接口。
   */
  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    trigger_ = true;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      waypoints.push_back(wp);
    }
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  /**
   * @brief 缓存车体位置、线速度和姿态，更新车体包络显示，并按模式触发首次导航准备。
   * @param msg body_pose 的里程计消息；位置单位 m，线速度 m/s，姿态使用四元数。
   *            速度分量被直接复制，未按 child_frame_id 旋转，接入方须核对速度坐标系。
   * @return 无（void）；更新 odom_*、have_odom_，手动模式保存首次高度，预设模式只启动一次。
   * @note 此函数没有空指针或时间戳新鲜度校验；“收到过里程计”不等于定位一直有效。
   */
  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    // 从里程计获取x,y,z坐标
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_) // 手动目标模式，且高度还未锁定
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  /**
   * @brief 保存控制器反馈的轨迹计时冻结状态。
   * @param msg data=true 表示执行时间应暂停推进；false 表示不冻结。不在这里发送停止指令。
   * @return 无（void）；仅修改 go2_execution_frozen_，实际时间补偿由 updateLocalTrajTimeFreeze 完成。
   */
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  /**
   * @brief 冻结期间向后移动局部轨迹起始时间，使 FSM 计算的相对轨迹时间暂停。
   * @details 无形参，读取 ROS 当前时间、上次更新时间和冻结标志；仅补偿 0 < dt <= 0.2 s
   *          且轨迹起始时间有效的情况。无论是否补偿，都会更新本次时间基准。
   * @return 无（void）；修改 last_freeze_update_time_，必要时修改 local_data_.start_time_。
   */
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  } // 同步什么计时器用的，反正加上就对了

  /**
   * @brief 将里程计姿态中的车体 X 轴投影到水平面，求车头朝向；无形参，不修改对象。
   * @return yaw，单位 rad，通常在 [-pi, pi]；水平投影退化时返回 0。
   */
  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0); // 四元数转旋转矩阵
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0; // 向量模长为0直接忽略不计
    return std::atan2(heading(1), heading(0)); // 返回车头朝向，范围[-pi~pi]
  }

  /**
   * @brief 用from~to两个点计算机器人朝向，如果两个点相距过近就直接用里程计数据
   * @param from 线段起点，三维坐标，单位 m。
   * @param to 同一坐标系下的线段终点，单位 m；计算时忽略两点的 Z 差。
   * @return atan2(dy, dx) 对应的角度（rad）；水平线段近乎为零时回退到里程计 yaw。
   * @note 全向侧移时路径方向不一定等于真实车头方向。
   */
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1)); // 从from到to的向量
    if (diff.squaredNorm() < 1e-8) 
      return getOdomYaw(); // 向量模长为0,直接忽略，改用里程计求机器人朝向
    return std::atan2(diff(1), diff(0)); // 返回朝向角
  }

  /**
   * @brief 根据当前车体位置、yaw 与双圆柱参数发布前后两个圆柱 Marker。
   * @details 无形参，读取 odom_* 和 self_* 参数；消息使用配置的 frame，不执行 TF 变换。
   * @return 无（void）；输出到 self_inflation，仅作可视化，不修改占据地图或控制指令。
   */
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  /**
   * @brief 设置 FSM 状态，维护连续同状态调用计数，并输出转换日志。
   * @param new_state 要进入的状态；与当前相同时计数递增，不同时计数重置为 1。
   * @param pos_call 调用来源标签（例如 FSM、SAFETY、TRIG），仅用于日志定位，不是空间位置。
   * @return 无（void）；修改 exec_state_ 与 continuously_called_times_，不立即执行目标状态分支。
   */
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    // 检查待切换的状态与当前正处于的状态是否相同
    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1; // 看起来像是调试用的

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"}; // 总共就这7种状态
    int pre_s = int(exec_state_); // exec_state_是一个枚举类型
    exec_state_ = new_state; // 更新当前执行状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  /**
   * @brief 读取最近连续设置同一 FSM 状态的计数及当前状态；无形参。
   * @return pair.first 为连续设置计数，pair.second 为当前状态。
   *         计数由 changeFSMExecState 更新，不等同于定时器执行次数或连续规划失败次数。
   */
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  /**
   * @brief 将当前 FSM 状态名称输出到标准输出，便于运行诊断；无形参。
   * @return 无（void）；不修改状态，也不发布结构化 ROS 状态消息。
   */
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl; // 输出日志
  }

  /**
   * @brief FSM 主定时回调：处理等待、轨迹生成、执行进度、重规划和急停恢复。
   * @details 无形参，依据 exec_state_、目标标志、里程计和轨迹数据选择分支；
   *          先补偿冻结时间，非提前返回路径最后执行失败次数检查并发布调试数据。
   * @return 无（void）；通过状态、目标标志、轨迹发布和失败计数体现处理结果。
   * @note 轨迹时间结束可使 FSM 回到等待，不代表真实车辆已到达；局部重规划不调用外部全局规划器。
   */
  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze(); // 同步一下时间什么的

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) // 每累加到100次输出一次日志
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl; // 等待里程计
      if (!trigger_)
        cout << "wait for goal." << endl; // 等待目标点
      fsm_num = 0;
    }

    switch (exec_state_) // 根据当前状态机状态选择要执行的分支
    {
    case INIT: // 初始状态
    {
      // 没有接收到请求或里程计都继续等待
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM"); // 接收到请求且里程计就绪以后进入WAIT_TARGET状态
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return; // 没有成功规划出到达目标点的路径
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  /**
   * @brief 检查连续规划失败上限，达到后要求急停并在停止后等待新目标。
   * @details 无形参，读取 replan_fail_count_ 与 max_replan_fail_count_；超限时清零计数，
   *          设置 need_hover_stop_ 和单次停止发布标志，再切到 EMERGENCY_STOP。
   * @return 无（void）；未达到上限时不改变状态。本函数不是任务成功结束通知。
   */
  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  /**
   * @brief 从实际位置和当前轨迹导数构造起始状态，刷新内部参考并尝试局部重规划。
   * @details 无形参，位置用 odom_pos_，速度/加速度从当前轨迹采样；若水平速度背离目标则清零。
   *          先重建当前位置到 end_pt_ 的参考并调整终点，再尝试普通和随机多项式初始化。
   * @return true 表示局部规划成功且新 Bspline 已发布；false 表示参考刷新、终点调整或两次局部尝试失败。
   * @note 此处刷新的是 SCAN 内部参考，不是请求 MeshNav/JIE 重新搜索；需关注原外部路径约束的保留问题。
   */
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  /**
   * @brief 为下一次规划填写起始位置、速度与加速度。
   * @details 无形参；位置始终用实际里程计。默认速度用里程计、加速度为零；
   *          若当前轨迹存在且时间在允许范围内，则改用轨迹导数以保持参考连续性。
   *          使用轨迹导数时，若水平速度背离目标，会将起始速度和加速度清零。
   * @return 无（void）；输出保存在 start_pt_、start_vel_、start_acc_。
   */
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    // 设置起始位置、速度、加速度：直接来自里程计
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    // 下面这些是检查局部轨迹是否存在
    // 如果存在，则从"当前正在执行的计划"里，取出此刻的速度和加速度，作为下一次规划的起点条件
    // 如果不存在，则直接将里程计数据作为下一次规划的起点条件
    LocalTrajData *info = &planner_manager_->local_data_; // local_data_是规划好的局部轨迹
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return; // start_time为0或轨迹长度为0->路径不存在

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return; // 路径已经执行的时长，raw_t_cur，其范围应当是[0, duration_]，越界说明有问题，return

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_); // 将执行时长raw_t_cur强制约束在[0, duration_]，获得t_cur
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur); // 从velocity_traj_函数中取出t_cur时刻的数值 
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 从acceleration_traj_函数中取出t_cur时刻的数值

    // 如果起点速度的方向是"背着目标"的，就把它丢掉，改成从静止开始规划
    // .head<2>()	Eigen 的"取前 2 个分量"，即 (x, y)，丢掉 z
    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>(); // 由机器人当前位置指向目标点的向量
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0) // 速度向量与方向向量的点乘小于0,说明夹角为钝角，机器人正在背离目标点运动
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
    // 这个可能主要用于处理目标点突变的情况。本来机器人朝着原来的目标点前进，此时突然换成了一个反方向的目标点，那么机器人立刻丢弃上一次规划的速度
    // 从0开始重新规划
  }

  /**
   * @brief 安全定时回调：沿当前局部轨迹的未来部分采样检查膨胀占据。
   * @details 无形参；先补偿冻结时间，在等待目标或轨迹起始时间无效时跳过。
   *          以 0.01 s 步长检查；当前进度在前 2/3 时，仅检查到该分界。
   *          碰撞时先尝试即时重规划，失败后按碰撞时间间隔选择急停或 REPLAN_TRAJ。
   * @return 无（void）；可能发布替换轨迹并修改 FSM 状态，不直接输出底盘速度。
   * @note 检查使用当前地图与参考路径方向，不包含其他机器人的未来运动预测。
   */
  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  /**
   * @brief 选择局部目标、调用 manager 的反弹优化规划，并在成功时序列化发布 B 样条。
   * @param flag_use_poly_init 是否请求多项式初始化；与 have_new_target_ 做逻辑或后传给 manager。
   * @param flag_randomPolyTraj 是否请求随机化多项式初值，用于尝试摆脱失败的初始化。
   * @return manager 本次规划是否成功；true 时已发布轨迹消息并更新可视化，不表示实际执行成功。
   * @note 无论规划是否成功，此次调用都会清除 have_new_target_ 标志。
   */
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  /**
   * @brief 请求 manager 构造保持指定位置的停止轨迹，并将其发布给轨迹跟踪器。
   * @param stop_pos 停止轨迹的参考位置，规划坐标系下的 XYZ，单位 m，通常传当前 odom_pos_。
   * @return 当前实现正常执行到末尾时固定返回 true；未检查 manager 返回值，也未等待底盘停车确认。
   * @note 这是停止轨迹请求，不是硬件急停或已完成制动的证明。
   */
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  /**
   * @brief 沿内部长程参考选择局部目标位置，并确定目标处的参考速度。
   * @details 无形参，使用 start_pt_、planning_horizon_ 与 global_data_ 的进度；
   *          在视距附近取目标，被占据时沿参考前后搜索空闲替代点，并更新参考进度信息。
   *          距终点小于按最大速度/加速度估计的制动距离时，将目标速度置零。
   * @return 无（void）；输出保存到 local_target_pt_（m）、local_target_vel_（m/s）。
   * @note 没找到空闲替代点时仅告警，不返回失败标志，后续规划仍须检查碰撞与可行性。
   */
  void SCANReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
