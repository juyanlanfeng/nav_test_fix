#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE // 状态机运行状态
    {
      INIT, // 初始等待状态，等待里程计和导航触发条件
      WAIT_TARGET, // 等待有效目标状态
      GEN_NEW_TRAJ, // 生成新轨迹
      REPLAN_TRAJ, // 重新规划轨迹
      EXEC_TRAJ, // 执行轨迹，监测轨迹进度，判断是否需要重规划或结束
      EMERGENCY_STOP // 紧急停止，发布停止轨迹，根据条件重新尝试规划或等待新目标
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1, // 手动目标模式
      PRESET_TARGET = 2, // 预设航点模式（yaml）
      REFERENCE_PATH = 3, // 接收外部全局路径
    };

    /* 规划协作对象：在同一节点/进程中通过 C++ 调用协作，不各自代表一个 ROS 节点。 */
    SCANPlannerManager::Ptr planner_manager_;  // 规划管理器，持有地图、优化器、内部参考和局部轨迹数据。
    PlanningVisualization::Ptr visualization_;  // 可视化工具，显示目标、参考路线及局部轨迹。
    scan_planner_msgs::msg::DataDisp data_disp_;  // 待发布的调试消息缓存，不是完整的导航任务状态/结果。

    /* parameters */
    int navi_mode_; // 输入模式：1=手动目标，2=参数预设航点，3=外部参考路径；由 init() 读取和检查。
    // 距离阈值（m）：no_replan 为参考位置接近终点时暂停进度重规划的阈值；
    // replan 为参考位置离本段起点尚近时暂停进度重规划的阈值。均不是时间间隔或实测到达判据。
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;  // 从 fsm.waypoints 解析的 XYZ 航点数组（m）。
    int waypoint_num_;  // 预设航点数量，只在预设模式初始化时赋值。
    double planning_horizon_;  // FSM 选择局部目标的空间视距（m），与 manager 的同名参数分开保存。
    double emergency_time_;  // 即时重规划失败后，参考轨迹上距离碰撞小于此时间（s）则进入急停。
    double rviz_goal_height_;  // 手动模式首次里程计的 Z（m），用作后续 RViz 单目标高度；不随每帧位姿更新。
    // FSM 包络显示用的上下膨胀参数（m）；GridMap 另读同名参数执行障碍膨胀。
    // 障碍向上/下膨胀与车体相对参考点的上/下范围不能直接同向等同。
    double self_inflation_z_up_, self_inflation_z_down_;
    // 双圆柱半径及圆心相对车体参考点的前后偏移（m）；两圆心间距为 2*offset。
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;  // 外部参考路径的 Z 到规划参考点高度的偏移（m），pathCallback 中加到每个点上。
    std::string self_inflation_frame_id_;  // 车体包络 Marker 使用的 frame 名；设置名称不执行坐标转换。

    /* planning data */
    // 依次表示：收到导航触发、已有有效目标、收到过里程计、存在需要重新初始化的新目标。
    // have_odom_ 不检查数据新鲜度；have_new_target_ 在局部规划调用后被清除，即使该次失败。
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};  // 防止每条里程计都重复启动预设航点；首次启动时置 true。
    bool rviz_height_ready_;  // 是否已从首次车体位姿建立手动目标高度参考。
    bool go2_execution_frozen_;  // 控制器反馈：转向对齐时是否冻结参考轨迹计时，不代表所有运动都停止。
    // enable_fail_safe_ 控制是否允许自动退出急停；need_hover_stop_ 区分停止后等新目标还是自动重试。
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;  // 当前状态；转换函数修改它，主回调下一次按状态执行对应分支。
    int continuously_called_times_{0};  // 连续设置同一状态的计数，不是定时器次数或失败次数。
    int replan_fail_count_{0};  // FSM 累计的连续规划失败次数，成功时清零。
    int max_replan_fail_count_{1000};  // 失败次数上限，可由参数覆盖；达到后要求停止并等待新目标。
    rclcpp::Time last_freeze_update_time_;  // 上次冻结补偿检查的 ROS 时间戳，用于求本次应补偿的 dt。

    // odom_pos_：消息 pose 的 XYZ（m）；odom_vel_：直接复制的 twist.linear（m/s）。
    // 当前回调未旋转速度到规划系，接入标准车体系 twist 时须核对坐标语义。
    // odom_acc_：预留加速度（预期 m/s²）；当前估计赋值已注释，无有效读写。
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
    Eigen::Quaterniond odom_orient_;  // 直接复制的里程计四元数，用于提取当前车头 yaw；回调未归一化校验。

    // init_pt_：导航触发时记录的实际位置（m），当前有效代码只赋值，读取出现在已注释表达式中。
    // start_pt_：本次规划起点（m），来自实际位置；start_vel_/start_acc_：起始速度/加速度（m/s、m/s²），
    //          按调用路径取里程计或旧参考导数，特定情况下置零，不保证全部来自实测。
    // start_yaw_：预留的朝向起始状态向量；当前赋值代码已注释，未参与有效规划。
    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_;
    // end_pt_：当前参考终点（m），预设模式下可能只是本航点，障碍调整还可能将其截短。
    // end_vel_：目标速度占位（m/s），当前置零但未发现有效读取，不等于 local_target_vel_。
    Eigen::Vector3d end_pt_, end_vel_;
    // 本次局部规划目标位置（m）和参考速度（m/s）；由 getLocalTarget() 选择，近终点时目标速度置零。
    Eigen::Vector3d local_target_pt_, local_target_vel_;
    std::vector<Eigen::Vector3d> active_waypoints_;  // 当前激活的预设航点序列；外部 Path 模式不按此数组逐点推进。
    int current_wp_;  // 活动航点数组的零起始索引；由航点切换逻辑推进，planNextWaypoint() 本身不递增。

    bool flag_escape_emergency_;  // 急停分支的单次停止轨迹发布门控：true 时发布，随后置 false，避免反复发布。

    /* ROS utils */
    rclcpp::Node *node_{nullptr};  // 借用外部节点，不接管所有权；必须在 FSM 使用期间保持有效。
    // 墙钟定时器句柄：主状态机 10 ms、安全检查 50 ms；单线程执行器下不是两个并行线程。
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;  // 手动模式目标输入 move_base_simple/goal。
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  // body_pose 里程计输入，SensorDataQoS。
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;  // 参考路径模式输入 initial_path。
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;  // planning/go2_execution_frozen 冻结反馈。
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;  // planning/bspline 局部/停止轨迹输出。
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;  // planning/data_display 调试输出。
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;  // self_inflation 双圆柱包络显示。

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    // 设置状态并维护连续设置计数；无返回值。
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    // 返回 pair<连续设置同状态的次数, 当前状态>。
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

    bool checkCollision();  // 仅保留声明，当前 plan_manage 源码未发现定义或调用；实际检查使用 checkCollisionCallback。

  public:
    // 空构造函数不会为所有基础类型/Eigen 成员赋有效值；须先调用 init()，并等待所需输入后再规划。
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
