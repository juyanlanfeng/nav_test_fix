#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace scan_planner
{

  // 内部长程参考轨迹容器：支持用一段局部 B 样条替换参考的一部分并补偿时间。
  // 这里的 global 是 SCAN 内部参考，不是 MeshNav/JIE 的全场搜索地图。
  // 当前 planner 源码未发现 setLocalTraj() 调用；局部替换字段不等于主流程已启用拼接。
  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_;  // 分段多项式参考；位置单位 m，曲线参数是相对时间（s）。
    // 可选的局部替换曲线：setLocalTraj() 将其设为 3 项，依次为位置、速度、加速度。
    // setGlobalTraj() 会清空它；与 manager.local_data_ 中正在执行的局部轨迹不是同一个成员。
    vector<UniformBspline> local_traj_;

    double global_duration_;  // 当前参考的有效总时长（s），不是路程；可被局部替换增时或终点调整截短。
    rclcpp::Time global_start_time_;  // 建立参考时保存的 ROS 时间戳；getPosition 等接收的是相对时间，而非此绝对时间。
    // 局部替换段在组合参考时间轴上的起止时刻（s），不是 ROS 时间戳，也不是局部样条节点值。
    // setGlobalTraj() 将二者置为 -1，表示尚未设置替换段。
    double local_start_time_, local_end_time_;
    double time_increase_;  // setLocalTraj() 累计的 time_inc（s）；用于把组合参考时间映射回原多项式时间。
    double last_time_inc_;  // 最近一次替换的 time_inc（s）；getPosition 的替换段之前分支用它扣除本次增量影响。
    double last_progress_time_;  // 选择局部目标时保存的参考进度（s），作为下次搜索起点；不是实测行驶时长或里程。

    // 下列空构造函数不初始化上述 double 成员，正常使用前须先调用 setGlobalTraj()。

    GlobalTrajData(/* args */) {}

    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const rclcpp::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    // get Bspline parameterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delta = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector3d prev_pt = first_pt;              // previous point
      Eigen::Vector3d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delta;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }

    // get Bspline parameterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }
  };

  struct PlanParameters
  {
    /* 规划算法参数：通常由 SCANPlannerManager::initPlanModules() 从 ROS 参数读取。 */
    // max_vel_：参考轨迹速度上限（m/s）；max_acc_：加速度上限（m/s²）。
    // 最终采样检查使用三维导数范数与加上绝对容差的限值，不是机体系 vx/vy 的独立限幅。
    // max_jerk_：加加速度上限参数（m/s³）；当前 planner 源码只读取它，未发现参与约束检查。
    double max_vel_, max_acc_, max_jerk_;
    // 绝对容差：速度 m/s、加速度 m/s²；checkDynamicFeasibility() 分别使用 max + tolerance。
    double vel_tolerance_, acc_tolerance_;
    // 控制点/采样点的目标空间间距尺度（m），用于初始时间间隔和重采样；不是曲线各控制点必然等距的保证。
    double ctrl_pt_dist;
    // 无量纲相对容差；B 样条 checkFeasibility() 将限值放宽为 limit*(1+tolerance)，另加数值裕量。
    // 与上面的 vel_tolerance_/acc_tolerance_（绝对增量）是两种不同检查。
    double feasibility_tolerance_;
    // manager 的规划视距尺度（m），用于初始化点集长度等判断；与 FSM 自己读取的 planning_horizon_ 分开保存。
    double planning_horizon_;

    /* 预留的处理耗时字段；当前 planner 源码未发现后续读写，不能当作已采集的性能统计。
     * 未发现计时赋值，因此不能仅凭字段名认定它们当前采用秒或毫秒。
     * reboundReplan() 的实际计时另存于函数内局部变量。
     */
    double time_search_ = 0.0;  // 预留搜索耗时，初始为 0。
    double time_optimize_ = 0.0;  // 预留轨迹优化耗时，初始为 0。
    double time_adjust_ = 0.0;  // 预留时间调整/细化耗时，初始为 0。
  };

  struct LocalTrajData
  {
    /* 当前已生成的局部轨迹数据，由 SCANPlannerManager::updateTrajInfo() 更新。
     * 下列基础类型没有在结构体声明中统一初始化；traj_id_ 在 initPlanModules() 中置零，
     * duration_ 等由后续轨迹更新填写，不能假定新建对象的所有字段天然为 0。
     */

    int traj_id_;  // 局部轨迹编号，每次 updateTrajInfo() 加 1，随 Bspline 消息发布；不是全局导航任务 ID。
    double duration_;  // 当前局部位置样条的总时长（s），由 getTimeSum() 得到，不是空间长度。
    // 原注释意图：局部轨迹结束并切回全局参考时的时间偏移；当前 planner 源码无读写，未参与时间同步。
    double global_time_offset;
    // 当前局部轨迹起始 ROS 时间戳；FSM 用 now-start_time_ 计算进度，冻结时会将其向后移动。
    rclcpp::Time start_time_;
    Eigen::Vector3d start_pos_;  // 位置样条在相对 t=0 的参考位置（m）；不是持续更新的机器人实际位置。
    // 三条样条依次表达位置（m）、一阶导数速度（m/s）、二阶导数加速度（m/s²），均在规划坐标系中。
    // evaluateDeBoorT(t) 使用相对轨迹时间（s）；这些参考值不等同于里程计测量值或最终机体系速度指令。
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace scan_planner

#endif
