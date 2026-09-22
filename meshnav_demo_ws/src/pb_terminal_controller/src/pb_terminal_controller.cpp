// Copyright 2026. Licensed under Apache-2.0.
//
// PbTerminalController: an independent MeshNav controller plugin that adds the
// generic terminal state machine of doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md
// section 6 on top of the stock tracking controller.
//
// Why a separate plugin: the scene-specific terminal behaviour must not be
// inserted into the shared MeshController.  This plugin is registered under its own
// name and is selected by the PB entry only; the tracking layer (mesh vector field
// following) stays untouched inside mesh_controller::MeshController, which this
// plugin owns as a delegate.
//
// What this layer adds:
//   * one arrival criterion (planar position + the goal's level + heading) with
//     hysteresis, shared by the state machine and by isGoalReached();
//   * dwell time before a goal counts as reached;
//   * level-aware goal handling: the goal is projected onto the robot's level, the
//     original and the projected goal are both kept and reported, and a goal whose
//     height error is too large is rejected with a clear error instead of being
//     chased (a ground robot must not correct height with a Z velocity);
//   * input-health gating: an unhealthy truth chain stops the robot instead of
//     driving on a stale pose;
//   * velocity and acceleration limits applied in every state.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mbf_mesh_core/mesh_controller.h>
#include <mbf_msgs/action/exe_path.hpp>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/mesh_map.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "pb_terminal_controller/terminal_logic.h"

namespace pb_terminal_controller
{

class PbTerminalController : public mbf_mesh_core::MeshController
{
public:
  PbTerminalController() = default;

  bool initialize(const std::string & name,
                  const std::shared_ptr<tf2_ros::Buffer> & tf_ptr,
                  const std::shared_ptr<mesh_map::MeshMap> & mesh_map_ptr,
                  const rclcpp::Node::SharedPtr & node) override
  {
    name_ = name;
    node_ = node;
    map_ = mesh_map_ptr;

    // The tracking layer keeps its own parameter namespace (`<name>.max_lin_velocity`
    // and friends) and is used unchanged.
    if (!tracking_.initialize(name, tf_ptr, mesh_map_ptr, node)) {
      RCLCPP_ERROR(node_->get_logger(), "PbTerminalController: tracking layer failed to initialize");
      return false;
    }

    params_.settle_radius = declare("settle_radius", params_.settle_radius);
    params_.align_radius = declare("align_radius", params_.align_radius);
    params_.hysteresis = declare("hysteresis", params_.hysteresis);
    params_.level_tolerance = declare("level_tolerance", params_.level_tolerance);
    params_.yaw_tolerance = declare("yaw_tolerance", params_.yaw_tolerance);
    params_.yaw_hysteresis = declare("yaw_hysteresis", params_.yaw_hysteresis);
    params_.dwell_s = declare("dwell_s", params_.dwell_s);
    params_.align_gain = declare("align_gain", params_.align_gain);
    params_.align_deadband = declare("align_deadband", params_.align_deadband);
    params_.max_linear_acceleration = declare("max_linear_acceleration", params_.max_linear_acceleration);
    params_.max_angular_acceleration = declare("max_angular_acceleration", params_.max_angular_acceleration);
    params_.settle_max_linear_velocity = declare("settle_max_linear_velocity", params_.settle_max_linear_velocity);
    params_.settle_max_angular_velocity = declare("settle_max_angular_velocity", params_.settle_max_angular_velocity);
    // P3: a hold is an active correction on this drive, and commanded motion that
    // produces none is detected during TRACK.
    params_.hold_gain = declare("hold_gain", params_.hold_gain);
    params_.hold_max_linear_velocity = declare("hold_max_linear_velocity", params_.hold_max_linear_velocity);
    params_.hold_max_angular_velocity = declare("hold_max_angular_velocity", params_.hold_max_angular_velocity);
    params_.stuck_command_threshold = declare("stuck_command_threshold", params_.stuck_command_threshold);
    params_.stuck_progress_fraction = declare("stuck_progress_fraction", params_.stuck_progress_fraction);
    params_.stuck_min_expected_m = declare("stuck_min_expected_m", params_.stuck_min_expected_m);
    params_.stuck_timeout_s = declare("stuck_timeout_s", params_.stuck_timeout_s);
    params_.recovery_dwell_s = declare("recovery_dwell_s", params_.recovery_dwell_s);
    // Height difference beyond which the goal is on another level and is rejected
    // rather than silently moved to the robot's level.
    max_goal_height_error_ = declare("max_goal_height_error", max_goal_height_error_);
    goal_search_radius_ = declare("goal_search_radius", goal_search_radius_);
    require_health_ = declare_bool("require_health", true);
    health_topic_ = declare_string("health_topic", "/pb/truth_health");

    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
      name_ + "/terminal_state", rclcpp::QoS(1).transient_local());
    // Per-cycle diagnostic trace: separating the tracker's raw command, its
    // outcome and the final published command is what makes a tracking regression
    // attributable to a layer instead of guessed at.
    debug_pub_ = node_->create_publisher<std_msgs::msg::String>(
      name_ + "/terminal_debug", 10);
    if (require_health_) {
      health_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        health_topic_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr message) { healthy_ = message->data; });
    }
    RCLCPP_INFO(node_->get_logger(),
                "PbTerminalController ready: settle %.3f m, align %.3f m, dwell %.2f s, health '%s' (%s)",
                params_.settle_radius, params_.align_radius, params_.dwell_s, health_topic_.c_str(),
                require_health_ ? "required" : "not required");
    return true;
  }

  bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped> & plan) override
  {
    if (plan.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "PbTerminalController: empty plan");
      return false;
    }
    goal_original_ = plan.back().pose;
    distance_to_goal_ = 0.0;
    if (state_.state != TerminalState::TRACK) {
      publish_state(TerminalState::TRACK, "new plan");
    }
    state_ = TerminalOutput();
    // Cancellation belongs to the previous task, not the lifetime of the plugin.
    cancel_requested_ = false;
    have_last_pose_ = false;
    last_step_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    // Hand the plan to the tracking layer FIRST: the level check below searches the
    // mesh, and doing it before the tracker copies its vector field perturbed the
    // tracker's initial face lookup (observed as a pure-rotation first command
    // while the goal was straight ahead).
    const bool tracking_ok = tracking_.setPlan(plan);
    goal_valid_ = resolve_goal_level(plan.back());
    return tracking_ok;
  }

  bool cancel() override
  {
    cancel_requested_ = true;
    return tracking_.cancel();
  }

  uint32_t computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
                                   const geometry_msgs::msg::TwistStamped & velocity,
                                   geometry_msgs::msg::TwistStamped & cmd_vel,
                                   std::string & message) override
  {
    if (pose.header.frame_id.empty()) {
      message = "PbTerminalController: pose has no frame";
      return mbf_msgs::action::ExePath::Result::TF_ERROR;
    }

    TerminalInput in;
    in.goal_valid = goal_valid_;
    in.healthy = !require_health_ || healthy_;
    in.cancel_requested = cancel_requested_;
    in.dt = last_step_time_.nanoseconds() == 0
              ? 0.0
              : (node_->now() - last_step_time_).seconds();
    last_step_time_ = node_->now();

    const double robot_yaw = yaw_from_quaternion(pose.pose.orientation);
    in.robot_yaw = robot_yaw;
    // Measured progress for the no-progress check: the distance the body actually
    // travelled since the previous control step.  It uses the same pose the arrival
    // criterion uses, so the stuck test is a statement about the world rather than
    // about what the controller asked for.
    if (have_last_pose_) {
      in.moved_distance = std::hypot(pose.pose.position.x - last_pose_.position.x,
                                     pose.pose.position.y - last_pose_.position.y);
    }
    have_last_pose_ = true;
    last_pose_ = pose.pose;
    last_robot_z_ = pose.pose.position.z;
    in.dx = goal_projected_.position.x - pose.pose.position.x;
    in.dy = goal_projected_.position.y - pose.pose.position.y;
    in.dz = goal_projected_.position.z - pose.pose.position.z;
    in.yaw_error = wrap_angle(yaw_from_quaternion(goal_projected_.orientation) - robot_yaw);
    last_yaw_error_ = in.yaw_error;
    distance_to_goal_ = planar_error(in);

    // The tracking layer always runs, so the terminal layer can hand back to it at
    // any time with a warm state.
    geometry_msgs::msg::TwistStamped tracking_cmd;
    std::string tracking_message;
    const uint32_t tracking_outcome = tracking_.computeVelocityCommands(
      pose, velocity, tracking_cmd, tracking_message);
    if (!cancel_requested_ && tracking_outcome >= 10 && state_.state == TerminalState::TRACK) {
      // The tracking layer failed and the terminal layer has nothing better to
      // offer.  Publish an explicit stop rather than leaving cmd_vel untouched,
      // which would let a stale command be sent again.
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      cmd_vel.header.stamp = node_->now();
      cmd_vel.header.frame_id = std::string();
      publish_debug(in, tracking_outcome, tracking_cmd, cmd_vel);
      message = tracking_message;
      return tracking_outcome;
    }
    in.tracking_linear_x = tracking_cmd.twist.linear.x;
    in.tracking_linear_y = tracking_cmd.twist.linear.y;
    in.tracking_angular_z = tracking_cmd.twist.angular.z;

    const TerminalState before = state_.state;
    state_ = step(params_, in, state_);
    if (state_.state != before || !state_.message.empty()) {
      publish_state(state_.state, state_.message);
    }

    cmd_vel.twist.linear.x = state_.linear_x;
    cmd_vel.twist.linear.y = state_.linear_y;
    cmd_vel.twist.angular.z = state_.angular_z;
    cmd_vel.header.stamp = node_->now();
    // Leave the frame empty: MBF fills it with its configured robot frame
    // (base_footprint).  Copying the pose frame here put "map" into the command,
    // which the velocity adapter rejects, so every command was silently dropped.
    cmd_vel.header.frame_id = std::string();

    if (state_.state == TerminalState::FAULT) {
      message = state_.message;
      return mbf_msgs::action::ExePath::Result::INTERNAL_ERROR;
    }
    if (state_.state == TerminalState::CANCELING) {
      message = state_.message;
      return mbf_msgs::action::ExePath::Result::CANCELED;
    }
    if (state_.state == TerminalState::BLOCKED) {
      message = state_.message;
      return mbf_msgs::action::ExePath::Result::NO_VALID_CMD;
    }
    publish_debug(in, tracking_outcome, tracking_cmd, cmd_vel);
    message = state_.message.empty() ? to_string(state_.state) : state_.message;
    return mbf_msgs::action::ExePath::Result::SUCCESS;
  }

  bool isGoalReached(double dist_tolerance, double angle_tolerance) override
  {
    // Only a settled, dwelled goal counts.  The caller's tolerances are honoured as
    // an *upper* bound, so a stricter caller stays stricter and a looser one cannot
    // turn a settled-but-misaligned goal into a success.
    if (state_.state != TerminalState::FINISHED || !state_.goal_reached) {
      return false;
    }
    const double position_bound = dist_tolerance > 0.0
                                    ? std::min(dist_tolerance, params_.settle_radius)
                                    : params_.settle_radius;
    const double yaw_bound = angle_tolerance > 0.0
                               ? std::min(angle_tolerance, params_.yaw_tolerance)
                               : params_.yaw_tolerance;
    return distance_to_goal_ <= position_bound + 1.0e-9 &&
           std::fabs(last_yaw_error_) <= yaw_bound + 1.0e-9;
  }

private:
  double declare(const std::string & suffix, double fallback)
  {
    return node_->declare_parameter(name_ + "." + suffix, fallback);
  }
  bool declare_bool(const std::string & suffix, bool fallback)
  {
    return node_->declare_parameter(name_ + "." + suffix, fallback);
  }
  std::string declare_string(const std::string & suffix, const std::string & fallback)
  {
    return node_->declare_parameter(name_ + "." + suffix, fallback);
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm < 1.0e-9) {
      return 0.0;
    }
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  }

  /// Project the goal onto the robot's level and keep both goals.
  bool resolve_goal_level(const geometry_msgs::msg::PoseStamped & goal)
  {
    goal_projected_ = goal.pose;
    if (!map_) {
      return true;  // no mesh to check against; keep the goal as given
    }
    // Query at the robot's current level so the goal is projected onto the surface
    // the robot can actually reach, not onto a roof directly above it.
    mesh_map::Vector query(goal.pose.position.x, goal.pose.position.y, last_robot_z_);
    auto containing = map_->searchContainingFace(query, goal_search_radius_);
    if (!containing) {
      last_goal_error_ = "no mesh face within " + std::to_string(goal_search_radius_) +
                         " m of the goal on the robot's level";
      RCLCPP_ERROR(node_->get_logger(), "PbTerminalController: goal rejected: %s",
                   last_goal_error_.c_str());
      return false;
    }
    const mesh_map::Vector surface = mesh_map::linearCombineBarycentricCoords(
      std::get<1>(*containing), std::get<2>(*containing));
    const double surface_z = surface.z;
    const double height_error = std::fabs(surface_z - goal.pose.position.z);
    goal_projected_.position.z = surface_z;
    if (height_error > max_goal_height_error_) {
      last_goal_error_ = "goal height differs from the surface on the robot's level by " +
                         std::to_string(height_error) + " m";
      RCLCPP_ERROR(node_->get_logger(),
                   "PbTerminalController: goal rejected: %s (original z %.3f, level z %.3f)",
                   last_goal_error_.c_str(), goal.pose.position.z, surface_z);
      return false;
    }
    if (height_error > params_.level_tolerance) {
      // Same level, slightly different height: keep both goals and say so; never
      // silently claim the original was reached.
      RCLCPP_WARN(node_->get_logger(),
                  "PbTerminalController: goal projected onto the robot's level by %.3f m "
                  "(original z %.3f -> %.3f); arrival will be reported for the projected goal",
                  height_error, goal.pose.position.z, surface_z);
    }
    return true;
  }

  void publish_debug(const TerminalInput & in, uint32_t tracking_outcome,
                     const geometry_msgs::msg::TwistStamped & tracking_cmd,
                     const geometry_msgs::msg::TwistStamped & final_cmd)
  {
    if (!debug_pub_) {
      return;
    }
    // Read-only replication of the tracking layer's own view: the vector-field
    // direction it sees at the robot's face.  This is what separates "the field
    // points sideways" from "the robot's heading is wrong".
    double field_yaw = std::numeric_limits<double>::quiet_NaN();
    double field_phi = std::numeric_limits<double>::quiet_NaN();
    if (map_) {
      mesh_map::Vector query(last_pose_.position.x, last_pose_.position.y,
                             last_pose_.position.z);
      if (auto found = map_->searchContainingFace(query, goal_search_radius_)) {
        const auto handles = map_->mesh()->getVerticesOfFace(std::get<0>(*found));
        const auto & bary = std::get<2>(*found);
        const auto direction = map_->directionAtPosition(map_->getVectorMap(), handles, bary);
        if (direction) {
          field_yaw = std::atan2(direction->y, direction->x);
          field_phi = wrap_angle(field_yaw - in.robot_yaw);
        }
      }
    }

    std_msgs::msg::String message;
    char buffer[640];
    std::snprintf(buffer, sizeof(buffer),
                  "state=%s tracking_outcome=%u tracking=(%.3f,%.3f,%.3f) "
                  "final=(%.3f,%.3f,%.3f) robot=(%.3f,%.3f,%.3f,yaw %.3f) "
                  "goal=(%.3f,%.3f,%.3f) dx=%.3f dy=%.3f dz=%.3f yaw_err=%.3f "
                  "level_ok=%d healthy=%d valid=%d dt=%.3f field_yaw=%.3f field_phi=%.3f",
                  to_string(state_.state), tracking_outcome,
                  tracking_cmd.twist.linear.x, tracking_cmd.twist.linear.y,
                  tracking_cmd.twist.angular.z,
                  final_cmd.twist.linear.x, final_cmd.twist.linear.y,
                  final_cmd.twist.angular.z,
                  last_pose_.position.x, last_pose_.position.y, last_pose_.position.z,
                  in.robot_yaw, goal_projected_.position.x, goal_projected_.position.y,
                  goal_projected_.position.z, in.dx, in.dy, in.dz, in.yaw_error,
                  in.goal_valid && std::fabs(in.dz) <= params_.level_tolerance ? 1 : 0,
                  in.healthy ? 1 : 0, in.goal_valid ? 1 : 0, in.dt, field_yaw, field_phi);
    message.data = buffer;
    debug_pub_->publish(message);
  }

  void publish_state(TerminalState state, const std::string & detail)
  {
    if (!state_pub_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = to_string(state) + (detail.empty() ? "" : ": " + detail);
    state_pub_->publish(message);
  }

  std::string name_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<mesh_map::MeshMap> map_;
  mesh_controller::MeshController tracking_;

  TerminalParams params_;
  TerminalOutput state_;
  TerminalInput last_input_;

  geometry_msgs::msg::Pose goal_original_;
  geometry_msgs::msg::Pose goal_projected_;
  bool goal_valid_ = true;
  std::atomic_bool cancel_requested_{false};
  bool healthy_ = true;
  bool require_health_ = true;
  double max_goal_height_error_ = 0.25;
  double goal_search_radius_ = 0.4;
  double distance_to_goal_ = 0.0;
  double last_yaw_error_ = 0.0;
  double last_robot_z_ = 0.0;
  std::string health_topic_;
  std::string last_goal_error_;
  rclcpp::Time last_step_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  geometry_msgs::msg::Pose last_pose_;

  /// \brief Whether last_pose_ holds a previous step's pose.  Without a previous
  /// pose the travelled distance is unknown, and guessing zero would look like a
  /// frozen robot on the very first step.
  bool have_last_pose_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
};

}  // namespace pb_terminal_controller

PLUGINLIB_EXPORT_CLASS(pb_terminal_controller::PbTerminalController, mbf_mesh_core::MeshController)
