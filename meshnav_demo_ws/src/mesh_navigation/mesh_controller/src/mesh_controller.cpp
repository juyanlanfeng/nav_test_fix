/*
 *  Copyright 2020, Sebastian Pütz, Sabrina Frohn
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uos.de>
 *
 */

#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/util/Meap.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <mesh_controller/mesh_controller.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mbf_utility/exe_path_exception.h>

#include <cmath>
#include <limits>

PLUGINLIB_EXPORT_CLASS(mesh_controller::MeshController, mbf_mesh_core::MeshController);

#define DEBUG

#ifdef DEBUG
#define DEBUG_CALL(method) method
#else
#define DEBUG_CALL(method)
#endif

namespace mesh_controller
{

namespace
{

double clampd(const double value, const double lo, const double hi)
{
  return std::max(lo, std::min(hi, value));
}

double wrapAngle(const double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0)
  {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

}  // namespace

MeshController::MeshController()
{
}

MeshController::~MeshController()
{
}

uint32_t MeshController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                 const geometry_msgs::msg::TwistStamped& velocity,
                                                 geometry_msgs::msg::TwistStamped& cmd_vel,
                                                 std::string& message) 
{
  const auto mesh = map_ptr_->mesh();

  robot_pos_ = poseToPositionVector(pose);
  robot_dir_ = poseToDirectionVector(pose);
  // Keep the raw 3-D pose for the ramp corridor constraint and diagnostics; the
  // position below is projected onto the mesh surface and must not be mistaken
  // for the real robot location.
  const mesh_map::Vector raw_robot_pos = robot_pos_;
  std::array<float, 3> bary_coords;
  std::array<mesh_map::Vector, 3> vertices;

  if (!current_face_)
  {
    // initially search current face on complete map
    if (auto search_res_opt = map_ptr_->searchContainingFace(
            robot_pos_, config_.max_search_distance))
    {
      auto search_res = *search_res_opt;
      current_face_ = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);

      // project position onto surface
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "MeshController: no containing face found for raw pose (%.3f, %.3f, %.3f) "
          "within search distance %.3f m; reporting OUT_OF_MAP",
          raw_robot_pos.x, raw_robot_pos.y, raw_robot_pos.z,
          config_.max_search_distance);
      return mbf_msgs::action::ExePath::Result::OUT_OF_MAP;
    }
  }
  else // current face is set
  {
    lvr2::FaceHandle face = current_face_.unwrap();
    vertices = mesh->getVertexPositionsOfFace(face);
    DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(1, 1, 1), "current_face");)
    DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(1, 1, 1), "robot_position");)

    float dist_to_surface;
    // check whether or not the position matches the current face
    // if not search for new current face
    if (mesh_map::projectedBarycentricCoords(
        robot_pos_, vertices, bary_coords, dist_to_surface)
        && dist_to_surface < config_.max_search_distance)
    {
      // current position is located inside and close enough to the face
      DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(0, 0, 1), "current_position");)
    }
    else if (auto search_res_opt = map_ptr_->searchNeighbourFaces(
                 robot_pos_, face, config_.max_search_radius, config_.max_search_distance))
    {
      // new face has been found out of the neighbour faces of the current face
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face_ = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
      DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(1, 0.5, 0), "search_neighbour_face");)
      DEBUG_CALL(map_ptr_->publishDebugPoint(robot_pos_, mesh_map::color(0, 0, 1), "search_neighbour_pos");)
    }
    else if(auto search_res_opt = map_ptr_->searchContainingFace(
        robot_pos_, config_.max_search_distance))
    {
      // update variables to new face
      auto search_res = *search_res_opt;
      current_face_ = face = std::get<0>(search_res);
      vertices = std::get<1>(search_res);
      bary_coords = std::get<2>(search_res);
      robot_pos_ = mesh_map::linearCombineBarycentricCoords(vertices, bary_coords);
    }
    else
    {
      // no corresponding face has been found
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "MeshController: lost mesh surface near raw pose (%.3f, %.3f, %.3f); "
          "last face %d could not be extended (search radius %.3f m, distance %.3f m); "
          "reporting OUT_OF_MAP",
          raw_robot_pos.x, raw_robot_pos.y, raw_robot_pos.z,
          face.idx(), config_.max_search_radius, config_.max_search_distance);
      return mbf_msgs::action::ExePath::Result::OUT_OF_MAP;
    }
  }

  const lvr2::FaceHandle& face = current_face_.unwrap();
  std::array<lvr2::VertexHandle, 3> handles = mesh->getVerticesOfFace(face);

  // update to which position of the plan the robot is closest
  const auto& opt_dir = map_ptr_->directionAtPosition(vector_map_, handles, bary_coords);
  if (!opt_dir)
  {
    DEBUG_CALL(map_ptr_->publishDebugFace(face, mesh_map::color(0.3, 0.4, 0), "no_directions");)
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "MeshController: could not access the vector field on face %d for raw pose "
        "(%.3f, %.3f, %.3f) projected to (%.3f, %.3f, %.3f)",
        face.idx(), raw_robot_pos.x, raw_robot_pos.y, raw_robot_pos.z,
        robot_pos_.x, robot_pos_.y, robot_pos_.z);
    return mbf_msgs::action::ExePath::Result::FAILURE;
  }
  mesh_map::Normal mesh_dir = opt_dir.get().normalized();
  float cost = map_ptr_->costAtPosition(handles, bary_coords);
  const mesh_map::Normal& mesh_normal = poseToDirectionVector(pose, tf2::Vector3(0,0,1));
  const std::array<float, 3> velocities =
      naiveControl(robot_pos_, robot_dir_, mesh_dir, mesh_normal, cost);

  double linear_x = velocities[0] * config_.lin_vel_factor;
  double linear_y = velocities[1] * config_.lin_vel_factor;
  double angular_z = velocities[2] * config_.ang_vel_factor;

  // RMUC ramp corridor constraint (phase 2).  This runs after the regular vector
  // field control and before the final clamping/assignment so that a matched
  // corridor can override the lateral and heading commands while traversing a ramp.
  applyCorridorConstraint(raw_robot_pos, linear_x, linear_y, angular_z, message);

  const double linear_speed = std::hypot(linear_x, linear_y);
  if (linear_speed > config_.max_lin_velocity)
  {
    const double scale = config_.max_lin_velocity / linear_speed;
    linear_x *= scale;
    linear_y *= scale;
  }

  cmd_vel.twist.linear.x = linear_x;
  cmd_vel.twist.linear.y = linear_y;
  cmd_vel.twist.angular.z = std::max(
      -config_.max_ang_velocity, std::min(config_.max_ang_velocity, angular_z));
  cmd_vel.header.stamp = node_->now();

  if (cancel_requested_)
  {
    return mbf_msgs::action::ExePath::Result::CANCELED;
  }
  return mbf_msgs::action::ExePath::Result::SUCCESS;
}

bool MeshController::isGoalReached(double dist_tolerance, double angle_tolerance)
{
  float goal_distance = (goal_pos_ - robot_pos_).length();
  float angle = acos(goal_dir_.dot(robot_dir_));
  return goal_distance <= static_cast<float>(dist_tolerance) && angle <= static_cast<float>(angle_tolerance);
}

bool MeshController::setPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  // copy vector field // TODO just use vector field without copying
  vector_map_ = map_ptr_->getVectorMap();
  DEBUG_CALL(map_ptr_->publishDebugPoint(poseToPositionVector(plan.front()), mesh_map::color(0, 1, 0), "plan_start");)
  DEBUG_CALL(map_ptr_->publishDebugPoint(poseToPositionVector(plan.back()), mesh_map::color(1, 0, 0), "plan_goal");)
  current_plan_ = plan;
  goal_pos_ = poseToPositionVector(current_plan_.back());
  goal_dir_ = poseToDirectionVector(current_plan_.back());

  // reset current and ahead face
  cancel_requested_ = false;
  current_face_ = lvr2::OptionalFaceHandle();
  resetCorridor();
  return true;
}

bool MeshController::cancel()
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "The MeshController has been requested to cancel!");
  cancel_requested_ = true;
  resetCorridor();
  return true;
}

mesh_map::Normal MeshController::poseToDirectionVector(const geometry_msgs::msg::PoseStamped& pose, const tf2::Vector3& axis)
{
  tf2::Transform transform;
  geometry_msgs::msg::Transform geom_transform;
  geom_transform.rotation = pose.pose.orientation;
  geom_transform.translation.x = pose.pose.position.x;
  geom_transform.translation.y = pose.pose.position.y;
  geom_transform.translation.z = pose.pose.position.z;
  tf2::fromMsg(geom_transform, transform);
  tf2::Vector3 v = transform.getBasis() * axis;
  return mesh_map::Normal(v.x(), v.y(), v.z());
}

mesh_map::Vector MeshController::poseToPositionVector(const geometry_msgs::msg::PoseStamped& pose)
{
  return mesh_map::Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

float MeshController::gaussValue(const float& sigma_squared, const float& value)
{
  return exp(-value * value / 2 * sigma_squared) / sqrt(2 * M_PI * sigma_squared);
}

std::array<float, 3> MeshController::naiveControl(
    const mesh_map::Vector& robot_pos,
    const mesh_map::Normal& robot_dir,
    const mesh_map::Vector& mesh_dir,
    const mesh_map::Normal& mesh_normal,
    const float& mesh_cost)
{
  float phi = acos(mesh_dir.dot(robot_dir));
  float sign_phi = mesh_dir.cross(robot_dir).dot(mesh_normal);
  // debug output angle between supposed and current angle
  DEBUG_CALL(example_interfaces::msg::Float32 angle32; angle32.data = phi * 180 / M_PI; angle_pub_->publish(angle32);)

  float angular_velocity = copysignf(phi * config_.max_ang_velocity / M_PI, -sign_phi);
  const float max_linear = config_.max_lin_velocity;

  if (config_.holonomic)
  {
    // Express the mesh vector field in the robot's tangent-plane frame.  A
    // holonomic base can follow this vector immediately while independently
    // rotating its body towards the path direction.
    const mesh_map::Normal robot_left = mesh_normal.cross(robot_dir).normalized();
    const float linear_x = max_linear * mesh_dir.dot(robot_dir);
    const float linear_y = max_linear * mesh_dir.dot(robot_left);
    return {linear_x, linear_y, angular_velocity};
  }

  const float max_angle = config_.max_angle * M_PI / 180.0;
  float linear_velocity = phi <= max_angle ? max_linear - (phi * max_linear / max_angle) : 0.0;
  return {linear_velocity, 0.0, angular_velocity};
}

rcl_interfaces::msg::SetParametersResult MeshController::reconfigureCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    if (parameter.get_name() == name_ + ".max_lin_velocity") {
      config_.max_lin_velocity = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_ang_velocity") {
      config_.max_ang_velocity= parameter.as_double();
    } else if (parameter.get_name() == name_ + ".arrival_fading") {
      config_.arrival_fading = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".ang_vel_factor") {
      config_.ang_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".lin_vel_factor") {
      config_.lin_vel_factor = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".holonomic") {
      config_.holonomic = parameter.as_bool();
    } else if (parameter.get_name() == name_ + ".max_angle") {
      config_.max_angle = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_radius") {
      config_.max_search_radius = parameter.as_double();
    } else if (parameter.get_name() == name_ + ".max_search_distance") {
      config_.max_search_distance = parameter.as_double();
    }
  }

  result.successful = true;
  return result;
}

bool MeshController::initialize(const std::string& plugin_name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf_ptr,
                                const std::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr,
                                const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  map_ptr_ = mesh_map_ptr;
  name_ = plugin_name;

  angle_pub_ = node_->create_publisher<example_interfaces::msg::Float32>("~/current_angle", rclcpp::QoS(1).transient_local());
  corridor_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/ramp_corridors", rclcpp::QoS(1).transient_local());

  loadCorridors();

  { // cost max_lin_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_lin_velocity = node->declare_parameter(name_ + ".max_lin_velocity", config_.max_lin_velocity);
  }
  { // cost max_ang_velocity
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Defines the maximum angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_ang_velocity = node->declare_parameter(name_ + ".max_ang_velocity", config_.max_ang_velocity);
  }
  { // cost arrival_fading
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Distance to goal position where the robot starts to fade down the linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.0;
    range.to_value = 5.0;
    descriptor.floating_point_range.push_back(range);
    config_.arrival_fading = node->declare_parameter(name_ + ".arrival_fading", config_.arrival_fading);
  }
  { // cost ang_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for angular velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.ang_vel_factor = node->declare_parameter(name_ + ".ang_vel_factor", config_.ang_vel_factor);
  }
  { // cost lin_vel_factor
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Factor for linear velocity";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.1;
    range.to_value = 10.0;
    descriptor.floating_point_range.push_back(range);
    config_.lin_vel_factor = node->declare_parameter(name_ + ".lin_vel_factor", config_.lin_vel_factor);
  }
  { // holonomic base
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
        "Enable planar holonomic velocity commands (linear.x, linear.y and angular.z)";
    config_.holonomic = node->declare_parameter(name_ + ".holonomic", config_.holonomic, descriptor);
  }
  { // cost max_angle
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum angle for the linear velocity function";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 1.0;
    range.to_value = 180.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_angle = node->declare_parameter(name_ + ".max_angle", config_.max_angle);
  }
  { // cost max_search_radius
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum radius in which to search for a consecutive neighbour face";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_radius = node->declare_parameter(name_ + ".max_search_radius", config_.max_search_radius);
  }
  { // cost max_search_distance
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "The maximum distance from the surface which is accepted for projection";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 2.0;
    descriptor.floating_point_range.push_back(range);
    config_.max_search_distance = node->declare_parameter(name_ + ".max_search_distance", config_.max_search_distance);
  }

  reconfiguration_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(
      &MeshController::reconfigureCallback, this, std::placeholders::_1));

  return true;
}

void MeshController::RampCorridor::project(const mesh_map::Vector& p, double& s,
                                           mesh_map::Vector& closest, double& lateral,
                                           mesh_map::Vector& tangent, double& tangent_yaw) const
{
  if (centerline.size() < 2)
  {
    s = 0.0;
    closest = centerline.empty() ? mesh_map::Vector(0, 0, 0) : centerline.front();
    lateral = 0.0;
    tangent = mesh_map::Vector(1, 0, 0);
    tangent_yaw = 0.0;
    return;
  }

  std::size_t best_i = 0;
  double best_t = 0.0;
  double best_dist2 = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < centerline.size(); ++i)
  {
    const mesh_map::Vector& a = centerline[i];
    const mesh_map::Vector& b = centerline[i + 1];
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    double t = (len2 > 0.0) ? ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2 : 0.0;
    t = clampd(t, 0.0, 1.0);
    const double cx = a.x + t * abx;
    const double cy = a.y + t * aby;
    const double d2 = (p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy);
    if (d2 < best_dist2)
    {
      best_dist2 = d2;
      best_i = i;
      best_t = t;
    }
  }

  const mesh_map::Vector& a = centerline[best_i];
  const mesh_map::Vector& b = centerline[best_i + 1];
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double seglen = std::hypot(abx, aby);
  const double ux = seglen > 0.0 ? abx / seglen : 1.0;
  const double uy = seglen > 0.0 ? aby / seglen : 0.0;
  closest = mesh_map::Vector(a.x + best_t * abx, a.y + best_t * aby, a.z + best_t * (b.z - a.z));
  const double s_clamped = seg_len[best_i] + best_t * seglen;
  tangent = mesh_map::Vector(ux, uy, 0.0);
  tangent_yaw = std::atan2(uy, ux);
  // Signed lateral offset: z-component of tangent x (p - closest); positive = left.
  lateral = ux * (p.y - closest.y) - uy * (p.x - closest.x);

  // Signed longitudinal progress: extend beyond the polyline ends using the
  // unclamped projection onto the first/last segment so points before the entry
  // report s < 0 and points past the exit report s > total_len.  Without this the
  // clamped projection maps both to the endpoints and APPROACH/EXIT never fire.
  s = s_clamped;
  if (best_i == 0 && best_t == 0.0)
  {
    const double d0x = centerline[1].x - centerline[0].x;
    const double d0y = centerline[1].y - centerline[0].y;
    const double len0 = std::hypot(d0x, d0y);
    if (len0 > 0.0)
    {
      const double t0 = ((p.x - centerline[0].x) * d0x + (p.y - centerline[0].y) * d0y) / (len0 * len0);
      if (t0 < 0.0)
      {
        s = t0 * len0;
      }
    }
  }
  else if (best_i + 2 == centerline.size() && best_t == 1.0)
  {
    const double dLx = b.x - a.x;
    const double dLy = b.y - a.y;
    const double lenL = std::hypot(dLx, dLy);
    if (lenL > 0.0)
    {
      const double tL = ((p.x - a.x) * dLx + (p.y - a.y) * dLy) / (lenL * lenL);
      if (tL > 1.0)
      {
        s = seg_len[best_i] + tL * lenL;
      }
    }
  }
}

void MeshController::loadCorridors()
{
  corridors_.clear();
  corridors_enabled_ = node_->declare_parameter(name_ + ".ramp_corridors_enabled", false);
  const std::vector<std::string> names =
      node_->declare_parameter(name_ + ".ramp_corridors", std::vector<std::string>{});

  for (const std::string& nm : names)
  {
    RampCorridor corridor;
    corridor.name = nm;
    const std::string prefix = name_ + "." + nm + ".";

    corridor.frame_id = node_->declare_parameter(prefix + "frame_id", std::string("map"));
    const std::vector<double> centerline =
        node_->declare_parameter(prefix + "centerline", std::vector<double>{});
    if (centerline.size() < 6 || centerline.size() % 3 != 0)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
          "Ramp corridor '" << nm << "' has an invalid centerline (needs >= 2 XYZ "
          "points, i.e. >= 6 numbers); ignoring it.");
      continue;
    }
    for (std::size_t k = 0; k + 2 < centerline.size(); k += 3)
    {
      corridor.centerline.emplace_back(centerline[k], centerline[k + 1], centerline[k + 2]);
    }

    corridor.half_width = node_->declare_parameter(prefix + "half_width", corridor.half_width);
    corridor.min_height = node_->declare_parameter(prefix + "min_height", corridor.min_height);
    corridor.max_height = node_->declare_parameter(prefix + "max_height", corridor.max_height);
    corridor.max_speed = node_->declare_parameter(prefix + "max_speed", corridor.max_speed);
    corridor.approach_speed = node_->declare_parameter(prefix + "approach_speed", corridor.approach_speed);
    corridor.align_dist = node_->declare_parameter(prefix + "align_dist", corridor.align_dist);
    corridor.align_lat_tol = node_->declare_parameter(prefix + "align_lat_tol", corridor.align_lat_tol);
    corridor.align_yaw_tol =
        node_->declare_parameter(prefix + "align_yaw_tol_deg", 5.0) * M_PI / 180.0;
    corridor.align_hold_s = node_->declare_parameter(prefix + "align_hold_s", corridor.align_hold_s);
    corridor.exit_dist = node_->declare_parameter(prefix + "exit_dist", corridor.exit_dist);
    corridor.k_lat = node_->declare_parameter(prefix + "k_lat", corridor.k_lat);
    corridor.k_yaw = node_->declare_parameter(prefix + "k_yaw", corridor.k_yaw);

    // Build cumulative (XY) arc lengths.
    corridor.seg_len.assign(corridor.centerline.size(), 0.0);
    corridor.total_len = 0.0;
    for (std::size_t i = 0; i + 1 < corridor.centerline.size(); ++i)
    {
      const double dx = corridor.centerline[i + 1].x - corridor.centerline[i].x;
      const double dy = corridor.centerline[i + 1].y - corridor.centerline[i].y;
      corridor.seg_len[i + 1] = corridor.seg_len[i] + std::hypot(dx, dy);
      corridor.total_len = corridor.seg_len[i + 1];
    }

    RCLCPP_INFO_STREAM(node_->get_logger(),
        "Loaded ramp corridor '" << nm << "' with " << corridor.centerline.size()
        << " points and length " << corridor.total_len << " m (frame " << corridor.frame_id << ")");
    corridors_.push_back(std::move(corridor));
  }

  RCLCPP_INFO_STREAM(node_->get_logger(),
      "Ramp corridor constraint is " << (corridors_enabled_ ? "enabled" : "disabled")
      << " with " << corridors_.size() << " corridor(s)");
}

void MeshController::resetCorridor()
{
  active_corridor_ = -1;
  corridor_phase_ = CorridorPhase::NONE;
  corridor_direction_ = 1;
  corridor_s_ = 0.0;
  aligned_since_ = -1.0;
  stall_window_start_s_ = 0.0;
  stall_window_start_t_ = -1.0;
  corridor_stalled_ = false;
}

int MeshController::findActiveCorridor(const mesh_map::Vector& raw_pos,
                                       double& s, double& lateral,
                                       mesh_map::Vector& tangent, double& tangent_yaw)
{
  int best = -1;
  double best_lateral = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(corridors_.size()); ++i)
  {
    const RampCorridor& c = corridors_[i];
    double s_i = 0.0, lat = 0.0, tan_yaw = 0.0;
    mesh_map::Vector closest, tan;
    c.project(raw_pos, s_i, closest, lat, tan, tan_yaw);

    const bool in_region = (s_i >= -c.align_dist) && (s_i <= c.total_len + c.exit_dist);
    const bool in_height = (raw_pos.z >= c.min_height) && (raw_pos.z <= c.max_height);
    const bool in_lateral = std::fabs(lat) <= c.half_width + 0.2;
    if (!in_region || !in_height || !in_lateral)
    {
      continue;
    }
    if (planCrossingDirection(c) == 0)
    {
      continue;
    }
    if (std::fabs(lat) < best_lateral)
    {
      best = i;
      best_lateral = std::fabs(lat);
      s = s_i;
      lateral = lat;
      tangent = tan;
      tangent_yaw = tan_yaw;
    }
  }
  return best;
}

int MeshController::planCrossingDirection(const RampCorridor& corridor) const
{
  // Classifies the ordered plan relative to the corridor using the first and last
  // plan poses:
  //   * full crossing (plan spans entry and exit) -> +/-1 by plan order,
  //   * traversal starting *inside* the corridor whose remaining path leaves
  //     through one end (first pose inside, last pose outside) -> direction
  //     toward that end,
  //   * goal inside the corridor, entering from outside to an inside goal, or a
  //     wrong terrain layer -> 0 (no engagement).
  if (current_plan_.empty())
  {
    return 0;
  }

  enum class Side { ENTRY, INSIDE, EXIT };

  bool entry_side = false;   // a plan pose strictly before the entry
  bool exit_side = false;    // a plan pose strictly after the exit
  int first_side = 0;        // +1 entry side seen first, -1 exit side seen first
  bool in_corridor = false;  // at least one pose inside on the right layer
  Side first_class = Side::ENTRY;
  Side last_class = Side::ENTRY;
  bool have_first = false;

  for (const auto& pose : current_plan_)
  {
    const mesh_map::Vector p(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    double s = 0.0, lat = 0.0, tan_yaw = 0.0;
    mesh_map::Vector closest, tan;
    corridor.project(p, s, closest, lat, tan, tan_yaw);

    Side cls;
    if (s < -0.05)
    {
      cls = Side::ENTRY;
      entry_side = true;
      if (first_side == 0)
      {
        first_side = 1;
      }
    }
    else if (s > corridor.total_len + 0.05)
    {
      cls = Side::EXIT;
      exit_side = true;
      if (first_side == 0)
      {
        first_side = -1;
      }
    }
    else
    {
      cls = Side::INSIDE;
      const bool right_layer = std::fabs(lat) <= corridor.half_width + 0.1 &&
          p.z >= corridor.min_height && p.z <= corridor.max_height;
      if (!right_layer)
      {
        // The plan passes through the corridor XY on a different height layer or
        // well outside it laterally: not a clean ramp crossing.
        return 0;
      }
      in_corridor = true;
    }

    if (!have_first)
    {
      first_class = cls;
      have_first = true;
    }
    last_class = cls;
  }

  if (!in_corridor)
  {
    return 0;
  }

  // Full crossing: the plan spans both ends; direction follows the plan order.
  if (entry_side && exit_side)
  {
    return first_side;
  }

  // Continuation: only valid when the plan *starts inside* the corridor and
  // leaves through one end (the remaining path).  Direction points toward that
  // end.  This rejects entering-from-outside to an inside goal (first pose
  // outside) and inside goals (last pose inside), which must not be taken over.
  if (first_class == Side::INSIDE && last_class != Side::INSIDE)
  {
    return (last_class == Side::EXIT) ? 1 : -1;
  }

  // Goal inside the corridor (or outside-to-inside): no traversal to take over.
  return 0;
}

void MeshController::applyCorridorConstraint(const mesh_map::Vector& raw_pos,
                                             double& linear_x, double& linear_y, double& angular_z,
                                             std::string& message)
{
  if (!corridors_enabled_ || corridors_.empty())
  {
    return;
  }

  int idx = active_corridor_;
  double s = 0.0, lateral = 0.0, tangent_yaw = 0.0;
  mesh_map::Vector tangent, closest;

  if (idx < 0 || idx >= static_cast<int>(corridors_.size()))
  {
    // No corridor is active: only engage a *new* one inside the engagement
    // region (which stops before the exit zone).  Engagement also requires a
    // genuine plan crossing, otherwise normal control keeps running.
    idx = findActiveCorridor(raw_pos, s, lateral, tangent, tangent_yaw);
    if (idx < 0)
    {
      return;
    }
    const int dir = planCrossingDirection(corridors_[idx]);
    if (dir == 0)
    {
      return;
    }
    active_corridor_ = idx;
    corridor_direction_ = dir;
    corridor_phase_ = CorridorPhase::NONE;
    corridor_stalled_ = false;
    aligned_since_ = -1.0;
    stall_window_start_s_ = 0.0;
    stall_window_start_t_ = -1.0;
  }
  else
  {
    // A corridor is already active: keep tracking it *past* the exit zone so the
    // exit check (tail clear) can run before control is handed back.
    corridors_[idx].project(raw_pos, s, closest, lateral, tangent, tangent_yaw);
  }

  const RampCorridor& corridor = corridors_[idx];

  // Latched stall: keep the robot stopped and preserve the stall message until
  // setPlan()/cancel() resets the corridor.
  if (corridor_stalled_)
  {
    linear_x = 0.0;
    linear_y = 0.0;
    angular_z = 0.0;
    message = "ramp corridor stall: no progress while commanding forward speed";
    return;
  }

  const int dir = corridor_direction_;
  corridor_s_ = s;

  // Heading and lateral error in the *travel* frame (depends on direction).
  const double yaw = std::atan2(robot_dir_.y, robot_dir_.x);
  const double travel_yaw = tangent_yaw + ((dir >= 0) ? 0.0 : M_PI);
  const double heading_err = wrapAngle(yaw - travel_yaw);
  const double lateral_travel = dir * lateral;

  // Signed progress relative to the entry (0 at entry, total_len at exit) along
  // the planned direction; negative = still approaching, > total_len = past exit.
  const double entry_s = (dir >= 0) ? 0.0 : corridor.total_len;
  const double progress = dir * (s - entry_s);

  if (progress > corridor.total_len + corridor.exit_dist)
  {
    corridor_phase_ = CorridorPhase::EXIT;
  }
  else
  {
    const bool aligned = std::fabs(lateral_travel) <= corridor.align_lat_tol &&
                         std::fabs(heading_err) <= corridor.align_yaw_tol;
    if (aligned)
    {
      if (aligned_since_ < 0.0)
      {
        aligned_since_ = node_->now().seconds();
      }
    }
    else
    {
      aligned_since_ = -1.0;
    }

    const bool entering = (progress >= 0.0);
    const bool aligned_held = aligned_since_ >= 0.0 &&
        (node_->now().seconds() - aligned_since_) >= corridor.align_hold_s;
    if (!entering)
    {
      corridor_phase_ = CorridorPhase::APPROACH;
    }
    else if (aligned_held)
    {
      corridor_phase_ = CorridorPhase::TRAVERSE;
    }
    else
    {
      corridor_phase_ = CorridorPhase::ALIGN;
    }
  }

  if (corridor_phase_ == CorridorPhase::EXIT)
  {
    resetCorridor();
    return;
  }

  double v_forward = 0.0;
  double v_lateral = 0.0;
  double w = 0.0;
  switch (corridor_phase_)
  {
    case CorridorPhase::APPROACH:
    {
      // Slow the advance while still off-centre or misaligned so the robot first
      // centres and aligns on the platform instead of driving onto the ramp
      // already leaning into a side structure.
      const double lat_penalty = std::min(1.0, std::fabs(lateral_travel) / std::max(corridor.half_width, 1e-3));
      const double yaw_penalty = std::min(1.0, std::fabs(heading_err) / std::max(corridor.align_yaw_tol * 4.0, 1e-3));
      v_forward = corridor.approach_speed * std::max(0.0, 1.0 - 0.7 * lat_penalty - 0.3 * yaw_penalty);
      v_lateral = -corridor.k_lat * lateral_travel;
      w = -corridor.k_yaw * heading_err;
      break;
    }
    case CorridorPhase::ALIGN:
      v_forward = 0.0;
      v_lateral = -corridor.k_lat * lateral_travel;
      w = -corridor.k_yaw * heading_err;
      break;
    case CorridorPhase::TRAVERSE:
      v_forward = corridor.max_speed;
      v_lateral = -corridor.k_lat * lateral_travel;
      w = -corridor.k_yaw * heading_err;
      break;
    case CorridorPhase::EXIT:
    case CorridorPhase::NONE:
      break;
  }

  v_forward = clampd(v_forward, 0.0, corridor.max_speed);
  v_lateral = clampd(v_lateral, -0.15, 0.15);
  w = clampd(w, -config_.max_ang_velocity, config_.max_ang_velocity);

  // Corridor travel-frame velocities -> world -> body frame (the base is
  // holonomic, so cmd_vel.twist.linear.x/y are body-frame components).  The
  // travel tangent and left normal depend on the traversal direction.
  const double tx = dir * tangent.x;
  const double ty = dir * tangent.y;
  const double world_vx = tx * v_forward + (-ty) * v_lateral;
  const double world_vy = ty * v_forward + tx * v_lateral;
  const double dx = std::cos(yaw);
  const double dy = std::sin(yaw);
  linear_x = world_vx * dx + world_vy * dy;
  linear_y = world_vx * (-dy) + world_vy * dx;
  angular_z = w;

  // Stall detection while traversing: a persistent forward command with no
  // along-corridor progress is reported, latched and turned into a safe stop.
  if (corridor_phase_ == CorridorPhase::TRAVERSE)
  {
    const double commanded = std::hypot(linear_x, linear_y);
    const double now = node_->now().seconds();
    if (commanded <= 0.05)
    {
      stall_window_start_t_ = -1.0;
    }
    else if (stall_window_start_t_ < 0.0)
    {
      stall_window_start_t_ = now;
      stall_window_start_s_ = s;
    }
    else if ((now - stall_window_start_t_) >= 2.0)
    {
      if (std::fabs(s - stall_window_start_s_) < 0.02)
      {
        corridor_stalled_ = true;
        linear_x = 0.0;
        linear_y = 0.0;
        angular_z = 0.0;
        message = "ramp corridor stall: no progress while commanding forward speed";
        RCLCPP_ERROR_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "MeshController: ramp corridor '%s' stalled (commanded %.3f m/s, "
            "progress %.4f m over %.1f s); holding in place",
            corridor.name.c_str(), commanded, std::fabs(s - stall_window_start_s_), now - stall_window_start_t_);
      }
      else
      {
        stall_window_start_t_ = now;
        stall_window_start_s_ = s;
      }
    }
  }

  publishCorridorMarkers(corridor);

  // Do not overwrite a latched stall message with the normal phase string.
  if (corridor_stalled_)
  {
    return;
  }

  const char* phase_str = "none";
  switch (corridor_phase_)
  {
    case CorridorPhase::APPROACH: phase_str = "approach"; break;
    case CorridorPhase::ALIGN: phase_str = "align"; break;
    case CorridorPhase::TRAVERSE: phase_str = "traverse"; break;
    case CorridorPhase::EXIT: phase_str = "exit"; break;
    case CorridorPhase::NONE: phase_str = "none"; break;
  }
  message = std::string("ramp corridor '") + corridor.name + "' (" + phase_str +
            ", s=" + std::to_string(s) + ", lat=" + std::to_string(lateral) + ")";
}

void MeshController::publishCorridorMarkers(const RampCorridor& corridor)
{
  if (!corridor_marker_pub_)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  auto add_line = [&](int id, const std::vector<mesh_map::Vector>& pts, double r, double g, double b)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = corridor.frame_id;
    m.header.stamp = node_->now();
    m.ns = "ramp_corridor_" + corridor.name;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
    for (const auto& p : pts)
    {
      geometry_msgs::msg::Point q;
      q.x = p.x; q.y = p.y; q.z = p.z;
      m.points.push_back(q);
    }
    array.markers.push_back(m);
  };

  // Centre line (green) and keep-in boundaries (orange).
  std::vector<mesh_map::Vector> left, right;
  for (std::size_t i = 0; i < corridor.centerline.size(); ++i)
  {
    const mesh_map::Vector& p = corridor.centerline[i];
    mesh_map::Vector u;
    if (i + 1 < corridor.centerline.size())
    {
      double ux = corridor.centerline[i + 1].x - p.x;
      double uy = corridor.centerline[i + 1].y - p.y;
      const double len = std::hypot(ux, uy);
      ux = len > 0.0 ? ux / len : 1.0;
      uy = len > 0.0 ? uy / len : 0.0;
      u = mesh_map::Vector(ux, uy, 0.0);
    }
    else
    {
      double ux = p.x - corridor.centerline[i - 1].x;
      double uy = p.y - corridor.centerline[i - 1].y;
      const double len = std::hypot(ux, uy);
      ux = len > 0.0 ? ux / len : 1.0;
      uy = len > 0.0 ? uy / len : 0.0;
      u = mesh_map::Vector(ux, uy, 0.0);
    }
    const double nx = -u.y;
    const double ny = u.x;
    left.emplace_back(p.x + nx * corridor.half_width, p.y + ny * corridor.half_width, p.z);
    right.emplace_back(p.x - nx * corridor.half_width, p.y - ny * corridor.half_width, p.z);
  }

  add_line(0, corridor.centerline, 0.0, 1.0, 0.0);
  add_line(1, left, 1.0, 0.5, 0.0);
  add_line(2, right, 1.0, 0.5, 0.0);

  corridor_marker_pub_->publish(array);
}

} /* namespace mesh_controller */
