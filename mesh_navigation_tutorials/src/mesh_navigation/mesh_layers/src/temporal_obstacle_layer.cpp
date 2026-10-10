#include <mesh_layers/temporal_obstacle_layer.h>
#include <mesh_map/mesh_map.h>
#include <mesh_map/timer.h>
#include "stvl_frustum/three_dimensional_lidar_frustum.hpp"

#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace mesh_layers
{
TemporalObstacleLayer::TemporalObstacleLayer() = default;
TemporalObstacleLayer::~TemporalObstacleLayer() = default;

bool TemporalObstacleLayer::ObservationKey::operator==(const ObservationKey& other) const
{
  return x == other.x && y == other.y && z == other.z && face == other.face;
}

size_t TemporalObstacleLayer::KeyHash::operator()(const ObservationKey& key) const
{
  size_t seed = std::hash<size_t>{}(key.face);
  for (const auto value : {key.x, key.y, key.z})
    seed ^= std::hash<int64_t>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

bool TemporalObstacleLayer::initialize()
{
  fixed_parameters_ = true;
  config_.max_obstacle_dist = 8.0;
  config_.robot_height = 0.50;
  history_.clear();
  previous_pose_.reset();
  last_stamp_.reset();
  last_clock_.reset();
  history_mesh_.reset();

  const auto parameter = [this](const std::string& name, const auto& value,
                               const std::string& description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description + " Change YAML and restart to reconfigure.";
    return node_->declare_parameter(layer_namespace_ + "." + name, value, descriptor);
  };
  history_voxel_size_ = parameter("history_voxel_size", 0.05, "History deduplication voxel size [m].");
  clearing_mode_ = parameter("clearing_mode", std::string("stvl"),
    "Temporal clearing: stvl (age decay) or visible_timeout (legacy missing-return window).");
  decay_model_ = parameter("decay_model", int64_t{0}, "STVL decay model: 0 linear, 1 exponential, -1 persistent.");
  decay_acceleration_ = parameter("decay_acceleration", 0.0,
    "STVL frustum acceleration [1/s^2]; zero disables accelerated clearing of unobserved surfaces.");
  const double keep = parameter("obstacle_keep_time", 10.0,
    "STVL voxel_decay parameter; linear lifetime [s], exponential scale, ignored for persistent.");
  const double visible = parameter("visible_keep_time", 0.10,
    "Legacy visible_timeout mode only: continuous missing-in-view window [s].");
  const double max_age = parameter("max_observation_age", 0.30, "Maximum observation age at commit [s].");
  sensor_frame_ = parameter("sensor_frame", std::string("front_mid360"), "Lidar TF frame.");
  const auto offset = parameter("sensor_offset_xyz", std::vector<double>{0.0, 0.0, 0.03},
    "Measurement origin offset in the lidar link frame [m].");
  min_sensor_range_ = parameter("min_sensor_range", 0.10, "Minimum spherical clearing range [m].");
  horizontal_fov_deg_ = parameter("horizontal_fov_deg", 360.0, "Horizontal FOV about sensor +X [deg].");
  vertical_min_deg_ = parameter("vertical_fov_min_deg", -7.0, "Minimum elevation [deg].");
  vertical_max_deg_ = parameter("vertical_fov_max_deg", 52.0, "Maximum elevation [deg].");
  reference_frame_ = parameter("localization_reference_frame", std::string("odom"),
    "Continuous odometry reference used to detect localization jumps.");
  jump_translation_ = parameter("pose_jump_translation", 0.75, "Pose jump translation threshold [m].");
  const double jump_degrees = parameter("pose_jump_rotation_deg", 45.0, "Pose jump angle threshold [deg].");
  const double jump_interval = parameter("pose_jump_max_interval", 0.20,
    "Maximum frame interval for sensor-pose jump checks [s].");

  const auto duration_valid = [](double value)
  {
    // Leave headroom for seconds-to-nanoseconds conversion and subtraction.
    return std::isfinite(value) && value >= 0.0 && value <= 1e9;
  };
  if ((clearing_mode_ != "stvl" && clearing_mode_ != "visible_timeout") ||
      (decay_model_ != -1 && decay_model_ != 0 && decay_model_ != 1) ||
      !std::isfinite(decay_acceleration_) || decay_acceleration_ < 0.0 ||
      !std::isfinite(history_voxel_size_) || history_voxel_size_ <= 0.0 ||
      !duration_valid(keep) || !duration_valid(visible) ||
      (clearing_mode_ == "visible_timeout" && visible > keep) ||
      !duration_valid(max_age) || !duration_valid(jump_interval) ||
      sensor_frame_.empty() || reference_frame_.empty() || offset.size() != 3 ||
      !std::isfinite(min_sensor_range_) || min_sensor_range_ < 0.0 ||
      !std::isfinite(horizontal_fov_deg_) || horizontal_fov_deg_ <= 0.0 || horizontal_fov_deg_ > 360.0 ||
      !std::isfinite(vertical_min_deg_) || !std::isfinite(vertical_max_deg_) ||
      vertical_min_deg_ <= -90.0 || vertical_max_deg_ >= 90.0 || vertical_min_deg_ >= vertical_max_deg_ ||
      !std::isfinite(jump_translation_) || jump_translation_ <= 0.0 ||
      !std::isfinite(jump_degrees) || jump_degrees <= 0.0 || jump_degrees > 180.0)
  {
    RCLCPP_ERROR(get_logger(), "Invalid temporal obstacle parameters; check ranges, lifetimes and FOV limits");
    return false;
  }
  sensor_offset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
  if (!sensor_offset_.allFinite())
  {
    RCLCPP_ERROR(get_logger(), "sensor_offset_xyz must contain three finite values");
    return false;
  }
  keep_ns_ = rclcpp::Duration::from_seconds(keep).nanoseconds();
  visible_ns_ = rclcpp::Duration::from_seconds(visible).nanoseconds();
  max_age_ns_ = rclcpp::Duration::from_seconds(max_age).nanoseconds();
  jump_interval_ns_ = rclcpp::Duration::from_seconds(jump_interval).nanoseconds();
  jump_rotation_ = jump_degrees * M_PI / 180.0;

  rcl_jump_threshold_t threshold{};
  threshold.on_clock_change = true;
  threshold.min_backward.nanoseconds = -1;
  clock_jump_handler_ = node_->get_clock()->create_jump_callback(nullptr,
    [this](const rcl_time_jump_t&) { clock_epoch_.fetch_add(1); }, threshold);
  accepted_epoch_ = clock_epoch_.load();
  // Base initialization declares projection parameters, calls configureProjection(),
  // and creates the subscription only when the complete model is ready.
  return ObstacleLayer::initialize();
}

bool TemporalObstacleLayer::configureProjection()
{
  if (min_sensor_range_ >= config_.max_obstacle_dist)
  {
    RCLCPP_ERROR(get_logger(), "min_sensor_range must be smaller than max_obstacle_dist");
    return false;
  }
  const double lower = std::tan(vertical_min_deg_ * M_PI / 180.0);
  const double upper = std::tan(vertical_max_deg_ * M_PI / 180.0);
  frustum_ = std::make_unique<stvl::ThreeDimensionalLidarFrustum>(
    2.0 * std::atan((upper - lower) / 2.0), std::atan((upper + lower) / 2.0), 0.0,
    horizontal_fov_deg_ * M_PI / 180.0, min_sensor_range_, config_.max_obstacle_dist);
  return true;
}

std::optional<TemporalObstacleLayer::ObservationKey> TemporalObstacleLayer::makeKey(
  const ProjectedObservation& observation) const
{
  std::array<int64_t, 3> index;
  for (size_t axis = 0; axis < index.size(); ++axis)
  {
    const double value = std::floor(observation.point_in_map[axis] / history_voxel_size_);
    // INT64_MAX rounds up when converted to double, so use an exclusive upper bound.
    if (!std::isfinite(value) || value < -std::ldexp(1.0, 63) || value >= std::ldexp(1.0, 63))
      return std::nullopt;
    index[axis] = static_cast<int64_t>(value);
  }
  return ObservationKey{index[0], index[1], index[2], observation.face.idx()};
}

bool TemporalObstacleLayer::fresh(int64_t stamp, int64_t now) const
{
  constexpr int64_t future_tolerance_ns = 1000000;  // 1 ms for clock precision only.
  return stamp > 0 && now >= 0 && now - stamp >= -future_tolerance_ns && now - stamp <= max_age_ns_;
}

uint64_t TemporalObstacleLayer::observeClock(int64_t now)
{
  if (last_clock_ && now < *last_clock_) clock_epoch_.fetch_add(1);
  last_clock_ = now;
  return clock_epoch_.load();
}

bool TemporalObstacleLayer::poseJumped(const PoseState& pose) const
{
  if (!previous_pose_) return false;
  const auto jumped = [this](const Eigen::Isometry3d& before, const Eigen::Isometry3d& after)
  {
    return (before.translation() - after.translation()).norm() > jump_translation_ ||
      Eigen::Quaterniond(before.rotation()).angularDistance(Eigen::Quaterniond(after.rotation())) > jump_rotation_;
  };
  if (jumped(previous_pose_->map_from_reference, pose.map_from_reference)) return true;
  const auto interval = pose.stamp - previous_pose_->stamp;
  return interval >= 0 && interval <= jump_interval_ns_ &&
    jumped(previous_pose_->map_from_sensor, pose.map_from_sensor);
}

bool TemporalObstacleLayer::decayObservation(ObservationRecord& record, int64_t stamp, bool inside) const
{
  // Equivalent to STVL's effective marking timestamp: each accepted clearing
  // cycle ages the voxel, then subtracts a*t^3/6 from its remaining lifetime
  // and (if still alive) from that timestamp. FOV membership is not proof of
  // free space: a missing return may be occluded or between scanning beams.
  if (decay_model_ == -1) return false;
  const double age = record.decay_age + (stamp - record.decay_updated) / 1e9;
  const double lifetime = keep_ns_ / 1e9;
  const double remaining = decay_model_ == 0 ? lifetime - age : lifetime * std::exp(-age);
  const double acceleration = inside && decay_acceleration_ > 0.0 ?
    decay_acceleration_ * age * age * age / 6.0 : 0.0;
  if (remaining - acceleration < 0.0) return true;
  record.decay_age = age + acceleration;
  record.decay_updated = stamp;
  return false;
}

void TemporalObstacleLayer::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  using Timer = mesh_map::LayerTimer;
  const auto start = Timer::Clock::now();
  const int64_t now = node_->now().nanoseconds();
  const uint64_t epoch = observeClock(now);
  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U || !validCloudLayout(*msg))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Skipping obstacle cloud: invalid timestamp or XYZ layout (frame='%s', points=%u x %u)",
      msg->header.frame_id.c_str(), msg->width, msg->height);
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
  const auto ns = stamp.nanoseconds();
  if (!fresh(ns, now))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Keeping obstacle history: cloud age %.3f s is outside [-0.001, %.3f] s; "
      "check upstream processing/queue latency and clock synchronization",
      (now - ns) / 1e9, max_age_ns_ / 1e9);
    return;
  }
  if (epoch == accepted_epoch_ && last_stamp_ && ns <= *last_stamp_) return;

  const auto map = map_ptr_.lock();
  if (!map || !map->mesh()) return;
  Eigen::Isometry3d cloud_from_lidar, map_from_reference;
  try
  {
    const auto sensor_tf = map->tf2Buffer().lookupTransform(
      msg->header.frame_id, sensor_frame_, msg->header.stamp, config_.tf_tolerance);
    const auto reference_tf = map->tf2Buffer().lookupTransform(
      map->mapFrame(), reference_frame_, msg->header.stamp, config_.tf_tolerance);
    if (!transformToEigen(sensor_tf.transform, cloud_from_lidar) ||
        !transformToEigen(reference_tf.transform, map_from_reference)) return;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Keeping obstacle history: required sensor/localization TF unavailable: %s", ex.what());
    return;
  }
  const Eigen::Isometry3d cloud_from_sensor = cloud_from_lidar * Eigen::Translation3d(sensor_offset_);
  const auto tf_ready = Timer::Clock::now();
  ProjectedFrame frame;
  if (!projectObservations(*msg, frame, cloud_from_sensor.translation())) return;
  const auto projected = Timer::Clock::now();

  // Preflight all keys before mutating history. A malformed coordinate cannot turn
  // a frame into an empty clearing observation or partially refresh records.
  std::vector<ObservationKey> keys;
  keys.reserve(frame.observations.size());
  for (const auto& observation : frame.observations)
  {
    const auto key = makeKey(observation);
    if (!key) return;
    keys.push_back(*key);
  }
  const int64_t commit_now = node_->now().nanoseconds();
  if (observeClock(commit_now) != epoch) return;
  if (!fresh(ns, commit_now))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Keeping obstacle history: cloud age %.3f s after TF/projection is outside [-0.001, %.3f] s",
      (commit_now - ns) / 1e9, max_age_ns_ / 1e9);
    return;
  }

  const PoseState pose{map_from_reference, frame.map_from_cloud * cloud_from_sensor, ns};
  const bool reset = epoch != accepted_epoch_ || history_mesh_.lock().get() != map->mesh().get() || poseJumped(pose);
  if (reset) history_.clear();

  geometry_msgs::msg::Point position;
  position.x = pose.map_from_sensor.translation().x();
  position.y = pose.map_from_sensor.translation().y();
  position.z = pose.map_from_sensor.translation().z();
  const Eigen::Quaterniond rotation(pose.map_from_sensor.rotation());
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = rotation.w(); orientation.x = rotation.x();
  orientation.y = rotation.y(); orientation.z = rotation.z();
  frustum_->SetPosition(position);
  frustum_->SetOrientation(orientation);
  frustum_->TransformModel();

  // Mark first: an obstacle seen after a long outage must not be cleared by its old age.
  for (size_t i = 0; i < frame.observations.size(); ++i)
  {
    const auto& observation = frame.observations[i];
    history_.insert_or_assign(keys[i], ObservationRecord{
      observation.point_in_map, observation.vertices, ns, std::nullopt, 0.0, ns});
  }
  std::set<lvr2::VertexHandle> active;
  for (auto it = history_.begin(); it != history_.end();)
  {
    auto& record = it->second;
    bool expired = false;
    if (record.last_seen != ns)
    {
      if (clearing_mode_ == "stvl")
        expired = decayObservation(record, ns, frustum_->IsInside(record.point_in_map));
      else
      {
        expired = ns - record.last_seen >= keep_ns_;
        if (!expired && frustum_->IsInside(record.point_in_map))
        {
          if (!record.visible_missing_since) record.visible_missing_since = ns;
          expired = ns - *record.visible_missing_since >= visible_ns_;
        }
        else if (!expired) record.visible_missing_since.reset();
      }
    }
    if (expired) it = history_.erase(it);
    else
    {
      active.insert(record.vertices.begin(), record.vertices.end());
      ++it;
    }
  }
  accepted_epoch_ = epoch;
  last_stamp_ = ns;
  previous_pose_ = pose;
  history_mesh_ = map->mesh();
  const auto history_done = Timer::Clock::now();
  const size_t changed = commitLethalSet(stamp, std::move(active));
  const auto finished = Timer::Clock::now();
  Timer::recordUpdateDuration(layer_name_, stamp, tf_ready - start,
    history_done - tf_ready, finished - history_done);
  const auto milliseconds = [](auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
  RCLCPP_DEBUG_THROTTLE(get_logger(), *node_->get_clock(), 5000,
    "Temporal obstacles: records=%zu changed=%zu age=%.1fms TF=%.2fms projection=%.2fms history=%.2fms downstream=%.2fms",
    history_.size(), changed, (commit_now - ns) / 1e6, milliseconds(tf_ready - start),
    milliseconds(projected - tf_ready), milliseconds(history_done - projected), milliseconds(finished - history_done));
}
}  // namespace mesh_layers

PLUGINLIB_EXPORT_CLASS(mesh_layers::TemporalObstacleLayer, mesh_map::AbstractLayer)
