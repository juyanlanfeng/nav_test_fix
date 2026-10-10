#include <mesh_layers/obstacle_layer.h>
#include <mesh_map/mesh_map.h>
#include <mesh_map/timer.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iterator>


namespace mesh_layers
{

std::optional<rclcpp::QoS> get_qos_profile_from_string(const std::string& str)
{
  if ("Reliable" == str)
  {
    return rclcpp::QoS(1).reliable();
  }
  else if ("BestEffort" == str)
  {
    return rclcpp::QoS(1).best_effort();
  }

  return std::nullopt;
}

bool ObstacleLayer::initialize()
{
  RCLCPP_DEBUG(get_logger(), "Initializing 'ObstacleLayer' with name '%s'", name().c_str());

  auto map = map_ptr_.lock();

  // Read parameters
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "robot_height";
    desc.description = "The height of the robot in meter. "
    "Obstacles with a larger distance to the surface along the vertical axis are not added to the cost map. "
    "This allows the robot to pass below overhanging obstacles like tables (if it can fit below the overhang). "
    "You should add a safety margin to the robot height!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    config_.robot_height = node_->declare_parameter(desc.name, config_.robot_height, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "max_obstacle_dist";
    desc.description = "The maximum distance to the sensor origin ([0.0, 0.0, 0.0] in the reference frame of the input point cloud!) an obstacle point may have. "
    "Obstacles with larger distances to the (sensor) origin are not processed and added to the map. "
    "Limiting the sensor range reduces the update latency, especially with lidar sensors with high ranges.";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    config_.max_obstacle_dist = node_->declare_parameter(desc.name, config_.max_obstacle_dist, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "topic";
    desc.description = "The ROS topic to subscribe to. The message type must be PointCloud2! "
      "This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    config_.topic = node_->declare_parameter<std::string>(desc.name, desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "qos";
    desc.description = "The QoS settings to use when subscribing to the ROS topic. Options are 'Reliable' and 'BestEffort'. "
      "This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    const std::string qos_str = node_->declare_parameter<std::string>(desc.name, "Reliable", desc);
    if (auto opt = get_qos_profile_from_string(qos_str))
    {
      config_.qos = opt.value();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid 'qos' parameter '%s'! Options are 'Reliable' and 'BestEffort'", qos_str.c_str());
      return false;
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "tf_tolerance";
    desc.description = "The time to wait for transforms in seconds.";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    const double tf_tolerance = node_->declare_parameter(desc.name, config_.tf_tolerance.seconds(), desc);
    if (!std::isfinite(tf_tolerance) || tf_tolerance < 0.0 || tf_tolerance > 60.0)
    {
      RCLCPP_ERROR(get_logger(), "tf_tolerance must be in [0, 60] seconds");
      return false;
    }
    config_.tf_tolerance = rclcpp::Duration::from_seconds(tf_tolerance);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "down_axis";
    desc.description = "The vector to use when projecting obstacle points to the surface. "
      "Defaults to [0.0, 0.0, -1.0]. This parameter is not reconfigurable at runtime!";
    desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    const auto dir = node_->declare_parameter<std::vector<double>>(desc.name, {0.0, 0.0, -1.0}, desc);
    // Check for exactly 3 coords!
    if (3 != dir.size())
    {
      RCLCPP_ERROR(get_logger(), "Invalid parameter value for 'down_axis'! Must be exactly 3 values!");
      return false;
    }
    const Eigen::Vector3d axis(dir[0], dir[1], dir[2]);
    if (!axis.allFinite() || !std::isfinite(axis.norm()) || axis.norm() < 1e-12)
    {
      RCLCPP_ERROR(get_logger(), "down_axis must be a finite nonzero vector");
      return false;
    }
    config_.down_axis = axis.normalized().cast<float>();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = fixed_parameters_;
    desc.name = layer_namespace_ + '.' + "axis_frame";
    desc.description = "The reference frame of the 'down_axis' parameter. Defaults to the value of the robot_frame parameter. "
      "This parameter is not reconfigurable at runtime!";
    desc.type =rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

    // Default to the 'robot_frame' parameter. This should always have a value
    const std::string robot_frame = node_->get_parameter("robot_frame").as_string();
    config_.axis_frame_id = node_->declare_parameter(desc.name, robot_frame, desc);
  }

  // Support reconfiguration of parameters at runtime
  if (config_.topic.empty() || config_.axis_frame_id.empty() ||
      std::isnan(config_.robot_height) || config_.robot_height < 0.0 ||
      std::isnan(config_.max_obstacle_dist) || config_.max_obstacle_dist <= 0.0 ||
      (fixed_parameters_ && (!std::isfinite(config_.robot_height) ||
                             !std::isfinite(config_.max_obstacle_dist))))
  {
    RCLCPP_ERROR(get_logger(), "Invalid obstacle projection parameters");
    return false;
  }
  if (!fixed_parameters_)
  {
    dyn_params_handler_ = node_->add_on_set_parameters_callback(
      std::bind(&ObstacleLayer::reconfigureCallback, this, std::placeholders::_1));
  }

  if (!configureProjection()) return false;

  // Setup a callback group for multithreaded ros callback execution
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  // Initialize ROS Subscriber
  sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_.topic,
    config_.qos,
    std::bind(&ObstacleLayer::processPointCloud, this, std::placeholders::_1),
    options
  );

  return true;
}


namespace
{
bool xyzOffsets(const sensor_msgs::msg::PointCloud2& msg, std::array<uint32_t, 3>& offsets)
{
  if (msg.header.frame_id.empty() || msg.point_step == 0 ||
      uint64_t(msg.row_step) < uint64_t(msg.width) * msg.point_step ||
      uint64_t(msg.data.size()) < uint64_t(msg.row_step) * msg.height ||
      (msg.height == 0 && msg.width != 0))
    return false;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (size_t i = 0; i < names.size(); ++i)
  {
    size_t found = 0;
    for (const auto& field : msg.fields)
    {
      if (field.name != names[i]) continue;
      if (++found != 1 || field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          field.count != 1 || uint64_t(field.offset) + sizeof(float) > msg.point_step)
        return false;
      offsets[i] = field.offset;
    }
    if (found != 1) return false;
  }
  for (size_t i = 0; i < offsets.size(); ++i)
    for (size_t j = i + 1; j < offsets.size(); ++j)
      if (uint64_t(offsets[i]) < uint64_t(offsets[j]) + sizeof(float) &&
          uint64_t(offsets[j]) < uint64_t(offsets[i]) + sizeof(float)) return false;
  return true;
}

float readFloat(const uint8_t* data, bool swap)
{
  uint8_t bytes[sizeof(float)];
  std::memcpy(bytes, data, sizeof(float));
  if (swap) std::reverse(std::begin(bytes), std::end(bytes));
  float result;
  std::memcpy(&result, bytes, sizeof(float));
  return result;
}
}  // namespace

bool ObstacleLayer::validCloudLayout(const sensor_msgs::msg::PointCloud2& msg)
{
  std::array<uint32_t, 3> offsets;
  return xyzOffsets(msg, offsets);
}

bool ObstacleLayer::transformToEigen(
  const geometry_msgs::msg::Transform& tf, Eigen::Isometry3d& output)
{
  const Eigen::Vector3d translation(tf.translation.x, tf.translation.y, tf.translation.z);
  Eigen::Quaterniond rotation(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      !std::isfinite(rotation.norm()) || rotation.norm() < 1e-12)
    return false;
  rotation.normalize();
  output = Eigen::Translation3d(translation) * rotation;
  return true;
}

bool ObstacleLayer::projectObservations(
  const sensor_msgs::msg::PointCloud2& msg, ProjectedFrame& output,
  const Eigen::Vector3d& range_origin_in_cloud)
{
  output.observations.clear();
  std::array<uint32_t, 3> offsets;
  const auto map = map_ptr_.lock();
  if (!map || !map->mesh() || !map->raycaster() ||
      !xyzOffsets(msg, offsets) || !range_origin_in_cloud.allFinite())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Skipping obstacle frame: invalid XYZ layout or unavailable mesh/raycaster");
    return false;
  }

  Eigen::Isometry3d map_from_axis;
  try
  {
    const auto cloud_tf = map->tf2Buffer().lookupTransform(
      map->mapFrame(), msg.header.frame_id, msg.header.stamp, config_.tf_tolerance);
    const auto axis_tf = map->tf2Buffer().lookupTransform(
      map->mapFrame(), config_.axis_frame_id, msg.header.stamp, config_.tf_tolerance);
    if (!transformToEigen(cloud_tf.transform, output.map_from_cloud) ||
        !transformToEigen(axis_tf.transform, map_from_axis)) return false;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 5000,
      "Skipping obstacle frame: %s", ex.what());
    return false;
  }

  // Respect organized cloud row padding and the wire byte order. Non-finite returns
  // are skipped, as in ordinary PointCloud2 processing; malformed layouts fail above.
  const uint16_t endian_probe = 1;
  const bool host_bigendian = *reinterpret_cast<const uint8_t*>(&endian_probe) == 0;
  std::vector<lvr2::Vector3f> origins;
  std::vector<Eigen::Vector3d> points_in_map;
  origins.reserve(size_t(msg.width) * msg.height);
  points_in_map.reserve(size_t(msg.width) * msg.height);
  for (uint32_t row = 0; row < msg.height; ++row)
  {
    for (uint32_t col = 0; col < msg.width; ++col)
    {
      const auto* data = msg.data.data() + size_t(row) * msg.row_step + size_t(col) * msg.point_step;
      const bool swap = msg.is_bigendian != host_bigendian;
      const Eigen::Vector3d point(readFloat(data + offsets[0], swap),
        readFloat(data + offsets[1], swap), readFloat(data + offsets[2], swap));
      if (!point.allFinite() ||
          (point - range_origin_in_cloud).norm() > config_.max_obstacle_dist) continue;
      const Eigen::Vector3d world = output.map_from_cloud * point;
      if (!world.allFinite() || !world.cast<float>().allFinite()) continue;
      origins.emplace_back(world.cast<float>());
      points_in_map.push_back(world);
    }
  }

  // A valid empty observation still required both TFs, but no raycast.
  if (origins.empty()) return true;
  const lvr2::Vector3f direction =
    (map_from_axis.rotation() * config_.down_axis.cast<double>()).cast<float>();
  std::vector<lvr2::Vector3f> directions(origins.size(), direction);
  std::vector<mesh_map::MeshMap::RayCastResult> results(origins.size());
  std::vector<uint8_t> hits(origins.size());
  map->raycaster()->castRays(origins, directions, results, hits);
  output.observations.reserve(origins.size());
  for (size_t i = 0; i < hits.size(); ++i)
  {
    if (!hits[i] || !std::isfinite(results[i].dist) || results[i].dist < 0 ||
        results[i].dist > config_.robot_height) continue;
    const lvr2::FaceHandle face(results[i].face_id);
    output.observations.push_back({points_in_map[i], face, map->mesh()->getVerticesOfFace(face)});
  }
  return true;
}

size_t ObstacleLayer::commitLethalSet(
  const rclcpp::Time& stamp, std::set<lvr2::VertexHandle> active_vertices)
{
  std::set<lvr2::VertexHandle> changed;
  {
    auto lock = writeLock();
    std::set_symmetric_difference(lethals_.begin(), lethals_.end(),
      active_vertices.begin(), active_vertices.end(), std::inserter(changed, changed.end()));
    if (changed.empty()) return 0;
    lvr2::SparseVertexMap<float> costs;
    for (const auto vertex : active_vertices)
      costs.insert(vertex, std::numeric_limits<float>::infinity());
    costs_ = std::move(costs);
    lethals_ = std::move(active_vertices);
  }
  // Release the write lock before downstream layers read our new state.
  notifyChange(stamp, changed);
  return changed.size();
}

void ObstacleLayer::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  const auto start = mesh_map::LayerTimer::Clock::now();
  ProjectedFrame frame;
  if (!projectObservations(*msg, frame)) return;
  std::set<lvr2::VertexHandle> active;
  for (const auto& observation : frame.observations)
    active.insert(observation.vertices.begin(), observation.vertices.end());
  const auto projected = mesh_map::LayerTimer::Clock::now();
  commitLethalSet(rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type()), std::move(active));
  const auto finished = mesh_map::LayerTimer::Clock::now();
  mesh_map::LayerTimer::recordUpdateDuration(layer_name_, msg->header.stamp,
    mesh_map::LayerTimer::Duration::zero(), projected - start, finished - projected);
}


rcl_interfaces::msg::SetParametersResult ObstacleLayer::reconfigureCallback(
  std::vector<rclcpp::Parameter> parameters
)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& param: parameters)
  {
    if (layer_namespace_ + ".robot_height" == param.get_name())
    {
      config_.robot_height = param.as_double();
      RCLCPP_INFO(get_logger(), "Parameter 'robot_height' reconfigured to '%f'", config_.robot_height);
    }
    else if (layer_namespace_ + ".max_obstacle_dist" == param.get_name())
    {
      config_.max_obstacle_dist = param.as_double();
      RCLCPP_INFO(get_logger(), "Parameter 'max_obstacle_dist' reconfigured to '%f'", config_.max_obstacle_dist);
    }
    else if (layer_namespace_ + ".tf_tolerance" == param.get_name())
    {
      config_.tf_tolerance = rclcpp::Duration::from_seconds(param.as_double());
      RCLCPP_INFO(get_logger(), "Parameter 'tf_tolerance' reconfigured to '%f'", config_.tf_tolerance.seconds());
    }
  }

  return res;
}

} // namespace mesh_layers

// Register the layer plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mesh_layers::ObstacleLayer, mesh_map::AbstractLayer)
