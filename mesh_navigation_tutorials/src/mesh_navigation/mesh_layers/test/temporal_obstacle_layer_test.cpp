#include <gtest/gtest.h>
#include <mesh_layers/temporal_obstacle_layer.h>
#include <mesh_map/mesh_map.h>
#include "stvl_frustum/three_dimensional_lidar_frustum.hpp"

#include <rcl/time.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <cstdlib>

namespace mesh_layers
{
struct ObstacleLayerTestAccess
{
  static void deliver(ObstacleLayer& layer, const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
  { layer.processPointCloud(msg); }
};
struct TemporalObstacleLayerTestAccess
{
  static size_t count(const TemporalObstacleLayer& layer) { return layer.history_.size(); }
  static std::optional<int64_t> stamp(const TemporalObstacleLayer& layer) { return layer.last_stamp_; }
  static std::optional<std::array<int64_t, 4>> key(
    const TemporalObstacleLayer& layer, const Eigen::Vector3d& point, size_t face)
  {
    const auto key = layer.makeKey({point, lvr2::FaceHandle(face),
      {lvr2::VertexHandle(0), lvr2::VertexHandle(1), lvr2::VertexHandle(2)}});
    if (!key) return std::nullopt;
    return std::array<int64_t, 4>{key->x, key->y, key->z, static_cast<int64_t>(key->face)};
  }
};
}  // namespace mesh_layers

namespace
{
using Access = mesh_layers::TemporalObstacleLayerTestAccess;
using Point = Eigen::Vector3d;
int64_t nanos(double time) { return std::llround(time * 1e9); }

class TemporalObstacleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    char name[] = "/tmp/mesh_temporal_test_XXXXXX";
    const auto path = mkdtemp(name);
    ASSERT_NE(path, nullptr);
    directory_ = path;
    // Real mesh + raycaster: 0.5 m squares split into triangles, including shared vertices.
    std::ofstream mesh(directory_ / "plane.ply");
    constexpr int nx = 25, ny = 9;
    mesh << "ply\nformat ascii 1.0\nelement vertex " << nx * ny
         << "\nproperty float x\nproperty float y\nproperty float z\nelement face "
         << 2 * (nx - 1) * (ny - 1)
         << "\nproperty list uchar int vertex_indices\nend_header\n";
    for (int y = 0; y < ny; ++y)
      for (int x = 0; x < nx; ++x) mesh << -2.0 + 0.5 * x << ' ' << -2.0 + 0.5 * y << " 0\n";
    for (int y = 0; y < ny - 1; ++y)
      for (int x = 0; x < nx - 1; ++x)
      {
        const int a = y * nx + x, b = a + 1, c = a + nx, d = c + 1;
        mesh << "3 " << a << ' ' << b << ' ' << d << "\n3 " << a << ' ' << d << ' ' << c << '\n';
      }
  }

  void init(bool legacy = false, const std::vector<rclcpp::Parameter>& overrides = {})
  {
    std::vector<rclcpp::Parameter> params{
      {"robot_frame", "base_footprint"},
      {"mesh_map.mesh_file", (directory_ / "plane.ply").string()},
      {"mesh_map.mesh_working_file", (directory_ / "plane.h5").string()},
      {"mesh_map.mesh_part", "/"}, {"mesh_map.mesh_working_part", "mesh"},
      {"mesh_map.default_layer", "final"}, {"mesh_map.edge_cost_factor", 1.0},
      {"mesh_map.layers", std::vector<std::string>{"border", "static_inflation", "obstacle", "obstacle_inflation", "final"}},
      {"mesh_map.border.type", "mesh_layers/BorderLayer"},
      {"mesh_map.border.border_cost", 1.0}, {"mesh_map.border.threshold", 0.2},
      {"mesh_map.static_inflation.type", "mesh_layers/InflationLayer"},
      {"mesh_map.static_inflation.inputs", std::vector<std::string>{"border"}},
      {"mesh_map.static_inflation.inflation_radius", 0.15},
      {"mesh_map.obstacle.type", legacy ? "mesh_layers/ObstacleLayer" : "mesh_layers/TemporalObstacleLayer"},
      {"mesh_map.obstacle.clearing_mode", "visible_timeout"},
      {"mesh_map.obstacle.topic", "/test_temporal_obstacles"},
      {"mesh_map.obstacle.axis_frame", "map"},
      {"mesh_map.obstacle.robot_height", 0.5},
      {"mesh_map.obstacle.max_obstacle_dist", 8.0},
      {"mesh_map.obstacle.tf_tolerance", 0.0},
      {"mesh_map.obstacle_inflation.type", "mesh_layers/InflationLayer"},
      {"mesh_map.obstacle_inflation.inputs", std::vector<std::string>{"obstacle"}},
      {"mesh_map.obstacle_inflation.inflation_radius", 0.60},
      {"mesh_map.obstacle_inflation.inscribed_radius", 0.30},
      {"mesh_map.obstacle_inflation.inscribed_value", 0.99},
      {"mesh_map.obstacle_inflation.lethal_value", std::numeric_limits<double>::infinity()},
      {"mesh_map.final.type", "mesh_layers/MaxCombinationLayer"},
      {"mesh_map.final.inputs", std::vector<std::string>{"static_inflation", "obstacle_inflation"}}
    };
    for (const auto& override : overrides)
    {
      params.erase(std::remove_if(params.begin(), params.end(), [&](const auto& p) {
        return p.get_name() == override.get_name(); }), params.end());
      params.push_back(override);
    }
    node_ = std::make_shared<rclcpp::Node>("temporal_test", rclcpp::NodeOptions().parameter_overrides(params));
    node_->declare_parameter("robot_frame", std::string("base_footprint"));
    ASSERT_EQ(rcl_enable_ros_time_override(node_->get_clock()->get_clock_handle()), RCL_RET_OK);
    time(10.0);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    transform("map", "odom");
    transform("odom", "base_footprint");
    transform("base_footprint", "front_mid360");
    map_ = std::make_shared<mesh_map::MeshMap>(*tf_, node_);
    ASSERT_TRUE(map_->readMap());
    layer_ = std::dynamic_pointer_cast<mesh_layers::ObstacleLayer>(map_->layer("obstacle"));
    ASSERT_NE(layer_, nullptr);
    temporal_ = std::dynamic_pointer_cast<mesh_layers::TemporalObstacleLayer>(layer_);
    if (!legacy) ASSERT_NE(temporal_, nullptr);
  }

  void TearDown() override
  {
    std::weak_ptr<rclcpp::Node> old_node = node_;
    temporal_.reset(); layer_.reset(); map_.reset(); tf_.reset(); node_.reset();
    EXPECT_TRUE(old_node.expired());  // pluginlib must run derived destructors.
    if (!directory_.empty()) std::filesystem::remove_all(directory_);
  }

  void time(double seconds)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), nanos(seconds)), RCL_RET_OK);
  }
  void transform(const std::string& parent, const std::string& child,
                 const Point& translation = Point::Zero(),
                 const Eigen::Quaterniond& rotation = Eigen::Quaterniond::Identity())
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent; tf.child_frame_id = child; tf.header.stamp = node_->now();
    tf.transform.translation.x = translation.x(); tf.transform.translation.y = translation.y();
    tf.transform.translation.z = translation.z();
    tf.transform.rotation.w = rotation.w(); tf.transform.rotation.x = rotation.x();
    tf.transform.rotation.y = rotation.y(); tf.transform.rotation.z = rotation.z();
    ASSERT_TRUE(tf_->setTransform(tf, "test", true));
  }
  sensor_msgs::msg::PointCloud2::SharedPtr cloud(double stamp, const std::vector<Point>& points = {})
  {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->header.frame_id = "base_footprint";
    msg->header.stamp = rclcpp::Time(nanos(stamp), RCL_ROS_TIME);
    sensor_msgs::PointCloud2Modifier modifier(*msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
    for (const auto& point : points)
    { *x = point.x(); *y = point.y(); *z = point.z(); ++x; ++y; ++z; }
    return msg;
  }
  void deliver(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
  { mesh_layers::ObstacleLayerTestAccess::deliver(*layer_, msg); }
  void frame(double stamp, const std::vector<Point>& points = {})
  { time(stamp); deliver(cloud(stamp, points)); }
  float cost(const std::shared_ptr<mesh_map::AbstractLayer>& layer, lvr2::VertexHandle vertex)
  {
    const float fallback = layer->defaultValue();
    return layer->costs().get(vertex).value_or(fallback);
  }

  std::filesystem::path directory_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<mesh_map::MeshMap> map_;
  std::shared_ptr<mesh_layers::ObstacleLayer> layer_;
  std::shared_ptr<mesh_layers::TemporalObstacleLayer> temporal_;
};

TEST_F(TemporalObstacleTest, LegacyReplacesEachFrameAndPropagatesRemoval)
{
  init(true);
  frame(10, {{2.1, 0.2, 0.2}});
  const auto old = layer_->lethals();
  ASSERT_EQ(old.size(), 3u);
  for (auto vertex : old) EXPECT_TRUE(std::isinf(map_->vertexCosts()[vertex]));
  frame(10.05);
  EXPECT_TRUE(layer_->lethals().empty());
  for (auto vertex : old) EXPECT_FLOAT_EQ(map_->vertexCosts()[vertex], 0.0f);
}

TEST_F(TemporalObstacleTest, OutsideViewExpiresAtTenSecondsIndependentOfCallbackCount)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{-1.1, 0.2, 0.2}});
  ASSERT_EQ(Access::count(*temporal_), 1u);
  frame(19.999999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  for (auto vertex : layer_->lethals()) EXPECT_TRUE(std::isinf(cost(layer_, vertex)));
  frame(20);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, VisibleMissingWindowStartsAtFirstMissingFrame)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}});
  frame(10.05);
  frame(10.149999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.15);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, StvlRetainsOccludedSideWhileAnotherSideIsObserved)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"}});
  frame(10, {{2.1, 0.2, 0.2}});
  const auto first_side = layer_->lethals();
  // Both sides are geometrically inside the FOV. Changing viewpoint is not
  // evidence that the previously observed side of the box has disappeared.
  for (double stamp : {10.05, 10.2, 11.0, 15.0, 19.99})
  {
    frame(stamp, {{3.1, 0.2, 0.2}});
    EXPECT_EQ(Access::count(*temporal_), 2u);
    for (auto vertex : first_side)
      EXPECT_TRUE(std::isinf(map_->vertexCosts()[vertex]));
  }
  frame(20.01, {{3.1, 0.2, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  for (auto vertex : first_side)
    EXPECT_FALSE(std::isinf(map_->vertexCosts()[vertex]));
}

TEST_F(TemporalObstacleTest, StvlLinearLifetimeUsesUpstreamStrictExpiryBoundary)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"}});
  frame(10, {{2.1, 0.2, 0.2}});
  frame(20);
  EXPECT_EQ(Access::count(*temporal_), 1u);  // remaining == 0 is still active upstream.
  frame(20.000001);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, StvlLinearAccelerationMatchesEffectiveTimestampRecurrence)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"},
               {"mesh_map.obstacle.decay_acceleration", 5.0}});
  frame(10, {{2.1, 0.2, 0.2}});
  double effective_mark = 10.0;
  bool expired = false;
  for (int i = 1; i <= 100 && !expired; ++i)
  {
    const double stamp = 10 + i * 0.05;
    const double age = stamp - effective_mark;
    const double acceleration = 5.0 * age * age * age / 6.0;
    expired = 10.0 - age - acceleration < 0.0;
    effective_mark -= acceleration;
    frame(stamp);
    EXPECT_EQ(Access::count(*temporal_), expired ? 0u : 1u) << "frame " << i;
  }
  EXPECT_TRUE(expired);
}

TEST_F(TemporalObstacleTest, StvlAccelerationDoesNotApplyOutsideFrustum)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"},
               {"mesh_map.obstacle.decay_acceleration", 100.0},
               {"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{-1.1, 0.2, 0.2}});
  frame(19.9);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(20.1);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, StvlExponentialUsesExponentialRemainingLifetime)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"},
               {"mesh_map.obstacle.decay_model", int64_t{1}},
               {"mesh_map.obstacle.decay_acceleration", 4.0 / 9.0}});
  frame(10, {{2.1, 0.2, 0.2}});
  // At age 3: exponential remaining = 10*exp(-3) < acceleration 2.
  // Linear remaining would be 7 and would retain this observation.
  frame(13);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, StvlPersistentSurvivesMissingReturnsButResetsOnClockJump)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"},
               {"mesh_map.obstacle.decay_model", int64_t{-1}},
               {"mesh_map.obstacle.decay_acceleration", 5.0}});
  frame(10, {{2.1, 0.2, 0.2}});
  frame(1000);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(2);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, StvlRefreshResetsAcceleratedAgeAndOutageDoesNotClear)
{
  init(false, {{"mesh_map.obstacle.clearing_mode", "stvl"},
               {"mesh_map.obstacle.decay_acceleration", 5.0}});
  frame(10, {{2.1, 0.2, 0.2}});
  frame(11);  // Accumulates accelerated age.
  frame(11.05, {{2.1, 0.2, 0.2}});
  frame(11.10);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  time(30);
  deliver(cloud(12));  // Rejected stale frame must not expire history.
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(30, {{2.1, 0.2, 0.2}});  // Mark before decay on recovery.
  frame(30.05);
  EXPECT_EQ(Access::count(*temporal_), 1u);
}

TEST_F(TemporalObstacleTest, DetectionCancelsMissingWindowAndRefreshesLifetime)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}});
  frame(10.05);
  frame(10.10, {{2.1, 0.2, 0.2}});
  frame(10.15);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.249999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.25);
  EXPECT_EQ(Access::count(*temporal_), 0u);
}

TEST_F(TemporalObstacleTest, LeavingViewCancelsWindowAndReentryStartsNewWindow)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{2.1, 0.2, 0.2}});
  frame(10.05);
  // Change only the sensor orientation; increase the frame interval to avoid jump detection.
  transform("base_footprint", "front_mid360", Point::Zero(), Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Point::UnitZ())));
  frame(10.30);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  transform("base_footprint", "front_mid360");
  frame(10.60);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.699999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.70);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, OutageHoldsCostsAndRecoveryMarksBeforeExpiring)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}, {3.1, 0.2, 0.2}});
  const auto old = layer_->lethals();
  time(25);
  EXPECT_EQ(layer_->lethals(), old);  // No timer-driven clearing.
  frame(25, {{2.1, 0.2, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  EXPECT_EQ(layer_->lethals().size(), 3u);
}

TEST_F(TemporalObstacleTest, RejectedFramesDoNotStartClearing)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}});
  time(11);
  deliver(cloud(10.5));  // Stale.
  deliver(cloud(11.1));  // Future.
  deliver(cloud(0));
  auto malformed = cloud(11);
  malformed->fields.clear(); deliver(malformed);
  malformed = cloud(11, {{2.1, 0.2, 0.2}});
  malformed->data.clear(); deliver(malformed);
  auto bad_tf = cloud(11); bad_tf->header.frame_id = "missing_frame"; deliver(bad_tf);
  EXPECT_EQ(Access::stamp(*temporal_), nanos(10));
  frame(11.05);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(11.149999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(11.15);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, DuplicateAndOutOfOrderFramesCannotRefreshOrClear)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}});
  frame(10.05);
  time(10.10);
  deliver(cloud(10.05, {{2.1, 0.2, 0.2}}));
  deliver(cloud(10.04, {{2.1, 0.2, 0.2}}));
  EXPECT_EQ(Access::stamp(*temporal_), nanos(10.05));
  frame(10.15);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, SensorAndReferenceTfFailuresPreserveHistory)
{
  init();
  frame(10, {{2.1, 0.2, 0.2}});
  // Static TF caches survive Buffer::clear(); disconnect the required frames explicitly.
  transform("map", "base_footprint");
  transform("disconnected_sensor", "front_mid360");
  transform("disconnected_reference", "odom");
  frame(10.05); // Sensor and reference missing.
  transform("base_footprint", "front_mid360");
  frame(10.10); // Reference still missing.
  EXPECT_EQ(Access::stamp(*temporal_), nanos(10));
  EXPECT_EQ(Access::count(*temporal_), 1u);
  transform("map", "odom");
  frame(10.15);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.25);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, SharedVerticesRemainUntilAllSupportingRecordsExpire)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  // Nearby points on opposite sides of the same square's diagonal share two vertices.
  frame(10, {{-1.30, 0.10, 0.2}, {-1.40, 0.20, 0.2}});
  ASSERT_EQ(Access::count(*temporal_), 2u);
  ASSERT_EQ(layer_->lethals().size(), 4u);
  frame(15, {{-1.40, 0.20, 0.2}});
  frame(20);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  EXPECT_EQ(layer_->lethals().size(), 3u);
  for (auto vertex : layer_->lethals()) EXPECT_TRUE(std::isinf(map_->vertexCosts()[vertex]));
  frame(25);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, KeysUseFloorFaceAndCheckedIntegerRange)
{
  init();
  auto negative = Access::key(*temporal_, {-0.001, -0.051, 0.049}, 1);
  ASSERT_TRUE(negative);
  EXPECT_EQ((*negative), (std::array<int64_t, 4>{-1, -2, 0, 1}));
  EXPECT_NE(negative, Access::key(*temporal_, {-0.001, -0.051, 0.049}, 2));
  EXPECT_FALSE(Access::key(*temporal_, {1e30, 0, 0}, 1));
  EXPECT_FALSE(Access::key(*temporal_, {NAN, 0, 0}, 1));
  // Both points are inside one 5 cm voxel, but project onto different faces.
  frame(10, {{2.01, 0.02, 0.2}, {2.02, 0.01, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 2u);
}

TEST_F(TemporalObstacleTest, PointHeightIsPreservedForVisibility)
{
  init(false, {{"mesh_map.obstacle.vertical_fov_min_deg", 5.0}});
  // Ground vertices are below the FOV, while the original return is above 5 degrees.
  frame(10, {{1.1, 0.2, 0.3}});
  ASSERT_EQ(Access::count(*temporal_), 1u);
  frame(10.05); frame(10.15);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, MarkingUsesActualSensorOriginAndSphericalRange)
{
  init();
  transform("base_footprint", "front_mid360", {1.0, 0.0, 0.0});
  frame(10, {{8.9, 0.1, 0.2}}); // >8 m from base, <8 m from lidar.
  ASSERT_EQ(Access::count(*temporal_), 1u);
  frame(10.05, {{9.1, 0.1, 0.2}}); // Outside range; cannot create another history.
  EXPECT_EQ(Access::count(*temporal_), 1u);
}

TEST_F(TemporalObstacleTest, SensorOffsetRotatesWithLink)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0},
               {"mesh_map.obstacle.vertical_fov_min_deg", -10.0},
               {"mesh_map.obstacle.vertical_fov_max_deg", 10.0}});
  const auto rotation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Point::UnitY()));
  transform("base_footprint", "front_mid360", {2.1, 0.2, 0.5}, rotation);
  // Offset is +0.03 along link Z -> +0.03 along base X. Local +X points down.
  frame(10, {{2.13, 0.2, 0.2}});
  ASSERT_EQ(Access::count(*temporal_), 1u);
  frame(10.05); frame(10.15);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, ReferenceJumpClearsOldHistoryBeforeMarkingNewFrame)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{-1.1, 0.2, 0.2}});
  transform("map", "odom", {1.0, 0, 0});
  frame(11, {{2.1, 0.2, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  EXPECT_EQ(layer_->lethals().size(), 3u);
}

TEST_F(TemporalObstacleTest, ShortSensorJumpResetsButLongNormalMotionDoesNot)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{-1.1, 0.2, 0.2}});
  transform("odom", "base_footprint", {1.0, 0, 0});
  frame(10.05, {{2.1, 0.2, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  transform("odom", "base_footprint", {5.0, 0, 0});
  frame(11);
  EXPECT_EQ(Access::count(*temporal_), 1u);
}

TEST_F(TemporalObstacleTest, ClockPauseAndRollbackUseNewEpochOnlyAfterValidFrame)
{
  init(false, {{"mesh_map.obstacle.horizontal_fov_deg", 90.0}});
  frame(10, {{-1.1, 0.2, 0.2}});
  for (int i = 0; i < 5; ++i) deliver(cloud(10));
  EXPECT_EQ(Access::count(*temporal_), 1u);
  time(2.0);
  auto bad = cloud(2); bad->fields.clear(); deliver(bad);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(2.05, {{2.1, 0.2, 0.2}});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  EXPECT_EQ(Access::stamp(*temporal_), nanos(2.05));
  // Roll back and catch up without any observations: the clock callback still detects it.
  time(1); time(3);
  deliver(cloud(3));
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, NewParametersAreReadOnlyAndLegacyRemainsReconfigurable)
{
  init();
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
      {"mesh_map.obstacle.robot_height", 0.8}, {"mesh_map.obstacle.max_obstacle_dist", 5.0},
      {"mesh_map.obstacle.visible_keep_time", 0.2}, {"mesh_map.obstacle.horizontal_fov_deg", 90.0},
      {"mesh_map.obstacle.clearing_mode", "stvl"}, {"mesh_map.obstacle.decay_model", int64_t{1}},
      {"mesh_map.obstacle.decay_acceleration", 5.0},
      {"mesh_map.obstacle.sensor_frame", "other_lidar"}})
    EXPECT_FALSE(node_->set_parameter(parameter).successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("mesh_map.obstacle.robot_height").as_double(), 0.5);
}

TEST_F(TemporalObstacleTest, LegacyParametersCanStillBeChanged)
{
  init(true);
  EXPECT_TRUE(node_->set_parameter({"mesh_map.obstacle.robot_height", 0.1}).successful);
  frame(10, {{2.1, 0.2, 0.2}});
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, YamlOverridesTakeEffectAtInitialization)
{
  init(false, {{"mesh_map.obstacle.visible_keep_time", 0.20}});
  frame(10, {{2.1, 0.2, 0.2}});
  frame(10.05); frame(10.15);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.25);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, InvalidConfigurationIsRejectedBeforeSubscribing)
{
  init(false, {{"mesh_map.bad_time.visible_keep_time", -0.1},
               {"mesh_map.bad_fov.vertical_fov_max_deg", 90.0},
               {"mesh_map.bad_order.obstacle_keep_time", 0.05},
               {"mesh_map.bad_order.clearing_mode", "visible_timeout"},
               {"mesh_map.bad_mode.clearing_mode", "unknown"},
               {"mesh_map.bad_model.decay_model", int64_t{2}},
               {"mesh_map.bad_acceleration.decay_acceleration", -1.0},
               {"mesh_map.bad_size.history_voxel_size", 0.0},
               {"mesh_map.bad_offset.sensor_offset_xyz", std::vector<double>{0.0, 0.0}}});
  for (const auto& name : {"bad_time", "bad_fov", "bad_order", "bad_size", "bad_offset",
                          "bad_mode", "bad_model", "bad_acceleration"})
  {
    auto probe = std::make_shared<mesh_layers::TemporalObstacleLayer>();
    EXPECT_FALSE(probe->mesh_map::AbstractLayer::initialize(name,
      [](const auto&, const auto&, const auto&) {}, map_, node_));
  }
}

TEST_F(TemporalObstacleTest, SubscriptionDispatchesToTemporalCallback)
{
  init();
  auto publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/test_temporal_obstacles", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  const auto publish_and_wait = [&](double stamp, const std::vector<Point>& points) {
    time(stamp);
    publisher->publish(*cloud(stamp, points));
    for (int i = 0; i < 100 && Access::stamp(*temporal_) != nanos(stamp); ++i)
    { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    ASSERT_EQ(Access::stamp(*temporal_), nanos(stamp));
  };
  publish_and_wait(10, {{2.1, 0.2, 0.2}});
  ASSERT_EQ(Access::count(*temporal_), 1u);
  publish_and_wait(10.05, {});
  EXPECT_EQ(Access::count(*temporal_), 1u);
  publish_and_wait(10.15, {});
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, FrameThatBecomesStaleWhileWaitingForTfDoesNotClear)
{
  init(false, {{"mesh_map.obstacle.tf_tolerance", 1.0}});
  tf_->setUsingDedicatedThread(true);
  frame(10, {{2.1, 0.2, 0.2}});
  time(10.05);
  auto delayed = cloud(10.05);
  delayed->header.frame_id = "late_cloud";
  std::thread transform_thread([&]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    time(10.45);
    transform("base_footprint", "late_cloud");
  });
  deliver(delayed);
  transform_thread.join();
  EXPECT_EQ(Access::stamp(*temporal_), nanos(10));
  EXPECT_EQ(Access::count(*temporal_), 1u);
  // A fresh frame starts the missing window now, without counting the rejected frame.
  frame(10.50); frame(10.599999);
  EXPECT_EQ(Access::count(*temporal_), 1u);
  frame(10.60);
  EXPECT_TRUE(layer_->lethals().empty());
}

TEST_F(TemporalObstacleTest, RemovalPropagatesThroughInflationAndFinalWithoutClearingStaticCosts)
{
  init();
  const auto static_vertex = *map_->layer("border")->lethals().begin();
  const float static_cost = map_->vertexCosts()[static_vertex];
  frame(10, {{2.1, 0.2, 0.2}});
  const auto marked = layer_->lethals();
  ASSERT_EQ(marked.size(), 3u);
  auto inflation = map_->layer("obstacle_inflation");
  size_t finite_inflated = 0;
  for (auto vertex : map_->mesh()->vertices())
    if (cost(inflation, vertex) > 0 && std::isfinite(cost(inflation, vertex))) ++finite_inflated;
  EXPECT_GT(finite_inflated, 0u);
  frame(10.05); frame(10.15);
  for (auto vertex : marked)
  {
    EXPECT_FLOAT_EQ(cost(inflation, vertex), 0.0f);
    EXPECT_FLOAT_EQ(map_->vertexCosts()[vertex], 0.0f);
  }
  EXPECT_EQ(map_->vertexCosts()[static_vertex], static_cost);
}

TEST_F(TemporalObstacleTest, PureRefreshDoesNotNotifyDownstream)
{
  init();
  size_t obstacle_updates = 0, inflation_updates = 0, final_updates = 0;
  auto subscription = node_->create_subscription<mesh_msgs::msg::MeshVertexCostsSparseStamped>(
    "~/vertex_costs/updates", rclcpp::QoS(10).transient_local(),
    [&](const mesh_msgs::msg::MeshVertexCostsSparseStamped& msg) {
      if (msg.type == "obstacle") ++obstacle_updates;
      if (msg.type == "obstacle_inflation") ++inflation_updates;
      if (msg.type == "final") ++final_updates;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  const auto drain = [&]() {
    for (int i = 0; i < 20; ++i) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
  };
  drain();
  frame(10, {{2.1, 0.2, 0.2}}); drain();
  ASSERT_EQ(obstacle_updates, 1u); ASSERT_EQ(inflation_updates, 1u); ASSERT_EQ(final_updates, 1u);
  frame(10.05, {{2.1, 0.2, 0.2}}); drain();
  EXPECT_EQ(obstacle_updates, 1u); EXPECT_EQ(inflation_updates, 1u); EXPECT_EQ(final_updates, 1u);
  frame(10.10); frame(10.20); drain();
  EXPECT_EQ(obstacle_updates, 2u); EXPECT_EQ(inflation_updates, 2u); EXPECT_EQ(final_updates, 2u);
}

TEST_F(TemporalObstacleTest, RowPaddingAndBigEndianPointsProjectCorrectly)
{
  init(true);
  auto msg = cloud(10, {{2.1, 0.2, 0.2}, {3.1, 0.2, 0.2}});
  const size_t step = msg->point_step;
  auto original = msg->data;
  msg->width = 1; msg->height = 2; msg->row_step = step + 8;
  msg->data.assign(msg->row_step * 2, 0);
  for (size_t row = 0; row < 2; ++row)
  {
    std::copy_n(original.begin() + row * step, step, msg->data.begin() + row * msg->row_step);
    for (const auto& field : msg->fields)
      std::reverse(msg->data.begin() + row * msg->row_step + field.offset,
                   msg->data.begin() + row * msg->row_step + field.offset + sizeof(float));
  }
  msg->is_bigendian = true;
  deliver(msg);
  EXPECT_EQ(layer_->lethals().size(), 6u);
}

mesh_layers::stvl::ThreeDimensionalLidarFrustum model(double min_range = 0.1, double max_range = 8.0,
                                                    double horizontal = 360.0)
{
  const double lower = std::tan(-7.0 * M_PI / 180), upper = std::tan(52.0 * M_PI / 180);
  mesh_layers::stvl::ThreeDimensionalLidarFrustum result(
    2 * std::atan((upper - lower) / 2), std::atan((upper + lower) / 2), 0,
    horizontal * M_PI / 180, min_range, max_range);
  geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Quaternion rotation; rotation.w = 1;
  result.SetPosition(origin); result.SetOrientation(rotation); result.TransformModel();
  return result;
}

TEST(StvlFrustum, ElevationBoundariesFullCircleAndInvalidPoints)
{
  auto frustum = model();
  const auto elevation = [](double angle) { return Point(2, 0, 2 * std::tan(angle * M_PI / 180)); };
  EXPECT_FALSE(frustum.IsInside(elevation(-7.001)));
  EXPECT_TRUE(frustum.IsInside(elevation(-6.999)));
  EXPECT_TRUE(frustum.IsInside(elevation(51.999)));
  EXPECT_FALSE(frustum.IsInside(elevation(52.001)));
  EXPECT_TRUE(frustum.IsInside({-2, 0, 0.2}));
  EXPECT_FALSE(frustum.IsInside({0, 0, 0}));
  EXPECT_FALSE(frustum.IsInside({0, 0, 1}));
  EXPECT_FALSE(frustum.IsInside({NAN, 1, 0}));
  EXPECT_FALSE(frustum.IsInside({INFINITY, 1, 0}));
}

TEST(StvlFrustum, SphericalRangeAndPartialHorizontalField)
{
  auto frustum = model();
  EXPECT_TRUE(frustum.IsInside({8.0, 0, 0}));
  EXPECT_FALSE(frustum.IsInside({8.0, 0, 0.1}));
  EXPECT_FALSE(frustum.IsInside({7.0, 0, 4.0}));
  EXPECT_FALSE(frustum.IsInside({0.099, 0, 0}));
  EXPECT_TRUE(frustum.IsInside({0.1, 0, 0}));
  auto sector = model(0.1, 8, 90);
  EXPECT_TRUE(sector.IsInside({2, 1.99, 0}));
  EXPECT_FALSE(sector.IsInside({2, 2.01, 0}));
  EXPECT_FALSE(sector.IsInside({-2, 0, 0}));
  auto almost_full = model(0.1, 8, 359.9);
  EXPECT_FALSE(almost_full.IsInside({-2, 0, 0}));
}

TEST(StvlFrustum, TranslationFullRotationAndPoseValidity)
{
  auto frustum = model(0.1, 8, 90);
  const Point translation(1, 2, 3);
  const Eigen::Quaterniond rotation = Eigen::AngleAxisd(0.8, Point::UnitZ()) *
    Eigen::AngleAxisd(0.4, Point::UnitY()) * Eigen::AngleAxisd(-0.3, Point::UnitX());
  geometry_msgs::msg::Point position;
  position.x = 1; position.y = 2; position.z = 3;
  geometry_msgs::msg::Quaternion quat;
  quat.w = rotation.w(); quat.x = rotation.x(); quat.y = rotation.y(); quat.z = rotation.z();
  frustum.SetPosition(position); frustum.SetOrientation(quat); frustum.TransformModel();
  EXPECT_TRUE(frustum.IsInside(translation + rotation * Point(2, 0, 0.2)));
  EXPECT_FALSE(frustum.IsInside(translation + rotation * Point(-2, 0, 0.2)));
  quat.w = quat.x = quat.y = quat.z = 0;
  frustum.SetOrientation(quat); frustum.TransformModel();
  EXPECT_FALSE(frustum.IsInside({2, 0, 0.2}));
}
}  // namespace
