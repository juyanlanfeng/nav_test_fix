#ifndef MESH_LAYERS__TEMPORAL_OBSTACLE_LAYER_H
#define MESH_LAYERS__TEMPORAL_OBSTACLE_LAYER_H

#include <mesh_layers/obstacle_layer.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace mesh_layers
{
namespace stvl { class ThreeDimensionalLidarFrustum; }

/** Retains projected obstacle observations and expires them on valid sensor frames.
 *  All history mutations run in the base subscription's mutually exclusive group.
 *  The ROS clock jump callback only advances an atomic epoch; it never edits history.
 */
class TemporalObstacleLayer : public ObstacleLayer
{
public:
  TemporalObstacleLayer();
  ~TemporalObstacleLayer() override;

protected:
  bool initialize() override;
  bool configureProjection() override;
  void processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) override;

private:
  friend struct TemporalObstacleLayerTestAccess;

  struct ObservationKey
  {
    int64_t x, y, z;
    size_t face;
    bool operator==(const ObservationKey& other) const;
  };
  struct KeyHash { size_t operator()(const ObservationKey& key) const; };
  struct ObservationRecord
  {
    Eigen::Vector3d point_in_map;
    std::array<lvr2::VertexHandle, 3> vertices;
    int64_t last_seen;
    std::optional<int64_t> visible_missing_since;
    double decay_age = 0.0;
    int64_t decay_updated = 0;
  };
  struct PoseState
  {
    Eigen::Isometry3d map_from_reference;
    Eigen::Isometry3d map_from_sensor;
    int64_t stamp;
  };

  std::optional<ObservationKey> makeKey(const ProjectedObservation& observation) const;
  bool fresh(int64_t stamp, int64_t now) const;
  bool poseJumped(const PoseState& pose) const;
  uint64_t observeClock(int64_t now);
  bool decayObservation(ObservationRecord& record, int64_t stamp, bool inside) const;

  std::string clearing_mode_ = "stvl";
  int64_t decay_model_ = 0;
  double decay_acceleration_ = 0.0;
  double history_voxel_size_ = 0.05;
  int64_t keep_ns_ = 10000000000LL;
  int64_t visible_ns_ = 100000000LL;
  int64_t max_age_ns_ = 300000000LL;
  std::string sensor_frame_;
  Eigen::Vector3d sensor_offset_ = Eigen::Vector3d::Zero();
  double min_sensor_range_ = 0.10;
  double horizontal_fov_deg_ = 360.0;
  double vertical_min_deg_ = -7.0;
  double vertical_max_deg_ = 52.0;
  std::string reference_frame_;
  double jump_translation_ = 0.75;
  double jump_rotation_ = M_PI / 4.0;
  int64_t jump_interval_ns_ = 200000000LL;

  std::unique_ptr<stvl::ThreeDimensionalLidarFrustum> frustum_;
  std::unordered_map<ObservationKey, ObservationRecord, KeyHash> history_;
  std::optional<PoseState> previous_pose_;
  std::optional<int64_t> last_stamp_;
  std::optional<int64_t> last_clock_;
  std::weak_ptr<const void> history_mesh_;
  std::atomic<uint64_t> clock_epoch_{0};
  uint64_t accepted_epoch_ = 0;
  rclcpp::JumpHandler::SharedPtr clock_jump_handler_;
};
}  // namespace mesh_layers
#endif
