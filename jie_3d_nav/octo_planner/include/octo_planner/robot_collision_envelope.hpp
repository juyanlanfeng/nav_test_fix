#ifndef OCTO_PLANNER__ROBOT_COLLISION_ENVELOPE_HPP_
#define OCTO_PLANNER__ROBOT_COLLISION_ENVELOPE_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

namespace octo_planner
{

struct RobotCollisionEnvelope
{
  double radius_xy;
  // Physical height from the supporting ground cell to the robot top.
  // This is deliberately not measured from the candidate free-cell centre.
  double height_from_support;
  double legacy_radius;
  bool use_legacy_hemisphere;
};

struct GridOffset
{
  int x;
  int y;
  int z;
};

// Return every candidate-cell offset that can be supported by one occupied
// fine-resolution voxel.  This is the inverse of groundSupportDepth(): if the
// support lookup probes candidate + (dx, dy, -depth), then the candidate lies
// at support + (-dx, -dy, depth).  The XY range is symmetric, so the generated
// set is simply [-radius, radius]^2 for each accepted depth.
inline std::vector<GridOffset> groundSupportCandidateOffsets(
  bool strict_direct_ground_support, int support_xy_radius_cells,
  int support_depth_cells)
{
  if (strict_direct_ground_support) {
    return {{0, 0, 1}};
  }
  if (support_xy_radius_cells < 0) {
    return {};
  }

  std::vector<GridOffset> offsets;
  const int depth_count = std::max(1, support_depth_cells);
  const int diameter = support_xy_radius_cells * 2 + 1;
  offsets.reserve(static_cast<std::size_t>(diameter * diameter * depth_count));
  for (int depth = 1; depth <= depth_count; ++depth) {
    for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
      for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
        offsets.push_back(GridOffset{dx, dy, depth});
      }
    }
  }
  return offsets;
}

inline RobotCollisionEnvelope resolveRobotCollisionEnvelope(
  double robot_radius, double robot_radius_xy, double robot_height)
{
  const double legacy_radius =
    std::isfinite(robot_radius) ? std::max(0.0, robot_radius) : 0.0;
  const bool has_radius_xy = std::isfinite(robot_radius_xy) && robot_radius_xy > 0.0;
  const bool has_height = std::isfinite(robot_height) && robot_height > 0.0;

  return RobotCollisionEnvelope{
    has_radius_xy ? robot_radius_xy : legacy_radius,
    has_height ? robot_height : legacy_radius,
    legacy_radius,
    !has_radius_xy && !has_height};
}

inline bool collisionEnvelopeContainsOffset(
  const RobotCollisionEnvelope & envelope, int dx, int dy, int dz, double resolution,
  int support_depth_cells = 1)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return false;
  }

  const double x = static_cast<double>(dx) * resolution;
  const double y = static_cast<double>(dy) * resolution;
  const double z = static_cast<double>(dz) * resolution;
  const double horizontal_distance_sq = x * x + y * y;
  constexpr double epsilon = 1.0e-12;

  if (horizontal_distance_sq > envelope.radius_xy * envelope.radius_xy + epsilon) {
    return false;
  }
  if (envelope.use_legacy_hemisphere) {
    if (dz < 0) {
      return false;
    }
    return horizontal_distance_sq + z * z <=
           envelope.legacy_radius * envelope.legacy_radius + epsilon;
  }

  // Candidate cells sit above their supporting occupied cell.  For a direct
  // support (depth=1), candidate-relative dz=0 is already one resolution
  // above support.  Measuring robot_height from the candidate would silently
  // add that resolution to the configured physical height.  A deeper support
  // may also put valid body cells below the candidate, hence dz can be
  // negative in anisotropic mode.
  const int depth = std::max(0, support_depth_cells);
  const int height_cell_from_support = dz + depth;
  const int first_body_cell = depth > 0 ? 1 : 0;
  if (height_cell_from_support < first_body_cell) {
    return false;
  }
  return static_cast<double>(height_cell_from_support) * resolution <=
         envelope.height_from_support + epsilon;
}

template<typename IsOccupied, typename IsPreblocked>
inline bool verticalColumnHasPreblockedGap(
  int candidate_z, int minimum_grid_z, IsOccupied is_occupied, IsPreblocked is_preblocked)
{
  for (int z = candidate_z - 1; z >= minimum_grid_z; --z) {
    if (is_occupied(z)) {
      return false;
    }
    if (is_preblocked(z)) {
      return true;
    }
  }
  return false;
}

}  // namespace octo_planner

#endif  // OCTO_PLANNER__ROBOT_COLLISION_ENVELOPE_HPP_
