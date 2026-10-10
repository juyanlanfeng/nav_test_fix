# STVL lidar frustum

Source: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/16cdf3b823e0b14886eb7571cd5f559b97fea19d

Only these files are vendored from `spatio_temporal_voxel_layer/`:

- `include/spatio_temporal_voxel_layer/frustum_models/frustum.hpp`
- `include/spatio_temporal_voxel_layer/frustum_models/three_dimensional_lidar_frustum.hpp`
- `src/frustum_models/three_dimensional_lidar_frustum.cpp`

The existing local copies were moved into this private source directory. Their original
copyright and BSD-style three-clause license headers are retained verbatim. The upstream
package separately declares LGPL v2.1 in
[package.xml](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/blob/16cdf3b823e0b14886eb7571cd5f559b97fea19d/spatio_temporal_voxel_layer/package.xml).
This note records both declarations, rather than assigning this package's license to
the entire upstream project.

Local adaptations:

1. Replace the query point's `openvdb::Vec3d` with `Eigen::Vector3d`; remove OpenVDB
   and unused ROS/visualization includes. There is no STVL, OpenVDB, or Nav2 dependency.
2. Use relative includes, private include guards and the `mesh_layers::stvl` namespace.
3. Remove the two unused `Dot()` overloads, which would otherwise become duplicates.
4. Initialize pose/validity state, reject invalid poses and non-finite query points,
   normalize the quaternion and guard the vertical-axis division.
5. Check three-dimensional spherical range against constructor min/max distances.
   The original horizontal check retains a zero minimum radius.
6. Use `atan2` for the horizontal angle (including the negative X axis) and reserve
   the full-circle shortcut for 360 degrees, rather than upstream's 6.27-radian cutoff.

The upstream vertical slope-offset formula is retained. `TemporalObstacleLayer`
converts explicit elevation limits into this model's vFOV and offset parameters;
the padding is zero. Time/history management is implemented by MeshNav, outside
these geometry files.

`TemporalObstacleLayer` also implements the temporal formulas documented by
`SpatioTemporalVoxelGrid::GetTemporalClearingDuration`, `GetFrustumAcceleration`
and `TemporalClearAndGenerateCostmap` at the same pinned revision (the grid source
is not vendored). In `clearing_mode: stvl`, `obstacle_keep_time` corresponds to
`voxel_decay`, with models 0 (linear), 1 (exponential) and -1 (persistent).
Each accepted missing-observation cycle computes the remaining lifetime and the
in-frustum acceleration `a * age^3 / 6`; surviving records accumulate that
accelerated age. Expiry uses the upstream strict `< 0` boundary.

This is a MeshNav adaptation, not a complete STVL plugin: storage is keyed by
voxel and mesh face, active returns are refreshed before expiration, and decay
is driven only by valid, fresh sensor frames using their timestamps. STVL's
OpenVDB grid, 2D costmap conversion, observation buffers and update timer are not
included. Neither the frustum geometry nor a missing return proves free space
behind an occluding object. The tutorial therefore defaults to zero accelerated
clearing; the legacy fixed missing-return window requires explicit
`clearing_mode: visible_timeout`.
