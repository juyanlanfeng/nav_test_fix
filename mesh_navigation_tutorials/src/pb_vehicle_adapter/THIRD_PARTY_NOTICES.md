# Third-party notices

This adapter does not modify the upstream PolarBear repositories.  It consumes
the following sources from `third_party/pb2025_sources`, whose original
licenses remain in those directories:

- `pb2025_robot_description` — MIT
- `rmoss_gazebo/rmoss_gz_plugins` — Apache-2.0
- `rmoss_gz_resources` — Apache-2.0
- `sdformat_tools` — Apache-2.0

The generated SDF and URDF describe the upstream PB2025 sentry model.  The
local wrapper removes the projectile-shooter plugin and adds only a Gazebo pose
publisher required for navigation truth localization.
