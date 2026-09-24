# PB2025 upstream lock

The runtime resources and Gazebo plugin are intentionally outside this ROS
workspace so that the existing navigation sources stay isolated from upstream
vehicle code.  They are checked out under
`/home/rainple/nav_test/third_party/pb2025_sources`.

| Source | Revision | Purpose |
| --- | --- | --- |
| `SMBU-PolarBear-Robotics-Team/pb2025_robot_description` | `a0541dddcbfe376f369a6345b532a10923ad9149` | SDF XMacro, lidar and camera models |
| `SMBU-PolarBear-Robotics-Team/rmoss_gazebo` | `7443ff06c345752d0e23172c479d99b27207e2c7` | `MecanumDrive2` and light-bar Gazebo systems |
| `SMBU-PolarBear-Robotics-Team/rmoss_gz_resources` | `b5c759f08844dfda19c79aa870866ace8d4c7b3a` | RM25 chassis and attached model resources |
| `gezp/sdformat_tools` | `47c2d1a37372281e5627d444f7537efc0bba11df` | SDF XMacro expansion and SDF-to-URDF conversion |

`pb2025_robot_description/dependencies.repos` names `gezp/sdformat_tools`;
the same path under the PolarBear organization is not a public repository.
