# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

SCAN-Planner is a spatial-collision-aware local planner for quadruped (Unitree Go2) navigation, ported to native ROS 2 Humble (Ubuntu 22.04, C++17, colcon). This workspace root IS the colcon workspace. It is a ROS 2 port of [wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) — the port is community-maintained, not the original authors' release. The README is in Chinese.

## Build and test

```bash
# Dependencies (once)
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libarmadillo-dev libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Build (CPU sensing backend is default)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# OpenGL rendering backend (optional)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_GPU=ON
source install/setup.bash
```

Tests are launch tests (`launch_testing_ament_cmake`) in `src/planner/plan_manage/test/` plus a gtest in `src/planner/bspline_opt/test/`:

```bash
colcon test --packages-select scan_planner
colcon test-result --verbose
```

To build/test a single package: `colcon build --packages-select <pkg>`. After editing a C++ header, rebuild before running launch tests — launch tests launch the installed binaries.

## Run

Simulation (deterministic, no physics — mock map + point-cloud renderer + kinematic robot):

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py is_real_world:=false navi_mode:=1 sensor_type:=lidar controller_mode:=closed_loop
```

Second terminal: `ros2 launch scan_planner rviz.launch.py`.

Full physics sim (Gazebo Fortress Go2, optional): `ros2 launch go2_description go2_sim.launch.py` (or `go2_rviz.launch.py` to view the model only).

Waypoint capture tool (native rclpy): `ros2 run scan_planner keypoint_recorder.py --odom /LIO/odom_vehicle --output keypoints.yaml` — output is a ROS 2 parameter YAML with a flat `fsm.waypoints: [x0,y0,z0,...]` array, passed via `keypoints_file:=` when `navi_mode:=2`.

## Architecture

### Workspace layout

- `src/planner/` — the planning stack, layered as libraries:
  - `plan_env` — `GridMap`: voxel-hashed occupancy grid built from lidar/depth via raycasting (log-odds, `raycast.cpp`). Parameters for sensor intrinsics/extrinsics, inflation (double-cylinder self model), sliding map. Core of the "spatial collision awareness".
  - `path_searching` — `DynAStar` (kinodynamic path search).
  - `bspline_opt` — `UniformBspline` + `BsplineOptimizer` (LBFGS gradient descent), `gradient_descent_optimizer`.
  - `traj_utils` — `planning_visualization`, `polynomial_traj`.
  - `plan_manage` — the application package: nodes, controllers, launch, config, custom-message consumers.
  - `scan_planner_msgs` — custom msgs: `Bspline`, `DataDisp`.
- `src/simulator/` — deterministic simulation:
  - `mockamap` (procedural maps) / `map_generator` (publishes a PCD map as `global_cloud`).
  - `local_sensing` — renders the global map into per-frame sensor data: `pcl_render_node` (CPU, default) or `opengl_render_node` (built only with `-DUSE_GPU=ON`); embeds `ikd-Tree` and `FOV_Checker`.
  - `Utils/go2_description` — Go2 URDF/xacro + Gazebo Fortress physics (12-joint `joint_trajectory_controller`, `/joint_states`, IMU, foot contacts, `/clock`).
  - `Utils/odom_visualization`, `Utils/waypoint_generator`, `Utils/pose_utils`.

### Planning pipeline and data flow

`scan_planner_node` creates a plain `rclcpp::Node` and hands it to `SCANReplanFSM::init(node)` — the FSM/planner are plain classes, not `rclcpp::Node` subclasses; all ROS plumbing lives in the FSM (`scan_replan_fsm.cpp`).

1. FSM state machine (INIT → WAIT_TARGET → GEN_NEW_TRAJ → REPLAN_TRAJ → EXEC_TRAJ → EMERGENCY_STOP) driven by two wall timers: `exec_timer_` (10 ms) triggers replanning when deviation/collision thresholds are exceeded; `safety_timer_` (50 ms) runs collision checks.
2. `SCANPlannerManager::reboundReplan` (planner_manager.cpp): front-end path search (DynAStar) + back-end B-spline optimization (LBFGS), reparametrization, dynamic-feasibility checks.
3. Result published as `planning/bspline` (`scan_planner_msgs/msg/Bspline`); FSM freezes the robot via `planning/go2_execution_frozen` (published by the controller, subscribed back by the FSM) when a plan fails, then `EMERGENCY_STOP`.
4. In closed loop, the controller tracks the B-spline and publishes `cmd_vel`; `go2_kinematic_sim` integrates `cmd_vel` into `/quad_0/body_pose`, which closes the loop back into the grid map via local_sensing.

### Simulation vs real robot

The same launch file serves both. `is_real_world:=true` remaps planner inputs to external driver topics — `/LIO/odom_vehicle`, `/LIO/odom_imu`, `/LIO/clouds_lidar`, `/camera/aligned_depth_to_color/image_raw` — and sets `grid_map.cloud_is_world:=false`, `grid_map.need_extrinsic:=true`, plus camera intrinsics overrides. In simulation, `/quad_0/*` topics are used, cloud is in the world frame, and the simulator stack is started automatically.

Controller modes: `open_loop` (publishes the raw B-spline as a path/velocity reference, no kinematic sim) vs `closed_loop` (tracking controller + `go2_kinematic_sim` in sim).

Navi modes: `navi_mode:=1` RViz 2D-goal (`/move_base_simple/goal`), `navi_mode:=2` preset `fsm.waypoints` from `keypoints_file`, `navi_mode:=3` follow `/initial_path` with local obstacle avoidance.

## Configuration and topics

- YAML configs: `src/planner/plan_manage/config/{planner,controllers,simulator}.yaml`. Parameter names use dot-separated ROS 2 style (`grid_map.resolution`, `fsm.navi_mode`).
- The planner's own topics are relative and remapped from `run.launch.py` — keep remappings in sync when adding topics. Key topics: `body_pose` (nav_msgs/Odometry), `cloud`/`depth` (sensor input), `planning/bspline`, `planning/data_display`, `planning/go2_execution_frozen`, `initial_path`, `move_base_simple/goal`.
- `launch/run.launch.py` validates launch args and builds per-mode parameter overrides with an `OpaqueFunction`; `launch/simulator.launch.py` composes the deterministic sim and also validates `use_pcd_map`/`pcd_map_file`.

## Gotchas

- Rebuild after header changes — libraries (`plan_env`, `bspline_opt`, `path_searching`, `traj_utils`) are separate ament packages.
- `install/`, `build/`, `log/` are in-tree; the repo also carries `map.pcd` (48 MB) at the workspace root used as a test map asset.
- GPU backend has no bundled GLFW anymore — it links the system packages only.
