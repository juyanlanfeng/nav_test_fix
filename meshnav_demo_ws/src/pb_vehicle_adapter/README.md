# PB2025 Gazebo vehicle adapter

This package runs the RMUC2026 field with a **low-clearance** PB2025 test vehicle:
the original wheels and omnidirectional motion, 2-D lidar, MID-360, IMU and
camera, but without the gimbal, armor, light bar, speed monitor and in-chassis
projectile container - the superstructure that put the collision envelope at
0.4491 m and made the robot unable to enter the RMUC tunnels (local clear height
0.2472 m).  The body is a 0.36 x 0.24 x 0.13 m box and the collision envelope is
0.5656 x 0.4332 x 0.2258 m, so it passes both tunnels with ~0.02 m of head
clearance.  See `doc/PB_LOW_CLEARANCE_VEHICLE_PLAN.md` for the design and its
acceptance record.

The package supplies exactly one navigation transform chain
(`map -> odom -> base_footprint`) from Gazebo world truth and routes the selected
stamped navigation command through a 0.5-second watchdog before it reaches
`/robot/cmd_vel`.

Build the package after generating or changing the vehicle descriptions:

```bash
source /opt/ros/humble/setup.bash
cd /home/rainple/nav_test/meshnav_demo_ws
colcon build --packages-select pb_vehicle_adapter --symlink-install
source install/setup.bash
```

Start the basic vehicle simulation first:

```bash
ros2 launch pb_vehicle_adapter pb_vehicle_sim.launch.py \
  world_name:=rmuc2026_field start_gazebo_gui:=False
```

Use one navigation stack at a time:

```bash
ros2 launch pb_vehicle_adapter pb_meshnav.launch.py
ros2 launch pb_vehicle_adapter pb_jie.launch.py
ros2 launch pb_vehicle_adapter pb_dddmr.launch.py   # source setup_dddmr_env.sh first
```

`pb_meshnav.launch.py` creates a PB-specific MeshNav cache and disables the
Ceres slope-corridor profile.  `pb_jie.launch.py` uses the existing converted
RMUC2026 PCD, the JIE planner/controller, and sends its stamped command only
to the adapter's JIE input.  `pb_dddmr.launch.py` starts the DDDMR map publisher
plus `global_planner_node` and `p2p_move_base_node` (Omni trajectory generator,
`/dddmr/mapcloud` and `/dddmr/mapground`, output on `/pb/dddmr_cmd_vel_stamped`).

All three navigation entries share the same contract: `start_sim` (true starts
the shared simulation, false reuses a running one and switches the velocity
selector), `start_rviz` (each entry opens its own framework-specific RViz),
`world_name`, `map_bundle`, `vehicle_profile`, `startup_timeout_s` (readiness
gate: the entry runs `pb_preflight --wait-timeout <value>` after spawning, and
prints the failing items instead of navigating blindly; `0` checks once) and
`spawn_rendering_sensors` (see below).  Only one framework may drive the vehicle
at a time.

`pb_jie.launch.py` also takes `start_click_selector` (default `True`): pass
`False` for scripted runs, which publish `/start_point` and `/goal_point`
directly, and to save one DDS participant on hosts with a low participant limit.

`spawn_rendering_sensors:=False` spawns the same robot without the two
`gpu_lidar`s and the camera, and is a **fallback for environments that cannot
render**: when the process has no GPU render node (`/dev/dri`), no `/dev/nvidia*`
and no X socket, Fortress renders those sensors in software and the simulation
real-time factor collapses from 0.997 to **0.059**, so no closed-loop run can
finish.  That is an environment condition, not a property of this project: on a
normal desktop session with a working GPU (for example the RTX 5070 + Xorg on
this machine) keep the default `True` and you get the full sensor set at speed.
Measure it before deciding:

```bash
bash log/probe_rtf2.sh                       # sensors on  (default model)
ACCEPT_SENSORS=False bash log/probe_rtf2.sh  # sensors off (reduced model)
watch -n1 nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
```

None of the frameworks need those sensors for navigation (they navigate from
their own static maps), so the reduced model is also useful for pure
navigation runs.  It is produced on the fly by `tools/reduced_robot_model.py`,
so `models/pb_navigation_robot.sdf` stays the single source of truth.

Read-only readiness check (exits non-zero with per-item failures; `--wait-timeout`
keeps re-running the checks until they pass or the timeout expires):

```bash
ros2 run pb_vehicle_adapter pb_preflight --framework meshnav
ros2 run pb_vehicle_adapter pb_preflight --framework jie
ros2 run pb_vehicle_adapter pb_preflight --framework dddmr
ros2 run pb_vehicle_adapter pb_preflight --framework dddmr --wait-timeout 120
```

Unified goal/cancel client (wraps each framework's native interface).  It runs on
simulation time, logs the start/final map pose, and takes `--timeout` (wall
seconds to wait for the action result, default 180):

```bash
ros2 run pb_vehicle_adapter pb_nav_goal --framework meshnav --case smoke
ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --case smoke
ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --case smoke --timeout 600
ros2 run pb_vehicle_adapter pb_nav_goal --framework dddmr --case smoke --cancel
```

Framework-specific notes:

- **JIE**: `/start_point` and `/goal_point` are published with
  `TRANSIENT_LOCAL + RELIABLE` because `jie_path_node` subscribes with
  `QoS(1).transient_local().reliable()`; a default (volatile) publisher is
  silently incompatible and the planner never answers.  `d1_controller` has no
  arrival topic, so the client judges arrival from the live map pose (0.30 m xy
  tolerance) after sending `/start_navigation`.
- **MeshNav**: `GetPath` is only followed by `ExePath` when the plan succeeded
  and is non-empty; the outcome codes are reported as-is.
- **DDDMR**: the action's SUCCESS is `status == 1`, not 0.

The three navigation stacks live in separate workspaces, so a shell that drives
them needs all of them sourced - for example:

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash   # MeshNav + this package
source /home/rainple/nav_test/jie_3d_nav/install/setup.bash        # jie_octomap + octo_planner
source /home/rainple/nav_test/third_party/setup_dddmr_env.sh       # DDDMR only
```

DDDMR map bundle (regenerate after a map or profile change).  `mapcloud` is the
**obstacle** cloud and `mapground` the traversable surface: DDDMR's StaticLayer
marks every ground node whose distance to `mapcloud` is below `inscribed_radius`
as lethal, so a `mapcloud` that also contains the floor makes the whole map
unreachable:

```bash
field/.step_convert_venv/bin/python \
  meshnav_demo_ws/src/pb_vehicle_adapter/tools/build_dddmr_maps.py
ros2 run pb_vehicle_adapter pb_dddmr_map_publisher   # Reliable/Transient Local
```

The generator writes `validation_report.json` with a `navigability` section
(distance from each ground node to the nearest obstacle) so connectivity can be
checked before a live run.

DDDMR also needs its dedicated runtime environment, which patches nothing in the
system but does apply one documented upstream fix:

```bash
source /home/rainple/nav_test/third_party/setup_dddmr_env.sh
# third_party/patches/dddmr-06d50cc-actuator-type.patch - upstream 06d50cc never
# calls configurateActuatorType(), so actuator_type_ stays indeterminate and
# P2PMoveBase::publishVelocity() discards every trajectory.
```

The wrapper no longer instantiates the upstream `rm25_example_robot` macro: it
defines a local `pb_low_chassis` block (base_footprint, base_to_chassis, box
chassis, four upstream wheel macros) plus local low-mounted sensor blocks whose
geometry is a simple box with a matching collision.  The sensor link/sensor
names and topics are unchanged, so the bridge and the topic assignment in
`tools/generate_descriptions.py` keep working.  Nothing under `third_party/` is
modified.

Rebuild the envelope after any model change and keep the profile honest:

```bash
python3 -m pb_vehicle_adapter.robot_envelope          # prints the AABB + per-primitive rows
python3 -m pytest test/test_low_vehicle_description.py # profile/DDDMR must match
```

Tunnel acceptance helpers:

```bash
field/.step_convert_venv/bin/python field/check_tunnel_clearance.py     # static + tilted sweep
bash log/accept_low_vehicle_tunnel.sh                                   # drive both tunnels, both ways
field/.step_convert_venv/bin/python field/check_trajectory_clearance.py \
  log/low_plus_y_trajectory.csv log/low_minus_y_trajectory.csv          # margins from the log
```

Generate the SDF and URDF only when changing `models/pb_navigation_robot.sdf.xmacro`:

```bash
PYTHONPATH=/home/rainple/nav_test/third_party/pb2025_python \
  python3 tools/generate_descriptions.py
```

## Acceptance fixes (2026-09-08)

These changes close the four items raised by the closed-loop review:

- **RViz action/goal chain**: `rviz/pb_navigation.rviz` now loads the
  `rviz_mbf_plugins/MbfGoalActions` panel and the `Mesh Goal` tool (topic
  `/rviz/goal_pose`) for MeshNav, plus the `Publish Point` tool
  (`/clicked_point`) and the `/selection_markers` MarkerArray for JIE.
- **JIE start/goal selection**: `launch/pb_jie.launch.py` starts
  `jie_octomap/rviz_click_selector_node`, which alternates RViz clicks between
  `/start_point` and `/goal_point` for `jie_path_node`.
- **Watchdog clock**: `pb_cmd_vel_adapter` creates its watchdog timer on a
  `ClockType.STEADY_TIME` clock, so it keeps clearing stale commands while
  `/clock` is paused. Verified directly: with `use_sim_time=true` and no
  `/clock` published, the steady-clock timer fires while the ROS-time timer
  does not.
- **Command timestamps**: `pb_cmd_vel_adapter` rejects unstamped commands and
  commands whose stamp is more than `max_stamp_age_s` (default 0.5 s) away from
  the current time.
- **Profile loading**: `launch/pb_vehicle_sim.launch.py` reads
  `config/pb_vehicle_profile.yaml` and passes `base_to_chassis_xyz` /
  `base_to_chassis_xyzw` explicitly to `pb_ground_truth_adapter`. The profile's
  top-level key (`pb_vehicle`) is a container, not a node name, so it cannot be
  loaded as a node parameter file directly.
- **Tests**: `test/test_cmd_vel_adapter.py` adds seven cases (steady-clock
  timer, fresh/stale/unstamped/wrong-frame rejection, timeout clearing,
  finiteness); `colcon test --packages-select pb_vehicle_adapter` runs them.

## Closed-loop acceptance fixes (2026-09-15)

Running the shared simulation with the DDDMR stack end to end exposed seven more
defects; all of them are fixed and covered by tests or recorded evidence (see
`doc/THREE_PLANNERS_PB_SIM_IMPLEMENTATION.md` section 17):

- **`mapcloud` semantics**: it must be the obstacle cloud, not the complete field
  geometry. `tools/build_dddmr_maps.py` now samples the non-support faces and
  drops anything below the local support surface, and reports the resulting
  ground-to-obstacle distances.
- **DDDMR runtime environment**: the isolated ackermann-msgs prefix's plain `lib`
  directory is now on `LD_LIBRARY_PATH`; its introspection library is dlopen'ed,
  so `ldd` never showed it and `p2p_move_base_node` died at startup.
- **Upstream actuator type**: the tracked patch
  `third_party/patches/dddmr-06d50cc-actuator-type.patch` makes the generators
  configure their actuator type, without which the vehicle never receives a
  command.
- **GPU-less simulation**: `spawn_rendering_sensors` (see above).
- **Rotation limits**: the two rotate-in-place generators sample `theta` inside
  `[min_vel_theta, max_vel_theta]`; with the inherited 0.1 rad/s ceiling the
  initial 136 deg alignment needed 24 s and always tripped `p2p_move_base`'s 15 s
  oscillation watchdog, so the ceiling is now 0.4 rad/s.
- **Oscillation watchdog distance**: it only resets after `oscillation_distance`
  metres or 1 rad. The upstream 5 m assumes a ~1 m/s platform; at the PB's
  0.2 m/s it can never be reached inside the 15 s patience, so every goal became
  a false oscillation. It is now 1.0 m, guarded by a regression test.
- **`pb_nav_goal` robustness**: goals run on simulation time, a result future
  that resolves to `None` (abort/cancel) is reported instead of raising, and
  `--timeout` controls the wall-clock budget.

- **JIE point QoS**: `pb_nav_goal` publishes `/start_point` and `/goal_point`
  with `TRANSIENT_LOCAL + RELIABLE` to match `jie_path_node`'s subscription;
  the default volatile publisher was dropped silently and no path was planned.
- **JIE arrival**: `pb_nav_goal` waits for the robot's map pose to reach the
  goal because `d1_controller` publishes no arrival topic. The tolerance is the
  controller's own (`jie` 0.10 m, `meshnav` 0.20 m) and has to hold for three
  consecutive samples, so a run cannot report "arrived" while the vehicle is
  still driving through the tolerance circle.

Closed-loop results in the shared simulation: MeshNav `GetPath outcome=0` ->
`ExePath outcome=0 message=Controller succeeded; arrived at goal!` (0.14 m from
the goal), JIE `planned_path with 20 poses` -> `Navigation execution started` ->
`arrived: 0.291 m from the goal`. Reproduce with `log/accept_meshnav_run.sh` and
`log/accept_jie_run.sh`.

DDDMR is no longer part of the acceptance scope (user decision, 2026-09-15).
Its artifacts stay in the package, and the one limitation that was measured - the
simulated PB cannot execute the lateral/diagonal body-frame commands that
DDDMR's omnidirectional controller produces, because upstream `MecanumDrive2`
applies a rotated PID force to the chassis and the wheels are isotropic
cylinders with `mu = mu2 = 0.2` - is recorded with numbers and next steps in the
doc's section 17.4.

## Tunnel acceptance with the low vehicle (2026-09-16)

The low-clearance vehicle (doc/PB_LOW_CLEARANCE_VEHICLE_PLAN.md) has to plan and
drive the two 0.85 m RMUC tunnels. What the tunnel cost that work, and what to
keep in mind when touching it:

- **The footprint is a disc in both planners.** MeshNav's
  `static_inflation.inscribed_radius` is a lethal disc around every lethal
  vertex, JIE's `robot_radius_xy` is a cylinder. Both are set from
  `pb_vehicle_profile.yaml`, so the value is derived from the envelope
  (`pb_vehicle_adapter.robot_envelope.footprint_radii()`): the *inscribed*
  radius = half width = 0.216608 m. For a straight corridor the free band is
  `w - 2 r`, so this radius refuses exactly the corridors the rectangular body
  cannot pass; the circumscribed radius (0.356 m) rejects the tunnels that the
  vehicle physically drives through. `test_low_vehicle_description.py` fails if
  either bound is violated.
- **Do not size the vehicle for the circle.** The vehicle is 0.4332 m wide in a
  0.85 m corridor; the physical drives leave 0.17 - 0.21 m per side, and the
  tunnelled manoeuvres are straight, so the disc model is sound there. Turning in
  place is *not* covered by it (the corners reach 0.356 m) - see the doc's
  limitations.
- **The tunnel wall has a 4 - 7 cm base plate** at `|y| ~ 6.36`. A body that
  yaws more than ~30 deg inside the corridor puts a wheel on it and jams. The
  acceptance therefore spawns the vehicle facing its travel direction
  (`spawn_yaw_deg`), drives heading-aligned
  (`mesh_controller_holonomic: false`, JIE `align_final_yaw: false`) and uses the
  measured corridor centreline (`y = +-6.00`) as the goal.
- **Path sampling matters**: the planner's default `step_width` (0.4 m) is wider
  than the 0.30 m free band, which made the published polyline weave across the
  corridor; it is 0.05 m for this map.
- **Every run needs its own Gazebo partition** (`IGN_PARTITION`/`GZ_PARTITION`,
  set by `log/accept_env.sh`) and its own ROS domain below 233: Gazebo transport
  is not ROS-domain scoped, so a leftover simulation keeps answering
  `ros_gz_sim create` and publishing `/odom` - the "phantom robot" that made an
  earlier run plan for a vehicle standing several metres away.

Evidence and reproduction commands: `doc/PB_LOW_CLEARANCE_VEHICLE_PLAN.md`
sections 9.5 and 10; planning grids with `log/probe_tunnel_plan_grid.sh` and
`log/probe_jie_plan_grid.sh`; layer forensics with `log/probe_save_layers.sh` +
`field/read_mesh_layers.py`.

### RViz shows an empty scene

The configs load correctly, so an empty view is almost always one of these:

- **The camera looks at the wrong place.** The orbit view focused on the launch
  file's default spawn `(-11.9, -4.4)` while the tunnel runs spawn the vehicle at
  `(-1.90, +-5.95)`, i.e. ~14 m to the side and outside the frustum. Both configs
  now use `Target Frame: base_footprint` and focus on the tunnel mouth, so the
  camera follows whatever vehicle is spawned; you can also press `FocusCamera` or
  set `Views -> Target Frame` yourself.
- **Displays subscribed to topics nobody publishes.** `/pcd_points` is
  `pcd_to_octomap`'s *input* point cloud (no publisher), and
  `/octomap_occupied_markers` does not exist: `jie_path_node` publishes the
  traversability/preblocked overlay as a single `Marker` on
  `/preblocked_cells_markers`. `pb_jie.rviz` now subscribes `/cloud` (live lidar
  cloud), `/preblocked_cells_markers`, `/scan`, `/tracking_point_marker`,
  `/selection_markers` and `/planned_path`.
- **`/octomap` needs a plugin that is not installed.** The voxel map is
  `octomap_msgs/Octomap`; showing it requires `ros-humble-octomap-rviz-plugins`
  (`sudo apt install ros-humble-octomap-rviz-plugins`). Without it, use
  `/preblocked_cells_markers`, which carries the same free/blocked information as
  markers.

Quick checks on a live system:

```bash
ros2 topic info /preblocked_cells_markers     # expect 1 publisher
ros2 topic info /pcd_points                   # 0 publishers -> that display stays empty
ros2 run tf2_ros tf2_echo map base_footprint  # expect the vehicle's spawn pose
```

`/planned_path` only appears after a goal request (`pb_nav_goal --plan-only ...`),
and the RobotModel only appears once the simulation publishes
`map -> base_footprint`. The configs are symlinked from `install/` into `src/`, so
edits take effect by re-opening them in RViz (`File -> Open Config`) without a
rebuild.

### Where is the floor? (JIE map visualisation)

The blue clutter in the default JIE view is `/preblocked_cells_markers`: the cells
`jie_path_node` pre-blocks, i.e. an obstacle map, not the ground. The floor,
walls and tunnel roofs live in two other places:

- **`/octomap`** (`octomap_msgs/Octomap`, 791664 voxels built from
  `field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd`). RViz can only show it
  with `ros-humble-octomap-rviz-plugins`, which is *not* installed here:
  `sudo apt install ros-humble-octomap-rviz-plugins`, then Add -> By topic ->
  `/octomap`.
- **`/cloud_pcd`** (the same PCD as a `PointCloud2`, ~631564 points, frame
  `map`), published by `pb_jie.launch.py` (`publish_map_cloud:=True`, node
  `pb_map_cloud`) and displayed by `pb_jie.rviz` as "MapCloud (static PCD)".

  **QoS matters here.** RViz's PointCloud2 display subscribes with
  `Durability: Transient Local`, so the publisher has to offer transient local.
  `pcl_ros/pcd_to_pointcloud` does **not** (`ros2 topic info --verbose` reports
  `Durability: VOLATILE`), and the mismatch is fatal: RViz logs

  ```text
  [pb_map_cloud]: ... incompatible QoS ... Last incompatible policy: DURABILITY_QOS_POLICY
  ```

  and shows nothing, whether the cloud is sent once or repeatedly. The
  replacement is `pb_vehicle_adapter/pcd_publisher.py` (entry point
  `pb_pcd_publisher`, used by the launch file): it reads the PCD once, publishes
  with `TRANSIENT_LOCAL + RELIABLE + KEEP_LAST(1)` and stays alive, so an RViz
  started later still receives the cloud.

Because only ~22 % of that cloud is floor, there is also a **ground-only layer**:
`field/build_jie_floor_pcd.py` keeps the lowest surface of every 4 cm (x, y) cell
(tolerance 0.05 m) below 0.25 m, which leaves the floor, the ramps and the
0.203 m platform but drops the tunnel roofs and the surrounding structures
(307206 of 631564 points, z -0.06 .. 0.18 m):

```bash
field/.step_convert_venv/bin/python field/build_jie_floor_pcd.py
# -> field/converted_rmuc2026/jie_nav/rmuc2026_field_floor.pcd
```

`pb_jie.launch.py` publishes it as `/cloud_pcd_floor` (`publish_floor_cloud`,
default True) and `pb_jie.rviz` shows it as "MapFloor (ground only)" in cyan,
next to "MapCloud". `AxisColor` was dropped from both displays: the floor (z ~ 0)
sits at the blue end of the rainbow and vanished into the blue
`/preblocked_cells_markers` overlay.

### Picking start/goal in RViz and starting the drive

The **Publish Point** tool feeds `/clicked_point`, and `rviz_click_selector_node`
alternates between `/start_point` (green arrow) and `/goal_point` (red arrow):
**the first click is the start, the second the goal**, the third is a new start
again. A click lands on the `map` z = 0 plane (the grid is 1 m), so no point
cloud is needed to pick coordinates - the clouds only make it easier to judge
where you are.

Clicking only *plans*: `jie_path_node` publishes `/planned_path` and
`d1_controller` waits. It needs `/start_navigation` **after** the path exists - a
start command that arrives first is dropped with "no pending planned_path is
available":

```bash
# after the path appears (RViz shows the blue line, or the log prints
# "Received planned_path with N poses. Waiting for /start_navigation confirmation.")
ros2 topic pub --once /start_navigation std_msgs/msg/Bool "{data: true}"
# stop
ros2 topic pub --once /stop_navigation std_msgs/msg/Bool "{data: true}"
```

To get one-click driving instead, start the stack with
`auto_start_navigation:=True`: the selector then watches `/planned_path` itself
and sends the confirmation as soon as the plan for your goal arrives.

```bash
ros2 launch pb_vehicle_adapter pb_jie.launch.py start_sim:=False start_rviz:=True \
  auto_start_navigation:=True
```

Verified end to end with `log/verify_rviz_point_nav.sh` (two `/clicked_point`
messages, no GUI): `Set START` -> `Set GOAL` -> `Auto start: sent
/start_navigation after a planned_path with 34 poses` -> `Navigation execution
started` -> `Goal reached ... controller is idle`, with the vehicle moving from
`x = -1.900` to `x = -0.668`.

Alternatives to clicking: `pb_nav_goal` does plan + start in one command, e.g.

```bash
ros2 run pb_vehicle_adapter pb_nav_goal --framework jie --x -0.60 --y 6.00 --z 0.06 --yaw 0
```

To get the clouds into a running session without restarting the launch, use the
latched publisher (the RViz displays expect `Transient Local`):

```bash
ros2 run pb_vehicle_adapter pb_pcd_publisher --ros-args \
  -r __node:=floor_display_debug \
  -p file_name:=/home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field_floor.pcd \
  -p topic:=/cloud_pcd_floor -p frame_id:=map -p period:=0.0
```

Verify the durability contract on a live system - a transient-local subscriber
started *after* the publisher is exactly what RViz does:

```bash
ros2 topic info /cloud_pcd_floor --verbose            # expect Durability: TRANSIENT_LOCAL
python3 - <<'PROBE'
import rclpy, time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
rclpy.init(); node = rclpy.create_node("probe")
qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
got = {}
node.create_subscription(PointCloud2, "/cloud_pcd_floor",
                         lambda m: got.update(width=m.width, frame=m.header.frame_id), qos)
deadline = time.time() + 10
while time.time() < deadline and not got:
    rclpy.spin_once(node, timeout_sec=0.2)
print(got or "not received")
PROBE
# -> {'width': 307206, 'frame': 'map'}
```

If you prefer to keep `pcd_to_pointcloud`: set the RViz displays'
`Durability Policy` to `Volatile` and run it with `period:=1.0` so it keeps
re-sending. That works, but it streams 631k points (~7.6 MB/s) forever, which is
why the latched publisher is the default.

For MeshNav the equivalent ground is `/move_base_flex/mesh` (the `Mesh Map`
display) plus `/move_base_flex/vertex_costs` for the cost colouring.
