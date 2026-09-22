# pb_gazebo_sim_support

Gazebo (Fortress / ignition-gazebo6) simulation-support systems for the PB
vehicle.  Simulation-only: no hardware driver belongs in this package, and a real
robot launch must not include it.

## ChassisTruth

`src/chassis_truth_system.cc` publishes a single, complete measurement of the
chassis:

* **same step** — the chassis `WorldPose` and the engine velocities are read in one
  `PostUpdate`, so pose and twist describe the same instant;
* **raw simulation time** — the message stamp is that step's `_info.simTime`;
* **pose in the world frame, twist in the chassis frame**, per the
  `ignition.msgs.Odometry` / `nav_msgs/Odometry` contract (the `frame_id` /
  `child_frame_id` header entries are what the ROS bridge maps);
* **six degrees of freedom** — full position (including z) and the full
  quaternion, including roll and pitch.  The drive plugin's own odometry publishes
  yaw-only orientation and no z, so it cannot represent a chassis on a ramp;
* **model-scoped entity resolution** — the chassis link is looked up inside the
  model the plugin is attached to (or an explicitly named model), never by bare
  link name across the world.

SDF parameters (all optional except the link):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `link_name` | `chassis` | chassis link to measure |
| `model_name` | `""` | only needed when the plugin is attached to the world |
| `topic` | `/pb_sim/chassis_truth` | Gazebo transport topic |
| `world_frame` | `rmuc2026_field` | `frame_id` of the pose |
| `child_frame` | `chassis` | `child_frame_id` of the twist |
| `publish_period_s` | `0.0` | 0 publishes every step; `0.01` = 100 Hz |

Wiring:

* `meshnav_demo_ws/src/pb_vehicle_adapter/models/pb_navigation_robot.sdf.xmacro`
  attaches the plugin; `tools/generate_descriptions.py` regenerates the SDF/URDF.
* `meshnav_demo_ws/src/pb_vehicle_adapter/config/pb_ros_gz_bridge.yaml` bridges
  `/pb_sim/chassis_truth` as `nav_msgs/Odometry`.
* `env-hooks/gazebo.dsv.in` puts this package's `lib/` on
  `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` / `GZ_SIM_SYSTEM_PLUGIN_PATH`, so nothing has to
  be hand-edited to find the plugin.
* `GroundTruthAdapter` in `pb_vehicle_adapter` consumes **only** this topic and
  re-references the measurement to `base_footprint` (see
  `doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md` section 4.3).

## ContactDiagnostics

`src/contact_diagnostics_system.cc` reads engine state and republishes it:

* **requested actuator effort** — the body wrench `MecanumDrive2` asked physics to
  apply this step (`ExternalWorldWrenchCmd`).  This is what the drive *wanted*, which
  is the only way to tell an actuator shortfall from a geometric block;
* **joint state** — per configured joint, engine velocity, position and the applied
  `JointForce`;
* **contacts** — contact positions, normals, depths and joint wrenches, when a
  contact sensor is actually feeding the system.

### Read-only, and default off

The plugin never writes a component, never applies a force and never touches mass,
friction, collision geometry, gravity or a controller parameter, so switching it on
cannot change the run it measures.  Every component access in the source is a
`Component<...>()` read; the only mutation is the publisher's own message.

`<enable>` defaults to **false**, and that default is implemented in
`DiagnosticsEnabledFromElement` (`include/pb_gazebo_sim_support/diagnostics_logic.h`)
rather than in the model file: an absent element, or any spelling other than
`true`/`false`/`1`/`0`, leaves the diagnostic off.  A diagnostic that instrumented a
run because of a typo would be worse than one that stayed silent.  With the switch off
the plugin creates no component and advertises no topic — verified live by starting
the simulation and observing that no `/pb_sim/diagnostics/*` topic exists.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enable` | `false` | master switch; nothing else happens while false |
| `enable_actuator` | `true` | requested wrench + joint state |
| `enable_contact` | `true` | republish contact sensor data |
| `model_name` | `""` | only needed when the plugin is attached to the world |
| `link_name` | `chassis` | link whose requested wrench is read |
| `joints` | the four wheels | whitespace/comma separated joint names |
| `topic_prefix` | `/pb_sim/diagnostics` | topic prefix |
| `publish_period_s` | `0.05` | publication throttle |

Topics (Gazebo transport, readable with `ign topic -e`):

| Topic | Type | Content |
| --- | --- | --- |
| `<prefix>/requested_wrench` | `ignition.msgs.Wrench` | force and torque the drive asked for |
| `<prefix>/joint_state` | `ignition.msgs.Float_V` | `[velocity * N, position * N, applied_force * N]` |
| `<prefix>/contact` | `ignition.msgs.Contacts` | contact positions, normals, depths, wrenches |

`JointStateLayout` fixes the `joint_state` layout in the shared header so the plugin
and any analyzer cannot drift apart, and the layout is unit tested.

### Enabling it for one run

Turn on the diagnostic in the model (this is the explicit switch, not a global flag):

```xml
<plugin filename="pb_contact_diagnostics_system"
        name="pb_gazebo_sim_support::ContactDiagnostics">
  <enable>true</enable>
  <enable_contact>false</enable_contact>   <!-- contact needs the prerequisites below -->
</plugin>
```

then regenerate and rebuild:

```bash
cd /home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter
PYTHONPATH=/home/rainple/nav_test/third_party/pb2025_python python3 tools/generate_descriptions.py \
  --xmacro models/pb_navigation_robot.sdf.xmacro \
  --robot-description-root /home/rainple/nav_test/third_party/pb2025_sources/pb2025_robot_description \
  --resources-root /home/rainple/nav_test/third_party/pb2025_sources/rmoss_gz_resources \
  --output-sdf models/pb_navigation_robot.sdf --output-urdf models/pb_navigation_robot.urdf
cd /home/rainple/nav_test/meshnav_demo_ws && colcon build --packages-select pb_vehicle_adapter --symlink-install
```

`tools/pb_sim/collect_run.sh` captures all three topics into every report
(`gz_requested_wrench.txt`, `gz_joint_state.txt`, `gz_contact.txt`), so an
instrumented run leaves the same kind of evidence as the rest of the matrix.

### Contact prerequisites

Actuator diagnostics have no prerequisite.  Contacts do: Fortress populates
`ContactSensorData` only when the model carries a `<sensor type="contact">` **and** the
world loads `ignition-gazebo-contact-system`.  When contacts are switched on without
those, the plugin says so explicitly instead of publishing an empty stream that would
read as "no contacts occurred" — the silent-empty-stream failure mode is exactly what
the plan's read-only requirement is trying to avoid.  No third-party file is edited to
provide them.

## Verify

```bash
source /opt/ros/humble/setup.bash
source /home/rainple/nav_test/meshnav_demo_ws/install/setup.bash
# with a simulation running in the same ROS domain and Gazebo partition:
python3 /home/rainple/nav_test/tools/pb_sim/verify_truth_source.py --seconds 6 --drive 0.2
python3 /home/rainple/nav_test/tools/pb_sim/verify_truth_source.py --seconds 3 --pause-test
```

The verifier checks non-zero strictly increasing stamps, 6-DOF finiteness, engine
twist vs the finite difference of the same poses, traceability of every published
`/odom` stamp, the health heartbeat, and that a paused source produces no new
samples or newer stamps.

## Build and test

```bash
cd /home/rainple/nav_test/meshnav_demo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select pb_gazebo_sim_support --symlink-install
colcon test --packages-select pb_gazebo_sim_support --event-handlers console_direct+
```

`test/test_diagnostics_logic.cc` covers the switches themselves (absent element stays
off, explicit true/false wins, an unparseable value falls back to the default), the
joint-list parsing and the `joint_state` layout.  The engines need a running
simulation, so they are covered by the live verification runs above and by the
`pb_vehicle_adapter` tests.

The CMake policy escape hatch in `CMakeLists.txt` is required because CMake 4
rejects the minimum-version declarations inside the system ignition/jsoncpp
config files.
