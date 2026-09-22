"""Start the existing RMUC world with the PB2025 navigation vehicle only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


PB_ROOT = "/home/rainple/nav_test/third_party/pb2025_sources"
PB_INSTALL = "/home/rainple/nav_test/third_party/pb2025_install"


def _profile():
    """Read the shared PB vehicle profile.

    The file's top-level key is the profile container, not a node name, so it
    cannot be loaded directly as a node parameter file; the values are passed
    explicitly to the nodes that need them.
    """
    profile_path = os.path.join(
        get_package_share_directory("pb_vehicle_adapter"), "config", "pb_vehicle_profile.yaml"
    )
    with open(profile_path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]


def generate_launch_description():
    profile = _profile()
    sim_share = get_package_share_directory("mesh_navigation_tutorials_sim")
    adapter_share = get_package_share_directory("pb_vehicle_adapter")
    world_name = LaunchConfiguration("world_name")
    gui = LaunchConfiguration("start_gazebo_gui")

    declared_arguments = [
        DeclareLaunchArgument("world_name", default_value="rmuc2026_field"),
        DeclareLaunchArgument("contact_diagnostics", default_value="False", choices=["True", "False"]),
        DeclareLaunchArgument("drive_model", default_value="legacy", choices=["legacy", "pi"]),
        DeclareLaunchArgument("start_gazebo_gui", default_value="True", choices=["True", "False"]),
        DeclareLaunchArgument("start_rviz", default_value="True", choices=["True", "False"]),
        DeclareLaunchArgument(
            "rviz_config", default_value="pb_navigation.rviz",
            description="RViz config file in this package's rviz directory.",
        ),
        DeclareLaunchArgument("control_source", default_value="meshnav", choices=["meshnav", "jie", "dddmr"]),
        DeclareLaunchArgument(
            "spawn_rendering_sensors", default_value="True", choices=["True", "False"],
            description=(
                "False spawns the same robot without the gpu_lidar/camera sensors. "
                "Needed on machines without a GPU: software rendering makes those "
                "sensors drop the real-time factor to ~0.06, which no closed-loop "
                "run can survive. The three frameworks navigate from static maps."
            ),
        ),
        DeclareLaunchArgument("spawn_x", default_value="-11.9"),
        DeclareLaunchArgument("spawn_y", default_value="-4.4"),
        DeclareLaunchArgument("spawn_z", default_value="0.25"),
        # Spawn heading in degrees.  The tunnel acceptance starts the vehicle
        # facing the direction it will drive, like the manual drives do: a
        # heading-aligned controller would otherwise turn it around in place at
        # spawn, and the 4 - 7 cm base plate of the tunnel wall leaves no room for
        # that (doc section 9.5).
        DeclareLaunchArgument("spawn_yaw_deg", default_value="0"),
        DeclareLaunchArgument(
            "pb_robot_description_root", default_value=PB_ROOT + "/pb2025_robot_description"
        ),
        DeclareLaunchArgument("pb_resources_root", default_value=PB_ROOT + "/rmoss_gz_resources"),
        DeclareLaunchArgument(
            "pb_plugin_path",
            default_value=PB_INSTALL + "/rmoss_gz_plugins/plugins",
            description="Directory containing upstream MecanumDrive2 and LightBarController.",
        ),
    ]

    world_path = PathJoinSubstitution(
        [FindPackageShare("mesh_navigation_tutorials_sim"), "worlds", PythonExpression(['"', world_name, '" + ".sdf"'])]
    )
    resource_path = [
        PathJoinSubstitution([FindPackageShare("mesh_navigation_tutorials_sim"), "models"]),
        ":",
        PathJoinSubstitution([LaunchConfiguration("pb_robot_description_root"), "resource", "models"]),
        ":",
        PathJoinSubstitution([LaunchConfiguration("pb_resources_root"), "resource", "models"]),
        ":",
        EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
    ]
    plugin_path = [
        LaunchConfiguration("pb_plugin_path"),
        ":",
        EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": ["-r ", world_path, PythonExpression(['"" if ', gui, ' else " -s"'])]
        }.items(),
    )
    spawn_arguments = [
        "-name", "robot",
        "-x", LaunchConfiguration("spawn_x"),
        "-y", LaunchConfiguration("spawn_y"),
        "-z", LaunchConfiguration("spawn_z"),
        "-Y", PythonExpression(
            ["str(float('", LaunchConfiguration("spawn_yaw_deg"),
             "') * 3.141592653589793 / 180.0)"]),
    ]
    robot_model = PathJoinSubstitution(
        [FindPackageShare("pb_vehicle_adapter"), "models", "pb_navigation_robot.sdf"]
    )
    reduced_model = Command(
        [FindExecutable(name="python3"), " ",
         PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "tools", "reduced_robot_model.py"]),
         " ", robot_model, " --diagnostics ", LaunchConfiguration("contact_diagnostics"),
         " --drive ", LaunchConfiguration("drive_model")]
    )
    full_model = Command(
        [FindExecutable(name="python3"), " ",
         PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "tools", "reduced_robot_model.py"]),
         " ", robot_model, " --rendering True --diagnostics ", LaunchConfiguration("contact_diagnostics"),
         " --drive ", LaunchConfiguration("drive_model")]
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pb_navigation_robot",
        output="screen",
        arguments=["-string", full_model] + spawn_arguments,
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_rendering_sensors")),
    )
    spawn_without_sensors = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pb_navigation_robot",
        output="screen",
        arguments=["-string", reduced_model] + spawn_arguments,
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(LaunchConfiguration("spawn_rendering_sensors")),
    )
    robot_description = Command(
        [FindExecutable(name="cat"), " ", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "urdf", "pb_navigation_robot.urdf"])]
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="pb_robot_state_publisher",
        output="screen",
        remappings=[("robot_description", "/meshnav/robot_description")],
        parameters=[{"use_sim_time": True, "publish_frequency": 100.0, "robot_description": ParameterValue(robot_description, value_type=str)}],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pb_ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "config", "pb_ros_gz_bridge.yaml"])}],
    )
    truth = Node(
        package="pb_vehicle_adapter",
        executable="pb_ground_truth_adapter",
        name="pb_ground_truth_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                # Single same-step source (pb_gazebo_sim_support / ChassisTruth).
                # The old two-stream composition and its raw_pose/world_pose
                # parameters are gone; those topics are still recorded for
                # comparison but no longer drive /odom or the TF tree.
                "truth_topic": "/pb_sim/chassis_truth",
                "health_topic": "/pb/truth_health",
                "base_to_chassis_xyz": [float(v) for v in profile["base_to_chassis_xyz"]],
                "base_to_chassis_xyzw": [float(v) for v in profile["base_to_chassis_xyzw"]],
            },
        ],
    )
    # Post-action station keeping (P3).  When the navigation command stops -- the
    # ExePath action has ended and force_stop_at_goal publishes zero -- this vehicle
    # keeps sliding, because zero command is not a brake on a force-based drive
    # (measured: 1.7698 m over 95 s, and still 0.2649 m over 103 s after the terminal
    # controller's own hold was added, since a controller plugin gets no execution
    # opportunity once its action ends).  The adapter owns the output topic and the
    # watchdog, so the hold belongs here.  It is enabled explicitly for the simulation;
    # the code default stays off so the real-robot chain is unchanged unless set.
    velocity = Node(
        package="pb_vehicle_adapter",
        executable="pb_cmd_vel_adapter",
        name="pb_cmd_vel_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "control_source": LaunchConfiguration("control_source"),
            "hold_enabled": True,
            "hold_pose_topic": "/odom",
            "hold_gain": 1.0,
            "hold_max_linear_velocity": 0.1,
            "hold_max_angular_velocity": 0.2,
            "hold_angular_deadband": 0.02,
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "rviz", LaunchConfiguration("rviz_config")])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
            gazebo,
            bridge,
            spawn,
            spawn_without_sensors,
            state_publisher,
            truth,
            velocity,
            rviz,
        ]
    )
