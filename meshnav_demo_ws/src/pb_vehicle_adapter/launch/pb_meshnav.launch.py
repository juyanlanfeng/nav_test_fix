"""Run MeshNav with the PB2025 vehicle and one selected velocity source."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pb_vehicle_adapter.readiness_gate import readiness_gate
import yaml


def _profile():
    profile_path = os.path.join(
        get_package_share_directory("pb_vehicle_adapter"), "config", "pb_vehicle_profile.yaml"
    )
    with open(profile_path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]


def generate_launch_description():
    profile = _profile()
    world_name = LaunchConfiguration("world_name")
    map_name = LaunchConfiguration("map_name")
    adapter_launch = PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "launch", "pb_vehicle_sim.launch.py"])
    mesh_launch = PathJoinSubstitution([FindPackageShare("mesh_navigation_tutorials"), "launch", "mbf_mesh_navigation_server_launch.py"])

    arguments = [
        DeclareLaunchArgument("world_name", default_value="rmuc2026_field"),
        DeclareLaunchArgument("map_name", default_value="rmuc2026_field"),
        DeclareLaunchArgument(
            "start_sim", default_value="True", choices=["True", "False"],
            description="Start the shared simulation; False reuses a running one.",
        ),
        DeclareLaunchArgument("start_gazebo_gui", default_value="True", choices=["True", "False"]),
        DeclareLaunchArgument("start_rviz", default_value="True", choices=["True", "False"]),
        DeclareLaunchArgument(
            "spawn_rendering_sensors", default_value="True", choices=["True", "False"],
            description="False spawns the robot without gpu_lidar/camera sensors (GPU-less hosts).",
        ),
        DeclareLaunchArgument("spawn_x", default_value="-11.9"),
        DeclareLaunchArgument("spawn_y", default_value="-4.4"),
        DeclareLaunchArgument("spawn_z", default_value="0.25"),
        DeclareLaunchArgument("spawn_yaw_deg", default_value="0"),
        DeclareLaunchArgument("drive_model", default_value="legacy", choices=["legacy", "pi"]),
        DeclareLaunchArgument(
            "mesh_map_working_path",
            default_value="/home/rainple/nav_test/meshnav_demo_ws/rmuc2026_pb_tunnel_relief_removed.h5",
            description="PB-specific MeshMap cache for the repaired RMUC2026 map; never reuse an older cache.",
        ),
        DeclareLaunchArgument(
            "startup_timeout_s", default_value="120",
            description="Readiness-gate timeout for pb_preflight; 0 = check once.",
        ),
        # The robot's footprint enters MeshNav only through the inflation layers:
        # `inscribed_radius` is the disc that is set lethal around every lethal
        # vertex.  These arguments expose the profile values so a footprint
        # experiment can be run without editing the shared profile
        # (config/pb_vehicle_profile.yaml stays the single source of truth for
        # the shipped configuration).
        # Heading-aligned control by default.  The base is a mecanum drive, so
        # holonomic control is possible, but in a 0.85 m corridor it lets the body
        # drift off the corridor axis while translating: the first negative-Y run
        # finished 39 deg off axis and put a rear wheel onto the 4-7 cm base plate
        # of the tunnel wall (doc section 9.5).  The physical tunnel drives
        # (log/accept_low_vehicle_tunnel.sh) steer along the corridor as well, so
        # this matches the verified capability.
        DeclareLaunchArgument(
            "mesh_controller_holonomic", default_value="false", choices=["true", "false"],
            description="true lets the mecanum base translate sideways.",
        ),
        DeclareLaunchArgument(
            "controller_plugin", default_value="mesh_controller",
            choices=["mesh_controller", "pb_terminal_controller"],
            description="Terminal-control layer used by this PB entry. "
                        "pb_terminal_controller is the self-developed state machine "
                        "(doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md R3); "
                        "mesh_controller stays the stock comparison path.",
        ),
        DeclareLaunchArgument("static_inscribed_radius", default_value=str(profile["static_inscribed_radius"])),
        DeclareLaunchArgument("static_inflation_radius", default_value=str(profile["static_inflation_radius"])),
        DeclareLaunchArgument("height_diff_threshold", default_value="0.2"),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adapter_launch),
        launch_arguments={
            "world_name": world_name,
            "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui"),
            # This entry starts its own MeshNav RViz below.
            "start_rviz": "False",
            "control_source": "meshnav",
            "spawn_rendering_sensors": LaunchConfiguration("spawn_rendering_sensors"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw_deg": LaunchConfiguration("spawn_yaw_deg"),
            "drive_model": LaunchConfiguration("drive_model"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_sim")),
    )
    # start_sim:=false reuses a running simulation: switch the existing velocity
    # selector to this framework instead of starting a second one.
    source_switch = ExecuteProcess(
        cmd=["ros2", "param", "set", "/pb_cmd_vel_adapter", "control_source", "meshnav"],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("start_sim")),
    )
    meshnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mesh_launch),
        launch_arguments={
            "mesh_map_path": PathJoinSubstitution([FindPackageShare("mesh_navigation_tutorials"), "maps", PythonExpression(['"', map_name, '" + ".ply"'])]),
            "mesh_map_working_path": LaunchConfiguration("mesh_map_working_path"),
            "mesh_controller_holonomic": LaunchConfiguration("mesh_controller_holonomic"),
            "height_diff_threshold": LaunchConfiguration("height_diff_threshold"),
            "static_inflation_radius": LaunchConfiguration("static_inflation_radius"),
            "static_inscribed_radius": LaunchConfiguration("static_inscribed_radius"),
            "obstacle_robot_height": str(profile["robot_height"]),
            "obstacle_inflation_radius": str(profile["obstacle_inflation_radius"]),
            "obstacle_inscribed_radius": str(profile["obstacle_inscribed_radius"]),
            "controller_plugin": LaunchConfiguration("controller_plugin"),
            # PB-owned parameter file, loaded only when this entry selects the
            # experimental terminal controller: the shared MeshNav config carries no
            # PB simulation parameters or health topic.
            "extra_params_file": PythonExpression([
                "'", PathJoinSubstitution([
                    FindPackageShare("pb_vehicle_adapter"), "config", "pb_terminal_controller.yaml"
                ]),
                "' if '", LaunchConfiguration("controller_plugin"),
                "' == 'pb_terminal_controller' else ''",
            ]),
        }.items(),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_meshnav", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "rviz", "pb_meshnav.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    readiness = readiness_gate("meshnav", LaunchConfiguration("startup_timeout_s"))
    return LaunchDescription(arguments + [simulation, source_switch, meshnav, rviz, readiness])
