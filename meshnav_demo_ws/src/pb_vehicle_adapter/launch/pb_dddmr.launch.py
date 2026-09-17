"""Run DDDMR's global planner and p2p move_base with the PB2025 vehicle.

Source the DDDMR runtime environment first (it provides gtsam/PCL/small_gicp and
the DDDMR packages):

    source /home/rainple/nav_test/third_party/setup_dddmr_env.sh

This entry deliberately does NOT start mcl_3dl, LeGO-LOAM, occupancy2ground or a
hardcoded base_link->laser_link TF; it uses the PB ground-truth TF and /odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pb_vehicle_adapter.readiness_gate import readiness_gate
import yaml


DDDMR_MAP_DIR = "/home/rainple/nav_test/field/converted_rmuc2026/dddmr_nav"


def _profile():
    profile_path = os.path.join(
        get_package_share_directory("pb_vehicle_adapter"), "config", "pb_vehicle_profile.yaml"
    )
    with open(profile_path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]


def generate_launch_description():
    _profile()
    vehicle_launch = PathJoinSubstitution(
        [FindPackageShare("pb_vehicle_adapter"), "launch", "pb_vehicle_sim.launch.py"]
    )
    dddmr_config = PathJoinSubstitution(
        [FindPackageShare("pb_vehicle_adapter"), "config", "dddmr_rmuc2026.yaml"]
    )
    arguments = [
        DeclareLaunchArgument("world_name", default_value="rmuc2026_field"),
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
        DeclareLaunchArgument("map_bundle", default_value=DDDMR_MAP_DIR),
        DeclareLaunchArgument(
            "vehicle_profile",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pb_vehicle_adapter"), "config", "pb_vehicle_profile.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "startup_timeout_s", default_value="120",
            description="Readiness-gate timeout for pb_preflight; 0 = check once.",
        ),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_launch),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui"),
            # This entry starts its own DDDMR RViz below.
            "start_rviz": "False",
            "control_source": "dddmr",
            "spawn_rendering_sensors": LaunchConfiguration("spawn_rendering_sensors"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_sim")),
    )
    source_switch = ExecuteProcess(
        cmd=["ros2", "param", "set", "/pb_cmd_vel_adapter", "control_source", "dddmr"],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("start_sim")),
    )
    map_publisher = Node(
        package="pb_vehicle_adapter", executable="pb_dddmr_map_publisher",
        name="pb_dddmr_map_publisher", output="screen",
        parameters=[{"use_sim_time": True, "map_dir": LaunchConfiguration("map_bundle")}],
    )
    global_planner = Node(
        package="global_planner", executable="global_planner_node", output="screen",
        parameters=[dddmr_config, {"use_sim_time": True}],
    )
    move_base = Node(
        package="p2p_move_base", executable="p2p_move_base_node", output="screen",
        parameters=[dddmr_config, {"use_sim_time": True}],
        remappings=[("cmd_vel_stamped", "/pb/dddmr_cmd_vel_stamped")],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_dddmr", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "rviz", "pb_dddmr.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    readiness = readiness_gate("dddmr", LaunchConfiguration("startup_timeout_s"))
    return LaunchDescription(
        arguments + [simulation, source_switch, map_publisher, global_planner, move_base, rviz, readiness]
    )
