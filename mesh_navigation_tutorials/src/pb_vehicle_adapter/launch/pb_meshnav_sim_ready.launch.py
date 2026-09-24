"""Practical simulation preset; not a hardware configuration or full acceptance."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "start_gazebo_gui": "True", "start_rviz": "True",
        "spawn_rendering_sensors": "False",
        "spawn_x": "-1.9", "spawn_y": "5.95", "spawn_z": "0.25",
        "spawn_yaw_deg": "0",
        "mesh_map_working_path": str(Path.home() / ".ros/pb_practical_navigation.h5"),
    }
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    settings = {k: LaunchConfiguration(k) for k in defaults}
    settings.update(start_sim="True", drive_model="pi",
                    controller_plugin="pb_terminal_controller",
                    height_diff_threshold="0.08", startup_timeout_s="180")
    entry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("pb_vehicle_adapter"), "launch", "pb_meshnav.launch.py"])),
        launch_arguments=settings.items())
    return LaunchDescription(arguments + [entry])
