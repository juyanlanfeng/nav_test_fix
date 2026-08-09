"""Run JIE's path controller against the Ceres Gazebo demo robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("octo_planner"),
        "config",
        "meshnav_ceres_controller.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "require_start_command",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Wait for std_msgs/Bool true on /start_navigation after a path arrives"
                ),
            ),
            Node(
                package="octo_planner",
                executable="d1_controller",
                name="d1_controller",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": True,
                        "path_topic": "/planned_path",
                        "start_navigation_topic": "/start_navigation",
                        "stop_navigation_topic": "/stop_navigation",
                        # MeshNav uses TwistStamped on /cmd_vel.  JIE uses an
                        # unstamped Twist on this separate bridge topic.
                        "cmd_vel_topic": "/cmd_vel_jie",
                        "manual_cmd_vel_topic": "/web_cmd_vel",
                        "tracking_point_marker_topic": "/tracking_point_marker",
                        "map_frame": "map",
                        "base_frame": "base_link",
                        "base_frame_candidates": "base_link,base_footprint",
                        "enable_lateral_motion": True,
                        "require_start_command": ParameterValue(
                            LaunchConfiguration("require_start_command"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="octo_planner",
                executable="jie_twist_stamper",
                name="jie_twist_stamper",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/cmd_vel_jie",
                        "output_topic": "/cmd_vel",
                        "frame_id": "base_link",
                    }
                ],
            ),
        ]
    )
