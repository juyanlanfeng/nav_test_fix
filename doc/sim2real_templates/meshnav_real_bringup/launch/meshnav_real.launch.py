"""Start only the navigation side of a validated hardware interface bundle."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from meshnav_real_bringup.bundle import BundleError, flatten_parameters, load_bundle


def _nodes(context):
    requested_rviz = LaunchConfiguration("start_rviz").perform(context).lower()
    if requested_rviz not in ("true", "false"):
        raise BundleError("start_rviz must be true or false")
    bundle = load_bundle(
        LaunchConfiguration("bundle_dir").perform(context),
        LaunchConfiguration("cache_dir").perform(context) or None,
    )
    # Validate optional UI dependencies before starting any process. The default
    # headless route never looks up a Gazebo, tutorial, PB or RViz package.
    rviz_config = None
    if requested_rviz == "true":
        try:
            get_package_share_directory("rviz2")
            get_package_share_directory("rviz_mesh_tools_plugins")
        except Exception as error:
            raise BundleError(
                "start_rviz=true requires rviz2 and rviz_mesh_tools_plugins in this shell; "
                "build/source them on the desktop or use start_rviz:=false"
            ) from error
        share = Path(get_package_share_directory("meshnav_real_bringup"))
        rviz_config = str(share / "config" / "meshnav_real.rviz")
    actions = [
        LogInfo(msg="Validated external bundle; command_gateway starts DISABLED."),
        LogInfo(msg=f"Chassis-facing Twist topic: {bundle['gateway']['output_topic']}"),
        LogInfo(msg=f"Map cache: {bundle['nav']['mesh_map']['mesh_working_file']}"),
        # Start the disabled gateway first. Every node has wall-time parameters
        # explicitly supplied; no /clock, truth, driver or transform is created.
        Node(
            package="meshnav_runtime", executable="command_gateway",
            name="command_gateway", output="screen",
            parameters=[bundle["gateway"]],
        ),
        Node(
            package="pcl_obstacle_point_subtraction",
            executable="pcl_obstacle_point_subtraction",
            name="obstacle_sub", output="screen", parameters=[bundle["pcl"]],
        ),
        Node(
            package="mbf_mesh_nav", executable="mbf_mesh_nav",
            name="move_base_flex", output="screen",
            parameters=[flatten_parameters(bundle["nav"])],
            remappings=[("/move_base_flex/cmd_vel", "/nav/cmd_vel")],
        ),
        Node(
            package="meshnav_runtime", executable="meshnav_navigator",
            name="meshnav_navigator", output="screen",
            parameters=[bundle["navigator"]],
        ),
    ]
    if rviz_config:
        actions.append(Node(
            package="rviz2", executable="rviz2", name="meshnav_real_rviz",
            output="screen", parameters=[{"use_sim_time": False}],
            arguments=["-d", rviz_config, "-f", bundle["robot"]["frames"]["map"]],
            remappings=[
                ("/rviz/goal_pose", bundle["navigator"]["goal_topic"]),
                ("/cloud", bundle["gateway"]["cloud_topic"]),
                ("/dynamic_obstacles", bundle["gateway"]["obstacle_topic"]),
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bundle_dir", default_value="",
            description="External prepared directory with four YAML files and frozen PLY/PCD assets",
        ),
        DeclareLaunchArgument(
            "cache_dir", default_value="",
            description="Writable H5 directory; empty uses ROS_HOME/meshnav/cache",
        ),
        DeclareLaunchArgument(
            "start_rviz", default_value="false", choices=["true", "false"],
            description="Optional desktop UI; default runtime remains headless",
        ),
        OpaqueFunction(function=_nodes),
    ])
