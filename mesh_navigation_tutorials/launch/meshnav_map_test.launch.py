"""Start MeshNav and RViz for map/path inspection without simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory("mesh_navigation_tutorials")
    arguments = [
        DeclareLaunchArgument("mesh_map_path", description="Input triangle mesh (PLY)"),
        DeclareLaunchArgument("mesh_map_working_path", description="Writable MeshNav H5 cache"),
        DeclareLaunchArgument("source_pcd_path", default_value="",
                              description="Original PCD for 3D scene display"),
        DeclareLaunchArgument("publish_source_cloud", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("source_cloud_scale", default_value="1.0"),
        DeclareLaunchArgument("source_cloud_translate_x", default_value="0.0"),
        DeclareLaunchArgument("source_cloud_translate_y", default_value="0.0"),
        DeclareLaunchArgument("source_cloud_translate_z", default_value="0.0"),
        DeclareLaunchArgument("start_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("height_diff_threshold", default_value="0.2"),
        DeclareLaunchArgument("static_inflation_radius", default_value="0.7"),
        DeclareLaunchArgument("static_inscribed_radius", default_value="0.22"),
    ]
    server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        output="screen",
        remappings=[("/move_base_flex/cmd_vel", "/cmd_vel")],
        parameters=[
            os.path.join(package, "config", "mbf_mesh_nav.yaml"),
            {
                "use_sim_time": False,
                "mesh_map.mesh_file": LaunchConfiguration("mesh_map_path"),
                "mesh_map.mesh_working_file": LaunchConfiguration("mesh_map_working_path"),
                "mesh_map.height_diff.threshold": ParameterValue(
                    LaunchConfiguration("height_diff_threshold"), value_type=float
                ),
                "mesh_map.static_inflation.inflation_radius": ParameterValue(
                    LaunchConfiguration("static_inflation_radius"), value_type=float
                ),
                "mesh_map.static_inflation.inscribed_radius": ParameterValue(
                    LaunchConfiguration("static_inscribed_radius"), value_type=float
                ),
            },
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_meshnav_map_test",
        output="screen",
        arguments=["-d", package + "/rviz/meshnav_map_test.rviz"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    cloud = Node(
        package="mesh_navigation_tutorials",
        executable="meshnav_source_cloud",
        name="meshnav_source_cloud",
        output="screen",
        arguments=["--pcd", LaunchConfiguration("source_pcd_path"),
                   "--scale", LaunchConfiguration("source_cloud_scale"),
                   "--translate", LaunchConfiguration("source_cloud_translate_x"),
                   LaunchConfiguration("source_cloud_translate_y"),
                   LaunchConfiguration("source_cloud_translate_z")],
        condition=IfCondition(LaunchConfiguration("publish_source_cloud")),
    )
    return LaunchDescription(arguments + [server, cloud, rviz])
