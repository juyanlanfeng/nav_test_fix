# 正常运行 MeshNav，或采用“仿真、导航分两个终端”

"""Run MeshNav with the PB2025 vehicle and one selected velocity source."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
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

    obstacle_segmentation = str(profile["obstacle_segmentation"])

    start_rviz = str(profile["start_rviz"])
    start_sim = str(profile["start_sim"])

    spawn_rendering_sensors = str(profile["spawn_rendering_sensors"])
    startup_timeout_s = str(profile["startup_timeout_s"])

    adapter_launch = PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "launch", "pb_vehicle_sim.launch.py"])
    mesh_launch = PathJoinSubstitution([FindPackageShare("mesh_navigation_tutorials"), "launch", "mbf_mesh_navigation_server_launch.py"])

    pkg_mesh_navigation_tutorials = get_package_share_directory("mesh_navigation_tutorials")

    simulation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(adapter_launch),
                condition=IfCondition(start_sim),
            )
        ],
        scoped=True,
    )

    # RMCL：网格分割（以及可选的 MICP-L 定位）节点。
    # 参数在这里显式接线 —— 之前只声明了 obstacle_segmentation 却没往 include 里传，
    # 于是 rmcl_launch.py 用的是它自己的默认值 none，三个节点一个都没起来，
    # /obstacle_points 的发布者数一直是 0，“动态避障”其实从未生效。
    rmcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mesh_navigation_tutorials, "launch", "rmcl_launch.py"]
            )
        ),
        launch_arguments={
            "localization": "ground_truth",
            "obstacle_segmentation": obstacle_segmentation,
        }.items(),
        # 没有渲染传感器就没有 /cloud，分割节点只会空等，直接不启动。
        condition=IfCondition(PythonExpression([
            '"', obstacle_segmentation, '" != "none"',
            ' and "', spawn_rendering_sensors, '" == "True"',
        ])),
    )

    source_switch = ExecuteProcess(
        cmd=["ros2", "param", "set", "/pb_cmd_vel_adapter", "control_source", "meshnav"],
        output="screen",
        condition=UnlessCondition(start_sim),
    )
    meshnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mesh_launch),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_meshnav", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "rviz", "pb_meshnav.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_rviz),
    )
    readiness = readiness_gate("meshnav", startup_timeout_s)
    return LaunchDescription( 
                             [simulation, 
                              rmcl,
                              source_switch, 
                              meshnav, 
                              rviz, 
                              readiness]
                            )
