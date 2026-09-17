"""Run JIE's existing PCD planner and controller with the PB2025 vehicle."""

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


def _profile():
    profile_path = os.path.join(
        get_package_share_directory("pb_vehicle_adapter"), "config", "pb_vehicle_profile.yaml"
    )
    with open(profile_path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]


def generate_launch_description():
    profile = _profile()
    vehicle_launch = PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "launch", "pb_vehicle_sim.launch.py"])
    controller_config = PathJoinSubstitution([FindPackageShare("octo_planner"), "config", "meshnav_ceres_controller.yaml"])
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
        DeclareLaunchArgument(
            "pcd_file", default_value="/home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd"
        ),
        # Publish the same PCD that is loaded into the OctoMap as a latched
        # PointCloud2 (/cloud_pcd).  RViz cannot show octomap_msgs/Octomap without
        # ros-humble-octomap-rviz-plugins, so this is what makes the map - floor,
        # walls and tunnel roofs - visible in the default JIE layout.
        DeclareLaunchArgument(
            "publish_map_cloud", default_value="True", choices=["True", "False"],
            description="Publish the static map PCD on /cloud_pcd for RViz.",
        ),
        DeclareLaunchArgument(
            "auto_start_navigation", default_value="False", choices=["True", "False"],
            description="RViz GOAL click also sends /start_navigation (one-click drive).",
        ),
        # Ground-only layer: field/build_jie_floor_pcd.py keeps the lowest surface of
        # every 4 cm cell below 0.25 m, i.e. floor, ramps and the 0.203 m platform but
        # not the tunnel roofs or the surrounding structures.  With only 22 % of the
        # full cloud being floor, this is the layer to look at when picking
        # start/goal points by hand.
        DeclareLaunchArgument(
            "floor_pcd_file",
            default_value="/home/rainple/nav_test/field/converted_rmuc2026/jie_nav/rmuc2026_field_floor.pcd",
        ),
        DeclareLaunchArgument(
            "publish_floor_cloud", default_value="True", choices=["True", "False"],
            description="Publish the ground-only PCD on /cloud_pcd_floor for RViz.",
        ),
        DeclareLaunchArgument("robot_radius_xy", default_value=str(profile["static_inscribed_radius"])),
        DeclareLaunchArgument("robot_height", default_value=str(profile["robot_height"])),
        DeclareLaunchArgument(
            "startup_timeout_s", default_value="120",
            description="Readiness-gate timeout for pb_preflight; 0 = check once.",
        ),
        DeclareLaunchArgument(
            "start_click_selector", default_value="True", choices=["True", "False"],
            description=(
                "False skips the RViz point-click start/goal helper. Scripted runs "
                "publish /start_point and /goal_point directly, and every skipped "
                "node frees a DDS participant on hosts with a low participant limit."
            ),
        ),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vehicle_launch),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui"),
            # This entry starts its own JIE RViz below.
            "start_rviz": "False",
            "control_source": "jie",
            "spawn_rendering_sensors": LaunchConfiguration("spawn_rendering_sensors"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_sim")),
    )
    source_switch = ExecuteProcess(
        cmd=["ros2", "param", "set", "/pb_cmd_vel_adapter", "control_source", "jie"],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("start_sim")),
    )
    pcd_to_octomap = Node(
        package="jie_octomap", executable="pcd_to_octomap_node", name="pcd_to_octomap", output="screen",
        parameters=[{"use_sim_time": True, "pcd_file": LaunchConfiguration("pcd_file"), "resolution": 0.04, "voxel_downsample_m": 0.0, "min_points_per_voxel": 1, "min_cluster_voxels": 1, "frame_id": "map", "octomap_topic": "/octomap"}],
    )
    # period 0.0 = publish once and latch, so a later RViz still receives the map.
    map_cloud = Node(
        package="pcl_ros", executable="pcd_to_pointcloud", name="pb_map_cloud", output="log",
        # use_sim_time stays False on purpose: with period 0.0 the publish happens
        # from a timer, and a ROS timer never fires while /clock is absent or
        # frozen, which would leave the map invisible.
        parameters=[{"use_sim_time": False, "file_name": LaunchConfiguration("pcd_file"),
                     "period": 0.0, "tf_frame": "map"}],
        remappings=[("cloud_pcd", "/cloud_pcd")],
        condition=IfCondition(LaunchConfiguration("publish_map_cloud")),
    )
    floor_cloud = Node(
        package="pcl_ros", executable="pcd_to_pointcloud", name="pb_floor_cloud", output="log",
        parameters=[{"use_sim_time": False, "file_name": LaunchConfiguration("floor_pcd_file"),
                     "period": 0.0, "tf_frame": "map"}],
        remappings=[("cloud_pcd", "/cloud_pcd_floor")],
        condition=IfCondition(LaunchConfiguration("publish_floor_cloud")),
    )
    planner = Node(
        package="octo_planner", executable="jie_path_node", name="jie_path_node", output="screen",
        parameters=[{"use_sim_time": True, "octomap_topic": "/octomap", "start_topic": "/start_point", "goal_topic": "/goal_point", "path_topic": "/planned_path", "frame_id": "map", "robot_radius_xy": LaunchConfiguration("robot_radius_xy"), "robot_height": LaunchConfiguration("robot_height"), "require_ground_support": True, "strict_direct_ground_support": False}],
    )
    controller = Node(
        package="octo_planner", executable="d1_controller", name="d1_controller", output="screen",
        parameters=[controller_config, {"use_sim_time": True, "path_topic": "/planned_path", "start_navigation_topic": "/start_navigation", "stop_navigation_topic": "/stop_navigation", "cmd_vel_topic": "/cmd_vel_jie", "map_frame": "map", "base_frame": "base_footprint", "base_frame_candidates": "base_footprint,base_link,chassis", "robot_center_offset_frame": "base_footprint", "robot_center_offset_x": 0.0, "robot_center_offset_y": 0.0, "robot_center_offset_z": 0.0, "enable_lateral_motion": True}],
    )
    stamper = Node(
        package="octo_planner", executable="jie_twist_stamper", name="pb_jie_twist_stamper", output="screen",
        parameters=[{"use_sim_time": True, "input_topic": "/cmd_vel_jie", "output_topic": "/pb/jie_cmd_vel_stamped", "frame_id": "base_footprint"}],
    )
    # RViz "Publish Point" clicks alternate between the JIE start and goal; the
    # selector publishes /start_point, /goal_point and /selection_markers, which
    # the planner subscribes to and the RViz config displays.
    # auto_start_navigation:=True makes a RViz GOAL click drive the robot: the
    # selector sends /start_navigation once the planner's /planned_path arrives
    # (d1_controller drops a start command that precedes the path).
    click_selector = Node(
        package="jie_octomap", executable="rviz_click_selector_node", name="pb_rviz_click_selector", output="screen",
        parameters=[{"use_sim_time": True, "clicked_topic": "/clicked_point", "marker_topic": "/selection_markers", "start_topic": "/start_point", "goal_topic": "/goal_point", "auto_start_navigation": LaunchConfiguration("auto_start_navigation"), "path_topic": "/planned_path", "start_navigation_topic": "/start_navigation"}],
        condition=IfCondition(LaunchConfiguration("start_click_selector")),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_jie", output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("pb_vehicle_adapter"), "rviz", "pb_jie.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    readiness = readiness_gate("jie", LaunchConfiguration("startup_timeout_s"))
    return LaunchDescription(arguments + [simulation, source_switch, pcd_to_octomap, map_cloud, floor_cloud, planner, controller, stamper, click_selector, rviz, readiness])
