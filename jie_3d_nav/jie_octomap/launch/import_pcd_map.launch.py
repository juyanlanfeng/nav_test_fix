from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_import_gui = LaunchConfiguration("start_import_gui")
    rmuc2026_profile = LaunchConfiguration("rmuc2026_profile")
    resolution = LaunchConfiguration("resolution")
    voxel_downsample_m = LaunchConfiguration("voxel_downsample_m")
    min_points_per_voxel = LaunchConfiguration("min_points_per_voxel")
    min_cluster_voxels = LaunchConfiguration("min_cluster_voxels")
    robot_radius_xy = LaunchConfiguration("robot_radius_xy")
    robot_height = LaunchConfiguration("robot_height")

    pcd_to_octomap_node = Node(
        package="jie_octomap",
        executable="pcd_to_octomap_node",
        name="pcd_to_octomap",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "pcd_file_cmd_topic": "/pcd_file_cmd",
                "octomap_topic": "/octomap",
                "frame_id": "map",
                "resolution": ParameterValue(resolution, value_type=float),
                "voxel_downsample_m": ParameterValue(voxel_downsample_m, value_type=float),
                "min_points_per_voxel": ParameterValue(
                    min_points_per_voxel, value_type=int
                ),
                "min_cluster_voxels": ParameterValue(min_cluster_voxels, value_type=int),
            }
        ],
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "imported_pcd_map",
                "source_world_file": "",
                "robot_radius": 0.25,
                "robot_radius_xy": ParameterValue(robot_radius_xy, value_type=float),
                "robot_height": ParameterValue(robot_height, value_type=float),
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": False,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 1,
                "enable_preblocked_costmap": True,
                "preblocked_costmap_radius_cells": 3,
                "preblocked_costmap_weight": 2.5,
            }
        ],
    )

    occupied_marker_node = Node(
        package="jie_octomap",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    map_package_manager_node = Node(
        package="jie_octomap",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
        ],
    )

    importer_gui_node = Node(
        package="jie_octomap",
        executable="pcd_map_import_gui",
        name="pcd_map_import_gui",
        output="screen",
        condition=IfCondition(start_import_gui),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "rmuc2026_profile": ParameterValue(
                    rmuc2026_profile, value_type=bool
                ),
                "initial_resolution": ParameterValue(resolution, value_type=float),
                "initial_preprocess_downsample_m": ParameterValue(
                    PythonExpression(
                        [
                            "'0.0' if '",
                            rmuc2026_profile,
                            "' == 'true' else '0.1'",
                        ]
                    ),
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                choices=["true", "false"],
                description="Use Gazebo /clock for message timestamps.",
            ),
            DeclareLaunchArgument(
                "start_import_gui",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Start the PyQt/Open3D import window. Set false for a "
                    "headless launch and publish /pcd_file_cmd separately."
                ),
            ),
            DeclareLaunchArgument(
                "rmuc2026_profile",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Use the CAD-derived RMUC simulation proxy: 0.05 m OctoMap, "
                    "0.28 m XY radius and 0.225 m physical height."
                ),
            ),
            DeclareLaunchArgument(
                "resolution",
                default_value=PythonExpression(
                    ["'0.05' if '", rmuc2026_profile, "' == 'true' else '0.5'"]
                ),
                description="OctoMap resolution in meters for imported PCD maps.",
            ),
            DeclareLaunchArgument(
                "robot_radius_xy",
                default_value=PythonExpression(
                    ["'0.28' if '", rmuc2026_profile, "' == 'true' else '-1.0'"]
                ),
                description=(
                    "Horizontal collision radius in meters. The RMUC Ceres profile "
                    "uses 0.28 m (half of its approximately 0.55 m width plus rounding)."
                ),
            ),
            DeclareLaunchArgument(
                "robot_height",
                default_value=PythonExpression(
                    ["'0.225' if '", rmuc2026_profile, "' == 'true' else '-1.0'"]
                ),
                description=(
                    "Physical collision height from support/ground to robot top. "
                    "The geometry-derived RMUC Ceres simulation profile uses 0.225 m."
                ),
            ),
            DeclareLaunchArgument(
                "voxel_downsample_m",
                default_value="0.0",
                description=(
                    "Additional downsample size inside pcd_to_octomap_node. "
                    "The GUI already writes a preprocessed temporary PCD, "
                    "so 0.0 avoids double downsampling."
                ),
            ),
            DeclareLaunchArgument(
                "min_points_per_voxel",
                default_value="1",
                description="Minimum source points required to keep an occupied voxel.",
            ),
            DeclareLaunchArgument(
                "min_cluster_voxels",
                default_value="1",
                description="Minimum connected occupied voxels required to keep a cluster.",
            ),
            pcd_to_octomap_node,
            planner_node,
            occupied_marker_node,
            map_package_manager_node,
            importer_gui_node,
        ]
    )
