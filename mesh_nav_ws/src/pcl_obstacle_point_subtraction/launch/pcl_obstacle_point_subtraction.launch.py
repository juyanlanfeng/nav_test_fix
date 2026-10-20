import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pcl_obstacle_point_subtraction')
    params_file = os.path.join(pkg_share, 'config', 'pcl_obstacle_point_subtraction.yaml')

    # 所有节点配置均在 YAML 中维护，launch 只提供参数文件路径。
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Path to the PCL obstacle subtraction parameters file',
    )

    tf_publisher_node = Node(
        package='pcl_obstacle_point_subtraction',
        executable='cloud_tf_publisher',
        name='cloud_tf_publisher',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    obstacle_sub_node = Node(
        package='pcl_obstacle_point_subtraction',
        executable='pcl_obstacle_point_subtraction',
        name='obstacle_sub',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        declare_params_file,
        tf_publisher_node,
        obstacle_sub_node,
    ])
