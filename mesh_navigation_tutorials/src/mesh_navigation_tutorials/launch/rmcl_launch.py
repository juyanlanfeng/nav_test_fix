# Copyright 2024 Nature Robots GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Nature Robots GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _create_actions(context):
    mesh_share = get_package_share_directory("mesh_navigation_tutorials")
    rmcl_params_file = os.path.join(mesh_share, "config", "rmcl.yaml")
    with open(os.path.join(mesh_share, "config", "mbf_mesh_nav.yaml"), encoding="utf-8") as stream:
        nav_config = yaml.safe_load(stream)
    nav_options = nav_config["meshnav_launch"]["ros__parameters"]
    mbf_params = nav_config["move_base_flex"]["ros__parameters"]

    requested_map = LaunchConfiguration("map_name").perform(context)
    map_name = requested_map or nav_options["map_name"]
    localization = LaunchConfiguration("localization").perform(context)
    localization = localization or nav_options["localization"]
    segmentation = LaunchConfiguration("obstacle_segmentation").perform(context)
    segmentation = segmentation or nav_options["obstacle_segmentation"]

    if not map_name:
        raise ValueError("RMCL requires map_name or meshnav_launch.map_name")
    if localization not in ("ground_truth", "rmcl_micpl"):
        raise ValueError("Unsupported localization: " + localization)
    if segmentation not in ("none", "ground_truth", "rmcl_seg"):
        raise ValueError("Unsupported obstacle_segmentation: " + segmentation)

    # 显式指定 map_name 时使用该地图；否则读取 MBF YAML 中的网格路径。
    map_file = os.path.join(mesh_share, "maps", map_name + ".ply")
    if not requested_map:
        map_file = mbf_params["mesh_map"]["mesh_file"]
    use_sim_time = mbf_params["use_sim_time"]

    conversion = Node(
        package="rmcl_ros",
        executable="conv_pc2_to_o1dn_node",
        name="rmcl_lidar3d_conversion",
        output="screen",
        remappings=[("input", "/cloud"), ("output", "/rmcl_inputs/cloud")],
        parameters=[rmcl_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(str(localization == "rmcl_micpl" or segmentation == "rmcl_seg")),
    )
    micp_localization = Node(
        package="rmcl_ros",
        executable="micp_localization_node",
        name="rmcl_micpl",
        output="screen",
        parameters=[rmcl_params_file, {
            "use_sim_time": use_sim_time,
            "map_file": map_file,
        }],
        condition=IfCondition(str(localization == "rmcl_micpl")),
    )
    obstacle_segmentation = Node(
        package="rmcl_ros",
        executable="o1dn_map_segmentation_embree_node",
        name="rmcl_seg",
        output="screen",
        remappings=[
            ("scan", "/rmcl_inputs/cloud"),
            ("outlier_map", "~/outlier_map"),
            ("outlier_scan", "obstacle_points"),
        ],
        parameters=[rmcl_params_file, {
            "use_sim_time": use_sim_time,
            "map_file": map_file,
        }],
        condition=IfCondition(str(segmentation == "rmcl_seg")),
    )
    return [conversion, micp_localization, obstacle_segmentation]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_name", default_value=LaunchConfiguration("world_name", default=""),
            description="Map name; empty uses mbf_mesh_nav.yaml",
        ),
        DeclareLaunchArgument(
            "localization", default_value="",
            description="Localization mode; empty uses YAML or ground_truth",
        ),
        DeclareLaunchArgument(
            "obstacle_segmentation", default_value="",
            description="Segmentation mode; empty uses YAML or none",
        ),
        OpaqueFunction(function=_create_actions),
    ])
