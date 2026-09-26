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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _create_actions(context):
    mesh_share = get_package_share_directory("mesh_navigation_tutorials")
    config_file = os.path.join(mesh_share, "config", "mbf_mesh_nav.yaml")
    with open(config_file, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["move_base_flex"]["ros__parameters"]
    parameter_files = [config_file]
    if "pb_terminal_controller" in params["controllers"]:
        pb_share = get_package_share_directory("pb_vehicle_adapter")
        parameter_files.append(os.path.join(pb_share, "config", "pb_terminal_controller.yaml"))

    # 通用教程可为所选地图覆盖 YAML 中的 PB 默认路径。
    overrides = {}
    map_file = LaunchConfiguration("mesh_map_path").perform(context)
    working_file = LaunchConfiguration("mesh_map_working_path").perform(context)
    if map_file:
        overrides["mesh_map.mesh_file"] = map_file
    if working_file:
        overrides["mesh_map.mesh_working_file"] = working_file

    meshnav = Node(
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        name="move_base_flex",
        remappings=[("/move_base_flex/cmd_vel", "/cmd_vel")],
        parameters=parameter_files + ([overrides] if overrides else []),
    )
    return [meshnav]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mesh_map_path", default_value="",
            description="Mesh map path; empty uses mbf_mesh_nav.yaml",
        ),
        DeclareLaunchArgument(
            "mesh_map_working_path", default_value="",
            description="HDF5 working path; empty uses mbf_mesh_nav.yaml",
        ),
        OpaqueFunction(function=_create_actions),
    ])
