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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "mesh_map_path",
            description="Path to the mesh file that defines the map."
            "Allowed formats are our internal HDF5 format and all"
            "standard mesh formats loadable by Assimp.",
        ),
        DeclareLaunchArgument(
            "mesh_map_working_path",
            description="Path to the mesh file used by the mesh navigation "
            "to store costs during operation. Only HDF5 formats are permitted.",
        ),
        DeclareLaunchArgument(
            "mesh_controller_holonomic",
            default_value="false",
            choices=["true", "false"],
            description="Enable body-frame linear.y commands in MeshController.",
        ),
        DeclareLaunchArgument(
            "height_diff_threshold",
            default_value="0.2",
            description="HeightDiffLayer lethal threshold.",
        ),
        DeclareLaunchArgument(
            "static_inflation_radius",
            default_value="1.5",
            description="Static map border inflation radius in metres.",
        ),
        DeclareLaunchArgument(
            "static_inscribed_radius",
            default_value="0.4",
            description="Static map border inscribed radius in metres.",
        ),
        DeclareLaunchArgument(
            "obstacle_robot_height",
            default_value="0.7",
            description="Robot height used by the dynamic obstacle layer.",
        ),
        DeclareLaunchArgument(
            "obstacle_inflation_radius",
            default_value="1.5",
            description="Dynamic obstacle inflation radius in metres.",
        ),
        DeclareLaunchArgument(
            "obstacle_inscribed_radius",
            default_value="0.4",
            description="Dynamic obstacle inscribed radius in metres.",
        ),
    ]
    mesh_map_path = LaunchConfiguration("mesh_map_path")
    mesh_map_working_path = LaunchConfiguration("mesh_map_working_path")

    mbf_mesh_nav_config = os.path.join(
        get_package_share_directory("mesh_navigation_tutorials"), "config", "mbf_mesh_nav.yaml"
    )

    mesh_nav_server = Node(
        name="move_base_flex",
        package="mbf_mesh_nav",
        executable="mbf_mesh_nav",
        remappings=[
            ("/move_base_flex/cmd_vel", "/cmd_vel"),
        ],
        parameters=[
            mbf_mesh_nav_config,
            {
                "mesh_map.mesh_file": mesh_map_path,
                "mesh_map.mesh_working_file": mesh_map_working_path,
                "mesh_controller.holonomic": ParameterValue(
                    LaunchConfiguration("mesh_controller_holonomic"),
                    value_type=bool,
                ),
                "mesh_map.height_diff.threshold": ParameterValue(
                    LaunchConfiguration("height_diff_threshold"), value_type=float
                ),
                "mesh_map.static_inflation.inflation_radius": ParameterValue(
                    LaunchConfiguration("static_inflation_radius"), value_type=float
                ),
                "mesh_map.static_inflation.inscribed_radius": ParameterValue(
                    LaunchConfiguration("static_inscribed_radius"), value_type=float
                ),
                "mesh_map.obstacle.robot_height": ParameterValue(
                    LaunchConfiguration("obstacle_robot_height"), value_type=float
                ),
                "mesh_map.obstacle_inflation.inflation_radius": ParameterValue(
                    LaunchConfiguration("obstacle_inflation_radius"), value_type=float
                ),
                "mesh_map.obstacle_inflation.inscribed_radius": ParameterValue(
                    LaunchConfiguration("obstacle_inscribed_radius"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        launch_args
        + [
            mesh_nav_server,
        ]
    )
