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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # path to this pkg

    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )

    # find all worlds available
    available_world_names = [
        f[:-4]
        for f in os.listdir(os.path.join(pkg_mesh_navigation_tutorials_sim, "worlds"))
        if f.endswith(".sdf")
    ]

    # Launch arguments (override/specialize base arguments)
    launch_args = [
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see mesh_navigation_tutorials\' "worlds" directory).',
            default_value=available_world_names[0],
            choices=available_world_names,
        ),
        DeclareLaunchArgument(
            "laser3d_collision",
            description=(
                "Include the tall 3-D lidar mast in Gazebo collisions. "
                "RMUC2026 defaults to the validated low-profile body."
            ),
            default_value=PythonExpression(
                ['"False" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "True"']
            ),
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "slope_aware_drive",
            description=(
                "Use slope-aware planar body commands. RMUC2026 enables this; "
                "legacy worlds retain their wheel-driven differential drive."
            ),
            default_value=PythonExpression(
                ['"True" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "False"']
            ),
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "robot_wheel_radius",
            description="Wheel radius used by both visual and collision geometry",
            default_value=PythonExpression(
                ['"0.10" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.125"']
            ),
        ),
        DeclareLaunchArgument(
            "laser2d_mount_z",
            description="2-D lidar height relative to base_link",
            default_value=PythonExpression(
                ['"0.060" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.125"']
            ),
        ),
        DeclareLaunchArgument(
            "laser3d_mount_z",
            description="3-D lidar height relative to base_link",
            default_value=PythonExpression(
                ['"0.05" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.475"']
            ),
        ),
        DeclareLaunchArgument(
            "spawn_x",
            description="Robot spawn X coordinate; defaults to an open RMUC2026 start",
            default_value=PythonExpression(
                ['"-11.9" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.0"']
            ),
        ),
        DeclareLaunchArgument(
            "spawn_y",
            description="Robot spawn Y coordinate; defaults to an open RMUC2026 start",
            default_value=PythonExpression(
                ['"-4.4" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.0"']
            ),
        ),
        DeclareLaunchArgument(
            "spawn_z",
            description="Robot spawn Z coordinate",
            default_value=PythonExpression(
                ['"0.15" if "', LaunchConfiguration("world_name"),
                 '" == "rmuc2026_field" else "0.1"']
            ),
        ),
    ]

    meshnav_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mesh_navigation_tutorials_sim"),
                    "launch",
                    "base_simulation_launch.py",
                ]
            )
        ),
        launch_arguments={
            "world_pkg": "mesh_navigation_tutorials_sim",
            "world_name": LaunchConfiguration("world_name"),
            "laser3d_collision": LaunchConfiguration("laser3d_collision"),
            "slope_aware_drive": LaunchConfiguration("slope_aware_drive"),
            "robot_wheel_radius": LaunchConfiguration("robot_wheel_radius"),
            "laser2d_mount_z": LaunchConfiguration("laser2d_mount_z"),
            "laser3d_mount_z": LaunchConfiguration("laser3d_mount_z"),
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
        }.items(),
    )

    return LaunchDescription(launch_args + [meshnav_sim])
