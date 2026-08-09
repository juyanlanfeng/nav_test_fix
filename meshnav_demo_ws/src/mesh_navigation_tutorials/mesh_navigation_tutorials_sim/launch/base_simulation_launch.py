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


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # path to this pkg
    pkg_mesh_navigation_tutorials_sim = get_package_share_directory(
        "mesh_navigation_tutorials_sim"
    )

    # Launch arguments
    launch_args = [DeclareLaunchArgument(
            "world_pkg",
            description="Package of the world to simulate",
        ),
        DeclareLaunchArgument(
            "world_name",
            description="Name of the world to simulate"
            + '(see mesh_navigation_tutorials\' "worlds" directory).'
        ),
        DeclareLaunchArgument(
            "start_gazebo_gui",
            description="Start Gazebo GUI",
            default_value="True",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "laser3d_collision",
            description="Include the tall 3-D lidar mast in Gazebo collision geometry",
            default_value="True",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "slope_aware_drive",
            description=(
                "Use the RMUC slope-aware holonomic Gazebo drive instead of "
                "the original differential-drive wheel plugin"
            ),
            default_value="False",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "robot_wheel_radius",
            description="Wheel radius used by the robot xacro",
            default_value="0.125",
        ),
        DeclareLaunchArgument(
            "laser2d_mount_z",
            description="2-D lidar height relative to base_link",
            default_value="0.125",
        ),
        DeclareLaunchArgument(
            "laser3d_mount_z",
            description="3-D lidar height relative to base_link",
            default_value="0.475",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            description="Robot spawn X coordinate in the Gazebo world frame",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            description="Robot spawn Y coordinate in the Gazebo world frame",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            description="Robot spawn Z coordinate in the Gazebo world frame",
            default_value="0.1",
        ),
    ]

    world_pkg = FindPackageShare(LaunchConfiguration("world_pkg"))
    world_name = LaunchConfiguration("world_name")
    world_path = PathJoinSubstitution(
        [
            world_pkg,
            "worlds",
            PythonExpression(['"', world_name, '" + ".sdf"']),
        ]
    )
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_mesh_navigation_tutorials_sim, "urdf/ceres.urdf.xacro"]),
            " name:=robot",
            " prefix:='robot'",
            " is_sim:=true",
            " laser3d_collision:=",
            LaunchConfiguration("laser3d_collision"),
            " slope_aware_drive:=",
            LaunchConfiguration("slope_aware_drive"),
            " wheel_radius:=",
            LaunchConfiguration("robot_wheel_radius"),
            " laser2d_mount_z:=",
            LaunchConfiguration("laser2d_mount_z"),
            " laser3d_mount_z:=",
            LaunchConfiguration("laser3d_mount_z"),
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="meshnav_robot_state_publisher",
        output="screen",
        # Do not publish on the global /robot_description topic.  Other robot
        # stacks (for example SCAN-Planner's Go2 simulation) may leave a
        # transient-local publisher there; ros_gz_sim/create would then spawn
        # whichever retained URDF it receives first.
        remappings=[
            ("robot_description", "/meshnav/robot_description"),
        ],
        parameters=[
            {
                "use_sim_time": True,
                "publish_frequency": 100.0,
                "robot_description": robot_description,
            }
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_path,  # which world to load
                PythonExpression(
                    ['"" if ', start_gazebo_gui, ' else " -s"']
                ),  # whether to start gui
            ]
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-topic",
            "/meshnav/robot_description",
            "-name",
            # The robot's name in simulation is always "robot", regardless of which model is chosen
            # This facilitates easier topic bridging.
            "robot",
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_mesh_navigation_tutorials_sim, "config", "ros_gazebo_bridge.yaml"]
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(launch_args + [gz_sim, spawn_robot, bridge, robot_state_publisher])
