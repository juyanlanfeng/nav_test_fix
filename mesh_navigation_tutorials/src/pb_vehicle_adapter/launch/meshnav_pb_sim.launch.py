"""Start PB simulation and MeshNav from their respective YAML files."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml

from pb_vehicle_adapter.profile import default_profile_path, load_profile


def generate_launch_description():
    profile_path = default_profile_path()
    params = load_profile(profile_path)

    pb_share = get_package_share_directory("pb_vehicle_adapter")
    sim_share = get_package_share_directory("mesh_navigation_tutorials_sim")
    mesh_share = get_package_share_directory("mesh_navigation_tutorials")
    gz_share = get_package_share_directory("ros_gz_sim")
    with open(os.path.join(mesh_share, "config", "mbf_mesh_nav.yaml"), encoding="utf-8") as stream:
        nav_config = yaml.safe_load(stream)
    nav_options = nav_config["meshnav_launch"]["ros__parameters"]
    nav_params = nav_config["move_base_flex"]["ros__parameters"]

    # Gazebo 模型、插件搜索路径。
    resource_path = [
        os.path.join(sim_share, "models"),
        ":",
        os.path.join(params["pb_robot_description_root"], "resource", "models"),
        ":",
        os.path.join(params["pb_resources_root"], "resource", "models"),
        ":",
        EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
    ]
    plugin_path = [
        params["pb_plugin_path"],
        ":",
        EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
    ]

    # Gazebo 世界。
    world_file = os.path.join(sim_share, "worlds", str(params["world_name"]) + ".sdf")
    gz_args = "-r " + world_file
    if str(params["start_gazebo_gui"]).lower() != "true":
        gz_args += " -s"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # 根据 YAML 选择带渲染传感器或不带渲染传感器的车辆模型。
    model_file = os.path.join(pb_share, "models", "pb_navigation_robot.sdf")
    model_script = os.path.join(pb_share, "tools", "reduced_robot_model.py")
    model_command = [FindExecutable(name="python3"), " ", model_script, " ", model_file]
    model_options = [
        " --diagnostics ", str(params["contact_diagnostics"]),
        " --drive ", str(params["drive_model"]),
    ]
    reduced_model = Command(model_command + model_options)
    full_model = Command(model_command + [" --rendering True"] + model_options)
    spawn_args = [
        "-name", "robot",
        "-x", str(params["spawn_x"]),
        "-y", str(params["spawn_y"]),
        "-z", str(params["spawn_z"]),
        "-Y", str(math.radians(float(params["spawn_yaw_deg"]))),
    ]
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pb_navigation_robot",
        output="screen",
        arguments=["-string", full_model] + spawn_args,
        parameters=[{"use_sim_time": params["use_sim_time"]}],
        condition=IfCondition(str(params["spawn_rendering_sensors"])),
    )
    spawn_without_sensors = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pb_navigation_robot",
        output="screen",
        arguments=["-string", reduced_model] + spawn_args,
        parameters=[{"use_sim_time": params["use_sim_time"]}],
        condition=UnlessCondition(str(params["spawn_rendering_sensors"])),
    )

    # 仿真时钟、机器人状态、里程计和速度适配器。
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pb_ros_gz_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(pb_share, "config", params["bridge_config_file"]),
        }],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="pb_robot_state_publisher",
        output="screen",
        remappings=[("robot_description", "/meshnav/robot_description")],
        parameters=[{
            "use_sim_time": params["use_sim_time"],
            "publish_frequency": params["robot_state_publish_frequency"],
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="cat"), " ",
                    os.path.join(pb_share, "urdf", "pb_navigation_robot.urdf"),
                ]),
                value_type=str,
            ),
        }],
    )
    truth = Node(
        package="pb_vehicle_adapter",
        executable="pb_ground_truth_adapter",
        name="pb_ground_truth_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": params["use_sim_time"],
            "truth_topic": params["truth_topic"],
            "health_topic": params["health_topic"],
            "base_to_chassis_xyz": [float(v) for v in params["base_to_chassis_xyz"]],
            "base_to_chassis_xyzw": [float(v) for v in params["base_to_chassis_xyzw"]],
        }],
    )
    velocity = Node(
        package="pb_vehicle_adapter",
        executable="pb_cmd_vel_adapter",
        name="pb_cmd_vel_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": params["use_sim_time"],
            "control_source": params["control_source"],
            "hold_enabled": params["hold_enabled"],
            "hold_pose_topic": params["hold_pose_topic"],
            "hold_gain": params["hold_gain"],
            "hold_max_linear_velocity": params["hold_max_linear_velocity"],
            "hold_max_angular_velocity": params["hold_max_angular_velocity"],
            "hold_angular_deadband": params["hold_angular_deadband"],
        }],
    )

    # start_sim=False 时沿用已运行的仿真，只切换底盘的速度来源。
    simulation = GroupAction(
        actions=[
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", plugin_path),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
            gazebo,
            bridge,
            spawn,
            spawn_without_sensors,
            state_publisher,
            truth,
            velocity,
        ],
        condition=IfCondition(str(params["start_sim"])),
        scoped=True,
    )
    source_switch = ExecuteProcess(
        cmd=[
            "ros2", "param", "set", "/pb_cmd_vel_adapter", "control_source",
            str(params["control_source"]),
        ],
        output="screen",
        condition=UnlessCondition(str(params["start_sim"])),
    )

    # 导航启动选项和 MBF 节点参数统一来自 mbf_mesh_nav.yaml。
    need_rmcl = (
        str(params["spawn_rendering_sensors"]).lower() == "true"
        and (nav_options["localization"] == "rmcl_micpl"
             or nav_options["obstacle_segmentation"] == "rmcl_seg")
    )
    rmcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mesh_share, "launch", "rmcl_launch.py")),
        condition=IfCondition(str(need_rmcl)),
    )
    meshnav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mesh_share, "launch", "mbf_mesh_navigation_server_launch.py",
        )),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_meshnav",
        output="screen",
        arguments=["-d", os.path.join(pb_share, "rviz", nav_options["meshnav_rviz_config"])],
        parameters=[{"use_sim_time": nav_params["use_sim_time"]}],
        condition=IfCondition(str(nav_options["start_rviz"])),
    )

    # 启动 2 秒后运行就绪检查；检查不会阻塞上面的节点启动。
    startup_check = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "pb_vehicle_adapter", "pb_preflight",
                "--framework", "meshnav",
                "--wait-timeout", str(nav_options["startup_timeout_s"]),
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        simulation,
        rmcl,
        source_switch,
        meshnav,
        rviz,
        startup_check,
    ])
