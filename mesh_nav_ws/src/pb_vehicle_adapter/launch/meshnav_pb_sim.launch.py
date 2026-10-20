"""实车 MeshNav 入口：MBF、导航协调器、速度接口和可选 RViz。

保留历史文件名。所有节点强制使用系统时间；定位、TF、雷达、点云差分及
底盘驱动需另行启动。本文件不会启动 Gazebo 或发布机器人真值。
"""

import atexit
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError(f"配置文件必须是 YAML 字典：{path}")
    return config


def _parameters(config, name):
    params = config.get(name, {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        raise ValueError(f"{name}.ros__parameters 必须是字典")
    return params


def _boolean(value):
    if isinstance(value, bool):
        return value
    if str(value).lower() in ("true", "1"):
        return True
    if str(value).lower() in ("false", "0"):
        return False
    raise ValueError(f"start_rviz 必须为 true 或 false：{value}")


def _topic(display, key, value):
    """保留 RViz 话题字段的 QoS 配置，仅替换话题名。"""
    if isinstance(display.get(key), dict):
        display[key]["Value"] = value
    else:
        display[key] = value


def _rviz_file(source, global_frame, robot_frame, navigator, view):
    """生成临时视图，使接口配置生效；不修改仓库中的 .rviz 文件。"""
    config = _load_yaml(source)
    manager = config["Visualization Manager"]
    manager.setdefault("Global Options", {})["Fixed Frame"] = global_frame
    current_view = manager.get("Views", {}).get("Current", {})
    if current_view.get("Target Frame") == "base_footprint":
        current_view["Target Frame"] = robot_frame
    displays = manager.setdefault("Displays", [])
    obstacle_found = False

    def update(items):
        nonlocal obstacle_found
        for display in items:
            kind = display.get("Class")
            if kind == "rviz_default_plugins/Grid":
                display["Reference Frame"] = global_frame
            elif kind == "rviz_default_plugins/RobotModel":
                _topic(display, "Description Topic", view["robot_description_topic"])
            elif kind == "rviz_default_plugins/PointCloud2":
                topic = display.get("Topic", {})
                name = topic.get("Value") if isinstance(topic, dict) else topic
                if name == "/dynamic_obstacles":
                    _topic(display, "Topic", view["obstacle_topic"])
                    obstacle_found = True
                elif name in ("/cloud", "/cloud_registered"):
                    _topic(display, "Topic", view["cloud_topic"])
            update(display.get("Displays", []))

    update(displays)
    if not obstacle_found:
        displays.append({
            "Class": "rviz_default_plugins/PointCloud2",
            "Name": "Dynamic Obstacles", "Enabled": True,
            "Color Transformer": "FlatColor", "Color": "255; 80; 30",
            "Position Transformer": "XYZ", "Style": "Points", "Size (Pixels)": 3,
            "Topic": {"Value": view["obstacle_topic"], "Depth": 1,
                      "Reliability Policy": "Best Effort", "Durability Policy": "Volatile",
                      "History Policy": "Keep Last"},
        })
    for tool in manager.get("Tools", []):
        if tool.get("Class") in ("rviz_mesh_tools_plugins/MeshGoal",
                                  "rviz_default_plugins/SetGoal"):
            _topic(tool, "Topic", navigator["goal_topic"])

    # 临时文件保持到 RViz 退出；正常 launch 关闭及解释器退出均会清理。
    temporary = tempfile.TemporaryDirectory(prefix="meshnav_rviz_")
    atexit.register(temporary.cleanup)
    path = os.path.join(temporary.name, "navigation.rviz")
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
    return path, temporary


def _create_nodes(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    nav_file = argument("navigation_params_file")
    interfaces_file = argument("interfaces_file")
    config = _load_yaml(nav_file)
    interfaces = _load_yaml(interfaces_file)
    nav_params = _parameters(config, "move_base_flex")
    nav_options = _parameters(config, "meshnav_launch")

    # MBF：完整导航 YAML 提供地图、地图层、规划器及控制插件；接口 YAML
    # 随后覆盖 frame/odom 等接线参数；系统时间覆盖任何遗留 use_sim_time。
    mbf_params = dict(nav_params)
    mbf_params.update(_parameters(interfaces, "move_base_flex"))
    navigator = dict(_parameters(interfaces, "meshnav_navigator"))
    adapter = _parameters(interfaces, "pb_cmd_vel_adapter")
    for group, selection in (("planners", "planner"), ("controllers", "controller")):
        loaded = mbf_params.get(group, [])
        if not loaded:
            raise ValueError(f"move_base_flex.{group} 至少需要一个插件实例")
        navigator.setdefault(selection, loaded[0])
        if navigator[selection] not in loaded:
            raise ValueError(f"协调器的 {selection} '{navigator[selection]}' 未在 MBF {group} 中配置")
    navigator.setdefault("goal_topic", "/rviz/goal_pose")
    navigator.setdefault("get_path_action", "/move_base_flex/get_path")
    navigator.setdefault("exe_path_action", "/move_base_flex/exe_path")
    navigator.setdefault("check_path_service", "/move_base_flex/check_path_cost")

    robot_frame = mbf_params.get("robot_frame", "base_footprint")
    if robot_frame.lstrip("/") != adapter.get("base_frame", "base_footprint").lstrip("/"):
        raise ValueError("move_base_flex.robot_frame 必须与 pb_cmd_vel_adapter.base_frame 一致")
    overrides = {"use_sim_time": False}
    # 可在部署时覆盖地图和可写的 HDF5 缓存路径，无需修改导航 YAML。
    for name, parameter in (("mesh_map_path", "mesh_map.mesh_file"),
                            ("mesh_map_working_path", "mesh_map.mesh_working_file")):
        if argument(name):
            overrides[parameter] = argument(name)

    meshnav = Node(
        package="mbf_mesh_nav", executable="mbf_mesh_nav", name="move_base_flex",
        output="screen", parameters=[nav_file, interfaces_file, overrides],
        remappings=[
            ("/move_base_flex/cmd_vel", adapter.get("input_topic", "/cmd_vel")),
            ("/move_base_flex/get_path", navigator["get_path_action"]),
            ("/move_base_flex/exe_path", navigator["exe_path_action"]),
            ("/move_base_flex/check_path_cost", navigator["check_path_service"]),
        ],
    )
    coordinator = Node(
        package="pb_vehicle_adapter", executable="meshnav_navigator", name="meshnav_navigator",
        output="screen", parameters=[interfaces_file, navigator, {"use_sim_time": False}],
    )
    velocity = Node(
        package="pb_vehicle_adapter", executable="pb_cmd_vel_adapter", name="pb_cmd_vel_adapter",
        output="screen", parameters=[interfaces_file, {"use_sim_time": False}],
    )
    nodes = [meshnav, coordinator, velocity]
    start_rviz = argument("start_rviz") or nav_options.get("start_rviz", True)
    if _boolean(start_rviz):
        pb_share = get_package_share_directory("pb_vehicle_adapter")
        source = argument("rviz_config") or os.path.join(
            pb_share, "rviz", nav_options.get("meshnav_rviz_config", "pb_meshnav.rviz"))
        view = {"cloud_topic": "/cloud_registered", "obstacle_topic": "/dynamic_obstacles",
                "robot_description_topic": "/robot_description"}
        view.update(_parameters(interfaces, "meshnav_rviz"))
        path, temporary = _rviz_file(
            source, mbf_params.get("global_frame", "map"), robot_frame, navigator, view)
        nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup()),
        ])))
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="rviz2_meshnav", output="screen",
            arguments=["-d", path], parameters=[{"use_sim_time": False}],
        ))
    # 进程异步启动。外部 TF、定位和障碍点云是否就绪仍需单独确认。
    return nodes


def generate_launch_description():
    pb_share = get_package_share_directory("pb_vehicle_adapter")
    mesh_share = get_package_share_directory("mesh_navigation_tutorials")
    return LaunchDescription([
        DeclareLaunchArgument("navigation_params_file", default_value=os.path.join(
            mesh_share, "config", "mbf_mesh_nav.yaml"), description="完整 MeshNav 导航参数文件"),
        DeclareLaunchArgument("interfaces_file", default_value=os.path.join(
            pb_share, "config", "interfaces.yaml"), description="实车接口参数文件，最后覆盖导航接口"),
        DeclareLaunchArgument("mesh_map_path", default_value="", description="PLY 地图；空值沿用导航 YAML"),
        DeclareLaunchArgument("mesh_map_working_path", default_value="",
                              description="可写 HDF5 缓存；空值沿用导航 YAML"),
        DeclareLaunchArgument("start_rviz", default_value="",
                              description="true/false；空值沿用导航 YAML"),
        DeclareLaunchArgument("rviz_config", default_value="", description="RViz 模板；空值使用包内配置"),
        OpaqueFunction(function=_create_nodes),
    ])
