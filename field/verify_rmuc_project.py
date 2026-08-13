#!/usr/bin/env python3
"""Verify the canonical RMUC2026 assets and their ROS workspace copies."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import struct
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent.parent
FIELD = ROOT / "field" / "converted_rmuc2026"
MESH_WS = ROOT / "meshnav_demo_ws"
ROS_MAPS = (
    MESH_WS
    / "src"
    / "mesh_navigation_tutorials"
    / "mesh_navigation_tutorials"
    / "maps"
)
ROS_SIM = (
    MESH_WS
    / "src"
    / "mesh_navigation_tutorials"
    / "mesh_navigation_tutorials_sim"
)

EXPECTED = {
    "visual_sha256": "af971f460ffc9327f35344abeb0840103382c5645fb884ca77fc4c559900af26",
    "collision_sha256": "0724d1375ead2e5739ffb12a841afaffb0e0240be466051203cf483d1fb6704c",
    "ply_sha256": "2dd79f2dbc501ad92a4e2f544789cb27e18802c6c9c794d3503dbab5d30ad065",
    "pcd_sha256": "ba548bd9fde09f65278f9f3117e9e8cd93079eb2025769d9cd169dcf8455de44",
    "collision_faces": 499999,
    "ply_vertices": 180417,
    "ply_faces": 348297,
    "pcd_points": 391226,
}


class Audit:
    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        if condition:
            print(f"PASS  {label}{': ' + detail if detail else ''}")
        else:
            message = f"{label}{': ' + detail if detail else ''}"
            self.failures.append(message)
            print(f"FAIL  {message}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cache_is_fresh(cache: Path, dependencies: list[Path]) -> bool:
    """Return whether a non-empty cache is at least as new as every input."""
    if not cache.is_file() or cache.stat().st_size == 0:
        return False
    cache_mtime = cache.stat().st_mtime_ns
    return all(
        dependency.is_file() and cache_mtime >= dependency.stat().st_mtime_ns
        for dependency in dependencies
    )


def mesh_header(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with path.open("rb") as stream:
        for raw_line in stream:
            line = raw_line.decode("ascii").strip()
            if line.startswith("format "):
                values["format"] = line.removeprefix("format ")
            elif line.startswith("element vertex "):
                values["vertices"] = line.removeprefix("element vertex ")
            elif line.startswith("element face "):
                values["faces"] = line.removeprefix("element face ")
            if line == "end_header":
                break
    return values


def pcd_header(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    with path.open("rb") as stream:
        for raw_line in stream:
            line = raw_line.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            values[key] = value.strip()
            if key == "DATA":
                break
    return values


def pcd_xyz_are_cell_centres(path: Path, pitch: float) -> bool:
    """Check the actual float32 payload, not only its generation report."""
    with path.open("rb") as stream:
        while True:
            raw_line = stream.readline()
            if not raw_line:
                return False
            if raw_line.startswith(b"DATA "):
                break
        payload = stream.read()
    if len(payload) % 12:
        return False
    for point in struct.iter_unpack("<fff", payload):
        for coordinate in point:
            lattice_key = coordinate / pitch - 0.5
            if abs(lattice_key - round(lattice_key)) > 2.0e-5:
                return False
    return True


def launch_argument_block(source: str, argument_name: str) -> str:
    """Return one DeclareLaunchArgument block for focused value checks."""
    match = re.search(
        rf"DeclareLaunchArgument\(\s*[\"']{re.escape(argument_name)}[\"']",
        source,
    )
    if match is None:
        return ""
    next_match = re.search(r"DeclareLaunchArgument\(", source[match.end() :])
    if next_match is None:
        return source[match.start() :]
    return source[match.start() : match.end() + next_match.start()]


def launch_mapping_block(source: str, parameter_name: str) -> str:
    """Return the nearby mapping expression for one launch parameter."""
    marker = f'"{parameter_name}":'
    start = source.find(marker)
    if start < 0:
        return ""
    end = source.find("\n            ),", start)
    if end < 0:
        return source[start : start + 320]
    return source[start : end + len("\n            ),")]


def main() -> int:
    audit = Audit()
    usage_doc = ROOT / "doc" / "CONVERSION_AND_USAGE.md"
    root_readme = ROOT / "README.md"
    field_readme = ROOT / "field" / "README.md"
    audit.check(usage_doc.is_file(), "主使用文档存在", str(usage_doc.relative_to(ROOT)))
    audit.check(root_readme.is_file(), "项目 README 存在")
    audit.check(field_readme.is_file(), "field README 存在")
    if root_readme.is_file():
        audit.check(
            "(doc/CONVERSION_AND_USAGE.md)" in root_readme.read_text(encoding="utf-8"),
            "项目 README 指向移动后的主文档",
        )
    if field_readme.is_file():
        audit.check(
            "(../doc/CONVERSION_AND_USAGE.md)"
            in field_readme.read_text(encoding="utf-8"),
            "field README 指向移动后的主文档",
        )

    field_model = FIELD / "gazebo" / "models" / "rmuc2026_field"
    field_meshes = field_model / "meshes"
    ros_model = ROS_SIM / "models" / "rmuc2026_field"
    ros_meshes = ros_model / "meshes"

    visual = field_meshes / "rmuc2026_field_visual.stl"
    collision = field_meshes / "rmuc2026_field_collision.stl"
    ply = FIELD / "mesh_planner" / "rmuc2026_field.ply"
    pcd = FIELD / "jie_nav" / "rmuc2026_field.pcd"
    paths = [visual, collision, ply, pcd]
    for path in paths:
        audit.check(path.is_file(), "文件存在", str(path.relative_to(ROOT)))
    if any(not path.is_file() for path in paths):
        return 1

    hashes = {
        "visual": sha256(visual),
        "collision": sha256(collision),
        "ply": sha256(ply),
    }
    audit.check(hashes["visual"] == EXPECTED["visual_sha256"], "visual STL 哈希")
    audit.check(
        hashes["collision"] == EXPECTED["collision_sha256"], "collision STL 哈希"
    )
    audit.check(hashes["ply"] == EXPECTED["ply_sha256"], "多层 PLY 哈希")
    audit.check(sha256(pcd) == EXPECTED["pcd_sha256"], "PCD 哈希")

    pairs = [
        (visual, ros_meshes / visual.name, "visual STL 已同步"),
        (collision, ros_meshes / collision.name, "collision STL 已同步"),
        (ply, ROS_MAPS / ply.name, "多层 PLY 已同步"),
        (field_model / "model.sdf", ros_model / "model.sdf", "model.sdf 已同步"),
        (
            FIELD / "gazebo" / "worlds" / "rmuc2026_field.sdf",
            ROS_SIM / "worlds" / "rmuc2026_field.sdf",
            "world SDF 已同步",
        ),
    ]
    for source, target, label in pairs:
        audit.check(target.is_file(), f"{label}（目标存在）")
        if target.is_file():
            audit.check(sha256(source) == sha256(target), label)

    header = mesh_header(ply)
    audit.check(header.get("format") == "binary_little_endian 1.0", "PLY 二进制格式")
    audit.check(int(header.get("vertices", -1)) == EXPECTED["ply_vertices"], "PLY 顶点数")
    audit.check(int(header.get("faces", -1)) == EXPECTED["ply_faces"], "PLY 三角形数")

    pcd_values = pcd_header(pcd)
    audit.check(pcd_values.get("FIELDS") == "x y z", "PCD 字段为 XYZ")
    audit.check(pcd_values.get("SIZE") == "4 4 4", "PCD 字段为 32 位")
    audit.check(pcd_values.get("TYPE") == "F F F", "PCD 字段为浮点")
    audit.check(int(pcd_values.get("POINTS", -1)) == EXPECTED["pcd_points"], "PCD 点数")
    audit.check(pcd_values.get("DATA") == "binary", "PCD 数据为 binary")
    audit.check(
        pcd_xyz_are_cell_centres(pcd, 0.05),
        "PCD 实际 float32 坐标位于 0.05 m 体素中心",
    )

    surface_report_path = FIELD / "jie_nav" / "rmuc2026_field.surface.json"
    tunnel_report_path = FIELD / "jie_nav" / "rmuc2026_field.tunnel.json"
    audit.check(surface_report_path.is_file(), "JIE 表面栅格生成报告存在")
    audit.check(tunnel_report_path.is_file(), "JIE 双隧道回归报告存在")
    if surface_report_path.is_file():
        surface_report = json.loads(surface_report_path.read_text(encoding="utf-8"))
        audit.check(
            surface_report.get("source_sha256") == hashes["collision"],
            "JIE PCD 源为 canonical collision STL",
        )
        audit.check(
            surface_report.get("output_sha256") == EXPECTED["pcd_sha256"],
            "JIE 表面栅格报告哈希",
        )
        audit.check(
            surface_report.get("output_points") == EXPECTED["pcd_points"],
            "JIE 表面栅格报告点数",
        )
        audit.check(
            surface_report.get("surface_voxel_m") == 0.05
            and surface_report.get("subdivision_edge_factor") == 1.1,
            "JIE 表面采样间距与细分参数",
        )
        audit.check(
            surface_report.get("output_coordinate_policy")
            == "(nearest_lattice_key + 0.5) * surface_voxel_m",
            "JIE PCD 使用稳定体素中心量化",
        )
        audit.check(
            surface_report.get("z_band_m") == [-0.08, 0.9],
            "JIE PCD 仅保留机器人相关高度带",
        )
    if tunnel_report_path.is_file():
        tunnel_report = json.loads(tunnel_report_path.read_text(encoding="utf-8"))
        tunnel_checks = tunnel_report.get("checks", [])
        audit.check(
            tunnel_report.get("resolution_m") == 0.05
            and tunnel_report.get("robot_radius_xy_m") == 0.28
            and tunnel_report.get("robot_physical_height_m") == 0.225,
            "JIE 双隧道回归使用 RMUC 物理包络",
        )
        audit.check(
            tunnel_report.get("all_connected_on_lower_layer") is True
            and len(tunnel_checks) == 2
            and all(check.get("lower_layer_only") for check in tunnel_checks),
            "JIE 正负 Y 真隧道在低层连通",
        )
        audit.check(
            all(check.get("path_z_range_m") == [0.07500000000000001] * 2 for check in tunnel_checks),
            "JIE 离线路径未误走隧道屋顶",
        )

    multilevel_report = json.loads(
        (FIELD / "mesh_planner" / "rmuc2026_field.multilevel.json").read_text(
            encoding="utf-8"
        )
    )
    audit.check(
        multilevel_report.get("output_vertices") == EXPECTED["ply_vertices"],
        "多层报告顶点数",
    )
    audit.check(
        multilevel_report.get("output_triangles") == EXPECTED["ply_faces"],
        "多层报告三角形数",
    )
    audit.check(multilevel_report.get("grid_resolution_m") == 0.05, "多层网格 0.05 m")
    audit.check(multilevel_report.get("max_slope_deg") == 55.0, "坡度阈值 55 度")
    audit.check(multilevel_report.get("robot_height_m") == 0.225, "净空阈值 0.225 m")
    audit.check(
        multilevel_report.get("surface_orientation_policy")
        == "signed_upward_plus_audited_bidirectional_regions",
        "反向法线仅在审计区域启用",
    )
    audit.check(
        multilevel_report.get("headroom_blocker_policy")
        == "next_slope_like_surface_ignore_vertical_seams",
        "竖直 CAD 接缝不作为顶板",
    )
    audit.check(
        multilevel_report.get("layer_matching_policy")
        == "reciprocal_nearest_height",
        "上下层使用一对一高度匹配",
    )
    audit.check(multilevel_report.get("nonmanifold_edges") == 0, "PLY 无非流形边")
    corridor_checks = multilevel_report.get("low_corridor_checks", [])
    audit.check(
        len(corridor_checks) == 4
        and all(check.get("connected_on_lower_layer") for check in corridor_checks),
        "四条 RMUC 下层通道回归通过",
    )

    collision_report_path = (
        FIELD
        / "gazebo"
        / "collision_candidates"
        / "rmuc2026_field_collision_component_budget_500k.json"
    )
    audit.check(collision_report_path.is_file(), "碰撞网格生成报告存在")
    if collision_report_path.is_file():
        collision_report = json.loads(
            collision_report_path.read_text(encoding="utf-8")
        )
        # The historical hand-audited report used candidate_*/sha256 while the
        # reproducible builder uses output_*.  Both describe the same artifact.
        collision_report_faces = collision_report.get(
            "output_faces", collision_report.get("candidate_faces")
        )
        collision_report_hash = collision_report.get(
            "output_sha256", collision_report.get("sha256")
        )
        collision_source_hash = collision_report.get("source_sha256")
        if collision_source_hash is None:
            collision_source_hash = (
                collision_report.get("comparison", {})
                .get("visual_reference", {})
                .get("sha256")
            )
        audit.check(
            collision_report_faces == EXPECTED["collision_faces"],
            "碰撞网格面数",
        )
        audit.check(
            collision_report_hash == hashes["collision"],
            "碰撞报告输出哈希对应 canonical collision STL",
        )
        audit.check(
            collision_source_hash == hashes["visual"],
            "碰撞报告输入哈希对应 canonical visual STL",
        )
        audit.check(
            "min_z <= 0.35 m" in collision_report.get("method", ""),
            "碰撞网格强制保留低层组件",
        )

    metadata = json.loads(
        (FIELD / "conversion_metadata.json").read_text(encoding="utf-8")
    )
    audit.check(
        metadata.get("mesh_planner_multilevel", {}).get("sha256") == hashes["ply"],
        "metadata 中的 PLY 哈希",
    )
    audit.check(
        metadata.get("gazebo_collision_postprocess", {}).get("collision_sha256")
        == hashes["collision"],
        "metadata 中的 collision 哈希",
    )
    jie_metadata = metadata.get("jie_nav_surface_pcd", {})
    audit.check(
        jie_metadata.get("sha256") == EXPECTED["pcd_sha256"]
        and jie_metadata.get("points") == EXPECTED["pcd_points"],
        "metadata 中的 canonical JIE PCD",
    )
    audit.check(
        jie_metadata.get("occupancy_semantics")
        == "deterministic collision-surface obstacle samples; never the Mesh navigation PLY",
        "metadata 区分 JIE 占据 PCD 与 Mesh 可通行 PLY",
    )
    physics_validation = metadata.get("gazebo_collision_postprocess", {}).get(
        "gazebo_physics_validation", []
    )
    if physics_validation:
        audit.check(
            any(
                "about 1.123 m" in item and "x=-0.32665055" in item
                for item in physics_validation
            )
            and any(
                "MoveBase succeeded with outcome=0" in item and "0.17776 m" in item
                for item in physics_validation
            )
            and not any("0.524 m" in item for item in physics_validation),
            "metadata 使用当前 Gazebo 直驱与 MoveBase 验证结果",
        )
    else:
        print("INFO  metadata 未记录可选的 Gazebo 人工运行验证；生成资产审计继续")

    sdf_root = ET.parse(field_model / "model.sdf").getroot()
    visual_uris = [node.text for node in sdf_root.findall(".//visual//uri")]
    collision_uris = [node.text for node in sdf_root.findall(".//collision//uri")]
    audit.check(
        "meshes/rmuc2026_field_visual.stl" in visual_uris,
        "SDF visual 使用高精度 STL",
    )
    audit.check(
        "meshes/rmuc2026_field_collision.stl" in collision_uris,
        "SDF collision 使用预算碰撞 STL",
    )

    integration_files = [
        ROOT
        / "jie_3d_nav"
        / "octo_planner"
        / "launch"
        / "meshnav_ceres_controller.launch.py",
        ROOT
        / "jie_3d_nav"
        / "octo_planner"
        / "config"
        / "meshnav_ceres_controller.yaml",
        ROOT / "jie_3d_nav" / "octo_planner" / "src" / "jie_twist_stamper.cpp",
        ROS_SIM / "src" / "slope_aware_holonomic_drive.cpp",
    ]
    for path in integration_files:
        audit.check(path.is_file(), "集成文件存在", str(path.relative_to(ROOT)))

    # Verify the critical integration choices, not just file existence. This
    # catches interrupted edits or an accidentally restored launch/config file.
    sim_launch = (ROS_SIM / "launch" / "simulation_launch.py").read_text(
        encoding="utf-8"
    )
    base_launch = (ROS_SIM / "launch" / "base_simulation_launch.py").read_text(
        encoding="utf-8"
    )
    robot_xacro = (ROS_SIM / "urdf" / "ceres.urdf.xacro").read_text(
        encoding="utf-8"
    )
    wheel_xacro = (ROS_SIM / "urdf" / "wheel.urdf.xacro").read_text(
        encoding="utf-8"
    )
    nav_launch_path = (
        ROS_SIM.parent
        / "mesh_navigation_tutorials"
        / "launch"
        / "mesh_navigation_tutorials_launch.py"
    )
    nav_config_path = (
        ROS_SIM.parent
        / "mesh_navigation_tutorials"
        / "config"
        / "mbf_mesh_nav.yaml"
    )
    nav_launch = nav_launch_path.read_text(encoding="utf-8")
    nav_config = nav_config_path.read_text(encoding="utf-8")
    bridge_config = (ROS_SIM / "config" / "ros_gazebo_bridge.yaml").read_text(
        encoding="utf-8"
    )
    body_height_argument = launch_argument_block(sim_launch, "robot_body_height")
    body_length_argument = launch_argument_block(sim_launch, "robot_body_length")
    body_width_argument = launch_argument_block(sim_launch, "robot_body_width")
    legacy_body_geometry = (
        '<xacro:arg name="body_height" default="0.15"/>' in robot_xacro
        and '<xacro:arg name="body_length" default="0.38"/>' in robot_xacro
        and '<xacro:arg name="body_width" default="0.32"/>' in robot_xacro
        and '<xacro:property name="body_height" value="$(arg body_height)"/>'
        in robot_xacro
        and '<xacro:property name="body_length" value="$(arg body_length)"/>'
        in robot_xacro
        and '<xacro:property name="body_width" value="$(arg body_width)"/>'
        in robot_xacro
        and '"0.10" if "' in body_height_argument
        and 'else "0.15"' in body_height_argument
        and '"0.32" if "' in body_length_argument
        and 'else "0.38"' in body_length_argument
        and '"0.26" if "' in body_width_argument
        and 'else "0.32"' in body_width_argument
        and "body_height:=" in base_launch
        and "body_length:=" in base_launch
        and "body_width:=" in base_launch
    )
    audit.check(
        legacy_body_geometry,
        "RMUC 低车体与 legacy 原尺寸通过 launch/xacro profile 隔离",
    )
    audit.check(
        '"True" if "' in sim_launch
        and '" == "rmuc2026_field" else "False"' in sim_launch
        and '"slope_aware_drive"' in sim_launch,
        "RMUC world 自动启用坡面底盘",
    )
    audit.check(
        '"slope_aware_drive"' in base_launch
        and "slope_aware_drive:=" in base_launch,
        "坡面底盘参数传入 xacro",
    )
    audit.check(
        '"0.10" if "' in sim_launch
        and '"0.060" if "' in sim_launch
        and '"robot_wheel_radius"' in base_launch
        and "wheel_radius:=" in base_launch
        and '<xacro:arg name="wheel_radius" default="0.125"/>' in robot_xacro
        and "wheel_radius / 0.125" in wheel_xacro,
        "RMUC 低轮廓车型与旧 world 尺寸隔离",
    )
    static_inscribed_block = launch_mapping_block(
        nav_launch, "static_inscribed_radius"
    )
    obstacle_inscribed_block = launch_mapping_block(
        nav_launch, "obstacle_inscribed_radius"
    )
    obstacle_height_block = launch_mapping_block(nav_launch, "obstacle_robot_height")
    audit.check(
        '"0.28" if "' in static_inscribed_block
        and '"rmuc2026_field"' in static_inscribed_block
        and '"0.28" if "' in obstacle_inscribed_block
        and '"rmuc2026_field"' in obstacle_inscribed_block
        and '"0.225" if "' in obstacle_height_block
        and '"rmuc2026_field"' in obstacle_height_block
        and '"height_diff_threshold": "0.2"' in nav_launch,
        "MeshNav RMUC 静态/动态内切半径均为 0.28 m，机器人高度为 0.225 m",
    )
    audit.check(
        re.search(
            r"(?ms)^\s+height_diff:\s*$.*?^\s+radius\s*:\s*0\.2\s*$"
            r".*?^\s+threshold:\s*0\.2\s*$",
            nav_config,
        )
        is not None
        and re.search(
            r"^\s+edge_cost_factor:\s*8(?:\.0)?\s*$",
            nav_config,
            re.MULTILINE,
        )
        is not None,
        "MeshNav 用户调参保持 height_diff radius/threshold=0.2、edge_cost_factor=8",
    )
    audit.check(
        '<xacro:if value="$(arg slope_aware_drive)">' in robot_xacro
        and "SlopeAwareHolonomicDrive" in robot_xacro
        and '<xacro:unless value="$(arg slope_aware_drive)">' in robot_xacro
        and "systems::DiffDrive" in robot_xacro,
        "RMUC 与旧 world 的底盘插件相互隔离",
    )
    active_bridge_lines = [
        line.strip()
        for line in bridge_config.splitlines()
        if line.lstrip().startswith("- ros_topic_name:")
        and '"/cmd_vel"' in line
    ]
    audit.check(
        len(active_bridge_lines) == 1
        and 'ros_type_name: "geometry_msgs/msg/TwistStamped"' in bridge_config,
        "Gazebo 只启用一个 TwistStamped /cmd_vel bridge",
    )

    jie_path_source = (
        ROOT / "jie_3d_nav" / "octo_planner" / "src" / "jie_path_node.cpp"
    ).read_text(encoding="utf-8")
    audit.check(
        "goal_pose_topic, rclcpp::QoS(1).reliable()" in jie_path_source,
        "JIE /goal_pose 使用 RViz 兼容的 volatile durability",
    )

    pcd_converter_source = (
        ROOT / "jie_3d_nav" / "jie_octomap" / "src" / "pcd_to_octomap_node.cpp"
    ).read_text(encoding="utf-8")
    audit.check(
        'declare_parameter<double>("republish_period_s", 0.0);'
        in pcd_converter_source
        and "if (republish_period_s > 0.0)" in pcd_converter_source,
        "PCD converter 默认不周期重发 OctoMap（republish_period_s=0）",
    )

    jie_import_launch = (
        ROOT / "jie_3d_nav" / "jie_octomap" / "launch" / "import_pcd_map.launch.py"
    ).read_text(encoding="utf-8")
    resolution_argument = launch_argument_block(jie_import_launch, "resolution")
    radius_argument = launch_argument_block(jie_import_launch, "robot_radius_xy")
    height_argument = launch_argument_block(jie_import_launch, "robot_height")
    gui_argument = launch_argument_block(jie_import_launch, "start_import_gui")
    audit.check(
        "'0.05' if '" in resolution_argument
        and "rmuc2026_profile" in resolution_argument
        and "'0.28' if '" in radius_argument
        and "rmuc2026_profile" in radius_argument
        and "'0.225' if '" in height_argument
        and "rmuc2026_profile" in height_argument
        and '"resolution": ParameterValue(resolution, value_type=float)'
        in jie_import_launch
        and '"robot_radius_xy": ParameterValue(robot_radius_xy, value_type=float)'
        in jie_import_launch
        and '"robot_height": ParameterValue(robot_height, value_type=float)'
        in jie_import_launch,
        "JIE RMUC profile 显式传入 0.05/0.28/0.225",
    )
    audit.check(
        'default_value="true"' in gui_argument
        and 'choices=["true", "false"]' in gui_argument
        and "condition=IfCondition(start_import_gui)" in jie_import_launch,
        "JIE PCD 导入 GUI 可由 start_import_gui 显式开关",
    )

    tutorial_package = ET.parse(
        ROS_SIM.parent / "mesh_navigation_tutorials" / "package.xml"
    ).getroot()
    tutorial_exec_depends = {
        node.text for node in tutorial_package.findall("exec_depend")
    }
    audit.check(
        {"ament_index_python", "launch", "launch_ros"}.issubset(
            tutorial_exec_depends
        ),
        "MeshNav tutorial launch 运行依赖已声明",
    )

    installed_files = [
        MESH_WS
        / "install"
        / "mesh_navigation_tutorials_sim"
        / "lib"
        / "libslope_aware_holonomic_drive_system.so",
        MESH_WS / "install" / "mbf_mesh_nav" / "lib" / "mbf_mesh_nav" / "mbf_mesh_nav",
        ROOT
        / "jie_3d_nav"
        / "install"
        / "octo_planner"
        / "lib"
        / "octo_planner"
        / "jie_twist_stamper",
    ]
    for path in installed_files:
        audit.check(path.is_file(), "已构建运行文件存在", str(path.relative_to(ROOT)))

    h5 = MESH_WS / "rmuc2026_field.h5"
    if h5.is_file():
        audit.check(
            cache_is_fresh(h5, [ply, nav_launch_path, nav_config_path]),
            "MeshNav H5 缓存不早于 PLY、顶层 launch 与 mbf_mesh_nav.yaml",
        )
    else:
        audit.check(True, "旧 H5 已失效；首次启动将按新 PLY 重建")

    if audit.failures:
        print(f"\n审计失败：{len(audit.failures)} 项。", file=sys.stderr)
        return 1
    print("\n审计通过：RMUC2026 canonical 资产、ROS 副本和集成文件一致。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
