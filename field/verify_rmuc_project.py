#!/usr/bin/env python3
"""Verify the canonical RMUC2026 assets and their ROS workspace copies."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
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


def main() -> int:
    audit = Audit()
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

    collision_report = json.loads(
        (
            FIELD
            / "gazebo"
            / "collision_candidates"
            / "rmuc2026_field_collision_component_budget_500k.json"
        ).read_text(encoding="utf-8")
    )
    audit.check(
        collision_report.get("candidate_faces") == EXPECTED["collision_faces"],
        "碰撞网格面数",
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
    nav_launch = (
        ROS_SIM.parent
        / "mesh_navigation_tutorials"
        / "launch"
        / "mesh_navigation_tutorials_launch.py"
    ).read_text(encoding="utf-8")
    bridge_config = (ROS_SIM / "config" / "ros_gazebo_bridge.yaml").read_text(
        encoding="utf-8"
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
    audit.check(
        '"0.225" if "' in nav_launch
        and '"0.28" if "' in nav_launch
        and '"height_diff_threshold": "0.2"' in nav_launch,
        "MeshNav RMUC 高度与水平包络参数一致",
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
            h5.stat().st_size > 0 and h5.stat().st_mtime >= ply.stat().st_mtime,
            "MeshNav H5 缓存与新 PLY 同步",
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
