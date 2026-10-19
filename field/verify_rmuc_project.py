#!/usr/bin/env python3
"""Audit Gazebo assets, MeshNav maps, ROS copies, and H5 cache freshness.

This is an offline consistency check. It does not launch ROS or establish that
a generated navigation surface supports a particular route or robot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent.parent
FIELD = ROOT / "field" / "converted_rmuc2026"
MESH_WS = ROOT / "mesh_navigation_tutorials"


class Audit:
    def __init__(self) -> None:
        self.failures: list[str] = []

    def check(self, condition: bool, label: str, detail: str = "") -> None:
        message = f"{label}{': ' + detail if detail else ''}"
        if condition:
            print(f"PASS  {message}")
        else:
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
        if stream.readline().strip() != b"ply":
            raise ValueError(f"not a PLY file: {path}")
        for raw_line in stream:
            line = raw_line.decode("ascii").strip()
            if line.startswith("format "):
                values["format"] = line.removeprefix("format ")
            elif line.startswith("element vertex "):
                values["vertices"] = line.removeprefix("element vertex ")
            elif line.startswith("element face "):
                values["faces"] = line.removeprefix("element face ")
            if line == "end_header":
                return values
    raise ValueError(f"missing PLY end_header: {path}")


def stl_face_count(path: Path) -> int:
    """Validate the binary STL layout produced by the conversion scripts."""
    with path.open("rb") as stream:
        header = stream.read(84)
    if len(header) != 84:
        raise ValueError(f"truncated binary STL: {path}")
    count = struct.unpack("<I", header[80:84])[0]
    if count == 0 or path.stat().st_size != 84 + count * 50:
        raise ValueError(f"invalid binary STL triangle count or length: {path}")
    return count


def read_json(path: Path, audit: Audit) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("expected a JSON object")
        return value
    except (OSError, ValueError) as error:
        audit.check(False, "JSON 报告可读取", f"{path}: {error}")
        return {}


def metadata_section(metadata: dict, name: str, audit: Audit) -> dict:
    value = metadata.get(name, {})
    if not isinstance(value, dict):
        audit.check(False, "metadata 分节为 JSON 对象", name)
        return {}
    return value


def check_sdf(model: Path, world: Path, model_name: str, audit: Audit) -> None:
    try:
        model_root = ET.parse(model).getroot()
        world_root = ET.parse(world).getroot()
    except (OSError, ET.ParseError) as error:
        audit.check(False, "Gazebo SDF 可读取", str(error))
        return
    entity = model_root.find("model")
    audit.check(
        entity is not None and entity.get("name") == model_name,
        "SDF 模型名称与资源目录一致",
    )
    audit.check(model_root.findtext("model/static") == "true", "Gazebo 场地为静态模型")
    for kind in ("visual", "collision"):
        meshes = model_root.findall(f".//{kind}//mesh")
        expected = f"meshes/{model_name}_{kind}.stl"
        audit.check(
            any(mesh.findtext("uri") == expected for mesh in meshes),
            f"SDF {kind} 使用对应 STL",
        )
        try:
            unit_scale = all(
                [float(value) for value in mesh.findtext("scale", "1 1 1").split()]
                == [1.0, 1.0, 1.0]
                for mesh in meshes
            )
        except ValueError:
            unit_scale = False
        audit.check(bool(meshes) and unit_scale, f"SDF {kind} 使用米制 scale=1")
    world_entity = world_root.find("world")
    audit.check(
        world_entity is not None and world_entity.get("name") == model_name,
        "world SDF 名称与所选场地一致",
    )
    audit.check(
        f"model://{model_name}"
        in [node.text for node in world_root.findall(".//include/uri")],
        "world SDF 引用所选模型",
    )


def verify_project(
    base: Path = FIELD,
    workspace: Path = MESH_WS,
    model_name: str = "rmuc2026_field",
    h5: Path | None = None,
    check_installed: bool = False,
    ros_map: Path | None = None,
) -> Audit:
    audit = Audit()
    base, workspace = Path(base), Path(workspace)
    model = base / "gazebo" / "models" / model_name
    meshes = model / "meshes"
    visual = meshes / f"{model_name}_visual.stl"
    collision = meshes / f"{model_name}_collision.stl"
    ply = base / "mesh_planner" / f"{model_name}.ply"
    world = base / "gazebo" / "worlds" / f"{model_name}.sdf"
    metadata_path = base / "conversion_metadata.json"
    ros_package = workspace / "src" / "mesh_navigation_tutorials"
    ros_sim = workspace / "src" / "mesh_navigation_tutorials_sim"
    ros_model = ros_sim / "models" / model_name
    ros_ply = Path(ros_map) if ros_map is not None else ros_package / "maps" / ply.name
    nav_launch = ros_package / "launch" / "mbf_mesh_navigation_server_launch.py"
    nav_config = ros_package / "config" / "mbf_mesh_nav.yaml"

    required = [
        visual, collision, ply, model / "model.sdf", model / "model.config",
        world, metadata_path, nav_launch, nav_config,
    ]
    for path in required:
        audit.check(path.is_file(), "文件存在", str(path))
    if any(not path.is_file() for path in required):
        return audit

    hashes = {"visual": sha256(visual), "collision": sha256(collision), "ply": sha256(ply)}
    pairs = [
        (visual, ros_model / "meshes" / visual.name),
        (collision, ros_model / "meshes" / collision.name),
        (ply, ros_ply),
        (model / "model.sdf", ros_model / "model.sdf"),
        (model / "model.config", ros_model / "model.config"),
        (world, ros_sim / "worlds" / world.name),
    ]
    for source, target in pairs:
        audit.check(target.is_file(), "ROS 副本存在", str(target))
        if target.is_file():
            audit.check(sha256(source) == sha256(target), "ROS 副本哈希一致", str(target))

    try:
        visual_faces = stl_face_count(visual)
        collision_faces = stl_face_count(collision)
        header = mesh_header(ply)
        vertices = int(header.get("vertices", -1))
        triangles = int(header.get("faces", -1))
    except (OSError, ValueError) as error:
        audit.check(False, "网格文件格式有效", str(error))
        return audit
    audit.check(header.get("format") == "binary_little_endian 1.0", "PLY 为小端二进制格式")
    audit.check(vertices > 0 and triangles > 0, "PLY 包含导航顶点与三角面")

    metadata = read_json(metadata_path, audit)
    audit.check(metadata.get("output_unit") == "metre", "转换 metadata 输出单位为米")
    audit.check(metadata.get("mesh_triangles") == visual_faces, "metadata visual STL 面数一致")

    postprocess = metadata_section(metadata, "gazebo_collision_postprocess", audit)
    if postprocess:
        audit.check(
            postprocess.get("collision_sha256") == hashes["collision"],
            "metadata collision STL 哈希一致",
        )
        audit.check(
            postprocess.get("source_sha256") == hashes["visual"],
            "碰撞后处理输入对应 visual STL",
        )
        audit.check(
            postprocess.get("output_faces", postprocess.get("candidate_faces")) == collision_faces,
            "metadata collision STL 面数一致",
        )
    else:
        audit.check(
            hashes["visual"] == hashes["collision"],
            "未后处理的 collision STL 与 visual STL 一致",
        )

    nav_section = "mesh_planner_multilevel"
    if nav_section not in metadata:
        nav_section = "pcd_to_mesh_navigation"
    navigation = metadata_section(metadata, nav_section, audit)
    audit.check(bool(navigation), "metadata 记录 canonical MeshNav PLY")
    audit.check(navigation.get("sha256") == hashes["ply"], "metadata PLY 哈希一致")
    canonical_file = navigation.get("canonical_file", "")
    if not isinstance(canonical_file, str):
        audit.check(False, "metadata canonical_file 为路径字符串")
        canonical_file = ""
    canonical_path = Path(canonical_file)
    if not canonical_path.is_absolute():
        canonical_path = base / canonical_path
    audit.check(canonical_path.resolve() == ply.resolve(), "metadata 指向所选 canonical PLY")
    report_suffix = ".multilevel.json" if nav_section == "mesh_planner_multilevel" else ".pcd_to_mesh.json"
    report_value = navigation.get("report", str(ply.with_suffix(report_suffix)))
    if not isinstance(report_value, str):
        audit.check(False, "metadata report 为路径字符串")
        report_value = str(ply.with_suffix(report_suffix))
    report_path = Path(report_value)
    if not report_path.is_absolute() and "report" in navigation:
        report_path = base / report_path
    report = read_json(report_path, audit)
    audit.check(report.get("output_vertices") == vertices, "MeshNav 报告顶点数一致")
    audit.check(report.get("output_triangles") == triangles, "MeshNav 报告三角面数一致")
    audit.check(report.get("nonmanifold_edges") == 0, "MeshNav 报告无非流形边")
    for parameter in ("grid_resolution_m", "max_slope_deg", "robot_height_m"):
        audit.check(
            parameter in navigation and navigation[parameter] == report.get(parameter),
            f"MeshNav metadata 与报告参数一致（{parameter}）",
        )
    if "output_ply_sha256" in report:
        audit.check(report["output_ply_sha256"] == hashes["ply"], "MeshNav 报告 PLY 哈希一致")
    corridor_checks = report.get("low_corridor_checks", [])
    if not isinstance(corridor_checks, list):
        audit.check(False, "MeshNav 通道回归报告为列表")
        corridor_checks = []
    for check in corridor_checks:
        audit.check(
            isinstance(check, dict) and check.get("connected_on_lower_layer") is True,
            "已记录的下层通道回归通过",
        )
    check_sdf(model / "model.sdf", world, model_name, audit)

    cache = Path(h5) if h5 is not None else ros_ply.with_suffix(".h5")
    if cache.exists() or cache.is_symlink():
        audit.check(
            cache_is_fresh(cache, [ply, ros_ply, nav_launch, nav_config]),
            "MeshNav H5 缓存不早于 PLY、导航 launch 与参数文件",
            str(cache),
        )
    else:
        print(f"INFO  H5 尚未生成；首次启动将按所选 PLY 重建：{cache}")

    if check_installed:
        installed = [
            workspace / "install/mbf_mesh_nav/lib/mbf_mesh_nav/mbf_mesh_nav",
            workspace / "install/pb_gazebo_sim_support/lib/libpb_chassis_truth_system.so",
            workspace / "install/pb_gazebo_sim_support/lib/libpb_velocity_drive_system.so",
        ]
        for path in installed:
            audit.check(path.is_file(), "已构建运行文件存在", str(path))
    return audit


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=FIELD, help="conversion output directory")
    parser.add_argument("--workspace", type=Path, default=MESH_WS, help="current ROS workspace")
    parser.add_argument("--model-name", default="rmuc2026_field")
    parser.add_argument("--ros-map", type=Path, help="actual navigation PLY; default is workspace/src/mesh_navigation_tutorials/maps/<model>.ply")
    parser.add_argument("--h5", type=Path, help="cache to audit; default is the ROS map's sibling .h5")
    parser.add_argument("--check-installed", action="store_true", help="also require built MeshNav/PB runtime files")
    arguments = parser.parse_args(argv)
    audit = verify_project(
        arguments.base, arguments.workspace, arguments.model_name,
        arguments.h5, arguments.check_installed, arguments.ros_map,
    )
    if audit.failures:
        print(f"\n审计失败：{len(audit.failures)} 项。", file=sys.stderr)
        return 1
    print("\n审计通过：Gazebo/MeshNav 生成资产、ROS 副本和缓存时间一致；仍需另行验证导航路线。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
