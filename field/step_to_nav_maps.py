#!/usr/bin/env python3
"""Convert a millimetre STEP arena into Gazebo, mesh_nav, and JIE map assets.

The script deliberately separates inspection from tessellation because large STEP
assemblies can require many gigabytes of memory.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

import trimesh


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def read_step(path: Path):
    log(f"Reading STEP: {path}")
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"OpenCascade failed to read STEP, status={status}")
    roots = reader.NbRootsForTransfer()
    log(f"STEP roots available for transfer: {roots}")
    transferred = reader.TransferRoots()
    if transferred <= 0:
        raise RuntimeError("STEP contains no transferable shape roots")
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError("Transferred STEP shape is null")
    log(f"Transferred roots: {transferred}")
    return shape, roots, transferred


def shape_bounds(shape) -> tuple[float, float, float, float, float, float]:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box, True)
    if box.IsVoid():
        raise RuntimeError("STEP shape has an empty bounding box")
    return tuple(float(value) for value in box.Get())


def count_faces(shape) -> int:
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    count = 0
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def inspect_step(step_path: Path, report_path: Path | None) -> dict:
    shape, roots, transferred = read_step(step_path)
    bounds = shape_bounds(shape)
    faces = count_faces(shape)
    xmin, ymin, zmin, xmax, ymax, zmax = bounds
    report = {
        "step_file": str(step_path.resolve()),
        "declared_source_unit": "millimetre",
        "source_to_ros_scale": 0.001,
        "roots": roots,
        "transferred_roots": transferred,
        "brep_faces": faces,
        "bounds_mm": {
            "min": [xmin, ymin, zmin],
            "max": [xmax, ymax, zmax],
            "size": [xmax - xmin, ymax - ymin, zmax - zmin],
        },
        "bounds_m_before_origin_shift": {
            "min": [xmin * 0.001, ymin * 0.001, zmin * 0.001],
            "max": [xmax * 0.001, ymax * 0.001, zmax * 0.001],
            "size": [
                (xmax - xmin) * 0.001,
                (ymax - ymin) * 0.001,
                (zmax - zmin) * 0.001,
            ],
        },
    }
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if report_path:
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        log(f"Wrote report: {report_path}")
    return report


def tessellate_shape(shape, linear_deflection_mm: float, angular_deflection_deg: float):
    angular_radians = math.radians(angular_deflection_deg)
    log(
        "Tessellating B-rep: "
        f"linear={linear_deflection_mm:g} mm, angular={angular_deflection_deg:g} deg"
    )
    mesher = BRepMesh_IncrementalMesh(
        shape, linear_deflection_mm, False, angular_radians, True
    )
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError("OpenCascade tessellation did not complete")

    vertex_blocks: list[np.ndarray] = []
    face_blocks: list[np.ndarray] = []
    vertex_offset = 0
    face_count = 0
    skipped_faces = 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        # pybind converts a null OpenCascade handle to None.  A valid
        # Poly_Triangulation object does not expose Handle::IsNull().
        if triangulation is None:
            skipped_faces += 1
            explorer.Next()
            continue

        transform = location.Transformation()
        node_count = triangulation.NbNodes()
        triangle_count = triangulation.NbTriangles()
        vertices = np.empty((node_count, 3), dtype=np.float64)
        for index in range(1, node_count + 1):
            point = triangulation.Node(index).Transformed(transform)
            vertices[index - 1] = (point.X(), point.Y(), point.Z())

        faces = np.empty((triangle_count, 3), dtype=np.int64)
        reversed_face = face.Orientation() == TopAbs_REVERSED
        for index in range(1, triangle_count + 1):
            a, b, c = triangulation.Triangle(index).Get()
            if reversed_face:
                b, c = c, b
            faces[index - 1] = (
                a - 1 + vertex_offset,
                b - 1 + vertex_offset,
                c - 1 + vertex_offset,
            )
        vertex_blocks.append(vertices)
        face_blocks.append(faces)
        vertex_offset += node_count
        face_count += triangle_count
        if len(vertex_blocks) % 5000 == 0:
            log(
                f"Tessellation extraction: {len(vertex_blocks)} facesets, "
                f"{vertex_offset} vertices, {face_count} triangles"
            )
        explorer.Next()

    if not vertex_blocks:
        raise RuntimeError("No triangulated STEP faces were produced")
    vertices_mm = np.concatenate(vertex_blocks, axis=0)
    faces = np.concatenate(face_blocks, axis=0)
    log(
        f"Raw mesh: {len(vertices_mm)} vertices, {len(faces)} triangles; "
        f"faces without triangulation={skipped_faces}"
    )
    return vertices_mm, faces, skipped_faces


def dominant_ground_z(vertices: np.ndarray, faces: np.ndarray, bin_size: float) -> float:
    triangles = vertices[faces]
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    twice_area = np.linalg.norm(cross, axis=1)
    valid = twice_area > 1e-12
    normal_z = np.zeros(len(faces), dtype=np.float64)
    normal_z[valid] = cross[valid, 2] / twice_area[valid]
    horizontal = valid & (normal_z > math.cos(math.radians(15.0)))
    if not np.any(horizontal):
        return float(vertices[:, 2].min())
    centroids_z = triangles[:, :, 2].mean(axis=1)
    bins = np.rint(centroids_z[horizontal] / bin_size).astype(np.int64)
    unique_bins, inverse = np.unique(bins, return_inverse=True)
    area_by_bin = np.bincount(inverse, weights=0.5 * twice_area[horizontal])
    best_bin = unique_bins[int(np.argmax(area_by_bin))]
    return float(best_bin * bin_size)


def clean_mesh(vertices_m: np.ndarray, faces: np.ndarray) -> trimesh.Trimesh:
    mesh = trimesh.Trimesh(vertices=vertices_m, faces=faces, process=False)
    mesh.update_faces(mesh.nondegenerate_faces(height=1e-7))
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices(digits_vertex=6)
    mesh.remove_unreferenced_vertices()
    return mesh


def write_binary_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype="<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(points.tobytes(order="C"))


def write_gazebo_model(
    output_dir: Path,
    model_name: str,
    collision_filename: str,
    visual_filename: str,
) -> None:
    model_dir = output_dir / "gazebo" / "models" / model_name
    model_dir.mkdir(parents=True, exist_ok=True)
    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Generated from STEP; mesh coordinates are metres.</description>
</model>
"""
    model_sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{model_name}">
    <static>true</static>
    <link name="map_link">
      <collision name="collision">
        <geometry><mesh><uri>meshes/{collision_filename}</uri><scale>1 1 1</scale></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>meshes/{visual_filename}</uri><scale>1 1 1</scale></mesh></geometry>
        <!-- STL carries no CAD material.  An explicit low-specular material
             keeps the field visible against Gazebo's light background. -->
        <cast_shadows>true</cast_shadows>
        <material>
          <ambient>0.08 0.12 0.18 1</ambient>
          <diffuse>0.28 0.46 0.68 1</diffuse>
          <specular>0.03 0.03 0.03 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    (model_dir / "model.config").write_text(model_config, encoding="utf-8")
    (model_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")


def write_gazebo_world(output_dir: Path, model_name: str) -> None:
    """Write the Gazebo Sim world required by the tutorial launch file."""
    world_dir = output_dir / "gazebo" / "worlds"
    world_dir.mkdir(parents=True, exist_ok=True)
    world_sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{model_name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="libignition-gazebo-physics-system.so"
            name="gz::sim::systems::Physics"/>
    <plugin filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-imu-system.so"
            name="ignition::gazebo::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.75 0.75 0.75 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://{model_name}</uri>
    </include>

    <scene>
      <ambient>0.22 0.24 0.28 1</ambient>
      <background>0.035 0.045 0.065 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
  </world>
</sdf>
"""
    (world_dir / f"{model_name}.sdf").write_text(world_sdf, encoding="utf-8")


def convert(args) -> None:
    shape, roots, transferred = read_step(args.step)
    source_bounds = shape_bounds(shape)
    vertices_mm, faces, skipped = tessellate_shape(
        shape, args.linear_deflection_mm, args.angular_deflection_deg
    )

    # STEP is explicitly declared as millimetres. Convert the geometry itself to
    # metres so every consumer can use scale=1 without hidden conventions.
    vertices_m = vertices_mm * 0.001
    xmin, ymin, zmin = vertices_m.min(axis=0)
    xmax, ymax, zmax = vertices_m.max(axis=0)
    if args.origin == "center-ground":
        ground_z = dominant_ground_z(vertices_m, faces, args.ground_bin_m)
        origin_shift = np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5, ground_z])
    elif args.origin == "center-min":
        origin_shift = np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin])
    else:
        origin_shift = np.zeros(3)
    vertices_m -= origin_shift
    log(f"Origin shift in original metre coordinates: {origin_shift.tolist()}")

    full_mesh = clean_mesh(vertices_m, faces)
    log(f"Clean mesh: {len(full_mesh.vertices)} vertices, {len(full_mesh.faces)} triangles")

    output = args.output
    gazebo_mesh_dir = output / "gazebo" / "models" / args.model_name / "meshes"
    mesh_nav_dir = output / "mesh_planner"
    jie_dir = output / "jie_nav"
    gazebo_mesh_dir.mkdir(parents=True, exist_ok=True)
    mesh_nav_dir.mkdir(parents=True, exist_ok=True)
    jie_dir.mkdir(parents=True, exist_ok=True)

    visual_name = f"{args.model_name}_visual.stl"
    collision_name = f"{args.model_name}_collision.stl"
    visual_path = gazebo_mesh_dir / visual_name
    collision_path = gazebo_mesh_dir / collision_name
    # Always overwrite both files.  The visual mesh is the authoritative
    # high-detail geometry; postprocess_nav_maps.py may simplify only the
    # collision copy afterwards.  This makes reruns safe for a changed STEP.
    log(f"Writing high-detail Gazebo visual STL: {visual_path}")
    full_mesh.export(visual_path, file_type="stl")
    log(f"Writing initial Gazebo collision STL: {collision_path}")
    full_mesh.export(collision_path, file_type="stl")
    write_gazebo_model(output, args.model_name, collision_name, visual_name)
    write_gazebo_world(output, args.model_name)

    slope_cos = math.cos(math.radians(args.max_slope_deg))
    walkable_indices = np.flatnonzero(full_mesh.face_normals[:, 2] >= slope_cos)
    if len(walkable_indices) == 0:
        raise RuntimeError("No upward walkable triangles matched max-slope")
    walkable = full_mesh.submesh([walkable_indices], append=True, repair=False)
    walkable.remove_unreferenced_vertices()
    # This is diagnostic input only.  A highest-Z/slope candidate cannot
    # represent overlapping tunnel and ramp surfaces.  The canonical PLY is
    # produced later by build_multilevel_nav_mesh.py from the visual STL.
    walkable_path = mesh_nav_dir / f"{args.model_name}_slope_candidates.ply"
    log(
        f"Writing diagnostic slope-candidate PLY: {walkable_path} "
        f"({len(walkable.faces)} triangles, max slope={args.max_slope_deg:g} deg)"
    )
    walkable.export(walkable_path, file_type="ply")

    total_area = float(full_mesh.area)
    requested_points = int(math.ceil(total_area / (args.sample_spacing_m**2)))
    sample_count = min(args.max_points, max(args.min_points, requested_points))
    if sample_count < requested_points:
        log(
            f"PCD sample count capped at {sample_count}; area/spacing requested "
            f"approximately {requested_points}"
        )
    # A global random sample is useful for visually inspecting STEP scaling,
    # but it is not a topology-safe occupancy map: small floors/walls may get
    # no sample at all.  Keep it under an explicitly diagnostic filename so a
    # full STEP conversion can never overwrite the canonical JIE map.  After
    # collision post-processing, build_jie_surface_pcd.py deterministically
    # rasterizes that collision mesh into the canonical *.pcd.
    log(f"Sampling {sample_count} points for diagnostic visual-surface PCD")
    points, _ = trimesh.sample.sample_surface(full_mesh, sample_count, seed=42)
    pcd_path = jie_dir / f"{args.model_name}_raw_visual_surface_sample.pcd"
    write_binary_pcd(pcd_path, points)
    log(f"Wrote diagnostic PCD (not canonical JIE occupancy): {pcd_path}")

    metadata = {
        "source_step": str(args.step.resolve()),
        "source_unit": "millimetre",
        "output_unit": "metre",
        "scale_applied": 0.001,
        "origin_mode": args.origin,
        "origin_shift_m_in_source_coordinates": origin_shift.tolist(),
        "source_brep_bounds_mm": list(source_bounds),
        "step_roots": roots,
        "transferred_roots": transferred,
        "faces_without_triangulation": skipped,
        "linear_deflection_mm": args.linear_deflection_mm,
        "angular_deflection_deg": args.angular_deflection_deg,
        "mesh_vertices": int(len(full_mesh.vertices)),
        "mesh_triangles": int(len(full_mesh.faces)),
        "slope_candidate_triangles": int(len(walkable.faces)),
        "slope_candidate_max_slope_deg": args.max_slope_deg,
        "raw_visual_surface_sample_pcd": str(pcd_path.relative_to(output)),
        "raw_surface_sample_points": int(sample_count),
        "bounds_m_after_shift": {
            "min": full_mesh.bounds[0].tolist(),
            "max": full_mesh.bounds[1].tolist(),
            "size": full_mesh.extents.tolist(),
        },
        "warning": (
            "The *_slope_candidates.ply file is diagnostic and cannot represent tunnels. "
            "Run build_multilevel_nav_mesh.py on the high-detail *_visual.stl before "
            "installing the canonical navigation PLY. The random *_raw_visual_surface_"
            "sample.pcd is also diagnostic: after collision post-processing, run "
            "build_jie_surface_pcd.py on *_collision.stl to create the canonical JIE PCD."
        ),
    }
    metadata_path = output / "conversion_metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    log(f"Conversion completed; metadata: {metadata_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="read STEP and report B-rep bounds")
    inspect_parser.add_argument("step", type=Path)
    inspect_parser.add_argument("--report", type=Path)

    convert_parser = subparsers.add_parser("convert", help="tessellate and export all map assets")
    convert_parser.add_argument("step", type=Path)
    convert_parser.add_argument("--output", type=Path, required=True)
    convert_parser.add_argument("--model-name", default="rmuc2026_field")
    convert_parser.add_argument("--linear-deflection-mm", type=float, default=20.0)
    convert_parser.add_argument("--angular-deflection-deg", type=float, default=15.0)
    convert_parser.add_argument("--max-slope-deg", type=float, default=35.0)
    convert_parser.add_argument("--sample-spacing-m", type=float, default=0.08)
    convert_parser.add_argument("--min-points", type=int, default=10000)
    convert_parser.add_argument("--max-points", type=int, default=5000000)
    convert_parser.add_argument("--ground-bin-m", type=float, default=0.02)
    convert_parser.add_argument(
        "--origin", choices=("center-ground", "center-min", "keep"), default="center-ground"
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        if args.command == "inspect":
            inspect_step(args.step, args.report)
        else:
            convert(args)
    except Exception as error:
        log(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
