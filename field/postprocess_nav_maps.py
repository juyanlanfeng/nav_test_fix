#!/usr/bin/env python3
"""Post-process raw STEP-derived meshes for Gazebo and mesh_navigation."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import shutil
import time

import numpy as np
import trimesh

from build_component_collision_mesh import (
    build_component_collision,
    load_single_mesh,
    sha256,
)
from conversion_metadata import (
    load_json_object,
    merge_hashed_object,
    merge_hashed_section,
    write_json_object,
)


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def ensure_visual_mesh_uri(model_sdf: Path, model_name: str) -> None:
    """Point visual elements at the detailed mesh without touching collision elements."""
    text = model_sdf.read_text(encoding="utf-8")
    collision_uri = f"<uri>meshes/{model_name}_collision.stl</uri>"
    visual_uri = f"<uri>meshes/{model_name}_visual.stl</uri>"
    visual_block = re.compile(r"<visual\b[^>]*>.*?</visual>", re.DOTALL)
    found = False
    changed = False

    def update_block(match: re.Match[str]) -> str:
        nonlocal found, changed
        block = match.group(0)
        if visual_uri in block:
            found = True
            return block
        if collision_uri in block:
            found = True
            changed = True
            return block.replace(collision_uri, visual_uri)
        return block

    updated = visual_block.sub(update_block, text)
    if not found:
        raise RuntimeError("Could not identify visual mesh URI in model.sdf")
    if changed:
        model_sdf.write_text(updated, encoding="utf-8")
        log("Updated model.sdf visual element to use the high-detail mesh")
    else:
        log("model.sdf already references the high-detail visual mesh")


def fill_small_holes(zgrid: np.ndarray, iterations: int = 2) -> np.ndarray:
    """Fill only cells surrounded by at least five finite neighbours."""
    result = zgrid.copy()
    for _ in range(iterations):
        finite = np.isfinite(result)
        neighbours = []
        neighbour_valid = []
        for dy, dx in ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)):
            shifted = np.roll(np.roll(result, dy, axis=0), dx, axis=1)
            valid = np.roll(np.roll(finite, dy, axis=0), dx, axis=1)
            # Rolled array wrap-around must not connect opposite map borders.
            if dy < 0:
                valid[dy:, :] = False
            elif dy > 0:
                valid[:dy, :] = False
            if dx < 0:
                valid[:, dx:] = False
            elif dx > 0:
                valid[:, :dx] = False
            neighbours.append(np.where(valid, shifted, np.nan))
            neighbour_valid.append(valid)
        stack = np.stack(neighbours, axis=0)
        count = np.sum(np.stack(neighbour_valid, axis=0), axis=0)
        fill = (~finite) & (count >= 5)
        if not np.any(fill):
            break
        median = np.nanmedian(stack, axis=0)
        result[fill] = median[fill]
    return result


def largest_edge_connected_component(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Return face indices in the largest component connected through full edges."""
    parent = np.arange(len(faces), dtype=np.int64)
    size = np.ones(len(faces), dtype=np.int64)

    def find(item: int) -> int:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = int(parent[item])
        return item

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if size[left_root] < size[right_root]:
            left_root, right_root = right_root, left_root
        parent[right_root] = left_root
        size[left_root] += size[right_root]

    edge_owner: dict[tuple[int, int], int] = {}
    for face_index, (a, b, c) in enumerate(faces):
        for u, v in ((int(a), int(b)), (int(b), int(c)), (int(c), int(a))):
            edge = (u, v) if u < v else (v, u)
            previous = edge_owner.get(edge)
            if previous is None:
                edge_owner[edge] = face_index
            else:
                union(face_index, previous)

    roots = np.fromiter((find(index) for index in range(len(faces))), dtype=np.int64)
    unique_roots, inverse = np.unique(roots, return_inverse=True)
    triangles = vertices[faces]
    xy_cross = np.abs(
        (triangles[:, 1, 0] - triangles[:, 0, 0])
        * (triangles[:, 2, 1] - triangles[:, 0, 1])
        - (triangles[:, 1, 1] - triangles[:, 0, 1])
        * (triangles[:, 2, 0] - triangles[:, 0, 0])
    )
    area_by_component = np.bincount(inverse, weights=0.5 * xy_cross)
    winner = int(np.argmax(area_by_component))
    log(
        f"Terrain components={len(unique_roots)}; largest projected area="
        f"{area_by_component[winner]:.3f} m^2"
    )
    return np.flatnonzero(inverse == winner)


def rebuild_terrain(candidate: trimesh.Trimesh, grid_m: float, max_slope_deg: float):
    bounds = candidate.bounds
    xmin = math.floor(bounds[0, 0] / grid_m) * grid_m
    ymin = math.floor(bounds[0, 1] / grid_m) * grid_m
    xmax = math.ceil(bounds[1, 0] / grid_m) * grid_m
    ymax = math.ceil(bounds[1, 1] / grid_m) * grid_m
    nx = int(round((xmax - xmin) / grid_m)) + 1
    ny = int(round((ymax - ymin) / grid_m)) + 1
    log(f"Terrain grid: {nx} x {ny}, resolution={grid_m:g} m")

    # Dense surface samples bridge CAD face/part seams.  Taking the highest
    # upward-facing surface means an obstacle footprint replaces the floor below
    # it; steep transitions are removed in the next stage.
    sample_count = max(100000, int(math.ceil(candidate.area / ((grid_m * 0.35) ** 2))))
    sampled, _ = trimesh.sample.sample_surface(candidate, sample_count, seed=43)
    centroids = candidate.triangles_center
    points = np.vstack((sampled, candidate.vertices, centroids))
    ix = np.rint((points[:, 0] - xmin) / grid_m).astype(np.int64)
    iy = np.rint((points[:, 1] - ymin) / grid_m).astype(np.int64)
    valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    zgrid = np.full((ny, nx), -np.inf, dtype=np.float64)
    np.maximum.at(zgrid, (iy[valid], ix[valid]), points[valid, 2])
    zgrid = fill_small_holes(zgrid, iterations=2)

    yy, xx = np.indices((ny, nx))
    vertices = np.column_stack(
        (xmin + xx.ravel() * grid_m, ymin + yy.ravel() * grid_m, zgrid.ravel())
    )
    grid_index = np.arange(nx * ny, dtype=np.int64).reshape(ny, nx)
    cell_valid = (
        np.isfinite(zgrid[:-1, :-1])
        & np.isfinite(zgrid[:-1, 1:])
        & np.isfinite(zgrid[1:, :-1])
        & np.isfinite(zgrid[1:, 1:])
    )
    iy_cell, ix_cell = np.nonzero(cell_valid)
    a = grid_index[iy_cell, ix_cell]
    b = grid_index[iy_cell, ix_cell + 1]
    c = grid_index[iy_cell + 1, ix_cell]
    d = grid_index[iy_cell + 1, ix_cell + 1]
    faces = np.vstack((np.column_stack((a, b, d)), np.column_stack((a, d, c))))

    triangles = vertices[faces]
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    norm = np.linalg.norm(cross, axis=1)
    normal_z = np.divide(cross[:, 2], norm, out=np.zeros_like(norm), where=norm > 1e-12)
    faces = faces[normal_z >= math.cos(math.radians(max_slope_deg))]
    log(f"Grid triangles after slope filter: {len(faces)}")
    if len(faces) == 0:
        raise RuntimeError("Terrain reconstruction produced no slope-valid triangles")

    keep = largest_edge_connected_component(vertices, faces)
    terrain = trimesh.Trimesh(vertices=vertices, faces=faces[keep], process=False)
    terrain.remove_unreferenced_vertices()
    terrain.merge_vertices(digits_vertex=6)
    terrain.remove_unreferenced_vertices()
    return terrain, sample_count


def simplify_gazebo(
    base: Path,
    model_name: str,
    target_faces: int,
    collision_method: str,
    mandatory_max_z: float,
    optional_max_z: float,
) -> dict:
    mesh_dir = base / "gazebo" / "models" / model_name / "meshes"
    source_collision = mesh_dir / f"{model_name}_collision.stl"
    visual_path = mesh_dir / f"{model_name}_visual.stl"
    if not visual_path.exists():
        log(f"Preserving high-detail visual mesh: {visual_path}")
        shutil.copy2(source_collision, visual_path)

    # Validate and, if needed, repair the SDF before replacing the collision
    # file.  A malformed SDF must not leave a half-updated output directory.
    model_sdf = base / "gazebo" / "models" / model_name / "model.sdf"
    ensure_visual_mesh_uri(model_sdf, model_name)

    source = load_single_mesh(visual_path)
    original_faces = len(source.faces)
    if collision_method == "component-budget":
        log(
            "Building component-preserving Gazebo collision mesh: "
            f"budget={target_faces}, mandatory_z<={mandatory_max_z:g} m, "
            f"optional_z<={optional_max_z:g} m"
        )
        collision, stats = build_component_collision(
            source, target_faces, mandatory_max_z, optional_max_z
        )
    elif original_faces <= target_faces:
        collision = source
        stats = {
            "method": "quadric decimation not required; source already within budget",
            "source_faces": original_faces,
            "output_faces": int(len(collision.faces)),
        }
    else:
        log(f"Simplifying Gazebo collision: {original_faces} -> target {target_faces} faces")
        # Retained only as an explicit legacy option.  Global decimation can
        # erase small ramps/tunnel surfaces even at a high global face count.
        collision = source.simplify_quadric_decimation(
            face_count=target_faces, aggression=5
        )
        stats = {
            "method": "legacy global quadric decimation, aggression=5",
            "source_faces": original_faces,
            "output_faces": int(len(collision.faces)),
        }
    collision.export(source_collision, file_type="stl")
    collision_hash = sha256(source_collision)
    stats.update(
        {
            "source": str(visual_path.resolve()),
            "output": str(source_collision.resolve()),
            "source_sha256": sha256(visual_path),
            "output_sha256": collision_hash,
            "visual_mesh_preserved": visual_path.name,
            "collision_mesh": source_collision.name,
            "collision_sha256": collision_hash,
            "collision_file_size_bytes": source_collision.stat().st_size,
        }
    )

    return stats


def default_collision_report_path(
    base: Path, model_name: str, collision_method: str, face_budget: int
) -> Path:
    """Return the report path consumed by the project verifier."""
    budget_label = (
        f"{face_budget // 1000}k" if face_budget % 1000 == 0 else str(face_budget)
    )
    method_label = collision_method.replace("-", "_")
    return (
        base
        / "gazebo"
        / "collision_candidates"
        / f"{model_name}_collision_{method_label}_{budget_label}.json"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("base", type=Path)
    parser.add_argument("--model-name", default="rmuc2026_field")
    parser.add_argument("--grid-m", type=float, default=0.1)
    parser.add_argument("--max-slope-deg", type=float, default=35.0)
    parser.add_argument("--collision-faces", type=int, default=500000)
    parser.add_argument(
        "--collision-method",
        choices=["component-budget", "quadric"],
        default="component-budget",
        help=(
            "component-budget preserves complete low CAD shells (recommended); "
            "quadric is the deprecated global decimator"
        ),
    )
    parser.add_argument("--collision-mandatory-max-z", type=float, default=0.35)
    parser.add_argument("--collision-optional-max-z", type=float, default=1.0)
    parser.add_argument(
        "--collision-report",
        type=Path,
        help="collision provenance JSON (default: gazebo/collision_candidates/...)",
    )
    parser.add_argument(
        "--allow-legacy-single-layer",
        action="store_true",
        help=(
            "Explicitly enable the deprecated highest-Z terrain rebuild. "
            "It cannot represent tunnels and must not be used for RMUC2026."
        ),
    )
    args = parser.parse_args()

    metadata_path = args.base / "conversion_metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(
            f"missing conversion metadata; run step_to_nav_maps.py first: {metadata_path}"
        )
    # Validate and capture prior evidence before rewriting a large collision
    # artifact, so malformed provenance cannot leave a half-updated tree.
    metadata = load_json_object(metadata_path)

    terrain = None
    samples = 0
    if args.allow_legacy_single_layer:
        candidate_path = args.base / "mesh_planner" / f"{args.model_name}.ply"
        candidate_backup = args.base / "mesh_planner" / f"{args.model_name}_slope_candidates.ply"
        if not candidate_backup.exists():
            shutil.copy2(candidate_path, candidate_backup)
        candidate = trimesh.load_mesh(candidate_backup, process=False)
        terrain, samples = rebuild_terrain(candidate, args.grid_m, args.max_slope_deg)
        terrain.export(candidate_path, file_type="ply")
        log(
            f"Wrote LEGACY single-layer terrain: {candidate_path}; "
            f"vertices={len(terrain.vertices)}, triangles={len(terrain.faces)}"
        )
    else:
        log(
            "Skipping deprecated highest-Z terrain rebuild. Use "
            "build_multilevel_nav_mesh.py for tunnel-aware navigation maps."
        )

    collision_stats = simplify_gazebo(
        args.base,
        args.model_name,
        args.collision_faces,
        args.collision_method,
        args.collision_mandatory_max_z,
        args.collision_optional_max_z,
    )
    log(
        "Gazebo collision triangles: "
        f"{collision_stats['source_faces']} -> {collision_stats['output_faces']}"
    )

    collision_report_path = args.collision_report or default_collision_report_path(
        args.base, args.model_name, args.collision_method, args.collision_faces
    )
    collision_report = merge_hashed_object(
        load_json_object(collision_report_path),
        collision_stats,
        ("output_sha256", "sha256"),
    )
    write_json_object(collision_report_path, collision_report)
    log(f"Wrote collision report: {collision_report_path}")

    if terrain is not None:
        metadata["mesh_planner_legacy_single_layer_postprocess"] = {
            "deprecated": True,
            "method": "highest-surface regular grid, slope filter, largest edge-connected component",
            "grid_resolution_m": args.grid_m,
            "surface_samples": samples,
            "vertices": int(len(terrain.vertices)),
            "triangles": int(len(terrain.faces)),
            "bounds_m": terrain.bounds.tolist(),
        }
    merge_hashed_section(
        metadata,
        "gazebo_collision_postprocess",
        collision_stats,
        "collision_sha256",
    )
    write_json_object(metadata_path, metadata)


if __name__ == "__main__":
    main()
