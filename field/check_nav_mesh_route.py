#!/usr/bin/env python3
"""Check whether two 3D poses land on the same edge-connected mesh component.

This is a geometry precheck. MeshNav GetPath remains the planning acceptance test.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from trimesh.proximity import closest_point_naive

from build_multilevel_nav_mesh import component_labels_by_edge


def check(mesh_path, start, goal, snap_max, clearance=0.0):
    mesh = trimesh.load_mesh(mesh_path, process=False)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError("input must be a nonempty triangle PLY")
    poses = np.asarray([start, goal], dtype=np.float64)
    snapped, distances, face_ids = closest_point_naive(mesh, poses)
    labels, roots, areas, _ = component_labels_by_edge(mesh.vertices, mesh.faces)
    same = bool(labels[face_ids[0]] == labels[face_ids[1]])
    close = bool(np.all(distances <= snap_max))
    edges, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    boundary = mesh.vertices[edges[counts == 1]]
    if len(boundary):
        ab = boundary[:, 1] - boundary[:, 0]
        length_squared = np.einsum("ij,ij->i", ab, ab)
        boundary_distances = []
        for position in snapped:
            projection = np.clip(
                np.einsum("ij,ij->i", position - boundary[:, 0], ab)
                / np.maximum(length_squared, 1e-12), 0.0, 1.0
            )
            closest = boundary[:, 0] + projection[:, None] * ab
            boundary_distances.append(float(np.min(np.linalg.norm(closest - position, axis=1))))
    else:
        boundary_distances = [float("inf")] * 2
    enough_clearance = bool(all(distance >= clearance for distance in boundary_distances))
    return {
        "mesh": str(Path(mesh_path).resolve()),
        "component_count": len(roots),
        "largest_component_area_m2": float(areas.max()),
        "poses": [
            {"input_xyz_m": pose.tolist(), "snapped_xyz_m": position.tolist(),
             "snap_distance_m": float(distance), "face": int(face),
             "boundary_clearance_m": boundary_distance,
             "component": int(labels[face]), "component_area_m2": float(areas[labels[face]])}
            for pose, position, distance, face, boundary_distance in
            zip(poses, snapped, distances, face_ids, boundary_distances)
        ],
        "same_edge_connected_component": same,
        "within_snap_limit": close,
        "minimum_boundary_clearance_m": clearance,
        "within_boundary_clearance": enough_clearance,
        "geometry_precheck_pass": same and close and enough_clearance,
        "meshnav_getpath_tested": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--start", nargs=3, type=float, required=True)
    parser.add_argument("--goal", nargs=3, type=float, required=True)
    parser.add_argument("--snap-max-m", type=float, default=0.2)
    parser.add_argument("--clearance-m", type=float, default=0.0,
                        help="required distance from start/goal to mesh boundary")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.snap_max_m <= 0 or args.clearance_m < 0:
        parser.error("--snap-max-m must be positive and --clearance-m nonnegative")
    result = check(args.mesh, args.start, args.goal, args.snap_max_m, args.clearance_m)
    message = json.dumps(result, ensure_ascii=False, indent=2)
    print(message)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(message + "\n", encoding="utf-8")
    return 0 if result["geometry_precheck_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
