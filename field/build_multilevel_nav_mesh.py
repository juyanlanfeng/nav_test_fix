#!/usr/bin/env python3
"""Build a tunnel-aware navigation manifold from a detailed source mesh.

Unlike a 2.5-D height map, this keeps every vertically separated surface layer
that has enough headroom for the robot.  Neighbouring layers are triangulated
only when their geometric slope is within the robot limit.  Use the high-detail
Gazebo visual STL as input so small ramp and tunnel components are not lost.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import trimesh
from trimesh.ray.ray_triangle import RayMeshIntersector


RMUC2026_REVERSED_NORMAL_REGIONS = [
    [-0.20, 0.90, 6.35, 7.60, -0.08, 0.10],
    [-0.90, 0.20, -7.60, -6.35, -0.08, 0.10],
]
# RMUC-specific simulated collision envelope is 0.215 m high.  Ten
# millimetres are reserved here for map/simulation discretization error.
RMUC2026_ROBOT_HEIGHT_M = 0.225
RMUC2026_LOW_CORRIDORS = [
    ("positive_y_low_tunnel", (-1.45, 5.95, 0.004), (-0.40, 5.95, 0.004)),
    ("negative_y_low_tunnel", (1.45, -5.95, 0.003), (0.40, -5.95, 0.003)),
    ("positive_y_reversed_seam", (-0.10, 6.20, 0.035), (-0.10, 7.55, -0.039)),
    ("negative_y_reversed_seam", (0.10, -6.20, 0.035), (0.10, -7.55, -0.039)),
]


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def cluster_ray_hits(z_values: np.ndarray, normal_z: np.ndarray, tolerance: float):
    """Cluster coincident hits and retain signed and orientation-free slope data.

    STEP assemblies exported as STL are not guaranteed to have consistent face
    winding.  In particular, one physical RMUC tunnel floor alternates between
    +Z and -Z normals.  ``max_abs_normal_z`` therefore describes geometric
    inclination, while ``max_upward_normal_z`` is retained for diagnostics.
    """
    if len(z_values) == 0:
        return []
    order = np.argsort(z_values)
    z_sorted = z_values[order]
    nz_sorted = normal_z[order]
    clusters: list[tuple[float, float, float]] = []
    start = 0
    for index in range(1, len(z_sorted) + 1):
        if index < len(z_sorted) and z_sorted[index] - z_sorted[index - 1] <= tolerance:
            continue
        block_z = z_sorted[start:index]
        block_nz = nz_sorted[start:index]
        # Median resists duplicate/near-coplanar triangle intersection noise.
        clusters.append(
            (
                float(np.median(block_z)),
                float(np.max(block_nz)),
                float(np.max(np.abs(block_nz))),
            )
        )
        start = index
    return clusters


def collect_surface_layers(
    mesh: trimesh.Trimesh,
    grid_m: float,
    max_slope_deg: float,
    robot_height_m: float,
    hit_merge_m: float,
    ray_batch: int,
    reversed_normal_regions: list[list[float]],
):
    bounds = mesh.bounds
    xmin = math.floor(bounds[0, 0] / grid_m) * grid_m
    ymin = math.floor(bounds[0, 1] / grid_m) * grid_m
    xmax = math.ceil(bounds[1, 0] / grid_m) * grid_m
    ymax = math.ceil(bounds[1, 1] / grid_m) * grid_m
    xs = np.arange(xmin, xmax + grid_m * 0.5, grid_m)
    ys = np.arange(ymin, ymax + grid_m * 0.5, grid_m)
    nx, ny = len(xs), len(ys)
    xx, yy = np.meshgrid(xs, ys)
    ray_xy = np.column_stack((xx.ravel(), yy.ravel()))
    ray_z = float(bounds[1, 2] + 1.0)
    intersector = RayMeshIntersector(mesh)
    slope_normal_z = math.cos(math.radians(max_slope_deg))

    layers_by_ray: list[list[float]] = [[] for _ in range(len(ray_xy))]
    all_cluster_counts = np.zeros(len(ray_xy), dtype=np.int16)
    reversed_walkable_hits = 0
    reversed_walkable_hits_rejected = 0
    ignored_vertical_headroom_hits = 0
    for batch_start in range(0, len(ray_xy), ray_batch):
        batch_end = min(batch_start + ray_batch, len(ray_xy))
        count = batch_end - batch_start
        origins = np.column_stack(
            (
                ray_xy[batch_start:batch_end],
                np.full(count, ray_z, dtype=np.float64),
            )
        )
        directions = np.tile((0.0, 0.0, -1.0), (count, 1))
        locations, ray_ids, triangle_ids = intersector.intersects_location(
            origins, directions, multiple_hits=True
        )
        if len(locations):
            order = np.argsort(ray_ids, kind="stable")
            locations = locations[order]
            ray_ids = ray_ids[order]
            triangle_ids = triangle_ids[order]
            split = np.flatnonzero(np.diff(ray_ids)) + 1
            loc_groups = np.split(locations, split)
            tri_groups = np.split(triangle_ids, split)
            grouped_ids = np.split(ray_ids, split)
            for loc_group, tri_group, id_group in zip(loc_groups, tri_groups, grouped_ids):
                local_ray = int(id_group[0])
                global_ray = batch_start + local_ray
                clusters = cluster_ray_hits(
                    loc_group[:, 2], mesh.face_normals[tri_group, 2], hit_merge_m
                )
                all_cluster_counts[global_ray] = len(clusters)
                for cluster_index, (
                    surface_z,
                    best_upward_normal_z,
                    max_abs_normal_z,
                ) in enumerate(clusters):
                    # Geometric slope is independent of triangle winding.  The
                    # source STL is a non-watertight 8666-body CAD assembly.
                    # Accept reversed winding only in an explicitly audited
                    # region: accepting it globally can turn the underside of
                    # a closed CAD solid into a fictitious driving surface.
                    if max_abs_normal_z < slope_normal_z:
                        continue
                    if best_upward_normal_z < slope_normal_z:
                        ray_x, ray_y = ray_xy[global_ray]
                        inside_audited_region = any(
                            xmin - 1e-9 <= ray_x <= xmax + 1e-9
                            and ymin - 1e-9 <= ray_y <= ymax + 1e-9
                            and zmin - 1e-9 <= surface_z <= zmax + 1e-9
                            for xmin, xmax, ymin, ymax, zmin, zmax in reversed_normal_regions
                        )
                        if not inside_audited_region:
                            reversed_walkable_hits_rejected += 1
                            continue

                    # A vertical wall can intersect a downward ray exactly on
                    # a CAD seam.  It is not a ceiling and must not reduce
                    # headroom.  Find the next higher approximately horizontal
                    # surface instead.  Its winding is deliberately ignored as
                    # well, because ceiling faces have the same STL issue.
                    next_higher_z = math.inf
                    for higher_z, _, higher_abs_normal_z in clusters[cluster_index + 1 :]:
                        if higher_abs_normal_z >= slope_normal_z:
                            next_higher_z = higher_z
                            break
                        ignored_vertical_headroom_hits += 1
                    headroom = next_higher_z - surface_z
                    if headroom + 1e-6 >= robot_height_m:
                        layers_by_ray[global_ray].append(surface_z)
                        if best_upward_normal_z < slope_normal_z:
                            reversed_walkable_hits += 1
        log(
            f"Vertical rays {batch_end}/{len(ray_xy)}; "
            f"navigation layers={sum(len(v) for v in layers_by_ray)}"
        )
    diagnostics = {
        "reversed_walkable_hits_retained": int(reversed_walkable_hits),
        "reversed_walkable_hits_rejected": int(reversed_walkable_hits_rejected),
        "vertical_headroom_hits_ignored": int(ignored_vertical_headroom_hits),
    }
    return xs, ys, layers_by_ray, all_cluster_counts, diagnostics


def reciprocal_nearest_pairs(
    heights_a: np.ndarray,
    heights_b: np.ndarray,
    max_delta_z: float,
) -> list[tuple[int, int]]:
    """Match two vertical columns one-to-one by reciprocal nearest height."""
    if len(heights_a) == 0 or len(heights_b) == 0:
        return []
    delta = np.abs(heights_a[:, None] - heights_b[None, :])
    nearest_b = np.argmin(delta, axis=1)
    nearest_a = np.argmin(delta, axis=0)
    return [
        (index_a, int(index_b))
        for index_a, index_b in enumerate(nearest_b)
        if int(nearest_a[int(index_b)]) == index_a
        and delta[index_a, int(index_b)] <= max_delta_z + 1e-9
    ]


def triangulate_layers(
    xs: np.ndarray,
    ys: np.ndarray,
    layers_by_ray: list[list[float]],
    max_slope_deg: float,
):
    """Triangulate tracked layers without joining nearby stacked surfaces."""
    nx, ny = len(xs), len(ys)
    vertices: list[tuple[float, float, float]] = []
    node_ids: list[np.ndarray] = []
    heights: list[np.ndarray] = []
    for ray_index, ray_heights in enumerate(layers_by_ray):
        iy, ix = divmod(ray_index, nx)
        ids = np.arange(
            len(vertices), len(vertices) + len(ray_heights), dtype=np.int64
        )
        node_ids.append(ids)
        height_array = np.asarray(ray_heights, dtype=np.float64)
        heights.append(height_array)
        vertices.extend(
            (float(xs[ix]), float(ys[iy]), float(height))
            for height in ray_heights
        )
    vertices_array = np.asarray(vertices, dtype=np.float64)
    if not len(vertices_array):
        raise RuntimeError("No navigation layers survived slope and headroom filtering")

    slope_normal_z = math.cos(math.radians(max_slope_deg))
    max_slope_tangent = math.tan(math.radians(max_slope_deg))
    faces: list[tuple[int, int, int]] = []

    def edge_pairs(ray_a: int, ray_b: int) -> set[tuple[int, int]]:
        if not len(heights[ray_a]) or not len(heights[ray_b]):
            return set()
        ay, ax = divmod(ray_a, nx)
        by, bx = divmod(ray_b, nx)
        horizontal_distance = math.hypot(xs[bx] - xs[ax], ys[by] - ys[ay])
        matches = reciprocal_nearest_pairs(
            heights[ray_a],
            heights[ray_b],
            horizontal_distance * max_slope_tangent,
        )
        return {
            (int(node_ids[ray_a][local_a]), int(node_ids[ray_b][local_b]))
            for local_a, local_b in matches
        }

    def add_matched_triangle(ray_a: int, ray_b: int, ray_c: int) -> None:
        pairs_ab = edge_pairs(ray_a, ray_b)
        pairs_bc = edge_pairs(ray_b, ray_c)
        pairs_ca = edge_pairs(ray_c, ray_a)
        if not pairs_ab or not pairs_bc or not pairs_ca:
            return
        b_to_c = {b: c for b, c in pairs_bc}
        c_to_a = {c: a for c, a in pairs_ca}
        for a, b in pairs_ab:
            c = b_to_c.get(b)
            if c is None or c_to_a.get(c) != a:
                continue
            points = vertices_array[[a, b, c]]
            cross = np.cross(points[1] - points[0], points[2] - points[0])
            length = float(np.linalg.norm(cross))
            if length <= 1e-10:
                continue
            if abs(cross[2]) / length + 1e-8 < slope_normal_z:
                continue
            faces.append((a, c, b) if cross[2] < 0.0 else (a, b, c))

    # Use a consistent diagonal and require a closed loop of three reciprocal
    # layer matches.  This prevents the Cartesian layer combinations used by
    # the old implementation from bridging a tunnel floor to its roof.
    for iy in range(ny - 1):
        row = iy * nx
        next_row = (iy + 1) * nx
        for ix in range(nx - 1):
            a = row + ix
            b = row + ix + 1
            c = next_row + ix
            d = next_row + ix + 1
            add_matched_triangle(a, b, d)
            add_matched_triangle(a, d, c)
    if not faces:
        raise RuntimeError("No triangles survived multi-layer triangulation")
    return vertices_array, np.asarray(faces, dtype=np.int64)


def component_labels_by_edge(vertices: np.ndarray, faces: np.ndarray):
    parent = np.arange(len(faces), dtype=np.int64)
    size = np.ones(len(faces), dtype=np.int64)

    def find(item: int) -> int:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = int(parent[item])
        return item

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
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
    projected_twice_area = np.abs(
        (triangles[:, 1, 0] - triangles[:, 0, 0])
        * (triangles[:, 2, 1] - triangles[:, 0, 1])
        - (triangles[:, 1, 1] - triangles[:, 0, 1])
        * (triangles[:, 2, 0] - triangles[:, 0, 0])
    )
    area = np.bincount(inverse, weights=0.5 * projected_twice_area)
    face_counts = np.bincount(inverse)
    return inverse, unique_roots, area, face_counts


def mesh_topology_stats(mesh: trimesh.Trimesh) -> dict[str, float | int]:
    faces = np.asarray(mesh.faces, dtype=np.int64)
    edges = np.sort(
        np.vstack((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])),
        axis=1,
    )
    _, edge_use_count = np.unique(edges, axis=0, return_counts=True)
    xy = np.round(np.asarray(mesh.vertices)[:, :2], decimals=6)
    face_normal_z = np.clip(np.asarray(mesh.face_normals)[:, 2], -1.0, 1.0)
    return {
        "boundary_edges": int(np.count_nonzero(edge_use_count == 1)),
        "nonmanifold_edges": int(np.count_nonzero(edge_use_count > 2)),
        "xy_locations_with_multiple_layers": int(len(xy) - len(np.unique(xy, axis=0))),
        "maximum_face_slope_deg": float(
            np.degrees(np.arccos(face_normal_z)).max(initial=0.0)
        ),
    }


def validate_low_corridors(
    mesh: trimesh.Trimesh,
    corridors: list[tuple[str, tuple[float, float, float], tuple[float, float, float]]],
    endpoint_radius_m: float,
    z_min_m: float = -0.08,
    z_max_m: float = 0.10,
) -> list[dict[str, object]]:
    """Prove audited corridors connect using only the lower navigation layer."""
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    low = (vertices[:, 2] >= z_min_m) & (vertices[:, 2] <= z_max_m)
    parent = np.arange(len(vertices), dtype=np.int64)

    def find(item: int) -> int:
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = int(parent[item])
        return item

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for a, b, c in faces[np.all(low[faces], axis=1)]:
        union(int(a), int(b))
        union(int(b), int(c))

    def nearest(target: tuple[float, float, float]):
        delta_xy = vertices[:, :2] - np.asarray(target[:2])
        distance_xy = np.linalg.norm(delta_xy, axis=1)
        candidates = np.flatnonzero(low & (distance_xy <= endpoint_radius_m))
        if not len(candidates):
            return None
        distance_xyz = np.linalg.norm(vertices[candidates] - np.asarray(target), axis=1)
        return int(candidates[int(np.argmin(distance_xyz))])

    results: list[dict[str, object]] = []
    for name, start, goal in corridors:
        start_vertex = nearest(start)
        goal_vertex = nearest(goal)
        connected = (
            start_vertex is not None
            and goal_vertex is not None
            and find(start_vertex) == find(goal_vertex)
        )
        results.append(
            {
                "name": name,
                "connected_on_lower_layer": bool(connected),
                "start_vertex": start_vertex,
                "goal_vertex": goal_vertex,
                "allowed_z_m": [z_min_m, z_max_m],
            }
        )
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "source_mesh",
        type=Path,
        help="high-detail source mesh; use the Gazebo *_visual.stl for RMUC2026",
    )
    parser.add_argument("output_ply", type=Path)
    # RMUC2026 has a 55 mm wide, 52.5 degree transition strip at each of
    # the two symmetric ramps.  A 0.1 m grid and a 35/50 degree limit erase
    # that strip.  The tunnel floor also needs the real low-profile body
    # clearance instead of the tutorial's sensor-tip height of 0.7 m.
    parser.add_argument("--grid-m", type=float, default=0.05)
    parser.add_argument("--max-slope-deg", type=float, default=55.0)
    parser.add_argument(
        "--rmuc2026-profile",
        action="store_true",
        help=(
            "Use the audited RMUC2026 tunnel regions and 0.225 m headroom "
            "profile (0.215 m collision envelope plus 0.010 m margin)"
        ),
    )
    parser.add_argument("--robot-height-m", type=float)
    parser.add_argument("--hit-merge-m", type=float, default=0.012)
    parser.add_argument("--ray-batch", type=int, default=5000)
    parser.add_argument(
        "--allow-reversed-region",
        action="append",
        nargs=6,
        type=float,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        default=[],
        help=(
            "Audited XYZ box in which slope-like -Z faces may be used as "
            "navigation surfaces; repeat for multiple regions"
        ),
    )
    parser.add_argument(
        "--min-component-area-m2",
        type=float,
        default=0.0,
        help="0 keeps only the largest component; positive keeps every component above this area",
    )
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    robot_height_m = (
        args.robot_height_m
        if args.robot_height_m is not None
        else RMUC2026_ROBOT_HEIGHT_M if args.rmuc2026_profile else 0.35
    )
    reversed_normal_regions = list(args.allow_reversed_region)
    if args.rmuc2026_profile:
        reversed_normal_regions.extend(RMUC2026_REVERSED_NORMAL_REGIONS)

    log(f"Loading source mesh: {args.source_mesh}")
    mesh = trimesh.load_mesh(args.source_mesh, process=True)
    log(f"Source mesh vertices={len(mesh.vertices)}, triangles={len(mesh.faces)}")
    xs, ys, layers, hit_counts, extraction_diagnostics = collect_surface_layers(
        mesh,
        args.grid_m,
        args.max_slope_deg,
        robot_height_m,
        args.hit_merge_m,
        args.ray_batch,
        reversed_normal_regions,
    )
    vertices, faces = triangulate_layers(xs, ys, layers, args.max_slope_deg)
    inverse, roots, areas, face_counts = component_labels_by_edge(vertices, faces)
    order = np.argsort(areas)[::-1]
    log(f"Raw multi-layer components={len(roots)}")
    for rank, component in enumerate(order[:10], 1):
        log(
            f"  component {rank}: projected_area={areas[component]:.3f} m^2, "
            f"triangles={face_counts[component]}"
        )
    if args.min_component_area_m2 > 0.0:
        selected_components = np.flatnonzero(areas >= args.min_component_area_m2)
    else:
        selected_components = np.asarray([order[0]])
    keep_faces = np.flatnonzero(np.isin(inverse, selected_components))
    output_mesh = trimesh.Trimesh(
        vertices=vertices, faces=faces[keep_faces], process=False
    )
    output_mesh.remove_unreferenced_vertices()
    output_mesh.merge_vertices(digits_vertex=6)
    output_mesh.remove_unreferenced_vertices()
    topology_stats = mesh_topology_stats(output_mesh)
    corridor_checks: list[dict[str, object]] = []
    if args.rmuc2026_profile:
        corridor_checks = validate_low_corridors(
            output_mesh, RMUC2026_LOW_CORRIDORS, endpoint_radius_m=args.grid_m * 1.5
        )
        failed_corridors = [
            check["name"]
            for check in corridor_checks
            if not check["connected_on_lower_layer"]
        ]
        if failed_corridors:
            raise RuntimeError(
                "RMUC2026 lower-layer corridor regression failed: "
                + ", ".join(str(name) for name in failed_corridors)
            )
    args.output_ply.parent.mkdir(parents=True, exist_ok=True)
    output_mesh.export(args.output_ply, file_type="ply")
    log(
        f"Wrote {args.output_ply}: vertices={len(output_mesh.vertices)}, "
        f"triangles={len(output_mesh.faces)}, bounds={output_mesh.bounds.tolist()}"
    )

    layer_count = np.asarray([len(item) for item in layers], dtype=np.int16)
    report = {
        "source_mesh": str(args.source_mesh.resolve()),
        "grid_resolution_m": args.grid_m,
        "grid_shape": [int(len(ys)), int(len(xs))],
        "max_slope_deg": args.max_slope_deg,
        "robot_height_m": robot_height_m,
        "hit_merge_m": args.hit_merge_m,
        "allow_reversed_regions": reversed_normal_regions,
        "surface_orientation_policy": "signed_upward_plus_audited_bidirectional_regions",
        "headroom_blocker_policy": "next_slope_like_surface_ignore_vertical_seams",
        "layer_matching_policy": "reciprocal_nearest_height",
        **extraction_diagnostics,
        "rays": int(len(layers)),
        "rays_with_geometry": int(np.count_nonzero(hit_counts)),
        "rays_with_navigation_surface": int(np.count_nonzero(layer_count)),
        "rays_with_multiple_navigation_layers": int(np.count_nonzero(layer_count > 1)),
        "maximum_navigation_layers_on_one_ray": int(layer_count.max(initial=0)),
        "raw_navigation_vertices": int(len(vertices)),
        "raw_navigation_triangles": int(len(faces)),
        "raw_components": int(len(roots)),
        "components_by_area": [
            {
                "projected_area_m2": float(areas[index]),
                "triangles": int(face_counts[index]),
            }
            for index in order[:20]
        ],
        "output_vertices": int(len(output_mesh.vertices)),
        "output_triangles": int(len(output_mesh.faces)),
        "output_bounds_m": output_mesh.bounds.tolist(),
        **topology_stats,
        "low_corridor_checks": corridor_checks,
    }
    report_path = args.report or args.output_ply.with_suffix(".multilevel.json")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    log(f"Wrote report: {report_path}")

    # Close the normal conversion pipeline automatically.  step_to_nav_maps.py
    # recreates conversion_metadata.json; without this update a clean rebuild
    # would produce a valid PLY but leave no record of its parameters or hash.
    conversion_base = args.output_ply.parent.parent
    metadata_path = conversion_base / "conversion_metadata.json"
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        try:
            source_name = str(args.source_mesh.resolve().relative_to(conversion_base.resolve()))
        except ValueError:
            source_name = str(args.source_mesh.resolve())
        try:
            output_name = str(args.output_ply.resolve().relative_to(conversion_base.resolve()))
        except ValueError:
            output_name = str(args.output_ply.resolve())
        output_hash = sha256(args.output_ply)
        generated_metadata = {
            "source_mesh": source_name,
            "canonical_file": output_name,
            "sha256": output_hash,
            "grid_resolution_m": args.grid_m,
            "max_slope_deg": args.max_slope_deg,
            "robot_height_m": robot_height_m,
            "hit_merge_m": args.hit_merge_m,
            "allow_reversed_regions": reversed_normal_regions,
            "surface_orientation_policy": report["surface_orientation_policy"],
            "headroom_blocker_policy": report["headroom_blocker_policy"],
            "layer_matching_policy": report["layer_matching_policy"],
            "vertices": int(len(output_mesh.vertices)),
            "triangles": int(len(output_mesh.faces)),
            "edge_connected_components": int(len(selected_components)),
            "bounds_m": output_mesh.bounds.tolist(),
            **topology_stats,
            "low_corridor_checks": corridor_checks,
        }
        previous_metadata = metadata.get("mesh_planner_multilevel", {})
        # Keep manually recorded feature validation only when the generated
        # bytes are identical.  Reusing it after a parameter/source change
        # would make conversion_metadata.json claim stale validation results.
        if previous_metadata.get("sha256") == output_hash:
            metadata["mesh_planner_multilevel"] = {
                **previous_metadata,
                **generated_metadata,
            }
        else:
            metadata["mesh_planner_multilevel"] = generated_metadata
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        log(f"Updated conversion metadata: {metadata_path}")


if __name__ == "__main__":
    main()
