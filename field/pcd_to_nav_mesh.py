#!/usr/bin/env python3
"""Build a MeshNav triangle mesh directly from an XYZ point cloud.

The conversion keeps vertically separated surfaces in the same XY column.  It
does not run Poisson reconstruction, because a watertight reconstruction can
close tunnel entrances and connect floors that must remain separate.

Supported PCD encodings are ``ascii`` and uncompressed ``binary``.  Extra
fields are accepted. Existing ``normal_x/y/z`` fields are used when valid;
otherwise normals are estimated after deterministic voxel downsampling.
"""

from __future__ import annotations

import argparse
import io
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial import cKDTree
import trimesh

from build_multilevel_nav_mesh import (
    component_labels_by_edge,
    mesh_topology_stats,
    triangulate_layers,
)
from conversion_metadata import (
    load_json_object,
    merge_hashed_section,
    relative_or_absolute,
    sha256_file,
    write_json_object,
)


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def read_pcd(path: Path) -> tuple[np.ndarray, np.ndarray | None, dict[str, object]]:
    """Read XYZ and optional normals from an ASCII or binary PCD file."""
    header: dict[str, list[str]] = {}
    header_bytes = 0
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA line")
            header_bytes += len(line)
            text = line.decode("ascii", errors="strict").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                payload = stream.read()
                break

    fields = header.get("FIELDS") or header.get("FIELD")
    if not fields:
        raise ValueError(f"{path}: missing FIELDS")
    sizes = [int(value) for value in header.get("SIZE", [])]
    types = header.get("TYPE", [])
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"{path}: FIELDS/SIZE/TYPE/COUNT lengths differ")
    point_count = int((header.get("POINTS") or header.get("WIDTH") or ["0"])[0])
    if point_count <= 0:
        raise ValueError(f"{path}: POINTS must be positive")
    encoding = header["DATA"][0].lower()
    offsets = np.cumsum([0] + counts[:-1]).tolist()
    trailing_padding_bytes = 0

    if encoding == "ascii":
        values = np.loadtxt(io.BytesIO(payload), dtype=np.float64, ndmin=2)
        expected_columns = sum(counts)
        if values.shape != (point_count, expected_columns):
            raise ValueError(
                f"{path}: expected {(point_count, expected_columns)} ASCII values, "
                f"got {values.shape}"
            )

        def field_values(name: str) -> np.ndarray:
            return values[:, offsets[fields.index(name)]]

    elif encoding == "binary":
        type_map = {
            ("F", 4): "<f4", ("F", 8): "<f8",
            ("I", 1): "<i1", ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
            ("U", 1): "<u1", ("U", 2): "<u2", ("U", 4): "<u4", ("U", 8): "<u8",
        }
        dtype_fields = []
        for name, size, kind, count in zip(fields, sizes, types, counts):
            scalar = type_map.get((kind.upper(), size))
            if scalar is None:
                raise ValueError(f"{path}: unsupported PCD scalar {kind}{size}")
            dtype_fields.append((name, scalar) if count == 1 else (name, scalar, (count,)))
        dtype = np.dtype(dtype_fields)
        expected_bytes = point_count * dtype.itemsize
        if len(payload) < expected_bytes:
            raise ValueError(
                f"{path}: binary payload is truncated: expected {expected_bytes} bytes, "
                f"got {len(payload)}"
            )
        trailing = payload[expected_bytes:]
        # PCL 1.12's binary converter may leave zero-filled bytes after the
        # POINTS-declared payload (observed with binary_compressed -> binary).
        # POINTS and the field layout define the data length, so harmless
        # zero/ASCII-whitespace padding is ignored. Any other trailing bytes
        # remain an error because they may be undeclared point records.
        allowed_padding = {0, 9, 10, 13, 32}
        if trailing and not set(trailing).issubset(allowed_padding):
            raise ValueError(
                f"{path}: found {len(trailing)} non-padding bytes after the "
                f"{expected_bytes}-byte POINTS payload"
            )
        trailing_padding_bytes = len(trailing)
        records = np.frombuffer(payload[:expected_bytes], dtype=dtype, count=point_count)

        def field_values(name: str) -> np.ndarray:
            value = records[name]
            return value if value.ndim == 1 else value[:, 0]

    elif encoding == "binary_compressed":
        raise ValueError(
            f"{path}: DATA binary_compressed is not supported; run "
            f"pcl_convert_pcd_ascii_binary {path} output.pcd 1 first"
        )
    else:
        raise ValueError(f"{path}: unsupported DATA encoding {encoding!r}")

    required = ("x", "y", "z")
    missing = [name for name in required if name not in fields]
    if missing:
        raise ValueError(f"{path}: missing fields {missing}")
    points = np.column_stack([field_values(name) for name in required]).astype(np.float64)
    normal_names = ("normal_x", "normal_y", "normal_z")
    normals = None
    if all(name in fields for name in normal_names):
        normals = np.column_stack([field_values(name) for name in normal_names]).astype(np.float64)
    metadata = {
        "encoding": encoding,
        "fields": fields,
        "points_declared": point_count,
        "header_bytes": header_bytes,
        "trailing_padding_bytes": trailing_padding_bytes,
    }
    return points, normals, metadata


def finite_rows(points: np.ndarray, normals: np.ndarray | None):
    """Drop invalid XYZ rows; invalid normals are retained as missing normals."""
    mask = np.all(np.isfinite(points), axis=1)
    if normals is not None:
        normals = normals.copy()
        normals[~np.all(np.isfinite(normals), axis=1)] = 0.0
    return points[mask], None if normals is None else normals[mask], int(np.count_nonzero(~mask))


def voxel_downsample(
    points: np.ndarray, normals: np.ndarray | None, voxel_m: float
) -> tuple[np.ndarray, np.ndarray | None]:
    """Average each occupied voxel deterministically, including supplied normals."""
    if voxel_m <= 0:
        return points, normals
    keys = np.floor(points / voxel_m).astype(np.int64)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    starts = np.r_[0, np.flatnonzero(np.any(np.diff(sorted_keys, axis=0), axis=1)) + 1]
    counts = np.diff(np.r_[starts, len(order)])
    sampled = np.add.reduceat(points[order], starts, axis=0) / counts[:, None]
    sampled_normals = None
    if normals is not None:
        sampled_normals = np.add.reduceat(normals[order], starts, axis=0) / counts[:, None]
    return sampled, sampled_normals


def normalize_valid_normals(normals: np.ndarray | None) -> tuple[np.ndarray | None, np.ndarray]:
    if normals is None:
        return None, np.zeros(0, dtype=bool)
    lengths = np.linalg.norm(normals, axis=1)
    valid = np.isfinite(lengths) & (lengths > 1e-6)
    result = np.zeros_like(normals)
    result[valid] = normals[valid] / lengths[valid, None]
    return result, valid


def estimate_normals(points: np.ndarray, neighbours: int, batch_size: int = 20000) -> np.ndarray:
    """Estimate unoriented local PCA normals in bounded-memory batches."""
    if len(points) < neighbours:
        raise ValueError(f"normal estimation needs at least {neighbours} points")
    tree = cKDTree(points)
    result = np.empty_like(points)
    for start in range(0, len(points), batch_size):
        stop = min(start + batch_size, len(points))
        _, indices = tree.query(points[start:stop], k=neighbours, workers=-1)
        neighbourhood = points[indices]
        centred = neighbourhood - neighbourhood.mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centred, centred) / neighbours
        _, vectors = np.linalg.eigh(covariance)
        result[start:stop] = vectors[:, :, 0]
        log(f"Estimated normals {stop}/{len(points)}")
    return result


def cluster_heights(values: np.ndarray, tolerance_m: float, minimum_points: int) -> list[float]:
    if len(values) == 0:
        return []
    values = np.sort(values)
    splits = np.flatnonzero(np.diff(values) > tolerance_m) + 1
    blocks = np.split(values, splits)
    return [float(np.median(block)) for block in blocks if len(block) >= minimum_points]


def point_layers(
    points: np.ndarray,
    normals: np.ndarray,
    grid_m: float,
    max_slope_deg: float,
    layer_merge_m: float,
    robot_height_m: float,
    min_points_per_cell: int,
):
    slope_limit = math.cos(math.radians(max_slope_deg))
    walkable = np.abs(normals[:, 2]) >= slope_limit
    selected = points[walkable]
    if not len(selected):
        raise RuntimeError("no points have a surface normal within the slope limit")
    xmin = math.floor(float(points[:, 0].min()) / grid_m) * grid_m
    ymin = math.floor(float(points[:, 1].min()) / grid_m) * grid_m
    xmax = math.ceil(float(points[:, 0].max()) / grid_m) * grid_m
    ymax = math.ceil(float(points[:, 1].max()) / grid_m) * grid_m
    xs = np.arange(xmin, xmax + grid_m * 0.5, grid_m)
    ys = np.arange(ymin, ymax + grid_m * 0.5, grid_m)
    ix = np.clip(np.rint((selected[:, 0] - xmin) / grid_m).astype(np.int64), 0, len(xs) - 1)
    iy = np.clip(np.rint((selected[:, 1] - ymin) / grid_m).astype(np.int64), 0, len(ys) - 1)
    cells = iy * len(xs) + ix
    order = np.argsort(cells, kind="stable")
    sorted_cells = cells[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_cells)) + 1]
    layers: list[list[float]] = [[] for _ in range(len(xs) * len(ys))]
    rejected_headroom = 0
    for start, stop in zip(starts, np.r_[starts[1:], len(order)]):
        heights = cluster_heights(
            selected[order[start:stop], 2], layer_merge_m, min_points_per_cell
        )
        retained = []
        for index, height in enumerate(heights):
            next_height = heights[index + 1] if index + 1 < len(heights) else math.inf
            if next_height - height + 1e-9 >= robot_height_m:
                retained.append(height)
            else:
                rejected_headroom += 1
        layers[int(sorted_cells[start])] = retained
    diagnostics = {
        "walkable_normal_points": int(np.count_nonzero(walkable)),
        "occupied_xy_cells": int(len(starts)),
        "headroom_layers_rejected": int(rejected_headroom),
    }
    return xs, ys, layers, diagnostics


def select_components(vertices: np.ndarray, faces: np.ndarray, minimum_area: float):
    inverse, roots, areas, face_counts = component_labels_by_edge(vertices, faces)
    order = np.argsort(areas)[::-1]
    selected = (
        np.flatnonzero(areas >= minimum_area)
        if minimum_area > 0
        else np.asarray([order[0]])
    )
    keep = np.flatnonzero(np.isin(inverse, selected))
    return faces[keep], roots, areas, face_counts, order, selected


def inspect(path: Path) -> dict[str, object]:
    points, normals, pcd = read_pcd(path)
    points, normals, removed = finite_rows(points, normals)
    _, normal_valid = normalize_valid_normals(normals)
    return {
        "source_pcd": str(path.resolve()),
        "source_sha256": sha256_file(path),
        **pcd,
        "finite_points": int(len(points)),
        "nonfinite_points_removed": removed,
        "valid_supplied_normals": int(np.count_nonzero(normal_valid)),
        "bounds": {"min": points.min(axis=0).tolist(), "max": points.max(axis=0).tolist()},
    }


def convert(args: argparse.Namespace) -> None:
    source_hash = sha256_file(args.pcd)
    points, normals, pcd_info = read_pcd(args.pcd)
    points, normals, nonfinite = finite_rows(points, normals)
    points = points * args.scale + np.asarray(args.translate, dtype=np.float64)
    points, normals = voxel_downsample(points, normals, args.voxel_m)
    normalized, valid_normals = normalize_valid_normals(normals)
    valid_fraction = float(np.mean(valid_normals)) if len(valid_normals) else 0.0
    estimated = valid_fraction < args.min_valid_normal_fraction
    if estimated:
        log(
            f"Supplied normal validity {valid_fraction:.1%}; estimating normals "
            f"from {len(points)} downsampled points"
        )
        normalized = estimate_normals(points, args.normal_k)
    else:
        log(f"Using supplied normals ({valid_fraction:.1%} valid after downsampling)")
        if not np.all(valid_normals):
            estimated_values = estimate_normals(points, args.normal_k)
            normalized[~valid_normals] = estimated_values[~valid_normals]

    xs, ys, layers, layer_diagnostics = point_layers(
        points, normalized, args.grid_m, args.max_slope_deg,
        args.layer_merge_m, args.robot_height_m, args.min_points_per_cell,
    )
    vertices, faces = triangulate_layers(xs, ys, layers, args.max_slope_deg)
    raw_face_count = len(faces)
    faces, roots, areas, face_counts, order, selected = select_components(
        vertices, faces, args.min_component_area_m2
    )
    log(
        f"Navigation surfaces: {len(roots)} edge-connected components; "
        f"largest {areas.max():.2f} m^2, keeping {len(selected)} "
        f"({len(faces)}/{raw_face_count} triangles). "
        "This PLY contains walkable candidates, not scene walls/ceiling."
    )
    if not len(selected):
        raise RuntimeError(
            "component filtering removed every triangle: largest component "
            f"{areas.max():.6f} m^2 is below --min-component-area-m2 "
            f"{args.min_component_area_m2:g}; only {raw_face_count} raw triangle(s) "
            f"formed with --grid-m {args.grid_m:g} and --min-points-per-cell "
            f"{args.min_points_per_cell}. Check point spacing and try a coarser "
            "grid before reducing the area threshold."
        )
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.remove_unreferenced_vertices()
    mesh.merge_vertices(digits_vertex=6)
    mesh.remove_unreferenced_vertices()
    args.output_ply.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(args.output_ply, file_type="ply")
    output_hash = sha256_file(args.output_ply)
    topology = mesh_topology_stats(mesh)
    layer_counts = np.asarray([len(value) for value in layers])
    report = {
        "generator": "field/pcd_to_nav_mesh.py",
        "source_pcd": str(args.pcd.resolve()),
        "source_pcd_sha256": source_hash,
        **pcd_info,
        "source_points_finite": int(pcd_info["points_declared"] - nonfinite),
        "nonfinite_points_removed": nonfinite,
        "scale": args.scale,
        "translation_m": args.translate,
        "voxel_m": args.voxel_m,
        "downsampled_points": int(len(points)),
        "normal_source": "estimated_local_pca" if estimated else "pcd_fields",
        "valid_supplied_normal_fraction": valid_fraction,
        "normal_k": args.normal_k,
        "grid_resolution_m": args.grid_m,
        "grid_shape": [int(len(ys)), int(len(xs))],
        "max_slope_deg": args.max_slope_deg,
        "layer_merge_m": args.layer_merge_m,
        "robot_height_m": args.robot_height_m,
        "min_points_per_cell": args.min_points_per_cell,
        "surface_orientation_policy": "unoriented_abs_normal_z",
        "headroom_policy": "next_point_surface_layer",
        "layer_matching_policy": "reciprocal_nearest_height",
        **layer_diagnostics,
        "cells_with_navigation_surface": int(np.count_nonzero(layer_counts)),
        "cells_with_multiple_layers": int(np.count_nonzero(layer_counts > 1)),
        "maximum_layers_per_cell": int(layer_counts.max(initial=0)),
        "raw_vertices": int(len(vertices)),
        "raw_triangles": int(raw_face_count),
        "raw_components": int(len(roots)),
        "raw_projected_area_m2": float(areas.sum()),
        "largest_component_area_m2": float(areas.max()),
        "selected_projected_area_m2": float(areas[selected].sum()),
        "components_by_area": [
            {"projected_area_m2": float(areas[index]), "triangles": int(face_counts[index])}
            for index in order[:20]
        ],
        "selected_components": [int(value) for value in selected],
        "min_component_area_m2": args.min_component_area_m2,
        "output_ply": str(args.output_ply.resolve()),
        "output_ply_sha256": output_hash,
        "output_vertices": int(len(mesh.vertices)),
        "output_triangles": int(len(mesh.faces)),
        "output_bounds_m": mesh.bounds.tolist(),
        "output_role": "candidate_walkable_surface_only",
        "meshnav_getpath_tested": False,
        **topology,
        "limitations": [
            "PCD has no free-space rays; missing points are unknown, not free space.",
            "Unoriented normals cannot by themselves distinguish a floor top from a ceiling underside.",
            "Closed-loop validation is required before deployment.",
        ],
    }
    report_path = args.report or args.output_ply.with_suffix(".pcd_to_mesh.json")
    write_json_object(report_path, report)
    log(
        f"Wrote {args.output_ply}: {len(mesh.vertices)} vertices, {len(mesh.faces)} triangles"
    )
    log(f"Wrote report: {report_path}")

    conversion_base = args.output_ply.parent.parent
    metadata_path = conversion_base / "conversion_metadata.json"
    if metadata_path.is_file():
        metadata = load_json_object(metadata_path)
        generated = {
            "generator": "field/pcd_to_nav_mesh.py",
            "source_pcd": relative_or_absolute(args.pcd, conversion_base),
            "source_pcd_sha256": source_hash,
            "canonical_file": relative_or_absolute(args.output_ply, conversion_base),
            "sha256": output_hash,
            "report": relative_or_absolute(report_path, conversion_base),
            "grid_resolution_m": args.grid_m,
            "max_slope_deg": args.max_slope_deg,
            "robot_height_m": args.robot_height_m,
        }
        merge_hashed_section(metadata, "pcd_to_mesh_navigation", generated, "sha256")
        write_json_object(metadata_path, metadata)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect_parser = commands.add_parser("inspect", help="validate header, fields and bounds")
    inspect_parser.add_argument("pcd", type=Path)
    inspect_parser.add_argument("--report", type=Path)

    convert_parser = commands.add_parser("convert", help="extract a layered MeshNav PLY")
    convert_parser.add_argument("pcd", type=Path)
    convert_parser.add_argument("output_ply", type=Path)
    convert_parser.add_argument("--report", type=Path)
    convert_parser.add_argument("--scale", type=float, default=1.0,
                                help="source coordinate units to metres (mm: 0.001)")
    convert_parser.add_argument("--translate", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                                metavar=("X", "Y", "Z"), help="translation after scaling [m]")
    convert_parser.add_argument("--voxel-m", type=float, default=0.025)
    convert_parser.add_argument("--normal-k", type=int, default=24)
    convert_parser.add_argument("--min-valid-normal-fraction", type=float, default=0.95)
    convert_parser.add_argument("--grid-m", type=float, default=0.10)
    convert_parser.add_argument("--max-slope-deg", type=float, default=55.0)
    convert_parser.add_argument("--layer-merge-m", type=float, default=0.04)
    convert_parser.add_argument("--robot-height-m", type=float, default=0.35)
    convert_parser.add_argument("--min-points-per-cell", type=int, default=1)
    convert_parser.add_argument("--min-component-area-m2", type=float, default=0.0,
                                help="0 keeps only the largest edge-connected component")
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if args.command != "convert":
        return
    positive = ("scale", "voxel_m", "grid_m", "layer_merge_m", "robot_height_m")
    for name in positive:
        if getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be positive")
    if args.normal_k < 3 or args.min_points_per_cell < 1:
        raise ValueError("--normal-k must be >= 3 and --min-points-per-cell >= 1")
    if not 0 <= args.max_slope_deg < 90:
        raise ValueError("--max-slope-deg must be in [0, 90)")
    if not 0 <= args.min_valid_normal_fraction <= 1:
        raise ValueError("--min-valid-normal-fraction must be in [0, 1]")


def main() -> int:
    args = build_parser().parse_args()
    validate_args(args)
    if args.command == "inspect":
        result = inspect(args.pcd)
        text = json.dumps(result, ensure_ascii=False, indent=2)
        print(text)
        if args.report:
            write_json_object(args.report, result)
    else:
        convert(args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
