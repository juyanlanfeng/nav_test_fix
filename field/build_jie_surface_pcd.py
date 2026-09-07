#!/usr/bin/env python3
"""Build a deterministic JIE occupancy PCD from the source surface mesh.

The JIE importer interprets every XYZ point as an occupied OctoMap cell.  A
uniform random sample of a large CAD assembly does not guarantee that every
floor or wall voxel is represented: small tunnel floors can therefore acquire
holes even when the total point count looks large.  This tool rasterizes the
actual collision surface instead.

The default triangle/AABB method enumerates intersected cells, including thin
edges which finite point sampling can miss. Subdivision remains an explicit
legacy option. A navigation PLY is never used as occupancy input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess
import tempfile
import time

import numpy as np
import trimesh
from trimesh.remesh import subdivide_to_size

from build_component_collision_mesh import load_single_mesh
from conversion_metadata import (
    load_json_object,
    merge_hashed_section,
    relative_or_absolute,
    write_json_object,
)
from step_to_nav_maps import write_binary_pcd


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _subdivision_iterations(triangles: np.ndarray, max_edge: float) -> int:
    edges = np.concatenate(
        (
            triangles[:, 1] - triangles[:, 0],
            triangles[:, 2] - triangles[:, 1],
            triangles[:, 0] - triangles[:, 2],
        ),
        axis=0,
    )
    longest = float(np.sqrt(np.max(np.einsum("ij,ij->i", edges, edges))))
    if longest <= max_edge:
        return 0
    return int(math.ceil(math.log2(longest / max_edge))) + 1


def rasterize_surface_voxels(
    mesh: trimesh.Trimesh,
    pitch: float,
    min_z: float,
    max_z: float,
    chunk_faces: int,
    edge_factor: float = 2.0,
    quantization: str = "containing-cell",
) -> np.ndarray:
    """Return sorted keys containing densely sampled source surface points.

    Each chunk is represented as independent triangles.  This deliberately
    trades duplicate edge work for a bounded peak memory footprint; the final
    integer-key union removes all duplicates deterministically.
    """
    if pitch <= 0.0:
        raise ValueError("pitch must be positive")
    if max_z < min_z:
        raise ValueError("max_z must be >= min_z")
    if chunk_faces <= 0:
        raise ValueError("chunk_faces must be positive")
    if edge_factor <= 1.0:
        raise ValueError("edge_factor must be > 1 for denser-than-pitch sampling")
    if quantization not in ("containing-cell", "legacy-nearest"):
        raise ValueError("unsupported quantization policy")

    triangles = mesh.triangles
    overlaps_band = (triangles[:, :, 2].max(axis=1) >= min_z) & (
        triangles[:, :, 2].min(axis=1) <= max_z
    )
    selected = np.flatnonzero(overlaps_band)
    log(
        f"Rasterizing {len(selected)}/{len(triangles)} source triangles; "
        f"surface voxel={pitch:g} m, z=[{min_z:g}, {max_z:g}] m"
    )

    max_edge = pitch / edge_factor
    key_blocks: list[np.ndarray] = []
    for block_index, begin in enumerate(range(0, len(selected), chunk_faces), start=1):
        face_indices = selected[begin : begin + chunk_faces]
        block_triangles = triangles[face_indices]
        vertices = block_triangles.reshape((-1, 3))
        faces = np.arange(len(vertices), dtype=np.int64).reshape((-1, 3))
        max_iter = _subdivision_iterations(block_triangles, max_edge)
        subdivided, _ = subdivide_to_size(
            vertices,
            faces,
            max_edge=max_edge,
            max_iter=max_iter,
        )
        in_band = (subdivided[:, 2] >= min_z) & (subdivided[:, 2] <= max_z)
        # OctoMap cells have boundaries at k*pitch, not (k-0.5)*pitch.
        # floor(sample/pitch), followed by the cell-centre encoding below,
        # keeps each sample inside its occupied cell. The historical rint
        # policy followed by +0.5 displaced the voxel grid by half a cell.
        # Preserve it only as an explicit reproduction option for old maps.
        quantize = np.floor if quantization == "containing-cell" else np.rint
        keys = quantize(subdivided[in_band] / pitch).astype(np.int32)
        if len(keys):
            key_blocks.append(np.unique(keys, axis=0))

        if block_index == 1 or block_index % 25 == 0 or begin + chunk_faces >= len(selected):
            log(
                f"  chunks={block_index}, faces={min(begin + chunk_faces, len(selected))}, "
                f"chunk_unique_keys={len(key_blocks[-1]) if key_blocks else 0}"
            )

    if not key_blocks:
        raise RuntimeError("surface rasterization produced no occupied voxels")
    log(f"Merging {sum(len(block) for block in key_blocks)} per-chunk voxel keys")
    keys = np.unique(np.concatenate(key_blocks, axis=0), axis=0)
    # np.unique sorts lexicographically, making PCD byte order reproducible.
    return keys


def update_conversion_metadata(args: argparse.Namespace, report: dict) -> bool:
    """Record the canonical JIE artifact when it lives in a conversion tree."""
    if args.output.parent.name != "jie_nav":
        return False
    conversion_base = args.output.parent.parent
    metadata_path = conversion_base / "conversion_metadata.json"
    if not metadata_path.is_file():
        return False

    generated = {
        "source_mesh": relative_or_absolute(args.source, conversion_base),
        "canonical_file": relative_or_absolute(args.output, conversion_base),
        "sha256": report["output_sha256"],
        "points": report["output_points"],
        "file_size_bytes": report["output_file_size_bytes"],
        "surface_voxel_m": report["surface_voxel_m"],
        "rasterizer": report.get("rasterizer", "subdivision"),
        "subdivision_edge_factor": report["subdivision_edge_factor"],
        "maximum_subdivided_edge_m": report["maximum_subdivided_edge_m"],
        "z_band_m": report["z_band_m"],
        "coordinate_policy": report["output_coordinate_policy"],
        "generator": "field/build_jie_surface_pcd.py",
        "occupancy_semantics": (
            "deterministic source-surface occupied cells; never the "
            "Mesh navigation PLY"
        ),
    }
    if args.report is not None:
        generated["generation_report"] = relative_or_absolute(
            args.report, conversion_base
        )

    metadata = load_json_object(metadata_path)
    merge_hashed_section(metadata, "jie_nav_surface_pcd", generated, "sha256")
    write_json_object(metadata_path, metadata)
    log(f"Updated conversion metadata: {metadata_path}")
    return True


def build(args: argparse.Namespace) -> dict:
    started = time.monotonic()
    log(f"Loading source surface: {args.source}")
    mesh = load_single_mesh(args.source)
    source_hash = sha256(args.source)
    if args.method == "triangle-box":
        if args.quantization != "containing-cell":
            raise ValueError("triangle-box requires containing-cell coordinates")
        keys = rasterize_triangle_boxes(
            args.source, args.surface_voxel_m, args.min_z, args.max_z
        )
    else:
        keys = rasterize_surface_voxels(
            mesh, args.surface_voxel_m, args.min_z, args.max_z,
            args.chunk_faces, args.edge_factor, args.quantization,
        )
    # Write cell centres, never lattice boundaries.  Boundary coordinates such
    # as -1.45 or 0.25 can round to the adjacent OctoMap key after float32 PCD
    # serialization; the half-cell offset makes coord-to-key stable for both
    # positive and negative coordinates.
    points = (keys.astype(np.float64) + 0.5) * args.surface_voxel_m

    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_binary_pcd(args.output, points)
    elapsed = time.monotonic() - started
    report = {
        "source": str(args.source.resolve()),
        "source_sha256": source_hash,
        "source_vertices": int(len(mesh.vertices)),
        "source_triangles": int(len(mesh.faces)),
        "method": "exact triangle/AABB surface intersection" if args.method == "triangle-box" else (
            "chunked deterministic triangle subdivision with maximum edge "
            f"{args.surface_voxel_m / args.edge_factor:g} m; {args.quantization} "
            "sorted unique surface-cell centres"
        ),
        "surface_voxel_m": args.surface_voxel_m,
        "rasterizer": args.method,
        "subdivision_edge_factor": args.edge_factor if args.method == "subdivision" else None,
        "maximum_subdivided_edge_m": (
            args.surface_voxel_m / args.edge_factor if args.method == "subdivision" else None
        ),
        "quantization": args.quantization,
        "output_coordinate_policy": (
            "(floor(surface_xyz / pitch) + 0.5) * pitch"
            if args.quantization == "containing-cell"
            else "(nearest_lattice_key + 0.5) * surface_voxel_m"
        ),
        "z_band_m": [args.min_z, args.max_z],
        "output": str(args.output.resolve()),
        "output_points": int(len(points)),
        "output_bounds_m": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
        "output_sha256": sha256(args.output),
        "output_file_size_bytes": args.output.stat().st_size,
        "generation_seconds": elapsed,
        "generator_numpy_version": np.__version__,
        "generator_trimesh_version": trimesh.__version__,
        "occupancy_semantics": (
        "source surface cells for PCD-to-OctoMap; not a traversability mesh"
        ),
    }
    log(
        f"Wrote {args.output}: {len(points)} points, "
        f"sha256={report['output_sha256']}, elapsed={elapsed:.1f}s"
    )
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        log(f"Wrote report: {args.report}")
    update_conversion_metadata(args, report)
    return report


def rasterize_triangle_boxes(source: Path, pitch: float, min_z: float, max_z: float) -> np.ndarray:
    """Enumerate cells intersecting source triangles, without sampling holes.

    The small native helper uses the separating-axis theorem and is compiled
    in a temporary directory; no binary or generated source is installed.
    """
    with tempfile.TemporaryDirectory(prefix="jie_surface_raster_") as temporary:
        binary = Path(temporary) / "surface_voxelizer"
        output = Path(temporary) / "keys.bin"
        subprocess.run([
            "g++", "-std=c++17", "-O3", str(Path(__file__).with_name("surface_voxelizer.cpp")),
            "-o", str(binary),
        ], check=True)
        subprocess.run([
            str(binary), str(source), str(output), str(pitch), str(min_z), str(max_z),
        ], check=True)
        keys = np.fromfile(output, dtype="<i4").reshape((-1, 3))
        if not len(keys):
            raise RuntimeError("no surface voxels in the requested band")
        return keys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="source surface mesh (binary STL for triangle-box)")
    parser.add_argument("output", type=Path, help="binary XYZ PCD to create")
    parser.add_argument("--report", type=Path, help="optional JSON report")
    parser.add_argument(
        "--surface-voxel-m",
        type=float,
        default=0.04,
        help="deterministic source-surface voxel pitch (default: 0.04 m)",
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=-0.08,
        help="lowest robot-relevant surface height to retain",
    )
    parser.add_argument(
        "--max-z",
        type=float,
        default=0.90,
        help="highest robot-relevant surface height to retain",
    )
    parser.add_argument("--chunk-faces", type=int, default=500)
    parser.add_argument("--edge-factor", type=float, default=1.1)
    parser.add_argument(
        "--method", choices=["triangle-box", "subdivision"], default="triangle-box",
        help="Exact binary-STL triangle/cell intersection (requires g++), or legacy sampling",
    )
    parser.add_argument(
        "--quantization", choices=["containing-cell", "legacy-nearest"],
        default="containing-cell",
        help="Use actual OctoMap cell boundaries; legacy-nearest only reproduces old maps",
    )
    parser.add_argument(
        "--force", action="store_true", help="allow replacing an existing PCD"
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if not args.source.is_file():
        parser.error(f"source file does not exist: {args.source}")
    if args.output.exists() and not args.force:
        parser.error(f"output already exists (use --force): {args.output}")
    try:
        build(args)
    except Exception as error:
        log(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
