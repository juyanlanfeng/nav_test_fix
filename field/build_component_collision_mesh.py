#!/usr/bin/env python3
"""Build a Gazebo collision mesh without decimating driveable CAD features.

The RMUC field is made of many disconnected CAD shells.  Global quadric
decimation gives small ramps and tunnel surfaces very little weight and can
erase them even when the overall triangle count looks generous.  This tool
keeps whole connected components instead:

* every component reaching the robot-clearance band is mandatory;
* complete components up to ``optional_max_z`` are ranked by projected XY
  area per triangle and added while the face budget permits;
* no selected component is decimated.
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import time

import numpy as np
from scipy import sparse
from scipy.sparse import csgraph
import trimesh

from conversion_metadata import (
    load_json_object,
    merge_hashed_object,
    write_json_object,
)


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_single_mesh(path: Path) -> trimesh.Trimesh:
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            raise RuntimeError(f"No geometry found in {path}")
        loaded = trimesh.util.concatenate(tuple(loaded.geometry.values()))
    if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
        raise RuntimeError(f"No triangle mesh found in {path}")
    # STEP tessellation writes coincident vertices independently for adjacent
    # CAD faces.  Merge at micrometre precision so connectivity represents the
    # physical shells rather than the source face boundaries.
    loaded.merge_vertices(digits_vertex=6)
    loaded.remove_unreferenced_vertices()
    return loaded


def projected_xy_area(mesh: trimesh.Trimesh) -> float:
    triangles = mesh.triangles
    twice_area = np.abs(
        (triangles[:, 1, 0] - triangles[:, 0, 0])
        * (triangles[:, 2, 1] - triangles[:, 0, 1])
        - (triangles[:, 1, 1] - triangles[:, 0, 1])
        * (triangles[:, 2, 0] - triangles[:, 0, 0])
    )
    return float(0.5 * np.sum(twice_area, dtype=np.float64))


def projected_xy_area_faces(mesh: trimesh.Trimesh) -> np.ndarray:
    """Return each triangle's unsigned projected area in the XY plane."""
    triangles = mesh.triangles
    return 0.5 * np.abs(
        (triangles[:, 1, 0] - triangles[:, 0, 0])
        * (triangles[:, 2, 1] - triangles[:, 0, 1])
        - (triangles[:, 1, 1] - triangles[:, 0, 1])
        * (triangles[:, 2, 0] - triangles[:, 0, 0])
    )


def build_component_collision(
    source: trimesh.Trimesh,
    face_budget: int,
    mandatory_max_z: float,
    optional_max_z: float,
) -> tuple[trimesh.Trimesh, dict]:
    # Trimesh's ``split`` uses shared-edge face adjacency.  CAD shells often
    # meet only at a vertex after tessellation, so that definition fragments
    # one physical object into many pieces.  Label the vertex graph instead;
    # all three vertices of a triangle then necessarily share one label.
    edges = source.edges_unique
    vertex_graph = sparse.coo_matrix(
        (
            np.ones(len(edges), dtype=np.uint8),
            (edges[:, 0], edges[:, 1]),
        ),
        shape=(len(source.vertices), len(source.vertices)),
    ).tocsr()
    component_count, vertex_labels = csgraph.connected_components(
        vertex_graph, directed=False, return_labels=True
    )
    face_labels = vertex_labels[source.faces[:, 0]]
    face_counts = np.bincount(face_labels, minlength=component_count)
    log(f"Found {component_count} vertex-connected CAD components")

    component_min_z = np.full(component_count, np.inf, dtype=np.float64)
    np.minimum.at(component_min_z, vertex_labels, source.vertices[:, 2])
    xy_area_faces = projected_xy_area_faces(source)
    component_xy_area = np.bincount(
        face_labels, weights=xy_area_faces, minlength=component_count
    )
    component_surface_area = np.bincount(
        face_labels, weights=source.area_faces, minlength=component_count
    )
    component_score = np.divide(
        component_xy_area,
        face_counts,
        out=np.zeros(component_count, dtype=np.float64),
        where=face_counts > 0,
    )

    mandatory = np.flatnonzero(
        (face_counts > 0) & (component_min_z <= mandatory_max_z)
    )
    mandatory_faces = int(np.sum(face_counts[mandatory], dtype=np.int64))
    if mandatory_faces > face_budget:
        raise RuntimeError(
            f"Mandatory low components require {mandatory_faces} faces, which "
            f"exceeds the {face_budget} face budget"
        )

    optional = np.flatnonzero(
        (face_counts > 0)
        & (component_min_z > mandatory_max_z)
        & (component_min_z <= optional_max_z)
    )
    optional = optional[
        np.argsort(-component_score[optional], kind="stable")
    ]

    selected = mandatory.tolist()
    selected_faces = mandatory_faces
    for component_index in optional:
        component_faces = int(face_counts[component_index])
        if selected_faces + component_faces <= face_budget:
            selected.append(int(component_index))
            selected_faces += component_faces

    selected_mask = np.zeros(component_count, dtype=bool)
    selected_mask[selected] = True
    collision = trimesh.Trimesh(
        vertices=source.vertices.copy(),
        faces=source.faces[selected_mask[face_labels]].copy(),
        process=False,
    )
    collision.remove_unreferenced_vertices()
    stats = {
        "method": (
            "preserve every complete vertex-connected component with min_z <= "
            f"{mandatory_max_z:g} m; add complete components with min_z <= "
            f"{optional_max_z:g} m in descending projected-XY-area-per-face "
            f"order while total <= {face_budget}; no triangle decimation"
        ),
        "source_faces": int(len(source.faces)),
        "output_faces": int(len(collision.faces)),
        "face_budget": face_budget,
        "source_components": int(component_count),
        "mandatory_components": len(mandatory),
        "added_components": len(selected) - len(mandatory),
        "output_components": len(selected),
        "mandatory_max_z_m": mandatory_max_z,
        "optional_max_z_m": optional_max_z,
        "source_surface_area_m2": float(np.sum(component_surface_area)),
        "output_surface_area_m2": float(np.sum(component_surface_area[selected])),
        "source_projected_xy_area_m2": float(np.sum(component_xy_area)),
        "output_projected_xy_area_m2": float(np.sum(component_xy_area[selected])),
        "bounds_m": collision.bounds.tolist(),
        "selected_component_indices": selected,
    }
    stats["surface_area_retention"] = (
        stats["output_surface_area_m2"] / stats["source_surface_area_m2"]
    )
    stats["projected_xy_area_retention"] = (
        stats["output_projected_xy_area_m2"]
        / stats["source_projected_xy_area_m2"]
    )
    return collision, stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="High-detail visual STL/mesh")
    parser.add_argument("output", type=Path, help="Collision STL to create")
    parser.add_argument("--report", type=Path, help="Optional JSON report path")
    parser.add_argument("--face-budget", type=int, default=500000)
    parser.add_argument("--mandatory-max-z", type=float, default=0.35)
    parser.add_argument("--optional-max-z", type=float, default=1.0)
    parser.add_argument(
        "--force", action="store_true", help="Allow replacing an existing output"
    )
    args = parser.parse_args()

    if args.face_budget <= 0:
        parser.error("--face-budget must be positive")
    if args.optional_max_z < args.mandatory_max_z:
        parser.error("--optional-max-z must be >= --mandatory-max-z")
    if not args.source.is_file():
        parser.error(f"source file does not exist: {args.source}")
    if args.output.exists() and not args.force:
        parser.error(f"output already exists (use --force): {args.output}")

    log(f"Loading {args.source}")
    source = load_single_mesh(args.source)
    collision, report = build_component_collision(
        source,
        args.face_budget,
        args.mandatory_max_z,
        args.optional_max_z,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    collision.export(args.output, file_type="stl")
    report.update(
        {
            "source": str(args.source.resolve()),
            "output": str(args.output.resolve()),
            "source_sha256": sha256(args.source),
            "output_sha256": sha256(args.output),
            "output_file_size_bytes": args.output.stat().st_size,
        }
    )
    log(
        f"Wrote {args.output}: {report['output_faces']} faces, "
        f"{report['output_components']} complete components"
    )

    if args.report:
        report = merge_hashed_object(
            load_json_object(args.report),
            report,
            ("output_sha256", "sha256"),
        )
        write_json_object(args.report, report)
        log(f"Wrote report {args.report}")


if __name__ == "__main__":
    main()
