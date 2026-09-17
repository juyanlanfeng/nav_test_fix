#!/usr/bin/env python3
"""Reproduce MeshNav's mesh layers offline at the RMUC tunnel positions.

MeshNav computes its layers from the navigation mesh inside `mbf_mesh_nav`
(`HeightDiffLayer` -> `lvr2::calcVertexHeightDifferences`, `BorderLayer` ->
`lvr2::calcBorderCosts`), then the inflation layers expand the lethal sets.  A
planner failure such as "Predecessor of the goal is not set" means the *goal
vertex* ended up with a cost above `mesh_planner.cost_limit`, so knowing the raw
layer values at the tunnel floor is what tells us which layer blocks the tunnel.

This tool re-implements the two geometry-based layers on the same PLY so the
question can be answered without runtime layer queries:

* height difference: geodesic (mesh edge) neighbourhood, limited by the distance
  *projected onto the seed vertex's normal plane* (lvr2 semantics, radius 0.2 m
  in `mbf_mesh_nav.yaml`), height diff = max - min of `normal . position`;
* border: vertices that touch an open (border) edge of the mesh surface.

Usage:
    PYTHONPATH=meshnav_demo_ws/src/pb_vehicle_adapter \
    field/.step_convert_venv/bin/python field/check_mesh_layers.py [--radius 0.2]
"""
import argparse
import json
import math
from collections import defaultdict, deque
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path("/home/rainple/nav_test")
DEFAULT_PLY = (ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials/mesh_navigation_tutorials"
               / "maps/rmuc2026_field.ply")
DEFAULT_OUT = ROOT / "field/converted_rmuc2026/tunnel_clearance/mesh_layers_report.json"

# Query points: both tunnel floors, the tunnel roofs, and reference points on the
# open field floor far away from any structure.
POINTS = [
    ("+Y tunnel floor (west end)", -1.45, 5.95),
    ("+Y tunnel floor (centre)", -1.00, 5.95),
    ("+Y tunnel floor (east end)", -0.70, 5.95),
    ("+Y outside east", -0.40, 5.95),
    ("-Y tunnel floor (east end)", 1.45, -5.95),
    ("-Y tunnel floor (centre)", 1.00, -5.95),
    ("-Y tunnel floor (west end)", 0.70, -5.95),
    ("-Y outside west", 0.40, -5.95),
    ("open floor reference", -11.90, -4.40),
    ("open floor reference 2", 3.00, 2.00),
]


def vertex_adjacency(mesh):
    adjacency = defaultdict(set)
    for face in mesh.faces:
        a, b, c = int(face[0]), int(face[1]), int(face[2])
        adjacency[a].update((b, c))
        adjacency[b].update((a, c))
        adjacency[c].update((a, b))
    return adjacency


def border_vertices(mesh):
    """Vertices on an open edge: an edge used by exactly one triangle."""
    counts = defaultdict(int)
    for face in mesh.faces:
        a, b, c = int(face[0]), int(face[1]), int(face[2])
        for edge in ((a, b), (b, c), (c, a)):
            counts[tuple(sorted(edge))] += 1
    border = set()
    for (u, v), count in counts.items():
        if count == 1:
            border.add(u)
            border.add(v)
    return border


def height_difference(mesh, adjacency, normals, index, radius):
    """lvr2::calcVertexHeightDifferences for one vertex (geodesic, plane-limited)."""
    normal = normals[index]
    if not np.all(np.isfinite(normal)):
        normal = np.array([0.0, 0.0, 1.0])
    position = mesh.vertices[index]
    projected = position - normal * float(position @ normal)
    radius_squared = radius * radius
    visited = {index}
    queue = deque([index])
    heights = [float(normal @ position)]
    while queue:
        current = queue.popleft()
        for neighbour in adjacency[current]:
            if neighbour in visited:
                continue
            neighbour_position = mesh.vertices[neighbour]
            neighbour_projected = neighbour_position - normal * float(neighbour_position @ normal)
            if float(((neighbour_projected - projected) ** 2).sum()) < radius_squared:
                visited.add(neighbour)
                heights.append(float(normal @ neighbour_position))
                queue.append(neighbour)
    return max(heights) - min(heights), len(heights)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ply", default=str(DEFAULT_PLY))
    parser.add_argument("--radius", type=float, default=0.2,
                        help="height_diff.radius from mbf_mesh_nav.yaml [m]")
    parser.add_argument("--threshold", type=float, default=0.2,
                        help="height_diff.threshold from mbf_mesh_nav.yaml [m]")
    parser.add_argument("--output", default=str(DEFAULT_OUT))
    arguments = parser.parse_args()

    mesh = trimesh.load(arguments.ply, process=False)
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    mesh.fix_normals()
    adjacency = vertex_adjacency(mesh)
    normals = np.asarray(mesh.vertex_normals)
    border = border_vertices(mesh)
    print("mesh: %d vertices, %d faces, %d border vertices, radius %.2f, threshold %.2f"
          % (len(mesh.vertices), len(mesh.faces), len(border), arguments.radius,
             arguments.threshold))

    rows = []
    for label, x, y in POINTS:
        xy = np.asarray(mesh.vertices)[:, :2]
        distance = np.linalg.norm(xy - np.array([x, y]), axis=1)
        index = int(np.argmin(distance))
        value, count = height_difference(mesh, adjacency, normals, index, arguments.radius)
        rows.append({
            "label": label,
            "query": [x, y],
            "vertex": [round(float(value_), 4) for value_ in mesh.vertices[index]],
            "vertex_snap_m": round(float(distance[index]), 4),
            "neighbourhood_vertices": count,
            "height_diff_m": round(float(value), 4),
            "height_diff_above_threshold": bool(value > arguments.threshold),
            "border_vertex": index in border,
        })
        print("  %-28s vertex %s (snap %.3f m)  height_diff %.4f m  %s%s"
              % (label, rows[-1]["vertex"], rows[-1]["vertex_snap_m"], value,
                 "LETHAL" if value > arguments.threshold else "ok",
                 "  [border]" if index in border else ""))

    payload = {"ply": arguments.ply, "radius_m": arguments.radius,
               "threshold_m": arguments.threshold,
               "border_vertices": len(border), "points": rows}
    Path(arguments.output).write_text(json.dumps(payload, indent=2))
    print("report: %s" % arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
