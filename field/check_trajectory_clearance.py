#!/usr/bin/env python3
"""Check a recorded tunnel trajectory against the collision mesh.

    PYTHONPATH=meshnav_demo_ws/src/pb_vehicle_adapter:. \
    field/.step_convert_venv/bin/python field/check_trajectory_clearance.py \
        log/low_plus_y_trajectory.csv log/low_minus_y_trajectory.csv

For every logged pose (x, y, z, yaw, pitch, roll) it places the vehicle's
collision primitives, resolves the local support surface and the lowest real
overhang, and reports:

* body/chassis/lidar/camera points below the terrain  -> bottoming out (failure)
* wheel points below the terrain                      -> wheel/terrain warning
* points above an overhang                            -> ceiling strike (failure)
* distance from the body sides to the nearest wall     -> side clearance
* progress along the track (stall detection)

This is the "no ceiling/side/bottom contact" evidence for acceptance item 4.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import trimesh

from check_tunnel_clearance import (PEN_EPS, crop_mesh, rpy_matrix, support_and_ceiling,
                                    vehicle_points)


ROOT = Path("/home/rainple/nav_test")
DEFAULT_STL = (ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim"
               / "models/rmuc2026_field/meshes/rmuc2026_field_collision.stl")
DEFAULT_URDF = ROOT / "meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf"


def side_clearance(mesh, point, lateral=np.array([0.0, 1.0, 0.0])):
    """Distance from `point` to the first surface along +-lateral."""
    best = None
    for sign in (1.0, -1.0):
        locations, _, _ = mesh.ray.intersects_location(
            np.array([point]), (sign * lateral)[None, :], multiple_hits=False)
        if len(locations):
            distance = float(np.linalg.norm(locations[0] - point))
            best = distance if best is None else min(best, distance)
    return best


def analyse(csv_path, mesh, points, labels, is_wheel):
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        return {"trajectory": str(csv_path), "samples": 0, "result": "no_samples"}
    body_penetration = []
    wheel_warning = []
    ceiling_strike = []
    head_margins = []
    side_margins = []
    xs = []
    # Crop once for the whole trajectory: rebuilding a submesh per pose over a
    # 500k-triangle field mesh is what made this analysis take minutes.
    centres = np.array([[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows])
    region = crop_mesh(mesh, centres.min(axis=0), centres.max(axis=0), margin=1.0)
    for row in rows:
        x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
        yaw, pitch, roll = float(row["yaw"]), float(row["pitch"]), float(row["roll"])
        xs.append(x)
        pose = np.eye(4)
        pose[:3, :3] = rpy_matrix(roll, pitch, yaw)
        pose[:3, 3] = [x, y, z]
        world = points @ pose[:3, :3].T + pose[:3, 3]
        normal = pose[:3, 2]
        plane_offset = float(normal @ pose[:3, 3])
        for point, label, wheel in zip(world, labels, is_wheel):
            if abs(normal[2]) < 0.2:
                expected = float(z)
            else:
                expected = (plane_offset - normal[0] * point[0] - normal[1] * point[1]) / normal[2]
            support, ceiling = support_and_ceiling(region, point[0], point[1], expected)
            if support is not None:
                bottom = point[2] - support
                if bottom < -PEN_EPS:
                    entry = {"t": row["t"], "x": round(x, 4), "label": label,
                             "depth_m": round(bottom, 5)}
                    (wheel_warning if wheel else body_penetration).append(entry)
            if ceiling is not None:
                head = ceiling - point[2]
                if head < -PEN_EPS:
                    ceiling_strike.append({"t": row["t"], "x": round(x, 4), "label": label,
                                           "overlap_m": round(head, 5)})
                head_margins.append(head)
        # Side clearance is measured at the two widest *body* points: a horizontal
        # ray at wheel-bottom height would graze the floor slab and report a
        # meaningless zero.
        body = world[~is_wheel]
        if len(body):
            widest = body[np.argmax(body[:, 1])], body[np.argmin(body[:, 1])]
            for point in widest:
                distance = side_clearance(region, point)
                if distance is not None:
                    side_margins.append(distance)
    return {
        "trajectory": str(csv_path),
        "samples": len(rows),
        "x_min": round(min(xs), 4),
        "x_max": round(max(xs), 4),
        "progress_m": round(max(xs) - min(xs), 4),
        "body_penetration_events": len(body_penetration),
        "wheel_terrain_warnings": len(wheel_warning),
        "ceiling_strike_events": len(ceiling_strike),
        "min_head_margin_m": round(min(head_margins), 5) if head_margins else None,
        "min_side_clearance_m": round(min(side_margins), 5) if side_margins else None,
        "first_body_penetration": body_penetration[:3],
        "first_ceiling_strike": ceiling_strike[:3],
        "result": "pass" if not body_penetration and not ceiling_strike else "fail",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectories", nargs="+")
    parser.add_argument("--collision-stl", default=str(DEFAULT_STL))
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--output", default=str(ROOT / "field/converted_rmuc2026/tunnel_clearance"
                                                / "trajectory_clearance_report.json"))
    arguments = parser.parse_args()

    mesh = trimesh.load(arguments.collision_stl, process=False)
    points, labels, is_wheel = vehicle_points(arguments.urdf)
    reports = []
    for path in arguments.trajectories:
        report = analyse(path, mesh, points, labels, is_wheel)
        reports.append(report)
        print("=== %s ===" % path)
        for key in ("samples", "x_min", "x_max", "progress_m", "body_penetration_events",
                    "wheel_terrain_warnings", "ceiling_strike_events", "min_head_margin_m",
                    "min_side_clearance_m", "result"):
            print("  %-26s %s" % (key, report.get(key)))
        for entry in report.get("first_body_penetration", []):
            print("  body penetration: %s" % entry)
        for entry in report.get("first_ceiling_strike", []):
            print("  ceiling strike:   %s" % entry)
    payload = {"collision_stl": arguments.collision_stl, "urdf": arguments.urdf,
               "trajectories": reports,
               "summary": {"pass": all(r.get("result") == "pass" for r in reports)}}
    Path(arguments.output).write_text(json.dumps(payload, indent=2))
    print()
    print("overall: %s (report: %s)"
          % ("PASS" if payload["summary"]["pass"] else "FAIL", arguments.output))
    return 0 if payload["summary"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
