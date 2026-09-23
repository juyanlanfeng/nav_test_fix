#!/usr/bin/env python3
"""Explain one "ceiling strike" of field/check_trajectory_clearance.py.

The replay reports a strike when a vehicle point lies above a surface that has a
downward normal.  This tool prints, for every collision point of the offending
primitive, the vertical ray result at that (x, y): the support height it picks,
the overhang it finds and how far the point is from it, so a real low overhang
can be told apart from a mis-resolved support surface (the replay alone cannot
say which of the two happened).

    PYTHONPATH=meshnav_demo_ws/src/pb_vehicle_adapter:. \
    field/.step_convert_venv/bin/python field/explain_ceiling_strike.py \
        log/tunnel_nav_meshnav_minus_y_227_trajectory.csv 0.6073
"""
import argparse
import csv
import math
from pathlib import Path

import numpy as np
import trimesh

from check_tunnel_clearance import (CEILING_MIN_GAP, hits_below, rpy_matrix,
                                    support_and_ceiling, vehicle_points)


ROOT = Path("/home/rainple/nav_test")
DEFAULT_STL = (ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim"
               / "models/rmuc2026_field/meshes/rmuc2026_field_collision.stl")
DEFAULT_URDF = ROOT / "meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory")
    parser.add_argument("x", type=float, help="station of the pose to explain")
    parser.add_argument("--stl", type=Path, default=DEFAULT_STL)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--label", default="rear_right_wheel")
    arguments = parser.parse_args()

    rows = list(csv.DictReader(open(arguments.trajectory)))
    pose = min(rows, key=lambda row: abs(float(row["x"]) - arguments.x))
    print("pose: x=%s y=%s z=%s yaw=%s pitch=%s roll=%s"
          % (pose["x"], pose["y"], pose["z"], pose.get("yaw"), pose.get("pitch"), pose.get("roll")))
    mesh = trimesh.load(arguments.stl, force="mesh")
    points, labels, _is_wheel = vehicle_points(arguments.urdf)
    rotation = rpy_matrix(float(pose.get("roll") or 0.0), float(pose.get("pitch") or 0.0),
                          float(pose.get("yaw") or 0.0))
    centre = np.array([float(pose["x"]), float(pose["y"]), float(pose["z"])])
    world = points @ rotation.T + centre
    selected = [index for index, label in enumerate(labels) if arguments.label in label]
    print("primitive points: %d" % len(selected))
    for index in selected:
        x, y, z = world[index]
        heights = hits_below(mesh, x, y, 4.0)
        support, ceiling = support_and_ceiling(mesh, x, y, float(pose["z"]))
        above = [round(height - z, 4) for height, _ in heights if height > z]
        strike = ceiling is not None and z > ceiling - 1.0e-9
        print("  %-28s%s x=%+.4f y=%+.4f z=%+.4f" % (labels[index],
              " STRIKE" if strike else "", x, y, z))
        print("       ray hits (height, normal_z): %s"
              % [(round(height, 4), round(normal, 2)) for height, normal in heights][:6])
        print("       support=%s ceiling=%s point_above_support=%s%s"
              % (None if support is None else round(support, 4),
                 None if ceiling is None else round(ceiling, 4),
                 None if support is None else round(z - support, 4),
                 "" if not above else "  surfaces above the point: %s" % above[:4]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
