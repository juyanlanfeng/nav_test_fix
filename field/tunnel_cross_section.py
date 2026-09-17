#!/usr/bin/env python3
"""Offline cross-sections of the RMUC2026 navigation mesh at the two tunnels.

The MeshNav planner works on vertices, so the question "does the low vehicle fit
through the tunnel as far as the planner is concerned" is answered by the *mesh*
cross-section, not by the collision STL: which surfaces exist at a given station,
how wide the floor is and where the roof sits above it.

    field/.step_convert_venv/bin/python field/tunnel_cross_section.py <cache.h5> \
        --stations 0.70,0.90,1.10,1.30 --sign -1
"""
import argparse

import h5py
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cache")
    parser.add_argument("--stations", default="0.70,0.90,1.10,1.30",
                        help="|x| of the stations to cut")
    parser.add_argument("--sign", type=float, default=-1.0,
                        help="sign of the tunnel's y coordinate (and of |x| when --x-sign is omitted)")
    parser.add_argument("--x-sign", type=float, default=None,
                        help="sign of the x stations; defaults to --sign")
    parser.add_argument("--tolerance", type=float, default=0.03)
    arguments = parser.parse_args()

    with h5py.File(arguments.cache, "r") as handle:
        vertices = np.asarray(handle["mesh/geometry/vertices"][:], dtype=float)

    sign = 1.0 if arguments.sign > 0 else -1.0
    x_sign = sign if arguments.x_sign is None else (1.0 if arguments.x_sign > 0 else -1.0)
    for station in (float(value) for value in arguments.stations.split(",")):
        x_target = x_sign * station
        y_centre = sign * 5.95
        near = np.nonzero(
            (np.abs(vertices[:, 0] - x_target) <= arguments.tolerance)
            & (np.abs(vertices[:, 1] - y_centre) <= 0.6)
        )[0]
        print("\n== station x=%+.3f (y around %+.2f): %d vertices"
              % (x_target, y_centre, len(near)))
        if not len(near):
            continue
        points = vertices[near]
        # Group by height band so surfaces (floor, walls, roof) become visible.
        order = np.lexsort((points[:, 2], points[:, 1]))
        for y, z in zip(points[order, 1], points[order, 2]):
            print("   y=%+.3f  z=%+.3f" % (y, z))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
