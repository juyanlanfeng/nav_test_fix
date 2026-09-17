#!/usr/bin/env python3
"""Extract the walkable ground from JIE's surface point cloud.

`field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd` is a *surface* cloud: only
~22 % of its 631k points are floor (z < 0.05), the rest are walls, tunnel roofs
and the surrounding structures.  RViz therefore shows blue clutter instead of a
floor, which makes picking start/goal points hard.

This tool keeps, for every 4 cm (x, y) cell, the points that sit within
`--tolerance` of the lowest point in that cell - i.e. everything you can see from
above, including the ramps and the raised platform, but not the walls or the
tunnel roofs above the floor.  The result is written as a binary PCD that can be
published with `ros2 run pcl_ros pcd_to_pointcloud` (see `pb_jie.launch.py`,
`publish_floor_cloud`).

    field/.step_convert_venv/bin/python field/build_jie_floor_pcd.py
"""
import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path("/home/rainple/nav_test")
DEFAULT_INPUT = ROOT / "field/converted_rmuc2026/jie_nav/rmuc2026_field.pcd"
DEFAULT_OUTPUT = ROOT / "field/converted_rmuc2026/jie_nav/rmuc2026_field_floor.pcd"


def read_pcd(path):
    fields, width, header_lines = None, None, []
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header ended before DATA")
            text = line.decode("ascii", "replace").strip()
            header_lines.append(text)
            if text.startswith("FIELDS"):
                fields = text.split()[1:]
            elif text.startswith("WIDTH"):
                width = int(text.split()[1])
            elif text.startswith("DATA"):
                if text.split()[1] != "binary":
                    raise RuntimeError("only binary PCD is supported")
                break
        values = np.fromfile(stream, dtype=np.float32)
    points = values.reshape(-1, len(fields))
    return points, fields, width


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (len(points), len(points))
    )
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        np.asarray(points, dtype=np.float32).tofile(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--cell", type=float, default=0.04,
                        help="(x, y) cell size used to find the local ground height")
    parser.add_argument("--tolerance", type=float, default=0.05,
                        help="keep points within this distance above the cell minimum")
    parser.add_argument("--max-z", type=float, default=0.25,
                        help="drop everything above this height: keeps floor, ramp and the "
                             "0.203 m platform, drops tunnel roofs (0.42) and the surrounding "
                             "structures (0.9) that a ground vehicle cannot use")
    parser.add_argument("--report", type=Path,
                        default=ROOT / "field/converted_rmuc2026/jie_nav/rmuc2026_field_floor.json")
    arguments = parser.parse_args()

    points, fields, _width = read_pcd(arguments.input)
    xyz = points[:, :3].astype(np.float64)
    cells = np.floor(xyz[:, :2] / arguments.cell).astype(np.int64)
    keys = cells[:, 0] * 100000 + cells[:, 1]
    order = np.argsort(keys, kind="stable")
    keys_sorted, z_sorted = keys[order], xyz[order, 2]
    # Lowest z per cell, broadcast back to every point of that cell.
    first = np.searchsorted(keys_sorted, keys_sorted, side="left")
    minimum = np.minimum.reduceat(z_sorted, np.unique(keys_sorted, return_index=True)[1])
    cell_index = np.searchsorted(np.unique(keys_sorted), keys_sorted)
    keep_sorted = z_sorted <= minimum[cell_index] + arguments.tolerance
    keep = order[keep_sorted]

    floor = xyz[keep]
    floor = floor[floor[:, 2] <= arguments.max_z]
    write_pcd(arguments.output, floor)
    report = {
        "input": str(arguments.input),
        "output": str(arguments.output),
        "input_points": int(len(xyz)),
        "floor_points": int(len(floor)),
        "cell_m": arguments.cell,
        "tolerance_m": arguments.tolerance,
        "max_z_m": arguments.max_z,
        "z_min": float(floor[:, 2].min()),
        "z_max": float(floor[:, 2].max()),
        "extent_x": [float(floor[:, 0].min()), float(floor[:, 0].max())],
        "extent_y": [float(floor[:, 1].min()), float(floor[:, 1].max())],
    }
    arguments.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("floor points: %d of %d (%.1f %%), z %.3f..%.3f"
          % (len(floor), len(xyz), 100.0 * len(floor) / len(xyz),
             floor[:, 2].min(), floor[:, 2].max()))
    print("written: %s" % arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
