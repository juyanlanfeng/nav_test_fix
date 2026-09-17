#!/usr/bin/env python3
"""Read the MeshNav layer values that MeshMap wrote into its h5 cache.

`mbf_mesh_nav` computes its layers at runtime; calling its `~/save_map` service
writes them into the working file, and this tool then reports, for each requested
map position, the value of every layer (including the combined `final` cost that
the planner's `cost_limit` is compared against).

    PYTHONPATH=meshnav_demo_ws/src/pb_vehicle_adapter \
    field/.step_convert_venv/bin/python field/read_mesh_layers.py <cache.h5> \
        --points "-1.45,5.95" --points "1.0,-5.95"
"""
import argparse
import json
from pathlib import Path

import h5py
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cache", help="MeshMap working file (h5) written by ~/save_map")
    parser.add_argument("--points", action="append", default=[],
                        help='"x,y" map position to query; repeatable')
    parser.add_argument("--json", action="store_true")
    arguments = parser.parse_args()

    with h5py.File(arguments.cache, "r") as handle:
        names = []
        handle.visititems(lambda name, obj: names.append(name)
                          if isinstance(obj, h5py.Dataset) else None)
        attribute_sets = [name for name in names if "attributes" in name]
        vertices = np.asarray(handle["mesh/geometry/vertices"][:], dtype=float)
        print("mesh: %d vertices" % len(vertices))
        print("layer datasets:")
        for name in attribute_sets:
            print("  %s %s" % (name, handle[name].shape))

        results = {}
        for point in arguments.points:
            x, y = (float(value) for value in point.split(","))
            distance = np.linalg.norm(vertices[:, :2] - np.array([x, y]), axis=1)
            index = int(np.argmin(distance))
            entry = {"query": [x, y], "vertex_index": index,
                     "vertex": [round(float(value), 4) for value in vertices[index]],
                     "snap_m": round(float(distance[index]), 4), "layers": {}}
            for name in attribute_sets:
                data = handle[name][:]
                if data.shape[0] != len(vertices):
                    continue
                value = data[index]
                array = np.asarray(value).ravel()
                entry["layers"][name.split("/")[-1]] = [round(float(v), 5) for v in array]
            results[point] = entry
            print("  (%6.2f,%6.2f) vertex %-28s snap %.3f"
                  % (x, y, entry["vertex"], entry["snap_m"]))
            for layer, value in sorted(entry["layers"].items()):
                print("      %-24s %s" % (layer, value))

    if arguments.json:
        print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
