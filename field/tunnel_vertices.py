#!/usr/bin/env python3
"""Offline inspection of the RMUC2026 mesh around the two tunnels.

Prints the mesh vertices that lie in a tunnel band, together with the local
vertex spacing, so that the free-space accounting of the low vehicle can be
checked against the *actual* discretisation instead of the STL geometry: the
MeshNav layers mark costs per vertex, and a free corridor that is narrower than
the local vertex spacing may contain no free vertex at all.

    field/.step_convert_venv/bin/python field/tunnel_vertices.py <cache.h5>
"""
import argparse

import h5py
import numpy as np


def band(vertices, x_lo, x_hi, y_lo, y_hi, z_hi):
    inside = (
        (vertices[:, 0] >= x_lo) & (vertices[:, 0] <= x_hi)
        & (vertices[:, 1] >= y_lo) & (vertices[:, 1] <= y_hi)
        & (vertices[:, 2] <= z_hi)
    )
    return np.nonzero(inside)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cache")
    arguments = parser.parse_args()

    with h5py.File(arguments.cache, "r") as handle:
        vertices = np.asarray(handle["mesh/geometry/vertices"][:], dtype=float)

    print("mesh: %d vertices" % len(vertices))
    for label, box in (
        ("+Y tunnel floor", (-1.45, -0.65, 5.45, 6.35, 0.05)),
        ("-Y tunnel floor", (0.65, 1.45, -6.35, -5.45, 0.05)),
        ("+Y roof", (-1.45, -0.65, 5.45, 6.35, 0.30)),
        ("-Y roof", (0.65, 1.45, -6.35, -5.45, 0.30)),
    ):
        index = band(vertices, *box)
        print("\n== %s: %d vertices" % (label, len(index)))
        if not len(index):
            continue
        points = vertices[index]
        print("   x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f"
              % (points[:, 0].min(), points[:, 0].max(), points[:, 1].min(),
                 points[:, 1].max(), points[:, 2].min(), points[:, 2].max()))
        # Longitudinal slices: how the tunnel interior is discretised in y.
        for x_centre in np.arange(-1.3, 1.35, 0.2):
            slice_index = np.nonzero(np.abs(points[:, 0] - x_centre) < 0.05)[0]
            if not len(slice_index):
                continue
            ordered = points[slice_index][np.argsort(points[slice_index][:, 1])]
            print("   x=%+.2f: %s"
                  % (x_centre, " ".join("%.3f" % value for value in ordered[:, 1])))
        # Local spacing inside the band.
        unique = np.unique(np.round(points, 4), axis=0)
        if len(unique) > 1:
            distances = np.linalg.norm(unique[:, None, :] - unique[None, :, :], axis=2)
            np.fill_diagonal(distances, np.inf)
            nearest = distances.min(axis=1)
            print("   spacing: min %.4f median %.4f max %.4f"
                  % (nearest.min(), float(np.median(nearest)), nearest.max()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
