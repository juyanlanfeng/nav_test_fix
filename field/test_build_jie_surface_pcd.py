#!/usr/bin/env python3
"""Focused regressions for deterministic JIE occupancy-surface rasterization."""

from pathlib import Path
import tempfile
import unittest

import numpy as np
import trimesh

from build_jie_surface_pcd import rasterize_surface_voxels
from step_to_nav_maps import write_binary_pcd
from verify_jie_tunnel_pcd import read_binary_xyz_pcd


class JieSurfacePcdBuilderTest(unittest.TestCase):
    @staticmethod
    def _sloped_test_mesh() -> trimesh.Trimesh:
        return trimesh.Trimesh(
            vertices=np.asarray(
                [
                    [-0.15, -0.10, -0.05],
                    [0.15, -0.10, 0.05],
                    [-0.15, 0.10, -0.05],
                    [0.15, 0.10, 0.05],
                ],
                dtype=np.float64,
            ),
            faces=np.asarray([[0, 1, 2], [2, 1, 3]], dtype=np.int64),
            process=False,
        )

    def test_raster_keys_are_deterministic_and_sorted(self):
        mesh = self._sloped_test_mesh()
        first = rasterize_surface_voxels(mesh, 0.05, -0.08, 0.09, 1, 1.1)
        second = rasterize_surface_voxels(mesh, 0.05, -0.08, 0.09, 2, 1.1)

        np.testing.assert_array_equal(first, second)
        expected_order = np.lexsort((first[:, 2], first[:, 1], first[:, 0]))
        np.testing.assert_array_equal(expected_order, np.arange(len(first)))

    def test_float32_pcd_cell_centres_round_trip_to_intended_signed_keys(self):
        mesh = self._sloped_test_mesh()
        pitch = 0.05
        keys = rasterize_surface_voxels(mesh, pitch, -0.08, 0.09, 2, 1.1)
        points = (keys.astype(np.float64) + 0.5) * pitch

        with tempfile.TemporaryDirectory() as directory:
            first_path = Path(directory) / "first.pcd"
            second_path = Path(directory) / "second.pcd"
            write_binary_pcd(first_path, points)
            write_binary_pcd(second_path, points)
            self.assertEqual(first_path.read_bytes(), second_path.read_bytes())
            loaded = read_binary_xyz_pcd(first_path).astype(np.float64)

        # This reproduces the signed floor conversion used by JIE/OctoMap.
        # Points on lattice boundaries are unstable here, especially for
        # negative coordinates; half-cell centres map back to the exact keys.
        recovered = np.floor(loaded / pitch).astype(np.int32)
        np.testing.assert_array_equal(recovered, keys)
        self.assertTrue(np.any(keys < 0))


if __name__ == "__main__":
    unittest.main()
