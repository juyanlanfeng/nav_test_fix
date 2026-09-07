#!/usr/bin/env python3
"""Focused regressions for deterministic JIE occupancy-surface rasterization."""

from pathlib import Path
import tempfile
import unittest

import numpy as np
import trimesh

from build_jie_surface_pcd import rasterize_surface_voxels, rasterize_triangle_boxes
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

    def test_source_surfaces_stay_inside_their_cells_without_half_cell_shift(self):
        for offset, expected_key in [(0.026, 0), (-0.024, -1)]:
            vertices = np.array([[offset, offset, offset],
                                 [offset + .001, offset, offset],
                                 [offset, offset + .001, offset]])
            mesh = trimesh.Trimesh(vertices=vertices, faces=[[0, 1, 2]], process=False)
            keys = rasterize_surface_voxels(mesh, .05, -.1, .1, 1, 2.)
            np.testing.assert_array_equal(keys, [[expected_key] * 3])
            lo = keys[0] * .05
            self.assertTrue(np.all(vertices >= lo))
            self.assertTrue(np.all(vertices < lo + .05))

    def test_legacy_quantization_is_only_an_explicit_reproduction_option(self):
        mesh = trimesh.Trimesh(
            vertices=[[.026, .026, .026], [.027, .026, .026], [.026, .027, .026]],
            faces=[[0, 1, 2]], process=False,
        )
        keys = rasterize_surface_voxels(mesh, .05, -.1, .1, 1, 2., 'legacy-nearest')
        np.testing.assert_array_equal(keys, [[1, 1, 1]])

    def test_exact_rasterizer_preserves_thin_surfaces_and_empty_tunnel(self):
        # Two horizontal sheets separated by an empty tunnel; no solid fill.
        mesh = trimesh.Trimesh(
            vertices=[[-.13, -.13, .006], [.13, -.13, .006],
                      [.13, .13, .006], [-.13, .13, .006],
                      [-.13, -.13, .246], [.13, -.13, .246],
                      [.13, .13, .246], [-.13, .13, .246]],
            faces=[[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6]], process=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'sheets.stl'
            mesh.export(path)
            keys = rasterize_triangle_boxes(path, .04, -.08, .9)
        cells = {tuple(k) for k in keys}
        for x in range(-3, 3):
            for y in range(-3, 3):
                self.assertIn((x, y, 0), cells)
                self.assertIn((x, y, 6), cells)
                for z in range(1, 6):
                    self.assertNotIn((x, y, z), cells)

    def test_exact_rasterizer_does_not_fill_triangle_bounding_box(self):
        mesh = trimesh.Trimesh(
            vertices=[[.001, .001, .001], [.199, .001, .199], [.001, .199, .001]],
            faces=[[0, 1, 2]], process=False,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'slope.stl'
            mesh.export(path)
            keys = rasterize_triangle_boxes(path, .04, -.08, .9)
        cells = {tuple(k) for k in keys}
        self.assertIn((2, 0, 2), cells)
        self.assertNotIn((4, 4, 4), cells)  # outside the triangle
        self.assertNotIn((0, 0, 4), cells)  # above its plane


if __name__ == "__main__":
    unittest.main()
