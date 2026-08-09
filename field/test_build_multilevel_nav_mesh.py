#!/usr/bin/env python3
"""Focused regression tests for the multi-level navigation mesh builder."""

import unittest

import numpy as np

import build_multilevel_nav_mesh as builder


class MultiLevelBuilderTest(unittest.TestCase):
    def test_cluster_keeps_geometric_slope_when_winding_is_reversed(self):
        clusters = builder.cluster_ray_hits(
            np.asarray([0.0, 0.001]),
            np.asarray([-1.0, -0.8]),
            tolerance=0.012,
        )
        self.assertEqual(len(clusters), 1)
        _, max_upward_normal_z, max_abs_normal_z = clusters[0]
        self.assertEqual(max_upward_normal_z, -0.8)
        self.assertEqual(max_abs_normal_z, 1.0)

    def test_reciprocal_matching_is_one_to_one(self):
        pairs = builder.reciprocal_nearest_pairs(
            np.asarray([0.0, 0.4]),
            np.asarray([0.01, 0.39, 0.8]),
            max_delta_z=0.05,
        )
        self.assertEqual(pairs, [(0, 0), (1, 1)])

    def test_triangulation_does_not_bridge_stacked_layers(self):
        xs = np.asarray([0.0, 0.05])
        ys = np.asarray([0.0, 0.05])
        layers = [[0.0, 0.4] for _ in range(4)]
        vertices, faces = builder.triangulate_layers(xs, ys, layers, 55.0)
        self.assertEqual(len(faces), 4)
        for face in faces:
            self.assertAlmostEqual(
                float(np.ptp(vertices[face, 2])), 0.0, places=9
            )

    def test_rmuc_profile_has_physical_headroom_margin(self):
        collision_height_m = 0.215
        self.assertAlmostEqual(
            builder.RMUC2026_ROBOT_HEIGHT_M - collision_height_m,
            0.010,
            places=9,
        )
        self.assertGreater(0.246, builder.RMUC2026_ROBOT_HEIGHT_M)
        self.assertEqual(len(builder.RMUC2026_REVERSED_NORMAL_REGIONS), 2)


if __name__ == "__main__":
    unittest.main()
