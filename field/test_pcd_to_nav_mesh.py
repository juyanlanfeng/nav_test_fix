#!/usr/bin/env python3
"""Regression tests for the PCD to layered navigation mesh workflow."""

import tempfile
import json
from pathlib import Path
import unittest

import numpy as np

import pcd_to_nav_mesh as converter
import check_nav_mesh_route
from step_to_nav_maps import write_binary_pcd


class PcdToNavMeshTest(unittest.TestCase):
    def test_reads_binary_xyz_and_ascii_with_extra_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "binary.pcd"
            expected = np.asarray([[0, 1, 2], [3, 4, 5]], dtype=float)
            write_binary_pcd(binary, expected)
            points, normals, metadata = converter.read_pcd(binary)
            np.testing.assert_allclose(points, expected)
            self.assertIsNone(normals)
            self.assertEqual(metadata["encoding"], "binary")

            ascii_path = Path(directory) / "ascii.pcd"
            ascii_path.write_text(
                "VERSION 0.7\nFIELDS x y z intensity normal_x normal_y normal_z\n"
                "SIZE 4 4 4 4 4 4 4\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\n"
                "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
                "0 1 2 7 0 0 1\n3 4 5 8 0 0 1\n",
                encoding="ascii",
            )
            points, normals, metadata = converter.read_pcd(ascii_path)
            np.testing.assert_allclose(points, expected)
            np.testing.assert_allclose(normals, [[0, 0, 1], [0, 0, 1]])
            self.assertEqual(metadata["encoding"], "ascii")

    def test_binary_accepts_pcl_zero_padding_but_rejects_trailing_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "padded.pcd"
            write_binary_pcd(path, np.asarray([[1.0, 2.0, 3.0]]))
            with path.open("ab") as stream:
                stream.write(b"\0" * 31)
            points, _, metadata = converter.read_pcd(path)
            np.testing.assert_allclose(points, [[1.0, 2.0, 3.0]])
            self.assertEqual(metadata["trailing_padding_bytes"], 31)

            with path.open("ab") as stream:
                stream.write(b"bad")
            with self.assertRaisesRegex(ValueError, "non-padding bytes"):
                converter.read_pcd(path)

    def test_voxel_downsample_is_deterministic(self):
        points = np.asarray([[0.01, 0.01, 0], [0.02, 0.02, 0.02], [1, 1, 1]])
        first, _ = converter.voxel_downsample(points, None, 0.1)
        second, _ = converter.voxel_downsample(points[::-1], None, 0.1)
        np.testing.assert_allclose(first, second)
        np.testing.assert_allclose(first[0], [0.015, 0.015, 0.01])

    def test_invalid_normal_does_not_discard_valid_xyz(self):
        points, normals, removed = converter.finite_rows(
            np.asarray([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0]]),
            np.asarray([[np.nan, 0.0, 1.0], [0.0, 0.0, 1.0]]),
        )
        self.assertEqual(removed, 1)
        np.testing.assert_allclose(points, [[1.0, 2.0, 3.0]])
        np.testing.assert_allclose(normals, [[0.0, 0.0, 0.0]])

    def test_stacked_surfaces_remain_separate_and_low_headroom_is_removed(self):
        xy = np.asarray([(x, y) for y in (0.0, 0.05) for x in (0.0, 0.05)])
        points = np.vstack([
            np.column_stack((xy, np.zeros(4))),
            np.column_stack((xy, np.full(4, 0.20))),
            np.column_stack((xy, np.full(4, 0.60))),
        ])
        normals = np.tile([0.0, 0.0, 1.0], (len(points), 1))
        xs, ys, layers, diagnostics = converter.point_layers(
            points, normals, grid_m=0.05, max_slope_deg=30,
            layer_merge_m=0.01, robot_height_m=0.25, min_points_per_cell=1,
        )
        self.assertEqual(diagnostics["headroom_layers_rejected"], 4)
        self.assertTrue(all(value == [0.2, 0.6] for value in layers))
        vertices, faces = converter.triangulate_layers(xs, ys, layers, 30)
        self.assertEqual(len(faces), 4)
        for face in faces:
            self.assertAlmostEqual(float(np.ptp(vertices[face, 2])), 0.0)

    def test_convert_writes_loadable_triangle_ply_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source = base / "plane.pcd"
            rows = [
                f"{x * 0.05} {y * 0.05} 0 0 0 1"
                for y in range(4) for x in range(4)
            ]
            source.write_text(
                "VERSION 0.7\nFIELDS x y z normal_x normal_y normal_z\n"
                "SIZE 4 4 4 4 4 4\nTYPE F F F F F F\nCOUNT 1 1 1 1 1 1\n"
                "WIDTH 16\nHEIGHT 1\nPOINTS 16\nDATA ascii\n"
                + "\n".join(rows) + "\n",
                encoding="ascii",
            )
            output = base / "mesh_planner" / "plane.ply"
            report = base / "plane.json"
            arguments = converter.build_parser().parse_args([
                "convert", str(source), str(output), "--report", str(report),
                "--voxel-m", "0.01", "--grid-m", "0.05",
                "--max-slope-deg", "30", "--robot-height-m", "0.3",
            ])
            converter.validate_args(arguments)
            converter.convert(arguments)
            mesh = converter.trimesh.load_mesh(output, process=False)
            self.assertEqual(len(mesh.vertices), 16)
            self.assertEqual(len(mesh.faces), 18)
            result = json.loads(report.read_text(encoding="utf-8"))
            self.assertEqual(result["normal_source"], "pcd_fields")
            self.assertEqual(result["output_triangles"], 18)

    def test_route_precheck_respects_layers_and_disconnected_surfaces(self):
        with tempfile.TemporaryDirectory() as directory:
            lower = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
            upper = lower + [0, 0, 1]
            mesh = converter.trimesh.Trimesh(
                vertices=np.vstack((lower, upper)),
                faces=[[0, 1, 2], [1, 3, 2], [4, 5, 6], [5, 7, 6]],
                process=False,
            )
            path = Path(directory) / "two_layers.ply"
            mesh.export(path)
            lower_route = check_nav_mesh_route.check(
                path, [0.2, 0.2, 0], [0.8, 0.8, 0], 0.1
            )
            self.assertTrue(lower_route["geometry_precheck_pass"])
            between_layers = check_nav_mesh_route.check(
                path, [0.2, 0.2, 0], [0.8, 0.8, 1], 0.1
            )
            self.assertFalse(between_layers["geometry_precheck_pass"])
            self.assertFalse(between_layers["same_edge_connected_component"])
            missing_surface = check_nav_mesh_route.check(
                path, [0.2, 0.2, 0], [3, 3, 0], 0.1
            )
            self.assertFalse(missing_surface["within_snap_limit"])
            touching_edge = check_nav_mesh_route.check(
                path, [0, 0.5, 0], [0.75, 0.5, 0], 0.1, clearance=0.22
            )
            self.assertFalse(touching_edge["within_boundary_clearance"])
            interior = check_nav_mesh_route.check(
                path, [0.5, 0.5, 0], [0.75, 0.5, 0], 0.1, clearance=0.22
            )
            self.assertTrue(interior["geometry_precheck_pass"])


if __name__ == "__main__":
    unittest.main()
