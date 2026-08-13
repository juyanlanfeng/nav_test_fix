from __future__ import annotations

import unittest
from types import SimpleNamespace

import numpy as np

from jie_octomap.map_package_schema import (
    MAP_PACKAGE_LOAD_TIMEOUT_SEC,
    MAP_PACKAGE_SAVE_TIMEOUT_SEC,
    PLANNER_EXPORT_TIMEOUT_SEC,
    external_preblocked_layer_from_archive,
    planner_metadata_from_response,
    planner_parameters_from_metadata,
)


class MapPackageSchemaTest(unittest.TestCase):
    def test_save_timeout_encloses_full_planner_export(self) -> None:
        self.assertGreaterEqual(PLANNER_EXPORT_TIMEOUT_SEC, 180.0)
        self.assertGreater(
            MAP_PACKAGE_SAVE_TIMEOUT_SEC, PLANNER_EXPORT_TIMEOUT_SEC
        )
        self.assertGreater(
            MAP_PACKAGE_LOAD_TIMEOUT_SEC, PLANNER_EXPORT_TIMEOUT_SEC
        )

    def test_saved_planner_metadata_keeps_lowest_layer_policy(self) -> None:
        response = SimpleNamespace(
            robot_radius=0.25,
            robot_radius_xy=0.28,
            robot_height=0.225,
            snap_search_radius_cells=12,
            require_ground_support=True,
            strict_direct_ground_support=False,
            ground_support_xy_radius_cells=1,
            ground_support_depth_cells=1,
            lowest_traversable_only=True,
            enable_preblocked_costmap=True,
            preblocked_costmap_radius_cells=3,
            preblocked_costmap_weight=2.5,
        )

        planner = planner_metadata_from_response(response)

        self.assertTrue(planner["lowest_traversable_only"])

    def test_planner_profile_includes_identity_and_lowest_layer_policy(self) -> None:
        values = planner_parameters_from_metadata(
            {
                "frame_id": "arena",
                "map_id": "rmuc",
                "source_world_file": "/tmp/arena.sdf",
                "planner": {
                    "robot_radius_xy": 0.28,
                    "robot_height": 0.225,
                    "lowest_traversable_only": True,
                },
            }
        )

        self.assertEqual(values["frame_id"], "arena")
        self.assertEqual(values["map_id"], "rmuc")
        self.assertEqual(values["source_world_file"], "/tmp/arena.sdf")
        self.assertEqual(values["robot_radius_xy"], 0.28)
        self.assertEqual(values["robot_height"], 0.225)
        self.assertTrue(values["lowest_traversable_only"])

    def test_external_layer_is_loaded_independently_of_aggregate_snapshot(self) -> None:
        archive = {
            "preblocked_points": np.array([[99.0, 99.0, 99.0]], dtype=np.float32),
            "external_preblocked_points": np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
            "external_preblocked_scale": np.array([0.05, 0.05, 0.05]),
            "external_preblocked_frame_id": np.array(["arena"]),
        }

        points, scale, frame_id = external_preblocked_layer_from_archive(
            archive, "map", np.array([0.2, 0.2, 0.2])
        )

        np.testing.assert_array_equal(points, np.array([[1.0, 2.0, 3.0]], dtype=np.float32))
        np.testing.assert_array_equal(scale, np.array([0.05, 0.05, 0.05]))
        self.assertEqual(frame_id, "arena")

    def test_legacy_package_gets_an_explicit_empty_external_layer(self) -> None:
        archive = {
            "preblocked_points": np.array([[4.0, 5.0, 6.0]], dtype=np.float32),
        }

        points, scale, frame_id = external_preblocked_layer_from_archive(
            archive, "legacy_map", np.array([0.1, 0.1, 0.1])
        )

        self.assertEqual(points.shape, (0, 3))
        np.testing.assert_array_equal(scale, np.array([0.1, 0.1, 0.1]))
        self.assertEqual(frame_id, "legacy_map")


if __name__ == "__main__":
    unittest.main()
