#!/usr/bin/env python3
"""Lightweight regressions for clean-rebuild conversion provenance."""

from argparse import Namespace
from pathlib import Path
import tempfile
import unittest

from build_jie_surface_pcd import update_conversion_metadata
from conversion_metadata import (
    load_json_object,
    merge_conversion_root,
    merge_hashed_object,
    merge_hashed_section,
    write_json_object,
)
from postprocess_nav_maps import default_collision_report_path


class ConversionMetadataTest(unittest.TestCase):
    def test_step_root_refresh_keeps_downstream_for_same_source_hash(self):
        previous = {
            "source_step_sha256": "same-step",
            "linear_deflection_mm": 20,
            "jie_nav_surface_pcd": {"sha256": "pcd", "manual_check": "pass"},
        }
        generated = {
            "source_step_sha256": "same-step",
            "linear_deflection_mm": 50,
            "source_unit": "millimetre",
        }

        merged = merge_conversion_root(previous, generated)

        self.assertEqual(merged["linear_deflection_mm"], 50)
        self.assertEqual(
            merged["jie_nav_surface_pcd"]["manual_check"], "pass"
        )

    def test_step_root_refresh_drops_downstream_for_changed_source_hash(self):
        previous = {
            "source_step_sha256": "old-step",
            "jie_nav_surface_pcd": {"sha256": "old-pcd", "manual_check": "pass"},
        }
        generated = {
            "source_step_sha256": "new-step",
            "source_unit": "millimetre",
        }

        self.assertEqual(merge_conversion_root(previous, generated), generated)

    def test_step_root_refresh_drops_downstream_for_unknown_source_hash(self):
        previous = {
            "jie_nav_surface_pcd": {"sha256": "old-pcd", "manual_check": "pass"},
        }
        generated = {"source_unit": "millimetre"}

        self.assertEqual(merge_conversion_root(previous, generated), generated)

    def test_hashed_section_keeps_evidence_only_for_identical_bytes(self):
        metadata = {
            "artifact": {
                "sha256": "same",
                "runtime_validation": ["kept"],
                "points": 1,
            }
        }
        merge_hashed_section(
            metadata, "artifact", {"sha256": "same", "points": 2}, "sha256"
        )
        self.assertEqual(metadata["artifact"]["runtime_validation"], ["kept"])
        self.assertEqual(metadata["artifact"]["points"], 2)

        merge_hashed_section(
            metadata, "artifact", {"sha256": "changed", "points": 3}, "sha256"
        )
        self.assertNotIn("runtime_validation", metadata["artifact"])
        self.assertEqual(metadata["artifact"]["points"], 3)

    def test_collision_report_preserves_evidence_across_hash_aliases(self):
        previous = {
            "sha256": "same",
            "vertical_ray_validation": {"tunnel": "pass"},
            "output_faces": 1,
        }
        generated = {"output_sha256": "same", "output_faces": 2}

        merged = merge_hashed_object(
            previous, generated, ("output_sha256", "sha256")
        )

        self.assertEqual(merged["vertical_ray_validation"], {"tunnel": "pass"})
        self.assertEqual(merged["output_faces"], 2)

    def test_collision_report_drops_evidence_when_bytes_change(self):
        previous = {
            "sha256": "old",
            "critical_components_preserved": True,
        }
        generated = {"output_sha256": "new", "output_faces": 2}

        merged = merge_hashed_object(
            previous, generated, ("output_sha256", "sha256")
        )

        self.assertNotIn("critical_components_preserved", merged)
        self.assertEqual(merged, generated)

    def test_jie_builder_closes_metadata_and_preserves_same_hash_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory) / "converted"
            output = base / "jie_nav" / "arena.pcd"
            source = base / "gazebo" / "arena_collision.stl"
            report_path = base / "jie_nav" / "arena.surface.json"
            metadata_path = base / "conversion_metadata.json"
            write_json_object(
                metadata_path,
                {
                    "source_unit": "millimetre",
                    "jie_nav_surface_pcd": {
                        "sha256": "pcd-hash",
                        "offline_validation": {"tunnel": "pass"},
                    },
                },
            )
            args = Namespace(source=source, output=output, report=report_path)
            report = {
                "output_sha256": "pcd-hash",
                "output_points": 42,
                "output_file_size_bytes": 504,
                "surface_voxel_m": 0.05,
                "subdivision_edge_factor": 1.1,
                "maximum_subdivided_edge_m": 0.05 / 1.1,
                "z_band_m": [-0.08, 0.9],
                "output_coordinate_policy": (
                    "(nearest_lattice_key + 0.5) * surface_voxel_m"
                ),
            }

            self.assertTrue(update_conversion_metadata(args, report))
            metadata = load_json_object(metadata_path)
            jie = metadata["jie_nav_surface_pcd"]
            self.assertEqual(jie["canonical_file"], "jie_nav/arena.pcd")
            self.assertEqual(jie["source_mesh"], "gazebo/arena_collision.stl")
            self.assertEqual(jie["points"], 42)
            self.assertEqual(jie["offline_validation"], {"tunnel": "pass"})
            self.assertIn("never the Mesh navigation PLY", jie["occupancy_semantics"])

    def test_default_collision_report_matches_verifier_path(self):
        base = Path("converted")
        self.assertEqual(
            default_collision_report_path(
                base, "rmuc2026_field", "component-budget", 500000
            ),
            base
            / "gazebo"
            / "collision_candidates"
            / "rmuc2026_field_collision_component_budget_500k.json",
        )


if __name__ == "__main__":
    unittest.main()
