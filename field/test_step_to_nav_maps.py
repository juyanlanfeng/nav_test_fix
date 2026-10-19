#!/usr/bin/env python3
"""Exercise STEP conversion through its Gazebo, MeshNav and point-cloud outputs."""

import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
import trimesh

from conversion_metadata import sha256_file, write_json_object
from pcd_to_nav_mesh import read_pcd
import step_to_nav_maps as converter


class StepConversionTest(unittest.TestCase):
    def test_conversion_preserves_navigation_and_writes_generic_point_cloud(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            source = base / "arena.stp"
            writer = STEPControl_Writer()
            shape = BRepPrimAPI_MakeBox(1000.0, 2000.0, 500.0).Shape()
            self.assertEqual(writer.Transfer(shape, STEPControl_AsIs), IFSelect_RetDone)
            self.assertEqual(writer.Write(str(source)), IFSelect_RetDone)

            output = base / "converted"
            canonical_ply = output / "mesh_planner" / "arena.ply"
            canonical_ply.parent.mkdir(parents=True)
            canonical_ply.write_bytes(b"existing navigation mesh")
            provenance = {"sha256": "navigation-hash", "runtime_validation": "passed"}
            write_json_object(output / "conversion_metadata.json", {
                "source_step_sha256": sha256_file(source),
                "mesh_planner_multilevel": provenance,
            })
            arguments = converter.build_parser().parse_args([
                "convert", str(source), "--output", str(output),
                "--model-name", "arena", "--origin", "keep",
                "--min-points", "32", "--max-points", "32",
            ])

            converter.convert(arguments)

            self.assertEqual(
                {path.name for path in output.iterdir() if path.is_dir()},
                {"gazebo", "mesh_planner", "point_clouds"},
            )
            model_dir = output / "gazebo" / "models" / "arena"
            for kind in ("visual", "collision"):
                mesh = trimesh.load_mesh(model_dir / "meshes" / f"arena_{kind}.stl")
                self.assertEqual(len(mesh.faces), 12)
                np.testing.assert_allclose(mesh.bounds, [[0, 0, 0], [1, 2, 0.5]])
            self.assertEqual(
                ET.parse(model_dir / "model.sdf").getroot().find("model").get("name"),
                "arena",
            )
            world = ET.parse(output / "gazebo" / "worlds" / "arena.sdf").getroot()
            self.assertEqual(world.findtext("world/include/uri"), "model://arena")

            metadata = json.loads((output / "conversion_metadata.json").read_text())
            cloud_path = Path(metadata["raw_visual_surface_sample_pcd"])
            self.assertEqual(
                cloud_path, Path("point_clouds/arena_raw_visual_surface_sample.pcd")
            )
            points, _, _ = read_pcd(output / cloud_path)
            self.assertEqual(points.shape, (32, 3))
            self.assertEqual(metadata["scale_applied"], 0.001)
            self.assertEqual(metadata["mesh_planner_multilevel"], provenance)
            self.assertEqual(canonical_ply.read_bytes(), b"existing navigation mesh")
            candidate = output / "mesh_planner" / "arena_slope_candidates.ply"
            self.assertGreater(len(trimesh.load_mesh(candidate).faces), 0)


if __name__ == "__main__":
    unittest.main()
