#!/usr/bin/env python3
"""Offline regression tests for Gazebo/MeshNav project consistency checks."""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
import unittest

from verify_rmuc_project import cache_is_fresh, main, sha256, verify_project


class VerifyRmucProjectTest(unittest.TestCase):
    def test_cache_must_be_newer_than_every_dependency(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cache = base / "map.h5"
            geometry = base / "map.ply"
            profile = base / "mbf_mesh_nav.yaml"
            for path in (cache, geometry, profile):
                path.write_bytes(b"data")
            os.utime(geometry, ns=(100, 100))
            os.utime(cache, ns=(200, 200))
            os.utime(profile, ns=(300, 300))

            self.assertFalse(cache_is_fresh(cache, [geometry, profile]))

            os.utime(cache, ns=(400, 400))
            self.assertTrue(cache_is_fresh(cache, [geometry, profile]))

    def test_empty_cache_is_never_fresh(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cache = base / "map.h5"
            source = base / "map.ply"
            cache.touch()
            source.write_bytes(b"data")

            self.assertFalse(cache_is_fresh(cache, [source]))

    def test_missing_cache_or_dependency_is_not_fresh(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cache = base / "map.h5"
            source = base / "map.ply"
            self.assertFalse(cache_is_fresh(cache, [source]))
            cache.write_bytes(b"cache")
            self.assertFalse(cache_is_fresh(cache, [source]))

    def test_equal_cache_and_input_timestamps_are_fresh(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cache, source = base / "map.h5", base / "map.ply"
            for path in (cache, source):
                path.write_bytes(b"data")
                os.utime(path, ns=(100, 100))
            self.assertTrue(cache_is_fresh(cache, [source]))


class GazeboMeshNavAuditTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        root = Path(self.temporary.name)
        self.base = root / "conversion"
        self.workspace = root / "mesh_navigation_tutorials"
        self.name = "test_arena"
        self.model = self.base / "gazebo" / "models" / self.name
        self.ros_package = self.workspace / "src" / "mesh_navigation_tutorials"
        self.ros_sim = self.workspace / "src" / "mesh_navigation_tutorials_sim"
        self.visual = self.model / "meshes" / f"{self.name}_visual.stl"
        self.collision = self.model / "meshes" / f"{self.name}_collision.stl"
        stl = b"test scene".ljust(80, b"\0") + struct.pack("<I", 1)
        stl += struct.pack("<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0)
        self.write(self.visual, stl)
        self.write(self.collision, stl)
        self.ply = self.base / "mesh_planner" / f"{self.name}.ply"
        ply = (
            b"ply\nformat binary_little_endian 1.0\nelement vertex 3\n"
            b"property float x\nproperty float y\nproperty float z\nelement face 1\n"
            b"property list uchar int vertex_indices\nend_header\n"
        )
        ply += struct.pack("<9fB3i", 0, 0, 0, 1, 0, 0, 0, 1, 0, 3, 0, 1, 2)
        self.write(self.ply, ply)
        model_sdf = f"""<sdf version="1.8"><model name="{self.name}">
<static>true</static><link name="map_link">
<visual name="scene"><geometry><mesh><uri>meshes/{self.visual.name}</uri>
<scale>1 1 1</scale></mesh></geometry></visual>
<collision name="scene"><geometry><mesh><uri>meshes/{self.collision.name}</uri>
<scale>1 1 1</scale></mesh></geometry></collision>
</link></model></sdf>"""
        self.write(self.model / "model.sdf", model_sdf)
        self.write(self.model / "model.config", f"<model><name>{self.name}</name></model>")
        self.world = self.base / "gazebo" / "worlds" / f"{self.name}.sdf"
        self.write(
            self.world,
            f'<sdf><world name="{self.name}"><include><uri>model://{self.name}</uri></include></world></sdf>',
        )
        shutil.copytree(self.base / "gazebo/models", self.ros_sim / "models")
        shutil.copytree(self.base / "gazebo/worlds", self.ros_sim / "worlds")
        self.ros_ply = self.ros_package / "maps" / self.ply.name
        self.write(self.ros_ply, ply)
        self.nav_launch = self.ros_package / "launch/mbf_mesh_navigation_server_launch.py"
        self.nav_config = self.ros_package / "config/mbf_mesh_nav.yaml"
        self.write(self.nav_launch, "# current MeshNav launch\n")
        self.write(self.nav_config, "move_base_flex:\n  ros__parameters: {}\n")
        self.report_path = self.ply.with_suffix(".multilevel.json")
        self.parameters = {"grid_resolution_m": 0.1, "max_slope_deg": 35.0, "robot_height_m": 0.25}
        self.report = {
            "output_vertices": 3, "output_triangles": 1, "nonmanifold_edges": 0,
            **self.parameters,
        }
        self.write_json(self.report_path, self.report)
        self.metadata_path = self.base / "conversion_metadata.json"
        self.metadata = {
            "output_unit": "metre", "mesh_triangles": 1,
            "mesh_planner_multilevel": {
                "canonical_file": str(self.ply.relative_to(self.base)),
                "sha256": sha256(self.ply), **self.parameters,
            },
        }
        self.write_json(self.metadata_path, self.metadata)

    @staticmethod
    def write(path, content):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content.encode() if isinstance(content, str) else content)

    def write_json(self, path, value):
        self.write(path, json.dumps(value))

    def audit(self, **options):
        with redirect_stdout(io.StringIO()):
            return verify_project(self.base, self.workspace, self.name, **options)

    def test_accepts_generated_assets_in_current_workspace_layout(self):
        self.assertEqual(self.audit().failures, [])
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main([
                "--base", str(self.base), "--workspace", str(self.workspace),
                "--model-name", self.name,
            ])
        self.assertEqual(result, 0)

    def test_rejects_unsynchronized_ros_copy(self):
        self.ros_ply.write_bytes(self.ros_ply.read_bytes() + b"changed")
        self.assertTrue(any("ROS 副本哈希一致" in failure for failure in self.audit().failures))

    def test_accepts_external_runtime_map_without_package_map_copy(self):
        self.ros_ply.unlink()
        self.assertEqual(self.audit(ros_map=self.ply).failures, [])
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main([
                "--base", str(self.base), "--workspace", str(self.workspace),
                "--model-name", self.name, "--ros-map", str(self.ply),
            ])
        self.assertEqual(result, 0)
        self.ply.with_suffix(".h5").touch()
        self.assertTrue(any("H5 缓存" in failure for failure in self.audit(ros_map=self.ply).failures))

    def test_rejects_metadata_for_an_old_navigation_map(self):
        self.metadata["mesh_planner_multilevel"]["sha256"] = "outdated"
        self.write_json(self.metadata_path, self.metadata)
        self.assertIn("metadata PLY 哈希一致", self.audit().failures)

    def test_rejects_stale_cache_after_any_runtime_input_changes(self):
        dependencies = [self.ply, self.ros_ply, self.nav_launch, self.nav_config]
        cache = self.ros_ply.with_suffix(".h5")
        cache.write_bytes(b"non-empty cache")
        for changed in dependencies:
            with self.subTest(changed=changed):
                for path in dependencies:
                    os.utime(path, ns=(100, 100))
                os.utime(cache, ns=(200, 200))
                os.utime(changed, ns=(300, 300))
                self.assertTrue(any("H5 缓存" in failure for failure in self.audit().failures))
        os.utime(cache, ns=(400, 400))
        self.assertEqual(self.audit().failures, [])

    def test_checks_explicit_cache_path_and_rejects_empty_cache(self):
        cache = self.base / "custom-cache.h5"
        cache.touch()
        self.assertTrue(any("H5 缓存" in failure for failure in self.audit(h5=cache).failures))

    def test_missing_canonical_ply_fails_without_traceback(self):
        self.ply.unlink()
        failures = self.audit().failures
        self.assertTrue(any(str(self.ply) in failure for failure in failures))

    def test_missing_or_malformed_report_is_an_audit_failure(self):
        self.report_path.unlink()
        self.assertTrue(any("JSON 报告可读取" in failure for failure in self.audit().failures))
        self.write(self.report_path, "[]")
        self.assertTrue(any("JSON 报告可读取" in failure for failure in self.audit().failures))

    def test_malformed_metadata_sections_fail_without_traceback(self):
        self.metadata["mesh_planner_multilevel"] = ["invalid section"]
        self.write_json(self.metadata_path, self.metadata)
        self.assertTrue(any("metadata 分节" in failure for failure in self.audit().failures))

    def test_checks_postprocessed_collision_provenance(self):
        data = self.collision.read_bytes().replace(b"test scene", b"processed ", 1)
        self.write(self.collision, data)
        shutil.copy2(self.collision, self.ros_sim / "models" / self.name / "meshes" / self.collision.name)
        self.metadata["gazebo_collision_postprocess"] = {
            "collision_sha256": sha256(self.collision),
            "source_sha256": sha256(self.visual), "output_faces": 1,
        }
        self.write_json(self.metadata_path, self.metadata)
        self.assertEqual(self.audit().failures, [])
        self.metadata["gazebo_collision_postprocess"]["source_sha256"] = "old visual"
        self.write_json(self.metadata_path, self.metadata)
        self.assertIn("碰撞后处理输入对应 visual STL", self.audit().failures)

    def test_accepts_pcd_navigation_provenance_as_a_meshnav_map(self):
        self.metadata["pcd_to_mesh_navigation"] = self.metadata.pop("mesh_planner_multilevel")
        self.metadata["pcd_to_mesh_navigation"]["report"] = "mesh_planner/pcd_surface.json"
        self.report["output_ply_sha256"] = sha256(self.ply)
        self.write_json(self.base / "mesh_planner/pcd_surface.json", self.report)
        self.write_json(self.metadata_path, self.metadata)
        self.report_path.unlink()
        self.assertEqual(self.audit().failures, [])

    def test_rejects_nonmanifold_or_failed_corridor_reports(self):
        self.report.update({"nonmanifold_edges": 1, "low_corridor_checks": [{"connected_on_lower_layer": False}]})
        self.write_json(self.report_path, self.report)
        failures = self.audit().failures
        self.assertIn("MeshNav 报告无非流形边", failures)
        self.assertIn("已记录的下层通道回归通过", failures)

    def test_rejects_visual_and_collision_uri_mixup_or_unit_rescaling(self):
        path = self.model / "model.sdf"
        text = path.read_text().replace(self.collision.name, self.visual.name)
        self.write(path, text.replace("1 1 1", "0.001 0.001 0.001"))
        shutil.copy2(path, self.ros_sim / "models" / self.name / "model.sdf")
        failures = self.audit().failures
        self.assertIn("SDF collision 使用对应 STL", failures)
        self.assertIn("SDF visual 使用米制 scale=1", failures)

    def test_preserves_optional_runtime_installation_checks(self):
        self.assertEqual(self.audit().failures, [])
        self.assertTrue(any("已构建运行文件存在" in failure for failure in self.audit(check_installed=True).failures))


if __name__ == "__main__":
    unittest.main()
