#!/usr/bin/env python3
"""Unit tests for cheap RMUC project audit helpers."""

import os
from pathlib import Path
import tempfile
import unittest

from verify_rmuc_project import cache_is_fresh


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


if __name__ == "__main__":
    unittest.main()
