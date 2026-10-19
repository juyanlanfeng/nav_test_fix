#!/usr/bin/env python3
"""Coverage analysis uses the common PCD reader for surface point clouds."""

from pathlib import Path
import tempfile
import unittest

import numpy as np

from compare_step_pcd_coverage import coverage


class CoverageTest(unittest.TestCase):
    def test_ascii_cloud_with_extra_fields_and_negative_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "surface.pcd"
            source.write_text(
                "VERSION 0.7\nFIELDS x y z intensity\n"
                "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
                "-0.01 0.01 0.01 7\n0.11 0.01 0.01 8\n"
            )
            hits = np.asarray([
                [-0.02, 0.02, 0.02], [0.12, 0.02, 0.02], [0.02, 0.02, 0.02]
            ])

            report = coverage(source, 0.1, hits)

            self.assertEqual(report["points"], 2)
            self.assertEqual(report["covered_hits"], 2)
            self.assertAlmostEqual(report["coverage_fraction"], 2 / 3)
            np.testing.assert_allclose(report["uncovered_xyz"], hits[2:])


if __name__ == "__main__":
    unittest.main()
