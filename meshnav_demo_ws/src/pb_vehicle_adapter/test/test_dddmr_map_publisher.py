"""Tests for the DDDMR map PCD reader and PointCloud2 builder."""

import struct

from builtin_interfaces.msg import Time
import numpy as np

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from pb_vehicle_adapter.dddmr_map_publisher import map_qos, make_pointcloud2, read_pcd_xyzi


def _write_pcd(path, points):
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
        "COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
        % (len(points), len(points))
    )
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        for x, y, z, intensity in points:
            stream.write(struct.pack("<ffff", x, y, z, intensity))


def test_read_pcd_xyzi_roundtrip(tmp_path):
    source = [(1.0, 2.0, 3.0, 0.0), (-1.5, 0.25, 0.5, 0.0)]
    path = tmp_path / "cloud.pcd"
    _write_pcd(path, source)
    points = read_pcd_xyzi(path)
    assert points.shape == (2, 4)
    assert np.allclose(points[:, :3], [[1.0, 2.0, 3.0], [-1.5, 0.25, 0.5]])


def test_make_pointcloud2_layout():
    points = np.array([[1.0, 2.0, 3.0, 0.0], [4.0, 5.0, 6.0, 0.0]], dtype=np.float32)
    msg = make_pointcloud2(points, "map", Time(sec=1, nanosec=0))
    assert msg.header.frame_id == "map"
    assert msg.height == 1 and msg.width == 2
    assert msg.point_step == 16
    assert msg.row_step == 32
    assert [f.name for f in msg.fields] == ["x", "y", "z", "intensity"]
    assert len(msg.data) == 32
    assert msg.is_dense


def test_map_qos_is_latched_reliable():
    qos = map_qos()
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.depth == 1
