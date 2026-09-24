#!/usr/bin/env python3
"""Show the source PCD beside a MeshNav surface during map-only inspection."""

import argparse
from pathlib import Path
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def read_xyz(path, scale=1.0, translate=(0.0, 0.0, 0.0)):
    header = {}
    with Path(path).open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA entry")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key.upper()] = values
            if key.upper() == "DATA":
                break
        names = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        kinds = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(names))))
        count = int(header["POINTS"][0])
        if not (len(names) == len(sizes) == len(kinds) == len(counts)):
            raise ValueError("PCD field layout is inconsistent")
        if not all(axis in names for axis in ("x", "y", "z")):
            raise ValueError("PCD must contain x/y/z fields")
        data_type = header["DATA"][0].lower()
        if data_type == "binary":
            types = {(k, s): "<" + t for k, s, t in (
                ("F", 4, "f4"), ("F", 8, "f8"), ("I", 4, "i4"),
                ("U", 4, "u4"), ("U", 1, "u1"), ("I", 1, "i1"),
                ("I", 2, "i2"), ("U", 2, "u2"), ("I", 8, "i8"), ("U", 8, "u8"),
            )}
            dtype = np.dtype([
                (name, types[(kind.upper(), size)]) if number == 1
                else (name, types[(kind.upper(), size)], (number,))
                for name, size, kind, number in zip(names, sizes, kinds, counts)
            ])
            payload = stream.read(count * dtype.itemsize)
            if len(payload) != count * dtype.itemsize:
                raise ValueError("PCD binary data is truncated")
            records = np.frombuffer(payload, dtype=dtype, count=count)
            points = np.column_stack([records[axis] for axis in ("x", "y", "z")])
        elif data_type == "ascii":
            values = np.loadtxt(stream, dtype=np.float64, ndmin=2)
            if values.shape != (count, sum(counts)):
                raise ValueError("PCD ASCII point count or field count is inconsistent")
            offsets = np.cumsum([0] + counts[:-1])
            points = np.column_stack([values[:, offsets[names.index(axis)]]
                                      for axis in ("x", "y", "z")])
        else:
            raise ValueError("PCD must use DATA ascii or binary")
    points = points[np.isfinite(points).all(axis=1)]
    return np.ascontiguousarray(points * scale + np.asarray(translate), dtype="<f4")


class SourceCloud(Node):
    def __init__(self, path, frame, topic, scale, translate):
        super().__init__("meshnav_source_cloud")
        xyz = read_xyz(path, scale, translate)
        if not len(xyz):
            raise ValueError("PCD contains no finite XYZ points")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = self.create_publisher(PointCloud2, topic, qos)
        cloud = PointCloud2()
        cloud.header.frame_id = frame
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = len(xyz)
        cloud.fields = [PointField(name=axis, offset=offset,
                                   datatype=PointField.FLOAT32, count=1)
                        for offset, axis in ((0, "x"), (4, "y"), (8, "z"))]
        cloud.point_step = 12
        cloud.row_step = 12 * len(xyz)
        cloud.is_dense = True
        cloud.data = xyz.tobytes()
        self.publisher = publisher
        self.cloud = cloud
        self.publish()
        # Large PCD samples can miss late DDS readers on some transports.
        # Re-send slowly so RViz still gets the scene after it starts.
        self.create_timer(10.0, self.publish)
        self.get_logger().info(f"Published {len(xyz)} source points on {topic} ({frame})")

    def publish(self):
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.cloud)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcd", required=True)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--topic", default="/meshnav/source_cloud")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--translate", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.scale <= 0 or not np.isfinite([args.scale, *args.translate]).all():
        parser.error("scale must be positive and all transform values must be finite")
    rclpy.init()
    node = None
    try:
        node = SourceCloud(args.pcd, args.frame, args.topic, args.scale, args.translate)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # Launch can deliver SIGINT while rclpy is destroying entities.
            pass


if __name__ == "__main__":
    main()
