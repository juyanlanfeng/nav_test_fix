"""Publish the DDDMR static map bundle as latched PointCloud2 topics.

Loads rmuc2026_mapcloud.pcd and rmuc2026_mapground.pcd (float32 x/y/z/intensity)
and publishes them on /dddmr/mapcloud and /dddmr/mapground with Reliable +
Transient Local QoS so that DDDMR perception started later still receives the
maps.  Exits non-zero when the bundle is missing, empty or non-finite instead of
publishing an empty cloud and claiming readiness.
"""

import sys
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import rclpy


DEFAULT_MAP_DIR = "/home/rainple/nav_test/field/converted_rmuc2026/dddmr_nav"
PCD_FIELDS = ("x", "y", "z", "intensity")


def read_pcd_xyzi(path):
    """Read a binary PCD with float32 x/y/z/intensity; return an (n, 4) array."""
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("%s: truncated header" % path)
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, _, value = text.partition(" ")
            header[key.upper()] = value
            if key.upper() == "DATA":
                break
        if header.get("DATA", "").lower() != "binary":
            raise ValueError("%s: only binary PCD is supported" % path)
        if header.get("FIELDS", "").split() != list(PCD_FIELDS):
            raise ValueError("%s: expected fields %s" % (path, PCD_FIELDS))
        count = int(header["POINTS"])
        raw = stream.read(count * 16)
    points = np.frombuffer(raw, dtype="<f4").reshape(-1, 4).astype(np.float64)
    if len(points) != count:
        raise ValueError("%s: expected %d points, read %d" % (path, count, len(points)))
    return points


def map_qos():
    """Reliable + Transient Local QoS so late DDDMR subscribers still get the map."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_pointcloud2(points, frame_id, stamp):
    """Build a PointCloud2 with float32 x/y/z/intensity from an (n, 4) array."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.is_dense = True
    msg.data = np.asarray(points, dtype="<f4").tobytes()
    return msg


class DddmrMapPublisher(Node):
    def __init__(self):
        super().__init__("pb_dddmr_map_publisher")
        self.declare_parameter("map_dir", DEFAULT_MAP_DIR)
        self.declare_parameter("mapcloud_file", "rmuc2026_mapcloud.pcd")
        self.declare_parameter("mapground_file", "rmuc2026_mapground.pcd")
        self.declare_parameter("mapcloud_topic", "/dddmr/mapcloud")
        self.declare_parameter("mapground_topic", "/dddmr/mapground")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_s", 2.0)

        map_dir = Path(self.get_parameter("map_dir").value)
        self._frame_id = self.get_parameter("frame_id").value
        cloud = self._load(map_dir / self.get_parameter("mapcloud_file").value)
        ground = self._load(map_dir / self.get_parameter("mapground_file").value)

        qos = map_qos()
        self._cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter("mapcloud_topic").value, qos)
        self._ground_pub = self.create_publisher(
            PointCloud2, self.get_parameter("mapground_topic").value, qos)
        self._cloud_points = cloud
        self._ground_points = ground
        self._publish()
        self._timer = self.create_timer(
            float(self.get_parameter("publish_period_s").value), self._publish)
        self.get_logger().info(
            "DDDMR map publisher ready: %d mapcloud / %d mapground points on %s and %s"
            % (len(cloud), len(ground), self.get_parameter("mapcloud_topic").value,
               self.get_parameter("mapground_topic").value))

    @staticmethod
    def _load(path):
        if not path.is_file():
            raise SystemExit("DDDMR map file missing: %s" % path)
        points = read_pcd_xyzi(path)
        if len(points) == 0:
            raise SystemExit("DDDMR map file is empty: %s" % path)
        if not np.isfinite(points[:, :3]).all():
            raise SystemExit("DDDMR map file has non-finite coordinates: %s" % path)
        return points

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self._cloud_pub.publish(make_pointcloud2(self._cloud_points, self._frame_id, stamp))
        self._ground_pub.publish(make_pointcloud2(self._ground_points, self._frame_id, stamp))


def main():
    rclpy.init()
    try:
        node = DddmrMapPublisher()
    except SystemExit as exc:
        print("pb_dddmr_map_publisher: %s" % exc, file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
