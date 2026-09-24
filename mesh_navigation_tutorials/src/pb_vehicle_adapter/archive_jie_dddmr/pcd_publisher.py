"""Publish a static PCD as a latched (transient local) PointCloud2.

`pcl_ros/pcd_to_pointcloud` publishes with `Durability: VOLATILE` (verified on
this machine), while RViz's PointCloud2 display subscribes with
`Durability: Transient Local` by default.  Those two are incompatible: RViz
prints "incompatible QoS ... Last incompatible policy: DURABILITY_QOS_POLICY"
and never receives a single point, no matter whether the publisher sent the cloud
once or repeatedly.  That is why the map/floor clouds were invisible.

This node is the drop-in replacement used by `pb_jie.launch.py`: it reads the PCD
once, publishes it with `TRANSIENT_LOCAL + RELIABLE + KEEP_LAST(1)` and stays
alive, so any subscriber that appears later - RViz restarted, a second RViz, a
debug echo - still receives the cloud without the publisher streaming megabytes
per second.

Parameters
    file_name (str)        PCD file to publish (binary or ascii, x/y/z fields)
    topic (str)            topic name, default /cloud_pcd
    frame_id (str)         header frame, default map
    period (float)         0.0 = publish once and latch (default); > 0 re-publishes
    durability (str)       transient_local (default) or volatile
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {
    ("I", 1): np.dtype("i1"), ("I", 2): np.dtype("i2"), ("I", 4): np.dtype("i4"), ("I", 8): np.dtype("i8"),
    ("U", 1): np.dtype("u1"), ("U", 2): np.dtype("u2"), ("U", 4): np.dtype("u4"), ("U", 8): np.dtype("u8"),
    ("F", 4): np.dtype("<f4"), ("F", 8): np.dtype("<f8"),
}
_OFFSETS = {"x": 0.0, "y": 4.0, "z": 8.0}


def read_pcd_xyz(path):
    """Return an (N, 3) float32 array of the x/y/z fields of a PCD file."""
    header = {}
    with open(path, "rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise RuntimeError("%s: PCD header ended before DATA" % path)
            line = raw.decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            key = key.upper()
            header[key] = value.strip()
            if key == "DATA":
                break
        data_kind = header["DATA"].lower()
        fields = header["FIELDS"].split()
        sizes = [int(value) for value in header["SIZE"].split()]
        types = header["TYPE"].split()
        counts = [int(value) for value in header.get("COUNT", " ".join("1" * len(fields))).split()]
        if not all(name in _OFFSETS for name in ("x", "y", "z")):
            raise RuntimeError("%s: PCD has no x/y/z fields (%s)" % (path, fields))
        if len(counts) < len(fields):
            counts = counts + [1] * (len(fields) - len(counts))
        offsets, itemsize = [], 0
        for size, count in zip(sizes, counts):
            offsets.append(itemsize)
            itemsize += size * count
        names = [name for name in fields]
        formats = [_DATATYPES[(kind, size)] for size, kind in zip(sizes, types)]
        structured = np.dtype({"names": names, "formats": formats,
                               "offsets": offsets, "itemsize": itemsize})

        if data_kind == "binary":
            raw_values = np.fromfile(stream, dtype=structured)
        elif data_kind == "ascii":
            raw_values = np.loadtxt(stream, dtype=structured, ndmin=1)
        else:
            raise RuntimeError("%s: unsupported DATA %s" % (path, data_kind))

    points = np.empty((len(raw_values), 3), dtype=np.float32)
    for axis, name in enumerate(("x", "y", "z")):
        points[:, axis] = raw_values[name].astype(np.float32)
    return points


class PcdPublisher(Node):
    def __init__(self):
        super().__init__("pb_pcd_publisher")
        self.declare_parameter("file_name", "")
        self.declare_parameter("topic", "/cloud_pcd")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("period", 0.0)
        self.declare_parameter("durability", "transient_local")

        # rclpy returns the value directly (rclcpp's as_string() does not exist here).
        path = str(self.get_parameter("file_name").value)
        topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        period = float(self.get_parameter("period").value)
        durability = str(self.get_parameter("durability").value)
        if not path:
            raise RuntimeError("parameter file_name is required")

        policy = (DurabilityPolicy.TRANSIENT_LOCAL if durability == "transient_local"
                  else DurabilityPolicy.VOLATILE)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE, durability=policy)
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.points = read_pcd_xyz(path)
        self.message = self.build_message(self.points, self.frame_id)
        self.get_logger().info(
            "publishing %d points from %s on %s (frame %s, %s, %s)"
            % (len(self.points), path, topic, self.frame_id, durability,
               "latched once" if period <= 0.0 else "every %.2f s" % period))
        # transient local keeps the last sample for later subscribers; the node
        # has to stay alive for that, which is why it spins instead of exiting.
        self.publish()
        if period > 0.0:
            self.create_timer(period, self.publish)

    def build_message(self, points, frame_id):
        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.height = 1
        message.width = int(len(points))
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * int(len(points))
        message.is_dense = True
        message.data = np.ascontiguousarray(points, dtype="<f4").tobytes()
        return message

    def publish(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = PcdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
