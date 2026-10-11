"""Create an isolated synthetic bundle and publish its fake hardware inputs.

These helpers never publish navigation goals, navigation enable messages, or
chassis commands.  Use a separate terminal in the same isolated ROS domain to
inspect the navigation stack and deliberately exercise its command gateway.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
from pathlib import Path
import struct
import sys

import yaml


_CONFIG_FILES = (
    "robot.yaml",
    "meshnav_real.yaml",
    "pcl_real.yaml",
    "interfaces_real.yaml",
)
_SMOKE_DOMAIN = 209
# The synthetic TF already names the measurement origin; offset_xyz therefore
# remains zero in robot.yaml.  This is a mount height, not a PCL origin offset.
_LIDAR_MOUNT = (0.0, 0.0, 0.40)


def _reference_config_dir() -> Path:
    """Find installed reference files, with a source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = Path(get_package_share_directory("meshnav_real_bringup")) / "config"
        if all((candidate / filename).is_file() for filename in _CONFIG_FILES):
            return candidate
    except (ImportError, LookupError):
        pass
    candidate = Path(__file__).resolve().parents[1] / "config"
    if not all((candidate / filename).is_file() for filename in _CONFIG_FILES):
        raise ValueError(f"reference configuration files are missing in {candidate}")
    return candidate


def _plane_points() -> list[tuple[float, float, float]]:
    # 32 cells per side; the 8 m square has upward-pointing face normals.
    return [(-4.0 + ix * 0.25, -4.0 + iy * 0.25, 0.0)
            for iy in range(33) for ix in range(33)]


def _plane_ply(points: list[tuple[float, float, float]]) -> bytes:
    rows = [
        "ply", "format ascii 1.0", "comment SYNTHETIC DESKTOP SMOKE MAP",
        f"element vertex {len(points)}", "property float x", "property float y",
        "property float z", f"element face {32 * 32 * 2}",
        "property list uchar int vertex_indices", "end_header",
    ]
    rows.extend(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
    for iy in range(32):
        for ix in range(32):
            lower_left = iy * 33 + ix
            rows.append(f"3 {lower_left} {lower_left + 1} {lower_left + 33}")
            rows.append(f"3 {lower_left + 1} {lower_left + 34} {lower_left + 33}")
    return ("\n".join(rows) + "\n").encode("ascii")


def _plane_pcd(points: list[tuple[float, float, float]]) -> bytes:
    rows = [
        "# .PCD v0.7 - SYNTHETIC DESKTOP SMOKE BACKGROUND", "VERSION 0.7",
        "FIELDS x y z", "SIZE 4 4 4", "TYPE F F F", "COUNT 1 1 1",
        f"WIDTH {len(points)}", "HEIGHT 1", "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}", "DATA ascii",
    ]
    rows.extend(f"{x:.6f} {y:.6f} {z:.6f}" for x, y, z in points)
    return ("\n".join(rows) + "\n").encode("ascii")


def _yaml_bytes(value: dict) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def make_bundle_main(args: list[str] | None = None) -> int:
    """Generate only the named smoke files; never delete an output directory."""
    parser = argparse.ArgumentParser(description="Create a synthetic desktop smoke bundle")
    parser.add_argument("--output", required=True, type=Path, help="separate smoke bundle directory")
    parser.add_argument("--overwrite", "--force", action="store_true",
                        help="overwrite the six generated files; do not remove other files")
    options = parser.parse_args(args)
    try:
        reference = _reference_config_dir()
        documents = {}
        for filename in _CONFIG_FILES:
            with (reference / filename).open(encoding="utf-8") as stream:
                documents[filename] = yaml.safe_load(stream)
            if not isinstance(documents[filename], dict):
                raise ValueError(f"{filename} must contain a YAML mapping")

        points = _plane_points()
        mesh = _plane_ply(points)
        background = _plane_pcd(points)
        robot = documents["robot.yaml"]
        robot.update({
            "schema_version": 1,
            "deployment_kind": "desktop_smoke",
            "integration": {
                "pcl_source_patch_verified": True,
                "background_alignment_verified": True,
                "localization_verified": True,
                "tf_ownership_verified": True,
            },
            "robot": {
                "length_m": 0.40, "width_m": 0.30, "height_m": 0.20,
                "collision_radius_m": 0.25, "safety_margin_m": 0.10,
                "height_diff_threshold_m": 0.10, "holonomic": True,
            },
            "frames": {"map": "map", "odom": "odom", "base": "base_footprint",
                       "lidar": "lidar_link"},
            "sensors": {
                "offset_xyz": [0.0, 0.0, 0.0], "min_range_m": 0.10,
                "max_range_m": 8.0, "horizontal_fov_deg": 360.0,
                "vertical_fov_min_deg": -85.0, "vertical_fov_max_deg": 85.0,
            },
            "perception": {
                "distance_threshold_m": 0.08, "self_filter_radius_m": 0.20,
                "obstacle_projection_height_m": 0.50, "obstacle_keep_time_s": 2.0,
                "max_observation_age_s": 0.30, "max_future_s": 0.001,
                "static_inflation_radius_m": 0.60, "dynamic_inflation_radius_m": 0.60,
            },
            "motion": {
                "max_linear_velocity": 0.15, "max_angular_velocity": 0.20,
                "max_linear_acceleration": 0.30, "max_angular_acceleration": 0.50,
            },
            "map": {
                "mesh_file": "maps/smoke_nav.ply", "background_file": "maps/smoke_background.pcd",
                "version": "synthetic_plane_v1",
                "mesh_sha256": hashlib.sha256(mesh).hexdigest(),
                "background_sha256": hashlib.sha256(background).hexdigest(),
            },
        })
        gateway = documents["interfaces_real.yaml"]["command_gateway"]["ros__parameters"]
        gateway["output_topic"] = "/meshnav_smoke/chassis_cmd"

        output = options.output.expanduser().resolve()
        if (output / "maps").is_symlink():
            raise ValueError("refusing to write through a symlinked maps directory")
        generated = {filename: _yaml_bytes(value) for filename, value in documents.items()}
        generated.update({"maps/smoke_nav.ply": mesh, "maps/smoke_background.pcd": background})
        for relative in generated:
            destination = output / relative
            if destination.is_symlink():
                raise ValueError(f"refusing to overwrite symlink: {destination}")
            if destination.exists() and (not destination.is_file() or not options.overwrite):
                raise ValueError(f"already exists: {destination}; use --overwrite for generated files")
        (output / "maps").mkdir(parents=True, exist_ok=True)
        for relative, data in generated.items():
            (output / relative).write_bytes(data)

        from meshnav_real_bringup.bundle import load_bundle

        load_bundle(output)
        print(f"Synthetic desktop bundle: {output}")
        print("Gateway output: /meshnav_smoke/chassis_cmd; navigation remains disabled.")
        print("Run fake inputs and navigation with ROS_DOMAIN_ID=209 and ROS_LOCALHOST_ONLY=1.")
        return 0
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"Cannot create smoke bundle: {error}", file=sys.stderr)
        return 2


def _read_ascii_background(path: Path) -> list[tuple[float, float, float]]:
    """Read the deliberately simple XYZ PCD produced by make_bundle_main."""
    points = []
    in_data = False
    for line in path.read_text(encoding="ascii").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if not in_data:
            if line.startswith("DATA "):
                if line != "DATA ascii":
                    raise ValueError("smoke inputs require the generated ASCII background PCD")
                in_data = True
            continue
        values = line.split()
        if len(values) != 3:
            raise ValueError("smoke background must contain three XYZ fields")
        point = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in point):
            raise ValueError("smoke background contains nonfinite coordinates")
        points.append(point)
    if not points:
        raise ValueError("smoke background contains no points")
    return points


def fake_hardware_main(args: list[str] | None = None) -> int:
    """Publish synthetic normalized hardware inputs in a localhost-only domain."""
    parser = argparse.ArgumentParser(description="Publish stationary synthetic hardware inputs")
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--domain-id", type=int, default=None,
                        help="isolated ROS domain (default: 209); never use the robot's live domain")
    parser.add_argument("--rate", type=float, default=10.0, help="input frequency in Hz (1 through 50)")
    options, ros_args = parser.parse_known_args(args)
    domain = _SMOKE_DOMAIN if options.domain_id is None else options.domain_id
    if not 1 <= domain <= 232:
        parser.error("--domain-id must be an isolated domain between 1 and 232")
    if not math.isfinite(options.rate) or not 1.0 <= options.rate <= 50.0:
        parser.error("--rate must be between 1 and 50 Hz")
    inherited_domain = os.environ.get("ROS_DOMAIN_ID")
    if options.domain_id is None and inherited_domain not in (None, "", str(domain)):
        parser.error(f"inherited ROS_DOMAIN_ID={inherited_domain}; pass an explicit isolated --domain-id")
    # Set isolation before importing ROS libraries, and always before rclpy.init().
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    try:
        from meshnav_real_bringup.bundle import load_bundle

        bundle = load_bundle(options.bundle.expanduser().resolve())
        robot = bundle["robot"]
        if robot["deployment_kind"] != "desktop_smoke":
            raise ValueError("fake hardware accepts only a desktop_smoke bundle")
        gateway = bundle["gateway"]
        if not gateway["output_topic"].startswith("/meshnav_smoke/"):
            raise ValueError("smoke gateway output must remain under /meshnav_smoke/")
        points_map = _read_ascii_background(Path(bundle["background_file"]))
        frames = robot["frames"]
        offset = _LIDAR_MOUNT
        points_lidar = [(x - offset[0], y - offset[1], z - offset[2])
                        for x, y, z in points_map]
        points_lidar = [point for point in points_lidar
                        if robot["sensors"]["min_range_m"] <= math.sqrt(sum(v * v for v in point))
                        <= robot["sensors"]["max_range_m"]
                        and robot["sensors"]["vertical_fov_min_deg"]
                        <= math.degrees(math.atan2(point[2], math.hypot(point[0], point[1])))
                        <= robot["sensors"]["vertical_fov_max_deg"]]
        if not points_lidar:
            raise ValueError("synthetic cloud is empty after the configured range limits")
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"Cannot start fake hardware: {error}", file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Bool
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class FakeHardware(Node):
        def __init__(self) -> None:
            super().__init__("meshnav_fake_hardware")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("fake hardware requires wall time (use_sim_time=false)")
            self.cloud = self.create_publisher(PointCloud2, "/cloud", qos_profile_sensor_data)
            self.odom = self.create_publisher(Odometry, "/odom", QoSProfile(depth=10))
            state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.ready = self.create_publisher(Bool, "/hardware/ready", state_qos)
            self.estop = self.create_publisher(Bool, "/hardware/estop", state_qos)
            self.healthy = self.create_publisher(Bool, "/localization/healthy", state_qos)
            self.dynamic_tf = TransformBroadcaster(self)
            self.static_tf = StaticTransformBroadcaster(self)
            lidar = TransformStamped()
            lidar.header.stamp = self.get_clock().now().to_msg()
            lidar.header.frame_id = frames["base"]
            lidar.child_frame_id = frames["lidar"]
            lidar.transform.translation.x = float(offset[0])
            lidar.transform.translation.y = float(offset[1])
            lidar.transform.translation.z = float(offset[2])
            lidar.transform.rotation.w = 1.0
            self.static_tf.sendTransform(lidar)
            self.cloud_data = b"".join(struct.pack("<fff", *point) for point in points_lidar)
            self.timer = self.create_timer(1.0 / options.rate, self.publish_inputs)
            self.get_logger().info(
                f"Synthetic stationary inputs in ROS_DOMAIN_ID={domain}, localhost only. "
                "No goals, navigation enable, or chassis commands are published.")

        def publish_inputs(self) -> None:
            stamp = self.get_clock().now().to_msg()
            transforms = []
            for parent, child in ((frames["map"], frames["odom"]),
                                  (frames["odom"], frames["base"])):
                transform = TransformStamped()
                transform.header.stamp = stamp
                transform.header.frame_id = parent
                transform.child_frame_id = child
                transform.transform.rotation.w = 1.0
                transforms.append(transform)
            self.dynamic_tf.sendTransform(transforms)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = frames["odom"]
            odom.child_frame_id = frames["base"]
            odom.pose.pose.orientation.w = 1.0
            for diagonal in (0, 7, 14, 21, 28, 35):
                odom.pose.covariance[diagonal] = 0.0001
                odom.twist.covariance[diagonal] = 0.0001
            self.odom.publish(odom)
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = frames["lidar"]
            cloud.height = 1
            cloud.width = len(points_lidar)
            cloud.fields = [PointField(name=name, offset=index * 4,
                                       datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(("x", "y", "z"))]
            cloud.is_bigendian = False
            cloud.point_step = 12
            cloud.row_step = cloud.point_step * cloud.width
            cloud.is_dense = True
            cloud.data = self.cloud_data
            self.cloud.publish(cloud)
            self.ready.publish(Bool(data=True))
            self.estop.publish(Bool(data=False))
            self.healthy.publish(Bool(data=True))

    node = None
    rclpy.init(args=["meshnav_fake_hardware", *ros_args])
    try:
        node = FakeHardware()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as error:
        print(f"Fake hardware stopped: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
