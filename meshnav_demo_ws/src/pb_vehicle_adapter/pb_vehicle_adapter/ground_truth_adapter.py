"""Publish one navigation TF/odometry chain from the PB chassis truth pose."""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


Quaternion = Tuple[float, float, float, float]
Vector = Tuple[float, float, float]


def _q_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _q_inverse(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    length_sq = x * x + y * y + z * z + w * w
    if length_sq < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / length_sq, -y / length_sq, -z / length_sq, w / length_sq)


def _rotate(quaternion: Quaternion, vector: Vector) -> Vector:
    rotated = _q_multiply(_q_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)), _q_inverse(quaternion))
    return (rotated[0], rotated[1], rotated[2])


def _subtract(left: Vector, right: Vector) -> Vector:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


class GroundTruthAdapter(Node):
    """Convert Gazebo chassis truth to map->odom->base_footprint and /odom."""

    def __init__(self) -> None:
        super().__init__("pb_ground_truth_adapter")
        self.declare_parameter("raw_pose_topic", "/pb/poses_raw")
        self.declare_parameter("world_pose_topic", "/pb/world_poses_raw")
        self.declare_parameter("world_frame", "rmuc2026_field")
        self.declare_parameter("model_frame", "robot")
        self.declare_parameter("chassis_frame", "robot/chassis")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_to_chassis_xyz", [0.0, 0.0, 0.076])
        self.declare_parameter("base_to_chassis_xyzw", [0.0, 0.0, 0.0, 1.0])

        self._world_frame = self.get_parameter("world_frame").value
        self._model_frame = self._normalise_frame(self.get_parameter("model_frame").value)
        self._chassis_frame = self._normalise_frame(self.get_parameter("chassis_frame").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._base_to_chassis_xyz = tuple(self.get_parameter("base_to_chassis_xyz").value)
        self._base_to_chassis_q = tuple(self.get_parameter("base_to_chassis_xyzw").value)
        if len(self._base_to_chassis_xyz) != 3 or len(self._base_to_chassis_q) != 4:
            raise ValueError("PB base/chassis extrinsic must contain xyz and xyzw")

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self.create_subscription(
            TFMessage, self.get_parameter("raw_pose_topic").value, self._pose_callback, 20
        )
        self.create_subscription(
            TFMessage, self.get_parameter("world_pose_topic").value, self._world_pose_callback, 20
        )
        self._previous: Optional[Tuple[float, Vector, Quaternion]] = None
        self._world_to_model: Optional[TransformStamped] = None
        self._publish_map_to_odom()

    @staticmethod
    def _normalise_frame(frame: str) -> str:
        return frame.lstrip("/").replace("::", "/")

    def _publish_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(transform)

    @staticmethod
    def _transform_tuple(transform: TransformStamped) -> Tuple[Vector, Quaternion]:
        return (
            (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
            (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        )

    def _find_chassis(self, message: TFMessage) -> Optional[TransformStamped]:
        model_to_chassis = None
        for transform in message.transforms:
            child = self._normalise_frame(transform.child_frame_id)
            parent = self._normalise_frame(transform.header.frame_id)
            if child == self._chassis_frame and parent == self._model_frame:
                model_to_chassis = transform
            if child == self._chassis_frame and parent in (self._world_frame, "world", ""):
                return transform
        if self._world_to_model is not None and model_to_chassis is not None:
            model_t, model_q = self._transform_tuple(self._world_to_model)
            chassis_t, chassis_q = self._transform_tuple(model_to_chassis)
            composed = TransformStamped()
            composed.header = self._world_to_model.header
            composed.child_frame_id = self._chassis_frame
            translated = _rotate(model_q, chassis_t)
            composed.transform.translation.x = model_t[0] + translated[0]
            composed.transform.translation.y = model_t[1] + translated[1]
            composed.transform.translation.z = model_t[2] + translated[2]
            composed.transform.rotation.x, composed.transform.rotation.y, composed.transform.rotation.z, composed.transform.rotation.w = _q_multiply(model_q, chassis_q)
            return composed
        return None

    def _world_pose_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            child = self._normalise_frame(transform.child_frame_id)
            if child == self._model_frame or child.endswith("/" + self._model_frame):
                self._world_to_model = transform
                return

    def _pose_callback(self, message: TFMessage) -> None:
        source = self._find_chassis(message)
        if source is None:
            self.get_logger().warn(
                "Waiting for chassis transform '%s' on %s" % (self._chassis_frame, self.get_parameter("raw_pose_topic").value),
                throttle_duration_sec=5.0,
            )
            return

        chassis_t = (
            source.transform.translation.x,
            source.transform.translation.y,
            source.transform.translation.z,
        )
        chassis_q = (
            source.transform.rotation.x,
            source.transform.rotation.y,
            source.transform.rotation.z,
            source.transform.rotation.w,
        )
        # world_T_base = world_T_chassis * inverse(base_T_chassis).
        inverse_q = _q_inverse(self._base_to_chassis_q)
        inverse_t = _rotate(inverse_q, tuple(-value for value in self._base_to_chassis_xyz))
        base_t = tuple(
            chassis_t[index] + _rotate(chassis_q, inverse_t)[index] for index in range(3)
        )
        base_q = _q_multiply(chassis_q, inverse_q)

        transform = TransformStamped()
        transform.header.stamp = source.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = base_t[0]
        transform.transform.translation.y = base_t[1]
        transform.transform.translation.z = base_t[2]
        transform.transform.rotation.x = base_q[0]
        transform.transform.rotation.y = base_q[1]
        transform.transform.rotation.z = base_q[2]
        transform.transform.rotation.w = base_q[3]
        self._tf_broadcaster.sendTransform(transform)
        self._publish_odom(source, base_t, base_q)

    def _publish_odom(self, source: TransformStamped, position: Vector, orientation: Quaternion) -> None:
        message = Odometry()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = position[0]
        message.pose.pose.position.y = position[1]
        message.pose.pose.position.z = position[2]
        message.pose.pose.orientation.x = orientation[0]
        message.pose.pose.orientation.y = orientation[1]
        message.pose.pose.orientation.z = orientation[2]
        message.pose.pose.orientation.w = orientation[3]

        stamp = source.header.stamp.sec + source.header.stamp.nanosec * 1.0e-9
        if self._previous is not None:
            previous_stamp, previous_position, previous_orientation = self._previous
            delta_t = stamp - previous_stamp
            if 1.0e-4 < delta_t < 1.0:
                world_velocity = tuple(
                    (position[index] - previous_position[index]) / delta_t for index in range(3)
                )
                body_velocity = _rotate(_q_inverse(orientation), world_velocity)
                message.twist.twist.linear = Vector3(
                    x=body_velocity[0], y=body_velocity[1], z=body_velocity[2]
                )
                delta_q = _q_multiply(_q_inverse(previous_orientation), orientation)
                vector_length = math.sqrt(delta_q[0] ** 2 + delta_q[1] ** 2 + delta_q[2] ** 2)
                if vector_length > 1.0e-9:
                    angle = 2.0 * math.atan2(vector_length, delta_q[3])
                    message.twist.twist.angular.x = delta_q[0] * angle / (vector_length * delta_t)
                    message.twist.twist.angular.y = delta_q[1] * angle / (vector_length * delta_t)
                    message.twist.twist.angular.z = delta_q[2] * angle / (vector_length * delta_t)
        self._previous = (stamp, position, orientation)
        self._odom_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = GroundTruthAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
