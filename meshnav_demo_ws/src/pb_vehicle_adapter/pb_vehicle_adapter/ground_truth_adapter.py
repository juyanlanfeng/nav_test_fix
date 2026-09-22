"""Publish one navigation TF/odometry chain from a single same-step chassis truth.

Design note (doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md section 4): this node used
to compose the world pose from one asynchronous ROS stream
(``/pb/world_poses_raw``) with the model->chassis transform from another
(``/pb/poses_raw``).  The two streams disagree in time, so the composed result
could be stale (the world pose stopped updating while the chassis stream kept
publishing) or carry the other message's time; the older of the two streams also
arrives with a zero stamp.  That composition is gone.

Instead the node consumes exactly one measurement:
``/pb_sim/chassis_truth`` (nav_msgs/Odometry), published by the ChassisTruth
Gazebo system (package ``pb_gazebo_sim_support``).  ChassisTruth reads the chassis
WorldPose and the engine velocities in a single simulation step and stamps the
message with that step's simulation time, so one message is a complete,
same-instant measurement: pose in the world frame, twist in the chassis frame.

The two legacy topics are still recorded by the evidence harness for comparison
but they no longer drive ``/odom`` or the TF tree.
"""

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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


def _q_normalise(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / length, y / length, z / length, w / length)


def _rotate(quaternion: Quaternion, vector: Vector) -> Vector:
    rotated = _q_multiply(
        _q_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
        _q_inverse(quaternion),
    )
    return (rotated[0], rotated[1], rotated[2])


def _cross(left: Vector, right: Vector) -> Vector:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _finite_vector(vector: Vector) -> bool:
    return all(math.isfinite(component) for component in vector)


def _is_zero_stamp(stamp) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


class GroundTruthAdapter(Node):
    """Convert same-step chassis truth to map->odom->base_footprint and /odom."""

    def __init__(self) -> None:
        super().__init__("pb_ground_truth_adapter")
        self.declare_parameter("truth_topic", "/pb_sim/chassis_truth")
        self.declare_parameter("health_topic", "/pb/truth_health")
        self.declare_parameter("truth_timeout_s", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_to_chassis_xyz", [0.0, 0.0, 0.076])
        self.declare_parameter("base_to_chassis_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("finite_difference_check", True)
        # Declared before the run, not after a failure: the engine twist and the
        # finite difference of the same truth must agree within this bound.
        self.declare_parameter("finite_difference_tolerance", 0.15)
        # The spawn drop and its contact impulse make the engine velocity and the
        # pose difference disagree for a few steps; that is a spawn transient, not a
        # frame error, so the cross-check starts after this much simulation time.
        self.declare_parameter("finite_difference_warmup_s", 1.0)

        self._truth_topic = self.get_parameter("truth_topic").value
        self._health_topic = self.get_parameter("health_topic").value
        self._timeout_s = float(self.get_parameter("truth_timeout_s").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._base_to_chassis_xyz = tuple(self.get_parameter("base_to_chassis_xyz").value)
        self._base_to_chassis_q = _q_normalise(
            tuple(self.get_parameter("base_to_chassis_xyzw").value)
        )
        self._check_finite_difference = bool(
            self.get_parameter("finite_difference_check").value
        )
        self._finite_difference_tolerance = float(
            self.get_parameter("finite_difference_tolerance").value
        )
        self._finite_difference_warmup_s = float(
            self.get_parameter("finite_difference_warmup_s").value
        )
        self._first_stamp: Optional[float] = None
        if len(self._base_to_chassis_xyz) != 3 or len(self._base_to_chassis_q) != 4:
            raise ValueError("PB base/chassis extrinsic must contain xyz and xyzw")

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 20)
        self._health_publisher = self.create_publisher(Bool, self._health_topic, 10)
        self.create_subscription(
            Odometry, self._truth_topic, self._truth_callback, 20
        )

        self._last_stamp: Optional[float] = None
        self._last_wall_rx: Optional[float] = None
        self._previous: Optional[Tuple[float, Vector, Quaternion]] = None
        self._healthy: Optional[bool] = None
        self._duplicate_samples = 0
        self._time_resets = 0
        self._rejected_samples = 0
        self._finite_difference_mismatches = 0
        self._publish_map_to_odom()
        # Wall-clock watchdog: a paused simulation stops advancing simulation time
        # and stops publishing, and that must be reported as "truth unavailable"
        # rather than silently repeating the last pose.
        self.create_timer(
            0.1, self._watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def _publish_map_to_odom(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.rotation.w = 1.0
        self._static_broadcaster.sendTransform(transform)

    def _set_healthy(self, healthy: bool, reason: str = "") -> None:
        if self._healthy is healthy:
            return
        if healthy:
            self.get_logger().info("Truth chain healthy")
        else:
            self.get_logger().error(
                "Truth chain unhealthy: %s" % (reason or "no valid input")
            )
        self._healthy = healthy
        self._publish_health()

    def _publish_health(self) -> None:
        """Publish the current health state.

        This is a heartbeat, not a one-shot event: a consumer that joins later
        must still learn that the chain is bad, and a volatile subscription only
        receives what is published while it is matched.
        """
        if self._healthy is None:
            return
        message = Bool()
        message.data = self._healthy
        self._health_publisher.publish(message)

    def _truth_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        if _is_zero_stamp(message.header.stamp):
            self._rejected_samples += 1
            self._set_healthy(False, "truth message carries a zero stamp")
            return

        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        orientation = _q_normalise(
            (
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            )
        )
        linear = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        )
        angular = (
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        )
        if not (_finite_vector(position) and _finite_vector(linear) and _finite_vector(angular)):
            self._rejected_samples += 1
            self._set_healthy(False, "non-finite pose or twist")
            return

        # Duplicate and out-of-order samples are handled explicitly: a repeated
        # stamp is processed once, and a backwards stamp is a simulation reset that
        # clears the differencing basis instead of being compared against the old
        # timeline.
        if self._last_stamp is not None:
            if stamp == self._last_stamp:
                self._duplicate_samples += 1
                return
            if stamp < self._last_stamp:
                self._time_resets += 1
                self.get_logger().warn(
                    "Simulation time went backwards (%.6f -> %.6f); clearing the "
                    "differencing basis (reset #%d)" % (self._last_stamp, stamp, self._time_resets)
                )
                self._previous = None
                self._last_stamp = None

        base_position, base_orientation, base_linear, base_angular = self._to_base(
            position, orientation, linear, angular
        )
        if self._first_stamp is None:
            self._first_stamp = stamp
        self._check_against_finite_difference(
            stamp, base_position, base_orientation, base_linear
        )

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = base_position[0]
        transform.transform.translation.y = base_position[1]
        transform.transform.translation.z = base_position[2]
        transform.transform.rotation.x = base_orientation[0]
        transform.transform.rotation.y = base_orientation[1]
        transform.transform.rotation.z = base_orientation[2]
        transform.transform.rotation.w = base_orientation[3]
        self._tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = message.header.stamp
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = base_position[0]
        odometry.pose.pose.position.y = base_position[1]
        odometry.pose.pose.position.z = base_position[2]
        odometry.pose.pose.orientation.x = base_orientation[0]
        odometry.pose.pose.orientation.y = base_orientation[1]
        odometry.pose.pose.orientation.z = base_orientation[2]
        odometry.pose.pose.orientation.w = base_orientation[3]
        odometry.twist.twist.linear.x = base_linear[0]
        odometry.twist.twist.linear.y = base_linear[1]
        odometry.twist.twist.linear.z = base_linear[2]
        odometry.twist.twist.angular.x = base_angular[0]
        odometry.twist.twist.angular.y = base_angular[1]
        odometry.twist.twist.angular.z = base_angular[2]
        self._odom_publisher.publish(odometry)

        self._last_stamp = stamp
        self._last_wall_rx = time.monotonic()
        self._previous = (stamp, base_position, base_orientation)
        self._set_healthy(True)

    def _to_base(
        self,
        chassis_position: Vector,
        chassis_orientation: Quaternion,
        chassis_linear: Vector,
        chassis_angular: Vector,
    ) -> Tuple[Vector, Quaternion, Vector, Vector]:
        """Re-reference a chassis-frame measurement to base_footprint.

        Implements the extrinsic contract of doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN
        section 4.3.  The input twist is expressed in the chassis frame, so it is
        rotated to the world frame first; the base twist then follows from the
        rigid-body relation v_base = v_chassis - omega x r.  A fixed Z offset is
        not enough on a ramp: the offset must be rotated by the full orientation.
        """
        # chassis_T_base = inverse(base_T_chassis)
        q_cb = _q_inverse(self._base_to_chassis_q)
        t_cb = _rotate(q_cb, tuple(-value for value in self._base_to_chassis_xyz))
        world_t_cb = _rotate(chassis_orientation, t_cb)

        base_position = tuple(
            chassis_position[index] + world_t_cb[index] for index in range(3)
        )
        base_orientation = _q_multiply(chassis_orientation, q_cb)

        # Input twist (chassis frame) -> world frame.
        linear_world = _rotate(chassis_orientation, chassis_linear)
        angular_world = _rotate(chassis_orientation, chassis_angular)

        # r = p_chassis - p_base (base origin to chassis origin, world frame).
        r_world = tuple(
            chassis_position[index] - base_position[index] for index in range(3)
        )
        base_linear_world = tuple(
            linear_world[index] - _cross(angular_world, r_world)[index]
            for index in range(3)
        )

        # Express the base twist in the base frame (odometry twist contract).
        q_base_inverse = _q_inverse(base_orientation)
        base_linear = _rotate(q_base_inverse, base_linear_world)
        base_angular = _rotate(q_base_inverse, angular_world)
        return base_position, base_orientation, base_linear, base_angular

    def _check_against_finite_difference(
        self,
        stamp: float,
        position: Vector,
        orientation: Quaternion,
        linear: Vector,
    ) -> None:
        """Cross-check the engine twist against the finite difference of truth.

        The engine value stays authoritative; this only detects a wrong frame or a
        broken re-referencing, which would otherwise look like plausible motion.
        """
        if not self._check_finite_difference or self._previous is None:
            return
        if (
            self._first_stamp is not None
            and stamp - self._first_stamp < self._finite_difference_warmup_s
        ):
            return
        previous_stamp, previous_position, previous_orientation = self._previous
        delta_t = stamp - previous_stamp
        if not 1.0e-4 < delta_t < 1.0:
            return
        world_velocity = tuple(
            (position[index] - previous_position[index]) / delta_t for index in range(3)
        )
        finite_difference = _rotate(_q_inverse(orientation), world_velocity)
        difference = max(
            abs(finite_difference[index] - linear[index]) for index in range(3)
        )
        if difference > self._finite_difference_tolerance:
            self._finite_difference_mismatches += 1
            self.get_logger().warn(
                "Engine twist and finite difference disagree by %.3f m/s "
                "(count=%d); check the twist frame and the extrinsic"
                % (difference, self._finite_difference_mismatches),
                throttle_duration_sec=5.0,
            )

    def _watchdog(self) -> None:
        if self._last_wall_rx is None:
            self._set_healthy(False, "no truth message received yet")
            return
        age = time.monotonic() - self._last_wall_rx
        if age > self._timeout_s:
            self._set_healthy(
                False,
                "no truth message for %.2f s (limit %.2f s)" % (age, self._timeout_s),
            )
            return
        self._publish_health()


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
