"""Select one navigation velocity source and continuously enforce a safe stop."""

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time
import rclpy


# Supported navigation velocity sources.  Only one is forwarded at a time.
SOURCES = ("meshnav", "jie", "dddmr")


def _finite_twist(message: Twist) -> bool:
    values = (
        message.linear.x,
        message.linear.y,
        message.linear.z,
        message.angular.x,
        message.angular.y,
        message.angular.z,
    )
    return all(math.isfinite(value) for value in values)


class CmdVelAdapter(Node):
    """Forward only commands from the selected source and time out to zero."""

    def __init__(self) -> None:
        super().__init__("pb_cmd_vel_adapter")
        self.declare_parameter("control_source", "meshnav")
        self.declare_parameter("meshnav_topic", "/cmd_vel")
        self.declare_parameter("jie_topic", "/pb/jie_cmd_vel_stamped")
        self.declare_parameter("dddmr_topic", "/pb/dddmr_cmd_vel_stamped")
        self.declare_parameter("output_topic", "/pb/cmd_vel_safe")
        self.declare_parameter("base_frames", ["base_footprint", "base_link", "chassis"])
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("max_stamp_age_s", 0.5)
        self.declare_parameter("publish_frequency_hz", 30.0)

        self._source = self.get_parameter("control_source").value
        self._base_frames = set(self.get_parameter("base_frames").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)
        self._max_stamp_age_s = float(self.get_parameter("max_stamp_age_s").value)
        rate = float(self.get_parameter("publish_frequency_hz").value)
        if self._source not in SOURCES:
            raise ValueError("control_source must be one of %s" % (SOURCES,))
        if self._timeout_s <= 0.0 or self._max_stamp_age_s <= 0.0 or rate <= 0.0:
            raise ValueError("timeout_s, max_stamp_age_s and publish_frequency_hz must be positive")

        self._last_command: Optional[Twist] = None
        self._last_receive_time: Optional[float] = None
        self._output = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("meshnav_topic").value,
            self._meshnav_callback,
            20,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("jie_topic").value,
            self._jie_callback,
            20,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("dddmr_topic").value,
            self._dddmr_callback,
            20,
        )
        # The watchdog must keep running while /clock is paused, so it uses a
        # steady (wall-clock) timer instead of the node's ROS-time clock.
        self._timer = self.create_timer(
            1.0 / rate,
            self._publish_safe_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.add_on_set_parameters_callback(self._parameters_callback)
        self.get_logger().info(
            "PB velocity adapter is active; selected source is '%s' and timeout is %.3f s"
            % (self._source, self._timeout_s)
        )

    def _parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name != "control_source":
                continue
            if parameter.value not in SOURCES:
                return SetParametersResult(successful=False, reason="source must be one of %s" % (SOURCES,))
            if parameter.value != self._source:
                self._source = parameter.value
                self._clear_command()
                self.get_logger().info("PB velocity source switched to '%s'; sent stop" % self._source)
        return SetParametersResult(successful=True)

    def _meshnav_callback(self, message: TwistStamped) -> None:
        if self._source != "meshnav":
            return
        self._accept(message, "meshnav")

    def _jie_callback(self, message: TwistStamped) -> None:
        if self._source != "jie":
            return
        self._accept(message, "jie")

    def _dddmr_callback(self, message: TwistStamped) -> None:
        if self._source != "dddmr":
            return
        self._accept(message, "dddmr")

    def _accept(self, message: TwistStamped, source: str) -> None:
        frame_id = message.header.frame_id.strip().lstrip("/")
        if frame_id and frame_id not in self._base_frames:
            self.get_logger().warn(
                "Dropped %s command expressed in '%s'; expected a chassis frame" % (source, frame_id),
                throttle_duration_sec=2.0,
            )
            return
        if not _finite_twist(message.twist):
            self.get_logger().warn("Dropped non-finite %s velocity command" % source)
            return
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            self.get_logger().warn(
                "Dropped unstamped %s velocity command" % source,
                throttle_duration_sec=2.0,
            )
            return
        # Reject stale or future-dated commands so a paused or delayed source
        # cannot latch an outdated velocity.
        age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1.0e-9
        if abs(age) > self._max_stamp_age_s:
            self.get_logger().warn(
                "Dropped %s command stamped %.3f s from the current time" % (source, age),
                throttle_duration_sec=2.0,
            )
            return
        self._last_command = message.twist
        self._last_receive_time = time.monotonic()

    def _clear_command(self) -> None:
        self._last_command = None
        self._last_receive_time = None
        self._output.publish(Twist())

    def _publish_safe_command(self) -> None:
        if self._last_command is None or self._last_receive_time is None:
            self._output.publish(Twist())
            return
        if time.monotonic() - self._last_receive_time > self._timeout_s:
            self._clear_command()
            return
        self._output.publish(self._last_command)

    def destroy_node(self):
        # A normal shutdown, cancellation, or source switch cannot leave the
        # MecanumDrive2 target velocity latched at its last nonzero value.
        if rclpy.ok():
            self._output.publish(Twist())
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
