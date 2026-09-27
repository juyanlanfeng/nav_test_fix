"""Select one navigation velocity source and continuously enforce a safe stop."""

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray, GoalStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from pb_vehicle_adapter.hold_controller import (
    offset_to_reference,
    should_hold,
    station_keeping_command,
)
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
        # Post-action station keeping (P3).  Default OFF in code so the real-robot
        # path is unchanged unless a configuration turns it on explicitly; the PB
        # simulation config sets it on and documents why.
        self.declare_parameter("hold_enabled", False)
        self.declare_parameter("hold_pose_topic", "/odom")
        self.declare_parameter("hold_gain", 1.0)
        self.declare_parameter("hold_max_linear_velocity", 0.1)
        self.declare_parameter("hold_max_angular_velocity", 0.2)
        self.declare_parameter("hold_angular_deadband", 0.02)
        self.declare_parameter("hold_pose_timeout_s", 0.3)

        self._source = self.get_parameter("control_source").value
        self._base_frames = set(self.get_parameter("base_frames").value)
        self._timeout_s = float(self.get_parameter("timeout_s").value)
        self._max_stamp_age_s = float(self.get_parameter("max_stamp_age_s").value)
        self._hold_enabled = bool(self.get_parameter("hold_enabled").value)
        self._hold_gain = float(self.get_parameter("hold_gain").value)
        self._hold_max_linear = float(self.get_parameter("hold_max_linear_velocity").value)
        self._hold_max_angular = float(self.get_parameter("hold_max_angular_velocity").value)
        self._hold_deadband = float(self.get_parameter("hold_angular_deadband").value)
        self._pose_timeout = float(self.get_parameter("hold_pose_timeout_s").value)
        rate = float(self.get_parameter("publish_frequency_hz").value)
        if self._source not in SOURCES:
            raise ValueError("control_source must be one of %s" % (SOURCES,))
        if self._timeout_s <= 0.0 or self._max_stamp_age_s <= 0.0 or rate <= 0.0:
            raise ValueError("timeout_s, max_stamp_age_s and publish_frequency_hz must be positive")

        self._last_command: Optional[Twist] = None
        self._last_receive_time: Optional[float] = None
        # Hold state.  The reference is the pose at the most recent accepted navigation
        # command, not the pose when the timeout fires: between the two the vehicle can
        # slide for up to timeout_s, and holding the drifted pose would simply freeze
        # the error the hold exists to remove.
        self._hold_reference: Optional[tuple] = None
        self._last_commanded_pose: Optional[tuple] = None
        self._pose: Optional[tuple] = None
        self._pose_received = None
        self._pose_stamp_ns = None
        self._motion_inhibited = False
        self._action_states = {}
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
        if self._hold_enabled:
            # The hold needs the pose it is holding; it uses the same odometry the
            # navigation stack consumes, not a second source of truth.
            self.create_subscription(
                Odometry,
                self.get_parameter("hold_pose_topic").value,
                self._pose_callback,
                20,
            )
            self.create_subscription(
                GoalStatusArray, "/move_base_flex/exe_path/_action/status",
                self._action_status_callback, 10)
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
                self._motion_inhibited = False
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

    def _pose_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        position = message.pose.pose.position
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        values = (position.x, position.y, position.z, orientation.x, orientation.y,
                  orientation.z, orientation.w)
        norm = sum(v * v for v in values[3:])
        if (not all(math.isfinite(v) for v in values) or norm < 1e-12 or stamp_ns <= 0
                or abs(self.get_clock().now().nanoseconds - stamp_ns) * 1e-9 > self._pose_timeout):
            self._pose = None
            self._clear_command()
            return
        if self._pose_stamp_ns is not None and stamp_ns <= self._pose_stamp_ns:
            if stamp_ns < self._pose_stamp_ns:
                self._pose = None
                self._pose_stamp_ns = None
                self._clear_command()
            return
        self._pose_stamp_ns = stamp_ns
        self._pose_received = time.monotonic()
        yaw = math.atan2(2.0 * (orientation.w * orientation.z +
                                orientation.x * orientation.y),
                         1.0 - 2.0 * (orientation.y * orientation.y +
                                      orientation.z * orientation.z))
        self._pose = ((message.pose.pose.position.x, message.pose.pose.position.y), yaw)

    def _action_status_callback(self, message: GoalStatusArray) -> None:
        if self._source != "meshnav":
            return
        changed = []
        for item in message.status_list:
            key = bytes(item.goal_info.goal_id.uuid)
            if self._action_states.get(key) != item.status:
                changed.append(item.status)
            self._action_states[key] = item.status
        # MBF replaces the old path after accepting the new one. These can be
        # separate status messages: the new goal may already be EXECUTING when
        # the old goal changes to ABORTED. Inspect all current goals, otherwise
        # periodic replanning would inhibit a controller that is still active.
        if any(item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
               for item in message.status_list):
            self._motion_inhibited = False
        elif any(s in (GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_CANCELED,
                       GoalStatus.STATUS_ABORTED) for s in changed):
            self._motion_inhibited = True
            self._clear_command()

    def _accept(self, message: TwistStamped, source: str) -> None:
        if self._motion_inhibited:
            return
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
        # A new navigation command takes over: release any active hold and remember the
        # pose it was steering toward as the pose to hold if commands stop again.
        self._hold_reference = None
        if self._pose is not None:
            self._last_commanded_pose = self._pose

    def _clear_command(self) -> None:
        self._last_command = None
        self._last_receive_time = None
        self._hold_reference = None
        self._last_commanded_pose = None
        self._output.publish(Twist())

    def _publish_safe_command(self) -> None:
        if self._hold_enabled and (self._pose is None or self._pose_received is None
                                  or time.monotonic() - self._pose_received > self._pose_timeout):
            # A stale localization must never sustain a corrective velocity. A
            # fresh navigation command is required to re-arm after this fault.
            self._clear_command()
            return
        fresh = (self._last_command is not None and self._last_receive_time is not None
                 and time.monotonic() - self._last_receive_time <= self._timeout_s)
        if should_hold(fresh, self._hold_enabled,
                       self._pose is not None and self._last_commanded_pose is not None):
            self._publish_hold()
            return
        if self._last_command is None or self._last_receive_time is None:
            self._output.publish(Twist())
            return
        if not fresh:
            self._clear_command()
            return
        self._output.publish(self._last_command)

    def _publish_hold(self) -> None:
        """Hold the pose reached when the navigation command went away.

        Capturing the reference once per arming is what makes this a hold rather than a
        chase: the correction opposes the drift measured from that pose.
        """
        if self._hold_reference is None:
            # Prefer the pose the navigation stack was last steering toward; fall back to
            # the current pose only when no command was ever accepted.
            self._hold_reference = (self._last_commanded_pose
                                    if self._last_commanded_pose is not None else self._pose)
            self.get_logger().info(
                "Navigation command gone; holding station (post-action hold active)",
                throttle_duration_sec=5.0,
            )
        dx, dy, yaw_error = offset_to_reference(self._hold_reference, self._pose)
        linear_x, linear_y, angular_z = station_keeping_command(
            dx, dy, yaw_error, self._pose[1], self._hold_gain,
            self._hold_max_linear, self._hold_max_angular, self._hold_deadband,
        )
        command = Twist()
        command.linear.x = linear_x
        command.linear.y = linear_y
        command.angular.z = angular_z
        self._output.publish(command)

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
