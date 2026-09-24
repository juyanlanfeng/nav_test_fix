"""Tests for the post-action station-keeping hold (P3).

The hold exists because zero command is not a brake on this vehicle: after the ExePath
action ends, MBF publishes zero through `force_stop_at_goal` and the chassis kept
sliding (measured 0.2649 m over 103 s, with no commands issued at all).  These tests
pin both the pure command shape and the adapter's decision to hold rather than zero.
"""

import math
import time

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as TimeMsg
import rclpy

from pb_vehicle_adapter.cmd_vel_adapter import CmdVelAdapter
from pb_vehicle_adapter.hold_controller import (
    offset_to_reference,
    should_hold,
    station_keeping_command,
    wrap_angle,
)


def setup_module(module):
    rclpy.init()


def teardown_module(module):
    if rclpy.ok():
        rclpy.shutdown()


def _adapter(**overrides):
    node = CmdVelAdapter()
    if overrides:
        node.set_parameters([
            rclpy.parameter.Parameter(name, value=value)
            for name, value in overrides.items()
        ])
        # Re-read every cached scalar the way a reconfigure would.  _timeout_s in
        # particular is read once in __init__, so a test that shortens it must refresh
        # it here or the command never goes stale and no hold is ever entered.
        node._timeout_s = float(node.get_parameter("timeout_s").value)
        node._hold_enabled = bool(node.get_parameter("hold_enabled").value)
        node._hold_gain = float(node.get_parameter("hold_gain").value)
        node._hold_max_linear = float(node.get_parameter("hold_max_linear_velocity").value)
        node._hold_max_angular = float(node.get_parameter("hold_max_angular_velocity").value)
        node._hold_deadband = float(node.get_parameter("hold_angular_deadband").value)
    return node


def _stamped(sec, nanosec=0, frame_id="base_footprint", linear_x=1.0):
    message = TwistStamped()
    message.header.frame_id = frame_id
    message.header.stamp = TimeMsg(sec=sec, nanosec=nanosec)
    message.twist.linear.x = linear_x
    return message


def _fresh(node, linear_x=1.0):
    """A command stamped now, with nanosecond precision (a sec-only stamp looks stale)."""
    now = node.get_clock().now().to_msg()
    return _stamped(now.sec, now.nanosec, linear_x=linear_x)


def _odom(x, y, yaw):
    message = Odometry()
    ns = time.time_ns()
    message.header.stamp = TimeMsg(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    return message


class _Recorder:
    """Capture what the adapter publishes on its output topic."""

    def __init__(self, node):
        self.messages = []
        node._output = type("P", (), {"publish": self.messages.append})()


# ---------------------------------------------------------------- pure logic

def test_wrap_angle_handles_the_crossing():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(2.0 * math.pi - 0.02) == pytest.approx(-0.02, abs=1e-9)


def test_hold_command_opposes_the_offset_and_stays_bounded():
    linear_x, linear_y, angular_z = station_keeping_command(
        dx=0.05, dy=-0.02, yaw_error=0.05, yaw=0.0,
        gain=1.0, max_linear=0.1, max_angular=0.2, angular_deadband=0.02)
    assert linear_x == pytest.approx(0.05)
    assert linear_y == pytest.approx(-0.02)
    assert angular_z == pytest.approx(0.05)
    # A metre-scale offset must not become a lunge.  The cap is per component, exactly
    # as in pb_terminal_controller's station_keeping_command, so the two holds behave
    # identically; a diagonal command is therefore bounded by max_linear per axis.
    linear_x, linear_y, _ = station_keeping_command(
        dx=5.0, dy=5.0, yaw_error=0.0, yaw=0.0,
        gain=1.0, max_linear=0.1, max_angular=0.2, angular_deadband=0.02)
    assert abs(linear_x) <= 0.1 + 1e-12
    assert abs(linear_y) <= 0.1 + 1e-12


def test_hold_command_is_expressed_in_the_body_frame():
    # Facing +90 degrees, an offset along world +X is lateral in the body frame.
    linear_x, linear_y, _ = station_keeping_command(
        dx=0.1, dy=0.0, yaw_error=0.0, yaw=math.pi / 2.0,
        gain=1.0, max_linear=1.0, max_angular=1.0, angular_deadband=0.0)
    assert linear_x == pytest.approx(0.0, abs=1e-9)
    assert linear_y == pytest.approx(-0.1)


def test_hold_command_suppresses_buzzing_inside_the_deadband():
    _, _, angular_z = station_keeping_command(
        dx=0.0, dy=0.0, yaw_error=0.01, yaw=0.0,
        gain=1.0, max_linear=0.1, max_angular=0.2, angular_deadband=0.02)
    assert angular_z == 0.0


def test_offset_to_reference_reports_position_and_wrapped_yaw():
    dx, dy, yaw_error = offset_to_reference(((1.0, 2.0), 0.05), ((0.9, 2.4), -0.02))
    assert (dx, dy) == pytest.approx((0.1, -0.4))
    assert yaw_error == pytest.approx(0.07)


def test_should_hold_requires_enabled_pose_and_no_fresh_command():
    assert should_hold(command_is_fresh=False, enabled=True, pose_known=True) is True
    assert should_hold(command_is_fresh=True, enabled=True, pose_known=True) is False
    assert should_hold(command_is_fresh=False, enabled=False, pose_known=True) is False
    assert should_hold(command_is_fresh=False, enabled=True, pose_known=False) is False


# ------------------------------------------------------- adapter behaviour

def test_adapter_holds_instead_of_zeroing_when_the_command_times_out():
    node = _adapter(hold_enabled=True, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        # A fresh command is forwarded unchanged and never held.
        node._accept(_fresh(node), "meshnav")
        node._publish_safe_command()
        assert recorder.messages[-1].linear.x == 1.0
        assert node._hold_reference is None

        # Let it go stale: the output must be a corrective command, not zero.
        time.sleep(0.08)
        node._pose_callback(_odom(0.98, 2.0, 0.0))   # drifted 2 cm back
        node._publish_safe_command()
        held = recorder.messages[-1]
        assert held.linear.x != 0.0
        assert held.linear.x > 0.0                   # pushing back toward the reference
        assert node._hold_reference is not None
    finally:
        node.destroy_node()


def test_adapter_releases_the_hold_when_a_new_command_arrives():
    node = _adapter(hold_enabled=True, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        node._accept(_fresh(node), "meshnav")
        time.sleep(0.08)
        node._publish_safe_command()
        assert node._hold_reference is not None

        node._accept(_fresh(node), "meshnav")
        assert node._hold_reference is None
        node._publish_safe_command()
        assert recorder.messages[-1].linear.x == 1.0
    finally:
        node.destroy_node()


def test_adapter_holds_a_stationary_robot_without_moving_it():
    node = _adapter(hold_enabled=True, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        time.sleep(0.08)
        node._publish_safe_command()
        held = recorder.messages[-1]
        # No offset and no yaw error: the hold must command no motion at all.
        assert held.linear.x == 0.0
        assert held.linear.y == 0.0
        assert held.angular.z == 0.0
    finally:
        node.destroy_node()


def test_adapter_publishes_zero_when_the_hold_is_disabled():
    node = _adapter(hold_enabled=False, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        node._accept(_fresh(node), "meshnav")
        time.sleep(0.08)
        node._publish_safe_command()
        zeroed = recorder.messages[-1]
        assert (zeroed.linear.x, zeroed.linear.y, zeroed.angular.z) == (0.0, 0.0, 0.0)
        assert node._hold_reference is None
    finally:
        node.destroy_node()


def test_adapter_without_a_pose_falls_back_to_zero():
    # The hold must never guess: with no odometry there is nothing to hold.
    node = _adapter(hold_enabled=True, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._accept(_fresh(node), "meshnav")
        time.sleep(0.08)
        node._publish_safe_command()
        fallback = recorder.messages[-1]
        assert (fallback.linear.x, fallback.linear.y, fallback.angular.z) == (0.0, 0.0, 0.0)
    finally:
        node.destroy_node()


def test_stale_pose_clears_hold_and_does_not_rearm_on_pose_alone():
    node = _adapter(hold_enabled=True, timeout_s=0.05)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        node._accept(_fresh(node), "meshnav")
        node._pose_received = time.monotonic() - 1.0
        node._publish_safe_command()
        assert recorder.messages[-1] == Twist()
        assert node._last_commanded_pose is None
        node._pose_callback(_odom(0.5, 2.0, 0.0))
        node._publish_safe_command()
        assert recorder.messages[-1] == Twist()
    finally:
        node.destroy_node()


def test_cancellation_inhibits_hold_until_a_new_goal():
    from action_msgs.msg import GoalStatusArray, GoalStatus
    node = _adapter(hold_enabled=True)
    recorder = _Recorder(node)
    try:
        node._pose_callback(_odom(1.0, 2.0, 0.0))
        node._accept(_fresh(node), "meshnav")
        status = GoalStatus(status=GoalStatus.STATUS_CANCELING)
        status.goal_info.goal_id.uuid[0] = 1
        node._action_status_callback(GoalStatusArray(status_list=[status]))
        node._accept(_fresh(node), "meshnav")
        node._publish_safe_command()
        assert recorder.messages[-1] == Twist()
        status.goal_info.goal_id.uuid[0] = 2
        status.status = GoalStatus.STATUS_EXECUTING
        node._action_status_callback(GoalStatusArray(status_list=[status]))
        node._accept(_fresh(node), "meshnav")
        node._publish_safe_command()
        assert recorder.messages[-1].linear.x == 1.0
    finally:
        node.destroy_node()


import pytest  # noqa: E402  (kept last so the module-level helpers read first)
