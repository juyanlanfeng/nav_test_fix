"""Regression tests for the PB velocity adapter safety logic.

Covers the four issues raised in the acceptance review: the watchdog must run on
a steady (wall-clock) timer, velocity messages must be timestamp-checked, and the
existing frame/finiteness/source checks must keep working.
"""

import time

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.clock import ClockType
from rclpy.parameter import Parameter
import rclpy

from pb_vehicle_adapter.cmd_vel_adapter import CmdVelAdapter, SOURCES, _finite_twist


def _stamped(sec: int, nanosec: int = 0, frame_id: str = "base_footprint",
             linear_x: float = 1.0) -> TwistStamped:
    message = TwistStamped()
    message.header.frame_id = frame_id
    message.header.stamp = TimeMsg(sec=sec, nanosec=nanosec)
    message.twist.linear.x = linear_x
    return message


def setup_module(module):
    rclpy.init()


def teardown_module(module):
    if rclpy.ok():
        rclpy.shutdown()


def _adapter():
    return CmdVelAdapter()


def test_finite_twist():
    good = Twist()
    good.linear.x = 1.0
    assert _finite_twist(good)
    bad = Twist()
    bad.angular.z = float("nan")
    assert not _finite_twist(bad)


def test_watchdog_uses_steady_clock():
    node = _adapter()
    try:
        assert node._timer.clock.clock_type == ClockType.STEADY_TIME
    finally:
        node.destroy_node()


def test_accepts_fresh_stamped_command():
    node = _adapter()
    try:
        now = node.get_clock().now().to_msg()
        node._accept(_stamped(now.sec, now.nanosec), "meshnav")
        assert node._last_command is not None
        assert node._last_receive_time is not None
    finally:
        node.destroy_node()


def test_rejects_stale_timestamp():
    node = _adapter()
    try:
        now = node.get_clock().now().to_msg()
        node._accept(_stamped(now.sec - 5, now.nanosec), "meshnav")
        assert node._last_command is None
    finally:
        node.destroy_node()


def test_rejects_unstamped_command():
    node = _adapter()
    try:
        node._accept(_stamped(0, 0), "meshnav")
        assert node._last_command is None
    finally:
        node.destroy_node()


def test_rejects_wrong_frame():
    node = _adapter()
    try:
        now = node.get_clock().now().to_msg()
        node._accept(_stamped(now.sec, now.nanosec, frame_id="map"), "meshnav")
        assert node._last_command is None
    finally:
        node.destroy_node()


def test_timeout_clears_command():
    node = _adapter()
    try:
        now = node.get_clock().now().to_msg()
        node._accept(_stamped(now.sec, now.nanosec), "meshnav")
        assert node._last_command is not None
        # Simulate a receive that happened longer ago than the timeout, then run
        # one watchdog cycle: the latched command must be cleared.
        node._last_receive_time = time.monotonic() - (node._timeout_s + 0.1)
        node._publish_safe_command()
        assert node._last_command is None
    finally:
        node.destroy_node()


def test_third_source_dddmr_is_defined():
    assert "dddmr" in SOURCES


def test_dddmr_selected_and_other_sources_ignored():
    node = _adapter()
    try:
        node.set_parameters([Parameter("control_source", value="dddmr")])
        assert node._source == "dddmr"
        now = node.get_clock().now().to_msg()
        # An unselected source must not latch a command.
        node._meshnav_callback(_stamped(now.sec, now.nanosec))
        assert node._last_command is None
        node._jie_callback(_stamped(now.sec, now.nanosec))
        assert node._last_command is None
        # The selected source does.
        node._dddmr_callback(_stamped(now.sec, now.nanosec, linear_x=0.3))
        assert node._last_command is not None
        assert abs(node._last_command.linear.x - 0.3) < 1.0e-9
    finally:
        node.destroy_node()


def test_source_switch_clears_latched_command():
    node = _adapter()
    try:
        now = node.get_clock().now().to_msg()
        node._meshnav_callback(_stamped(now.sec, now.nanosec))
        assert node._last_command is not None
        node.set_parameters([Parameter("control_source", value="dddmr")])
        assert node._last_command is None
    finally:
        node.destroy_node()


def test_invalid_source_rejected():
    node = _adapter()
    try:
        result = node.set_parameters([Parameter("control_source", value="bogus")])
        assert not result[0].successful
        assert node._source == "meshnav"
    finally:
        node.destroy_node()


def test_steady_timer_fires_with_frozen_sim_time():
    """The watchdog must tick even when /clock is paused (use_sim_time=true)."""
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node

    node = Node("steady_timer_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        ticks = []
        node.create_timer(0.05, lambda: ticks.append(1),
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert ticks, "steady-clock timer did not fire while sim time was frozen"
    finally:
        node.destroy_node()
