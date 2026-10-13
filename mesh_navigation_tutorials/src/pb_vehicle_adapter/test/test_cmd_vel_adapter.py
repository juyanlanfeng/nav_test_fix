"""Verify command conversion and stop behavior without sending to hardware."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import Twist, TwistStamped
import pytest

from pb_vehicle_adapter.cmd_vel_adapter import CmdVelAdapter

MODULE = 'pb_vehicle_adapter.cmd_vel_adapter'


def adapter():
    node = CmdVelAdapter.__new__(CmdVelAdapter)
    node.cfg = dict(timeout_s=0.5, max_stamp_age_s=0.5, future_tolerance_s=0.05)
    node._base_frame = 'base_footprint'
    node._last_command = None
    node._last_received = None
    node._last_stamp_ns = None
    node._output = Mock()
    node.get_logger = Mock()
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value = SimpleNamespace(nanoseconds=10_000_000_000)
    return node


def command():
    msg = TwistStamped()
    msg.header.frame_id = 'base_footprint'
    msg.header.stamp.sec = 10
    msg.twist.linear.x = 0.1
    msg.twist.linear.y = -0.2
    msg.twist.linear.z = 0.03
    msg.twist.angular.x = 0.04
    msg.twist.angular.y = -0.05
    msg.twist.angular.z = 0.3
    return msg


def accept(node, message=None):
    with patch(MODULE + '.time.monotonic', return_value=100.0):
        node._accept(message or command())


def tick(node, monotonic=100.1, ros_ns=10_100_000_000):
    node.get_clock.return_value.now.return_value.nanoseconds = ros_ns
    with patch(MODULE + '.time.monotonic', return_value=monotonic):
        node._publish_command()


def test_conversion_preserves_all_six_components():
    node = adapter()
    msg = command()
    accept(node, msg)
    tick(node)
    assert node._output.publish.call_args.args[0] == msg.twist


@pytest.mark.parametrize('invalid', ['frame', 'empty_frame', 'nan', 'inf', 'zero_stamp',
                                     'old_stamp', 'future_stamp', 'duplicate_stamp'])
def test_invalid_command_immediately_clears_previous_motion(invalid):
    node = adapter()
    accept(node)
    msg = command()
    if invalid == 'frame':
        msg.header.frame_id = 'map'
    elif invalid == 'empty_frame':
        msg.header.frame_id = ''
    elif invalid == 'nan':
        msg.twist.linear.x = float('nan')
    elif invalid == 'inf':
        msg.twist.angular.z = float('inf')
    elif invalid == 'zero_stamp':
        msg.header.stamp.sec = 0
    elif invalid == 'old_stamp':
        msg.header.stamp.sec = 9
    elif invalid == 'future_stamp':
        msg.header.stamp.sec = 11
    accept(node, msg)
    assert node._output.publish.call_args.args[0] == Twist()
    assert node._last_command is None
    tick(node)
    assert node._output.publish.call_args.args[0] == Twist()


@pytest.mark.parametrize('monotonic,ros_ns', [
    (100.6, 10_100_000_000),  # Reception deadline, even with a slow system clock.
    (100.1, 10_600_000_000),  # Original timestamp expiration, even after recent receipt.
    (100.1, 9_000_000_000),   # Backward clock jump invalidates cached velocity.
])
def test_timeout_and_clock_changes_stop(monotonic, ros_ns):
    node = adapter()
    accept(node)
    tick(node, monotonic, ros_ns)
    assert node._last_command is None
    assert node._output.publish.call_args.args[0] == Twist()


def test_startup_and_timeout_keep_zero_until_a_new_valid_command():
    node = adapter()
    tick(node)
    assert node._output.publish.call_args.args[0] == Twist()
    accept(node)
    tick(node, 100.6, 10_600_000_000)
    tick(node, 100.7, 10_700_000_000)
    assert node._output.publish.call_args.args[0] == Twist()
    node.get_clock.return_value.now.return_value.nanoseconds = 11_000_000_000
    msg = command()
    msg.header.stamp.sec = 11
    with patch(MODULE + '.time.monotonic', return_value=101.0):
        node._accept(msg)
    tick(node, 101.1, 11_100_000_000)
    assert node._output.publish.call_args.args[0] == msg.twist


def test_republishing_does_not_refresh_original_timestamp():
    node = adapter()
    msg = command()
    msg.header.stamp.sec = 9
    msg.header.stamp.nanosec = 600_000_000
    accept(node, msg)  # Already 0.4 seconds old on arrival.
    tick(node, 100.05, 10_050_000_000)
    assert node._output.publish.call_args.args[0] == msg.twist
    tick(node, 100.15, 10_150_000_000)
    assert node._output.publish.call_args.args[0] == Twist()
