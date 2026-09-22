"""Regression tests for the PB ground-truth adapter.

doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md section 4 replaced the old
two-asynchronous-stream composition with a single same-step source
(``/pb_sim/chassis_truth``).  The tests cover the contract that replaced it:

* re-referencing the measurement from the chassis link to ``base_footprint``,
  including an extrinsic offset rotated by the full chassis orientation (a fixed Z
  subtraction is wrong on a ramp) and the ``omega x r`` term of the twist;
* input validation and time bookkeeping: zero stamp, non-finite data, duplicate
  stamp, backwards stamp (simulation reset), stale input via the watchdog;
* the finite-difference cross-check that detects a wrong twist frame.

The two-robot entity isolation of the Gazebo system cannot be covered here; it is
scoped by construction (the system resolves the link inside the model it is
attached to) and is verified live.
"""

import math

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
import rclpy

from pb_vehicle_adapter.ground_truth_adapter import GroundTruthAdapter


def setup_module(module):
    rclpy.init()


def teardown_module(module):
    if rclpy.ok():
        rclpy.shutdown()


def _odometry(sec, nanosec=0, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0),
              linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
    message = Odometry()
    message.header.stamp = TimeMsg(sec=sec, nanosec=nanosec)
    message.header.frame_id = "rmuc2026_field"
    message.child_frame_id = "chassis"
    message.pose = PoseWithCovariance(pose=Pose())
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
     message.pose.pose.orientation.z, message.pose.pose.orientation.w) = orientation
    message.twist = TwistWithCovariance(twist=Twist())
    message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z = linear
    message.twist.twist.angular.x, message.twist.twist.angular.y, message.twist.twist.angular.z = angular
    return message


def _adapter():
    adapter = GroundTruthAdapter()
    adapter.published = []
    adapter.transforms = []
    adapter.health = []
    adapter._odom_publisher.publish = adapter.published.append
    adapter._tf_broadcaster.sendTransform = adapter.transforms.append
    adapter._health_publisher.publish = adapter.health.append
    return adapter


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_translation_only_extrinsic_on_flat_ground():
    adapter = _adapter()
    position, orientation, linear, angular = adapter._to_base(
        (1.0, 2.0, 0.076), (0.0, 0.0, 0.0, 1.0), (0.3, 0.0, 0.0), (0.0, 0.0, 0.1)
    )
    assert position == (1.0, 2.0, 0.0)
    assert orientation == (0.0, 0.0, 0.0, 1.0)
    assert abs(linear[0] - 0.3) < 1.0e-12 and abs(linear[1]) < 1.0e-12
    assert abs(angular[2] - 0.1) < 1.0e-12
    adapter.destroy_node()


def test_extrinsic_offset_is_rotated_by_full_chassis_orientation():
    # 90 deg roll about X: the 0.076 m base->chassis offset (chassis -Z) must rotate
    # into world +Y, so the base lands at chassis + (0, 0.076, 0).  A fixed
    # "subtract 0.076 from z" implementation would give (0, 0, 0) instead.
    adapter = _adapter()
    half = math.sin(math.pi / 4.0)
    position, orientation, _, _ = adapter._to_base(
        (0.0, 0.0, 0.076), (half, 0.0, 0.0, half), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    )
    assert abs(position[0]) < 1.0e-12
    assert abs(position[1] - 0.076) < 1.0e-12
    assert abs(position[2] - 0.076) < 1.0e-12
    adapter.destroy_node()


def test_in_place_rotation_produces_omega_cross_r_term():
    # omega = (1,0,0), r = (0,0,0.076) -> v_base = -(omega x r) = (0, 0.076, 0)
    adapter = _adapter()
    _, _, linear, angular = adapter._to_base(
        (0.0, 0.0, 0.076), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    )
    assert abs(linear[0]) < 1.0e-12
    assert abs(linear[1] - 0.076) < 1.0e-12
    assert abs(linear[2]) < 1.0e-12
    assert abs(angular[0] - 1.0) < 1.0e-12
    adapter.destroy_node()


def test_body_twist_is_rotated_into_the_base_frame():
    # Chassis at 90 deg yaw moving along its own +X: the body twist stays +X even
    # though the world velocity is +Y.
    adapter = _adapter()
    yaw = math.pi / 2.0
    position, _, linear, _ = adapter._to_base(
        (3.0, 4.0, 0.076), _yaw_quaternion(yaw), (0.4, 0.0, 0.0), (0.0, 0.0, 0.0)
    )
    assert abs(position[0] - 3.0) < 1.0e-9 and abs(position[1] - 4.0) < 1.0e-9
    assert abs(linear[0] - 0.4) < 1.0e-9
    assert abs(linear[1]) < 1.0e-9
    adapter.destroy_node()


def test_zero_stamp_is_rejected_and_reported_unhealthy():
    adapter = _adapter()
    adapter._truth_callback(_odometry(0, 0))
    assert adapter.published == []
    assert adapter.transforms == []
    assert adapter._rejected_samples == 1
    assert adapter._healthy is False
    adapter.destroy_node()


def test_non_finite_input_is_rejected():
    adapter = _adapter()
    adapter._truth_callback(_odometry(5, 0, position=(float("nan"), 0.0, 0.0)))
    assert adapter.published == []
    assert adapter._rejected_samples == 1
    adapter.destroy_node()


def test_duplicate_stamp_is_processed_once():
    adapter = _adapter()
    adapter._truth_callback(_odometry(7, 0))
    adapter._truth_callback(_odometry(7, 0))
    assert len(adapter.published) == 1
    assert adapter._duplicate_samples == 1
    adapter.destroy_node()


def test_backwards_stamp_is_treated_as_simulation_reset():
    adapter = _adapter()
    adapter._truth_callback(_odometry(10, 0, position=(1.0, 0.0, 0.076)))
    adapter._truth_callback(_odometry(2, 0, position=(0.0, 0.0, 0.076)))
    assert len(adapter.published) == 2
    assert adapter._time_resets == 1
    # The reset cleared the differencing basis, so the second sample starts fresh.
    assert adapter._previous[0] == 2.0
    adapter.destroy_node()


def test_watchdog_reports_stale_input():
    adapter = _adapter()
    adapter._truth_callback(_odometry(3, 0))
    assert adapter._healthy is True
    adapter._last_wall_rx -= adapter._timeout_s + 1.0
    adapter._watchdog()
    assert adapter._healthy is False
    adapter.destroy_node()


def test_finite_difference_cross_check_flags_a_wrong_twist_frame():
    adapter = _adapter()
    adapter._finite_difference_warmup_s = 0.0
    # 20 Hz samples moving +0.02 m per step in X, but the reported body twist is
    # zero: the cross-check must notice.
    adapter._truth_callback(_odometry(1, 0, position=(0.0, 0.0, 0.076), linear=(0.4, 0.0, 0.0)))
    adapter._truth_callback(
        _odometry(1, 50_000_000, position=(0.02, 0.0, 0.076), linear=(0.0, 0.0, 0.0))
    )
    assert adapter._finite_difference_mismatches == 1
    adapter.destroy_node()


def test_finite_difference_cross_check_accepts_consistent_twist():
    adapter = _adapter()
    adapter._finite_difference_warmup_s = 0.0
    adapter._truth_callback(_odometry(1, 0, position=(0.0, 0.0, 0.076), linear=(0.4, 0.0, 0.0)))
    adapter._truth_callback(
        _odometry(1, 50_000_000, position=(0.02, 0.0, 0.076), linear=(0.4, 0.0, 0.0))
    )
    assert adapter._finite_difference_mismatches == 0
    adapter.destroy_node()


def test_spawn_transient_is_excluded_by_the_cross_check_warmup():
    adapter = _adapter()
    adapter._truth_callback(_odometry(1, 0, position=(0.0, 0.0, 0.076), linear=(0.4, 0.0, 0.0)))
    adapter._truth_callback(
        _odometry(1, 50_000_000, position=(0.02, 0.0, 0.076), linear=(0.0, 0.0, 0.0))
    )
    assert adapter._finite_difference_mismatches == 0
    adapter.destroy_node()


def test_health_is_a_heartbeat_not_a_one_shot_event():
    adapter = _adapter()
    adapter._truth_callback(_odometry(4, 0))
    adapter._truth_callback(_odometry(4, 10_000_000))
    adapter._watchdog()
    # One transition plus one heartbeat, so a late subscriber still learns the state.
    assert [message.data for message in adapter.health] == [True, True]
    adapter.destroy_node()
