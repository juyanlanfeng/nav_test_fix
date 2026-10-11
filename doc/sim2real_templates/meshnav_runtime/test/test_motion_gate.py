"""Drive the real pure gate through faults; no ROS process or motor is needed."""

from dataclasses import replace
import math

import pytest

from meshnav_runtime.motion_gate import Limits, MotionGate, STAMPED_INPUTS, ZERO


def healthy(gate, wall=1.0, ros=100.0):
    for name in STAMPED_INPUTS:
        gate.observe_data(name, ros, wall, ros)
    gate.observe_health("hardware_ready", True, wall)
    gate.observe_health("estop", False, wall)
    gate.observe_health("localization", True, wall)
    gate.observe_tf(ros, wall, ros)


def armed(limits=None):
    gate = MotionGate(limits)
    healthy(gate)
    assert gate.set_enabled(True, 1.0, 100.0)[0]
    gate.observe_command((0.2, 0.0, 0.3), "base_footprint", 100.01, 1.01, 100.01)
    return gate


def test_startup_missing_inputs_prevents_enable_and_always_outputs_zero():
    gate = MotionGate()
    assert not gate.set_enabled(True, 1.0, 100.0)[0]
    assert gate.tick(1.0, 100.0) == ZERO
    assert gate.status(1.0, 100.0)["state"] == "fault"
    assert not gate.enabled


def test_healthy_startup_stays_disabled_until_explicit_enable():
    gate = MotionGate()
    healthy(gate)
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.001, 1.001, 100.001)
    assert gate.tick(1.02, 100.02) == ZERO
    assert not gate.enabled
    assert gate.set_enabled(True, 1.02, 100.02)[0]
    assert gate.tick(1.03, 100.03) == ZERO
    assert gate.status(1.03, 100.03)["waiting_for_command"]
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.04, 1.04, 100.04)
    assert gate.tick(1.05, 100.05)[0] > 0.0


def test_buffered_command_cannot_be_released_by_enabling():
    gate = MotionGate()
    healthy(gate)
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.005, 1.005, 100.005)
    assert gate.set_enabled(True, 1.02, 100.02)[0]
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.01, 1.03, 100.03)
    assert not gate.enabled
    assert gate.reason == "command:before_enable"
    assert gate.tick(1.04, 100.04) == ZERO


def test_fault_recovery_requires_manual_enable_and_fresh_command():
    gate = armed()
    assert gate.tick(1.05, 100.05)[0] > 0.0
    gate.observe_health("estop", True, 1.06)
    assert gate._output == ZERO
    gate.observe_health("estop", False, 1.07)
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.08, 1.08, 100.08)
    assert gate.tick(1.09, 100.09) == ZERO
    assert not gate.enabled
    assert gate.set_enabled(True, 1.10, 100.10)[0]
    assert gate.tick(1.11, 100.11) == ZERO
    gate.observe_command((0.1, 0.0, 0.0), "base_footprint", 100.12, 1.12, 100.12)
    assert gate.tick(1.15, 100.15)[0] > 0.0


@pytest.mark.parametrize("name,value", [("hardware_ready", False), ("localization", False), ("estop", True)])
def test_each_boolean_fault_stops_immediately(name, value):
    gate = armed()
    assert gate.tick(1.05, 100.05) != ZERO
    assert gate.observe_health(name, value, 1.06) == ZERO
    assert not gate.enabled


def test_boolean_heartbeat_timeout_uses_steady_receipt_clock():
    limits = replace(Limits(), health_timeout_s=0.1)
    gate = armed(limits)
    for name in STAMPED_INPUTS:
        gate.observe_data(name, 100.1, 1.11, 100.1)
    gate.observe_tf(100.1, 1.11, 100.1)
    assert gate.tick(1.11, 100.1) == ZERO
    assert gate.reason == "hardware_ready:receipt_timeout"
    assert not gate.enabled


@pytest.mark.parametrize("name", STAMPED_INPUTS)
def test_invalid_data_does_not_refresh_age_or_allow_enable(name):
    gate = armed()
    gate.observe_data(name, 100.05, 1.05, 100.05, valid=False)
    assert name not in gate._data
    assert not gate.set_enabled(True, 1.06, 100.06)[0]
    assert gate.tick(1.07, 100.07) == ZERO


@pytest.mark.parametrize("stamp,reason", [(0.0, "invalid_stamp"), (99.0, "stale_stamp"),
                                         (101.0, "future_stamp"), (float("nan"), "invalid_stamp")])
def test_stamp_faults_latch_and_do_not_refresh_inputs(stamp, reason):
    gate = armed()
    gate.observe_data("cloud", stamp, 1.02, 100.02)
    assert gate.reason == "cloud:" + reason
    assert "cloud" not in gate._data
    assert not gate.enabled


def test_receipt_timeout_still_stops_when_ros_clock_is_paused():
    limits = replace(Limits(), data_timeout_s=0.1)
    gate = armed(limits)
    assert gate.tick(1.11, 100.02) == ZERO
    assert gate.reason == "odom:receipt_timeout"


def test_measurement_age_timeout_stops_despite_recent_receipt():
    gate = MotionGate(replace(Limits(), max_data_age_s=0.1))
    healthy(gate, wall=1.0, ros=100.0)
    assert gate.set_enabled(True, 1.0, 100.0)[0]
    assert gate.tick(1.01, 100.11) == ZERO
    assert gate.reason == "odom:stale_stamp"


def test_duplicate_and_out_of_order_data_stamps_are_faults():
    gate = armed()
    gate.observe_data("odom", 100.0, 1.02, 100.02)
    assert gate.reason == "odom:nonincreasing_stamp"
    assert "odom" not in gate._data


@pytest.mark.parametrize("frame,velocity", [("map", (0.1, 0.0, 0.0)),
                                            ("", (0.1, 0.0, 0.0)),
                                            ("base_footprint", (float("inf"), 0.0, 0.0))])
def test_invalid_nav_frame_or_velocity_latches_zero(frame, velocity):
    gate = armed()
    assert gate.observe_command(velocity, frame, 100.02, 1.02, 100.02) == ZERO
    assert not gate.enabled


def test_command_timeout_uses_receipt_clock_and_stamped_age():
    gate = armed(replace(Limits(), command_timeout_s=0.1))
    assert gate.tick(1.12, 100.02) == ZERO
    assert gate.reason == "command:receipt_timeout"
    gate = armed(replace(Limits(), command_timeout_s=0.1))
    assert gate.tick(1.02, 100.12) == ZERO
    assert gate.reason == "command:stale_stamp"


def test_tf_failure_stops_and_prevents_enable_until_valid_tf_returns():
    gate = armed()
    assert gate.observe_tf(100.02, 1.02, 100.02, False) == ZERO
    assert not gate.set_enabled(True, 1.03, 100.03)[0]
    gate.observe_tf(100.04, 1.04, 100.04)
    assert not gate.enabled
    assert gate.set_enabled(True, 1.05, 100.05)[0]
    assert gate.tick(1.06, 100.06) == ZERO


def test_velocity_norm_and_angular_caps_apply_after_acceleration_ramp():
    gate = armed(replace(Limits(), max_linear_acceleration=100.0, max_angular_acceleration=100.0))
    gate.observe_command((3.0, 4.0, -20.0), "base_footprint", 100.02, 1.02, 100.02)
    vx, vy, wz = gate.tick(1.05, 100.05)
    assert math.hypot(vx, vy) == pytest.approx(0.2)
    assert vx / vy == pytest.approx(3.0 / 4.0)
    assert wz == pytest.approx(-0.3)


def test_acceleration_caps_apply_to_planar_vector_and_angular_change():
    gate = armed()
    gate.observe_command((1.0, 1.0, 1.0), "base_footprint", 100.02, 1.02, 100.02)
    vx, vy, wz = gate.tick(1.10, 100.10)
    assert math.hypot(vx, vy) == pytest.approx(0.05)
    assert wz == pytest.approx(0.1)


def test_disable_fault_and_zero_command_bypass_deceleration_smoothing():
    gate = armed()
    assert gate.tick(1.10, 100.10) != ZERO
    assert gate.set_enabled(False, 1.11, 100.11)[0]
    assert gate._output == ZERO
    gate = armed()
    assert gate.tick(1.10, 100.10) != ZERO
    assert gate.reject("sensor_fault") == ZERO
    gate = armed()
    assert gate.tick(1.10, 100.10) != ZERO
    gate.observe_command(ZERO, "base_footprint", 100.11, 1.11, 100.11)
    assert gate.tick(1.12, 100.12) == ZERO


def test_ros_time_reset_clears_epoch_and_keeps_gate_disabled():
    gate = armed()
    gate.tick(1.05, 100.05)
    assert gate.tick(1.06, 99.0) == ZERO
    assert gate.reason == "ros_time_went_backwards"
    assert not gate.enabled
    assert not gate._data
    healthy(gate, 1.07, 99.01)
    assert gate.tick(1.08, 99.02) == ZERO


@pytest.mark.parametrize("field", vars(Limits()))
def test_limits_reject_zero_values(field):
    with pytest.raises(ValueError):
        replace(Limits(), **{field: 0.0})
