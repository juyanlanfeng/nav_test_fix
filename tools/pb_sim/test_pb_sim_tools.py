"""Tests for the migrated PB simulation tools (R5 of the repair plan).

Only the pure parts are covered here: the session identity rules (strict boolean
parsing, atomic unique session files, nine-digit nanosecond formatting) and the
isolation verdict rules.  The collector and the analyzer are exercised by real
runs; their parsing helpers are tested where they are pure functions.
"""

import json
import math
from pathlib import Path

import pytest

from pb_sim import check_isolation, session


# --------------------------------------------------------------- time formatting
def test_format_sim_time_uses_nine_nanosecond_digits():
    assert session.format_sim_time(1, 2) == "1.000000002"
    assert session.format_sim_time(12, 60_000_000) == "12.060000000"
    assert session.format_sim_time(0, 0) == "0.000000000"


# --------------------------------------------------------------- boolean parsing
@pytest.mark.parametrize("value", ["1", "true", "TRUE", "yes", "on", True])
def test_parse_bool_accepts_truthy(value):
    assert session.parse_bool(value) is True


@pytest.mark.parametrize("value", ["0", "false", "no", "off", "", False])
def test_parse_bool_rejects_the_old_truthy_string_bug(value):
    # The old shell test `[ -n "${ACCEPT_NEW_RUN:-}" ]` treated "0" as "start a new
    # run"; strict parsing must not.
    assert session.parse_bool(value) is False


def test_parse_bool_rejects_nonsense():
    with pytest.raises(ValueError):
        session.parse_bool("maybe")


# --------------------------------------------------------------------- sessions
def test_session_file_is_unique_default_partition_and_no_overwrite(tmp_path):
    first_path, first = session.create_session(187, label="ramp", session_dir=tmp_path)
    second_path, second = session.create_session(187, label="ramp", session_dir=tmp_path)
    assert first_path != second_path
    assert first["partition"] != second["partition"]
    assert first["domain"] == 187
    # Loading is explicit: there is no "latest session" lookup.
    with pytest.raises(FileNotFoundError):
        session.load_session(tmp_path / "does-not-exist.json")
    assert session.load_session(first_path)["session_id"] == first["session_id"]


def test_session_environment_is_mutually_consistent(tmp_path):
    path, record = session.create_session(191, label="env", session_dir=tmp_path)
    environment = session.session_environment(session.load_session(path))
    assert environment["ROS_DOMAIN_ID"] == "191"
    assert environment["IGN_PARTITION"] == environment["GZ_PARTITION"] == record["partition"]
    assert environment["PB_SIM_SESSION"] == record["session_id"]


def test_env_file_is_sourceable_and_updates_every_derived_value(tmp_path):
    path, record = session.create_session(192, label="envfile", session_dir=tmp_path)
    env_path = session.write_env_file(record, tmp_path / "env.sh")
    text = env_path.read_text()
    assert "export ROS_DOMAIN_ID='192'" in text
    assert "export IGN_PARTITION='%s'" % record["partition"] in text
    assert "export GZ_PARTITION='%s'" % record["partition"] in text
    assert text.endswith("\n")


# ------------------------------------------------------------------ isolation
def _endpoints(clock=(1, ["pb_ros_gz_bridge"]), odom=(1, ["pb_ground_truth_adapter"]),
               cmd=(1, ["pb_cmd_vel_adapter"]), tf=(2, ["pb_ground_truth_adapter",
                                                       "pb_robot_state_publisher"])):
    def entry(value):
        return {"publishers": value[0], "nodes": value[1]}
    return {"/clock": entry(clock), "/odom": entry(odom),
            "/pb/cmd_vel_safe": entry(cmd), "/tf": entry(tf)}


GZ = {"/robot/cmd_vel", "/world/rmuc2026_field/dynamic_pose/info", "/pb_sim/chassis_truth"}


def _verdict(checks):
    return {entry["name"]: entry["ok"] for entry in checks}


def test_isolation_accepts_a_single_owner_for_every_required_topic():
    verdict = _verdict(check_isolation.evaluate(_endpoints(), GZ, "rmuc2026_field"))
    assert all(verdict.values()), verdict


def test_isolation_rejects_a_second_session_sharing_the_domain():
    endpoints = _endpoints(clock=(2, ["pb_ros_gz_bridge"]),
                           odom=(2, ["pb_ground_truth_adapter"]))
    verdict = _verdict(check_isolation.evaluate(endpoints, GZ, "rmuc2026_field"))
    assert verdict["single owner for /clock"] is False
    assert verdict["single owner for /odom"] is False


def test_isolation_rejects_an_unexpected_command_owner():
    endpoints = _endpoints(cmd=(1, ["someone_else"]))
    verdict = _verdict(check_isolation.evaluate(endpoints, GZ, "rmuc2026_field"))
    assert verdict["single owner for /pb/cmd_vel_safe"] is False


def test_isolation_requires_the_tf_chain_owners():
    endpoints = _endpoints(tf=(1, ["pb_ground_truth_adapter"]))
    verdict = _verdict(check_isolation.evaluate(endpoints, GZ, "rmuc2026_field"))
    assert verdict["tf chain owners present"] is False


def test_isolation_treats_a_failed_query_as_a_failure():
    verdict = _verdict(check_isolation.evaluate(_endpoints(), GZ, "rmuc2026_field",
                                                query_failed={"/odom"}))
    assert verdict["query /odom"] is False


def test_isolation_requires_the_gazebo_partition_topics():
    verdict = _verdict(check_isolation.evaluate(_endpoints(), {"/robot/cmd_vel"},
                                                "rmuc2026_field"))
    assert verdict["gazebo partition exposes /world/rmuc2026_field/dynamic_pose/info"] is False


def test_isolation_result_is_json_serialisable():
    checks = check_isolation.evaluate(_endpoints(), GZ, "rmuc2026_field")
    assert json.loads(json.dumps(checks)) == checks
    assert Path("/nonexistent").is_absolute()


# ---------------------------------------------------------------- analyzer helpers
from pb_sim import analyze_report as analyzer
from pb_sim import compare_plugin_runs as comparison


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class _Stamp:
    def __init__(self, sec, nanosec=0):
        self.sec, self.nanosec = sec, nanosec


class _Header:
    def __init__(self, sec, nanosec=0, frame_id="odom"):
        self.stamp = _Stamp(sec, nanosec)
        self.frame_id = frame_id


class _Twist:
    def __init__(self, x=0.0, z=0.0):
        self.linear = _Vector(x=x)
        self.angular = _Vector(z=z)


class _Odom:
    def __init__(self, sec, x=0.0, y=0.0, z=0.0, yaw=0.0, linear=0.0, angular=0.0):
        self.header = _Header(sec)
        self.pose = type("P", (), {})()
        self.pose.pose = type("Q", (), {})()
        self.pose.pose.position = _Vector(x, y, z)
        self.pose.pose.orientation = _Quaternion(0.0, 0.0, math.sin(yaw / 2.0),
                                                   math.cos(yaw / 2.0))
        self.twist = type("T", (), {})()
        self.twist.twist = _Twist(linear, angular)


class _Transform:
    def __init__(self, sec, nanosec=0):
        self.header = _Header(sec, nanosec)


class _TFMessage:
    def __init__(self, *transforms):
        self.transforms = list(transforms)


class _GoalInfo:
    def __init__(self, uuid):
        self.goal_id = type("G", (), {"uuid": uuid})()


class _StatusEntry:
    def __init__(self, uuid, status):
        self.goal_info = _GoalInfo(uuid)
        self.status = status


class _StatusArray:
    def __init__(self, *entries):
        self.status_list = list(entries)


def test_stamp_text_uses_nine_digits():
    message = type("M", (), {"header": _Header(3, 4)})()
    assert analyzer.stamp_text(message) == "3.000000004"


def test_bare_twist_commands_are_placed_on_the_simulation_timeline():
    # A bare geometry_msgs/Twist has no header, so only the bag receive time and
    # the /clock map can locate it.  The old analyzer reported it as a zero stamp.
    clock_map = [(0, 10.0), (1_000_000_000, 11.0), (2_000_000_000, 12.0)]
    samples = [(0, _Twist()), (500_000_000, _Twist(x=0.4)), (1_500_000_000, _Twist()),
               (2_500_000_000, _Twist(x=0.4, z=0.6))]
    summary = analyzer.command_summary(samples, clock_map)
    assert summary["state"] == "ok"
    assert summary["count"] == 4
    assert summary["nonzero_samples"] == 2
    # Linear and angular magnitudes stay separate: 0.4 m/s and 0.6 rad/s, not a
    # combined "speed" of the squared sum.
    assert summary["max_linear_mps"] == 0.4
    assert summary["max_angular_rps"] == 0.6
    # The clock sample at 1.0 s wall reports sim 11.0, so the command received at
    # 0.5 s wall maps to the latest earlier clock sample: sim 10.0.
    assert summary["windows_sim"][0][0] == 10.0


def test_command_summary_reports_missing_without_samples():
    assert analyzer.command_summary([], [])["state"] == "missing"


def test_action_sequences_are_grouped_by_uuid_with_a_terminal_result():
    first = bytes(range(16))
    second = bytes(range(16, 32))
    samples = [
        (0, _StatusArray(_StatusEntry(first, 2))),
        (1, _StatusArray(_StatusEntry(first, 2), _StatusEntry(second, 1))),
        (2, _StatusArray(_StatusEntry(first, 4), _StatusEntry(second, 5))),
    ]
    result = analyzer.action_sequences(samples)
    assert result["goals"][first.hex()]["terminal_name"] == "SUCCEEDED"
    assert result["goals"][second.hex()]["terminal_name"] == "CANCELED"
    assert result["goals"][first.hex()]["statuses"] == [2, 4]


def test_tf_stamp_summary_counts_per_transform_stamps():
    # TFMessage has no top-level header, so "no envelope" must not be reported as
    # "zero stamp therefore fine": the per-transform stamps are counted.
    samples = [(0, _TFMessage(_Transform(5), _Transform(0, 0)))]
    summary = analyzer.tf_stamp_summary(samples)
    assert summary["transforms"] == 2
    assert summary["zero_stamped_transforms"] == 1
    assert "no top-level header" in summary["note"]


def test_gazebo_records_are_counted_structurally_not_by_lines():
    text = "pose {\n  name: \"robot\"\n}\n\npose {\n  name: \"robot\"\n}\n\n"
    assert analyzer.count_gazebo_records(text, "pose")["records"] == 2
    assert analyzer.count_gazebo_records("", "pose")["state"] == "missing"


def test_gazebo_rtf_reports_min_and_max():
    text = "real_time_factor: 0.94\nreal_time_factor: 1.0011\n"
    summary = analyzer.gazebo_rtf(text)
    assert summary["samples"] == 2
    assert summary["min"] == 0.94 and summary["max"] == 1.0011


def test_extrinsic_consistency_compares_truth_and_odom():
    truth = [(0, _Odom(7, x=1.0, y=2.0, z=0.076))]
    odom = [(0, _Odom(7, x=1.0, y=2.0, z=0.0))]
    summary = analyzer.extrinsic_consistency(truth, odom)
    assert summary["state"] == "ok" and summary["ok"] is True
    assert summary["worst_position_error_m"] == 0.0
    mismatched = analyzer.extrinsic_consistency(truth, [(0, _Odom(7, x=1.5, y=2.0, z=0.0))])
    assert mismatched["ok"] is False


def test_extrinsic_consistency_is_missing_without_matching_stamps():
    summary = analyzer.extrinsic_consistency([(0, _Odom(7))], [(0, _Odom(9))])
    assert summary["state"] == "missing"


def test_motion_summary_reports_ranges_and_stop_intervals():
    samples = [
        (0, _Odom(0, x=0.0, y=0.0, z=0.0, linear=0.3)),
        (1, _Odom(1, x=1.0, y=0.5, z=0.1, linear=0.3)),
        (2, _Odom(2, x=1.0, y=0.5, z=0.1, linear=0.0)),
        (3, _Odom(3, x=1.0, y=0.5, z=0.1, linear=0.0)),
    ]
    summary = analyzer.motion_summary(samples)
    assert summary["x_range"] == [0.0, 1.0]
    assert summary["z_range"] == [0.0, 0.1]
    assert summary["cumulative_displacement"] > 1.0
    assert summary["stop_intervals_sim"] == [[2.0, 3.0]]


# ------------------------------------------------------------- isolation: free mode
def _free_endpoints():
    def entry():
        return {"publishers": 0, "nodes": []}
    return {topic: entry() for topic in ("/clock", "/odom", "/pb/cmd_vel_safe", "/tf")}


def test_free_mode_passes_when_nothing_owns_the_domain():
    verdict = _verdict(check_isolation.evaluate(_free_endpoints(), set(), "rmuc2026_field",
                                                (), "free"))
    assert all(verdict.values()), verdict


def test_free_mode_passes_when_no_gazebo_server_answers():
    verdict = _verdict(check_isolation.evaluate(_free_endpoints(), None, "rmuc2026_field",
                                                (), "free"))
    assert all(verdict.values()), verdict


def test_free_mode_fails_on_a_leftover_session():
    endpoints = _free_endpoints()
    endpoints["/clock"] = {"publishers": 1, "nodes": ["pb_ros_gz_bridge"]}
    verdict = _verdict(check_isolation.evaluate(endpoints, set(), "rmuc2026_field",
                                                (), "free"))
    assert verdict["/clock is free"] is False


def test_free_mode_fails_when_the_domain_cannot_be_proven_free():
    verdict = _verdict(check_isolation.evaluate(_free_endpoints(), set(), "rmuc2026_field",
                                                {"/odom"}, "free"))
    assert verdict["query /odom"] is False


def test_free_mode_fails_when_a_gazebo_server_is_in_the_partition():
    verdict = _verdict(check_isolation.evaluate(_free_endpoints(), {"/robot/cmd_vel"},
                                                "rmuc2026_field", (), "free"))
    assert verdict["gazebo partition is free"] is False


def test_extrinsic_consistency_rotates_the_offset_on_a_pitched_chassis():
    # 90 deg roll about X: the base offset (chassis -Z) points along world +Y, so the
    # flat-ground shortcut "subtract 0.076 from z" is wrong by the full 0.076 m.
    half = math.sin(math.pi / 4.0)
    truth = [(0, _Odom(11, x=1.0, y=2.0, z=0.076))]
    truth[0][1].pose.pose.orientation = _Quaternion(half, 0.0, 0.0, half)
    odom = [(0, _Odom(11, x=1.0, y=2.076, z=0.076))]
    summary = analyzer.extrinsic_consistency(truth, odom)
    assert summary["ok"] is True, summary
    assert summary["worst_position_error_m"] < 1.0e-9


def test_post_stop_drift_reports_peak_and_net_drift():
    samples = [
        (0, _Odom(10, x=0.0, y=0.0, z=0.0)),
        (1, _Odom(20, x=0.05, y=0.0, z=0.0)),
        (2, _Odom(30, x=0.10, y=0.02, z=0.0)),
    ]
    summary = analyzer.post_stop_drift(samples, last_command_sim=10.0)
    assert summary["state"] == "ok"
    assert summary["peak_drift_m"] == pytest.approx(0.10198, abs=1e-4)
    assert summary["net_drift_m"] == pytest.approx(0.10198, abs=1e-4)
    assert summary["window_s"] == 20.0


def test_post_stop_drift_is_missing_without_a_window():
    assert analyzer.post_stop_drift([(0, _Odom(1))], 5.0)["state"] == "missing"
    assert analyzer.post_stop_drift([], None)["state"] == "missing"


def _joint_sample(stamp, **velocities):
    names = {"front_left_wheel_joint": 0.0, "front_right_wheel_joint": 0.0,
             "rear_left_wheel_joint": 0.0, "rear_right_wheel_joint": 0.0}
    names.update(velocities)
    return (stamp, names)


def test_wheel_vs_body_is_missing_without_both_streams():
    assert analyzer.wheel_vs_body([], [])["state"] == "missing"
    assert analyzer.wheel_vs_body([_joint_sample(0.0)], [])["state"] == "missing"


def test_wheel_vs_body_calls_a_slow_creep_moving_not_held():
    """A vehicle creeping at 4 cm/s is moving; an instantaneous-speed test said held."""
    joints = [_joint_sample(stamp, front_left_wheel_joint=0.53)
              for stamp in (0.0, 1.0, 2.0)]
    body = [(0.0, 0.0, 0.0, 0.04), (1.0, 0.04, 0.0, 0.04), (2.0, 0.08, 0.0, 0.04)]
    summary = analyzer.wheel_vs_body(joints, body)
    assert summary["fractions"]["held"] == 0.0, summary
    assert summary["move_displacement_m"] == 0.01
    assert summary["moving_window_s"] == 1.0


def test_wheel_vs_body_still_holds_a_body_that_only_jitters():
    """Sub-centimetre noise inside the window is not motion."""
    joints = [_joint_sample(stamp, rear_left_wheel_joint=0.5)
              for stamp in (0.0, 1.0, 2.0)]
    body = [(0.0, 0.0, 0.0, 0.02), (1.0, 0.004, 0.0, 0.02), (2.0, 0.004, 0.0, 0.02)]
    summary = analyzer.wheel_vs_body(joints, body)
    assert summary["fractions"]["held"] == 1.0, summary


def test_wheel_vs_body_classifies_rolling_slip_held_and_static():
    radius = analyzer.WHEEL_RADIUS
    body = [(0.0, 0.00, 0.0, radius * 4.0),   # reference sample, also the warm-up
            (0.1, 0.05, 0.0, 0.50),           # advancing at the wheel surface speed
            (0.2, 0.06, 0.0, 0.10),           # 1 cm only: below the window threshold
            (0.25, 0.08, 0.0, 0.20),          # 2 cm, but far slower than the wheels
            (0.3, 0.08, 0.0, 0.00)]           # stopped
    joints = [
        _joint_sample(0.0, front_left_wheel_joint=4.0, front_right_wheel_joint=4.0,
                      rear_left_wheel_joint=4.0, rear_right_wheel_joint=4.0),
        _joint_sample(0.1, front_left_wheel_joint=4.0, front_right_wheel_joint=4.0,
                      rear_left_wheel_joint=4.0, rear_right_wheel_joint=4.0),
        _joint_sample(0.2, front_left_wheel_joint=0.36),   # one wheel turns, body held
        _joint_sample(0.25, front_left_wheel_joint=20.0, front_right_wheel_joint=20.0,
                      rear_left_wheel_joint=20.0, rear_right_wheel_joint=20.0),
        _joint_sample(0.3),
    ]
    summary = analyzer.wheel_vs_body(joints, body, window_s=0.05)
    assert summary["state"] == "ok"
    assert summary["warmup_skipped"] == 1, summary
    assert summary["fractions"] == {"rolling": 0.25, "slip": 0.25, "held": 0.25,
                                   "static": 0.25, "idle": 0.0}, summary
    assert summary["longest_held_s"] == pytest.approx(0.0)


def test_wheel_vs_body_measures_the_longest_held_interval():
    joints = [_joint_sample(stamp, front_left_wheel_joint=0.36)
              for stamp in (0.0, 1.0, 2.0, 3.0)]
    joints.append(_joint_sample(4.0))
    body = [(stamp, 0.0, 0.0, 0.0) for stamp in (0.0, 1.0, 2.0, 3.0, 4.0)]
    summary = analyzer.wheel_vs_body(joints, body)
    # t=0.0 is warm-up; t=1,2,3 are held and t=4 has stopped turning.
    assert summary["warmup_skipped"] == 1
    assert summary["fractions"]["held"] == pytest.approx(0.75)
    assert summary["longest_held_s"] == pytest.approx(2.0)


def test_wheel_vs_body_uses_the_latest_body_sample_at_or_before_the_joint_time():
    joints = [_joint_sample(0.5, front_left_wheel_joint=4.0, front_right_wheel_joint=4.0,
                            rear_left_wheel_joint=4.0, rear_right_wheel_joint=4.0)]
    # The sample after the joint time shows a stopped body; if the code looked
    # forward it would report "held" instead of "rolling".
    body = [(0.0, 0.00, 0.0, 0.0), (0.4, 0.05, 0.0, 0.5), (0.6, 0.05, 0.0, 0.0)]
    summary = analyzer.wheel_vs_body(joints, body, window_s=0.5)
    assert summary["fractions"]["rolling"] == 1.0, summary


def test_wheel_vs_body_records_the_held_window_and_per_wheel_detail():
    joints = [
        _joint_sample(0.0, front_left_wheel_joint=2.0, front_right_wheel_joint=2.0),
        _joint_sample(1.0, front_left_wheel_joint=0.36),      # only one wheel turns
        _joint_sample(2.0, front_left_wheel_joint=0.36),
        _joint_sample(3.0),
    ]
    body = [(stamp, 0.0, 0.0, 0.0) for stamp in (0.0, 1.0, 2.0, 3.0)]
    summary = analyzer.wheel_vs_body(joints, body)
    # t=0.0 is warm-up for the moving test; two samples (t=1,2) are held.
    assert summary["longest_held_window_sim_s"] == [1.0, 2.0]
    assert summary["held_wheels_turning_histogram"] == {"0": 0, "1": 2, "2": 0, "3": 0, "4": 0}
    detail = summary["held_mean_abs_wheel_rad_s"]
    assert detail["front_left_wheel_joint"] == pytest.approx(0.36)
    assert detail["front_right_wheel_joint"] == pytest.approx(0.0)
    assert detail["rear_right_wheel_joint"] == pytest.approx(0.0)


def test_cancel_latency_breakdown_splits_the_observable_segments():
    uuid = bytes.fromhex("11" * 16)
    status = [
        (0, _StatusArray(_StatusEntry(uuid, 2))),
        (1_000_000_000, _StatusArray(_StatusEntry(uuid, 3))),
        (4_000_000_000, _StatusArray(_StatusEntry(uuid, 5))),
    ]
    clock = [(500_000_000 * index, 0.5 * index) for index in range(11)]
    commands = [(2_000_000_000, _Twist(0.2, 0.0)), (2_500_000_000, _Twist(0.0, 0.0))]
    odom = [(2_600_000_000, _Odom(2, linear=0.2)), (4_500_000_000, _Odom(4, linear=0.0))]
    summary = analyzer.cancel_latency_breakdown(status, commands, odom, clock)
    assert summary["state"] == "ok"
    assert summary["accept_to_zero_command_s"] == pytest.approx(1.0)
    # standstill is observed 0.5 s after the CANCELED status, so the segment is
    # reported as a positive "still rolling after terminal", not a negative span.
    assert summary["zero_command_to_standstill_s"] == pytest.approx(2.5)
    assert summary["standstill_after_terminal_s"] == pytest.approx(0.5)
    assert summary["accept_to_terminal_s"] == pytest.approx(3.0)
    assert summary["request_to_accept_s"] is None


def test_cancel_latency_breakdown_is_missing_without_transitions():
    assert analyzer.cancel_latency_breakdown([], [], [], [])["state"] == "missing"
    uuid = bytes.fromhex("22" * 16)
    only_active = [(0, _StatusArray(_StatusEntry(uuid, 2)))]
    assert analyzer.cancel_latency_breakdown(only_active, [], [], [])["state"] == "missing"


def _comparison_run(label, poses, first_segment, error, displacement, start=None):
    rounded = [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]]
    return {
        "label": label,
        "plan": {"poses": poses, "first_segment_world_rad": first_segment,
                 "polyline_sha256": "%s-hash" % label, "polyline_rounded_mm": rounded},
        "robot_start": start or {"x": -1.9, "y": -5.95, "yaw_rad": 0.0},
        "motion": {"final_goal_error_m": error, "net_displacement_m": displacement},
    }


def test_comparison_normalize_wraps_angles():
    assert comparison.normalize(0.0) == pytest.approx(0.0)
    assert comparison.normalize(math.pi) == pytest.approx(math.pi)
    assert comparison.normalize(3.0 * math.pi) == pytest.approx(math.pi)
    # atan2 keeps the sign of the negative-y zero, so -3*pi lands on -pi.
    assert comparison.normalize(-3.0 * math.pi) == pytest.approx(-math.pi)


def test_comparison_implicates_the_controller_when_plan_and_start_match():
    runs = [_comparison_run("a", 68, 1.5708, 2.9878, 0.3204),
            _comparison_run("b", 68, 1.5708, 0.1682, 3.0931)]
    result = comparison.compare(runs)
    assert result["same_plan"] is True
    assert result["same_robot_start"] is True
    assert result["outcomes_diverge"] is True
    assert "tracking layer" in result["conclusion"]


def test_comparison_rejects_a_different_plan():
    runs = [_comparison_run("a", 68, 1.5708, 2.9878, 0.3204),
            _comparison_run("b", 42, 1.5708, 0.1682, 3.0931)]
    result = comparison.compare(runs)
    assert result["same_plan"] is False
    assert "planner" in result["conclusion"]


def test_comparison_detects_a_plan_beyond_the_tolerance():
    runs = [_comparison_run("a", 68, 1.5708, 2.9878, 0.3204),
            _comparison_run("b", 68, 1.5708, 0.1682, 3.0931)]
    runs[1]["plan"]["polyline_rounded_mm"] = [[0.0, 0.0, 0.0], [0.5, 0.02, 0.0]]
    result = comparison.compare(runs)
    assert result["same_plan"] is False
    assert result["max_plan_deviation_m"][0] == pytest.approx(0.02)


def test_comparison_needs_a_real_divergence():
    runs = [_comparison_run("a", 68, 1.5708, 0.10, 3.0),
            _comparison_run("b", 68, 1.5708, 0.15, 3.0)]
    result = comparison.compare(runs)
    assert result["outcomes_diverge"] is False
    assert "no controller difference is demonstrated" in result["conclusion"]


def test_comparison_command_split_separates_rotation_from_translation():
    commands = [(0, 0.0, 0.0, 0.5), (10**9, 0.3, 0.0, 0.0)]
    summary = comparison.command_split(commands, window_s=12.0)
    assert summary["nonzero"] == 2
    assert summary["pure_rotation"] == 1
    assert summary["pure_rotation_fraction"] == pytest.approx(0.5)
    assert summary["first_nonzero"] == [0.0, 0.0, 0.5]
    assert comparison.command_split([])["state"] == "missing"
    assert comparison.command_split([(0, 0.0, 0.0, 0.0)])["state"] == "idle"


def test_wheel_vs_body_does_not_call_in_place_rotation_held():
    """Rotating on the spot during alignment is motion, not a pinned chassis."""
    joints = [_joint_sample(stamp, front_left_wheel_joint=0.5,
                            front_right_wheel_joint=0.5)
              for stamp in (0.0, 1.0, 2.0)]
    body = [(0.0, 0.0, 0.0, 0.00, 0.0), (1.0, 0.0, 0.0, 0.10, 0.0),
            (2.0, 0.0, 0.0, 0.20, 0.0)]
    summary = analyzer.wheel_vs_body(joints, body, window_s=1.0)
    assert summary["fractions"]["held"] == 0.0, summary
    assert summary["fractions"]["slip"] == 1.0, summary
    assert summary["move_yaw_rad"] == 0.02


def test_wheel_vs_body_holds_a_body_that_neither_moves_nor_turns():
    joints = [_joint_sample(stamp, rear_left_wheel_joint=0.9) for stamp in (0.0, 1.0, 2.0)]
    body = [(0.0, 0.0, 0.0, 0.5, 0.0), (1.0, 0.002, 0.0, 0.505, 0.0),
            (2.0, 0.002, 0.0, 0.505, 0.0)]
    summary = analyzer.wheel_vs_body(joints, body, window_s=1.0)
    assert summary["fractions"]["held"] == 1.0, summary


def test_wheel_vs_body_separates_uncommanded_spin_from_being_held():
    """Zero command with a turning wheel is idle (actuator drift), not pinned."""
    joints = [_joint_sample(stamp, rear_left_wheel_joint=0.9) for stamp in (0.0, 1.0, 2.0)]
    body = [(stamp, 0.0, 0.0, 0.0, 0.0) for stamp in (0.0, 1.0, 2.0)]

    idle = analyzer.wheel_vs_body(joints, body, [(0.0, 0.0)], window_s=1.0)
    assert idle["command_aware"] is True
    assert idle["fractions"]["idle"] == 1.0, idle
    assert idle["fractions"]["held"] == 0.0

    driven = analyzer.wheel_vs_body(joints, body, [(0.0, 0.4)], window_s=1.0)
    assert driven["fractions"]["held"] == 1.0, driven
    assert driven["fractions"]["idle"] == 0.0

    # No command stream at all: the classification stays command-unaware.
    blind = analyzer.wheel_vs_body(joints, body, window_s=1.0)
    assert blind["command_aware"] is False
    assert blind["fractions"]["held"] == 1.0


def test_wheel_vs_body_idle_ends_when_a_command_returns():
    joints = [_joint_sample(stamp, rear_left_wheel_joint=0.9)
              for stamp in (0.0, 1.0, 2.0, 3.0)]
    body = [(stamp, 0.0, 0.0, 0.0, 0.0) for stamp in (0.0, 1.0, 2.0, 3.0)]
    # Driven up to t=1, zero from 1.5 onwards.
    commands = [(0.0, 0.4), (1.5, 0.0)]
    summary = analyzer.wheel_vs_body(joints, body, commands, window_s=1.0)
    # t=0.0 is warm-up; t=1.0 is still driven, t=2.0 and t=3.0 are uncommanded.
    assert summary["fractions"]["held"] == pytest.approx(0.3333, abs=1e-4)
    assert summary["fractions"]["idle"] == pytest.approx(0.6667, abs=1e-4)
