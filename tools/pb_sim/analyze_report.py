#!/usr/bin/env python3
"""Analyze one PB simulation evidence directory (R5, plan section 8).

Replaces ``log/analyze_slope_report.py``.  Fixes, per the plan's table:

* parses ``/clock`` and keeps the bag receive time, so simulation duration and wall
  duration are reported separately and a bare ``Twist`` command can still be placed
  on the simulation timeline (defect: bare Twist had no header to read);
* reads the per-transform stamps of ``TFMessage`` and never treats "no top-level
  header" as "stamp zero, therefore fine";
* lists action sequences by goal UUID with their terminal result instead of
  accumulating every historical status value;
* counts Gazebo messages structurally instead of counting non-empty text lines, and
  cross-checks the Gazebo chassis truth against the ROS ``/odom`` through the known
  base->chassis extrinsic;
* reports linear (m/s) and angular (rad/s) magnitudes separately;
* reports command frequency and gaps, motion and stop intervals, XYZ and attitude
  ranges, cumulative displacement, goal error, post-stop drift and cancel latency;
* marks missing or unparsable inputs as ``missing``/``failed`` and never lets a
  report pass on absent data.

    python3 tools/pb_sim/analyze_report.py log/<label>_report_<stamp>
"""

import argparse
import bisect
import json
import math
from pathlib import Path
import re
import sys

# Runnable both as `python3 tools/pb_sim/analyze_report.py` and as
# `pb_sim.analyze_report` from the test suite.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pb_sim.session import format_sim_time

# Physics/acceptance constants are declared here, before a run, not chosen after
# looking at the result.
STOP_LINEAR_TOLERANCE = 0.05      # m/s
STOP_ANGULAR_TOLERANCE = 0.20     # rad/s
STOP_HOLD_S = 0.5
BASE_TO_CHASSIS_Z = 0.076         # m, PB base_footprint -> chassis
EXTRINSIC_POSITION_TOLERANCE = 0.01  # m
# Deserializing these at their publish rate costs more than the information they add
# here; everything else (including /odom, /tf and the truth topic) is parsed.
SKIP_DESERIALIZE = {"/joint_states", "/clock", "/rosout"}


def _q_inverse(quaternion):
    x, y, z, w = quaternion
    length_sq = x * x + y * y + z * z + w * w
    if length_sq < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (-x / length_sq, -y / length_sq, -z / length_sq, w / length_sq)


def _q_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def yaw_of(message):
    """Planar yaw from an odometry pose (the same convention the rest of the file uses)."""
    orientation = message.pose.pose.orientation
    return 2.0 * math.atan2(orientation.z, orientation.w)


def rotate_vector(quaternion, vector):
    """Rotate a vector by a quaternion (x, y, z, w)."""
    rotated = _q_multiply(_q_multiply(quaternion, (vector[0], vector[1], vector[2], 0.0)),
                          _q_inverse(quaternion))
    return (rotated[0], rotated[1], rotated[2])


def stamp_of(message):
    header = getattr(message, "header", None)
    if header is None:
        return None
    return header.stamp.sec + header.stamp.nanosec * 1.0e-9


def stamp_text(message):
    header = getattr(message, "header", None)
    if header is None:
        return None
    return format_sim_time(header.stamp.sec, header.stamp.nanosec)


# ------------------------------------------------------------------- pure helpers
def sim_time_for(receive_ns, clock_map):
    """Nearest simulation time at or before a bag receive time (None if unknown)."""
    if not clock_map:
        return None
    best = None
    for sample_ns, sim in clock_map:
        if sample_ns <= receive_ns:
            best = sim
        else:
            break
    return best if best is not None else clock_map[0][1]


def twist_magnitudes(twist):
    linear = math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2)
    angular = abs(twist.angular.z)
    return linear, angular


def command_summary(samples, clock_map):
    """Frequency, gaps and non-zero windows of one command stream.

    `samples` is [(receive_ns, message)]; the message may be a Twist (no stamp), so
    the simulation time comes from the bag receive time through /clock.
    """
    if not samples:
        return {"state": "missing", "count": 0}
    receive_times = [receive_ns for receive_ns, _ in samples]
    span_wall = (receive_times[-1] - receive_times[0]) / 1.0e9
    rate = (len(samples) - 1) / span_wall if span_wall > 0 else None
    windows = []
    open_window = None
    gaps = []
    previous_nonzero = None
    nonzero = 0
    max_linear = 0.0
    max_angular = 0.0
    for receive_ns, message in samples:
        twist = getattr(message, "twist", message)
        linear, angular = twist_magnitudes(twist)
        active = linear > 1.0e-3 or angular > 1.0e-3
        if active:
            nonzero += 1
            max_linear = max(max_linear, linear)
            max_angular = max(max_angular, angular)
            sim = sim_time_for(receive_ns, clock_map)
            if open_window is None:
                open_window = [sim, sim]
            else:
                open_window[1] = sim
            if previous_nonzero is not None:
                gap = (receive_ns - previous_nonzero) / 1.0e9
                if gap > 0.5:
                    gaps.append(round(gap, 3))
            previous_nonzero = receive_ns
        elif open_window is not None:
            windows.append(open_window)
            open_window = None
    if open_window is not None:
        windows.append(open_window)
    return {
        "state": "ok",
        "count": len(samples),
        "rate_hz": None if rate is None else round(rate, 2),
        "nonzero_samples": nonzero,
        "max_linear_mps": round(max_linear, 4),
        "max_angular_rps": round(max_angular, 4),
        "windows_sim": [[None if v is None else round(v, 3) for v in window] for window in windows],
        "gaps_over_0_5s": gaps,
    }


def motion_summary(samples):
    """XYZ and attitude ranges, cumulative displacement, stop intervals, drift."""
    if not samples:
        return {"state": "missing", "count": 0}
    positions = []
    attitudes = []
    speeds = []
    for receive_ns, message in samples:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        positions.append((stamp_of(message), position.x, position.y, position.z))
        attitudes.append(2.0 * math.atan2(orientation.z, orientation.w))
        linear, angular = twist_magnitudes(message.twist.twist)
        speeds.append((stamp_of(message), linear, angular))
    xs = [p[1] for p in positions]
    ys = [p[2] for p in positions]
    zs = [p[3] for p in positions]
    cumulative = sum(math.dist(positions[i][1:], positions[i + 1][1:])
                     for i in range(len(positions) - 1))
    stopped_since = None
    stops = []
    for stamp, linear, angular in speeds:
        stopped = linear <= STOP_LINEAR_TOLERANCE and angular <= STOP_ANGULAR_TOLERANCE
        if stopped and stopped_since is None:
            stopped_since = stamp
        elif not stopped and stopped_since is not None:
            if stamp - stopped_since >= STOP_HOLD_S:
                stops.append([round(stopped_since, 3), round(stamp, 3)])
            stopped_since = None
    if stopped_since is not None and speeds and speeds[-1][0] - stopped_since >= STOP_HOLD_S:
        stops.append([round(stopped_since, 3), round(speeds[-1][0], 3)])
    return {
        "state": "ok",
        "count": len(samples),
        "x_range": [round(min(xs), 4), round(max(xs), 4)],
        "y_range": [round(min(ys), 4), round(max(ys), 4)],
        "z_range": [round(min(zs), 4), round(max(zs), 4)],
        "yaw_range": [round(min(attitudes), 4), round(max(attitudes), 4)],
        "first_position": [round(v, 4) for v in positions[0][1:]],
        "last_position": [round(v, 4) for v in positions[-1][1:]],
        "cumulative_displacement": round(cumulative, 4),
        "stop_intervals_sim": stops,
    }


def post_stop_drift(samples, last_command_sim):
    """Parking evidence: how far the robot moves after the last non-zero command.

    The plan asks for peak and net drift plus the attitude change over an
    observation window, because "the controller stopped commanding" is not the same
    as "the vehicle stays where it was told to stop".
    """
    if not samples or last_command_sim is None:
        return {"state": "missing", "reason": "no command timeline available"}
    after = []
    for _receive_ns, message in samples:
        stamp = stamp_of(message)
        if stamp is not None and stamp >= last_command_sim:
            after.append((stamp, message))
    if len(after) < 2:
        return {"state": "missing", "reason": "no samples after the last command"}
    first_stamp, first = after[0]
    first_position = (first.pose.pose.position.x, first.pose.pose.position.y,
                      first.pose.pose.position.z)
    yaws = []
    peak = 0.0
    net = 0.0
    for _stamp, message in after:
        position = (message.pose.pose.position.x, message.pose.pose.position.y,
                    message.pose.pose.position.z)
        distance = math.dist(position, first_position)
        peak = max(peak, distance)
        net = distance
        orientation = message.pose.pose.orientation
        yaws.append(2.0 * math.atan2(orientation.z, orientation.w))
    return {
        "state": "ok",
        "window_s": round(after[-1][0] - first_stamp, 3),
        "peak_drift_m": round(peak, 4),
        "net_drift_m": round(net, 4),
        "yaw_change_rad": round(yaws[-1] - yaws[0], 4),
        "yaw_range_rad": [round(min(yaws), 4), round(max(yaws), 4)],
    }


WHEEL_RADIUS = 0.0758        # m, PB2025 wheel (see the vehicle description)
WHEEL_JOINTS = ("front_left_wheel_joint", "front_right_wheel_joint",
                "rear_left_wheel_joint", "rear_right_wheel_joint")


def wheel_vs_body(joint_samples, body_samples, command_samples=None, window_s=1.0,
                  move_displacement_m=0.01, move_yaw_rad=0.02, spin_threshold=0.2,
                  slip_ratio=3.0, command_hold_s=0.5):
    """Classify the ground contact from engine wheel spin against body motion.

    This is the discriminator the plan asks for between "the wheels keep slipping"
    and "the vehicle is held": the wheel joints are engine state, not an inference
    from ROS velocity.  Classes:

      rolling  the fastest wheel turns at a surface speed matching the body
      slip     the wheels turn much faster than the body moves
      held     the body does not move while at least one wheel turns
      static   neither the body nor any wheel turns (no command or fully blocked)

    "Does not move" is judged over a sliding window from **both** the translation and
    the yaw change, never from instantaneous speed alone:

    * an instantaneous-speed threshold misreports a vehicle creeping at 0.04 m/s as
      held — it did, on a run that travelled 3.2 m;
    * a translation-only window misreports a vehicle **rotating in place** during
      final alignment as held — it did, on the successful -Y run, for 31 s.

    ``held`` therefore means the body neither translated nor rotated while a wheel
    turned **and the controller was asking for motion**.  A wheel that turns while
    every command is zero is a separate class, ``idle``: on this vehicle the
    force-based drive does not hold station at zero command, so an uncommanded
    wheel is parking/actuator evidence (T7), not evidence of a pinned chassis.  It
    deliberately keys off the fastest wheel rather than the mean: three stationary
    wheels plus one turning wheel is a held vehicle, and averaging would report it
    as static.

    ``body_samples`` are ``(stamp, x, y, yaw, speed)``; ``(stamp, x, y, speed)``
    drops the yaw test and ``(stamp, speed)`` falls back to the speed test.

    Contact points/normals/forces are NOT derivable from this and must be recorded
    separately when contact sensors are available.
    """
    if not joint_samples or not body_samples:
        return {"state": "missing", "reason": "joint_states or odom absent"}
    body = sorted(body_samples, key=lambda entry: entry[0])

    def latest(stamp):
        """Index of the newest body sample at or before ``stamp`` (None if none)."""
        low, high, found = 0, len(body) - 1, None
        while low <= high:
            middle = (low + high) // 2
            if body[middle][0] <= stamp:
                found = middle
                low = middle + 1
            else:
                high = middle - 1
        return found

    commanded = None
    if command_samples:
        # Caller supplies (sim_time, magnitude); keep the sort and split the times so
        # the per-sample lookup is a binary search rather than a scan.
        commanded = sorted((float(entry[0]), float(entry[1])) for entry in command_samples)
        if not commanded:
            commanded = None
    command_times = [entry[0] for entry in commanded] if commanded else []
    nonzero_times = [entry[0] for entry in commanded if entry[1] > 0.0] if commanded else []

    def was_commanded(stamp):
        """True if a non-zero velocity command was in force within the hold window."""
        if commanded is None:
            return None
        index = bisect.bisect_right(command_times, stamp) - 1
        if index < 0:
            return None
        if commanded[index][1] > 0.0:
            return True
        last = bisect.bisect_right(nonzero_times, stamp) - 1
        if last < 0:
            return False
        return (stamp - nonzero_times[last]) <= command_hold_s

    counts = {"rolling": 0, "slip": 0, "held": 0, "static": 0, "idle": 0}
    samples = []
    warmup = 0
    for stamp, velocities in joint_samples:
        index = latest(stamp)
        if index is None:
            continue
        entry = body[index]
        speed = entry[-1]
        if len(entry) >= 4:
            earlier = latest(stamp - window_s)
            if earlier is None:
                # No sample old enough to measure a full window: the displacement
                # would be zero by construction, so "held" here would be an artifact
                # of the warm-up rather than evidence of a pinned chassis.
                warmup += 1
                continue
            moved = math.hypot(entry[1] - body[earlier][1], entry[2] - body[earlier][2])
            turned = 0.0
            if len(entry) >= 5:
                turned = abs(math.atan2(math.sin(entry[3] - body[earlier][3]),
                                        math.cos(entry[3] - body[earlier][3])))
            moving = moved > move_displacement_m or turned > move_yaw_rad
        else:
            moving = speed > move_displacement_m
        wheel_speeds = [abs(velocities.get(name, 0.0)) for name in WHEEL_JOINTS]
        fastest = max(wheel_speeds)
        mean_wheel = sum(wheel_speeds) / len(wheel_speeds)
        surface = fastest * WHEEL_RADIUS
        if not moving and fastest <= spin_threshold:
            label = "static"
        elif not moving:
            label = "held" if was_commanded(stamp) is not False else "idle"
        elif surface > slip_ratio * max(speed, 1.0e-6):
            label = "slip"
        else:
            label = "rolling"
        counts[label] += 1
        samples.append((stamp, label, speed, fastest, wheel_speeds))
    total = sum(counts.values())
    if total == 0:
        return {"state": "missing", "reason": "no body sample covers a full window",
                "warmup_skipped": warmup}
    longest_held = 0.0
    window = None
    start = None
    previous = None
    for stamp, label, _speed, _fastest, _wheels in samples:
        if label == "held":
            if start is None:
                start = stamp
            previous = stamp
        else:
            if start is not None and previous - start > longest_held:
                longest_held = previous - start
                window = [round(start, 3), round(previous, 3)]
            start = None
    if start is not None and previous - start > longest_held:
        longest_held = previous - start
        window = [round(start, 3), round(previous, 3)]

    # How many wheels are actually turning while the body is pinned decides between
    # "the whole drivetrain is stalled" and "the chassis rests on something and one
    # wheel is unloaded".
    held_wheels = [entry[4] for entry in samples if entry[1] == "held"]
    turning_histogram = {str(count): 0 for count in range(len(WHEEL_JOINTS) + 1)}
    held_mean = [0.0] * len(WHEEL_JOINTS)
    for wheels in held_wheels:
        turning_histogram[str(sum(1 for value in wheels if value > spin_threshold))] += 1
        for index, value in enumerate(wheels):
            held_mean[index] += value
    if held_wheels:
        held_mean = [round(value / len(held_wheels), 4) for value in held_mean]
    return {
        "state": "ok",
        "samples": total,
        "fractions": {key: round(value / total, 4) for key, value in counts.items()},
        "command_aware": commanded is not None,
        "longest_held_s": round(longest_held, 3),
        "longest_held_window_sim_s": window,
        "held_wheels_turning_histogram": turning_histogram,
        "held_mean_abs_wheel_rad_s": dict(zip(WHEEL_JOINTS, held_mean)),
        "wheel_radius_m": WHEEL_RADIUS,
        "moving_window_s": window_s,
        "move_displacement_m": move_displacement_m,
        "move_yaw_rad": move_yaw_rad,
        "warmup_skipped": warmup,
    }


CANCEL_PREEMPTING = 3
CANCEL_CANCELED = 5


def cancel_latency_breakdown(status_samples, command_samples, odom_samples, clock_map,
                             standstill_speed=0.01):
    """Split the cancel latency into the segments the bag can actually observe.

    ``request -> accept`` is deliberately absent: no ROS message carries the cancel
    request itself, so only the server-side ``PREEMPTING`` transition is visible.
    The reported segments are therefore:

      accept_to_zero_command   PREEMPTING seen -> last non-zero command published
      zero_command_to_standstill  that instant -> odometry first below threshold
      standstill_to_terminal   that instant -> CANCELED status seen
    """
    def command_speed(message):
        # /cmd_vel is a bare geometry_msgs/Twist; tolerance for a stamped wrapper.
        twist = getattr(message, "twist", message)
        if twist is None:
            return None
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        if linear is None or angular is None:
            return None
        return max(abs(linear.x), abs(linear.y), abs(angular.z))

    accept = None
    terminal = None
    for receive_ns, message in status_samples:
        sim = sim_time_for(receive_ns, clock_map)
        if sim is None:
            continue
        for entry in getattr(message, "status_list", []):
            if entry.status == CANCEL_PREEMPTING and accept is None:
                accept = sim
            elif entry.status == CANCEL_CANCELED:
                terminal = sim
    last_command = None
    for receive_ns, message in command_samples:
        speed = command_speed(message)
        if speed is None or speed <= 0.0:
            continue
        sim = sim_time_for(receive_ns, clock_map)
        if sim is None:
            continue
        if accept is not None and sim < accept:
            continue
        if terminal is not None and sim > terminal:
            continue
        if last_command is None or sim > last_command:
            last_command = sim
    standstill = None
    if last_command is not None:
        for receive_ns, message in odom_samples:
            sim = sim_time_for(receive_ns, clock_map)
            if sim is None or sim < last_command:
                continue
            speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
            if speed < standstill_speed:
                standstill = sim
                break
    if accept is None or terminal is None:
        return {"state": "missing", "reason": "no PREEMPTING/CANCELED transition in the bag"}
    # The action reaches its terminal state on its own schedule: on the live cancel
    # run CANCELED was reported 0.05 s after PREEMPTING while the chassis kept
    # rolling for another 0.20 s. Report that ordering instead of a negative segment.
    after_terminal = None if standstill is None else round(standstill - terminal, 3)
    return {
        "state": "ok",
        "accept_to_zero_command_s": (None if last_command is None
                                     else round(last_command - accept, 3)),
        "zero_command_to_standstill_s": (None if last_command is None or standstill is None
                                         else round(standstill - last_command, 3)),
        "accept_to_terminal_s": round(terminal - accept, 3),
        "standstill_after_terminal_s": (None if after_terminal is None
                                        else max(after_terminal, 0.0)),
        "request_to_accept_s": None,
        "note": "the cancel request carries no ROS message; only the server-side "
                "PREEMPTING transition is observable. A positive "
                "standstill_after_terminal_s means CANCELED is reported before the "
                "chassis is physically stopped.",
    }


def action_sequences(status_samples):
    """Per-UUID action history: first/last status and the terminal status."""
    if not status_samples:
        return {"state": "missing", "goals": {}}
    goals = {}
    for receive_ns, message in status_samples:
        for entry in getattr(message, "status_list", []):
            key = bytes(entry.goal_info.goal_id.uuid).hex()
            record = goals.setdefault(key, {"statuses": [], "first_receive_ns": receive_ns})
            if not record["statuses"] or record["statuses"][-1] != entry.status:
                record["statuses"].append(entry.status)
            record["last_receive_ns"] = receive_ns
    summary = {}
    for key, record in goals.items():
        terminal = record["statuses"][-1] if record["statuses"] else None
        summary[key] = {
            "statuses": record["statuses"],
            "terminal_status": terminal,
            "terminal_name": {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}.get(terminal, None),
            "active_to_terminal_s": round(
                (record["last_receive_ns"] - record["first_receive_ns"]) / 1.0e9, 3),
        }
    return {"state": "ok", "goals": summary}


def tf_stamp_summary(tf_samples):
    """Per-transform stamps of TFMessage: there is no usable top-level header."""
    total = 0
    zero = 0
    for _receive_ns, message in tf_samples:
        for transform in message.transforms:
            total += 1
            if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
                zero += 1
    return {
        "state": "ok" if total else "missing",
        "transforms": total,
        "zero_stamped_transforms": zero,
        "note": "TFMessage has no top-level header; only per-transform stamps exist",
    }


def count_gazebo_records(text, top_level_field):
    """Count Gazebo transport messages structurally, not by non-empty text lines."""
    if not text:
        return {"state": "missing", "records": 0}
    # `ign topic -e` prints one message per block; each block repeats the top-level
    # field name at column zero.
    records = len(re.findall(r"^%s\s*\{" % re.escape(top_level_field), text, re.M))
    return {"state": "ok" if records else "missing", "records": records}


def gazebo_rtf(text):
    values = [float(value) for value in re.findall(r"real_time_factor:\s*([\d.eE+-]+)", text)]
    if not values:
        return {"state": "missing", "samples": 0}
    return {"state": "ok", "samples": len(values),
            "min": round(min(values), 4), "max": round(max(values), 4)}


def extrinsic_consistency(chassis_samples, odom_samples,
                          offset_z=BASE_TO_CHASSIS_Z):
    """Compare the ROS /odom against the Gazebo chassis truth through the extrinsic."""
    if not chassis_samples or not odom_samples:
        return {"state": "missing"}
    odom_by_stamp = {}
    for _receive_ns, message in odom_samples:
        odom_by_stamp[round(stamp_of(message), 6)] = message
    worst = 0.0
    compared = 0
    for _receive_ns, message in chassis_samples:
        match = odom_by_stamp.get(round(stamp_of(message), 6))
        if match is None:
            continue
        compared += 1
        # The base->chassis offset must be rotated by the full chassis orientation:
        # on a ramp the flat-ground shortcut "subtract offset_z from z" is wrong by
        # offset_z * sin(pitch) (about 13 mm at 10 degrees for the PB vehicle).
        orientation = (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
                       message.pose.pose.orientation.z, message.pose.pose.orientation.w)
        offset = rotate_vector(orientation, (0.0, 0.0, -offset_z))
        expected = (message.pose.pose.position.x + offset[0],
                    message.pose.pose.position.y + offset[1],
                    message.pose.pose.position.z + offset[2])
        actual = (match.pose.pose.position.x, match.pose.pose.position.y,
                  match.pose.pose.position.z)
        worst = max(worst, max(abs(expected[i] - actual[i]) for i in range(3)))
    if not compared:
        return {"state": "missing", "reason": "no /odom sample shares a stamp with the truth"}
    return {"state": "ok", "compared": compared, "worst_position_error_m": round(worst, 5),
            "tolerance_m": EXTRINSIC_POSITION_TOLERANCE,
            "ok": worst <= EXTRINSIC_POSITION_TOLERANCE}


# ------------------------------------------------------------------------ bag I/O
def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    samples = {name: [] for name in types}
    counts = {name: 0 for name in types}
    undecodable = 0
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic in SKIP_DESERIALIZE:
            continue
        try:
            samples[topic].append((receive_ns, deserialize_message(data, get_message(types[topic]))))
        except Exception:  # noqa: BLE001 - reported through undecodable
            undecodable += 1
    return samples, counts, types, undecodable


def read_joint_samples(bag_dir, clock_map=None):
    """[(sim_stamp, {joint: velocity})] from /joint_states (a dedicated pass).

    A zero header stamp falls back to the bag receive time mapped through /clock,
    so the wheel trace stays comparable with the odometry even on a bag recorded
    before the truth-stamp fix.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if "/joint_states" not in types:
        return []
    samples = []
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if topic != "/joint_states":
            continue
        message = deserialize_message(data, get_message(types[topic]))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        if stamp <= 0.0:
            stamp = sim_time_for(receive_ns, clock_map or [])
        if stamp is None:
            continue
        velocities = dict(zip(message.name, message.velocity))
        samples.append((stamp, velocities))
    return samples


def read_clock_map(bag_dir):
    """[(receive_ns, sim_seconds)] from /clock, read in a dedicated pass."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if "/clock" not in types:
        return []
    clock_map = []
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if topic != "/clock":
            continue
        message = deserialize_message(data, get_message(types[topic]))
        clock_map.append((receive_ns, message.clock.sec + message.clock.nanosec * 1.0e-9))
    return clock_map


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    arguments = parser.parse_args()

    report = arguments.report_dir
    if not report.is_dir():
        raise SystemExit("no such report directory: %s" % report)

    result = {"report_dir": str(report), "checks": []}

    def check(name, ok, detail):
        result["checks"].append({"name": name, "ok": bool(ok), "detail": detail})

    clock_samples = []
    bag_dir = report / "closed_loop"
    if bag_dir.is_dir():
        samples, counts, types, undecodable = read_bag(bag_dir)
        result["topic_counts"] = counts
        result["undecodable_messages"] = undecodable
        clock_map = read_clock_map(bag_dir)
        joint_samples = read_joint_samples(bag_dir, clock_map)
        result["clock"] = {
            "samples": len(clock_map),
            "sim_start": None if not clock_map else round(clock_map[0][1], 6),
            "sim_end": None if not clock_map else round(clock_map[-1][1], 6),
            "wall_span_s": None if len(clock_map) < 2 else round(
                (clock_map[-1][0] - clock_map[0][0]) / 1.0e9, 3),
            "sim_span_s": None if len(clock_map) < 2 else round(clock_map[-1][1] - clock_map[0][1], 3),
        }
        result["commands"] = {
            topic: command_summary(samples.get(topic, []), clock_map)
            for topic in ("/cmd_vel", "/pb/cmd_vel_safe")
        }
        result["motion"] = motion_summary(samples.get("/odom", []))
        body_samples = []
        for receive_ns, message in samples.get("/odom", []):
            stamp = stamp_of(message)
            if not stamp:
                stamp = sim_time_for(receive_ns, clock_map)
            if stamp is None:
                continue
            position = message.pose.pose.position
            body_samples.append((stamp, position.x, position.y,
                                 yaw_of(message),
                                 math.hypot(message.twist.twist.linear.x,
                                            message.twist.twist.linear.y)))
        command_marks = []
        for receive_ns, message in (samples.get("/cmd_vel", []) +
                                    samples.get("/pb/cmd_vel_safe", [])):
            sim = sim_time_for(receive_ns, clock_map)
            if sim is None:
                continue
            twist = getattr(message, "twist", message)
            command_marks.append(
                (sim, max(abs(twist.linear.x), abs(twist.linear.y), abs(twist.angular.z))))
        result["wheel_vs_body"] = wheel_vs_body(joint_samples, body_samples, command_marks)
        result["truth_motion"] = motion_summary(samples.get("/pb_sim/chassis_truth", []))
        result["actions"] = action_sequences(
            samples.get("/move_base_flex/exe_path/_action/status", []))
        result["cancel_latency"] = cancel_latency_breakdown(
            samples.get("/move_base_flex/exe_path/_action/status", []),
            samples.get("/cmd_vel", []) + samples.get("/pb/cmd_vel_safe", []),
            samples.get("/odom", []), clock_map)
        result["tf"] = tf_stamp_summary(samples.get("/tf", []))
        result["extrinsic"] = extrinsic_consistency(
            samples.get("/pb_sim/chassis_truth", []), samples.get("/odom", []))
        windows = [window for entry in result["commands"].values()
                   for window in entry.get("windows_sim", []) if window and window[1] is not None]
        last_command_sim = max((window[1] for window in windows), default=None)
        result["post_stop_drift"] = post_stop_drift(samples.get("/odom", []), last_command_sim)

        check("bag contains /clock", bool(clock_map), "%d samples" % len(clock_map))
        motion = result["motion"]
        check("odom samples present", motion.get("state") == "ok",
              "%s samples" % motion.get("count"))
        commands = result["commands"]
        check("command stream present", any(entry.get("state") == "ok"
                                            for entry in commands.values()),
              json.dumps({k: v.get("state") for k, v in commands.items()}))
        actions = result["actions"]
        terminal = [goal["terminal_name"] for goal in actions.get("goals", {}).values()]
        check("action reached a terminal state", bool(terminal) and all(terminal),
              json.dumps(actions.get("goals", {}), sort_keys=True))
        wheel_body = result["wheel_vs_body"]
        check("wheel/body contact classification available",
              wheel_body.get("state") == "ok", json.dumps(wheel_body))
        drift = result["post_stop_drift"]
        check("parking drift measured after the last command",
              drift.get("state") == "ok",
              json.dumps(drift))
        extrinsic = result["extrinsic"]
        check("gazebo truth matches ROS odom through the extrinsic",
              extrinsic.get("state") == "ok" and extrinsic.get("ok"),
              json.dumps(extrinsic))
        if arguments.goal_x is not None and motion.get("state") == "ok":
            last = motion["last_position"]
            error = math.hypot(last[0] - arguments.goal_x, last[1] - arguments.goal_y)
            result["final_goal_error_m"] = round(error, 4)
            check("final position error reported", True, "%.4f m" % error)
    else:
        check("bag directory present", False, "closed_loop/ not found")

    gz_pose = report / "gz_dynamic_pose.txt"
    gz_stats = report / "gz_stats.txt"
    gz_cmd = report / "gz_cmd_vel.txt"
    result["gazebo"] = {
        "dynamic_pose": count_gazebo_records(
            gz_pose.read_text(errors="replace") if gz_pose.is_file() else "", "pose"),
        "cmd_vel": count_gazebo_records(
            gz_cmd.read_text(errors="replace") if gz_cmd.is_file() else "", "linear"),
        "rtf": gazebo_rtf(gz_stats.read_text(errors="replace") if gz_stats.is_file() else ""),
    }
    check("gazebo native captures parsed",
          all(entry.get("state") == "ok" for entry in result["gazebo"].values()),
          json.dumps(result["gazebo"]))

    timeline = report / "timeline.txt"
    if timeline.is_file():
        result["timeline"] = dict(line.split("=", 1) for line in timeline.read_text().splitlines()
                                  if "=" in line)

    result["ok"] = all(entry["ok"] for entry in result["checks"])
    for entry in result["checks"]:
        print("%-4s %-52s %s" % ("PASS" if entry["ok"] else "FAIL", entry["name"], entry["detail"]))
    print("analysis ok:", result["ok"])
    output = report / "analysis.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True, default=str))
    print("wrote", output)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
