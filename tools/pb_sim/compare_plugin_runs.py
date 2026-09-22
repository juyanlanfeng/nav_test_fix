#!/usr/bin/env python3
"""Matched-control comparison of two closed-loop runs (plan P2).

The plan asks for a same-condition comparison of the tracking layer before any fix:
same map, same start, same goal, one run per controller plugin.  This tool reduces a
pair of report directories to the quantities that decide *where* a divergence comes
from, so the conclusion does not depend on anyone's memory of two runs:

* the plan the planner produced (pose count, start/goal, first-segment direction,
  a hash of the quantised polyline) -- if these differ, the planner is implicated;
* the first-segment direction **in the robot's actual body frame** at the plan start,
  which is the lateral-offset figure the tracking layer reacts to.  This is computed
  from the recorded chassis pose, not from the plan's own start orientation, which
  would be a tautology;
* the motion that actually happened (span, net displacement, final goal error);
* how each run's first seconds split between rotation and translation.

Usage:
  tools/pb_sim/compare_plugin_runs.py --run label=log/<report> [--run label=log/<other>]
      [--out log/p2_plugin_comparison/comparison.json]
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as error:  # pragma: no cover - environment guard
    print("ROS 2 Python packages unavailable: %s" % error, file=sys.stderr)
    raise SystemExit(2)

TRUTH_TOPIC = "/pb_sim/chassis_truth"
PATH_TOPIC = "/move_base_flex/path"
COMMAND_TOPICS = ("/pb/cmd_vel_safe", "/cmd_vel")
PURE_ROTATION_LINEAR = 0.01
PURE_ROTATION_ANGULAR = 0.01


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def load_run(report):
    """Reduce one report directory to the comparison inputs."""
    bag = Path(report) / "closed_loop"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    truth, commands, plans = [], [], []
    while reader.has_next():
        topic, data, receive = reader.read_next()
        if topic == TRUTH_TOPIC:
            message = deserialize_message(data, get_message(types[topic]))
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
            truth.append((stamp, message.pose.pose.position.x, message.pose.pose.position.y,
                          yaw_from_quaternion(message.pose.pose.orientation)))
        elif topic in COMMAND_TOPICS:
            message = deserialize_message(data, get_message(types[topic]))
            twist = getattr(message, "twist", message)
            commands.append((receive, twist.linear.x, twist.linear.y, twist.angular.z))
        elif topic == PATH_TOPIC:
            message = deserialize_message(data, get_message(types[topic]))
            plans.append(message.poses)
    if not truth or not plans:
        raise SystemExit("FAIL %s lacks %s or %s" % (report, TRUTH_TOPIC, PATH_TOPIC))

    poses = max(plans, key=len)
    points = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]
    digest = hashlib.sha256(json.dumps([[round(v, 6) for v in p] for p in points]).encode())
    # Two runs of the same planner differ in the last ulps of the same solution, so
    # the exact hash is reported for traceability while equality is judged on
    # millimetre-scale agreement of the polyline.
    rounded = [[round(v, 4) for v in p] for p in points]
    start = truth[0]
    goal = points[-1]
    first = points[1] if len(points) > 1 else points[0]
    world = math.atan2(first[1] - points[0][1], first[0] - points[0][0])
    commands.sort(key=lambda entry: entry[0])
    return {
        "report": str(report),
        "bag": str(bag),
        "plan": {
            "poses": len(points),
            "stride_m": round(math.hypot(first[0] - points[0][0], first[1] - points[0][1]), 6),
            "start": [round(v, 6) for v in points[0]],
            "goal": [round(v, 6) for v in goal],
            "z_range": [round(min(p[2] for p in points), 6),
                        round(max(p[2] for p in points), 6)],
            "first_segment_world_rad": round(world, 6),
            "polyline_sha256": digest.hexdigest(),
            "polyline_rounded_mm": rounded,
        },
        "robot_start": {
            "x": round(start[1], 4), "y": round(start[2], 4), "yaw_rad": round(start[3], 4),
        },
        "first_segment_in_body_rad": round(normalize(world - start[3]), 6),
        "first_segment_off_forward_deg": round(math.degrees(abs(normalize(world - start[3]))), 2),
        "motion": {
            "span_s": round(truth[-1][0] - truth[0][0], 3),
            "net_displacement_m": round(math.hypot(truth[-1][1] - start[1],
                                                   truth[-1][2] - start[2]), 4),
            "final_goal_error_m": round(math.hypot(truth[-1][1] - goal[0],
                                                   truth[-1][2] - goal[1]), 4),
        },
        "commands": command_split(commands),
    }


def command_split(commands, window_s=12.0):
    """Rotation/translation split over the first ``window_s`` seconds of commands."""
    if not commands:
        return {"state": "missing"}
    base = commands[0][0]
    early = [entry for entry in commands if (entry[0] - base) / 1.0e9 <= window_s]
    nonzero = [entry for entry in early
               if max(abs(entry[1]), abs(entry[2]), abs(entry[3])) > 1.0e-6]
    if not nonzero:
        return {"state": "idle", "samples": len(early)}
    rotation = sum(1 for entry in nonzero
                   if math.hypot(entry[1], entry[2]) < PURE_ROTATION_LINEAR
                   and abs(entry[3]) > PURE_ROTATION_ANGULAR)
    return {
        "state": "ok",
        "window_s": window_s,
        "samples": len(early),
        "nonzero": len(nonzero),
        "mean_linear_m_s": round(sum(math.hypot(e[1], e[2]) for e in nonzero)
                                 / len(nonzero), 4),
        "mean_angular_rad_s": round(sum(abs(e[3]) for e in nonzero) / len(nonzero), 4),
        "pure_rotation": rotation,
        "pure_rotation_fraction": round(rotation / len(nonzero), 4),
        "first_nonzero": [round(nonzero[0][1], 4), round(nonzero[0][2], 4),
                          round(nonzero[0][3], 4)],
    }


def compare(runs):
    """Compare loaded runs and state plainly which layer the evidence implicates."""
    result = {"runs": runs}
    plans = [run["plan"] for run in runs]
    starts = [run["robot_start"] for run in runs]
    deviations = []
    for plan in plans[1:]:
        if plan["poses"] != plans[0]["poses"]:
            deviations.append(float("inf"))
            continue
        deviations.append(max(math.dist(a, b) for a, b in
                              zip(plan["polyline_rounded_mm"],
                                  plans[0]["polyline_rounded_mm"])))
    result["max_plan_deviation_m"] = [round(value, 9) for value in deviations]
    result["plan_tolerance_m"] = 0.001
    same_direction = all(plan["first_segment_world_rad"] ==
                         plans[0]["first_segment_world_rad"] for plan in plans)
    shapes_agree = same_direction and all(value <= result["plan_tolerance_m"]
                                          for value in deviations)
    starts_agree = all(start == starts[0] for start in starts)
    result["same_plan"] = shapes_agree
    result["exact_polyline_hashes_equal"] = all(plan["polyline_sha256"] ==
                                                plans[0]["polyline_sha256"] for plan in plans)
    result["same_robot_start"] = starts_agree
    errors = [run["motion"]["final_goal_error_m"] for run in runs]
    displacements = [run["motion"]["net_displacement_m"] for run in runs]
    result["outcomes_diverge"] = bool(errors) and (max(errors) - min(errors)) > 0.2
    if shapes_agree and starts_agree and result["outcomes_diverge"]:
        result["conclusion"] = (
            "identical plan and identical start pose with divergent outcomes: the "
            "planner, map and start state are excluded, the difference is inside the "
            "controller/tracking layer")
    elif not shapes_agree:
        result["conclusion"] = ("the runs did not receive the same plan: the planner or "
                                "its inputs differ, so the controller comparison is invalid")
    else:
        result["conclusion"] = ("plan and start agree but the outcomes do not diverge "
                                "beyond 0.2 m: no controller difference is demonstrated")
    result["final_goal_error_m"] = errors
    result["net_displacement_m"] = displacements
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", action="append", required=True,
                        help="label=report_dir (repeat for each run to compare)")
    parser.add_argument("--out", default=None, help="write the comparison JSON here")
    arguments = parser.parse_args(argv)

    runs = []
    for entry in arguments.run:
        if "=" not in entry:
            raise SystemExit("FAIL --run wants label=report_dir, got %r" % entry)
        label, report = entry.split("=", 1)
        loaded = load_run(report)
        loaded["label"] = label
        runs.append(loaded)
    result = compare(runs)
    print(json.dumps(result, indent=2, sort_keys=True))
    if arguments.out:
        out = Path(arguments.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        print("wrote %s" % out)
    return 0 if result["same_plan"] and result["same_robot_start"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
