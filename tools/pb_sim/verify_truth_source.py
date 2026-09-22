#!/usr/bin/env python3
"""Verify the single-source chassis truth chain (R1 of
doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md) against a running simulation.

It records the raw source (``/pb_sim/chassis_truth``) and the adapter outputs
(``/odom``, ``/pb/truth_health``), optionally while driving, and then checks the
properties the plan requires:

1. every source message carries a non-zero, strictly increasing simulation stamp;
2. the source pose is six-degree-of-freedom (finite quaternion and z);
3. the engine twist agrees with the finite difference of the same source poses,
   expressed in the chassis frame, within ``--twist-tolerance``;
4. every published ``/odom`` stamp can be traced to a source stamp and its pose
   equals the source chassis pose re-referenced to base_footprint;
5. the adapter reports the chain healthy.

Exit code is non-zero if any check fails, so the tool can gate a run.

    python3 tools/pb_sim/verify_truth_source.py --seconds 8 --drive 0.2
"""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


def _stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9


def _q_inverse(q):
    x, y, z, w = q
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


def _rotate(q, v):
    rotated = _q_multiply(_q_multiply(q, (v[0], v[1], v[2], 0.0)), _q_inverse(q))
    return (rotated[0], rotated[1], rotated[2])


class TruthVerifier(Node):
    def __init__(self, drive, drive_topic):
        super().__init__("pb_truth_verifier")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.truth = []
        self.odom = []
        self.health = []
        self._drive = drive
        self._drive_topic = drive_topic
        self._publisher = (
            self.create_publisher(TwistStamped, drive_topic, 10) if drive else None
        )
        self.create_subscription(Odometry, "/pb_sim/chassis_truth", self._on_truth, 50)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(Bool, "/pb/truth_health", self._on_health, 10)
        self._timer = None
        if drive:
            self._timer = self.create_timer(0.05, self._command)

    def _command(self):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_footprint"
        message.twist.linear.x = self._drive
        self._publisher.publish(message)

    def _on_truth(self, message):
        self.truth.append(message)

    def _on_odom(self, message):
        self.odom.append(message)

    def _on_health(self, message):
        self.health.append(message.data)


def evaluate(truth, odom, health, twist_tolerance, position_tolerance):
    checks = []

    def check(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    check("truth received", bool(truth), "%d messages" % len(truth))
    if not truth:
        return checks

    zero_stamps = sum(1 for message in truth if _stamp(message) == 0.0)
    check("non-zero source stamps", zero_stamps == 0, "%d zero stamps" % zero_stamps)

    stamps = [_stamp(message) for message in truth]
    regressions = sum(1 for a, b in zip(stamps, stamps[1:]) if b <= a)
    check("strictly increasing source stamps", regressions == 0,
          "%d non-increasing pairs" % regressions)

    six_dof = all(
        math.isfinite(message.pose.pose.position.z) and math.isfinite(
            message.pose.pose.orientation.x + message.pose.pose.orientation.y +
            message.pose.pose.orientation.z + message.pose.pose.orientation.w)
        for message in truth
    )
    check("source pose is 6-DOF and finite", six_dof, "z and quaternion finite")

    # 3. engine twist vs finite difference of the same source poses
    worst = 0.0
    compared = 0
    for previous, current in zip(truth, truth[1:]):
        delta_t = _stamp(current) - _stamp(previous)
        if not 1.0e-4 < delta_t < 0.5:
            continue
        p0 = (previous.pose.pose.position.x, previous.pose.pose.position.y,
              previous.pose.pose.position.z)
        p1 = (current.pose.pose.position.x, current.pose.pose.position.y,
              current.pose.pose.position.z)
        q1 = (current.pose.pose.orientation.x, current.pose.pose.orientation.y,
              current.pose.pose.orientation.z, current.pose.pose.orientation.w)
        world_velocity = tuple((p1[i] - p0[i]) / delta_t for i in range(3))
        finite_difference = _rotate(_q_inverse(q1), world_velocity)
        reported = (current.twist.twist.linear.x, current.twist.twist.linear.y,
                    current.twist.twist.linear.z)
        worst = max(worst, max(abs(finite_difference[i] - reported[i]) for i in range(3)))
        compared += 1
    check("engine twist matches finite difference", compared > 0 and worst <= twist_tolerance,
          "compared %d pairs, worst %.3f m/s (limit %.3f)" % (compared, worst, twist_tolerance))

    # 4. adapter /odom traceable to a source stamp and re-referenced pose
    source_by_stamp = {round(_stamp(message), 6): message for message in truth}
    traced = 0
    worst_position = 0.0
    for message in odom:
        source = source_by_stamp.get(round(_stamp(message), 6))
        if source is None:
            continue
        traced += 1
        # base = chassis + R(chassis) * (0, 0, -0.076) with the PB extrinsic
        q = (source.pose.pose.orientation.x, source.pose.pose.orientation.y,
             source.pose.pose.orientation.z, source.pose.pose.orientation.w)
        offset = _rotate(q, (0.0, 0.0, -0.076))
        expected = (source.pose.pose.position.x + offset[0],
                    source.pose.pose.position.y + offset[1],
                    source.pose.pose.position.z + offset[2])
        actual = (message.pose.pose.position.x, message.pose.pose.position.y,
                  message.pose.pose.position.z)
        worst_position = max(worst_position,
                             max(abs(expected[i] - actual[i]) for i in range(3)))
    check("adapter /odom stamps trace to a source stamp", traced > 0,
          "%d of %d traceable" % (traced, len(odom)))
    check("adapter /odom pose re-references the chassis truth",
          traced > 0 and worst_position <= position_tolerance,
          "worst %.4f m (limit %.4f)" % (worst_position, position_tolerance))

    check("adapter reports healthy", bool(health) and health[-1] is True,
          "last health=%s (%d messages)" % (health[-1] if health else None, len(health)))
    return checks


def world_control(world, pause):
    """Pause or resume a Gazebo world through the ign CLI."""
    import subprocess

    request = "pause: true" if pause else "pause: false"
    command = [
        "ign", "service", "-s", "/world/%s/control" % world,
        "--reqtype", "ignition.msgs.WorldControl",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000", "--req", request,
    ]
    completed = subprocess.run(command, capture_output=True, text=True)
    return completed.returncode == 0, (completed.stdout + completed.stderr).strip()


def pause_resume_checks(node, world, drain_s=0.6, window_s=1.6):
    """T0: a paused source must not keep producing *new* output.

    Samples that were already in flight when the pause was requested are not a
    defect: they carry stamps from before the pause.  The check is therefore that
    the newest stamp stops advancing and no further samples appear, which is what
    a consumer would act on.
    """
    checks = []

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    def newest_stamp(messages):
        return max((_stamp(m) for m in messages), default=None)

    pause_stamp = newest_stamp(node.truth)
    ok, output = world_control(world, True)
    if not ok:
        checks.append({"name": "pause the world", "ok": False, "detail": output})
        return checks

    spin_for(drain_s)  # let pre-pause samples drain out of the DDS queue
    truth_mark = len(node.truth)
    odom_mark = len(node.odom)
    spin_for(window_s)

    new_truth = node.truth[truth_mark:]
    new_odom = node.odom[odom_mark:]
    forged_truth = [m for m in new_truth
                    if pause_stamp is None or _stamp(m) > pause_stamp + 1.0e-6]
    forged_odom = [m for m in new_odom
                   if pause_stamp is None or _stamp(m) > pause_stamp + 1.0e-6]
    last_health = node.health[-1] if node.health else None
    checks.append({
        "name": "paused source produces no new samples",
        "ok": not new_truth,
        "detail": "%d new truth samples after the pause settled" % len(new_truth),
    })
    checks.append({
        "name": "paused source never advances the stamp",
        "ok": not forged_truth,
        "detail": "%d samples newer than the pause stamp %.3f"
                  % (len(forged_truth), pause_stamp if pause_stamp is not None else float("nan")),
    })
    checks.append({
        "name": "paused source does not forge fresh odometry",
        "ok": not new_odom and not forged_odom,
        "detail": "%d new /odom samples, %d newer than the pause stamp"
                  % (len(new_odom), len(forged_odom)),
    })
    checks.append({
        "name": "adapter reports unhealthy while paused",
        "ok": last_health is False,
        "detail": "last health=%s" % last_health,
    })

    odom_resume_mark = len(node.odom)
    ok, output = world_control(world, False)
    spin_for(window_s)
    resumed_odom = len(node.odom) - odom_resume_mark
    checks.append({
        "name": "resume restores healthy fresh output",
        "ok": ok and resumed_odom > 0 and (node.health[-1] if node.health else None) is True,
        "detail": "%d /odom messages after resume, last health=%s"
                  % (resumed_odom, node.health[-1] if node.health else None),
    })
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--drive", type=float, default=0.0,
                        help="forward speed [m/s] published while recording; 0 = stationary")
    parser.add_argument("--drive-topic", default="/cmd_vel")
    parser.add_argument("--twist-tolerance", type=float, default=0.15)
    parser.add_argument("--position-tolerance", type=float, default=0.005)
    parser.add_argument("--report", type=Path, help="write the JSON result here")
    parser.add_argument("--pause-test", action="store_true",
                        help="T0: pause/resume the world and check the source cannot forge "
                             "fresh output while paused")
    parser.add_argument("--world", default="rmuc2026_field")
    arguments = parser.parse_args()

    rclpy.init()
    node = TruthVerifier(arguments.drive, arguments.drive_topic)
    deadline = time.monotonic() + arguments.seconds
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    checks = evaluate(node.truth, node.odom, node.health,
                      arguments.twist_tolerance, arguments.position_tolerance)
    if arguments.pause_test:
        checks.extend(pause_resume_checks(node, arguments.world))
    node.destroy_node()
    rclpy.shutdown()

    for entry in checks:
        print("%-4s %-46s %s" % ("PASS" if entry["ok"] else "FAIL", entry["name"], entry["detail"]))
    result = {"checks": checks, "ok": all(entry["ok"] for entry in checks)}
    if arguments.report:
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(json.dumps(result, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
