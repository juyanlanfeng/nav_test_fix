"""Read-only preflight checks for the PB three-framework simulation.

    ros2 run pb_vehicle_adapter pb_preflight --framework meshnav
    ros2 run pb_vehicle_adapter pb_preflight --framework jie
    ros2 run pb_vehicle_adapter pb_preflight --framework dddmr

Exits 0 only when every check for the requested framework passes; otherwise it
prints each failing item and exits non-zero.  It never publishes velocity or
goals, so it is safe to run against a live simulation.

`--wait-timeout S` turns the same checks into the launch-time readiness gate:
it re-runs them until they all pass or S seconds elapse, then prints the last
report (so a timeout still names the missing conditions).
"""

import argparse
import sys
import time
from pathlib import Path

from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.node import Node
import tf2_ros


DDDMR_MAP_DIR = Path("/home/rainple/nav_test/field/converted_rmuc2026/dddmr_nav")

FRAMEWORKS = {
    "meshnav": {
        "topics": ["/odom", "/tf"],
        "actions": ["/move_base_flex/get_path", "/move_base_flex/exe_path"],
        "nodes": ["move_base_flex"],
        "control_source": "meshnav",
        "velocity_topic": "/cmd_vel",
    },
    "jie": {
        "topics": ["/odom", "/tf", "/planned_path"],
        "actions": [],
        "nodes": ["jie_path_node", "d1_controller"],
        "control_source": "jie",
        "velocity_topic": "/cmd_vel_jie",
    },
    "dddmr": {
        "topics": ["/odom", "/tf", "/dddmr/mapcloud", "/dddmr/mapground"],
        "actions": ["/p2p_move_base"],
        # These are the node names the two DDDMR executables really create
        # (global_planner_node and p2p_move_base_node); there is no node called
        # p2p_move_base_node.
        "nodes": ["global_planner", "perception_3d_global", "p2p_move_base", "perception_3d_local"],
        "control_source": "dddmr",
        "velocity_topic": "/pb/dddmr_cmd_vel_stamped",
        "map_files": [
            DDDMR_MAP_DIR / "rmuc2026_mapcloud.pcd",
            DDDMR_MAP_DIR / "rmuc2026_mapground.pcd",
            DDDMR_MAP_DIR / "map_manifest.yaml",
        ],
    },
}


class Preflight(Node):
    def __init__(self):
        super().__init__("pb_preflight")
        self._clock_seen = []
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _on_clock(self, msg):
        self._clock_seen.append(msg.clock.sec + msg.clock.nanosec * 1e-9)
        del self._clock_seen[:-4]

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def check_clock(self):
        """Return (ok, detail): /clock advances by more than zero."""
        self.spin_for(2.0)
        if len(self._clock_seen) < 2:
            return False, "no advancing /clock received"
        if self._clock_seen[-1] <= self._clock_seen[0]:
            return False, "sim time is not advancing (paused?)"
        return True, "clock advances (%.3f -> %.3f)" % (self._clock_seen[0], self._clock_seen[-1])

    def check_tf(self, parent="map", child="base_footprint"):
        deadline = time.monotonic() + 5.0
        last_error = "not published"
        while time.monotonic() < deadline:
            try:
                self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                return True, "%s -> %s available" % (parent, child)
            except Exception as exc:  # noqa: BLE001 - report the transform error text
                last_error = str(exc).splitlines()[0]
            rclpy.spin_once(self, timeout_sec=0.1)
        return False, "%s -> %s unavailable: %s" % (parent, child, last_error)

    def check_topics(self, topics):
        missing = []
        for topic in topics:
            if self.count_publishers(topic) == 0:
                missing.append(topic)
        if missing:
            return False, "no publisher on: %s" % ", ".join(missing)
        return True, "all topics published: %s" % ", ".join(topics)

    def check_nodes(self, names):
        live = {name.lstrip("/") for name in self.get_node_names()}
        missing = [name for name in names if name not in live]
        if missing:
            return False, "missing nodes: %s" % ", ".join(missing)
        return True, "nodes present: %s" % ", ".join(names)

    def check_actions(self, actions):
        """Check action servers through the ROS graph.

        Deliberately not `ros2 action list` as a subprocess: running the CLI from
        a process that already owns a CycloneDDS domain fails with "failed to
        create domain", so the graph API is the reliable path.
        """
        if not actions:
            return True, "no action required for this framework"
        listed = set()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            listed = {name for name, _types in get_action_names_and_types(self)}
            if all(action in listed for action in actions):
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        missing = [action for action in actions if action not in listed]
        if missing:
            return False, "missing actions: %s" % ", ".join(missing)
        return True, "actions present: %s" % ", ".join(actions)

    def check_control_source(self, expected):
        """Read /pb_cmd_vel_adapter's control_source through its param service."""
        client = self.create_client(GetParameters, "/pb_cmd_vel_adapter/get_parameters")
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "velocity adapter not reachable (is the simulation running?)"
        request = GetParameters.Request()
        request.names = ["control_source"]
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return False, "velocity adapter did not answer get_parameters"
        values = future.result().values
        if not values:
            return False, "velocity adapter returned no value for control_source"
        value = values[0].string_value
        if value != expected:
            return False, "control_source is not '%s' (got: '%s')" % (expected, value)
        return True, "velocity source is '%s'" % expected

    @staticmethod
    def check_map_files(files):
        missing = [str(path) for path in files if not path.is_file()]
        if missing:
            return False, "missing DDDMR map artifacts: %s" % ", ".join(missing)
        return True, "DDDMR map artifacts present"


def collect_results(node, spec, tf_parent, tf_child):
    """Run every check once and return [(name, ok, detail), ...]."""
    results = [
        ("clock",) + node.check_clock(),
        ("tf",) + node.check_tf(tf_parent, tf_child),
        ("topics",) + node.check_topics(spec["topics"]),
        ("nodes",) + node.check_nodes(spec["nodes"]),
        ("actions",) + node.check_actions(spec["actions"]),
        ("velocity-source",) + node.check_control_source(spec["control_source"]),
    ]
    if "map_files" in spec:
        results.append(("map-artifacts",) + node.check_map_files(spec["map_files"]))
    return results


def wait_for_ready(node, spec, tf_parent, tf_child, wait_timeout, poll_interval=2.0,
                   monotonic=time.monotonic, sleep=time.sleep, log=print):
    """Repeat the checks until they all pass or `wait_timeout` seconds elapse.

    `wait_timeout <= 0` performs a single pass, which is the plain preflight
    behaviour.  This is the readiness gate used by the launch files instead of a
    fixed sleep; every iteration re-runs the real checks, and the returned
    results are the last ones, so a timeout still lists what is missing.
    """
    deadline = monotonic() + max(0.0, wait_timeout)
    attempt = 0
    while True:
        attempt += 1
        results = collect_results(node, spec, tf_parent, tf_child)
        failed = [name for name, ok, _ in results if not ok]
        if not failed:
            return results
        remaining = deadline - monotonic()
        if remaining <= 0:
            return results
        log("pb_preflight: attempt %d not ready yet (%s); retrying for %.0fs"
            % (attempt, ", ".join(failed), remaining))
        sleep(min(poll_interval, remaining))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--framework", required=True, choices=sorted(FRAMEWORKS))
    parser.add_argument("--tf-parent", default="map")
    parser.add_argument("--tf-child", default="base_footprint")
    parser.add_argument(
        "--wait-timeout", type=float, default=0.0,
        help="wait up to this many seconds for all checks to pass (0 = check once)",
    )
    parser.add_argument("--poll-interval", type=float, default=2.0)
    args = parser.parse_args()

    spec = FRAMEWORKS[args.framework]
    rclpy.init()
    node = Preflight()
    try:
        results = wait_for_ready(
            node, spec, args.tf_parent, args.tf_child,
            args.wait_timeout, args.poll_interval,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    failed = 0
    print("pb_preflight --framework %s" % args.framework)
    for name, ok, detail in results:
        print("  [%s] %-16s %s" % ("PASS" if ok else "FAIL", name, detail))
        failed += 0 if ok else 1
    print("%d/%d checks passed" % (len(results) - failed, len(results)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
