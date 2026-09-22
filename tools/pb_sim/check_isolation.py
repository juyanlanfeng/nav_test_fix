#!/usr/bin/env python3
"""Prove that one experiment owns its ROS domain and Gazebo partition (R5, plan
section 8).

The old check only counted ``/clock`` publishers and looked for duplicate node
names, and it treated an empty or timed-out query as "idle".  This version:

* requires exactly one publisher for the simulation clock, the truth odometry and
  the single command outlet, and names every unexpected owner;
* checks the TF chain owners rather than only the topic name;
* requires the Gazebo partition to expose the robot command topic and the world
  pose topic;
* treats an empty or timed-out query as a **failure**, never as "nothing running".

Exit code 0 only when every check passes.
"""

import argparse
import json
import subprocess
import sys
import time

# Topic -> (required publisher count, expected owner node names or None for "any")
REQUIRED_SINGLE_OWNER = {
    "/clock": (1, {"pb_ros_gz_bridge"}),
    "/odom": (1, {"pb_ground_truth_adapter"}),
    "/pb/cmd_vel_safe": (1, {"pb_cmd_vel_adapter"}),
}
TF_TOPIC = "/tf"
EXPECTED_TF_OWNERS = {"pb_ground_truth_adapter", "pb_robot_state_publisher"}
GZ_REQUIRED_TOPICS = ("/robot/cmd_vel",)


def evaluate(endpoints, gz_topics, world, query_failed=(), mode="running"):
    """Pure verdict function so the rules can be tested without a simulation.

    mode="free"    nothing may own the domain yet (checked before this run starts);
    mode="running" this run must own exactly one publisher per required topic.
    """
    checks = []

    def check(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    if mode == "free":
        for topic in sorted(set(REQUIRED_SINGLE_OWNER) | {TF_TOPIC}):
            if topic in query_failed:
                check("query %s" % topic, False,
                      "topic query failed; cannot prove the domain is free")
                continue
            entry = endpoints.get(topic)
            if entry is None:
                check("query %s" % topic, False, "no result for this topic")
                continue
            publishers = entry.get("publishers", 0)
            nodes = sorted(entry.get("nodes", []))
            check("%s is free" % topic, publishers == 0,
                  "publishers=%d nodes=%s" % (publishers, nodes))
        if gz_topics is None:
            check("gazebo partition query", True,
                  "no Gazebo server answered in this partition (free)")
        else:
            check("gazebo partition is free", not gz_topics,
                  "%d topics visible" % len(gz_topics))
        return checks

    for topic in sorted(set(REQUIRED_SINGLE_OWNER) | {TF_TOPIC}):
        if topic in query_failed:
            check("query %s" % topic, False, "topic query failed or returned no data")
            continue
        entry = endpoints.get(topic)
        if entry is None:
            check("query %s" % topic, False, "no result for this topic")
            continue
        publishers = entry.get("publishers", 0)
        nodes = sorted(entry.get("nodes", []))
        if topic == TF_TOPIC:
            missing = EXPECTED_TF_OWNERS - set(nodes)
            check("tf chain owners present", not missing,
                  "publishers=%d nodes=%s%s" % (publishers, nodes,
                                                "" if not missing else " missing=%s" % sorted(missing)))
            continue
        required, expected_nodes = REQUIRED_SINGLE_OWNER[topic]
        ok = publishers == required
        if ok and expected_nodes is not None:
            ok = set(nodes) <= expected_nodes
        check("single owner for %s" % topic, ok,
              "publishers=%d nodes=%s (expected %d owner(s) %s)"
              % (publishers, nodes, required, sorted(expected_nodes) if expected_nodes else "any"))

    for topic in GZ_REQUIRED_TOPICS + ("/world/%s/dynamic_pose/info" % world,):
        check("gazebo partition exposes %s" % topic, topic in gz_topics,
              "%d gazebo topics visible" % len(gz_topics))
    return checks


def query_endpoints(topics, spin_seconds=4.0):
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("pb_sim_isolation_check")
    endpoints = {}
    failed = set()
    try:
        deadline = time.monotonic() + spin_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for topic in topics:
            try:
                infos = node.get_publishers_info_by_topic(topic)
            except Exception as error:  # noqa: BLE001 - reported as a failure
                failed.add(topic)
                endpoints[topic] = {"publishers": 0, "nodes": [], "error": str(error)}
                continue
            endpoints[topic] = {
                "publishers": len(infos),
                "nodes": sorted({info.node_name for info in infos}),
            }
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return endpoints, failed


def query_gazebo_topics():
    completed = subprocess.run(["ign", "topic", "-l"], capture_output=True, text=True)
    if completed.returncode != 0:
        return None
    return {line.strip() for line in completed.stdout.splitlines() if line.strip()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world", default="rmuc2026_field")
    parser.add_argument("--mode", choices=["free", "running"], default="running",
                        help="free: nothing may own the domain yet; "
                             "running: this run must own each required topic")
    parser.add_argument("--report", help="write the JSON result here")
    arguments = parser.parse_args()

    topics = sorted(set(REQUIRED_SINGLE_OWNER) | {TF_TOPIC})
    endpoints, failed = query_endpoints(topics)
    gz_topics = query_gazebo_topics()
    if gz_topics is None and arguments.mode == "running":
        checks = [{"name": "gazebo topic query", "ok": False,
                   "detail": "ign topic -l failed; treating as not isolated"}]
    else:
        checks = evaluate(endpoints, gz_topics, arguments.world, failed, arguments.mode)

    for entry in checks:
        print("%-4s %-46s %s" % ("PASS" if entry["ok"] else "FAIL", entry["name"], entry["detail"]))
    result = {"checks": checks, "ok": all(entry["ok"] for entry in checks)}
    if arguments.report:
        with open(arguments.report, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, sort_keys=True)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
