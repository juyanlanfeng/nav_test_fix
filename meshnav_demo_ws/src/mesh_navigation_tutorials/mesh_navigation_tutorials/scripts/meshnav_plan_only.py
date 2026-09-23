#!/usr/bin/env python3
"""Request one MeshNav path from explicit poses and publish it for RViz.

This client calls GetPath only. It never sends ExePath and therefore needs no
robot, odometry, localization, controller, or simulator.
"""

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from mbf_msgs.action import GetPath
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def pose(frame, x, y, z, yaw, stamp):
    result = PoseStamped()
    result.header.frame_id = frame
    result.header.stamp = stamp
    result.pose.position.x = x
    result.pose.position.y = y
    result.pose.position.z = z
    result.pose.orientation.z = math.sin(yaw * 0.5)
    result.pose.orientation.w = math.cos(yaw * 0.5)
    return result


class PlanOnly(Node):
    def __init__(self, arguments):
        super().__init__("meshnav_plan_only")
        self.arguments = arguments
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, arguments.path_topic, qos)
        self.client = ActionClient(self, GetPath, arguments.action)
        self.path = None

    def request(self):
        if not self.client.wait_for_server(timeout_sec=self.arguments.server_timeout):
            raise RuntimeError(f"GetPath action is unavailable: {self.arguments.action}")
        stamp = self.get_clock().now().to_msg()
        goal = GetPath.Goal()
        goal.use_start_pose = True
        goal.start_pose = pose(
            self.arguments.frame, self.arguments.start[0], self.arguments.start[1],
            self.arguments.start[2], self.arguments.start_yaw, stamp,
        )
        goal.target_pose = pose(
            self.arguments.frame, self.arguments.goal[0], self.arguments.goal[1],
            self.arguments.goal[2], self.arguments.goal_yaw, stamp,
        )
        goal.tolerance = self.arguments.tolerance
        goal.planner = self.arguments.planner
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.arguments.plan_timeout)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError("GetPath goal was rejected or acceptance timed out")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.arguments.plan_timeout
        )
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("GetPath result timed out")
        result = result_future.result().result
        if result.outcome != GetPath.Result.SUCCESS or not result.path.poses:
            raise RuntimeError(
                f"planning failed: outcome={result.outcome} message={result.message!r}"
            )
        self.path = result.path
        # Replace a zero/simulation stamp so wall-time RViz accepts the message.
        now = self.get_clock().now().to_msg()
        self.path.header.stamp = now
        for item in self.path.poses:
            item.header.stamp = now
        self.path_publisher.publish(self.path)
        print(
            f"PATH_READY poses={len(self.path.poses)} cost={result.cost:.6f} "
            f"topic={self.arguments.path_topic}",
            flush=True,
        )

    def hold(self):
        started = time.monotonic()
        while rclpy.ok():
            if self.arguments.hold_seconds >= 0:
                if time.monotonic() - started >= self.arguments.hold_seconds:
                    return
            stamp = self.get_clock().now().to_msg()
            self.path.header.stamp = stamp
            for item in self.path.poses:
                item.header.stamp = stamp
            self.path_publisher.publish(self.path)
            rclpy.spin_once(self, timeout_sec=1.0)


def parse_arguments(values=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--goal", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--start-yaw", type=float, default=0.0, help="radians")
    parser.add_argument("--goal-yaw", type=float, default=0.0, help="radians")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--planner", default="mesh_planner")
    parser.add_argument("--tolerance", type=float, default=0.1)
    parser.add_argument("--action", default="/move_base_flex/get_path")
    parser.add_argument("--path-topic", default="/move_base_flex/path")
    parser.add_argument("--server-timeout", type=float, default=120.0)
    parser.add_argument("--plan-timeout", type=float, default=120.0)
    parser.add_argument(
        "--hold-seconds", type=float, default=-1.0,
        help="keep republishing for N seconds; negative means until Ctrl+C",
    )
    arguments = parser.parse_args(values)
    numbers = (*arguments.start, *arguments.goal, arguments.start_yaw, arguments.goal_yaw)
    if not all(math.isfinite(value) for value in numbers):
        parser.error("poses must contain finite numbers")
    return arguments


def main(values=None):
    arguments = parse_arguments(values)
    rclpy.init()
    node = PlanOnly(arguments)
    try:
        node.request()
        node.hold()
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
