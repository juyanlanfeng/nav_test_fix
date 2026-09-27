"""Coordinate PB Mesh Goal requests with periodic GetPath / ExePath updates.

All action callbacks run on one executor. New paths use MBF's same-controller
preemption (setNewPlan); canceling execution before every replan would stop the
robot and reset MPPI. Epoch and sequence numbers discard superseded results.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mbf_msgs.action import ExePath, GetPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MeshnavNavigator(Node):
    def __init__(self):
        super().__init__("meshnav_navigator")
        defaults = {
            "goal_topic": "/rviz/goal_pose",
            "get_path_action": "/move_base_flex/get_path",
            "exe_path_action": "/move_base_flex/exe_path",
            "planner": "mesh_planner",
            "controller": "mesh_controller",
            "planner_frequency": 2.0,
            "replan_stop_distance": 0.4,
            "action_timeout": 15.0,
        }
        self.cfg = {key: self.declare_parameter(key, value).value
                    for key, value in defaults.items()}
        if self.cfg["planner_frequency"] < 0 or self.cfg["action_timeout"] <= 0:
            raise ValueError("planner_frequency must be >= 0 and action_timeout > 0")
        self.planner = ActionClient(self, GetPath, self.cfg["get_path_action"])
        self.controller = ActionClient(self, ExePath, self.cfg["exe_path_action"])
        self.status = self.create_publisher(
            String, "~/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, self.cfg["goal_topic"], self.new_goal, 1)
        self.create_service(Trigger, "~/cancel", self.cancel)
        self.epoch = 0
        self.sequence = 0
        self.executions = {}
        self.pending_sends = 0
        self.plan_busy = False
        self.plan_handle = None
        self.target = None
        self.queued_target = None
        self.distance = math.inf
        self.plan_count = 0
        self.last_plan = 0.0
        self.request_started = 0.0
        self.stop_started = None
        self.stop_state = None
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status("idle")

    def publish_status(self, state, **fields):
        text = json.dumps(dict(state=state, plans=self.plan_count, **fields), ensure_ascii=False)
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def stop(self, state, **fields):
        self.epoch += 1
        self.target = None
        self.stop_started = time.monotonic()
        self.stop_state = state
        if self.plan_handle is not None:
            self.plan_handle.cancel_goal_async()
        for handle in list(self.executions.values()):
            handle.cancel_goal_async()
        self.publish_status(state, **fields)

    def new_goal(self, pose):
        values = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                  pose.pose.orientation.x, pose.pose.orientation.y,
                  pose.pose.orientation.z, pose.pose.orientation.w)
        if not pose.header.frame_id or not all(math.isfinite(v) for v in values):
            self.get_logger().error("Rejected goal: missing frame or non-finite pose")
            return
        if sum(v * v for v in values[3:]) < 1e-6:
            self.get_logger().error("Rejected goal: zero quaternion")
            return
        self.queued_target = pose
        self.stop("replacing_goal")

    def cancel(self, request, response):
        self.queued_target = None
        self.stop("cancel_requested")
        response.success = True
        response.message = "Cancellation requested; see /meshnav_navigator/status for completion"
        return response

    def tick(self):
        busy = self.plan_busy or self.pending_sends or bool(self.executions)
        if self.target is None:
            if busy:
                if self.stop_started and time.monotonic() - self.stop_started > self.cfg["action_timeout"]:
                    self.publish_status("cancel_timeout", message="Waiting for MBF to finish cancellation")
                    self.stop_started = time.monotonic()
                return
            if self.stop_started is not None:
                self.stop_started = None
                if self.stop_state == "cancel_requested":
                    self.publish_status("canceled")
            if self.queued_target is None:
                return
            if not self.planner.server_is_ready() or not self.controller.server_is_ready():
                return
            self.target, self.queued_target = self.queued_target, None
            self.distance = math.inf
            self.plan_count = 0
            self.publish_status("planning", x=self.target.pose.position.x, y=self.target.pose.position.y)
        if self.plan_busy or self.pending_sends:
            if time.monotonic() - self.request_started > self.cfg["action_timeout"]:
                self.stop("failed", message="Planning or goal acceptance timed out")
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        frequency = self.cfg["planner_frequency"]
        if self.plan_count:
            if frequency == 0 or self.distance <= self.cfg["replan_stop_distance"]:
                return
            if now - self.last_plan < 1.0 / frequency:
                return
        self.request_plan(now)

    def request_plan(self, now):
        goal = GetPath.Goal()
        goal.target_pose = self.target
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.use_start_pose = False
        goal.planner = self.cfg["planner"]
        self.plan_busy = True
        self.last_plan = now
        self.request_started = time.monotonic()
        epoch = self.epoch
        self.planner.send_goal_async(goal).add_done_callback(
            lambda future: self.plan_accepted(future, epoch))

    def plan_accepted(self, future, epoch):
        try:
            handle = future.result()
        except Exception as exc:
            self.plan_busy = False
            if epoch == self.epoch:
                self.stop("failed", message=str(exc))
            return
        if not handle.accepted:
            self.plan_busy = False
            if epoch == self.epoch:
                self.stop("failed", message="Planner rejected goal")
            return
        self.plan_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.plan_done(f, epoch))
        if epoch != self.epoch:
            handle.cancel_goal_async()

    def plan_done(self, future, epoch):
        self.plan_busy = False
        self.plan_handle = None
        if epoch != self.epoch:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.stop("failed", message=str(exc))
            return
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED or
                result.outcome != GetPath.Result.SUCCESS or not result.path.poses):
            self.stop("failed", outcome=result.outcome, message="Replan failed: " + result.message)
            return
        self.plan_count += 1
        self.sequence += 1
        sequence = self.sequence
        goal = ExePath.Goal()
        goal.controller = self.cfg["controller"]
        goal.path = result.path
        self.pending_sends += 1
        self.request_started = time.monotonic()
        self.controller.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback(msg, epoch, sequence)
        ).add_done_callback(lambda f: self.execution_accepted(f, epoch, sequence))
        self.publish_status("following", path_poses=len(result.path.poses), path_cost=result.cost)

    def feedback(self, msg, epoch, sequence):
        if epoch == self.epoch and sequence == self.sequence:
            self.distance = msg.feedback.dist_to_goal

    def execution_accepted(self, future, epoch, sequence):
        self.pending_sends -= 1
        try:
            handle = future.result()
        except Exception as exc:
            if epoch == self.epoch:
                self.stop("failed", message=str(exc))
            return
        if not handle.accepted:
            if epoch == self.epoch:
                self.stop("failed", message="Controller rejected path")
            return
        self.executions[sequence] = handle
        handle.get_result_async().add_done_callback(
            lambda f: self.execution_done(f, epoch, sequence))
        if epoch != self.epoch:
            handle.cancel_goal_async()

    def execution_done(self, future, epoch, sequence):
        self.executions.pop(sequence, None)
        if epoch != self.epoch or sequence != self.sequence:
            return  # MBF canceled this path because a newer plan replaced it.
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:
            self.stop("failed", message=str(exc))
            return
        succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.outcome == 0
        self.stop("succeeded" if succeeded else "failed", outcome=result.outcome,
                  message=result.message, dist_to_goal=result.dist_to_goal)


def main(args=None):
    rclpy.init(args=args)
    node = MeshnavNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
