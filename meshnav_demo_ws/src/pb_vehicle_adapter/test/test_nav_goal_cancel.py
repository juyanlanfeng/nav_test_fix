"""Cancellation-semantics tests (R2 of doc/PB_SLOPE_REPAIR_AND_DEPLOYMENT_PLAN.md).

The reported defect: ``--cancel`` ran GetPath/ExePath first and only then tried to
cancel the goal it had just created, so a standalone cancel could not affect a task
started by an earlier process, and its result was a boolean instead of a
distinguishable state.

These tests drive the real CLI logic (`parse_arguments` + `run_cli`) against fake
MeshNav action servers and assert:

1. the cancel branch never plans and never sends a goal, and cancels the goal that
   was already active;
2. "no active goal" is reported as its own result (exit 3), not as a pass;
3. an unknown UUID is not a pass either;
4. ``--cancel-after`` cancels the same UUID that the run created, and a later goal
   gets a new UUID instead of reviving the cancelled one.
"""

import threading
import time

from geometry_msgs.msg import Twist
from mbf_msgs.action import ExePath, GetPath
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from pb_vehicle_adapter.pb_nav_goal import (
    CANCEL_EXIT_NO_ACTIVE_GOAL,
    CANCEL_EXIT_OK,
    NavGoal,
    parse_arguments,
    run_cli,
)


def setup_module(module):
    rclpy.init()


def teardown_module(module):
    if rclpy.ok():
        rclpy.shutdown()


class FakeMeshNavServers:
    """GetPath answers with a two-pose plan; ExePath runs until cancelled."""

    def __init__(self, abort_immediately=False):
        self.abort_immediately = abort_immediately
        self.node = Node("fake_mesh_nav_servers")
        # A separate node starts the "already active" goal.  It must not be the
        # node the background executor spins, otherwise rclpy refuses to await a
        # future on a node that another spinning executor already owns.
        self.client_node = Node("fake_mesh_nav_client")
        self.get_path_goals = []
        self.exe_path_goals = []
        self.cancelled = []
        self._get_path = ActionServer(
            self.node, GetPath, "/move_base_flex/get_path", execute_callback=self._get_path_cb
        )
        self._exe_path = ActionServer(
            self.node, ExePath, "/move_base_flex/exe_path",
            execute_callback=self._exe_path_cb, cancel_callback=self._cancel_cb,
        )
        # Multi-threaded: the ExePath execute callback blocks until cancelled, so
        # the cancel callback must be able to run concurrently.
        self._stop = False
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _get_path_cb(self, goal_handle):
        self.get_path_goals.append(bytes(goal_handle.goal_id.uuid))
        path = Path()
        path.header.frame_id = "map"
        path.poses = [goal_handle.request.target_pose, goal_handle.request.target_pose]
        goal_handle.succeed()
        return GetPath.Result(outcome=0, message="fake plan", path=path)

    def _exe_path_cb(self, goal_handle):
        self.exe_path_goals.append(bytes(goal_handle.goal_id.uuid))
        if self.abort_immediately:
            goal_handle.abort()
            return ExePath.Result(outcome=103, message="robot stuck")
        # Also exits on teardown: otherwise a still-executing goal keeps the
        # executor's shutdown waiting and the whole test process hangs.
        while not goal_handle.is_cancel_requested and not self._stop:
            time.sleep(0.05)
        goal_handle.canceled()
        return ExePath.Result(outcome=1, message="cancelled")

    def _cancel_cb(self, goal_handle):
        self.cancelled.append(bytes(goal_handle.goal_id.uuid))
        return CancelResponse.ACCEPT

    def _pump(self, predicate, timeout):
        """Spin only the client node; the server node belongs to the background executor."""
        executor = SingleThreadedExecutor()
        executor.add_node(self.client_node)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline and not predicate():
                executor.spin_once(timeout_sec=0.05)
        finally:
            executor.remove_node(self.client_node)
        return predicate()

    def start_active_exe_path_goal(self):
        """Stand in for a task started by another process."""
        client = rclpy.action.ActionClient(self.client_node, ExePath, "/move_base_flex/exe_path")
        if not self._pump(lambda: client.server_is_ready(), 10.0):
            raise AssertionError("fake ExePath server did not appear")
        goal = ExePath.Goal()
        goal.controller = "mesh_controller"
        future = client.send_goal_async(goal)
        if not self._pump(future.done, 10.0):
            raise AssertionError("goal acceptance timed out")
        handle = future.result()
        assert handle is not None and handle.accepted
        assert self._pump(lambda: bool(self.exe_path_goals), 10.0)
        return client, handle

    def shutdown(self):
        self._stop = True
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self.client_node.destroy_node()
        self.node.destroy_node()


def _attach_zero_velocity(node):
    """Publish a zero command and a zero twist so the stop checks can pass.

    A separate node with wall-clock time does the publishing: the CLI node runs
    with `use_sim_time=True`, and a timer on a sim-time node never fires while no
    `/clock` is published, which would make the stop check unreachable in a test.
    """
    del node  # kept for call-site symmetry; the source owns its own node
    return ZeroVelocitySource()


class ZeroVelocitySource:
    def __init__(self):
        self.node = Node("fake_zero_velocity")
        self.command = self.node.create_publisher(Twist, "/pb/cmd_vel_safe", 10)
        self.odometry = self.node.create_publisher(Odometry, "/odom", 10)
        self.node.create_timer(0.05, self._tick)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _tick(self):
        self.command.publish(Twist())
        message = Odometry()
        message.header.stamp = self.node.get_clock().now().to_msg()
        self.odometry.publish(message)

    def shutdown(self):
        self._executor.shutdown()
        self.node.destroy_node()


def test_cancel_never_plans_and_cancels_the_active_goal():
    servers = FakeMeshNavServers()
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        client, _handle = servers.start_active_exe_path_goal()
        active_uuid = servers.exe_path_goals[0]

        args = parse_arguments(["--framework", "meshnav", "--cancel", "--timeout", "5"])
        status = run_cli(args, node)

        assert status == CANCEL_EXIT_OK
        assert servers.get_path_goals == [], "cancel must never plan"
        assert servers.cancelled == [active_uuid], "cancel must target the active goal"
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()


def test_no_active_goal_is_a_distinct_result_not_a_pass():
    servers = FakeMeshNavServers()
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        args = parse_arguments(["--framework", "meshnav", "--cancel", "--timeout", "5"])
        status = run_cli(args, node)
        assert status == CANCEL_EXIT_NO_ACTIVE_GOAL
        assert servers.cancelled == []
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()


def test_unknown_uuid_is_not_reported_as_cancelled():
    servers = FakeMeshNavServers()
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        client, _handle = servers.start_active_exe_path_goal()
        # A non-zero UUID no goal has; the all-zero UUID is the "cancel all" sentinel.
        unknown = "0102030405060708090a0b0c0d0e0f10"
        args = parse_arguments(["--framework", "meshnav", "--cancel", "--goal-uuid", unknown, "--timeout", "5"])
        status = run_cli(args, node)
        assert status == CANCEL_EXIT_NO_ACTIVE_GOAL
        assert servers.cancelled == []
        assert servers.exe_path_goals, "the unrelated active goal must not be touched"
        client.destroy()
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()


def test_cancel_after_cancels_the_same_uuid_and_a_new_goal_gets_a_new_one():
    servers = FakeMeshNavServers()
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        args = parse_arguments(
            ["--framework", "meshnav", "--x", "1.0", "--y", "0.0", "--cancel-after", "0.5", "--timeout", "5"]
        )
        status = run_cli(args, node)
        assert status == CANCEL_EXIT_OK
        assert len(servers.exe_path_goals) == 1
        assert servers.cancelled == [servers.exe_path_goals[0]]

        # A later goal is a new UUID: the cancelled task is not revived.
        args = parse_arguments(
            ["--framework", "meshnav", "--x", "1.0", "--y", "0.0", "--cancel-after", "0.5", "--timeout", "5"]
        )
        status = run_cli(args, node)
        assert status == CANCEL_EXIT_OK
        assert len(servers.exe_path_goals) == 2
        assert servers.exe_path_goals[0] != servers.exe_path_goals[1]
        assert servers.cancelled == servers.exe_path_goals
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()


def test_cancel_after_the_goal_already_finished_is_not_a_false_cancel(capsys):
    """Cancel/arrival race: a goal that finished must never be reported CANCELED."""
    servers = FakeMeshNavServers()
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        # The fake GetPath server succeeds immediately, so its goal is terminal by
        # the time the cancel is issued.
        client = rclpy.action.ActionClient(servers.client_node, GetPath, "/move_base_flex/get_path")
        assert servers._pump(lambda: client.server_is_ready(), 10.0)
        goal = GetPath.Goal()
        goal.planner = "mesh_planner"
        future = client.send_goal_async(goal)
        assert servers._pump(future.done, 10.0)
        handle = future.result()
        assert handle is not None and handle.accepted
        result_future = handle.get_result_async()
        assert servers._pump(result_future.done, 10.0)
        assert servers._pump(lambda: bool(servers.get_path_goals), 10.0)
        finished_uuid = servers.get_path_goals[0].hex()
        capsys.readouterr()

        args = parse_arguments(
            ["--framework", "meshnav", "--cancel", "--goal-uuid", finished_uuid, "--timeout", "5"]
        )
        status = run_cli(args, node)
        output = capsys.readouterr().out

        assert status == CANCEL_EXIT_OK
        assert "cancel_result=CANCELED" not in output, output
        assert ("cancel_result=ALREADY_TERMINAL" in output
                or "cancel_result=NO_ACTIVE_GOAL" in output), output
        assert servers.cancelled == [], "a finished goal must not be cancelled again"
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()


def test_cancel_after_aborted_goal_returns_failure_even_when_stopped(capsys):
    servers = FakeMeshNavServers(abort_immediately=True)
    node = NavGoal()
    zero = _attach_zero_velocity(node)
    try:
        args = parse_arguments([
            "--framework", "meshnav", "--x", "1.0", "--y", "0.0",
            "--cancel-after", "1", "--timeout", "5",
        ])
        assert run_cli(args, node) == 1
        assert "cancel_result=ABORTED" in capsys.readouterr().out
    finally:
        node.destroy_node()
        zero.shutdown()
        servers.shutdown()
