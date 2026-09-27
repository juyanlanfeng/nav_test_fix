"""Regression checks for cancellation races in periodic path replacement."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from mbf_msgs.action import ExePath, GetPath

from pb_vehicle_adapter.meshnav_navigator import MeshnavNavigator


def completed(value):
    future = Future()
    future.set_result(value)
    return future


def navigator():
    node = MeshnavNavigator.__new__(MeshnavNavigator)
    node.epoch = 1
    node.sequence = 2
    node.executions = {}
    node.pending_sends = 0
    node.plan_busy = False
    node.plan_handle = None
    node.target = object()
    node.queued_target = None
    node.plan_count = 2
    node.publish_status = Mock()
    return node


def test_superseded_path_cancel_does_not_abort_current_navigation():
    node = navigator()
    current = Mock()
    node.executions = {1: Mock(), 2: current}
    node.execution_done(completed(SimpleNamespace(
        status=GoalStatus.STATUS_CANCELED, result=ExePath.Result())), 1, 1)
    assert node.target is not None
    assert node.executions == {2: current}
    current.cancel_goal_async.assert_not_called()
    node.publish_status.assert_not_called()


def test_cancel_while_controller_acceptance_pending_cancels_late_handle():
    node = navigator()
    node.pending_sends = 1
    node.stop("cancel_requested")
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    node.execution_accepted(completed(handle), 1, 2)
    handle.cancel_goal_async.assert_called_once()
    assert node.pending_sends == 0
    assert node.target is None


def test_failed_replan_stops_the_previous_path():
    node = navigator()
    controller = Mock()
    node.executions = {2: controller}
    result = GetPath.Result()
    result.outcome = GetPath.Result.NO_PATH_FOUND
    node.plan_done(completed(SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED, result=result)), 1)
    controller.cancel_goal_async.assert_called_once()
    assert node.target is None
    assert node.publish_status.call_args.args[0] == "failed"


def test_cancel_while_planner_acceptance_pending_cancels_late_handle():
    node = navigator()
    node.plan_busy = True
    node.stop("cancel_requested")
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    node.plan_accepted(completed(handle), 1)
    handle.cancel_goal_async.assert_called_once()
    assert node.plan_busy  # Do not send a replacement goal before this result drains.


def test_success_status_survives_the_next_timer_tick():
    node = navigator()
    result = ExePath.Result()
    result.outcome = 0
    node.execution_done(completed(SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED, result=result)), 1, 2)
    node.tick()
    assert node.publish_status.call_count == 1
    assert node.publish_status.call_args.args[0] == "succeeded"
