"""Regression checks for route retention, path trimming and cancellation races."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from mbf_msgs.action import ExePath, GetPath
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from mbf_msgs.srv import CheckPath

from pb_vehicle_adapter.meshnav_navigator import MeshnavNavigator, remaining_path


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
    node.check_busy = False
    node.active_path = None
    node.current_pose = None
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

# Route retention is deliberately independent of candidate cost/length.


def pose(x, y=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def path(points):
    p = Path()
    p.header.frame_id = 'map'
    p.poses = [pose(x, y) for x, y in points]
    return p


def test_free_old_route_never_plans_or_adopts_cheaper_candidate():
    node = navigator()
    node.request_plan = Mock()
    node.adopt_plan = Mock()
    response = completed(CheckPath.Response(state=CheckPath.Response.FREE))
    node.path_checked(response, 1)
    node.path_checked(response, 1, SimpleNamespace(cost=0.0))
    node.request_plan.assert_not_called()
    node.adopt_plan.assert_not_called()
    assert node.target is not None


def test_blocked_old_route_requests_planning():
    node = navigator()
    node.request_plan = Mock()
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10**9
    node.path_checked(completed(CheckPath.Response(state=CheckPath.Response.LETHAL)), 1)
    node.request_plan.assert_called_once_with(1.0)


def test_candidate_only_adopted_when_old_route_still_blocked():
    node = navigator()
    node.adopt_plan = Mock()
    candidate = object()
    node.path_checked(completed(CheckPath.Response(state=CheckPath.Response.LETHAL)),
                      1, candidate)
    node.adopt_plan.assert_called_once_with(candidate, 1)


def test_planning_result_requires_second_old_route_check_even_on_failure():
    node = navigator()
    node.active_path = path([(0, 0), (1, 0)])
    node.check_path = Mock()
    result = GetPath.Result(outcome=GetPath.Result.NO_PATH_FOUND)
    wrapped = SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result)
    node.plan_done(completed(wrapped), 1)
    node.check_path.assert_called_once_with(wrapped)
    assert node.target is not None


def test_unknown_check_stops_instead_of_treating_old_route_as_free():
    node = navigator()
    handle = Mock()
    node.executions = {2: handle}
    node.path_checked(completed(CheckPath.Response(state=CheckPath.Response.UNKNOWN)), 1)
    handle.cancel_goal_async.assert_called_once()
    assert node.target is None


def test_check_response_after_cancel_does_not_replan():
    node = navigator()
    node.request_plan = Mock()
    node.stop('cancel_requested')
    node.path_checked(completed(CheckPath.Response(state=CheckPath.Response.LETHAL)), 1)
    node.request_plan.assert_not_called()


def test_remaining_path_excludes_passed_obstacle_and_keeps_connector():
    route = path([(0, 0), (1, 0), (2, 0), (3, 0)])
    remaining, progress = remaining_path(route, pose(1.5, 0.2), 0, 2.0)
    assert progress == 1
    assert remaining.poses[0].pose.position.y == 0.2
    assert [p.pose.position.x for p in remaining.poses] == [1.5, 1.5, 2.0, 3.0]
    assert len(route.poses) == 4  # Input remains unchanged.


def test_progress_does_not_jump_to_later_crossing_or_move_to_passed_segment():
    route = path([(0, 0), (1, 0), (2, 0), (2, 1), (1, 0), (0, 1)])
    _, progress = remaining_path(route, pose(1, 0), 0, 2.0)
    assert progress < 2
    _, progress = remaining_path(route, pose(0.1, 0), 2, 2.0)
    assert progress >= 2


def test_timer_checks_old_route_even_close_to_goal_without_requesting_plan():
    node = navigator()
    node.active_path = path([(0, 0), (1, 0)])
    node.cfg = dict(planner_frequency=2.0, action_timeout=15.0)
    node.last_plan = 0.0
    node.distance = 0.1
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10**9
    node.request_plan = Mock()
    node.check_path = Mock()
    node.tick()
    node.check_path.assert_called_once_with()
    node.request_plan.assert_not_called()


def test_stale_pose_stops_before_service_or_planner_request():
    node = navigator()
    node.cfg = dict(pose_timeout=2.0)
    node.current_pose = pose(0)
    node.pose_received = 0.0
    node.path_checker = Mock()
    node.check_path()
    node.path_checker.call_async.assert_not_called()
    assert node.target is None
