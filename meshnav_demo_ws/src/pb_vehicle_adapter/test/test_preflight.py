"""Specification tests for the pb_preflight framework table and readiness gate."""

from launch import LaunchContext
from launch.utilities import perform_substitutions
from pb_vehicle_adapter.pb_preflight import FRAMEWORKS, wait_for_ready
from pb_vehicle_adapter.readiness_gate import readiness_gate


def test_all_three_frameworks_defined():
    assert set(FRAMEWORKS) == {"meshnav", "jie", "dddmr"}


def test_control_source_matches_framework_name():
    for name, spec in FRAMEWORKS.items():
        assert spec["control_source"] == name
        assert spec["topics"]


def test_meshnav_requires_mbf_actions():
    assert "/move_base_flex/get_path" in FRAMEWORKS["meshnav"]["actions"]
    assert "/move_base_flex/exe_path" in FRAMEWORKS["meshnav"]["actions"]


def test_jie_requires_planner_and_controller_nodes():
    nodes = FRAMEWORKS["jie"]["nodes"]
    assert "jie_path_node" in nodes
    assert "d1_controller" in nodes


def test_dddmr_requires_map_artifacts_and_action():
    spec = FRAMEWORKS["dddmr"]
    assert "map_files" in spec
    assert "/p2p_move_base" in spec["actions"]
    assert "/dddmr/mapcloud" in spec["topics"]
    assert "/dddmr/mapground" in spec["topics"]


def test_dddmr_checks_the_real_node_names():
    """p2p_move_base_node is an executable, not a node: the live nodes are these."""
    nodes = FRAMEWORKS["dddmr"]["nodes"]
    assert "p2p_move_base" in nodes
    assert "global_planner" in nodes
    assert "p2p_move_base_node" not in nodes


class _FakeNode:
    """Node stand-in whose clock check fails for the first `fail_times` passes."""

    def __init__(self, fail_times):
        self.calls = 0
        self.fail_times = fail_times

    def check_clock(self):
        self.calls += 1
        if self.calls <= self.fail_times:
            return False, "sim time is not advancing (paused?)"
        return True, "clock advances"

    def check_tf(self, parent, child):
        return True, "%s -> %s available" % (parent, child)

    def check_topics(self, topics):
        return True, "all topics published"

    def check_nodes(self, names):
        return True, "nodes present"

    def check_actions(self, actions):
        return True, "actions present"

    def check_control_source(self, expected):
        return True, "velocity source is '%s'" % expected

    def check_map_files(self, files):
        return True, "map artifacts present"


def test_wait_for_ready_returns_after_recovery():
    node = _FakeNode(fail_times=2)
    results = wait_for_ready(
        node, FRAMEWORKS["dddmr"], "map", "base_footprint", 30.0,
        poll_interval=0.0, monotonic=lambda: 0.0, sleep=lambda _s: None, log=lambda _m: None,
    )
    assert node.calls == 3
    assert all(ok for _name, ok, _detail in results)


def test_wait_for_ready_timeout_reports_missing_condition():
    node = _FakeNode(fail_times=10 ** 6)
    results = wait_for_ready(
        node, FRAMEWORKS["dddmr"], "map", "base_footprint", 0.0,
        poll_interval=0.0, monotonic=lambda: 0.0, sleep=lambda _s: None, log=lambda _m: None,
    )
    failed = [name for name, ok, _detail in results if not ok]
    assert failed == ["clock"]


def test_zero_timeout_checks_once():
    node = _FakeNode(fail_times=10 ** 6)
    wait_for_ready(
        node, FRAMEWORKS["meshnav"], "map", "base_footprint", 0.0,
        poll_interval=0.0, monotonic=lambda: 0.0, sleep=lambda _s: None, log=lambda _m: None,
    )
    assert node.calls == 1


def test_readiness_gate_runs_preflight_with_the_timeout():
    action = readiness_gate("jie", "120")
    assert action.period == 2.0
    context = LaunchContext()
    words = [perform_substitutions(context, item) for item in action.actions[0].cmd]
    assert words[:4] == ["ros2", "run", "pb_vehicle_adapter", "pb_preflight"]
    assert words[words.index("--framework") + 1] == "jie"
    assert words[words.index("--wait-timeout") + 1] == "120"
