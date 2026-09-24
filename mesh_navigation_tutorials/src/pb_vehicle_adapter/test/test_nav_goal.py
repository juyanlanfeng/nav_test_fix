"""Tests for the pb_nav_goal helper logic and the smoke goal bundle."""

import inspect
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml

from pb_vehicle_adapter.pb_nav_goal import (
    FRAMEWORKS,
    JIE_POINT_QOS,
    NavGoal,
    _load_smoke,
    _yaw_quaternion,
)


def test_nav_goal_runs_on_simulation_time():
    """Section 14: goals are stamped with simulation time, not wall clock."""
    assert "use_sim_time" in inspect.getsource(NavGoal.__init__)


def test_plan_only_and_path_recording_exist():
    """The tunnel acceptance needs the plan itself (z profile) without driving."""
    mesh = inspect.getsource(NavGoal.run_meshnav)
    jie = inspect.getsource(NavGoal.run_jie)
    assert "save_path" in mesh and "plan_only" in mesh
    assert "save_path" in jie and "plan_only" in jie
    # MeshNav must stop before ExePath and JIE before /start_navigation.
    assert mesh.index("plan_only") < mesh.index("exe_path.send_goal_async")
    assert jie.index("plan_only") < jie.index("_jie_start_cmd.publish")
    saver = inspect.getsource(NavGoal.save_path)
    assert "x,y,z,yaw" in saver


def test_jie_waits_for_arrival():
    """d1_controller publishes no arrival topic, so the client must judge it from TF."""
    source = inspect.getsource(NavGoal.run_jie)
    assert "wait_for_arrival" in source
    arrival = inspect.getsource(NavGoal.wait_for_arrival)
    assert "robot_pose" in arrival and "tolerance_xy" in arrival


def test_jie_point_publishers_match_the_planner_durability():
    """jie_path_node subscribes to /start_point and /goal_point with
    transient_local; a volatile publisher is silently incompatible and the
    planner never receives the request."""
    from rclpy.qos import QoSDurabilityPolicy

    assert JIE_POINT_QOS.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL
    assert JIE_POINT_QOS.reliability == 1  # RELIABLE
    assert inspect.getsource(NavGoal.__init__).count("JIE_POINT_QOS") == 2


def test_frameworks():
    assert FRAMEWORKS == ("meshnav", "jie", "dddmr")


def test_yaw_quaternion_identity():
    x, y, z, w = _yaw_quaternion(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0)
    assert abs(w - 1.0) < 1.0e-9


def test_yaw_quaternion_ninety_degrees():
    x, y, z, w = _yaw_quaternion(math.pi / 2.0)
    assert abs(z - math.sin(math.pi / 4.0)) < 1.0e-9
    assert abs(w - math.cos(math.pi / 4.0)) < 1.0e-9


def test_smoke_bundle_has_all_frameworks():
    path = os.path.join(get_package_share_directory("pb_vehicle_adapter"), "config", "smoke_goal.yaml")
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    assert "default" in data
    for framework in FRAMEWORKS:
        smoke = _load_smoke(framework)
        goal = smoke["goal"]
        assert {"x", "y", "z", "yaw"} <= set(goal)


def _dddmr_config():
    path = os.path.join(
        get_package_share_directory("pb_vehicle_adapter"), "config", "dddmr_rmuc2026.yaml"
    )
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_dddmr_config_uses_omni_and_dddmr_topics():
    data = _dddmr_config()
    assert "occupancy2ground" not in data
    generator = data["trajectory_generators"]["ros__parameters"]["differential_drive_simple"]
    assert generator["plugin"] == "trajectory_generators::OmniSimpleTrajectoryGeneratorTheory"
    assert generator["max_vel_y"] > 0.0 and generator["min_vel_y"] < 0.0
    for node in ("perception_3d_local", "perception_3d_global"):
        params = data[node]["ros__parameters"]
        assert params["robot_base_frame"] == "base_footprint"
        assert params["map"]["map_topic"] == "/dddmr/mapcloud"
        assert params["map"]["ground_topic"] == "/dddmr/mapground"
    assert data["p2p_move_base"]["ros__parameters"]["use_twist_stamped"] is True


def test_dddmr_oscillation_watchdog_matches_the_vehicle_speed():
    """The watchdog resets only after oscillation_distance metres (or 1 rad).

    A distance the vehicle cannot cover inside oscillation_patience turns every
    goal into a false oscillation and hands control to the recovery behaviour,
    so it must stay below max_vel_trans * oscillation_patience.
    """
    data = _dddmr_config()
    planner = data["p2p_move_base"]["ros__parameters"]
    generator = data["trajectory_generators"]["ros__parameters"]["differential_drive_simple"]
    reachable = generator["max_vel_trans"] * planner["oscillation_patience"]
    assert planner["oscillation_distance"] < reachable


def test_dddmr_rotation_limits_beat_the_oscillation_watchdog():
    """p2p_move_base recovers if heading alignment exceeds oscillation_patience.

    The alignment generators only sample theta inside [min_vel_theta,
    max_vel_theta], so a 0.1 rad/s ceiling cannot finish a 2.4 rad turn inside
    the 15 s watchdog and the goal always ends in recovery.
    """
    data = _dddmr_config()
    patience = data["p2p_move_base"]["ros__parameters"]["oscillation_patience"]
    generators = data["trajectory_generators"]["ros__parameters"]
    for name in ("differential_drive_rotate_inplace",
                 "differential_drive_rotate_shortest_angle"):
        params = generators[name]
        assert params["min_vel_theta"] <= params["max_vel_theta"]
        worst_case_seconds = math.pi / params["max_vel_theta"]
        assert worst_case_seconds < patience, name
