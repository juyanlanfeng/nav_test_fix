"""Launch contract tests for the three navigation entries.

Enforces the section 11 contract: every navigation entry declares start_sim,
starts the shared simulation only when start_sim is true, opens exactly its own
framework RViz, and switches the running velocity selector when it reuses a
simulation (start_sim:=false) instead of starting a second one.
"""

import os

from ament_index_python.packages import get_package_share_directory


LAUNCH_DIR = os.path.join(get_package_share_directory("pb_vehicle_adapter"), "launch")
NAV_LAUNCHES = ("pb_meshnav", "pb_jie", "pb_dddmr")
RVIV_CONFIGS = ("pb_meshnav.rviz", "pb_jie.rviz", "pb_dddmr.rviz")
CONTROL_SOURCES = ("meshnav", "jie", "dddmr")


def _read(name):
    with open(os.path.join(LAUNCH_DIR, name + ".launch.py"), encoding="utf-8") as stream:
        return stream.read()


def test_nav_launches_declare_start_sim():
    for name in NAV_LAUNCHES:
        text = _read(name)
        assert '"start_sim"' in text
        # The shared simulation include must be conditional, so two entries can
        # never start two Gazebo worlds / vehicles / TF owners.
        assert 'IfCondition(LaunchConfiguration("start_sim"))' in text


def test_nav_launches_start_their_own_framework_rviz():
    for name, config in zip(NAV_LAUNCHES, RVIV_CONFIGS):
        text = _read(name)
        assert config in text
        # The simulation include must not also open RViz.
        assert '"start_rviz": "False"' in text


def test_source_switch_when_reusing_a_simulation():
    for name, source in zip(NAV_LAUNCHES, CONTROL_SOURCES):
        text = _read(name)
        assert "UnlessCondition" in text
        assert '"/pb_cmd_vel_adapter", "control_source", "%s"' % source in text


def test_vehicle_sim_supports_all_three_sources():
    text = _read("pb_vehicle_sim")
    for source in CONTROL_SOURCES:
        assert '"%s"' % source in text


def test_nav_launches_declare_and_wire_the_readiness_gate():
    """Section 11: startup_timeout_s gates readiness instead of a fixed sleep."""
    for name, source in zip(NAV_LAUNCHES, CONTROL_SOURCES):
        text = _read(name)
        assert '"startup_timeout_s"' in text
        assert 'readiness_gate("%s", LaunchConfiguration("startup_timeout_s"))' % source in text


def test_nav_launches_pass_the_rendering_sensor_switch():
    """Headless GPU-less hosts need spawn_rendering_sensors:=False (RTF 0.06 -> 1.0)."""
    for name in NAV_LAUNCHES:
        text = _read(name)
        assert '"spawn_rendering_sensors"' in text
        assert 'LaunchConfiguration("spawn_rendering_sensors")' in text
    sim = _read("pb_vehicle_sim")
    assert '"spawn_rendering_sensors"' in sim
    assert "spawn_without_sensors" in sim
    assert "reduced_robot_model.py" in sim
