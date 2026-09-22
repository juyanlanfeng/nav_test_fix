"""Tests for the reduced (GPU-less) robot model used by headless acceptance runs."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from pb_vehicle_adapter.robot_model import reduced_model_root, simulation_model_xml


SHARE = get_package_share_directory("pb_vehicle_adapter")
FULL_MODEL = os.path.join(SHARE, "models", "pb_navigation_robot.sdf")


def test_optional_diagnostics_and_pi_preserve_physical_geometry():
    full = ET.parse(FULL_MODEL).getroot()
    configured = ET.fromstring(simulation_model_xml(FULL_MODEL, diagnostics=True, drive="pi"))
    for tag in ("collision", "inertial", "joint"):
        # Only collision elements with geometry (not contact sensor references).
        before = [ET.tostring(e) for e in full.iter(tag) if tag != "collision" or e.find("geometry") is not None]
        after = [ET.tostring(e) for e in configured.iter(tag) if tag != "collision" or e.find("geometry") is not None]
        assert before == after
    assert configured.find(".//plugin[@filename='MecanumDrive2']") is None
    assert configured.find(".//plugin[@filename='pb_velocity_drive_system']") is not None
    assert configured.find(".//plugin[@name='pb_gazebo_sim_support::ContactDiagnostics']/enable").text == "true"


def test_diagnostics_off_keeps_original_drive_and_has_no_contact_sensors():
    configured = ET.fromstring(simulation_model_xml(FULL_MODEL))
    assert configured.find(".//plugin[@filename='MecanumDrive2']") is not None
    assert configured.find(".//sensor[@type='contact']") is None


def _names(root, tag):
    return sorted(element.get("name") for element in root.iter(tag) if element.get("name"))


def test_reduced_model_drops_only_the_rendering_sensors():
    full = ET.parse(FULL_MODEL).getroot()
    reduced, removed = reduced_model_root(FULL_MODEL)

    # Exactly the three rendering sensors go; the IMUs stay.
    assert sorted(removed) == ["front_industrial_camera", "front_mid360_lidar",
                               "front_rplidar_a2"]
    assert set(_names(full, "sensor")) - set(_names(reduced, "sensor")) == set(removed)
    assert len(list(reduced.iter("sensor"))) == len(list(full.iter("sensor"))) - 3
    assert "chassis_imu" in _names(reduced, "sensor")


def test_reduced_model_keeps_links_joints_and_plugins():
    full = ET.parse(FULL_MODEL).getroot()
    reduced, _removed = reduced_model_root(FULL_MODEL)
    for tag in ("link", "joint", "plugin"):
        assert _names(full, tag) == _names(reduced, tag), tag
