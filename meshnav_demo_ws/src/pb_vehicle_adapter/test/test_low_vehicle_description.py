"""Description consistency and envelope tests for the low-clearance vehicle.

Acceptance item 1 of doc/PB_LOW_CLEARANCE_VEHICLE_PLAN.md: the generated SDF and
URDF must no longer contain the gimbal/projectile/armor superstructure, must keep
the four wheel joints and the three sensors, must not contain dangling
parent/child/relative_to references, and must agree with the shared profile and
with every DDDMR cuboid.
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import yaml

from pb_vehicle_adapter.robot_envelope import envelope


SHARE = get_package_share_directory("pb_vehicle_adapter")
SDF_PATH = os.path.join(SHARE, "models", "pb_navigation_robot.sdf")
URDF_PATH = os.path.join(SHARE, "urdf", "pb_navigation_robot.urdf")
PROFILE_PATH = os.path.join(SHARE, "config", "pb_vehicle_profile.yaml")
DDDMR_CONFIG_PATH = os.path.join(SHARE, "config", "dddmr_rmuc2026.yaml")

REMOVED_STRUCTURE = ("gimbal", "armor", "projectile", "light", "speed_monitor", "barrel")
WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
SENSORS = {
    "chassis_imu": "imu",
    "front_rplidar_a2": "gpu_lidar",
    "front_mid360_lidar": "gpu_lidar",
    "front_industrial_camera": "camera",
}
TOPICS = {
    "chassis_imu": "/robot/imu",
    "front_rplidar_a2": "/robot/scan",
    "front_mid360_lidar": "/robot/cloud/points",
    "front_industrial_camera": "/robot/camera/image",
}


def _profile():
    with open(PROFILE_PATH, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["pb_vehicle"]["ros__parameters"]


def _sdf_model():
    return ET.parse(SDF_PATH).getroot().find("model")


def test_no_high_superstructure_remains():
    """The gimbal, armor, light bar, speed monitor and projectile container are gone."""
    model = _sdf_model()
    names = [element.get("name") or "" for element in model.iter()]
    for marker in REMOVED_STRUCTURE:
        assert not [name for name in names if marker in name], marker
    # The URDF is generated from the same tree and must agree.
    urdf = ET.parse(URDF_PATH).getroot()
    urdf_names = [element.get("name") or "" for element in urdf.iter()]
    for marker in REMOVED_STRUCTURE:
        assert not [name for name in urdf_names if marker in name], marker


def test_wheels_and_sensors_are_preserved():
    model = _sdf_model()
    links = {link.get("name") for link in model.findall("link")}
    joints = {joint.get("name") for joint in model.findall("joint")}
    for wheel in WHEELS:
        assert wheel in links
        assert "%s_joint" % wheel in joints
    found = {sensor.get("name"): sensor.get("type") for sensor in model.iter("sensor")}
    for name, kind in SENSORS.items():
        assert found.get(name) == kind, name
    for sensor in model.iter("sensor"):
        expected = TOPICS.get(sensor.get("name"))
        if expected is not None:
            assert sensor.findtext("topic") == expected, sensor.get("name")
    # The two lidar bodies and the camera must carry a collision so they cannot
    # pass through a ceiling (the upstream livox macro had none).
    for link_name in ("front_rplidar_a2", "front_mid360", "front_industrial_camera"):
        link = model.find("link[@name='%s']" % link_name)
        assert link is not None and link.findall("collision"), link_name


def test_no_dangling_references():
    model = _sdf_model()
    known = {model.get("name"), "world"}
    known |= {link.get("name") for link in model.findall("link")}
    known |= {joint.get("name") for joint in model.findall("joint")}
    for joint in model.findall("joint"):
        for tag in ("parent", "child"):
            node = joint.find(tag)
            assert node is not None and node.text in known, (joint.get("name"), tag)
    for pose in model.iter("pose"):
        reference = pose.get("relative_to")
        if reference:
            assert reference in known, reference

    urdf = ET.parse(URDF_PATH).getroot()
    urdf_links = {link.get("name") for link in urdf.findall("link")}
    for joint in urdf.findall("joint"):
        assert joint.find("parent").get("link") in urdf_links, joint.get("name")
        assert joint.find("child").get("link") in urdf_links, joint.get("name")


def test_envelope_matches_the_shared_profile():
    """robot_height and collision_aabb_* must come from the generated model."""
    report = envelope(URDF_PATH)
    profile = _profile()
    for actual, expected in zip(report["min"], profile["collision_aabb_min"]):
        assert abs(actual - expected) < 1.0e-3, (actual, expected)
    for actual, expected in zip(report["max"], profile["collision_aabb_max"]):
        assert abs(actual - expected) < 1.0e-3, (actual, expected)
    assert report["max"][2] <= profile["robot_height"] + 1.0e-9
    # The low vehicle must actually be low: the tunnel clear height is ~0.247 m.
    assert report["max"][2] < 0.24


def test_footprint_radius_matches_the_disc_model():
    """The disc planners must be configured from the measured footprint.

    MeshNav makes a disc of `inscribed_radius` lethal around every lethal vertex
    and JIE blocks the same disc through `robot_radius_xy`, so the value decides
    which gaps the planners accept.  It has to be the footprint's inscribed
    radius (half width): a smaller disc accepts gaps the body cannot pass, a
    bigger one rejects the RMUC2026 tunnels (0.85 m) that the vehicle does drive
    through.
    """
    from pb_vehicle_adapter.robot_envelope import footprint_radii

    report = envelope(URDF_PATH)
    radii = footprint_radii(report)
    profile = _profile()
    for key in ("static_inscribed_radius", "obstacle_inscribed_radius"):
        assert abs(profile[key] - radii["inscribed"]) < 5.0e-4, (key, profile[key], radii)

    # Soundness bound: the free band of a straight corridor of width w is
    # w - 2 r, so a disc radius below the half width would let both planners
    # through corridors narrower than the body itself.
    half_width = max(-report["min"][1], report["max"][1])
    assert profile["static_inscribed_radius"] >= half_width - 1.0e-6
    # The tunnels are on the other side of the bound: 0.85 m of clear width minus
    # the two lethal bands has to stay wide enough to contain a path.
    assert 0.85 - 2.0 * profile["static_inscribed_radius"] > 0.30
    # Outer ring: costs still grow towards obstacles, i.e. planning prefers the
    # corridor centre.
    assert profile["static_inflation_radius"] > profile["static_inscribed_radius"]


def test_dddmr_cuboids_match_the_envelope():
    """All five DDDMR cuboids must use the same eight corners as the envelope."""
    profile = _profile()
    low = profile["collision_aabb_min"]
    high = profile["collision_aabb_max"]
    expected = {
        "flb": (high[0], high[1], low[2]), "frb": (high[0], low[1], low[2]),
        "flt": (high[0], high[1], high[2]), "frt": (high[0], low[1], high[2]),
        "blb": (low[0], high[1], low[2]), "brb": (low[0], low[1], low[2]),
        "blt": (low[0], high[1], high[2]), "brt": (low[0], low[1], high[2]),
    }
    with open(DDDMR_CONFIG_PATH, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    cuboids = []

    def collect(node):
        if isinstance(node, dict):
            for key, value in node.items():
                if key in ("flb", "frb", "flt", "frt", "blb", "brb", "blt", "brt"):
                    if not cuboids or set(cuboids[-1]) == set(expected):
                        cuboids.append({})
                    cuboids[-1][key] = value
                else:
                    collect(value)
        elif isinstance(node, list):
            for item in node:
                collect(item)

    collect(data)
    assert len(cuboids) == 5, len(cuboids)
    for index, cuboid in enumerate(cuboids):
        assert set(cuboid) == set(expected), index
        for key, corner in expected.items():
            for actual, wanted in zip(cuboid[key], corner):
                assert math.isclose(actual, wanted, abs_tol=1.0e-3), (index, key)
