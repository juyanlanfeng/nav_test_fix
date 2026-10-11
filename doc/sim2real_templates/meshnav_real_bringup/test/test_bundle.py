"""Test deployable assets and reject configurations that cross hardware boundaries."""

from pathlib import Path

import pytest
import yaml

from meshnav_real_bringup.bundle import BundleError, flatten_parameters, load_bundle
from meshnav_real_bringup.smoke import make_bundle_main


@pytest.fixture
def smoke_bundle(tmp_path):
    bundle = tmp_path / "bundle"
    assert make_bundle_main(["--output", str(bundle)]) == 0
    return bundle


def mutate(bundle, filename, change):
    path = bundle / filename
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    change(document)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def test_valid_bundle_resolves_shared_hardware_contract(smoke_bundle, tmp_path):
    loaded = load_bundle(smoke_bundle, tmp_path / "cache")
    nav, pcl, gateway = loaded["nav"], loaded["pcl"], loaded["gateway"]
    assert gateway["output_topic"] == "/meshnav_smoke/chassis_cmd"
    assert pcl["odom_frame"] == nav["global_frame"] == gateway["map_frame"] == "map"
    assert nav["robot_frame"] == pcl["base_frame"] == gateway["base_frame"] == "base_footprint"
    assert nav["odom_topic"] == gateway["odom_topic"] == "/odom"
    assert nav["odom_frame"] == gateway["odom_frame"] == "odom"
    assert pcl["lidar_frame"] == nav["mesh_map"]["obstacle"]["sensor_frame"] == "lidar_link"
    assert pcl["sensor_offset_xyz"] == nav["mesh_map"]["obstacle"]["sensor_offset_xyz"] == [0.0, 0.0, 0.0]
    assert pcl["input_cloud_topic"] == gateway["cloud_topic"] == "/cloud"
    assert pcl["output_cloud_topic"] == nav["mesh_map"]["obstacle"]["topic"] == gateway["obstacle_topic"]
    assert gateway["max_linear_velocity"] == nav["mesh_controller"]["max_lin_velocity"]
    assert gateway["max_angular_velocity"] == nav["mesh_controller"]["max_ang_velocity"]
    assert nav["mesh_map"]["static_inflation"]["inscribed_radius"] == pytest.approx(0.35)
    assert nav["mesh_map"]["obstacle_inflation"]["inscribed_radius"] == pytest.approx(0.35)
    assert all(loaded[name]["use_sim_time"] is False for name in ("nav", "pcl", "gateway", "navigator"))
    assert Path(loaded["map_file"]).is_absolute()
    assert Path(loaded["background_file"]).is_absolute()
    assert Path(nav["mesh_map"]["mesh_working_file"]).parent == tmp_path / "cache"
    assert not list((tmp_path / "cache").glob(".meshnav_write_probe_*"))
    effective = flatten_parameters(nav)
    assert effective["mesh_map.obstacle.type"] == "mesh_layers/TemporalObstacleLayer"
    assert effective["mesh_map.mesh_file"] == loaded["map_file"]


@pytest.mark.parametrize("filename,change,reason", [
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"].update(use_sim_time=True),
     "use_sim_time"),
    ("pcl_real.yaml", lambda d: d["obstacle_sub"]["ros__parameters"].update(use_sim_time=True),
     "use_sim_time"),
    ("interfaces_real.yaml", lambda d: d["command_gateway"]["ros__parameters"].update(start_enabled=True),
     "startup is always disabled"),
    ("interfaces_real.yaml", lambda d: d["command_gateway"]["ros__parameters"].update(output_topic="/hardware/cmd_vel"),
     "desktop_smoke"),
    ("interfaces_real.yaml", lambda d: d["command_gateway"]["ros__parameters"].update(output_topic="/cloud"),
     "own name"),
    ("interfaces_real.yaml", lambda d: d["command_gateway"]["ros__parameters"].update(odom_qos="best_effort"),
     "MBF odometry subscribes Reliable"),
    ("robot.yaml", lambda d: d["map"].update(mesh_sha256="0" * 64), "Hash mismatch"),
    ("robot.yaml", lambda d: d["map"].update(mesh_file="../escape.ply"), "stay inside"),
    ("robot.yaml", lambda d: d["robot"].update(collision_radius_m=None), "Unfilled measured"),
    ("robot.yaml", lambda d: d["robot"].update(collision_radius_m=0.10), "rectangular envelope"),
    ("robot.yaml", lambda d: d["perception"].update(dynamic_inflation_radius_m=0.10), "collision radius"),
    ("robot.yaml", lambda d: d["perception"].update(max_future_s=0.0), "must be finite"),
    ("robot.yaml", lambda d: d["perception"].update(max_future_s=0.010), "fixed 1 ms future"),
    ("robot.yaml", lambda d: d["motion"].update(max_linear_velocity=float("nan")), "must be finite"),
    ("robot.yaml", lambda d: d["frames"].update(lidar="/lidar_link"), "without a leading slash"),
    ("robot.yaml", lambda d: d["frames"].update(odom="map"), "distinct"),
    ("robot.yaml", lambda d: d["integration"].update(pcl_source_patch_verified=False), "record"),
    ("robot.yaml", lambda d: d["sensors"].update(vertical_fov_min_deg=30.0, vertical_fov_max_deg=20.0),
     "minimum must be less"),
    ("robot.yaml", lambda d: d["sensors"].update(vertical_fov_min_deg=-90.0), "strictly between"),
    ("robot.yaml", lambda d: d["sensors"].update(vertical_fov_max_deg=90.0), "strictly between"),
    ("robot.yaml", lambda d: d["perception"].update(obstacle_keep_time_s=1e10), "<= 1e9"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["obstacle"].update(pose_jump_rotation_deg=181.0),
     "<= 180 degrees"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["obstacle"].update(tf_tolerance=61.0),
     "<= 60 seconds"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["obstacle"].update(history_voxel_size=0.0),
     "must be finite"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["obstacle"].update(decay_model=False),
     "linear STVL decay"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_controller"].update(type="mesh_mppi/DiffDriveMPC"),
     "CVPMeshPlanner"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_planner"].update(cost_limit=1.0),
     "less than"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["final"].update(type="mesh_layers/AvgCombinationLayer"),
     "final must combine"),
    ("meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["obstacle_inflation"].update(inputs=["static_combination"]),
     "must inflate only obstacle"),
])
def test_invalid_bundle_cannot_become_launch_parameters(smoke_bundle, tmp_path, filename, change, reason):
    mutate(smoke_bundle, filename, change)
    with pytest.raises(BundleError, match=reason):
        load_bundle(smoke_bundle, tmp_path / "cache")


def test_unfilled_hardware_template_cannot_launch(tmp_path):
    template = Path(__file__).resolve().parents[1] / "config"
    with pytest.raises(BundleError, match="Unfilled measured"):
        load_bundle(template, tmp_path / "cache")


def test_missing_asset_is_rejected_before_navigation_starts(smoke_bundle, tmp_path):
    (smoke_bundle / "maps/smoke_background.pcd").unlink()
    with pytest.raises(BundleError, match="missing or empty"):
        load_bundle(smoke_bundle, tmp_path / "cache")


def test_lfs_pointer_is_rejected_as_mesh(smoke_bundle, tmp_path):
    (smoke_bundle / "maps/smoke_nav.ply").write_text("version https://git-lfs.github.com/spec/v1\n")
    with pytest.raises(BundleError, match="header boundary"):
        load_bundle(smoke_bundle, tmp_path / "cache")


def test_nonempty_point_header_without_payload_is_rejected(smoke_bundle, tmp_path):
    path = smoke_bundle / "maps/smoke_background.pcd"
    header = path.read_text().split("DATA ascii\n", 1)[0]
    path.write_text(header + "DATA ascii\n")
    with pytest.raises(BundleError, match="no point/mesh payload"):
        load_bundle(smoke_bundle, tmp_path / "cache")


def test_changed_navigation_parameters_choose_new_h5_cache(smoke_bundle, tmp_path):
    first = load_bundle(smoke_bundle, tmp_path / "cache")["nav"]["mesh_map"]["mesh_working_file"]
    mutate(smoke_bundle, "meshnav_real.yaml", lambda d: d["move_base_flex"]["ros__parameters"]["mesh_map"]["roughness"].update(threshold=0.8))
    second = load_bundle(smoke_bundle, tmp_path / "cache")["nav"]["mesh_map"]["mesh_working_file"]
    assert first != second
    assert Path(first).parent == Path(second).parent


def test_custom_continuous_odom_frame_reaches_mbf_and_gateway(smoke_bundle, tmp_path):
    mutate(smoke_bundle, "robot.yaml", lambda d: d["frames"].update(odom="wheel_odom"))
    loaded = load_bundle(smoke_bundle, tmp_path / "cache")
    assert loaded["nav"]["odom_frame"] == loaded["gateway"]["odom_frame"] == "wheel_odom"
    assert loaded["nav"]["mesh_map"]["obstacle"]["localization_reference_frame"] == "wheel_odom"
    assert loaded["pcl"]["odom_frame"] == "map"
