"""Validate and resolve an external bundle without importing ROS or simulators."""

import argparse
from copy import deepcopy
import hashlib
import math
import os
from pathlib import Path
import re
import tempfile

import yaml


class BundleError(ValueError):
    """A deployment bundle is incomplete or internally inconsistent."""


def _mapping(value, label):
    if not isinstance(value, dict):
        raise BundleError(f"{label} must be a YAML mapping")
    return value


def _yaml(path):
    try:
        return _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), path.name)
    except (OSError, yaml.YAMLError) as error:
        raise BundleError(f"Cannot read {path.name}: {error}") from error


def _number(value, label, minimum=0.0, allow_equal=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BundleError(f"{label} must be a measured finite number, not {value!r}")
    value = float(value)
    if not math.isfinite(value) or (value < minimum if allow_equal else value <= minimum):
        comparison = ">=" if allow_equal else ">"
        raise BundleError(f"{label} must be finite and {comparison} {minimum}")
    return value


def _no_nulls(value, label):
    if value is None or (isinstance(value, str) and "__" in value):
        raise BundleError(f"Unfilled measured value: {label}")
    if isinstance(value, dict):
        for key, item in value.items():
            _no_nulls(item, f"{label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _no_nulls(item, f"{label}[{index}]")


def _real_time(value, label):
    if not isinstance(value, dict):
        return
    for key, item in value.items():
        if key == "use_sim_time" and item is not False:
            raise BundleError(f"{label}.use_sim_time must be the YAML boolean false")
        _real_time(item, f"{label}.{key}")


def _node(document, name, filename):
    _real_time(document, filename)
    return deepcopy(_mapping(
        _mapping(document.get(name), f"{filename}.{name}").get("ros__parameters"),
        f"{filename}.{name}.ros__parameters"))


def _frame(value, label):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/-]*", value):
        raise BundleError(f"{label} must be a TF frame name without a leading slash")
    return value


def _topic(value, label):
    if not isinstance(value, str) or not re.fullmatch(r"/[A-Za-z_][A-Za-z0-9_/]*", value):
        raise BundleError(f"{label} must be an absolute ROS topic/service name")
    return value


def _asset(root, relative_path, extension, label):
    if not isinstance(relative_path, str) or not relative_path:
        raise BundleError(f"{label} must name a file inside the external bundle")
    relative = Path(relative_path)
    if relative.is_absolute():
        raise BundleError(f"{label} must be relative to bundle_dir for deployment")
    asset = (root / relative).resolve()
    if not asset.is_relative_to(root) or asset.suffix.lower() != extension:
        raise BundleError(f"{label} must stay inside bundle_dir and end in {extension}")
    if not asset.is_file() or asset.stat().st_size == 0:
        raise BundleError(f"{label} missing or empty: {asset}")
    return asset


def _header(path, terminator):
    lines = []
    with path.open("rb") as stream:
        for _ in range(512):
            raw = stream.readline(8192)
            if not raw:
                break
            try:
                line = raw.decode("ascii").strip()
            except UnicodeDecodeError as error:
                raise BundleError(f"Invalid text header: {path.name}") from error
            lines.append(line)
            if line == terminator or (terminator == "DATA" and line.startswith("DATA ")):
                if not stream.read(1):
                    raise BundleError(f"{path.name} has a header but no point/mesh payload")
                return lines
    raise BundleError(f"Missing {terminator} header boundary: {path.name}")


def _geometry_headers(mesh, background):
    ply = _header(mesh, "end_header")
    if not ply or ply[0] != "ply":
        raise BundleError("Navigation map must be a PLY file, not an LFS pointer")
    elements = {}
    for line in ply:
        words = line.split()
        if len(words) == 3 and words[0] == "element":
            try:
                elements[words[1]] = int(words[2])
            except ValueError as error:
                raise BundleError("PLY element counts must be integers") from error
    if elements.get("vertex", 0) < 3 or elements.get("face", 0) < 1:
        raise BundleError("Navigation PLY needs >= 3 vertices and >= 1 face")
    pcd = {}
    for line in _header(background, "DATA"):
        if line and not line.startswith("#"):
            words = line.split()
            pcd[words[0]] = words[1:]
    if not {"x", "y", "z"}.issubset(set(pcd.get("FIELDS", []))):
        raise BundleError("Background PCD must contain x/y/z fields")
    try:
        points = int(pcd["POINTS"][0])
    except (KeyError, IndexError, ValueError) as error:
        raise BundleError("Background PCD must declare a nonzero POINTS count") from error
    if points <= 0 or pcd.get("DATA", []) not in (["ascii"], ["binary"], ["binary_compressed"]):
        raise BundleError("Background PCD must contain points and a supported DATA format")


def _hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def flatten_parameters(parameters, prefix=""):
    """Produce explicit ROS dotted names from the standalone nested config."""
    result = {}
    for key, value in parameters.items():
        name = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            result.update(flatten_parameters(value, name))
        else:
            result[name] = value
    return result


def load_bundle(bundle_dir, cache_dir=None):
    """Check measurements/assets and return the four nodes' effective parameters.

    A temporary write probes cache_dir, and is removed before this returns.
    This verifies configuration consistency, not calibration or actual hardware.
    """
    if not bundle_dir:
        raise BundleError("bundle_dir is required; copy/fill the installed config templates first")
    root = Path(bundle_dir).expanduser().resolve()
    if not root.is_dir():
        raise BundleError(f"bundle_dir does not exist: {root}")
    robot = _yaml(root / "robot.yaml")
    _no_nulls(robot, "robot.yaml")
    _real_time(robot, "robot.yaml")
    if type(robot.get("schema_version")) is not int or robot["schema_version"] != 1:
        raise BundleError("robot.yaml schema_version must be 1")
    kind = robot.get("deployment_kind")
    if kind not in ("hardware", "desktop_smoke"):
        raise BundleError("deployment_kind must be hardware or desktop_smoke")
    integration = _mapping(robot.get("integration"), "robot.yaml.integration")
    for key in ("pcl_source_patch_verified", "background_alignment_verified",
                "localization_verified", "tf_ownership_verified"):
        if integration.get(key) is not True:
            raise BundleError(f"Complete and record {key} before setting it true")

    frames = _mapping(robot.get("frames"), "robot.yaml.frames")
    for name in ("map", "odom", "base", "lidar"):
        _frame(frames.get(name), f"frames.{name}")
    if len({frames[key] for key in ("map", "odom", "base", "lidar")}) != 4:
        raise BundleError("map, odom, base and lidar frame names must be distinct")
    body = _mapping(robot.get("robot"), "robot.yaml.robot")
    for key in ("length_m", "width_m", "height_m", "collision_radius_m",
                "height_diff_threshold_m"):
        _number(body.get(key), f"robot.{key}")
    margin = _number(body.get("safety_margin_m"), "robot.safety_margin_m", allow_equal=True)
    if not isinstance(body.get("holonomic"), bool):
        raise BundleError("robot.holonomic must be a YAML boolean")
    rectangle_radius = math.hypot(body["length_m"], body["width_m"]) / 2.0
    if body["collision_radius_m"] + 1e-9 < rectangle_radius:
        raise BundleError("collision_radius_m must cover the measured rectangular envelope")
    hard_radius = body["collision_radius_m"] + margin

    sensor = _mapping(robot.get("sensors"), "robot.yaml.sensors")
    offset = sensor.get("offset_xyz")
    if not isinstance(offset, list) or len(offset) != 3 or any(
            isinstance(item, bool) or not isinstance(item, (int, float))
            or not math.isfinite(item) for item in offset):
        raise BundleError("sensors.offset_xyz must be three finite metre offsets")
    for key in ("min_range_m", "max_range_m", "horizontal_fov_deg"):
        _number(sensor.get(key), f"sensors.{key}", allow_equal=key == "min_range_m")
    if sensor["max_range_m"] <= sensor["min_range_m"] or sensor["horizontal_fov_deg"] > 360:
        raise BundleError("Sensor ranges/FOV are inconsistent")
    for key in ("vertical_fov_min_deg", "vertical_fov_max_deg"):
        value = sensor.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise BundleError(f"sensors.{key} must be a finite angle")
        if not -90 < value < 90:
            raise BundleError(f"sensors.{key} must be strictly between -90 and 90 degrees")
    if sensor["vertical_fov_min_deg"] >= sensor["vertical_fov_max_deg"]:
        raise BundleError("Vertical FOV minimum must be less than maximum")

    perception = _mapping(robot.get("perception"), "robot.yaml.perception")
    for key in ("distance_threshold_m", "self_filter_radius_m",
                "obstacle_projection_height_m", "obstacle_keep_time_s",
                "max_observation_age_s", "static_inflation_radius_m",
                "dynamic_inflation_radius_m", "max_future_s"):
        _number(perception.get(key), f"perception.{key}")
    for key in ("obstacle_keep_time_s", "max_observation_age_s"):
        if perception[key] > 1e9:
            raise BundleError(f"perception.{key} must be <= 1e9 seconds for TemporalObstacleLayer")
    if perception["max_future_s"] > 0.001:
        raise BundleError("perception.max_future_s must be <= 0.001: TemporalObstacleLayer has a fixed 1 ms future limit")
    for key in ("static_inflation_radius_m", "dynamic_inflation_radius_m"):
        if perception[key] < hard_radius:
            raise BundleError(f"perception.{key} must cover collision radius + margin ({hard_radius:.3f} m)")
    limits = _mapping(robot.get("motion"), "robot.yaml.motion")
    for key in ("max_linear_velocity", "max_angular_velocity",
                "max_linear_acceleration", "max_angular_acceleration"):
        _number(limits.get(key), f"motion.{key}")

    nav = _node(_yaml(root / "meshnav_real.yaml"), "move_base_flex", "meshnav_real.yaml")
    pcl = _node(_yaml(root / "pcl_real.yaml"), "obstacle_sub", "pcl_real.yaml")
    interfaces = _yaml(root / "interfaces_real.yaml")
    gateway = _node(interfaces, "command_gateway", "interfaces_real.yaml")
    navigator = _node(interfaces, "meshnav_navigator", "interfaces_real.yaml")
    allowed_gateway = {
        "use_sim_time", "input_topic", "output_topic", "odom_topic", "cloud_topic",
        "obstacle_topic", "hardware_ready_topic", "estop_topic", "localization_health_topic",
        "cloud_qos", "odom_qos", "obstacle_qos", "map_frame", "odom_frame", "base_frame",
        "lidar_frame", "publish_frequency_hz", "command_timeout_s", "data_timeout_s",
        "max_data_age_s", "health_timeout_s", "tf_timeout_s", "max_future_s",
        "max_linear_velocity", "max_angular_velocity", "max_linear_acceleration",
        "max_angular_acceleration",
    }
    unexpected = set(gateway) - allowed_gateway
    if unexpected:
        raise BundleError(f"Unknown gateway parameter(s): {sorted(unexpected)}; startup is always disabled")
    for key in ("input_topic", "output_topic", "odom_topic", "cloud_topic", "obstacle_topic",
                "hardware_ready_topic", "estop_topic", "localization_health_topic"):
        _topic(gateway.get(key), f"command_gateway.{key}")
    if gateway["input_topic"] != "/nav/cmd_vel":
        raise BundleError("Keep the normalized navigation output input_topic: /nav/cmd_vel")
    if len({gateway[key] for key in ("input_topic", "output_topic", "odom_topic", "cloud_topic",
            "obstacle_topic", "hardware_ready_topic", "estop_topic", "localization_health_topic")}) != 8:
        raise BundleError("Each monitored/input/output topic must have its own name")
    if kind == "desktop_smoke" and not gateway["output_topic"].startswith("/meshnav_smoke/"):
        raise BundleError("desktop_smoke can publish only to /meshnav_smoke/... test output")
    for key in ("cloud_qos", "odom_qos", "obstacle_qos"):
        if gateway.get(key) not in ("reliable", "best_effort"):
            raise BundleError(f"command_gateway.{key} must be reliable or best_effort")
    if gateway["obstacle_qos"] != "reliable":
        raise BundleError("PCL output is Reliable; keep gateway obstacle_qos: reliable")
    if gateway["odom_qos"] != "reliable":
        raise BundleError("MBF odometry subscribes Reliable; normalize driver odometry to Reliable")
    for key in ("publish_frequency_hz", "command_timeout_s", "data_timeout_s",
                "max_data_age_s", "health_timeout_s", "tf_timeout_s"):
        gateway[key] = _number(gateway.get(key), f"command_gateway.{key}")
    for key in ("pose_timeout", "path_progress_window", "action_timeout",
                "replan_patience", "replan_retry_period"):
        navigator[key] = _number(navigator.get(key), f"meshnav_navigator.{key}")
    navigator["planner_frequency"] = _number(navigator.get("planner_frequency"), "meshnav_navigator.planner_frequency")
    _topic(navigator.get("goal_topic"), "meshnav_navigator.goal_topic")
    expected_interfaces = {
        "get_path_action": "/move_base_flex/get_path",
        "exe_path_action": "/move_base_flex/exe_path",
        "check_path_service": "/move_base_flex/check_path_cost",
        "planner": "mesh_planner", "controller": "mesh_controller",
    }
    for key, expected in expected_interfaces.items():
        if navigator.get(key) != expected:
            raise BundleError(f"Keep meshnav_navigator.{key}: {expected}")

    assets = _mapping(robot.get("map"), "robot.yaml.map")
    version = assets.get("version")
    if not isinstance(version, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", version):
        raise BundleError("map.version must contain only letters, numbers, underscores or hyphens")
    mesh_file = _asset(root, assets.get("mesh_file"), ".ply", "map.mesh_file")
    background_file = _asset(root, assets.get("background_file"), ".pcd", "map.background_file")
    _geometry_headers(mesh_file, background_file)
    for key, path in (("mesh_sha256", mesh_file), ("background_sha256", background_file)):
        expected = assets.get(key)
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", expected):
            raise BundleError(f"map.{key} must be the sha256sum of the frozen asset")
        if _hash(path) != expected.lower():
            raise BundleError(f"Hash mismatch for {path.name}; update assets and the map version together")

    if nav.get("planners") != ["mesh_planner"] or nav.get("controllers") != ["mesh_controller"]:
        raise BundleError("This profile requires the single mesh_planner/MeshController pair")
    planner = _mapping(nav.get("mesh_planner"), "mesh_planner")
    controller = _mapping(nav.get("mesh_controller"), "mesh_controller")
    if planner.get("type") != "cvp_mesh_planner/CVPMeshPlanner" or controller.get("type") != "mesh_controller/MeshController":
        raise BundleError("This standalone profile is CVPMeshPlanner + MeshController")
    mesh = _mapping(nav.get("mesh_map"), "mesh_map")
    expected_layers = ["border", "height_diff", "roughness", "static_combination",
                       "static_inflation", "obstacle", "obstacle_inflation", "final"]
    if mesh.get("layers") != expected_layers or mesh.get("default_layer") != "final":
        raise BundleError("Keep the complete static → temporal obstacle → inflation → final layer chain")
    obstacle = _mapping(mesh.get("obstacle"), "mesh_map.obstacle")
    if obstacle.get("type") != "mesh_layers/TemporalObstacleLayer":
        raise BundleError("Real profile requires TemporalObstacleLayer")
    if (obstacle.get("clearing_mode") != "stvl" or type(obstacle.get("decay_model")) is not int
            or obstacle["decay_model"] != 0 or obstacle.get("decay_acceleration") != 0):
        raise BundleError("Commission this profile with linear STVL decay and no accelerated clearing")
    for key in ("history_voxel_size", "pose_jump_translation", "pose_jump_rotation_deg"):
        obstacle[key] = _number(obstacle.get(key), f"mesh_map.obstacle.{key}")
    if obstacle["pose_jump_rotation_deg"] > 180.0:
        raise BundleError("mesh_map.obstacle.pose_jump_rotation_deg must be <= 180 degrees")
    for key in ("pose_jump_max_interval", "visible_keep_time"):
        if key in obstacle:
            obstacle[key] = _number(obstacle[key], f"mesh_map.obstacle.{key}", allow_equal=True)
            if obstacle[key] > 1e9:
                raise BundleError(f"mesh_map.obstacle.{key} must be <= 1e9 seconds")
    obstacle["tf_tolerance"] = _number(obstacle.get("tf_tolerance"), "mesh_map.obstacle.tf_tolerance", allow_equal=True)
    if obstacle["tf_tolerance"] > 60.0:
        raise BundleError("mesh_map.obstacle.tf_tolerance must be <= 60 seconds")
    # The plugin declares this as double even when a YAML user wrote integer 0.
    obstacle["decay_acceleration"] = 0.0
    if (mesh.get("final", {}).get("type") != "mesh_layers/MaxCombinationLayer"
            or mesh["final"].get("inputs") != ["static_inflation", "obstacle_inflation"]):
        raise BundleError("final must combine static_inflation and obstacle_inflation")
    for name in ("static_inflation", "obstacle_inflation"):
        layer = _mapping(mesh.get(name), f"mesh_map.{name}")
        if layer.get("type") != "mesh_layers/InflationLayer":
            raise BundleError(f"mesh_map.{name} must load InflationLayer")
        expected_input = "static_combination" if name == "static_inflation" else "obstacle"
        if layer.get("inputs") != [expected_input]:
            raise BundleError(f"mesh_map.{name} must inflate only {expected_input}")
        layer["inscribed_radius"] = float(hard_radius)
        layer["inscribed_value"] = 1.0
    cost_limit = _number(planner.get("cost_limit"), "mesh_planner.cost_limit")
    if cost_limit >= 1.0:
        raise BundleError("mesh_planner.cost_limit must be less than the 1.0 hard-inflation cost")
    for key in ("controller_frequency", "planner_patience", "controller_patience",
                "path_check_step", "path_check_max_distance", "dist_tolerance", "angle_tolerance"):
        nav[key] = _number(nav.get(key), f"move_base_flex.{key}")
    planner["step_width"] = _number(planner.get("step_width"), "mesh_planner.step_width")
    planner["cost_limit"] = cost_limit
    for key in ("lin_vel_factor", "ang_vel_factor", "arrival_fading"):
        controller[key] = _number(controller.get(key), f"mesh_controller.{key}")
    if nav.get("force_stop_at_goal") is not True or nav.get("force_stop_on_cancel") is not True:
        raise BundleError("Keep force_stop_at_goal and force_stop_on_cancel true")

    nav.update(use_sim_time=False, global_frame=frames["map"], odom_frame=frames["odom"], robot_frame=frames["base"],
               odom_topic=gateway["odom_topic"], planner_frequency=navigator["planner_frequency"])
    controller.update(holonomic=body["holonomic"], max_lin_velocity=float(limits["max_linear_velocity"]),
                      max_ang_velocity=float(limits["max_angular_velocity"]))
    mesh["mesh_file"] = str(mesh_file)
    mesh["height_diff"]["threshold"] = float(body["height_diff_threshold_m"])
    mesh["static_inflation"]["inflation_radius"] = float(perception["static_inflation_radius_m"])
    mesh["obstacle_inflation"]["inflation_radius"] = float(perception["dynamic_inflation_radius_m"])
    obstacle.update(topic=gateway["obstacle_topic"], qos="Reliable", axis_frame=frames["map"],
                    down_axis=[0.0, 0.0, -1.0], sensor_frame=frames["lidar"],
                    sensor_offset_xyz=[float(item) for item in offset],
                    robot_height=float(perception["obstacle_projection_height_m"]),
                    max_obstacle_dist=float(sensor["max_range_m"]),
                    min_sensor_range=float(sensor["min_range_m"]),
                    horizontal_fov_deg=float(sensor["horizontal_fov_deg"]),
                    vertical_fov_min_deg=float(sensor["vertical_fov_min_deg"]),
                    vertical_fov_max_deg=float(sensor["vertical_fov_max_deg"]),
                    obstacle_keep_time=float(perception["obstacle_keep_time_s"]),
                    max_observation_age=float(perception["max_observation_age_s"]),
                    localization_reference_frame=frames["odom"])
    pcl.update(use_sim_time=False, input_cloud_topic=gateway["cloud_topic"],
               output_cloud_topic=gateway["obstacle_topic"], base_frame=frames["base"],
               lidar_frame=frames["lidar"], odom_frame=frames["map"], pcd_file_path=str(background_file),
               distance_threshold=float(perception["distance_threshold_m"]),
               robot_radius=float(perception["self_filter_radius_m"]),
               sensor_offset_xyz=[float(item) for item in offset],
               input_qos="Reliable" if gateway["cloud_qos"] == "reliable" else "BestEffort",
               max_cloud_age_s=float(perception["max_observation_age_s"]),
               max_future_s=float(perception["max_future_s"]))
    gateway.update(use_sim_time=False, map_frame=frames["map"], odom_frame=frames["odom"],
                   base_frame=frames["base"], lidar_frame=frames["lidar"],
                   max_data_age_s=float(perception["max_observation_age_s"]),
                   max_future_s=float(perception["max_future_s"]))
    gateway.update({key: float(value) for key, value in limits.items()})
    navigator["use_sim_time"] = False
    if gateway["command_timeout_s"] <= 1.0 / nav["controller_frequency"]:
        raise BundleError("command_timeout_s must exceed one navigation control period")

    if not cache_dir:
        cache_dir = Path(os.environ.get("ROS_HOME", str(Path.home() / ".ros"))) / "meshnav" / "cache"
    cache = Path(cache_dir).expanduser().resolve()
    try:
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=".meshnav_write_probe_", dir=cache):
            pass
    except OSError as error:
        raise BundleError(f"cache_dir must be writable: {cache}: {error}") from error
    config_hash = hashlib.sha256(yaml.safe_dump(nav, sort_keys=True).encode()).hexdigest()[:12]
    mesh["mesh_working_file"] = str(cache / f"{version}_{assets['mesh_sha256'][:12]}_{config_hash}.h5")
    for label, params in (("navigation", nav), ("PCL", pcl), ("gateway", gateway), ("navigator", navigator)):
        _no_nulls(params, label)
        _real_time(params, label)
    return {"robot": robot, "nav": nav, "pcl": pcl, "gateway": gateway,
            "navigator": navigator, "cache_dir": str(cache), "map_file": str(mesh_file),
            "background_file": str(background_file)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle-dir", required=True)
    parser.add_argument("--cache-dir", default=None)
    parser.add_argument("--dump-effective", action="store_true", help="Print resolved ROS parameters")
    args = parser.parse_args()
    try:
        bundle = load_bundle(args.bundle_dir, args.cache_dir)
    except BundleError as error:
        parser.exit(2, f"Bundle rejected: {error}\n")
    print(f"Bundle configuration/assets valid: {Path(args.bundle_dir).resolve()}")
    print(f"Writable map cache: {bundle['nav']['mesh_map']['mesh_working_file']}")
    print(f"Gateway starts disabled; Twist output: {bundle['gateway']['output_topic']}")
    print("Live TF, calibration, stop behavior and truthful health inputs still require interface checks.")
    if args.dump_effective:
        print(yaml.safe_dump({
            "move_base_flex": {"ros__parameters": flatten_parameters(bundle["nav"])},
            "obstacle_sub": {"ros__parameters": bundle["pcl"]},
            "meshnav_navigator": {"ros__parameters": bundle["navigator"]},
            "command_gateway": {"ros__parameters": bundle["gateway"]},
        }, sort_keys=False))
    return 0


if __name__ == "__main__":
    main()
