#!/usr/bin/env python3

from __future__ import annotations

import copy
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from jie_map_msgs.srv import (
    ExportNavigationSnapshot,
    GetNavigationMapMeta,
    LoadNavigationMapPackage,
    SaveNavigationMapPackage,
)
from octomap_msgs.msg import Octomap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker


class MapPackageManager(Node):
    def __init__(self) -> None:
        super().__init__("map_package_manager")

        self.declare_parameter("octomap_topic", "/octomap")
        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_topic", "/preblocked_cells_markers")
        self.declare_parameter(
            "external_preblocked_topic", "/edited_preblocked_cells_markers"
        )
        self.declare_parameter("traversable_topic", "/traversable_cells_markers")
        self.declare_parameter("risk_cost_topic", "/risk_cost_cells")
        self.declare_parameter("planner_meta_service", "/jie_path_node/get_meta")
        self.declare_parameter("planner_export_service", "/jie_path_node/export_snapshot")
        self.declare_parameter("planner_parameter_service", "/jie_path_node/set_parameters")
        self.declare_parameter(
            "selector_parameter_service", "/web_click_selector/set_parameters"
        )
        self.declare_parameter("publish_cached_derived_layers_on_load", False)
        self.declare_parameter("autoload_package_path", "")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        callback_group = ReentrantCallbackGroup()

        self._latest_octomap: Optional[Octomap] = None
        self._latest_occupied: Optional[Marker] = None
        self._latest_preblocked: Optional[Marker] = None
        self._latest_external_preblocked: Optional[Marker] = None
        self._latest_traversable: Optional[Marker] = None
        self._latest_risk_cost: Optional[PointCloud2] = None

        self.create_subscription(
            Octomap,
            self.get_parameter("octomap_topic").value,
            self._on_octomap,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("occupied_marker_topic").value,
            self._on_occupied,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("preblocked_topic").value,
            self._on_preblocked,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("external_preblocked_topic").value,
            self._on_external_preblocked,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("traversable_topic").value,
            self._on_traversable,
            qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("risk_cost_topic").value,
            self._on_risk_cost,
            qos,
            callback_group=callback_group,
        )

        self.octomap_pub = self.create_publisher(Octomap, self.get_parameter("octomap_topic").value, qos)
        self.occupied_pub = self.create_publisher(
            Marker, self.get_parameter("occupied_marker_topic").value, qos
        )
        self.preblocked_pub = self.create_publisher(
            Marker, self.get_parameter("preblocked_topic").value, qos
        )
        self.external_preblocked_pub = self.create_publisher(
            Marker, self.get_parameter("external_preblocked_topic").value, qos
        )
        self.traversable_pub = self.create_publisher(
            Marker, self.get_parameter("traversable_topic").value, qos
        )
        self.risk_cost_pub = self.create_publisher(
            PointCloud2, self.get_parameter("risk_cost_topic").value, qos
        )

        self.meta_client = self.create_client(
            GetNavigationMapMeta,
            self.get_parameter("planner_meta_service").value,
            callback_group=callback_group,
        )
        self.export_client = self.create_client(
            ExportNavigationSnapshot,
            self.get_parameter("planner_export_service").value,
            callback_group=callback_group,
        )
        self.planner_parameter_client = self.create_client(
            SetParameters,
            self.get_parameter("planner_parameter_service").value,
            callback_group=callback_group,
        )
        self.selector_parameter_client = self.create_client(
            SetParameters,
            self.get_parameter("selector_parameter_service").value,
            callback_group=callback_group,
        )

        self.create_service(
            SaveNavigationMapPackage,
            "~/save_package",
            self._handle_save_package,
            callback_group=callback_group,
        )
        self.create_service(
            LoadNavigationMapPackage,
            "~/load_package",
            self._handle_load_package,
            callback_group=callback_group,
        )
        self._autoload_timer = self.create_timer(1.0, self._autoload_package_once)

        self.get_logger().info(
            "map_package_manager started. save_service=~/save_package load_service=~/load_package"
        )

    def _on_octomap(self, msg: Octomap) -> None:
        self._latest_octomap = copy.deepcopy(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_occupied = copy.deepcopy(msg)

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_preblocked = copy.deepcopy(msg)

    def _on_external_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_external_preblocked = copy.deepcopy(msg)

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            self._latest_traversable = copy.deepcopy(msg)

    def _on_risk_cost(self, msg: PointCloud2) -> None:
        self._latest_risk_cost = copy.deepcopy(msg)

    def _marker_points_to_numpy(self, marker: Marker) -> np.ndarray:
        return np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)

    def _make_marker_from_points(
        self,
        frame_id: str,
        ns: str,
        scale: np.ndarray,
        points: np.ndarray,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for xyz in points:
            point = Point()
            point.x = float(xyz[0])
            point.y = float(xyz[1])
            point.z = float(xyz[2])
            marker.points.append(point)
        return marker

    def _wait_for_future(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _call_export_snapshot(self) -> tuple[bool, str, Optional[ExportNavigationSnapshot.Response]]:
        if not self.export_client.wait_for_service(timeout_sec=1.0):
            return False, "planner export service unavailable", None
        request = ExportNavigationSnapshot.Request()
        request.recompute_layers = True
        future = self.export_client.call_async(request)
        result = self._wait_for_future(future, 60.0)
        if result is None:
            return False, "planner export service timed out", None
        return result.success, result.message, result

    def _call_get_meta(self) -> tuple[bool, str, Optional[GetNavigationMapMeta.Response]]:
        if not self.meta_client.wait_for_service(timeout_sec=1.0):
            return False, "planner meta service unavailable", None
        future = self.meta_client.call_async(GetNavigationMapMeta.Request())
        result = self._wait_for_future(future, 5.0)
        if result is None:
            return False, "planner meta service timed out", None
        return result.success, result.message, result

    @staticmethod
    def _planner_parameter(name: str, value) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        if isinstance(value, bool):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=value
            )
        elif isinstance(value, int):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=value
            )
        else:
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)
            )
        return parameter

    def _apply_planner_profile(self, planner_meta: dict) -> tuple[bool, str]:
        if not self.planner_parameter_client.wait_for_service(timeout_sec=2.0):
            return False, "planner parameter service unavailable"

        # Old packages did not store the anisotropic fields. Explicitly reset
        # them to legacy mode instead of retaining a profile from a previously
        # loaded RMUC package.
        values = {
            "robot_radius": float(planner_meta.get("robot_radius", 0.25)),
            "robot_radius_xy": float(planner_meta.get("robot_radius_xy", -1.0)),
            "robot_height": float(planner_meta.get("robot_height", -1.0)),
            "snap_search_radius_cells": int(
                planner_meta.get("snap_search_radius_cells", 12)
            ),
            "require_ground_support": bool(
                planner_meta.get("require_ground_support", True)
            ),
            "strict_direct_ground_support": bool(
                planner_meta.get("strict_direct_ground_support", False)
            ),
            "ground_support_xy_radius_cells": int(
                planner_meta.get("ground_support_xy_radius_cells", 1)
            ),
            "ground_support_depth_cells": int(
                planner_meta.get("ground_support_depth_cells", 1)
            ),
            "lowest_traversable_only": bool(
                planner_meta.get("lowest_traversable_only", False)
            ),
            "enable_preblocked_costmap": bool(
                planner_meta.get("enable_preblocked_costmap", True)
            ),
            "preblocked_costmap_radius_cells": int(
                planner_meta.get("preblocked_costmap_radius_cells", 3)
            ),
            "preblocked_costmap_weight": float(
                planner_meta.get("preblocked_costmap_weight", 2.5)
            ),
        }
        request = SetParameters.Request()
        request.parameters = [
            self._planner_parameter(name, value) for name, value in values.items()
        ]
        result = self._wait_for_future(
            self.planner_parameter_client.call_async(request), 5.0
        )
        if result is None:
            return False, "planner parameter update timed out"
        failures = [
            item.reason or "unknown" for item in result.results if not item.successful
        ]
        if failures:
            return False, "planner parameter update failed: " + "; ".join(failures)

        # The web selector independently evaluates the same collision/support
        # rules before publishing a click. Keep it on the loaded package's
        # profile when it is present, while leaving non-web load workflows
        # independent of that optional node.
        if self.selector_parameter_client.wait_for_service(timeout_sec=0.5):
            selector_names = (
                "robot_radius",
                "robot_radius_xy",
                "robot_height",
                "snap_search_radius_cells",
                "require_ground_support",
                "strict_direct_ground_support",
                "ground_support_xy_radius_cells",
                "ground_support_depth_cells",
            )
            selector_request = SetParameters.Request()
            selector_request.parameters = [
                self._planner_parameter(name, values[name])
                for name in selector_names
            ]
            selector_result = self._wait_for_future(
                self.selector_parameter_client.call_async(selector_request), 5.0
            )
            if selector_result is None:
                return False, "web selector parameter update timed out"
            selector_failures = [
                item.reason or "unknown"
                for item in selector_result.results
                if not item.successful
            ]
            if selector_failures:
                return False, "web selector parameter update failed: " + "; ".join(
                    selector_failures
                )
            return True, "planner and web selector profiles applied"

        self.get_logger().debug(
            "web_click_selector is not running; skipped optional profile sync"
        )
        return True, "planner profile applied; web selector not running"

    def _handle_save_package(
        self,
        request: SaveNavigationMapPackage.Request,
        response: SaveNavigationMapPackage.Response,
    ) -> SaveNavigationMapPackage.Response:
        export_ok, export_msg, export_result = self._call_export_snapshot()
        if not export_ok or export_result is None:
            response.success = False
            response.message = export_msg
            return response

        meta_ok, meta_msg, meta = self._call_get_meta()
        if not meta_ok or meta is None:
            response.success = False
            response.message = meta_msg
            return response

        if self._latest_octomap is None:
            response.success = False
            response.message = "octomap message not received yet"
            return response
        if self._latest_preblocked is None:
            response.success = False
            response.message = "preblocked marker not received yet"
            return response
        if self._latest_traversable is None:
            response.success = False
            response.message = "traversable marker not received yet"
            return response
        if self._latest_risk_cost is None:
            response.success = False
            response.message = "risk cost cloud not received yet"
            return response

        package_dir = Path(request.package_path).expanduser()
        if package_dir.exists():
            if not request.overwrite:
                response.success = False
                response.message = f"package path already exists: {package_dir}"
                return response
        try:
            package_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            response.success = False
            response.message = f"failed to create package directory {package_dir}: {exc}"
            return response

        octomap_file = package_dir / "octomap_msg.npz"
        layers_file = package_dir / "layers.npz"
        meta_file = package_dir / "meta.yaml"

        try:
            np.savez_compressed(
                octomap_file,
                binary=np.array([self._latest_octomap.binary], dtype=np.bool_),
                octomap_id=np.array([self._latest_octomap.id]),
                resolution=np.array([self._latest_octomap.resolution], dtype=np.float64),
                frame_id=np.array([self._latest_octomap.header.frame_id]),
                data=np.array(self._latest_octomap.data, dtype=np.int8),
            )
        except (OSError, ValueError, RuntimeError) as exc:
            response.success = False
            response.message = f"failed to write {octomap_file}: {exc}"
            return response

        preblocked_points = self._marker_points_to_numpy(self._latest_preblocked)
        traversable_points = self._marker_points_to_numpy(self._latest_traversable)
        risk_records = list(
            point_cloud2.read_points(
                self._latest_risk_cost,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        )
        risk_points = np.array(
            [[row[0], row[1], row[2], row[3]] for row in risk_records],
            dtype=np.float32,
        )
        try:
            np.savez_compressed(
                layers_file,
                preblocked_points=preblocked_points,
                preblocked_scale=np.array(
                    [
                        self._latest_preblocked.scale.x,
                        self._latest_preblocked.scale.y,
                        self._latest_preblocked.scale.z,
                    ],
                    dtype=np.float64,
                ),
                preblocked_frame_id=np.array([self._latest_preblocked.header.frame_id]),
                traversable_points=traversable_points,
                traversable_scale=np.array(
                    [
                        self._latest_traversable.scale.x,
                        self._latest_traversable.scale.y,
                        self._latest_traversable.scale.z,
                    ],
                    dtype=np.float64,
                ),
                traversable_frame_id=np.array([self._latest_traversable.header.frame_id]),
                risk_points=risk_points[:, :3]
                if risk_points.size
                else np.empty((0, 3), dtype=np.float32),
                risk_intensity=risk_points[:, 3]
                if risk_points.size
                else np.empty((0,), dtype=np.float32),
                risk_frame_id=np.array([self._latest_risk_cost.header.frame_id]),
            )
        except (OSError, ValueError, RuntimeError) as exc:
            response.success = False
            response.message = f"failed to write {layers_file}: {exc}"
            return response

        meta_yaml = {
            "map_id": meta.map_id,
            "frame_id": meta.frame_id,
            "resolution": meta.resolution,
            "octomap_file": octomap_file.name,
            "layers_file": layers_file.name,
            "source_world_file": meta.source_world_file,
            "snapshot_stamp": {
                "sec": int(export_result.snapshot_stamp.sec),
                "nanosec": int(export_result.snapshot_stamp.nanosec),
            },
            "bounds": {
                "min": [meta.min_bound.x, meta.min_bound.y, meta.min_bound.z],
                "max": [meta.max_bound.x, meta.max_bound.y, meta.max_bound.z],
            },
            "planner": {
                "robot_radius": meta.robot_radius,
                "robot_radius_xy": meta.robot_radius_xy,
                "robot_height": meta.robot_height,
                "snap_search_radius_cells": meta.snap_search_radius_cells,
                "require_ground_support": meta.require_ground_support,
                "strict_direct_ground_support": meta.strict_direct_ground_support,
                "ground_support_xy_radius_cells": meta.ground_support_xy_radius_cells,
                "ground_support_depth_cells": meta.ground_support_depth_cells,
                "enable_preblocked_costmap": meta.enable_preblocked_costmap,
                "preblocked_costmap_radius_cells": meta.preblocked_costmap_radius_cells,
                "preblocked_costmap_weight": meta.preblocked_costmap_weight,
            },
            "layers": {
                "preblocked_count": int(preblocked_points.shape[0]),
                "traversable_count": int(traversable_points.shape[0]),
                "risk_cost_count": int(risk_points.shape[0]),
            },
        }

        try:
            with meta_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(meta_yaml, f, sort_keys=False, allow_unicode=True)
        except (OSError, yaml.YAMLError) as exc:
            response.success = False
            response.message = f"failed to write {meta_file}: {exc}"
            return response

        response.success = True
        response.message = "map package saved"
        response.manifest_path = str(meta_file)
        return response

    def _handle_load_package(
        self,
        request: LoadNavigationMapPackage.Request,
        response: LoadNavigationMapPackage.Response,
    ) -> LoadNavigationMapPackage.Response:
        success, message, map_id = self._load_package(request.package_path)
        response.success = success
        response.message = message
        response.map_id = map_id
        return response

    def _autoload_package_once(self) -> None:
        self._autoload_timer.cancel()
        package_path = str(self.get_parameter("autoload_package_path").value).strip()
        if not package_path:
            return

        success, message, map_id = self._load_package(package_path)
        if success:
            self.get_logger().info(
                f"autoloaded map package: {package_path} map_id={map_id}"
            )
        else:
            self.get_logger().error(
                f"failed to autoload map package {package_path}: {message}"
            )

    def _load_package(self, package_path: str) -> tuple[bool, str, str]:
        package_dir = Path(package_path).expanduser()
        meta_file = package_dir / "meta.yaml"
        if not meta_file.exists():
            return False, f"meta file not found: {meta_file}", ""

        with meta_file.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        profile_ok, profile_message = self._apply_planner_profile(
            meta.get("planner", {})
        )
        if not profile_ok:
            return False, profile_message, str(meta.get("map_id", ""))

        octomap_npz = np.load(package_dir / meta["octomap_file"], allow_pickle=False)
        layers_npz = np.load(package_dir / meta["layers_file"], allow_pickle=False)

        octomap_msg = Octomap()
        octomap_msg.header.frame_id = str(octomap_npz["frame_id"][0])
        octomap_msg.header.stamp = self.get_clock().now().to_msg()
        octomap_msg.binary = bool(octomap_npz["binary"][0])
        octomap_msg.id = str(octomap_npz["octomap_id"][0])
        octomap_msg.resolution = float(octomap_npz["resolution"][0])
        octomap_msg.data = octomap_npz["data"].astype(np.int8).tolist()

        occupied_msg = None
        if "occupied_points" in layers_npz:
            occupied_msg = self._make_marker_from_points(
                str(layers_npz["occupied_frame_id"][0]),
                "occupied_voxels",
                layers_npz["occupied_scale"],
                layers_npz["occupied_points"],
                (0.95, 0.45, 0.15, 0.95),
            )
        preblocked_msg = self._make_marker_from_points(
            str(layers_npz["preblocked_frame_id"][0]),
            "preblocked_cells",
            layers_npz["preblocked_scale"],
            layers_npz["preblocked_points"],
            (0.15, 0.35, 1.0, 0.95),
        )
        traversable_msg = self._make_marker_from_points(
            str(layers_npz["traversable_frame_id"][0]),
            "traversable_cells",
            layers_npz["traversable_scale"],
            layers_npz["traversable_points"],
            (0.20, 0.95, 0.55, 0.55),
        )
        risk_msg = PointCloud2()
        risk_msg.header.frame_id = str(layers_npz["risk_frame_id"][0])
        risk_msg.header.stamp = self.get_clock().now().to_msg()
        risk_points = layers_npz["risk_points"]
        risk_intensity = layers_npz["risk_intensity"]
        risk_msg = point_cloud2.create_cloud(
            risk_msg.header,
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            [
                (float(p[0]), float(p[1]), float(p[2]), float(i))
                for p, i in zip(risk_points, risk_intensity)
            ],
        )

        self._latest_octomap = copy.deepcopy(octomap_msg)
        self._latest_occupied = copy.deepcopy(occupied_msg) if occupied_msg is not None else None
        self._latest_preblocked = copy.deepcopy(preblocked_msg)
        self._latest_traversable = copy.deepcopy(traversable_msg)
        self._latest_risk_cost = copy.deepcopy(risk_msg)
        self.octomap_pub.publish(octomap_msg)
        if occupied_msg is not None:
            self.occupied_pub.publish(occupied_msg)
        if bool(self.get_parameter("publish_cached_derived_layers_on_load").value):
            # This compatibility mode is useful only without a live planner.
            # With jie_path_node running, the loaded OctoMap is authoritative
            # and all profile-dependent layers must be rebuilt by that node.
            self.preblocked_pub.publish(preblocked_msg)
            self.traversable_pub.publish(traversable_msg)
            self.risk_cost_pub.publish(risk_msg)

        return True, "map package loaded", str(meta.get("map_id", ""))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPackageManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
