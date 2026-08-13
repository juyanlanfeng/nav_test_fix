#!/usr/bin/env python3

from __future__ import annotations

import copy
import shutil
import tempfile
import threading
import time
import uuid
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from jie_octomap.map_package_schema import (
    PLANNER_EXPORT_TIMEOUT_SEC,
    external_preblocked_layer_from_archive,
    planner_metadata_from_response,
    planner_parameters_from_metadata,
)
from jie_map_msgs.srv import (
    ApplyNavigationMapSnapshot,
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
from sensor_msgs.msg import PointCloud2
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
        self.declare_parameter(
            "external_preblocked_command_topic",
            "/edited_preblocked_cells_commands",
        )
        self.declare_parameter("traversable_topic", "/traversable_cells_markers")
        self.declare_parameter("risk_cost_topic", "/risk_cost_cells")
        self.declare_parameter("planner_meta_service", "/jie_path_node/get_meta")
        self.declare_parameter("planner_export_service", "/jie_path_node/export_snapshot")
        self.declare_parameter("planner_apply_service", "/jie_path_node/apply_snapshot")
        self.declare_parameter(
            "selector_parameter_service", "/web_click_selector/set_parameters"
        )
        self.declare_parameter("publish_cached_derived_layers_on_load", False)
        self.declare_parameter(
            "planner_export_timeout_sec", PLANNER_EXPORT_TIMEOUT_SEC
        )
        self.declare_parameter("snapshot_delivery_timeout_sec", 30.0)
        self.declare_parameter("authority_watchdog_period_sec", 2.0)
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
        self._pending_external_command: Optional[Marker] = None
        self._uncertain_apply = None
        self._authority_octomap: Optional[Octomap] = None
        self._authority_external_preblocked: Optional[Marker] = None
        self._authority_planner_values: Optional[dict] = None
        self._suppressed_authority_mismatch = None
        self._cache_lock = threading.Lock()
        self._cache_changed = threading.Condition(self._cache_lock)
        self._package_operation_lock = threading.Lock()
        self._snapshot_revisions = {
            "octomap": 0,
            "preblocked": 0,
            "external": 0,
            "traversable": 0,
            "risk": 0,
        }

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
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Marker,
            self.get_parameter("external_preblocked_command_topic").value,
            self._on_external_preblocked_command,
            volatile_qos,
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

        self.octomap_pub = self.create_publisher(
            Octomap, self.get_parameter("octomap_topic").value, qos
        )
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
        self.apply_client = self.create_client(
            ApplyNavigationMapSnapshot,
            self.get_parameter("planner_apply_service").value,
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
        self._external_command_timer = self.create_timer(
            0.25, self._try_apply_pending_external_command
        )
        watchdog_period = max(
            0.5,
            float(self.get_parameter("authority_watchdog_period_sec").value),
        )
        self._authority_watchdog_timer = self.create_timer(
            watchdog_period, self._authority_watchdog_once
        )

        self.get_logger().info(
            "map_package_manager started. save_service=~/save_package load_service=~/load_package"
        )

    def _on_octomap(self, msg: Octomap) -> None:
        with self._cache_changed:
            self._latest_octomap = copy.deepcopy(msg)
            if (
                self._authority_octomap is not None
                and msg.header.frame_id
                == self._authority_octomap.header.frame_id
                and np.isclose(
                    msg.resolution, self._authority_octomap.resolution
                )
            ):
                self._authority_octomap = copy.deepcopy(msg)
            self._snapshot_revisions["octomap"] += 1
            self._cache_changed.notify_all()

    def _on_occupied(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._cache_lock:
                self._latest_occupied = copy.deepcopy(msg)

    def _on_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._cache_changed:
                self._latest_preblocked = copy.deepcopy(msg)
                self._snapshot_revisions["preblocked"] += 1
                self._cache_changed.notify_all()

    def _on_external_preblocked(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._cache_changed:
                self._latest_external_preblocked = copy.deepcopy(msg)
                if self._authority_planner_values is not None:
                    expected_frame = str(
                        self._authority_planner_values.get("frame_id", "")
                    )
                    if not msg.header.frame_id or (
                        msg.header.frame_id == expected_frame
                    ):
                        self._authority_external_preblocked = copy.deepcopy(msg)
                self._snapshot_revisions["external"] += 1
                self._cache_changed.notify_all()

    def _on_external_preblocked_command(self, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            self.get_logger().warning(
                "ignored external-preblocked command because it is not CUBE_LIST"
            )
            return
        with self._cache_lock:
            self._pending_external_command = copy.deepcopy(msg)
        self._try_apply_pending_external_command()

    def _try_apply_pending_external_command(self) -> None:
        if not self._package_operation_lock.acquire(blocking=False):
            return
        msg: Optional[Marker] = None
        command_committed = False
        try:
            apply_ready, _ = self._resolve_uncertain_apply()
            if not apply_ready:
                return
            with self._cache_lock:
                msg = self._pending_external_command
                self._pending_external_command = None
            if msg is None:
                return
            with self._cache_changed:
                if (
                    self._latest_external_preblocked is not None
                    and self._marker_content_key(msg)
                    == self._marker_content_key(self._latest_external_preblocked)
                ):
                    command_committed = True
                    return
                authoritative = copy.deepcopy(msg)
                octomap_msg = copy.deepcopy(self._latest_octomap)
            if octomap_msg is None:
                self._requeue_external_command(authoritative)
                return
            apply_ok, apply_message, apply_result, snapshot = (
                self._call_apply_snapshot(
                    octomap_msg,
                    authoritative,
                    clear_navigation=False,
                    planner_parameters=[],
                )
            )
            if apply_result is None or not apply_result.success:
                self._requeue_external_command(authoritative)
                self.get_logger().error(
                    "failed to apply authored preblocked update: " + apply_message
                )
                return

            applied_external = copy.deepcopy(authoritative)
            applied_external.header.stamp = apply_result.snapshot_stamp
            applied_octomap = copy.deepcopy(octomap_msg)
            applied_octomap.header.stamp = apply_result.snapshot_stamp
            delivered_octomap = self._copy_octomap_at_stamp(
                apply_result.snapshot_stamp
            )
            if delivered_octomap is not None:
                applied_octomap = delivered_octomap
            if snapshot is not None:
                snapshot_external = snapshot.get("external")
                if snapshot_external is not None:
                    applied_external = copy.deepcopy(snapshot_external)
                snapshot_octomap = snapshot.get("octomap")
                if snapshot_octomap is not None:
                    applied_octomap = copy.deepcopy(snapshot_octomap)
            self._publish_applied_authority(applied_external, applied_octomap)
            command_committed = True
            if not apply_ok:
                self.get_logger().warning(
                    "authored preblocked update was applied, but snapshot "
                    "delivery confirmation was incomplete: " + apply_message
                )
        finally:
            if msg is not None and not command_committed:
                self._requeue_external_command(msg)
            self._package_operation_lock.release()

    def _requeue_external_command(self, msg: Marker) -> None:
        """Retry an uncommitted command without overwriting a newer edit."""
        with self._cache_lock:
            if self._pending_external_command is None:
                self._pending_external_command = copy.deepcopy(msg)

    def _clear_pending_external_command(self) -> None:
        with self._cache_lock:
            self._pending_external_command = None

    def _publish_applied_authority(
        self,
        external_preblocked_msg: Marker,
        octomap_msg: Optional[Octomap],
    ) -> None:
        with self._cache_lock:
            self._latest_external_preblocked = copy.deepcopy(
                external_preblocked_msg
            )
            if self._authority_planner_values is not None:
                self._authority_external_preblocked = copy.deepcopy(
                    external_preblocked_msg
                )
                if octomap_msg is not None:
                    self._authority_octomap = copy.deepcopy(octomap_msg)
        self.external_preblocked_pub.publish(external_preblocked_msg)
        if octomap_msg is not None:
            self.octomap_pub.publish(octomap_msg)

    def _cache_navigation_authority(
        self,
        octomap_msg: Octomap,
        external_preblocked_msg: Marker,
        planner_values: dict,
    ) -> None:
        with self._cache_lock:
            self._authority_octomap = copy.deepcopy(octomap_msg)
            self._authority_external_preblocked = copy.deepcopy(
                external_preblocked_msg
            )
            self._authority_planner_values = copy.deepcopy(planner_values)
            self._suppressed_authority_mismatch = None

    def _navigation_authority_snapshot(
        self,
    ) -> tuple[Optional[Octomap], Optional[Marker], Optional[dict]]:
        with self._cache_lock:
            return (
                copy.deepcopy(self._authority_octomap),
                copy.deepcopy(self._authority_external_preblocked),
                copy.deepcopy(self._authority_planner_values),
            )

    def _navigation_authority_descriptor(
        self,
    ) -> Optional[tuple[str, float, tuple, dict]]:
        with self._cache_lock:
            if (
                self._authority_octomap is None
                or self._authority_external_preblocked is None
                or self._authority_planner_values is None
            ):
                return None
            return (
                self._authority_octomap.header.frame_id,
                float(self._authority_octomap.resolution),
                self._marker_content_key(
                    self._authority_external_preblocked
                ),
                copy.deepcopy(self._authority_planner_values),
            )

    @classmethod
    def _planner_values_from_meta(
        cls, meta: GetNavigationMapMeta.Response
    ) -> dict:
        metadata = {
            "frame_id": meta.frame_id,
            "map_id": meta.map_id,
            "source_world_file": meta.source_world_file,
            "planner": planner_metadata_from_response(meta),
        }
        return cls._validated_planner_values(metadata)

    @staticmethod
    def _planner_values_match(expected: dict, actual: dict) -> bool:
        if set(expected) != set(actual):
            return False
        for name, expected_value in expected.items():
            actual_value = actual[name]
            if isinstance(expected_value, float):
                if not np.isclose(
                    expected_value,
                    float(actual_value),
                    rtol=0.0,
                    atol=1.0e-12,
                ):
                    return False
            elif expected_value != actual_value:
                return False
        return True

    def _authority_matches_meta(
        self,
        authority_frame_id: str,
        authority_resolution: float,
        authority_external_key: tuple,
        planner_values: dict,
        meta: GetNavigationMapMeta.Response,
        current_values: dict,
    ) -> bool:
        if authority_frame_id != meta.frame_id:
            return False
        if not np.isclose(
            authority_resolution,
            meta.resolution,
            rtol=0.0,
            atol=1.0e-12,
        ):
            return False
        if not self._planner_values_match(planner_values, current_values):
            return False
        current_external = getattr(meta, "external_preblocked", None)
        if current_external is None or current_external.type != Marker.CUBE_LIST:
            return False
        return authority_external_key == self._marker_content_key(
            current_external
        )

    def _authority_watchdog_once(self) -> None:
        if self._navigation_authority_descriptor() is None:
            return
        if not self._package_operation_lock.acquire(blocking=False):
            return
        try:
            apply_ready, _ = self._resolve_uncertain_apply()
            if not apply_ready:
                return
            descriptor = self._navigation_authority_descriptor()
            if descriptor is None:
                return
            (
                authority_frame_id,
                authority_resolution,
                authority_external_key,
                planner_values,
            ) = descriptor
            meta_ok, _, meta = self._call_get_meta()
            if not meta_ok or meta is None:
                return
            try:
                current_values = self._planner_values_from_meta(meta)
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f"authority watchdog ignored invalid planner metadata: {exc}"
                )
                return
            if self._authority_matches_meta(
                authority_frame_id,
                authority_resolution,
                authority_external_key,
                planner_values,
                meta,
                current_values,
            ):
                with self._cache_lock:
                    self._suppressed_authority_mismatch = None
                return

            mismatch_signature = self._meta_signature(meta)
            with self._cache_lock:
                if mismatch_signature == self._suppressed_authority_mismatch:
                    return
            octomap_msg, external_preblocked_msg, planner_values = (
                self._navigation_authority_snapshot()
            )
            if (
                octomap_msg is None
                or external_preblocked_msg is None
                or planner_values is None
            ):
                return
            parameters = [
                self._planner_parameter(name, value)
                for name, value in planner_values.items()
            ]
            apply_ok, apply_message, apply_result, snapshot = (
                self._call_apply_snapshot(
                    octomap_msg,
                    external_preblocked_msg,
                    clear_navigation=False,
                    planner_parameters=parameters,
                    authority_values=planner_values,
                )
            )
            if apply_result is None or not apply_result.success:
                self.get_logger().error(
                    "authority watchdog could not restore planner state: "
                    + apply_message
                )
                return

            restored_external = copy.deepcopy(external_preblocked_msg)
            restored_external.header.stamp = apply_result.snapshot_stamp
            restored_octomap = copy.deepcopy(octomap_msg)
            restored_octomap.header.stamp = apply_result.snapshot_stamp
            if snapshot is not None:
                snapshot_external = snapshot.get("external")
                if isinstance(snapshot_external, Marker):
                    restored_external = copy.deepcopy(snapshot_external)
                snapshot_octomap = snapshot.get("octomap")
                if isinstance(snapshot_octomap, Octomap):
                    restored_octomap = copy.deepcopy(snapshot_octomap)
            self._publish_applied_authority(
                restored_external, restored_octomap
            )
            self._cache_navigation_authority(
                restored_octomap, restored_external, planner_values
            )
            with self._cache_lock:
                self._suppressed_authority_mismatch = mismatch_signature
            selector_ok, selector_message = self._sync_selector_profile(
                planner_values
            )
            if not selector_ok:
                self.get_logger().warning(selector_message)
            if not apply_ok:
                self.get_logger().warning(
                    "authority restored, but snapshot delivery was incomplete: "
                    + apply_message
                )
            else:
                self.get_logger().warning(
                    "planner profile mismatch detected; navigation authority "
                    "was restored atomically"
                )
        finally:
            self._package_operation_lock.release()

    def _resolve_uncertain_apply(self) -> tuple[bool, str]:
        """Resolve a timed-out apply before permitting another map operation."""
        if self._uncertain_apply is None:
            return True, ""
        (
            future,
            octomap_msg,
            external_preblocked_msg,
            authority_values,
        ) = self._uncertain_apply
        if not future.done():
            return (
                False,
                "a previous planner apply is still pending with unknown outcome; "
                "wait before issuing another map operation",
            )
        self._uncertain_apply = None
        try:
            result = future.result()
        except Exception as exc:  # rclpy futures surface transport exceptions here.
            message = f"previous planner apply finished with an exception: {exc}"
            self.get_logger().error(message)
            return True, message
        if result is None:
            message = "previous planner apply completed without a response"
            self.get_logger().error(message)
            return True, message
        if not result.success:
            message = "previous planner apply failed: " + result.message
            self.get_logger().error(message)
            return True, message

        authoritative_external = copy.deepcopy(external_preblocked_msg)
        authoritative_external.header.stamp = result.snapshot_stamp
        durable_octomap = copy.deepcopy(octomap_msg)
        durable_octomap.header.stamp = result.snapshot_stamp
        self._publish_applied_authority(
            authoritative_external, durable_octomap
        )
        if authority_values is not None:
            self._cache_navigation_authority(
                durable_octomap,
                authoritative_external,
                authority_values,
            )
        message = (
            "previous planner apply completed successfully after its client "
            "timeout; durable authority was restored"
        )
        self.get_logger().warning(message)
        return True, message

    def _on_traversable(self, msg: Marker) -> None:
        if msg.type == Marker.CUBE_LIST:
            with self._cache_changed:
                self._latest_traversable = copy.deepcopy(msg)
                self._snapshot_revisions["traversable"] += 1
                self._cache_changed.notify_all()

    def _on_risk_cost(self, msg: PointCloud2) -> None:
        with self._cache_changed:
            self._latest_risk_cost = copy.deepcopy(msg)
            self._snapshot_revisions["risk"] += 1
            self._cache_changed.notify_all()

    def _marker_points_to_numpy(self, marker: Marker) -> np.ndarray:
        return np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)

    @staticmethod
    def _marker_content_key(marker: Marker) -> tuple:
        return (
            marker.header.frame_id,
            marker.type,
            marker.action,
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
            marker.scale.x,
            marker.scale.y,
            marker.scale.z,
            tuple((point.x, point.y, point.z) for point in marker.points),
        )

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

    @staticmethod
    def _stamp_key(stamp) -> tuple[int, int]:
        return int(stamp.sec), int(stamp.nanosec)

    def _wait_for_snapshot_delivery(
        self,
        before: dict[str, int],
        snapshot_stamp,
        timeout_sec: float,
        external_override: Optional[Marker] = None,
    ) -> Optional[dict[str, object]]:
        """Wait for, then freeze, one complete stamped planner generation.

        The cache condition stays locked while all five messages are copied.
        A later topic callback therefore cannot mix the next generation into
        the bundle after the delivery predicate has been satisfied.
        """
        expected_stamp = self._stamp_key(snapshot_stamp)
        names_and_messages = (
            ("octomap", "_latest_octomap"),
            ("preblocked", "_latest_preblocked"),
            ("traversable", "_latest_traversable"),
            ("risk", "_latest_risk_cost"),
        )
        deadline = time.monotonic() + timeout_sec
        with self._cache_changed:
            while rclpy.ok():
                complete = True
                for revision_name, attribute_name in names_and_messages:
                    message = getattr(self, attribute_name)
                    if (
                        self._snapshot_revisions[revision_name]
                        <= before[revision_name]
                        or message is None
                        or self._stamp_key(message.header.stamp) != expected_stamp
                    ):
                        complete = False
                        break
                if complete:
                    external = (
                        external_override
                        if external_override is not None
                        else self._latest_external_preblocked
                    )
                    return {
                        "octomap": copy.deepcopy(self._latest_octomap),
                        "preblocked": copy.deepcopy(self._latest_preblocked),
                        "external": copy.deepcopy(external),
                        "traversable": copy.deepcopy(self._latest_traversable),
                        "risk": copy.deepcopy(self._latest_risk_cost),
                        "revisions": dict(self._snapshot_revisions),
                    }
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._cache_changed.wait(timeout=min(remaining, 0.25))
        return None

    def _snapshot_revisions_match(
        self, snapshot: dict[str, object]
    ) -> bool:
        frozen = snapshot.get("revisions")
        if not isinstance(frozen, dict):
            return False
        names = ("octomap", "preblocked", "traversable", "risk")
        with self._cache_lock:
            return all(
                frozen.get(name) == self._snapshot_revisions[name]
                for name in names
            )

    def _copy_octomap_at_stamp(self, snapshot_stamp) -> Optional[Octomap]:
        expected_stamp = self._stamp_key(snapshot_stamp)
        with self._cache_lock:
            if (
                self._latest_octomap is not None
                and self._stamp_key(self._latest_octomap.header.stamp)
                == expected_stamp
            ):
                return copy.deepcopy(self._latest_octomap)
        return None

    def _call_export_snapshot(
        self,
        external_override: Optional[Marker] = None,
    ) -> tuple[
        bool,
        str,
        Optional[ExportNavigationSnapshot.Response],
        Optional[dict[str, object]],
    ]:
        if not self.export_client.wait_for_service(timeout_sec=1.0):
            return False, "planner export service unavailable", None, None
        request = ExportNavigationSnapshot.Request()
        request.recompute_layers = True
        with self._cache_lock:
            before = dict(self._snapshot_revisions)
        future = self.export_client.call_async(request)
        timeout_sec = float(self.get_parameter("planner_export_timeout_sec").value)
        result = self._wait_for_future(future, timeout_sec)
        if result is None:
            return (
                False,
                f"planner export service timed out after {timeout_sec:.1f} s",
                None,
                None,
            )
        if not result.success:
            return False, result.message, result, None
        delivery_timeout = float(
            self.get_parameter("snapshot_delivery_timeout_sec").value
        )
        snapshot = self._wait_for_snapshot_delivery(
            before,
            result.snapshot_stamp,
            delivery_timeout,
            external_override=external_override,
        )
        if snapshot is None:
            return (
                False,
                "planner snapshot topics did not arrive as one stamped generation "
                f"within {delivery_timeout:.1f} s",
                result,
                None,
            )
        return result.success, result.message, result, snapshot

    def _call_apply_snapshot(
        self,
        octomap_msg: Octomap,
        external_preblocked_msg: Marker,
        clear_navigation: bool,
        planner_parameters: list[Parameter],
        authority_values: Optional[dict] = None,
    ) -> tuple[
        bool,
        str,
        Optional[ApplyNavigationMapSnapshot.Response],
        Optional[dict[str, object]],
    ]:
        if not self.apply_client.wait_for_service(timeout_sec=2.0):
            return (
                False,
                "planner apply-snapshot service unavailable",
                None,
                None,
            )
        request = ApplyNavigationMapSnapshot.Request()
        request.octomap = octomap_msg
        request.external_preblocked = external_preblocked_msg
        request.planner_parameters = planner_parameters
        request.clear_navigation = clear_navigation
        with self._cache_lock:
            before = dict(self._snapshot_revisions)
        future = self.apply_client.call_async(request)
        timeout_sec = float(self.get_parameter("planner_export_timeout_sec").value)
        result = self._wait_for_future(future, timeout_sec)
        if result is None:
            # Recheck briefly at the timeout edge. The service may already have
            # installed map/parameters and merely still be rebuilding layers.
            result = self._wait_for_future(future, 5.0)
        if result is None:
            self._uncertain_apply = (
                future,
                copy.deepcopy(octomap_msg),
                copy.deepcopy(external_preblocked_msg),
                copy.deepcopy(authority_values),
            )
            return (
                False,
                "planner apply-snapshot timed out after "
                f"{timeout_sec + 5.0:.1f} s; outcome is unknown and further "
                "map operations are paused until the response completes",
                None,
                None,
            )
        if not result.success:
            return False, result.message, result, None
        delivery_timeout = float(
            self.get_parameter("snapshot_delivery_timeout_sec").value
        )
        authoritative_external = copy.deepcopy(external_preblocked_msg)
        authoritative_external.header.stamp = result.snapshot_stamp
        snapshot = self._wait_for_snapshot_delivery(
            before,
            result.snapshot_stamp,
            delivery_timeout,
            external_override=authoritative_external,
        )
        if snapshot is None:
            return (
                False,
                "snapshot was applied, but its topics did not arrive as one "
                "stamped generation "
                f"within {delivery_timeout:.1f} s",
                result,
                None,
            )
        return True, result.message, result, snapshot

    def _call_get_meta(self) -> tuple[bool, str, Optional[GetNavigationMapMeta.Response]]:
        if not self.meta_client.wait_for_service(timeout_sec=1.0):
            return False, "planner meta service unavailable", None
        future = self.meta_client.call_async(GetNavigationMapMeta.Request())
        result = self._wait_for_future(future, 5.0)
        if result is None:
            return False, "planner meta service timed out", None
        return result.success, result.message, result

    @classmethod
    def _meta_signature(cls, meta: GetNavigationMapMeta.Response) -> tuple:
        external = getattr(meta, "external_preblocked", None)
        external_key = None
        if external is not None and external.type == Marker.CUBE_LIST:
            external_key = cls._marker_content_key(external)
        return (
            meta.map_id,
            meta.frame_id,
            float(meta.resolution),
            meta.source_world_file,
            (meta.min_bound.x, meta.min_bound.y, meta.min_bound.z),
            (meta.max_bound.x, meta.max_bound.y, meta.max_bound.z),
            tuple(sorted(planner_metadata_from_response(meta).items())),
            external_key,
        )

    @staticmethod
    def _planner_parameter(name: str, value) -> Parameter:
        parameter = Parameter()
        parameter.name = name
        if isinstance(value, str):
            parameter.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=value
            )
        elif isinstance(value, bool):
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

    @staticmethod
    def _validated_planner_values(metadata: dict) -> dict:
        planner = metadata.get("planner", {}) or {}
        if not isinstance(planner, dict):
            raise TypeError("metadata planner entry must be a mapping")
        boolean_names = (
            "require_ground_support",
            "strict_direct_ground_support",
            "lowest_traversable_only",
            "enable_preblocked_costmap",
        )
        for name in boolean_names:
            if name in planner and type(planner[name]) is not bool:
                raise ValueError(
                    f"planner parameter {name} must be a YAML boolean"
                )
        values = planner_parameters_from_metadata(metadata)

        def finite_float(name: str) -> float:
            value = float(values[name])
            if not np.isfinite(value):
                raise ValueError(f"planner parameter {name} must be finite")
            return value

        robot_radius = finite_float("robot_radius")
        if robot_radius <= 0.0:
            raise ValueError("planner parameter robot_radius must be positive")
        for name in ("robot_radius_xy", "robot_height"):
            value = finite_float(name)
            if value != -1.0 and value <= 0.0:
                raise ValueError(
                    f"planner parameter {name} must be -1 or positive"
                )
        cost_weight = finite_float("preblocked_costmap_weight")
        if cost_weight < 0.0:
            raise ValueError(
                "planner parameter preblocked_costmap_weight cannot be negative"
            )
        nonnegative_integer_names = (
            "snap_search_radius_cells",
            "ground_support_xy_radius_cells",
            "preblocked_costmap_radius_cells",
        )
        for name in nonnegative_integer_names:
            if int(values[name]) < 0:
                raise ValueError(f"planner parameter {name} cannot be negative")
        if int(values["ground_support_depth_cells"]) < 1:
            raise ValueError(
                "planner parameter ground_support_depth_cells must be at least 1"
            )
        if not str(values["frame_id"]):
            raise ValueError("planner parameter frame_id must be non-empty")
        if not str(values["map_id"]):
            raise ValueError("planner parameter map_id must be non-empty")
        return values

    def _sync_selector_profile(self, values: dict) -> tuple[bool, str]:
        """Best-effort sync for the optional web click selector."""
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
            return True, "web selector profile synchronized"

        self.get_logger().debug(
            "web_click_selector is not running; skipped optional profile sync"
        )
        return True, "web selector not running"

    def _handle_save_package(
        self,
        request: SaveNavigationMapPackage.Request,
        response: SaveNavigationMapPackage.Response,
    ) -> SaveNavigationMapPackage.Response:
        if not self._package_operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another map-package save/load operation is active"
            return response
        try:
            apply_ready, apply_message = self._resolve_uncertain_apply()
            if not apply_ready:
                response.success = False
                response.message = apply_message
                return response
            return self._save_package_locked(request, response)
        finally:
            self._package_operation_lock.release()

    def _save_package_locked(
        self,
        request: SaveNavigationMapPackage.Request,
        response: SaveNavigationMapPackage.Response,
    ) -> SaveNavigationMapPackage.Response:
        meta_ok, meta_msg, meta_before = self._call_get_meta()
        if not meta_ok or meta_before is None:
            response.success = False
            response.message = meta_msg
            return response

        meta_external = copy.deepcopy(
            getattr(meta_before, "external_preblocked", None)
        )
        if meta_external is not None and meta_external.type != Marker.CUBE_LIST:
            meta_external = None
        export_ok, export_msg, export_result, snapshot = (
            self._call_export_snapshot(external_override=meta_external)
        )
        if not export_ok or export_result is None or snapshot is None:
            response.success = False
            response.message = export_msg
            return response

        meta_ok, meta_msg, meta = self._call_get_meta()
        if not meta_ok or meta is None:
            response.success = False
            response.message = meta_msg
            return response
        if self._meta_signature(meta_before) != self._meta_signature(meta):
            response.success = False
            response.message = (
                "planner map metadata changed during snapshot export; retry save"
            )
            return response
        if not self._snapshot_revisions_match(snapshot):
            response.success = False
            response.message = (
                "planner snapshot topics changed after export; retry save"
            )
            return response

        octomap_msg = snapshot.get("octomap")
        preblocked_msg = snapshot.get("preblocked")
        external_preblocked_msg = snapshot.get("external")
        traversable_msg = snapshot.get("traversable")
        risk_cost_msg = snapshot.get("risk")

        if not isinstance(octomap_msg, Octomap):
            response.success = False
            response.message = "octomap message not received yet"
            return response
        if not isinstance(preblocked_msg, Marker):
            response.success = False
            response.message = "preblocked marker not received yet"
            return response
        if not isinstance(traversable_msg, Marker):
            response.success = False
            response.message = "traversable marker not received yet"
            return response
        if not isinstance(risk_cost_msg, PointCloud2):
            response.success = False
            response.message = "risk cost cloud not received yet"
            return response

        requested_path = str(request.package_path).strip()
        if not requested_path:
            response.success = False
            response.message = "package path is empty"
            return response
        package_dir = Path(requested_path).expanduser().absolute()
        if package_dir == package_dir.parent:
            response.success = False
            response.message = "filesystem root cannot be used as a map package"
            return response
        target_present = package_dir.exists() or package_dir.is_symlink()
        if target_present and package_dir.resolve() in {
            Path.cwd().resolve(),
            Path.home().resolve(),
        }:
            response.success = False
            response.message = (
                f"refusing to replace broad directory: {package_dir}"
            )
            return response
        if target_present:
            if not request.overwrite:
                response.success = False
                response.message = f"package path already exists: {package_dir}"
                return response
            if package_dir.is_symlink():
                response.success = False
                response.message = (
                    f"refusing to replace symbolic-link path: {package_dir}"
                )
                return response
            if not package_dir.is_dir():
                response.success = False
                response.message = (
                    f"refusing to replace non-directory path: {package_dir}"
                )
                return response
            existing_meta_file = package_dir / "meta.yaml"
            if not existing_meta_file.is_file():
                response.success = False
                response.message = (
                    "refusing to replace a directory that is not a "
                    f"map package: {package_dir}"
                )
                return response
            try:
                with existing_meta_file.open("r", encoding="utf-8") as f:
                    existing_meta = yaml.safe_load(f)
                if not isinstance(existing_meta, dict):
                    raise TypeError("meta.yaml is not a mapping")
                self._package_member_path(
                    package_dir,
                    existing_meta["octomap_file"],
                    "existing octomap_file",
                )
                self._package_member_path(
                    package_dir,
                    existing_meta["layers_file"],
                    "existing layers_file",
                )
            except (
                OSError,
                KeyError,
                TypeError,
                ValueError,
                yaml.YAMLError,
            ) as exc:
                response.success = False
                response.message = (
                    "refusing to replace an invalid existing map package: "
                    f"{exc}"
                )
                return response
        try:
            package_dir.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            response.success = False
            response.message = (
                f"failed to create package parent {package_dir.parent}: {exc}"
            )
            return response

        preblocked_points = self._marker_points_to_numpy(preblocked_msg)
        if not isinstance(external_preblocked_msg, Marker):
            external_preblocked_points = np.empty((0, 3), dtype=np.float32)
            external_preblocked_scale = np.array(
                [
                    preblocked_msg.scale.x,
                    preblocked_msg.scale.y,
                    preblocked_msg.scale.z,
                ],
                dtype=np.float64,
            )
            external_preblocked_frame_id = meta.frame_id
        else:
            external_preblocked_points = self._marker_points_to_numpy(
                external_preblocked_msg
            )
            external_preblocked_scale = np.array(
                [
                    external_preblocked_msg.scale.x,
                    external_preblocked_msg.scale.y,
                    external_preblocked_msg.scale.z,
                ],
                dtype=np.float64,
            )
            external_preblocked_frame_id = (
                external_preblocked_msg.header.frame_id or meta.frame_id
            )
        traversable_points = self._marker_points_to_numpy(traversable_msg)
        risk_records = list(
            point_cloud2.read_points(
                risk_cost_msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        )
        risk_points = np.array(
            [[row[0], row[1], row[2], row[3]] for row in risk_records],
            dtype=np.float32,
        ).reshape((-1, 4))

        meta_yaml = {
            "map_id": meta.map_id,
            "frame_id": meta.frame_id,
            "resolution": meta.resolution,
            "octomap_file": "octomap_msg.npz",
            "layers_file": "layers.npz",
            "source_world_file": meta.source_world_file,
            "snapshot_stamp": {
                "sec": int(export_result.snapshot_stamp.sec),
                "nanosec": int(export_result.snapshot_stamp.nanosec),
            },
            "bounds": {
                "min": [meta.min_bound.x, meta.min_bound.y, meta.min_bound.z],
                "max": [meta.max_bound.x, meta.max_bound.y, meta.max_bound.z],
            },
            "planner": planner_metadata_from_response(meta),
            "layers": {
                "preblocked_count": int(preblocked_points.shape[0]),
                "external_preblocked_count": int(
                    external_preblocked_points.shape[0]
                ),
                "traversable_count": int(traversable_points.shape[0]),
                "risk_cost_count": int(risk_points.shape[0]),
            },
        }

        staging_dir: Optional[Path] = None
        try:
            staging_dir = Path(
                tempfile.mkdtemp(
                    prefix=f".{package_dir.name}.staging-",
                    dir=str(package_dir.parent),
                )
            )
            octomap_file = staging_dir / "octomap_msg.npz"
            layers_file = staging_dir / "layers.npz"
            meta_file = staging_dir / "meta.yaml"
            np.savez_compressed(
                octomap_file,
                binary=np.array([octomap_msg.binary], dtype=np.bool_),
                octomap_id=np.array([octomap_msg.id]),
                resolution=np.array([octomap_msg.resolution], dtype=np.float64),
                frame_id=np.array([octomap_msg.header.frame_id]),
                data=np.array(octomap_msg.data, dtype=np.int8),
            )
            np.savez_compressed(
                layers_file,
                preblocked_points=preblocked_points,
                preblocked_scale=np.array(
                    [
                        preblocked_msg.scale.x,
                        preblocked_msg.scale.y,
                        preblocked_msg.scale.z,
                    ],
                    dtype=np.float64,
                ),
                preblocked_frame_id=np.array([preblocked_msg.header.frame_id]),
                external_preblocked_points=external_preblocked_points,
                external_preblocked_scale=external_preblocked_scale,
                external_preblocked_frame_id=np.array(
                    [external_preblocked_frame_id]
                ),
                traversable_points=traversable_points,
                traversable_scale=np.array(
                    [
                        traversable_msg.scale.x,
                        traversable_msg.scale.y,
                        traversable_msg.scale.z,
                    ],
                    dtype=np.float64,
                ),
                traversable_frame_id=np.array([traversable_msg.header.frame_id]),
                risk_points=risk_points[:, :3],
                risk_intensity=risk_points[:, 3],
                risk_frame_id=np.array([risk_cost_msg.header.frame_id]),
            )
            with meta_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(meta_yaml, f, sort_keys=False, allow_unicode=True)
            if not self._snapshot_revisions_match(snapshot):
                raise RuntimeError(
                    "planner snapshot topics changed while files were staged; "
                    "retry save"
                )
            self._commit_staging_directory(
                staging_dir, package_dir, allow_replace=request.overwrite
            )
            staging_dir = None
        except (OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
            response.success = False
            response.message = f"failed to save map package atomically: {exc}"
            return response
        finally:
            if staging_dir is not None and staging_dir.exists():
                shutil.rmtree(staging_dir, ignore_errors=True)

        response.success = True
        response.message = "map package saved"
        response.manifest_path = str(package_dir / "meta.yaml")
        return response

    @staticmethod
    def _remove_exact_path(path: Path) -> None:
        if path.is_symlink() or path.is_file():
            path.unlink()
        elif path.exists():
            shutil.rmtree(path)

    def _commit_staging_directory(
        self, staging: Path, target: Path, allow_replace: bool
    ) -> None:
        """Atomically switch a complete sibling staging directory into place."""
        backup: Optional[Path] = None
        if target.exists() or target.is_symlink():
            if not allow_replace:
                raise FileExistsError(f"package path appeared during save: {target}")
            backup = target.parent / (
                f".{target.name}.backup-{uuid.uuid4().hex}"
            )
            target.rename(backup)
        try:
            staging.rename(target)
        except OSError as exc:
            if backup is not None and backup.exists():
                try:
                    backup.rename(target)
                except OSError as rollback_exc:
                    raise RuntimeError(
                        "failed to install staged package and rollback failed; "
                        f"previous package remains at {backup}: {rollback_exc}"
                    ) from exc
            raise
        if backup is not None:
            try:
                self._remove_exact_path(backup)
            except OSError as exc:
                self.get_logger().warning(
                    f"saved package, but could not remove backup {backup}: {exc}"
                )

    def _handle_load_package(
        self,
        request: LoadNavigationMapPackage.Request,
        response: LoadNavigationMapPackage.Response,
    ) -> LoadNavigationMapPackage.Response:
        if not self._package_operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another map-package save/load operation is active"
            response.map_id = ""
            return response
        try:
            apply_ready, apply_message = self._resolve_uncertain_apply()
            if not apply_ready:
                response.success = False
                response.message = apply_message
                response.map_id = ""
                return response
            self._clear_pending_external_command()
            try:
                success, message, map_id = self._load_package_locked(
                    request.package_path
                )
            except (
                OSError,
                KeyError,
                TypeError,
                ValueError,
                IndexError,
                yaml.YAMLError,
            ) as exc:
                success = False
                message = f"invalid or unreadable map package: {exc}"
                map_id = ""
        finally:
            if "apply_ready" in locals() and apply_ready:
                self._clear_pending_external_command()
            self._package_operation_lock.release()
        response.success = success
        response.message = message
        response.map_id = map_id
        return response

    def _autoload_package_once(self) -> None:
        package_path = str(self.get_parameter("autoload_package_path").value).strip()
        if not package_path:
            self._bootstrap_external_preblocked_once()
            return
        if not self._package_operation_lock.acquire(blocking=False):
            return
        autoload_started = False
        try:
            apply_ready, _ = self._resolve_uncertain_apply()
            if not apply_ready:
                return
            self._clear_pending_external_command()
            autoload_started = True
            try:
                success, message, map_id = self._load_package_locked(package_path)
            except (
                OSError,
                KeyError,
                TypeError,
                ValueError,
                IndexError,
                yaml.YAMLError,
            ) as exc:
                success = False
                message = f"invalid or unreadable map package: {exc}"
                map_id = ""
        finally:
            if autoload_started:
                self._clear_pending_external_command()
            self._package_operation_lock.release()
        if success:
            self._autoload_timer.cancel()
            self.get_logger().info(
                f"autoloaded map package: {package_path} map_id={map_id}"
            )
        elif "service unavailable" in message:
            self.get_logger().debug(
                "planner is not ready for map-package autoload; will retry"
            )
        else:
            self._autoload_timer.cancel()
            self.get_logger().error(
                f"failed to autoload map package {package_path}: {message}"
            )

    def _bootstrap_external_preblocked_once(self) -> None:
        """Recover the planner's authored layer after a manager restart."""
        if not self._package_operation_lock.acquire(blocking=False):
            return
        try:
            apply_ready, _ = self._resolve_uncertain_apply()
            if not apply_ready:
                return
            meta_ok, _, meta = self._call_get_meta()
            if not meta_ok or meta is None:
                return
            marker = copy.deepcopy(getattr(meta, "external_preblocked", None))
            if marker is None or marker.type != Marker.CUBE_LIST:
                self.get_logger().debug(
                    "planner metadata has no valid external-preblocked marker yet"
                )
                return
            if not marker.header.frame_id:
                marker.header.frame_id = meta.frame_id
            resolution = float(meta.resolution)
            if resolution > 0.0:
                if marker.scale.x <= 0.0:
                    marker.scale.x = resolution
                if marker.scale.y <= 0.0:
                    marker.scale.y = resolution
                if marker.scale.z <= 0.0:
                    marker.scale.z = resolution
            export_ok, _, _, snapshot = self._call_export_snapshot(
                external_override=marker
            )
            if not export_ok or snapshot is None:
                return
            meta_after_ok, _, meta_after = self._call_get_meta()
            if (
                not meta_after_ok
                or meta_after is None
                or self._meta_signature(meta) != self._meta_signature(meta_after)
                or not self._snapshot_revisions_match(snapshot)
            ):
                return
            try:
                planner_values = self._planner_values_from_meta(meta_after)
            except (TypeError, ValueError) as exc:
                self.get_logger().warning(
                    f"cannot bootstrap invalid planner metadata: {exc}"
                )
                return
            durable_octomap = snapshot.get("octomap")
            authoritative_external = snapshot.get("external")
            if not isinstance(durable_octomap, Octomap) or not isinstance(
                authoritative_external, Marker
            ):
                return
            self._publish_applied_authority(
                authoritative_external, durable_octomap
            )
            self._cache_navigation_authority(
                durable_octomap, authoritative_external, planner_values
            )
            self._autoload_timer.cancel()
            self.get_logger().info(
                "restored authoritative external-preblocked layer from planner"
            )
        finally:
            self._package_operation_lock.release()

    @staticmethod
    def _package_member_path(
        package_dir: Path, member_name: object, label: str
    ) -> Path:
        relative = Path(str(member_name))
        if relative.is_absolute():
            raise ValueError(f"{label} must be relative to the package directory")
        root = package_dir.resolve()
        candidate = (root / relative).resolve()
        try:
            candidate.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"{label} escapes the package directory") from exc
        if not candidate.is_file():
            raise OSError(f"{label} not found: {candidate}")
        return candidate

    @staticmethod
    def _read_npz_archive(path: Path) -> dict[str, np.ndarray]:
        with np.load(path, allow_pickle=False) as archive:
            # NPZ members are lazy. Copy every member now so a corrupt cached
            # layer cannot be discovered only after the planner was changed.
            return {name: np.array(archive[name], copy=True) for name in archive.files}

    @staticmethod
    def _archive_scalar(archive: dict[str, np.ndarray], key: str):
        if key not in archive:
            raise KeyError(f"missing NPZ member: {key}")
        values = np.asarray(archive[key]).reshape(-1)
        if values.size != 1:
            raise ValueError(f"NPZ member {key} must contain exactly one value")
        return values[0]

    @staticmethod
    def _validated_points(
        archive: dict[str, np.ndarray], key: str
    ) -> np.ndarray:
        values = np.asarray(archive[key], dtype=np.float64)
        if values.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        if values.size % 3 != 0:
            raise ValueError(f"NPZ member {key} is not an Nx3 point array")
        values = values.reshape((-1, 3))
        if not np.all(np.isfinite(values)):
            raise ValueError(f"NPZ member {key} contains non-finite points")
        return values.astype(np.float32)

    @staticmethod
    def _validated_scale(
        archive: dict[str, np.ndarray], key: str
    ) -> np.ndarray:
        values = np.asarray(archive[key], dtype=np.float64).reshape(-1)
        if values.size != 3 or not np.all(np.isfinite(values)):
            raise ValueError(f"NPZ member {key} must be a finite xyz scale")
        if np.any(values <= 0.0):
            raise ValueError(f"NPZ member {key} must be positive")
        return values

    def _validate_layer_archive(
        self, archive: dict[str, np.ndarray], meta: dict
    ) -> None:
        point_keys = (
            "occupied_points",
            "preblocked_points",
            "external_preblocked_points",
            "traversable_points",
            "risk_points",
        )
        validated_points: dict[str, np.ndarray] = {}
        for key in point_keys:
            if key in archive:
                validated_points[key] = self._validated_points(archive, key)
        for key in (
            "occupied_scale",
            "preblocked_scale",
            "external_preblocked_scale",
            "traversable_scale",
        ):
            if key in archive:
                self._validated_scale(archive, key)
        for key in (
            "occupied_frame_id",
            "preblocked_frame_id",
            "external_preblocked_frame_id",
            "traversable_frame_id",
            "risk_frame_id",
        ):
            if key in archive and not str(self._archive_scalar(archive, key)):
                raise ValueError(f"NPZ member {key} is empty")
        layer_groups = {
            "occupied": (
                "occupied_points",
                "occupied_scale",
                "occupied_frame_id",
            ),
            "preblocked": (
                "preblocked_points",
                "preblocked_scale",
                "preblocked_frame_id",
            ),
            "external_preblocked": (
                "external_preblocked_points",
                "external_preblocked_scale",
                "external_preblocked_frame_id",
            ),
            "traversable": (
                "traversable_points",
                "traversable_scale",
                "traversable_frame_id",
            ),
            "risk": ("risk_points", "risk_intensity", "risk_frame_id"),
        }
        for label, keys in layer_groups.items():
            if any(key in archive for key in keys) and not all(
                key in archive for key in keys
            ):
                missing = [key for key in keys if key not in archive]
                raise KeyError(
                    f"incomplete {label} layer; missing {', '.join(missing)}"
                )
        if "risk_intensity" in archive:
            intensity = np.asarray(archive["risk_intensity"], dtype=np.float64)
            intensity = intensity.reshape(-1)
            if not np.all(np.isfinite(intensity)):
                raise ValueError("NPZ member risk_intensity is not finite")
            risk_count = validated_points.get(
                "risk_points", np.empty((0, 3), dtype=np.float32)
            ).shape[0]
            if intensity.size != risk_count:
                raise ValueError("risk_points and risk_intensity lengths differ")
        layer_meta = meta.get("layers", {}) or {}
        if not isinstance(layer_meta, dict):
            raise TypeError("metadata layers entry must be a mapping")
        count_keys = {
            "preblocked_count": "preblocked_points",
            "external_preblocked_count": "external_preblocked_points",
            "traversable_count": "traversable_points",
            "risk_cost_count": "risk_points",
        }
        for count_key, points_key in count_keys.items():
            if count_key in layer_meta:
                if points_key not in validated_points:
                    raise KeyError(
                        f"metadata {count_key} has no archive point layer"
                    )
                expected = int(layer_meta[count_key])
                actual = int(validated_points[points_key].shape[0])
                if expected != actual:
                    raise ValueError(
                        f"metadata {count_key}={expected}, archive has {actual}"
                    )

    def _load_package_locked(self, package_path: str) -> tuple[bool, str, str]:
        requested_path = str(package_path).strip()
        if not requested_path:
            return False, "package path is empty", ""
        package_dir = Path(requested_path).expanduser()
        meta_file = package_dir / "meta.yaml"
        if not meta_file.exists():
            return False, f"meta file not found: {meta_file}", ""

        with meta_file.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        if not isinstance(meta, dict):
            raise TypeError("meta.yaml must contain a mapping")
        octomap_path = self._package_member_path(
            package_dir, meta["octomap_file"], "octomap_file"
        )
        layers_path = self._package_member_path(
            package_dir, meta["layers_file"], "layers_file"
        )
        octomap_archive = self._read_npz_archive(octomap_path)
        layers_archive = self._read_npz_archive(layers_path)
        self._validate_layer_archive(layers_archive, meta)

        frame_id = str(self._archive_scalar(octomap_archive, "frame_id"))
        octomap_id = str(self._archive_scalar(octomap_archive, "octomap_id"))
        resolution = float(self._archive_scalar(octomap_archive, "resolution"))
        if not frame_id or not octomap_id:
            raise ValueError("OctoMap frame_id and octomap_id must be non-empty")
        if not np.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("OctoMap resolution must be finite and positive")
        raw_data = np.asarray(octomap_archive.get("data"))
        if raw_data.dtype.kind not in "iu":
            raise ValueError("OctoMap data must contain integers")
        raw_data = raw_data.reshape(-1)
        if raw_data.size and (
            int(raw_data.min()) < -128 or int(raw_data.max()) > 127
        ):
            raise ValueError("OctoMap data contains values outside int8 range")
        if meta.get("frame_id") and str(meta["frame_id"]) != frame_id:
            raise ValueError("metadata frame_id differs from OctoMap frame_id")
        if "resolution" in meta and not np.isclose(
            float(meta["resolution"]), resolution
        ):
            raise ValueError("metadata resolution differs from OctoMap resolution")

        octomap_msg = Octomap()
        octomap_msg.header.frame_id = frame_id
        octomap_msg.header.stamp = self.get_clock().now().to_msg()
        binary_value = self._archive_scalar(octomap_archive, "binary")
        if isinstance(binary_value, (bool, np.bool_)):
            octomap_msg.binary = bool(binary_value)
        elif isinstance(binary_value, (int, np.integer)) and int(
            binary_value
        ) in (0, 1):
            octomap_msg.binary = bool(int(binary_value))
        else:
            raise ValueError("OctoMap binary must be bool or integer 0/1")
        octomap_msg.id = octomap_id
        octomap_msg.resolution = resolution
        octomap_msg.data = raw_data.astype(np.int8).tolist()

        occupied_msg = None
        if "occupied_points" in layers_archive:
            occupied_msg = self._make_marker_from_points(
                str(self._archive_scalar(layers_archive, "occupied_frame_id")),
                "occupied_voxels",
                self._validated_scale(layers_archive, "occupied_scale"),
                self._validated_points(layers_archive, "occupied_points"),
                (0.95, 0.45, 0.15, 0.95),
            )
        external_points, external_scale, external_frame_id = (
            external_preblocked_layer_from_archive(
                layers_archive,
                str(meta.get("frame_id", octomap_msg.header.frame_id)),
                np.array(
                    [
                        octomap_msg.resolution,
                        octomap_msg.resolution,
                        octomap_msg.resolution,
                    ],
                    dtype=np.float64,
                ),
            )
        )
        expected_frame_id = str(meta.get("frame_id", frame_id) or frame_id)
        if external_frame_id != expected_frame_id:
            raise ValueError(
                "external_preblocked_frame_id differs from the OctoMap frame; "
                "the planner does not transform authored marker coordinates"
            )
        external_preblocked_msg = self._make_marker_from_points(
            external_frame_id,
            "external_preblocked_cells",
            external_scale,
            external_points,
            (0.95, 0.10, 0.10, 0.95),
        )
        values = self._validated_planner_values(meta)
        planner_parameters = [
            self._planner_parameter(name, value) for name, value in values.items()
        ]

        apply_ok, apply_message, apply_result, snapshot = (
            self._call_apply_snapshot(
                octomap_msg,
                external_preblocked_msg,
                clear_navigation=True,
                planner_parameters=planner_parameters,
                authority_values=values,
            )
        )
        if apply_result is None or not apply_result.success:
            return False, apply_message, str(meta.get("map_id", ""))

        authoritative_external = copy.deepcopy(external_preblocked_msg)
        authoritative_external.header.stamp = apply_result.snapshot_stamp
        durable_octomap = copy.deepcopy(octomap_msg)
        durable_octomap.header.stamp = apply_result.snapshot_stamp
        delivered_octomap = self._copy_octomap_at_stamp(
            apply_result.snapshot_stamp
        )
        if delivered_octomap is not None:
            durable_octomap = delivered_octomap
        if snapshot is not None:
            snapshot_external = snapshot.get("external")
            if isinstance(snapshot_external, Marker):
                authoritative_external = copy.deepcopy(snapshot_external)
            snapshot_octomap = snapshot.get("octomap")
            if isinstance(snapshot_octomap, Octomap):
                durable_octomap = copy.deepcopy(snapshot_octomap)
        self._publish_applied_authority(
            authoritative_external, durable_octomap
        )
        self._cache_navigation_authority(
            durable_octomap, authoritative_external, values
        )

        with self._cache_lock:
            self._latest_occupied = (
                copy.deepcopy(occupied_msg) if occupied_msg is not None else None
            )
        if occupied_msg is not None:
            self.occupied_pub.publish(occupied_msg)
        if bool(self.get_parameter("publish_cached_derived_layers_on_load").value):
            self.get_logger().warning(
                "publish_cached_derived_layers_on_load is ignored after an "
                "authoritative planner rebuild"
            )

        selector_ok, selector_message = self._sync_selector_profile(values)
        warnings = []
        if not apply_ok:
            warnings.append(apply_message)
        if not selector_ok:
            warnings.append(selector_message)
            self.get_logger().warning(selector_message)
        detail = ""
        if warnings:
            detail = "; applied state is active, warning: " + "; ".join(warnings)
        return (
            True,
            "map package loaded atomically; planner profile applied with map; "
            f"{selector_message}{detail}",
            str(meta.get("map_id", "")),
        )


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
