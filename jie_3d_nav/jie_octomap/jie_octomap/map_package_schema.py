"""Pure helpers for version-tolerant navigation map packages."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np


# The RMUC2026 0.05 m map currently needs about 158 seconds to rebuild all
# planner-derived layers.  Keep the internal planner wait below the outer GUI
# service wait so failures still return a useful message instead of timing out
# first in the client.
PLANNER_EXPORT_TIMEOUT_SEC = 300.0
MAP_PACKAGE_SAVE_TIMEOUT_SEC = 360.0
MAP_PACKAGE_LOAD_TIMEOUT_SEC = 360.0


def planner_metadata_from_response(response: Any) -> dict[str, Any]:
    """Serialize every parameter which affects planner-derived map layers."""
    return {
        "robot_radius": response.robot_radius,
        "robot_radius_xy": response.robot_radius_xy,
        "robot_height": response.robot_height,
        "snap_search_radius_cells": response.snap_search_radius_cells,
        "require_ground_support": response.require_ground_support,
        "strict_direct_ground_support": response.strict_direct_ground_support,
        "ground_support_xy_radius_cells": response.ground_support_xy_radius_cells,
        "ground_support_depth_cells": response.ground_support_depth_cells,
        "lowest_traversable_only": response.lowest_traversable_only,
        "enable_preblocked_costmap": response.enable_preblocked_costmap,
        "preblocked_costmap_radius_cells": response.preblocked_costmap_radius_cells,
        "preblocked_costmap_weight": response.preblocked_costmap_weight,
    }


def planner_parameters_from_metadata(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Return the complete planner state represented by map-package metadata."""
    planner = metadata.get("planner", {}) or {}
    return {
        "frame_id": str(metadata.get("frame_id", "map") or "map"),
        "map_id": str(metadata.get("map_id", "loaded_map") or "loaded_map"),
        "source_world_file": str(metadata.get("source_world_file", "") or ""),
        "robot_radius": float(planner.get("robot_radius", 0.25)),
        "robot_radius_xy": float(planner.get("robot_radius_xy", -1.0)),
        "robot_height": float(planner.get("robot_height", -1.0)),
        "snap_search_radius_cells": int(planner.get("snap_search_radius_cells", 12)),
        "require_ground_support": bool(planner.get("require_ground_support", True)),
        "strict_direct_ground_support": bool(
            planner.get("strict_direct_ground_support", False)
        ),
        "ground_support_xy_radius_cells": int(
            planner.get("ground_support_xy_radius_cells", 1)
        ),
        "ground_support_depth_cells": int(
            planner.get("ground_support_depth_cells", 1)
        ),
        "lowest_traversable_only": bool(planner.get("lowest_traversable_only", False)),
        "enable_preblocked_costmap": bool(
            planner.get("enable_preblocked_costmap", True)
        ),
        "preblocked_costmap_radius_cells": int(
            planner.get("preblocked_costmap_radius_cells", 3)
        ),
        "preblocked_costmap_weight": float(
            planner.get("preblocked_costmap_weight", 2.5)
        ),
    }


def external_preblocked_layer_from_archive(
    archive: Mapping[str, Any],
    default_frame_id: str,
    default_scale: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, str]:
    """Load only the authored external-preblocked layer from an NPZ-like object.

    Packages written before this layer was stored independently are interpreted
    as having no authored cells.  Their aggregate ``preblocked_points`` snapshot
    must never be promoted to an external layer because it also contains cells
    derived automatically by the planner.
    """
    fallback_scale = np.asarray(default_scale, dtype=np.float64).reshape(3)
    if "external_preblocked_points" not in archive:
        return (
            np.empty((0, 3), dtype=np.float32),
            fallback_scale,
            str(default_frame_id or "map"),
        )

    points = np.asarray(archive["external_preblocked_points"], dtype=np.float32)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    else:
        points = points.reshape((-1, 3))

    scale = fallback_scale
    if "external_preblocked_scale" in archive:
        scale = np.asarray(archive["external_preblocked_scale"], dtype=np.float64).reshape(3)

    frame_id = str(default_frame_id or "map")
    if "external_preblocked_frame_id" in archive:
        stored_frame = np.asarray(archive["external_preblocked_frame_id"]).reshape(-1)
        if stored_frame.size and str(stored_frame[0]):
            frame_id = str(stored_frame[0])
    return points, scale, frame_id
