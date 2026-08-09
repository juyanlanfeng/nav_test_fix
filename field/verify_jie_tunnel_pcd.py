#!/usr/bin/env python3
"""Offline regression for JIE's two symmetric RMUC lower tunnels.

This mirrors the occupancy, preblocked-cell, ground-support, anisotropic body
and 26-neighbour A* rules used by ``jie_path_node``.  It intentionally limits
the search window to each audited tunnel, making it useful before a slower
full ROS/OctoMap launch.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
from pathlib import Path
import time

import numpy as np


CORRIDORS = (
    {
        "name": "positive_y_low_tunnel",
        "start": (-1.45, 5.95, 0.004),
        "goal": (-0.40, 5.95, 0.004),
    },
    {
        "name": "negative_y_low_tunnel",
        "start": (1.45, -5.95, 0.004),
        "goal": (0.40, -5.95, 0.004),
    },
)


Grid = tuple[int, int, int]


def read_binary_xyz_pcd(path: Path) -> np.ndarray:
    header: dict[str, str] = {}
    with path.open("rb") as stream:
        while True:
            raw = stream.readline()
            if not raw:
                raise RuntimeError("PCD header ended before DATA")
            line = raw.decode("ascii").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key] = value.strip()
            if key == "DATA":
                break
        if header.get("FIELDS") != "x y z" or header.get("DATA") != "binary":
            raise RuntimeError("expected binary XYZ PCD")
        points = np.frombuffer(stream.read(), dtype="<f4")
    expected = int(header["POINTS"])
    if points.size != expected * 3:
        raise RuntimeError("PCD payload size does not match POINTS")
    return points.reshape((-1, 3))


def euclidean(left: Grid, right: Grid) -> float:
    return math.sqrt(sum((left[i] - right[i]) ** 2 for i in range(3)))


class TunnelGrid:
    def __init__(
        self,
        occupied: set[Grid],
        resolution: float,
        radius_xy: float,
        physical_height: float,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> None:
        self.occupied = occupied
        self.resolution = resolution
        self.radius_xy = radius_xy
        self.physical_height = physical_height
        margin_xy = radius_xy + 0.55
        self.min_x = min(start[0], goal[0]) - margin_xy
        self.max_x = max(start[0], goal[0]) + margin_xy
        self.min_y = min(start[1], goal[1]) - margin_xy
        self.max_y = max(start[1], goal[1]) + margin_xy
        self.min_z = -0.15
        self.max_z = 0.65
        self.minimum_occupied_z = min(cell[2] for cell in occupied)
        self.preblocked = self._build_preblocked()

    def world_to_grid(self, point: tuple[float, float, float]) -> Grid:
        return tuple(int(math.floor(value / self.resolution)) for value in point)  # type: ignore[return-value]

    def grid_to_world(self, cell: Grid) -> tuple[float, float, float]:
        return tuple((value + 0.5) * self.resolution for value in cell)  # type: ignore[return-value]

    def inside_window(self, cell: Grid) -> bool:
        x, y, z = self.grid_to_world(cell)
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and self.min_z <= z <= self.max_z
        )

    def _same_level_neighbor(self, cell: Grid, occupied_dz: int) -> bool:
        x, y, z = cell
        return any(
            (x + dx, y + dy, z + occupied_dz) in self.occupied
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx or dy
        )

    def _has_nonoccupied_same_level_neighbor(self, cell: Grid) -> bool:
        x, y, z = cell
        return any(
            (x + dx, y + dy, z) not in self.occupied
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx or dy
        )

    def _build_preblocked(self) -> set[Grid]:
        local_occupied = {cell for cell in self.occupied if self.inside_window(cell)}
        candidates = {
            (x + dx, y + dy, z)
            for x, y, z in local_occupied
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx or dy
        }
        result: set[Grid] = set()
        for cell in candidates:
            if not self.inside_window(cell) or cell in self.occupied:
                continue
            x, y, z = cell
            if (x, y, z - 1) in self.occupied and self._same_level_neighbor(cell, 1):
                result.add(cell)
                continue
            if not self._has_nonoccupied_same_level_neighbor(cell):
                continue
            if (x, y, z + 1) in self.occupied:
                continue
            if (x, y, z - 1) not in self.occupied:
                result.add(cell)
        return result

    def traversable(self, cell: Grid) -> bool:
        if not self.inside_window(cell) or cell in self.occupied or cell in self.preblocked:
            return False
        x, y, z = cell
        if not any(
            (x + dx, y + dy, z - 1) in self.occupied
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
        ):
            return False

        # Match jie_path_node: preblocked voids below a surface cannot be
        # crossed to reach a higher layer.
        for below_z in range(z - 1, self.minimum_occupied_z - 1, -1):
            below = (x, y, below_z)
            if below in self.occupied:
                break
            if below in self.preblocked:
                return False

        n_xy = int(math.ceil(self.radius_xy / self.resolution))
        n_z = int(math.ceil(self.physical_height / self.resolution))
        for dx in range(-n_xy, n_xy + 1):
            for dy in range(-n_xy, n_xy + 1):
                horizontal_sq = (dx * self.resolution) ** 2 + (dy * self.resolution) ** 2
                if horizontal_sq > self.radius_xy**2 + 1.0e-12:
                    continue
                for dz in range(n_z + 1):
                    # Candidate is one cell above its support.  robot_height
                    # is physical height from support/ground, not an extra
                    # height starting at the candidate centre.
                    if (dz + 1) * self.resolution > self.physical_height + 1.0e-12:
                        continue
                    nearby = (x + dx, y + dy, z + dz)
                    if nearby in self.occupied or nearby in self.preblocked:
                        return False
        return True

    def snap(self, seed: Grid, radius_cells: int = 12) -> Grid | None:
        if self.traversable(seed):
            return seed
        for radius in range(1, radius_cells + 1):
            for dz in range(radius + 1):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy), abs(dz)) != radius:
                            continue
                        above = (seed[0] + dx, seed[1] + dy, seed[2] + dz)
                        if self.traversable(above):
                            return above
                        if dz:
                            below = (seed[0] + dx, seed[1] + dy, seed[2] - dz)
                            if self.traversable(below):
                                return below
        return None

    def plan(self, start: Grid, goal: Grid, max_iterations: int = 500_000) -> tuple[list[Grid], int]:
        queue: list[tuple[float, float, Grid]] = [(euclidean(start, goal), 0.0, start)]
        costs = {start: 0.0}
        parents: dict[Grid, Grid] = {}
        closed: set[Grid] = set()
        iterations = 0
        while queue and iterations < max_iterations:
            _, cost, current = heapq.heappop(queue)
            iterations += 1
            if current in closed:
                continue
            closed.add(current)
            if current == goal:
                path = [current]
                while current in parents:
                    current = parents[current]
                    path.append(current)
                path.reverse()
                return path, iterations
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                        if neighbor in closed or not self.traversable(neighbor):
                            continue
                        candidate_cost = cost + math.sqrt(dx * dx + dy * dy + dz * dz)
                        if candidate_cost >= costs.get(neighbor, math.inf):
                            continue
                        costs[neighbor] = candidate_cost
                        parents[neighbor] = current
                        heapq.heappush(
                            queue,
                            (candidate_cost + euclidean(neighbor, goal), candidate_cost, neighbor),
                        )
        return [], iterations


def verify(
    pcd: Path, resolution: float, radius_xy: float, physical_height: float
) -> dict:
    started = time.monotonic()
    points = read_binary_xyz_pcd(pcd)
    occupied = {
        tuple(cell)
        for cell in np.floor(points.astype(np.float64) / resolution).astype(np.int32)
    }
    checks = []
    for corridor in CORRIDORS:
        grid = TunnelGrid(
            occupied,
            resolution,
            radius_xy,
            physical_height,
            corridor["start"],
            corridor["goal"],
        )
        raw_start = grid.world_to_grid(corridor["start"])
        raw_goal = grid.world_to_grid(corridor["goal"])
        start = grid.snap(raw_start)
        goal = grid.snap(raw_goal)
        path: list[Grid] = []
        iterations = 0
        if start is not None and goal is not None:
            path, iterations = grid.plan(start, goal)
        world_path = np.asarray([grid.grid_to_world(cell) for cell in path])
        checks.append(
            {
                "name": corridor["name"],
                "connected": bool(path),
                "raw_start_grid": list(raw_start),
                "raw_goal_grid": list(raw_goal),
                "snapped_start_grid": list(start) if start is not None else None,
                "snapped_goal_grid": list(goal) if goal is not None else None,
                "path_cells": len(path),
                "a_star_iterations": iterations,
                "path_z_range_m": (
                    [float(world_path[:, 2].min()), float(world_path[:, 2].max())]
                    if len(world_path)
                    else None
                ),
                "lower_layer_only": bool(path) and float(world_path[:, 2].max()) < 0.15,
                "local_preblocked_cells": len(grid.preblocked),
            }
        )
    return {
        "pcd": str(pcd.resolve()),
        "pcd_points": int(len(points)),
        "occupied_voxels": len(occupied),
        "resolution_m": resolution,
        "robot_radius_xy_m": radius_xy,
        "robot_physical_height_m": physical_height,
        "height_reference": "support_ground_to_robot_top",
        "checks": checks,
        "all_connected_on_lower_layer": all(
            check["connected"] and check["lower_layer_only"] for check in checks
        ),
        "elapsed_seconds": time.monotonic() - started,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--robot-radius-xy", type=float, default=0.28)
    parser.add_argument("--robot-height", type=float, default=0.225)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = verify(args.pcd, args.resolution, args.robot_radius_xy, args.robot_height)
    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output + "\n", encoding="utf-8")
    return 0 if report["all_connected_on_lower_layer"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
