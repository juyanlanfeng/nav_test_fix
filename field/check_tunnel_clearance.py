#!/usr/bin/env python3
"""Sweep the low-clearance PB vehicle through the RMUC tunnels.

Implements acceptance items 2 and 3 of doc/PB_LOW_CLEARANCE_VEHICLE_PLAN.md:

* item 2 (static clearance): the whole tunnel interior of both tunnels is
  sampled, not just the centre point, and every vehicle collision primitive
  (body, four wheels, both lidars and the camera) is taken into account;
* item 3 (approach attitude): the chassis is fitted onto the real support
  surface through its four wheel contact points, so the roll/pitch of the
  vehicle - and therefore the swept height of the body and sensors - is what the
  terrain actually imposes, not a flat assumption.

The RMUC field is a multi-layer mesh (floor, tunnel roof, raised platform), so a
point's height is resolved by *layer continuity*: the support surface is the
surface closest to the expected ground height, and the ceiling is the next
surface above it.  A vehicle point below its support is a penetration; a point
above its ceiling is a head strike; a wheel with no surface under it is missing
support.

Usage:
    field/.step_convert_venv/bin/python field/check_tunnel_clearance.py \
        [--step 0.02] [--lateral -0.05,0.0,0.05] [--yaw-offsets-deg -3,0,3]
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

from pb_vehicle_adapter.robot_envelope import link_poses, parse_collisions, primitive_points


ROOT = Path("/home/rainple/nav_test")
DEFAULT_STL = (
    ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim"
    / "models/rmuc2026_field/meshes/rmuc2026_field_collision.stl"
)
DEFAULT_URDF = ROOT / "meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf"
DEFAULT_OUT = ROOT / "field/converted_rmuc2026/tunnel_clearance"

# Tracks come from the project's documented tunnel poses (doc/CONVERSION_AND_USAGE.md
# section on the tunnel acceptance): the +Y tunnel connects the lower floor with
# the ramp side, the -Y tunnel is its point-symmetric counterpart.
TRACKS = {
    "plus_y": {"start": (-1.45, 5.95), "end": (-0.40, 5.95), "expected_z": 0.0039},
    "minus_y": {"start": (0.40, -5.95), "end": (1.45, -5.95), "expected_z": 0.0030},
}
WHEEL_OFFSETS = ((0.207, 0.194), (0.207, -0.194), (-0.207, 0.194), (-0.207, -0.194))
PEN_EPS = 2.0e-3
CONTACT_TOL = 5.0e-3
CEILING_MIN_GAP = 0.02


def crop_mesh(mesh, lo, hi, margin=0.5):
    """Keep triangles overlapping the box (a strict test drops straddling ones)."""
    lo = np.asarray(lo, dtype=float) - margin
    hi = np.asarray(hi, dtype=float) + margin
    bounds = np.asarray(mesh.triangles).reshape(-1, 3, 3)
    tri_lo = bounds.min(axis=1)
    tri_hi = bounds.max(axis=1)
    keep = np.all(tri_hi >= lo, axis=1) & np.all(tri_lo <= hi, axis=1)
    return mesh.submesh([np.flatnonzero(keep)], append=True)


def hits_below(mesh, x, y, z):
    """Every surface below `z` at (x, y) as (height, face_normal_z)."""
    locations, index_ray, index_tri = mesh.ray.intersects_location(
        np.array([[x, y, z]]), np.array([[0.0, 0.0, -1.0]]), multiple_hits=True)
    if len(locations) == 0:
        return []
    normals = mesh.face_normals[index_tri]
    return sorted(zip((float(value) for value in locations[:, 2]),
                      (float(value) for value in normals[:, 2])))


def support_and_ceiling(mesh, x, y, expected_z):
    """Support surface nearest `expected_z`, plus the lowest real overhang above it.

    A surface only counts as a ceiling when its face normal points downwards: the
    ramp or plateau that lies ahead and above the floor has an upward normal and
    is terrain, not an overhang, so a body point above it is not a head strike.
    """
    heights = hits_below(mesh, x, y, 4.0)
    if not heights:
        return None, None
    support = min(heights, key=lambda item: abs(item[0] - expected_z))[0]
    overhangs = [height for height, normal_z in heights
                 if height > support + CEILING_MIN_GAP and normal_z < -0.3]
    ceiling = min(overhangs) if overhangs else None
    return support, ceiling


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def chassis_pose_from_terrain(mesh, centre, yaw, expected_z):
    """Place the rigid chassis on the terrain through its four wheel contacts.

    The plane is fitted through the four contact heights and then *lifted* until
    no wheel is below the terrain, which is how a rigid body really rests on a
    slope break (it touches the highest contact and the others float).  Skipping
    the lift would report phantom wheel penetration exactly at the ramp toes.
    """
    ca, sa = math.cos(yaw), math.sin(yaw)
    lifts = []
    for offset_x, offset_y in WHEEL_OFFSETS:
        wx = centre[0] + ca * offset_x - sa * offset_y
        wy = centre[1] + sa * offset_x + ca * offset_y
        support, _ceiling = support_and_ceiling(mesh, wx, wy, expected_z)
        if support is None:
            return None, None
        lifts.append((wx, wy, support))
    contacts = np.asarray(lifts, dtype=float)
    centroid = contacts.mean(axis=0)
    _u, _s, vh = np.linalg.svd(contacts - centroid)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    forward = np.array([ca, sa, 0.0])
    x_axis = forward - float(forward @ normal) * normal
    norm = float(np.linalg.norm(x_axis))
    x_axis = x_axis / norm if norm > 1e-9 else forward
    y_axis = np.cross(normal, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    if abs(normal[2]) > 1e-6:
        plane_offset = float(normal @ centroid)
        heights = [(plane_offset - normal[0] * wx - normal[1] * wy) / normal[2]
                   for wx, wy, _tz in contacts]
        lift = max(terrain - plane for terrain, plane in zip(contacts[:, 2], heights))
        if lift > 0.0:
            centroid = centroid + normal * lift
    pose = np.eye(4)
    pose[:3, 0] = x_axis
    pose[:3, 1] = y_axis
    pose[:3, 2] = normal
    pose[:3, 3] = centroid
    return pose, contacts


def vehicle_points(urdf_path, samples_per_ring=24):
    """Every collision primitive's surface points in base_footprint."""
    root, primitives = parse_collisions(urdf_path)
    poses = link_poses(root)
    points = []
    for link, name, kind, dims, origin in primitives:
        local = primitive_points(kind, dims)
        if kind == "cylinder":
            angles = np.linspace(0.0, 2.0 * math.pi, samples_per_ring, endpoint=False)
            radius, length = dims
            ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
            local = np.vstack([
                np.column_stack([ring, np.full(len(ring), length / 2.0)]),
                np.column_stack([ring, np.full(len(ring), -length / 2.0)]),
            ])
        transform = poses[link] @ origin
        world = local @ transform[:3, :3].T + transform[:3, 3]
        for point in world:
            points.append((point, "%s:%s" % (link, name), "wheel" in link))
    return np.asarray([item[0] for item in points]), [item[1] for item in points], \
        np.asarray([item[2] for item in points])


def sweep_track(mesh, urdf_path, name, track, step, lateral_offsets, yaw_offsets_deg,
                margin=0.35):
    start = np.array(track["start"], dtype=float)
    end = np.array(track["end"], dtype=float)
    direction = end - start
    length = float(np.linalg.norm(direction))
    unit = direction / length
    yaw = math.atan2(unit[1], unit[0])
    lateral_dir = np.array([-unit[1], unit[0]])

    lo = np.minimum(start, end) - 0.6
    hi = np.maximum(start, end) + 0.6
    region = crop_mesh(mesh, [lo[0], lo[1], -0.5], [hi[0], hi[1], 4.0])

    points, labels, is_wheel = vehicle_points(urdf_path)
    stations = []
    for distance in np.arange(-margin, length + margin + 1e-9, step):
        base = start + unit * distance
        for lateral in lateral_offsets:
            for yaw_offset in yaw_offsets_deg:
                centre = base + lateral_dir * lateral
                pose, contacts = chassis_pose_from_terrain(
                    region, centre, yaw + math.radians(yaw_offset),
                    track["expected_z"] + 0.02 * distance)
                if pose is None:
                    stations.append({"distance": round(float(distance), 4),
                                     "lateral": lateral, "yaw_offset_deg": yaw_offset,
                                     "result": "no_wheel_support"})
                    continue
                world = points @ pose[:3, :3].T + pose[:3, 3]
                # Support expectation per point is the rigid wheel plane, so a
                # front wheel on the ramp start is not compared against the mean.
                normal = pose[:3, 2]
                plane_offset = float(normal @ pose[:3, 3])
                floor_margin = 1e9
                head_margin = 1e9
                body_penetration = 0
                wheel_penetration = 0
                head_strike = 0
                no_support = 0
                worst = None
                for point, label, wheel in zip(world, labels, is_wheel):
                    if abs(normal[2]) < 0.2:
                        expected_point = float(contacts[:, 2].mean())
                    else:
                        expected_point = (plane_offset - normal[0] * point[0]
                                          - normal[1] * point[1]) / normal[2]
                    support, ceiling = support_and_ceiling(region, point[0], point[1],
                                                           expected_point)
                    if support is None:
                        if wheel:
                            no_support += 1
                        continue
                    bottom = point[2] - support
                    if bottom < -PEN_EPS:
                        # A rigid plane over-predicts wheel/terrain interference at
                        # slope breaks, so wheel overlap is a warning; anything
                        # else below the terrain is a real bottoming-out failure.
                        if wheel:
                            wheel_penetration += 1
                        else:
                            body_penetration += 1
                            if worst is None or bottom < worst[1]:
                                worst = (label, float(bottom), float(point[2]), float(support))
                    if wheel:
                        floor_margin = min(floor_margin, bottom)
                    if ceiling is not None:
                        head = ceiling - point[2]
                        if head < -PEN_EPS:
                            head_strike += 1
                            if worst is None or head < worst[1]:
                                worst = (label, float(head), float(point[2]), float(ceiling))
                        head_margin = min(head_margin, head)
                centre_support, centre_ceiling = support_and_ceiling(
                    region, centre[0], centre[1], expected_point)
                stations.append({
                    "distance": round(float(distance), 4),
                    "lateral": lateral,
                    "yaw_offset_deg": yaw_offset,
                    "under_overhang": centre_ceiling is not None,
                    "result": "pass" if (body_penetration == 0 and head_strike == 0
                                         and no_support == 0) else "fail",
                    "body_penetration_points": body_penetration,
                    "wheel_terrain_warnings": wheel_penetration,
                    "head_strike_points": head_strike,
                    "no_support_wheels": no_support,
                    "floor_margin_m": round(float(floor_margin), 5) if floor_margin < 1e8 else None,
                    "head_margin_m": round(float(head_margin), 5) if head_margin < 1e8 else None,
                    "worst": worst,
                })
    return {
        "tunnel": name,
        "start": list(track["start"]),
        "end": list(track["end"]),
        "length_m": round(length, 4),
        "step_m": step,
        "stations": stations,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-stl", default=str(DEFAULT_STL))
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--step", type=float, default=0.02)
    parser.add_argument("--lateral", default="-0.05,0.0,0.05",
                        help="comma list of lateral offsets from the track centre [m]")
    parser.add_argument("--yaw-offsets-deg", default="-3,0,3")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUT))
    arguments = parser.parse_args()

    lateral_offsets = [float(value) for value in arguments.lateral.split(",")]
    yaw_offsets = [float(value) for value in arguments.yaw_offsets_deg.split(",")]
    mesh = trimesh.load(arguments.collision_stl, process=False)
    output = Path(arguments.output_dir)
    output.mkdir(parents=True, exist_ok=True)

    report = {"collision_stl": arguments.collision_stl, "urdf": arguments.urdf,
              "tunnels": [], "summary": {}}
    all_pass = True
    for name, track in TRACKS.items():
        result = sweep_track(mesh, arguments.urdf, name, track, arguments.step,
                             lateral_offsets, yaw_offsets)
        stations = result["stations"]
        failures = [s for s in stations if s["result"] != "pass"]
        interior = [s for s in stations if s.get("under_overhang")]
        floor = [s["floor_margin_m"] for s in stations if s.get("floor_margin_m") is not None]
        head = [s["head_margin_m"] for s in stations if s.get("head_margin_m") is not None]
        result["summary"] = {
            "stations": len(stations),
            "interior_stations": len(interior),
            "interior_failures": len([s for s in interior if s["result"] != "pass"]),
            "wheel_terrain_warnings": len([s for s in stations
                                           if s.get("wheel_terrain_warnings")]),
            "failures": len(failures),
            "worst_floor_margin_m": round(min(floor), 5) if floor else None,
            "worst_head_margin_m": round(min(head), 5) if head else None,
            "first_failures": failures[:5],
        }
        all_pass = all_pass and not failures
        report["tunnels"].append(result)
        print("=== tunnel %s (%.0f mm long, %d stations) ==="
              % (name, result["length_m"] * 1000, len(stations)))
        print("  stations under the overhang (tunnel interior): %d, failures there: %d"
              % (len(interior), len([s for s in interior if s["result"] != "pass"])))
        print("  failures: %d (wheel-terrain warnings: %d)"
              % (len(failures), result["summary"]["wheel_terrain_warnings"]))
        print("  worst wheel-to-floor margin: %s m" % result["summary"]["worst_floor_margin_m"])
        print("  worst ceiling margin:        %s m" % result["summary"]["worst_head_margin_m"])
        for failure in failures[:5]:
            print("  FAIL at distance %.2f m lateral %+.2f yaw %+.0f deg: %s"
                  % (failure["distance"], failure["lateral"], failure["yaw_offset_deg"],
                     failure["worst"] or failure["result"]))
    report["summary"] = {"pass": all_pass}
    (output / "tunnel_clearance_report.json").write_text(json.dumps(report, indent=2))
    print()
    print("overall: %s (report: %s)"
          % ("PASS" if all_pass else "FAIL", output / "tunnel_clearance_report.json"))
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
