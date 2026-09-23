"""Offline whole-robot clearance check for ramp corridors (phase 3).

Given the *current* expanded robot collision geometry, the field collision mesh and
a corridor centre line, this tool sweeps candidate robot poses along the corridor and
reports the minimum clearance between the robot surface and the field, plus any
contact/penetration candidates.

This is a conservative *offline* validator: it samples the robot surface and measures
the closest distance to the field mesh, so it can flag touching but is not a
watertight collision proof, and it does not model runtime dynamic obstacles.

Admissible wheel-on-ground support is separated from real obstacle contact by part
and by the contacted face normal: only a *wheel* resting on an upward-facing face
(normal z > 0.5) is treated as support.  Wheels on walls or ceilings, and any
contact of the body/laser (including with the ground), are reported as obstacles.
The nominal pose pitch follows the centre-line slope.  This is a heuristic, so
concave fillets at ramp edges can read as sub-centimetre clearance even where the
robot physically passes; interpret the metric as a conservative lower bound and
compare candidate corridors against each other rather than against a hard zero.

Run with the conversion venv so `trimesh`/`numpy` are available:

    field/.step_convert_venv/bin/python field/check_ramp_clearance.py

The default centre line is the measured RMUC2026 short ramp (see the module-level
constant).  Coordinates are metres in the field/collision/map frame, matching the
MeshNav `rmuc2026_field` map and the ground-truth localization.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
SIM_PKG = ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim"
COLLISION_STL = SIM_PKG / "models/rmuc2026_field/meshes/rmuc2026_field_collision.stl"
ROBOT_XACRO = SIM_PKG / "urdf/ceres.urdf.xacro"

# Vertical epsilon [m] for the penetration proxy: a wheel sample below its
# closest field surface by more than this is considered penetrating the ground
# rather than resting on it.  Sized above the ~1 mm triangulation error of the
# collision STL so mesh discretisation is not reported as penetration.
PEN_EPS = 5e-3

# Measured RMUC2026 short-ramp centre line (see doc/MESHNAV_SLOPE_PHYSICS_AUDIT_20260907.md
# and the collision-mesh sampling; y = 5.95 is the lateral centre of the strip
# y in [5.55, 6.40]).
DEFAULT_CENTERLINE = [
    (-0.7, 5.95, 0.005),
    (-0.5, 5.95, 0.005),
    (-0.3, 5.95, 0.008),
    (-0.1, 5.95, 0.045),
    (0.1, 5.95, 0.082),
    (0.3, 5.95, 0.123),
    (0.5, 5.95, 0.163),
    (0.7, 5.95, 0.193),
    (0.9, 5.95, 0.203),
]


def parse_vec3(text):
    parts = [float(x) for x in text.split()]
    return parts[0], parts[1], parts[2]


def origin_matrix(origin_el):
    xyz = (0.0, 0.0, 0.0)
    rpy = (0.0, 0.0, 0.0)
    if origin_el is not None:
        if origin_el.get("xyz"):
            xyz = parse_vec3(origin_el.get("xyz"))
        if origin_el.get("rpy"):
            rpy = parse_vec3(origin_el.get("rpy"))
    return rpy_xyz_matrix(rpy, xyz)


def rpy_xyz_matrix(rpy, xyz):
    roll, pitch, yaw = rpy
    x, y, z = xyz
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    m = np.eye(4)
    m[0, 0] = cy * cp
    m[0, 1] = cy * sp * sr - sy * cr
    m[0, 2] = cy * sp * cr + sy * sr
    m[1, 0] = sy * cp
    m[1, 1] = sy * sp * sr + cy * cr
    m[1, 2] = sy * sp * cr - cy * sr
    m[2, 0] = -sp
    m[2, 1] = cp * sr
    m[2, 2] = cp * cr
    m[0, 3] = x
    m[1, 3] = y
    m[2, 3] = z
    return m


def build_robot_urdf_text():
    return subprocess.check_output(
        ["xacro", str(ROBOT_XACRO),
         "slope_aware_drive:=true", "wheel_radius:=0.10", "body_height:=0.10",
         "body_length:=0.32", "body_width:=0.26", "laser2d_mount_z:=0.060",
         "laser3d_mount_z:=0.05", "laser3d_collision:=false"]
    )


def parse_robot_collision(urdf_text):
    """Return robot primitives as (local_mesh, T_base, label, supportable, radius).

    `local_mesh` is centred at the origin in the primitive's own frame (cylinder
    axis along Z); `T_base` maps it into base_footprint.  Wheels additionally
    report their radius so they can be re-placed onto the local terrain height.
    """
    root = ET.fromstring(urdf_text)

    links = {}
    for link in root.findall("link"):
        prims = []
        for col in link.findall("collision"):
            geom = col.find("geometry")
            if geom is None:
                continue
            o = origin_matrix(col.find("origin"))
            if geom.find("box") is not None:
                sx, sy, sz = parse_vec3(geom.find("box").get("size"))
                prims.append(("box", (sx, sy, sz), o, None))
            elif geom.find("cylinder") is not None:
                r = float(geom.find("cylinder").get("radius"))
                length = float(geom.find("cylinder").get("length"))
                prims.append(("cylinder", (r, length), o, r))
            elif geom.find("sphere") is not None:
                prims.append(("sphere", float(geom.find("sphere").get("radius")), o, None))
        links[link.get("name")] = prims

    joints = []
    for j in root.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        joints.append((parent, child, origin_matrix(j.find("origin"))))

    # Resolve each link pose in base_footprint via the joint tree (all joints are
    # treated at their zero/fixed origin; the wheel cylinders are coaxial with their
    # rotation axis so this preserves the swept collision volume).
    link_pose = {"base_footprint": np.eye(4)}
    changed = True
    while changed:
        changed = False
        for parent, child, o in joints:
            if parent in link_pose and child not in link_pose:
                link_pose[child] = link_pose[parent] @ o
                changed = True

    prims_out = []
    for link_name, prims in links.items():
        if link_name not in link_pose:
            continue
        for prim in prims:
            kind, dims, o, radius = prim
            local = None
            if kind == "box":
                local = trimesh.creation.box(extents=dims)
            elif kind == "cylinder":
                r, length = dims
                local = trimesh.creation.cylinder(radius=r, height=length, sections=48)
            elif kind == "sphere":
                local = trimesh.creation.uv_sphere(radius=dims, count=[16, 16])
            if local is None:
                continue
            # `o` places the primitive inside its link frame; `link_pose` then
            # places the link inside base_footprint.
            T_base = link_pose[link_name] @ o
            # Only wheels legitimately rest on the ground; every other part must
            # keep clear of *all* surfaces, including the ground and any ceiling.
            supportable = "wheel" in link_name
            prims_out.append((local, T_base, f"{link_name}:{kind}", supportable, radius))
    return prims_out


def support_surface_z(mesh, x, y, expected_z, zmax=5.0):
    """Support-surface height at (x, y) by height continuity; None if no ground.

    Casts straight down and picks the surface whose height is closest to
    `expected_z` (the current terrain layer) rather than the highest hit, so an
    overhanging ceiling above the robot is not mistaken for the support surface.
    Returns None when the ray hits nothing, i.e. there is no ground to stand on;
    callers must treat that as missing support, not fabricate a height.
    """
    locs, _, _ = mesh.ray.intersects_location(
        np.array([[x, y, zmax]]), np.array([[0.0, 0.0, -1.0]]), multiple_hits=True)
    if len(locs) == 0:
        return None
    zs = locs[:, 2]
    return float(zs[int(np.argmin(np.abs(zs - expected_z)))])


def chassis_pose_from_terrain(mesh, cx, cy, yaw, wheel_offsets, expected_z):
    """Fit the whole rigid chassis onto the terrain; return a 4x4 pose.

    The four wheel contact points are sampled on the support surface and a plane
    is fitted through them; the chassis is then oriented so its base plane matches
    that plane while its forward axis stays aligned with `yaw`.  Wheels keep their
    URDF joint offsets (no independent wheel lifting).
    """
    ca, sa = math.cos(yaw), math.sin(yaw)
    pts = []
    for ax, ay in wheel_offsets:
        wx = cx + ca * ax - sa * ay
        wy = cy + sa * ax + ca * ay
        z = support_surface_z(mesh, wx, wy, expected_z)
        if z is None:
            return None  # no ground under this wheel: pose is unsupported
        pts.append((wx, wy, z))
    pts = np.asarray(pts, dtype=float)
    c = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - c)
    n = vh[-1]
    if n[2] < 0.0:
        n = -n
    fwd = np.array([ca, sa, 0.0])
    x_axis = fwd - (float(fwd @ n)) * n
    norm = float(np.linalg.norm(x_axis))
    x_axis = x_axis / norm if norm > 1e-9 else fwd
    y_axis = np.cross(n, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    T = np.eye(4)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = n
    T[:3, 3] = c
    return T


def crop_mesh_to_box(mesh, lo, hi):
    """Keep every triangle whose axis-aligned bounding box *overlaps* the box.

    Using a strict containment test here would silently drop triangles that
    straddle the box boundary, i.e. exactly the walls/roofs that may collide with
    the robot.  Overlap is therefore the safe (conservative) choice.
    """
    tri = mesh.triangles
    tri_min = tri.min(axis=1)
    tri_max = tri.max(axis=1)
    lo = np.asarray(lo)
    hi = np.asarray(hi)
    overlap = np.all(tri_max >= lo, axis=1) & np.all(tri_min <= hi, axis=1)
    faces = np.flatnonzero(overlap)
    if faces.size == 0:
        raise SystemExit("crop box overlaps no field triangles; check the corridor/bbox")
    return mesh.submesh([faces], append=True)


def resample_centerline(points, step):
    """Return points spaced ~step apart along the XY polyline (keeps endpoints)."""
    pts = [np.array(p, dtype=float) for p in points]
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        seg = b - a
        length = float(np.hypot(seg[0], seg[1]))
        n = max(1, int(math.ceil(length / step)))
        for k in range(1, n + 1):
            out.append(a + seg * (k / n))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--centerline", help="x,y,z;x,y,z;... (defaults to the measured short ramp)")
    parser.add_argument("--half-width", type=float, default=0.275)
    parser.add_argument("--step", type=float, default=0.02, help="sweep step along the centre line [m]")
    parser.add_argument("--lateral-offsets", default="0.0", help="comma list of lateral offsets [m]")
    parser.add_argument("--yaw-offsets-deg", default="0.0")
    parser.add_argument("--pitch-offsets-deg", default="0.0",
                        help="extra pitch [deg] added on top of the slope-derived nominal pitch")
    parser.add_argument("--roll-offsets-deg", default="0.0")
    parser.add_argument("--contact-threshold", type=float, default=0.005)
    parser.add_argument("--points-per-primitive", type=int, default=400)
    parser.add_argument("--output-dir", default=str(ROOT / "field/converted_rmuc2026/ramp_clearance"))
    args = parser.parse_args()

    if args.centerline:
        centerline = [tuple(map(float, p.split(","))) for p in args.centerline.split(";")]
    else:
        centerline = DEFAULT_CENTERLINE

    lat_offsets = [float(x) for x in args.lateral_offsets.split(",")]
    yaw_offsets = [math.radians(float(x)) for x in args.yaw_offsets_deg.split(",")]
    pitch_offsets = [math.radians(float(x)) for x in args.pitch_offsets_deg.split(",")]
    roll_offsets = [math.radians(float(x)) for x in args.roll_offsets_deg.split(",")]

    robot = parse_robot_collision(build_robot_urdf_text())
    prim_desc = ", ".join("%s(%s)" % (label, "wheel" if sup else "body") for _, _, label, sup, _ in robot)
    print("Robot collision primitives: " + prim_desc)

    field = trimesh.load(str(COLLISION_STL))
    print(f"Field collision mesh: {len(field.triangles)} triangles, bounds {field.bounds.tolist()}")

    sweep_pts = resample_centerline(centerline, args.step)

    # Crop the field mesh to a generous box around the swept corridor.
    arr = np.array(sweep_pts)
    margin = args.half_width + 0.6
    lo = [arr[:, 0].min() - margin, arr[:, 1].min() - margin, max(field.bounds[0][2], arr[:, 2].min() - 0.6)]
    hi = [arr[:, 0].max() + margin, arr[:, 1].max() + margin, arr[:, 2].max() + 0.8]
    field = crop_mesh_to_box(field, lo, hi)
    print(f"Field mesh cropped to {len(field.triangles)} triangles")

    # Pre-sample each robot primitive in the base_footprint frame; also keep the
    # primitive centre so "wheel bottom" support can be distinguished from the
    # rest of the wheel surface.
    samples = []
    for local, T_base, label, supportable, radius in robot:
        pts = local.sample(args.points_per_primitive)  # primitive-local frame
        samples.append((pts, T_base, label, supportable, radius))

    # Wheel contact offsets in base_footprint (from the URDF joint transforms).
    wheel_offsets = [(T_base[0, 3], T_base[1, 3]) for _, T_base, label, sup, _ in robot if sup]

    rows = []
    contacts = []
    min_clearance = float("inf")
    worst = None

    # Sweep poses along the centre line; yaw follows the local tangent.
    for i, p in enumerate(sweep_pts):
        if i + 1 < len(sweep_pts):
            d = sweep_pts[i + 1] - p
        elif i > 0:
            d = p - sweep_pts[i - 1]
        else:
            d = np.array([1.0, 0.0, 0.0])
        yaw = math.atan2(d[1], d[0])
        # left normal in XY
        nx, ny = -math.sin(yaw), math.cos(yaw)

        for lat in lat_offsets:
            cx = p[0] + nx * lat
            cy = p[1] + ny * lat
            for yaw_off in yaw_offsets:
                for pitch_off in pitch_offsets:
                    for roll_off in roll_offsets:
                        # Whole rigid chassis fitted onto the terrain; wheels keep
                        # their URDF offsets (no independent wheel lifting).  The
                        # support surface is chosen by height continuity with the
                        # corridor layer, not by taking the highest ray hit.
                        W_terrain = chassis_pose_from_terrain(
                            field, cx, cy, yaw + yaw_off, wheel_offsets, expected_z=p[2])
                        if W_terrain is None:
                            # No support surface under at least one wheel: this is a
                            # missing-ground hazard, not a supported pose.
                            row = {
                                "x": round(cx, 4), "y": round(cy, 4), "z": round(float(p[2]), 4),
                                "yaw_deg": round(math.degrees(yaw + yaw_off), 3),
                                "pitch_deg": round(math.degrees(pitch_off), 3),
                                "pitch_nom_deg": 0.0,
                                "roll_deg": round(math.degrees(roll_off), 3),
                                "roll_nom_deg": 0.0,
                                "lateral": round(lat, 4),
                                "min_clearance_all": 0.0,
                                "min_obstacle_clearance": 0.0,
                            }
                            if 0.0 < min_clearance:
                                min_clearance = 0.0
                                worst = row
                            contacts.append({"pose": row, "primitive": "no_support",
                                             "point": [cx, cy, float(p[2])], "distance": 0.0})
                            rows.append(row)
                            continue
                        R = W_terrain[:3, :3]
                        pitch_nom = math.degrees(math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0])))
                        roll_nom = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                        # Extra perturbation rotation on top of the fitted pose.
                        W = W_terrain @ rpy_xyz_matrix((roll_off, pitch_off, 0.0), (0.0, 0.0, 0.0))
                        all_dist = []
                        all_tri = []
                        all_sup = []
                        all_below = []
                        all_pts = []
                        all_cp = []
                        for pts, T_base, label, supportable, radius in samples:
                            M = W @ T_base
                            pts_world = (M @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
                            center_world = (M @ np.r_[0.0, 0.0, 0.0, 1.0])[:3]
                            cp, dist, tri = trimesh.proximity.closest_point(field, pts_world)
                            all_dist.append(dist)
                            all_tri.append(tri)
                            all_sup.append(np.full(len(pts), supportable, dtype=bool))
                            all_below.append(pts_world[:, 2] < center_world[2])
                            all_pts.append(pts_world)
                            all_cp.append(cp)
                        dist_all = np.concatenate(all_dist)
                        tri_all = np.concatenate(all_tri)
                        sup_all = np.concatenate(all_sup)
                        below_all = np.concatenate(all_below)
                        pts_all = np.concatenate(all_pts)
                        cp_all = np.concatenate(all_cp)

                        # Admissible support = a *wheel* touching an upward-facing
                        # surface (nz > 0.5) with the contact on its *lower* half and
                        # without penetrating the surface.  Everything else is an
                        # obstacle: wheel contacts on walls/ceilings or above the
                        # centre, penetrated ground, and any contact of the body/laser
                        # (including with the ground).  The signed normal (not its
                        # absolute value) keeps ceiling undersides flagged.
                        nz = field.face_normals[tri_all, 2]
                        upward = nz > 0.5
                        penetrating = pts_all[:, 2] < (cp_all[:, 2] - PEN_EPS)
                        admissible = sup_all & upward & below_all & (~penetrating)
                        obstacle = ~admissible
                        d_min = float(dist_all.min()) if dist_all.size else float("inf")
                        d_obs = float(dist_all[obstacle].min()) if obstacle.any() else float("inf")

                        row = {
                            "x": round(cx, 4), "y": round(cy, 4), "z": round(float(W_terrain[2, 3]), 4),
                            "yaw_deg": round(math.degrees(yaw + yaw_off), 3),
                            "pitch_deg": round(math.degrees(pitch_off), 3),
                            "pitch_nom_deg": round(pitch_nom, 3),
                            "roll_deg": round(math.degrees(roll_off), 3),
                            "roll_nom_deg": round(roll_nom, 3),
                            "lateral": round(lat, 4),
                            "min_clearance_all": round(d_min, 5),
                            "min_obstacle_clearance": round(d_obs, 5),
                        }
                        if d_obs < min_clearance:
                            min_clearance = d_obs
                            worst = row
                        if d_obs < args.contact_threshold:
                            # locate the offending primitive/sample (obstacle-masked)
                            obs_idx = np.flatnonzero(obstacle)
                            argmin = obs_idx[int(np.argmin(dist_all[obstacle]))]
                            base = 0
                            hit_label = None
                            for pts, _T_base, label, _supportable, _radius in samples:
                                if argmin < base + len(pts):
                                    hit_label = label
                                    hit_local = pts[argmin - base]
                                    break
                                base += len(pts)
                            hit_world = pts_all[argmin]
                            contacts.append({"pose": row, "primitive": hit_label,
                                             "point": hit_world.tolist(), "distance": d_obs})
                        rows.append(row)

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "arguments.json").write_text(json.dumps(vars(args), indent=2))
    with (out / "clearance.csv").open("w") as f:
        if rows:
            keys = list(rows[0].keys())
            f.write(",".join(keys) + "\n")
            for r in rows:
                f.write(",".join(str(r[k]) for k in keys) + "\n")
    (out / "contacts.json").write_text(json.dumps(contacts, indent=2))

    # Human summary.
    print(f"\nPoses swept: {len(rows)}")
    print(f"Minimum obstacle clearance over all poses: {min_clearance:.5f} m "
          f"(ground/support contact excluded)")
    if worst:
        print(f"Worst pose: {worst}")
    print(f"Obstacle contact candidates (distance < {args.contact_threshold} m): {len(contacts)}")
    for c in contacts[:10]:
        print(f"  {c['pose']['x']},{c['pose']['y']},{c['pose']['z']} yaw={c['pose']['yaw_deg']} "
              f"lat={c['pose']['lateral']} -> {c['primitive']} dist={c['distance']:.5f}")
    print(f"\nWrote results to {out}")


if __name__ == "__main__":
    main()
