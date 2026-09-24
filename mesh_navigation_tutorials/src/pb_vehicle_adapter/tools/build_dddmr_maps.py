"""Generate the DDDMR static map artifacts for RMUC2026 from audited geometry.

Produces, under field/converted_rmuc2026/dddmr_nav/:

  rmuc2026_mapcloud.pcd   obstacle (non-walkable) geometry sample
  rmuc2026_mapground.pcd  traversable support surface (floors, ramps, platforms)
  map_manifest.yaml       provenance: source hash, transform, spacing, profile
  validation_report.json  point counts, bounds, NaN/empty and navigability checks

`mapcloud` is the obstacle cloud, not the complete field geometry.  DDDMR's
StaticLayer computes each ground node's distance to the nearest `mapcloud` point
and treats nodes closer than `inscribed_radius` as lethal (a_star_on_pc.cpp), so
a mapcloud that also contains the floor makes every node lethal and the planner
reports "No path found" between any two points.  Upstream is explicit about the
semantics: `occupancy2ground` publishes the vertical wall structure as
`mapcloud` and the free-space surface as `mapground`.  The obstacle sample is
therefore taken from the faces that are not support surfaces (walls, ceilings,
undersides and steep faces).

Both PCDs are float32 x/y/z/intensity in the `map` frame, metres, with intensity
set to 0.0 (it is NOT lidar reflectivity or a terrain label).  Run with the
conversion venv so trimesh/numpy/scipy are available:

    field/.step_convert_venv/bin/python \
      meshnav_demo_ws/src/pb_vehicle_adapter/tools/build_dddmr_maps.py
"""
import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path("/home/rainple/nav_test")
DEFAULT_STL = (
    ROOT / "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials_sim"
    / "models/rmuc2026_field/meshes/rmuc2026_field_collision.stl"
)
DEFAULT_PROFILE = ROOT / "meshnav_demo_ws/src/pb_vehicle_adapter/config/pb_vehicle_profile.yaml"
DEFAULT_OUT = ROOT / "field/converted_rmuc2026/dddmr_nav"


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_pcd_xyzi(path, points):
    """Write a binary PCD with float32 x/y/z/intensity (little endian)."""
    points = np.asarray(points, dtype=np.float32)
    intensity = np.zeros((len(points), 1), dtype=np.float32)
    data = np.hstack([points, intensity]).astype("<f4")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n"
    ) % (len(points), len(points))
    with open(path, "wb") as stream:
        stream.write(header.encode("ascii"))
        stream.write(data.tobytes())
    return len(points)


def voxel_downsample(points, voxel):
    """Keep one point per voxel (uniform spatial density)."""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]


def sample_mesh(mesh, spacing):
    """Area-weighted sample then voxel-downsample to a uniform density."""
    area = float(mesh.area)
    target = max(1000, int(area / max(spacing * spacing * 0.25, 1e-9)))
    points = np.asarray(mesh.sample(min(target, 6_000_000)), dtype=np.float64)
    return voxel_downsample(points, spacing)


def upward_faces(mesh, max_slope_deg):
    """Face indices whose normal is close enough to +Z to be a support surface."""
    nz = mesh.face_normals[:, 2]
    return np.flatnonzero(nz >= math.cos(math.radians(max_slope_deg)))


def filter_clearance(mesh, points, clearance, ceiling_margin=0.05):
    """Drop ground points that have a surface less than `clearance` above them."""
    if len(points) == 0:
        return points
    origins = points + np.array([0.0, 0.0, ceiling_margin])
    directions = np.tile(np.array([0.0, 0.0, 1.0]), (len(points), 1))
    hits, index_ray, _ = mesh.ray.intersects_location(origins, directions, multiple_hits=False)
    if len(hits) == 0:
        return points
    heights = hits[:, 2] - points[index_ray, 2]
    blocked = set(index_ray[heights < clearance].tolist())
    keep = np.array([i for i in range(len(points)) if i not in blocked], dtype=np.int64)
    return points[keep]


def obstacle_faces(mesh, max_slope_deg):
    """Face indices that are NOT support surfaces: walls, ceilings, steep faces."""
    walkable = upward_faces(mesh, max_slope_deg)
    return np.setdiff1d(np.arange(len(mesh.faces)), walkable)


def drop_below_support(points, ground, cell=0.10, z_tol=0.03):
    """Drop obstacle samples that sit at or below the walkable surface there.

    The collision mesh is a closed solid, so its slab undersides (for example the
    floor's own bottom face) would otherwise sit a few centimetres under the
    walkable surface and make every ground node lethal.  Only geometry that rises
    above the *lowest* support surface within one cell is kept, which preserves
    walls, step edges and ceilings while removing undersides.
    """
    if len(points) == 0 or len(ground) == 0:
        return points
    origin = ground[:, :2].min(axis=0) - cell
    ground_cell = np.floor((ground[:, :2] - origin) / cell).astype(np.int64)
    point_cell = np.floor((points[:, :2] - origin) / cell).astype(np.int64)
    shape = ground_cell.max(axis=0) + 3
    min_z = np.full((shape[1], shape[0]), np.inf, dtype=np.float64)
    np.minimum.at(min_z, (ground_cell[:, 1], ground_cell[:, 0]), ground[:, 2])
    # 3x3 minimum so a sample just across a cell border still sees the surface.
    window = np.full_like(min_z, np.inf)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            shifted = np.full_like(min_z, np.inf)
            ys = slice(max(dy, 0), min_z.shape[0] + min(dy, 0))
            xs = slice(max(dx, 0), min_z.shape[1] + min(dx, 0))
            shifted[ys, xs] = min_z[max(-dy, 0):min_z.shape[0] - max(dy, 0),
                                    max(-dx, 0):min_z.shape[1] - max(dx, 0)]
            np.minimum(window, shifted, out=window)
    local_min = window[point_cell[:, 1], point_cell[:, 0]]
    return points[points[:, 2] >= local_min + z_tol]


def navigability_report(ground, cloud, inscribed_radius):
    """Distance from each ground node to the nearest obstacle point.

    DDDMR treats a ground node as lethal when that distance is below the
    configured inscribed radius, so this predicts whether the static graph is
    actually traversable before any live run.
    """
    if len(ground) == 0 or len(cloud) == 0:
        return {"ground_points": int(len(ground)), "obstacle_points": int(len(cloud))}
    try:
        from scipy.spatial import cKDTree
    except ImportError:  # pragma: no cover - scipy is present in the convert venv
        return {"error": "scipy unavailable; navigability not computed"}
    distance, _ = cKDTree(cloud).query(ground, k=1)
    return {
        "inscribed_radius_m": inscribed_radius,
        "ground_points": int(len(ground)),
        "obstacle_points": int(len(cloud)),
        "ground_points_below_inscribed_radius": int((distance < inscribed_radius).sum()),
        "nearest_obstacle_m": {
            "min": round(float(distance.min()), 4),
            "p05": round(float(np.percentile(distance, 5)), 4),
            "p50": round(float(np.percentile(distance, 50)), 4),
            "max": round(float(distance.max()), 4),
        },
    }


def bounds(points):
    return {
        "min": [round(float(v), 4) for v in points.min(axis=0)],
        "max": [round(float(v), 4) for v in points.max(axis=0)],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-stl", default=str(DEFAULT_STL))
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--output-dir", default=str(DEFAULT_OUT))
    parser.add_argument("--cloud-spacing", type=float, default=0.05)
    parser.add_argument("--ground-spacing", type=float, default=0.05)
    parser.add_argument("--max-slope-deg", type=float, default=30.0)
    parser.add_argument("--clearance", type=float, default=0.55,
                        help="minimum vertical clearance kept for ground points [m]")
    parser.add_argument("--inscribed-radius", type=float, default=0.5,
                        help="DDDMR inscribed_radius used for the navigability report [m]")
    parser.add_argument("--no-clearance-check", action="store_true")
    args = parser.parse_args()

    stl = Path(args.collision_stl)
    profile = Path(args.profile)
    out = Path(args.output_dir)
    for required in (stl, profile):
        if not required.is_file():
            raise SystemExit("missing input: %s" % required)
    out.mkdir(parents=True, exist_ok=True)

    print("loading %s ..." % stl)
    mesh = trimesh.load(str(stl), force="mesh")
    print("  %d triangles, area %.1f m^2" % (len(mesh.triangles), mesh.area))

    print("sampling mapground (slope <= %.1f deg, spacing %.3f m) ..."
          % (args.max_slope_deg, args.ground_spacing))
    walkable = upward_faces(mesh, args.max_slope_deg)
    if len(walkable) == 0:
        raise SystemExit("no walkable faces found; check --max-slope-deg")
    ground_mesh = mesh.submesh([walkable], append=True)
    ground = sample_mesh(ground_mesh, args.ground_spacing)
    if not args.no_clearance_check:
        before = len(ground)
        ground = filter_clearance(mesh, ground, args.clearance)
        print("  clearance %.2f m removed %d of %d points" % (args.clearance, before - len(ground), before))
    print("  %d points" % len(ground))

    print("sampling mapcloud (obstacle faces only, spacing %.3f m) ..." % args.cloud_spacing)
    obstacles = obstacle_faces(mesh, args.max_slope_deg)
    if len(obstacles) == 0:
        raise SystemExit("no obstacle faces found; check --max-slope-deg")
    obstacle_mesh = mesh.submesh([obstacles], append=True)
    cloud = sample_mesh(obstacle_mesh, args.cloud_spacing)
    raw_cloud = len(cloud)
    cloud = drop_below_support(cloud, ground)
    print("  %d points from %d of %d faces (dropped %d below the support surface)"
          % (len(cloud), len(obstacles), len(mesh.faces), raw_cloud - len(cloud)))

    cloud_file = out / "rmuc2026_mapcloud.pcd"
    ground_file = out / "rmuc2026_mapground.pcd"
    write_pcd_xyzi(cloud_file, cloud)
    write_pcd_xyzi(ground_file, ground)

    report = {
        "source": {"file": str(stl), "sha256": sha256(stl)},
        "frame": "map",
        "unit": "m",
        "fields": ["x", "y", "z", "intensity"],
        "intensity_semantics": "constant 0.0; not lidar reflectivity or a terrain label",
        "mapcloud": {
            "file": str(cloud_file), "points": len(cloud), "spacing_m": args.cloud_spacing,
            "bounds": bounds(cloud), "nan": int(np.isnan(cloud).any(axis=1).sum()),
            "semantics": "obstacle geometry (non-support faces); DDDMR lethal-distance source",
            "faces": {"total": int(len(mesh.faces)), "obstacle": int(len(obstacles))},
        },
        "mapground": {
            "file": str(ground_file), "points": len(ground), "spacing_m": args.ground_spacing,
            "max_slope_deg": args.max_slope_deg, "clearance_m": args.clearance,
            "clearance_checked": not args.no_clearance_check,
            "bounds": bounds(ground), "nan": int(np.isnan(ground).any(axis=1).sum()),
            "semantics": "traversable support surface",
            "faces": {"total": int(len(mesh.faces)), "support": int(len(walkable))},
        },
        "profile": {"file": str(profile), "sha256": sha256(profile)},
        "navigability": navigability_report(ground, cloud, args.inscribed_radius),
        "terrain_layer_policy": (
            "mapground keeps every upward-facing surface (floors, ramps, platforms and the "
            "tunnel top/bottom) as separate layers; low-clearance points under a ceiling are "
            "removed. No XY highest-only or lowest-only collapsing is applied."
        ),
    }
    (out / "validation_report.json").write_text(json.dumps(report, indent=2))

    manifest = (
        "# DDDMR static map bundle for RMUC2026 (generated; do not hand-edit)\n"
        "frame: map\n"
        "unit: m\n"
        "fields: [x, y, z, intensity]\n"
        "intensity: constant 0.0 (not lidar reflectivity, not a terrain label)\n"
        "mapcloud:\n"
        "  path: %s\n"
        "  sha256: %s\n"
        "  points: %d\n"
        "  semantics: obstacle geometry only (DDDMR computes ground-to-lethal distance from it)\n"
        "mapground:\n"
        "  path: %s\n"
        "  sha256: %s\n"
        "  points: %d\n"
        "  semantics: traversable support surface\n"
        "source:\n"
        "  collision_stl: %s\n"
        "  sha256: %s\n"
        "vehicle_profile:\n"
        "  path: %s\n"
        "  sha256: %s\n"
        "sampling:\n"
        "  cloud_spacing_m: %s\n"
        "  ground_spacing_m: %s\n"
        "  max_slope_deg: %s\n"
        "  clearance_m: %s\n"
        "  inscribed_radius_m: %s\n"
        "transform: identity (geometry already in the map frame)\n"
    ) % (
        cloud_file, sha256(cloud_file), len(cloud),
        ground_file, sha256(ground_file), len(ground),
        stl, report["source"]["sha256"],
        profile, report["profile"]["sha256"],
        args.cloud_spacing, args.ground_spacing, args.max_slope_deg, args.clearance,
        args.inscribed_radius,
    )
    (out / "map_manifest.yaml").write_text(manifest)
    print("wrote %s" % out)
    print(json.dumps({k: report[k]["points"] for k in ("mapcloud", "mapground")}))
    print(json.dumps(report["navigability"]))


if __name__ == "__main__":
    main()
