"""Collision envelope of the PB navigation robot, computed from its URDF.

The envelope is derived by walking the URDF joint tree from `base_footprint` and
transforming every collision primitive's sampled surface points into that frame,
so joint origins, collision origins and rotations are all taken into account.
Summing unrotated extents by hand gets the wheels wrong: their cylinders carry a
90 deg roll, so their width contributes to the vehicle's length/width axes only
after the rotation is applied.

Used by `test/test_robot_envelope.py` (profile must match the generated model)
and runnable as a CLI for the clearance records:

    python3 -m pb_vehicle_adapter.robot_envelope \
        meshnav_demo_ws/src/pb_vehicle_adapter/urdf/pb_navigation_robot.urdf
"""

import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np


CIRCLE_SAMPLES = 72
# Wheels legitimately touch the ground; everything else must stay on top of it.
WHEEL_MARKER = "wheel"


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _origin_matrix(element):
    matrix = np.eye(4)
    if element is None:
        return matrix
    xyz = element.get("xyz") or "0 0 0"
    rpy = element.get("rpy") or "0 0 0"
    roll, pitch, yaw = (float(value) for value in rpy.split())
    matrix[:3, :3] = _rpy_matrix(roll, pitch, yaw)
    matrix[:3, 3] = [float(value) for value in xyz.split()]
    return matrix


def _floats(text):
    return [float(value) for value in (text or "").split()]


def parse_collisions(urdf_path):
    """Return (root, [(link, name, kind, dims, origin_matrix), ...])."""
    root = ET.parse(str(urdf_path)).getroot()
    primitives = []
    for link in root.findall("link"):
        link_name = link.get("name")
        for index, collision in enumerate(link.findall("collision")):
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) == 0:
                continue
            shape = geometry[0]
            if shape.tag == "box":
                dims = _floats(shape.get("size"))
                kind = "box"
            elif shape.tag == "cylinder":
                dims = [float(shape.get("radius")), float(shape.get("length"))]
                kind = "cylinder"
            elif shape.tag == "sphere":
                dims = [float(shape.get("radius"))]
                kind = "sphere"
            else:
                # No mesh collisions in this model; skip loudly instead of
                # silently producing an envelope that is too small.
                raise ValueError("unsupported collision geometry '%s' on link '%s'"
                                 % (shape.tag, link_name))
            name = collision.get("name") or "collision_%d" % index
            primitives.append((link_name, name, kind, dims, _origin_matrix(collision.find("origin"))))
    return root, primitives


def link_poses(root):
    """Map every link to its pose in base_footprint (all joints at zero)."""
    poses = {"base_footprint": np.eye(4)}
    joints = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        joints.append((parent.get("link"), child.get("link"), _origin_matrix(joint.find("origin"))))
    changed = True
    while changed:
        changed = False
        for parent, child, origin in joints:
            if parent in poses and child not in poses:
                poses[child] = poses[parent] @ origin
                changed = True
    return poses


def primitive_points(kind, dims):
    """Sample the primitive surface in its own frame (cylinder axis = Z)."""
    if kind == "box":
        sx, sy, sz = dims
        signs = np.array([[x, y, z] for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])
        return signs * np.array([sx, sy, sz])
    if kind == "cylinder":
        radius, length = dims
        angles = np.linspace(0.0, 2.0 * math.pi, CIRCLE_SAMPLES, endpoint=False)
        ring = np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1)
        top = np.column_stack([ring, np.full(len(ring), length / 2.0)])
        bottom = np.column_stack([ring, np.full(len(ring), -length / 2.0)])
        return np.vstack([top, bottom, [[0.0, 0.0, length / 2.0], [0.0, 0.0, -length / 2.0]]])
    if kind == "sphere":
        (radius,) = dims
        angles = np.linspace(0.0, 2.0 * math.pi, CIRCLE_SAMPLES, endpoint=False)
        ring = np.stack([radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles)], axis=1)
        return np.vstack([ring, [[0, 0, radius], [0, 0, -radius]]])
    raise ValueError("unsupported primitive '%s'" % kind)


def envelope(urdf_path):
    """Return a dict with the base_footprint AABB and each primitive's own AABB."""
    root, primitives = parse_collisions(urdf_path)
    poses = link_poses(root)
    rows = []
    overall_min = np.full(3, np.inf)
    overall_max = np.full(3, -np.inf)
    for link, name, kind, dims, origin in primitives:
        if link not in poses:
            raise ValueError("collision on link '%s' is not connected to base_footprint" % link)
        points = primitive_points(kind, dims)
        transform = poses[link] @ origin
        world = points @ transform[:3, :3].T + transform[:3, 3]
        low, high = world.min(axis=0), world.max(axis=0)
        overall_min = np.minimum(overall_min, low)
        overall_max = np.maximum(overall_max, high)
        rows.append({
            "link": link,
            "collision": name,
            "kind": kind,
            "dims": [round(float(value), 6) for value in dims],
            "min": [round(float(value), 6) for value in low],
            "max": [round(float(value), 6) for value in high],
            "supportable": WHEEL_MARKER in link,
        })
    return {
        "urdf": str(urdf_path),
        "min": [round(float(value), 6) for value in overall_min],
        "max": [round(float(value), 6) for value in overall_max],
        "size": [round(float(value), 6) for value in (overall_max - overall_min)],
        "primitives": rows,
    }


def footprint_radii(report):
    """Radii of the two discs that can describe the horizontal footprint.

    Both MeshNav (`mesh_map.<layer>.inscribed_radius`, the disc that is made
    lethal around every lethal vertex) and JIE (`robot_radius_xy`, an upright
    cylinder) model the robot as a *disc*, while the vehicle is a rectangle.
    The two meaningful radii of that rectangle are:

    * ``inscribed`` - the largest circle that fits inside the footprint, i.e. the
      half width.  For a straight corridor the disc of this radius reproduces the
      rectangle exactly, and any *smaller* disc would let the planner accept
      corridors that the body cannot fit through;
    * ``circumscribed`` - the circle around the whole footprint (farthest
      corner).  Safe in every direction, but too large for the RMUC2026 tunnels
      (0.85 m wide against a 0.4332 m body).

    The values are derived here so that the profile cannot drift away from the
    model it is supposed to describe.
    """
    size = np.asarray(report["size"], dtype=float)
    return {
        "inscribed": float(np.min(size[:2]) / 2.0),
        "circumscribed": float(np.hypot(size[0] / 2.0, size[1] / 2.0)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?",
                        default="/home/rainple/nav_test/meshnav_demo_ws/src/pb_vehicle_adapter/"
                                "urdf/pb_navigation_robot.urdf")
    parser.add_argument("--json", action="store_true", help="print the full per-primitive report")
    arguments = parser.parse_args()

    report = envelope(arguments.urdf)
    if arguments.json:
        import json
        print(json.dumps(report, indent=2))
        return 0
    print("envelope in base_footprint (m):")
    print("  min  %s" % report["min"])
    print("  max  %s" % report["max"])
    print("  size %s" % report["size"])
    radii = footprint_radii(report)
    print("footprint discs (m): inscribed %.6f circumscribed %.6f"
          % (radii["inscribed"], radii["circumscribed"]))
    print("per primitive (highest point first):")
    for row in sorted(report["primitives"], key=lambda item: -item["max"][2]):
        print("  %-24s %-10s z=[%7.4f, %7.4f] x=[%7.4f, %7.4f] y=[%7.4f, %7.4f]"
              % (row["link"] + ":" + row["collision"], row["kind"],
                 row["min"][2], row["max"][2], row["min"][0], row["max"][0],
                 row["min"][1], row["max"][1]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
