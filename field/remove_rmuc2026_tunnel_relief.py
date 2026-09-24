#!/usr/bin/env python3
"""Remove the ten detached raised markings at the two RMUC2026 tunnel entries.

The markings are separate, watertight STL solids above an intact floor.  This
patch copies every other 50-byte binary-STL triangle record *unchanged*.  It is
deliberately tied to the audited STEP conversion hashes; a different export
must be inspected again instead of applying face numbers blindly.

Example (write to a staging directory, then validate before installation)::

    field/.step_convert_venv/bin/python field/remove_rmuc2026_tunnel_relief.py \
      --visual INPUT_VISUAL.stl --collision INPUT_COLLISION.stl --output-dir STAGING
"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import struct

import numpy as np
import trimesh


EXPECTED_SHA256 = {
    "visual": "af971f460ffc9327f35344abeb0840103382c5645fb884ca77fc4c559900af26",
    "collision": "0724d1375ead2e5739ffb12a841afaffb0e0240be466051203cf483d1fb6704c",
}

# One half-open interval per detached, watertight solid.  The same original
# face numbers occur in both STL files; the collision export retained these
# small components verbatim.  Bounds are the measured XYZ limits in metres.
MARKINGS = (
    ("south_outer_west", 25228, 25248, (-5.0865, -5.0363, 0.0983), (-5.0465, -4.9153, 0.1283)),
    ("south_outer_east", 25248, 25268, (-4.2465, -5.0363, 0.0983), (-4.2057, -4.9153, 0.1283)),
    ("south_left", 25560, 25632, (-4.5488, -4.9779, 0.1248), (-4.4470, -4.9223, 0.1390)),
    ("south_right", 25632, 25704, (-4.8460, -4.9779, 0.1248), (-4.7441, -4.9223, 0.1390)),
    ("south_centre", 25704, 25852, (-4.7648, -4.9932, 0.1078), (-4.5286, -4.8515, 0.1427)),
    ("north_outer_east", 26092, 26112, (5.0465, 4.9153, 0.0983), (5.0865, 5.0363, 0.1283)),
    ("north_outer_west", 26112, 26132, (4.2057, 4.9153, 0.0983), (4.2465, 5.0363, 0.1283)),
    ("north_left", 26424, 26496, (4.4470, 4.9223, 0.1248), (4.5488, 4.9779, 0.1390)),
    ("north_right", 26496, 26568, (4.7441, 4.9223, 0.1248), (4.8460, 4.9779, 0.1390)),
    ("north_centre", 26568, 26716, (4.5286, 4.8515, 0.1078), (4.7648, 4.9932, 0.1427)),
)
REMOVED_FACES = sum(stop - start for _, start, stop, _, _ in MARKINGS)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stl_count(path: Path) -> int:
    with path.open("rb") as stream:
        header = stream.read(84)
    if len(header) != 84:
        raise ValueError(f"not a binary STL: {path}")
    count = struct.unpack_from("<I", header, 80)[0]
    if path.stat().st_size != 84 + 50 * count:
        raise ValueError(f"binary STL size/count mismatch: {path}")
    return count


def records(path: Path, start: int, stop: int) -> bytes:
    with path.open("rb") as stream:
        stream.seek(84 + 50 * start)
        result = stream.read(50 * (stop - start))
    if len(result) != 50 * (stop - start):
        raise ValueError(f"short STL triangle read: {path}")
    return result


def check_marking(name: str, data: bytes, low: tuple, high: tuple) -> None:
    # STL record: 12-byte normal, 36-byte XYZ vertices, 2-byte attribute.
    raw = np.frombuffer(data, dtype=np.uint8).reshape((-1, 50))
    points = raw[:, 12:48].copy().view("<f4").reshape((-1, 3, 3))
    actual = np.stack((points.min(axis=(0, 1)), points.max(axis=(0, 1))))
    expected = np.asarray((low, high))
    if not np.allclose(actual, expected, atol=0.0002, rtol=0):
        raise ValueError(f"{name}: unexpected bounds {actual.tolist()}")
    part = trimesh.Trimesh(
        vertices=points.reshape((-1, 3)),
        faces=np.arange(len(points) * 3).reshape((-1, 3)),
        process=True,
    )
    if not part.is_watertight or not part.is_winding_consistent or part.volume <= 0:
        raise ValueError(f"{name}: target is not a separate closed solid")


def copy_records(source: Path, destination: Path, count: int) -> None:
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    intervals = [(start, stop) for _, start, stop, _, _ in MARKINGS]
    with source.open("rb") as src, destination.open("xb") as dst:
        dst.write(src.read(80))
        dst.write(struct.pack("<I", count - REMOVED_FACES))
        previous = 0
        for start, stop in intervals + [(count, count)]:
            src.seek(84 + 50 * previous)
            remaining = 50 * (start - previous)
            while remaining:
                block = src.read(min(4 * 1024 * 1024, remaining))
                if not block:
                    raise ValueError(f"short STL copy: {source}")
                dst.write(block)
                remaining -= len(block)
            previous = stop
    if stl_count(destination) != count - REMOVED_FACES:
        raise ValueError(f"invalid output STL: {destination}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--visual", required=True, type=Path)
    parser.add_argument("--collision", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    sources = {"visual": args.visual, "collision": args.collision}
    counts = {}
    for kind, path in sources.items():
        actual_hash = sha256(path)
        if actual_hash != EXPECTED_SHA256[kind]:
            raise ValueError(
                f"{kind} STL changed since the geometry audit: {actual_hash}; "
                "inspect it before removing face-numbered parts"
            )
        counts[kind] = stl_count(path)
        if counts[kind] <= MARKINGS[-1][2]:
            raise ValueError(f"{kind} STL is missing audited face ranges")
    for name, start, stop, low, high in MARKINGS:
        visual_records = records(args.visual, start, stop)
        collision_records = records(args.collision, start, stop)
        if visual_records != collision_records:
            raise ValueError(f"{name}: visual and collision geometry differ")
        check_marking(name, visual_records, low, high)
    for kind, path in sources.items():
        output = args.output_dir / f"rmuc2026_field_{kind}.stl"
        copy_records(path, output, counts[kind])
        print(f"{kind}: {counts[kind]} -> {stl_count(output)} faces, "
              f"removed {REMOVED_FACES}, sha256={sha256(output)}")


if __name__ == "__main__":
    main()
