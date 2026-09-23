#!/usr/bin/env python3
"""Emit an evidence manifest for a report directory or for the repository baseline
(doc/PB_SLOPE_NEXT_STEPS_20260921.md sections 3.1 and 9).

For each file it records the path relative to the repository root, the size, the
SHA256, and — for a run report — the experiment identity taken from the session
file / goal log when those are present.  Code and configuration versions are
recorded once per manifest.

    python3 tools/pb_sim/manifest.py log/<label>_report_<stamp> --out <dir>
    python3 tools/pb_sim/manifest.py --baseline --out log/pb_sim_baseline

Notes:
* `git diff` does not list untracked files, so the baseline manifest always lists
  tracked modifications, untracked files and deletions separately.
* Large bags are hashed in a streaming fashion; nothing is copied or deleted.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha256_file(path, chunk=1 << 20):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        while True:
            block = stream.read(chunk)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def git(*arguments):
    completed = subprocess.run(["git", *arguments], cwd=ROOT, capture_output=True, text=True)
    return completed.stdout.strip()


def code_version():
    return {
        "head": git("rev-parse", "HEAD"),
        "head_subject": git("log", "-1", "--format=%s"),
        "tracked_modifications": git("diff", "--name-status").splitlines(),
        "staged": git("diff", "--cached", "--name-status").splitlines(),
        "untracked": git("ls-files", "--others", "--exclude-standard").splitlines(),
        "deleted": [line for line in git("status", "--short").splitlines()
                    if line.startswith(" D")],
    }


def file_entries(directory):
    entries = []
    for path in sorted(Path(directory).rglob("*")):
        if not path.is_file():
            continue
        entries.append({
            "path": str(path.resolve().relative_to(ROOT)),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        })
    return entries


def run_identity(directory):
    """Experiment identity of a collector report directory, when available."""
    identity = {}
    session = Path(directory) / "session.json"
    if session.is_file():
        identity.update(json.loads(session.read_text()))
    goal_log = Path(directory) / "goal.log"
    if goal_log.is_file():
        lines = [line for line in goal_log.read_text(errors="replace").splitlines()
                 if "pb_nav_goal" in line or "cancel_result" in line]
        identity["goal_log_tail"] = lines[-6:]
    commands = Path(directory) / "commands.txt"
    if commands.is_file():
        identity["commands"] = commands.read_text(errors="replace").strip().splitlines()
    return identity


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("report_dir", nargs="?")
    parser.add_argument("--baseline", action="store_true",
                        help="manifest the tracked/untracked source and configuration "
                             "baseline instead of one report directory")
    parser.add_argument("--out", required=True, help="directory for the manifest files")
    parser.add_argument("--extra", nargs="*", default=[],
                        help="additional files (for example installed libraries) to hash")
    arguments = parser.parse_args()

    out = Path(arguments.out)
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"code_version": code_version()}

    if arguments.baseline:
        manifest["kind"] = "baseline"
        targets = [
            "meshnav_demo_ws/src/pb_gazebo_sim_support",
            "meshnav_demo_ws/src/pb_terminal_controller",
            "meshnav_demo_ws/src/pb_vehicle_adapter",
            "meshnav_demo_ws/src/mesh_navigation/mesh_controller",
            "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials/config",
            "tools/pb_sim",
            "doc",
        ]
        entries = []
        for target in targets:
            path = ROOT / target
            if path.is_dir():
                entries.extend(file_entries(path))
        manifest["files"] = entries
        for extra in arguments.extra:
            target = Path(extra)
            if target.is_file():
                manifest.setdefault("installed", []).append({
                    "path": str(target),
                    "bytes": target.stat().st_size,
                    "sha256": sha256_file(target),
                    "link_target": os.readlink(target) if target.is_symlink() else None,
                })
        # Deliberately hash the configuration the runs depend on.
        for name in ("meshnav_demo_ws/rmuc2026_pb_low_navigation.h5",
                     "meshnav_demo_ws/src/mesh_navigation_tutorials-v1/mesh_navigation_tutorials/maps/rmuc2026_field.ply"):
            path = ROOT / name
            if path.is_file():
                manifest.setdefault("runtime_assets", []).append({
                    "path": name, "bytes": path.stat().st_size, "sha256": sha256_file(path),
                })
    else:
        if not arguments.report_dir:
            parser.error("report_dir is required unless --baseline is given")
        directory = Path(arguments.report_dir).resolve()
        if not directory.is_dir():
            raise SystemExit("no such report directory: %s" % directory)
        manifest["kind"] = "report"
        manifest["report_dir"] = str(directory)
        manifest["experiment"] = run_identity(directory)
        manifest["files"] = file_entries(directory)

    text = json.dumps(manifest, indent=2, sort_keys=True)
    path = out / ("manifest_%s.json" % (manifest["kind"]))
    path.write_text(text + "\n")
    print("wrote %s (%d files)" % (path, len(manifest.get("files", []))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
