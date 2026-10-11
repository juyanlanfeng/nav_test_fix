#!/usr/bin/env python3
"""Stage the documented real packages and PCL patch, after a dry preflight."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path,
                        default=Path(__file__).resolve().parents[3])
    parser.add_argument("--apply", action="store_true",
                        help="write after all preflight checks pass; default only checks")
    args = parser.parse_args()
    repo = args.repo.resolve()
    templates = Path(__file__).resolve().parents[1]
    workspace = repo / "mesh_navigation_tutorials"
    source = workspace / "src"
    patch = templates / "patches/pcl_real_interfaces.patch"
    pcl = source / "pcl_obstacle_point_subtraction/src/pcl_obstacle_point_subtraction.cpp"
    package_names = ("meshnav_runtime", "meshnav_real_bringup")
    if not (repo / "SIM2REAL指南.md").is_file() or not pcl.is_file():
        parser.error("--repo must name the project root containing mesh_navigation_tutorials/src")
    for name in package_names:
        if not (templates / name / "package.xml").is_file():
            parser.error(f"missing reference package: {name}")
        if (source / name).exists() or (source / name).is_symlink():
            parser.error(f"{source / name} already exists; inspect it instead of overwriting")
    check = subprocess.run(["git", "apply", "--check", str(patch)], cwd=repo,
                           text=True, capture_output=True)
    if check.returncode:
        print(check.stderr, file=sys.stderr)
        parser.error("PCL patch does not match this checkout; reconcile source changes first")
    print(f"Repository: {repo}\nPCL patch: {patch}")
    for name in package_names:
        print(f"Copy: {templates / name} -> {source / name}")
    if not args.apply:
        print("Preflight passed. Run again with --apply to stage these changes.")
        return
    original = pcl.read_bytes()
    created = []
    try:
        for name in package_names:
            target = source / name
            created.append(target)
            shutil.copytree(templates / name, target,
                            ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache", "*.pyc"))
        subprocess.run(["git", "apply", str(patch)], cwd=repo, check=True)
    except Exception:
        pcl.write_bytes(original)
        for target in created:
            if target.exists():
                shutil.rmtree(target)
        raise
    print("Staged. Review git diff and the two new source directories before building.")


if __name__ == "__main__":
    main()
