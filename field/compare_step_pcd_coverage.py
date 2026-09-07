#!/usr/bin/env python3
"""Check occupied-cell coverage at independently computed exact STEP hits."""
import argparse
import json
from pathlib import Path

import numpy as np

from conversion_metadata import sha256_file, write_json_object
from verify_jie_tunnel_pcd import read_binary_xyz_pcd


def coverage(pcd, pitch, xyz):
    points = read_binary_xyz_pcd(pcd).astype(np.float64)
    keys = {tuple(k) for k in np.floor(points / pitch).astype(np.int32)}
    covered = np.array([tuple(k) in keys for k in np.floor(xyz / pitch).astype(np.int32)])
    return {
        'pcd': str(pcd.resolve()), 'sha256': sha256_file(pcd),
        'points': len(points), 'pitch': pitch, 'exact_surface_hits': len(xyz),
        'covered_hits': int(covered.sum()), 'coverage_fraction': float(covered.mean()),
        'uncovered_xyz': xyz[~covered].tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('intersections', type=Path)
    parser.add_argument('old_pcd', type=Path)
    parser.add_argument('new_pcd', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    data = json.loads(args.intersections.read_text())
    xyz = np.asarray([[x, y, z] for (x, y), heights in
                      zip(data['xy'], data['exact_step_z']) for z in heights])
    report = {'old': coverage(args.old_pcd, .05, xyz),
              'new': coverage(args.new_pcd, .04, xyz),
              'scope': 'two symmetric tunnel/ramp regions; exact STEP intersections',
              'intersections_sha256': sha256_file(args.intersections)}
    write_json_object(args.output, report)
    for name in ['old', 'new']:
        r = report[name]
        print(name, r['covered_hits'], '/', r['exact_surface_hits'], r['coverage_fraction'])
    return 0 if report['new']['coverage_fraction'] == 1. else 1


if __name__ == '__main__':
    raise SystemExit(main())
