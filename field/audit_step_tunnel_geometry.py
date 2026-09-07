#!/usr/bin/env python3
"""Independently compare exact STEP intersections with both exported meshes.

This is a read-only asset audit. Reports and a locally retessellated reference
mesh are written only to the explicitly selected output directory.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRep import BRep_Builder
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound

from step_to_nav_maps import read_step, shape_bounds, tessellate_shape, clean_mesh
from conversion_metadata import sha256_file, write_json_object


def unique_heights(values):
    result = []
    for z in sorted(values):
        if not result or z - result[-1] > 0.0001:
            result.append(float(z))
    return result


def mesh_hits(path, origins):
    mesh = trimesh.load(path, process=False)
    directions = np.tile([0., 0., -1.], (len(origins), 1))
    points, indices, _ = mesh.ray.intersects_location(origins, directions)
    hits = [[] for _ in origins]
    for p, i in zip(points, indices):
        if -.08 <= p[2] <= .9:
            hits[i].append(p[2])
    return [unique_heights(v) for v in hits]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('step', type=Path)
    parser.add_argument('conversion', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((args.conversion / 'conversion_metadata.json').read_text())
    shift = np.asarray(metadata['origin_shift_m_in_source_coordinates'])
    shape, roots, transferred = read_step(args.step)
    builder = BRep_Builder()
    local = TopoDS_Compound()
    builder.MakeCompound(local)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    total = selected = 0
    while explorer.More():
        face = explorer.Current()
        bounds = np.asarray(shape_bounds(face)).reshape(2, 3) * .001 - shift
        lo, hi = bounds
        positive = hi[0] >= -1.7 and lo[0] <= 1.2 and hi[1] >= 5.2 and lo[1] <= 7.7
        negative = hi[0] >= -1.2 and lo[0] <= 1.7 and hi[1] >= -7.7 and lo[1] <= -5.2
        if (positive or negative) and hi[2] >= -.08 and lo[2] <= .9:
            builder.Add(local, face)
            selected += 1
        total += 1
        explorer.Next()
    print(f'Exact STEP faces: total={total}, selected={selected}', flush=True)
    intersector = IntCurvesFace_ShapeIntersector()
    intersector.Load(local, 1.e-5)
    # A dense cross-section grid includes the actual transverse tunnels and
    # the adjacent longitudinal ramp seams; both mirrored sides are audited.
    xy = np.asarray([(x, y) for x in np.arange(-1.5, 1.01, .1)
                     for y in np.arange(5.4, 7.61, .05)])
    xy = np.concatenate([xy, -xy])
    origins = np.column_stack([xy, np.full(len(xy), 1.2)])
    exact = []
    for i, origin in enumerate(origins):
        source = (origin + shift) * 1000.
        intersector.Perform(gp_Lin(gp_Pnt(*source), gp_Dir(0., 0., -1.)), 0., 1400.)
        heights = []
        for j in range(1, intersector.NbPnt() + 1):
            z = intersector.Pnt(j).Z() * .001 - shift[2]
            if -.08 <= z <= .9:
                heights.append(z)
        exact.append(unique_heights(heights))
        if i % 500 == 0:
            print(f'Exact intersections {i}/{len(origins)}', flush=True)
    meshes = args.conversion / 'gazebo/models/rmuc2026_field/meshes'
    comparisons = {}
    data = {'xy': xy.tolist(), 'exact_step_z': exact}
    for kind in ['visual', 'collision']:
        path = meshes / f'rmuc2026_field_{kind}.stl'
        hits = mesh_hits(path, origins)
        data[kind + '_z'] = hits
        distances = [min((abs(z - v) for v in got), default=1.2)
                     for ref, got in zip(exact, hits) for z in ref]
        comparisons[kind] = {
            'sha256': sha256_file(path),
            'exact_hits': len(distances),
            'within_1mm': sum(d <= .001 for d in distances),
            'within_5mm': sum(d <= .005 for d in distances),
            'max_nearest_surface_error_m': max(distances, default=0.),
        }
        print(kind, comparisons[kind], flush=True)
    vertices, faces, skipped = tessellate_shape(local, 1., 5.)
    mesh = clean_mesh(vertices * .001 - shift, faces)
    mesh.export(args.output / 'step_tunnel_reference_1mm.stl')
    write_json_object(args.output / 'step_surface_intersections.json', data)
    write_json_object(args.output / 'step_geometry_audit.json', {
        'source_step': str(args.step.resolve()),
        'source_step_sha256': sha256_file(args.step),
        'origin_shift_m': shift.tolist(),
        'source_faces': total, 'selected_faces': selected,
        'roots': roots, 'transferred_roots': transferred,
        'exact_ray_count': len(origins), 'comparisons': comparisons,
        'local_retessellation_skipped_faces': skipped,
        'local_retessellation_triangles': len(mesh.faces),
    })
    print('STEP geometry audit complete', flush=True)


if __name__ == '__main__':
    main()
