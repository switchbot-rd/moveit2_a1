#!/usr/bin/env python3
"""Build low-face outer-profile prisms; visuals and kinematics are not modified.

Offline dependencies: numpy, shapely==2.1.2 and mapbox-earcut==2.0.0.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import mapbox_earcut
import numpy as np
import shapely
from shapely.geometry import Polygon, box
from shapely.geometry.polygon import orient


ROOT = Path(__file__).resolve().parent.parent
STL_RECORD = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


def read_triangles(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84+50*count:
        raise ValueError(f'需要二进制 STL：{path}')
    return np.frombuffer(data, dtype=STL_RECORD, offset=84, count=count)['vertices'].astype(float)


def write_triangles(path, triangles):
    values = np.zeros(len(triangles), dtype=STL_RECORD)
    values['vertices'] = triangles
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths <= 1e-18):
        raise ValueError('代理包含退化三角片')
    values['normal'] = normals/lengths[:, None]
    Path(path).write_bytes(b'Onero outer-profile collision proxy'.ljust(80, b' ')
                          + struct.pack('<I', len(values))+values.tobytes())


def _parts(shape):
    return list(shape.geoms) if shape.geom_type == 'MultiPolygon' else [shape]


def projection(triangles, axis):
    indices = ((axis+1) % 3, (axis+2) % 3)
    xy = triangles[:, :, indices]
    delta1, delta2 = xy[:, 1]-xy[:, 0], xy[:, 2]-xy[:, 0]
    areas = delta1[:, 0]*delta2[:, 1]-delta1[:, 1]*delta2[:, 0]
    return shapely.union_all(shapely.polygons(xy[abs(areas) > 1e-16]))


def covers_triangles(profile, triangles, axis):
    """Check every projected triangle, including edge-on/degenerate projections."""
    i, j = (axis+1) % 3, (axis+2) % 3
    xy = triangles[:, :, (i, j)]
    a, b = xy[:, 1]-xy[:, 0], xy[:, 2]-xy[:, 0]
    flat = abs(a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]) <= 1e-16
    guard = profile.buffer(1e-8)
    return (bool(np.all(shapely.covers(guard, shapely.polygons(xy[~flat]))))
            and bool(np.all(shapely.covers(guard, shapely.linestrings(xy[flat])))))


def outer_profile(source, tolerance):
    # Fill closed screw/bearing holes, retaining open notches. Uniform global
    # inflation would move the jaw contact plane, so clip to the original bounds.
    source = shapely.union_all([Polygon(part.exterior) for part in _parts(source)])
    limits = box(*source.bounds)
    for factor in (1.1, 1.5, 2.):
        candidate = source.simplify(tolerance, preserve_topology=True).buffer(
            tolerance*factor+1e-7, join_style='mitre').intersection(limits)
        if candidate.buffer(1e-10).covers(source):
            return candidate
    raise ValueError('轮廓代理没有完整覆盖原始投影')


def extrude_profile(profile, axis, lower, upper):
    triangles = []
    i, j = (axis+1) % 3, (axis+2) % 3
    for part in _parts(profile):
        part = orient(part, sign=1.)
        if part.interiors:
            raise ValueError('轮廓应已填充内部小孔')
        xy = np.asarray(part.exterior.coords[:-1], dtype=float)
        indices = mapbox_earcut.triangulate_float64(xy, np.array([len(xy)], dtype=np.uint32)).reshape(-1, 3)
        bottom, top = np.zeros((len(xy), 3)), np.zeros((len(xy), 3))
        bottom[:, i], bottom[:, j], bottom[:, axis] = xy[:, 0], xy[:, 1], lower
        top[:, i], top[:, j], top[:, axis] = xy[:, 0], xy[:, 1], upper
        for face in indices:
            triangles.extend((top[face], bottom[face[::-1]]))
        for a in range(len(xy)):
            b = (a+1) % len(xy)
            triangles.extend((np.array([bottom[a], bottom[b], top[b]]),
                              np.array([bottom[a], top[b], top[a]])))
    return np.asarray(triangles)


def build(source, destination):
    triangles = read_triangles(source)
    tolerance = .00005 if source.name in ('Link_L3.STL', 'Link_R3.STL') else .0002
    choices = []
    for axis in range(3):
        original = projection(triangles, axis)
        profile = outer_profile(original, tolerance)
        lower, upper = float(triangles[:, :, axis].min()), float(triangles[:, :, axis].max())
        choices.append((profile.area*(upper-lower), axis, original, profile, lower, upper))
    _, axis, original, profile, lower, upper = min(choices, key=lambda item: item[0])
    proxy = extrude_profile(profile, axis, lower, upper)
    write_triangles(destination, proxy)
    exported = read_triangles(destination)
    # Validate the exported geometry. Coverage is of complete projected
    # triangles, not merely a cloud of source vertices.
    actual = projection(exported, axis)
    if (not covers_triangles(actual, triangles, axis)
            or exported[:, :, axis].min() > lower+1e-8
            or exported[:, :, axis].max() < upper-1e-8):
        raise ValueError(f'{source.name} 导出后没有包住原三角片')
    return dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                path=str(destination), triangles=len(exported), source_triangles=len(triangles),
                axis=axis, profile_tolerance_m=tolerance, covers_source_triangles=True,
                coverage_numeric_tolerance_m=1e-8,
                bounds=[triangles.min(axis=(0, 1)).tolist(), triangles.max(axis=(0, 1)).tolist()])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    records = {}
    for path in sorted((ROOT/'src/onero_a1_description/meshes/grippers/woan_picker').glob('*.STL')):
        result = build(path, args.output/path.name)
        records['grippers/woan_picker/'+path.name] = result
        print(f'{path.name}: {result["source_triangles"]} → {result["triangles"]}', flush=True)
    args.manifest.write_text(json.dumps(records, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
