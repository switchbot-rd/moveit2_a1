#!/usr/bin/env python3
"""生成双臂碰撞代理；原 CAD 和 URDF 不会被修改。

运行：python3 tools/build_arm_collision_proxies.py
结果：/tmp/onero_arm_proxies.json，凸网格位于 /tmp/onero_arm_proxy_meshes。
需要 numpy、scipy。尺寸单位为米，所有坐标使用原 STL 的局部坐标系。
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection


ROOT = Path(__file__).resolve().parents[1]
STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                      ('attribute', '<u2')])
# 仅补偿 STL 的 float32 舍入，0.00002 毫米；不是碰撞间隙或外轮廓放大系数。
ROUNDING_GUARD = 2e-8


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count:
        raise ValueError(f'不是受支持的二进制 STL：{path}')
    return np.frombuffer(data, dtype=STL_DTYPE, offset=84, count=count)[
        'vertices'].astype(np.float64)


def write_stl(path, vertices, faces):
    triangles = vertices[faces].astype(np.float32)
    precise = triangles.astype(np.float64)
    normals = np.cross(precise[:, 1] - precise[:, 0], precise[:, 2] - precise[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    nonzero = lengths > 0.
    triangles = triangles[nonzero]
    normals = normals[nonzero] / lengths[nonzero, None]
    records = np.zeros(len(triangles), dtype=STL_DTYPE)
    records['normal'] = normals
    records['vertices'] = triangles
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'Onero A1 conservative convex collision proxy'.ljust(80, b'\0')
                     + struct.pack('<I', len(triangles)) + records.tobytes())
    return len(triangles)


def triangle_groups(triangles, side, link):
    """按实体壳体和关节开口分区；每个原三角片始终保留完整。"""
    centers = triangles.mean(axis=1)
    if link == 'base_link':
        labels = np.digitize(centers[:, 2], [.0045, .062])
        axes = [2, 2, None]
    elif link == 'Link7':
        labels = (triangles[:, :, 1].min(axis=1) < -.02001).astype(int)
        axes = [2, None]
    elif link in ('Link1', 'Link3', 'Link5'):
        cuts = {'Link1': [.075, .110], 'Link3': [.060, .109],
                'Link5': [.070, .108]}[link]
        labels = np.digitize(centers[:, 2], cuts)
        axes = [2, 2, 1]
    else:
        # 将 U 形开口两侧及背部展开成七段，避免整体凸包封闭开口。
        sign = 1.
        if (link == 'Link4' and side == 'A1_l') or (link == 'Link6' and side == 'A1_r'):
            sign = -1.
        vertex_angles = np.degrees(np.arctan2(triangles[:, :, 2],
                                             -sign * triangles[:, :, 1]))
        # 平面上的长三角片可从背部一直伸到叉口端部；按最靠近端部的顶点
        # 归组，避免其质心把叉口端面与后方竖壁合并成填入开口的大凸体。
        angle = vertex_angles[np.arange(len(triangles)),
                              np.argmax(np.abs(vertex_angles), axis=1)]
        labels = np.digitize(angle, [-75., -45., -15., 15., 45., 75.])
        axes = [2, None, None, 1, None, None, 2]
    return [(index, triangles[labels == index], axes[index])
            for index in sorted(set(labels.tolist()))]


def primitive_fit(points, axis, hull_volume, cylinder_ratio=1.12):
    """只在外包图元与该凸分区体积接近时采用原生图元。"""
    low, high = points.min(axis=0), points.max(axis=0)
    size = high - low
    center = (low + high) / 2.
    if np.prod(size) <= 1.06 * hull_volume:
        return {'type': 'box', 'xyz': center.tolist(), 'rpy': [0., 0., 0.],
                'size': np.nextafter(size, np.inf).tolist()}
    if axis is None:
        return None
    radial_axes = [index for index in range(3) if index != axis]
    radius = np.linalg.norm(points[:, radial_axes] - center[radial_axes], axis=1).max()
    volume = math.pi * radius * radius * size[axis]
    if volume > cylinder_ratio * hull_volume:
        return None
    rotations = {0: [0., math.pi / 2., 0.], 1: [math.pi / 2., 0., 0.],
                 2: [0., 0., 0.]}
    return {'type': 'cylinder', 'xyz': center.tolist(), 'rpy': rotations[axis],
            'radius': float(np.nextafter(radius, np.inf)),
            'length': float(np.nextafter(size[axis], np.inf))}


def primitive_contains(triangles, primitive):
    delta = triangles - np.asarray(primitive['xyz'])
    if primitive['type'] == 'box':
        return np.all(np.abs(delta) <= np.asarray(primitive['size']) / 2. + 1e-15,
                      axis=(1, 2))
    rotation = primitive['rpy']
    axis = 0 if rotation[1] else 1 if rotation[0] else 2
    radial_axes = [index for index in range(3) if index != axis]
    radial = np.linalg.norm(delta[:, :, radial_axes], axis=2)
    return np.all((radial <= primitive['radius'] + 1e-15)
                  & (np.abs(delta[:, :, axis]) <= primitive['length'] / 2. + 1e-15),
                  axis=1)


def cylindrical_shells(triangles, link):
    """下端竖直电机壳与上端横向轴承壳分别使用圆柱。"""
    if link not in ('Link1', 'Link3', 'Link5'):
        return [], triangles
    lower_top = {'Link1': .067, 'Link3': .054, 'Link5': .058}[link]
    primitives = []
    remaining = triangles
    for axis, selection in ((2, triangles[:, :, 2].max(axis=1) <= lower_top),
                            (1, triangles[:, :, 2].min(axis=1) >= .110)):
        points = triangles[selection].reshape(-1, 3)
        low, high = points.min(axis=0), points.max(axis=0)
        center = (low + high) / 2.
        radial_axes = [index for index in range(3) if index != axis]
        if axis == 1:
            center[2] = high[2] - (high[0] - low[0]) / 2.
        radius = np.linalg.norm(points[:, radial_axes] - center[radial_axes], axis=1).max()
        primitive = {'type': 'cylinder', 'xyz': center.tolist(),
                     'rpy': [math.pi / 2., 0., 0.] if axis == 1 else [0., 0., 0.],
                     'radius': float(np.nextafter(radius, np.inf)),
                     'length': float(np.nextafter(high[axis] - low[axis], np.inf))}
        contains = primitive_contains(remaining, primitive)
        primitive['source_triangles'] = int(contains.sum())
        primitives.append(primitive)
        remaining = remaining[~contains]
    return primitives, remaining


def outer_polyhedron(points, max_planes, error):
    """保留支撑平面构成外包凸体，逐步增加误差最大的方向。"""
    exact = ConvexHull(points)
    directions = np.vstack((np.eye(3), -np.eye(3)))
    interior = points.mean(axis=0)
    equations = exact.equations
    max_excess = math.inf
    while True:
        offsets = (points @ directions.T).max(axis=0) + ROUNDING_GUARD
        planes = np.column_stack((directions, -offsets))
        poly = HalfspaceIntersection(planes, interior)
        vertices = poly.intersections
        violations = vertices @ equations[:, :3].T + equations[:, 3]
        vertex_excess = violations.max(axis=1)
        max_excess = float(vertex_excess.max())
        if max_excess <= error or len(directions) >= max_planes:
            break
        indices = np.argmax(violations, axis=1)
        added = []
        for vertex in np.argsort(vertex_excess)[::-1]:
            normal = equations[indices[vertex], :3]
            if np.max(directions @ normal) > 1. - 1e-10:
                continue
            if any(np.dot(normal, previous) > 1. - 1e-10 for previous in added):
                continue
            added.append(normal)
            if len(added) >= 1:
                break
        if not added:
            break
        directions = np.vstack((directions, added))
    # 验证实际写入 STL 的 float32 顶点，而不是只验证未导出的双精度候选。
    vertices = vertices.astype(np.float32).astype(np.float64)
    exported = ConvexHull(vertices)
    excess = (points @ exported.equations[:, :3].T + exported.equations[:, 3]).max()
    if excess > 1e-10:
        raise ValueError(f'STL 舍入导致覆盖不足：{excess:g} m')
    faces = exported.simplices.copy()
    normals = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                       vertices[faces[:, 2]] - vertices[faces[:, 0]])
    flip = np.einsum('ij,ij->i', normals, exported.equations[:, :3]) < 0.
    faces[flip] = faces[flip][:, [0, 2, 1]]
    return vertices, faces, exported.equations, max_excess


def build(meshes_dir, output, mesh_output, max_planes, error):
    entries = {}
    for side in ('A1_l', 'A1_r'):
        for link in ['base_link'] + [f'Link{index}' for index in range(1, 8)]:
            relative = f'{side}/{link}.STL'
            source = meshes_dir / relative
            triangles = read_stl(source)
            primitives, remaining = cylindrical_shells(triangles, link)
            convex_meshes = []
            covered = len(triangles) - len(remaining)
            for number, group, axis in triangle_groups(remaining, side, link):
                points = np.unique(group.reshape(-1, 3), axis=0)
                exact = ConvexHull(points)
                primitive = primitive_fit(points, axis, exact.volume,
                                          1.25 if link == 'base_link' else 1.12)
                if primitive is not None:
                    if not primitive_contains(group, primitive).all():
                        raise ValueError(f'图元覆盖检查失败：{relative}/{number}')
                    primitive['source_triangles'] = len(group)
                    primitives.append(primitive)
                else:
                    complex_transition = ((link in ('Link1', 'Link3', 'Link5') and number < 2)
                                          or (link == 'Link4' and number in (2, 4)))
                    plane_limit = max_planes + (10 if complex_transition else 0)
                    vertices, faces, planes, excess = outer_polyhedron(points, plane_limit, error)
                    path = mesh_output / side / f'{link}_{number}.STL'
                    face_count = write_stl(path, vertices, faces)
                    # 同一分区所有顶点都在同一个凸体内，因此每个完整三角片也在其中。
                    reread = read_stl(path)
                    exported = ConvexHull(np.unique(reread.reshape(-1, 3), axis=0))
                    if (points @ exported.equations[:, :3].T
                            + exported.equations[:, 3]).max() > 1e-10:
                        raise ValueError(f'导出后的覆盖检查失败：{relative}/{number}')
                    convex_meshes.append({'path': str(path), 'triangles': face_count,
                                          'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                          'source_triangles': len(group),
                                          'planes': planes.tolist(),
                                          'outer_support_error_m': excess})
                covered += len(group)
            if covered != len(triangles):
                raise ValueError(f'三角片分区计数不一致：{relative}')
            entries[relative] = {'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                                 'source_triangles': len(triangles),
                                 'verified_source_triangles': covered,
                                 'primitives': primitives, 'convex_meshes': convex_meshes}
            faces = sum(mesh['triangles'] for mesh in convex_meshes)
            print(f'{relative}: {len(primitives)} 图元 + {len(convex_meshes)} 凸体 / '
                  f'{faces} 网格面；覆盖 {covered}/{len(triangles)} 源三角片', flush=True)
    output.write_text(json.dumps(entries, indent=2) + '\n')
    total = sum(mesh['triangles'] for entry in entries.values() for mesh in entry['convex_meshes'])
    print(f'臂部总网格面数：{total}；结果：{output}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/onero_arm_proxies.json'))
    parser.add_argument('--mesh-output', type=Path, default=Path('/tmp/onero_arm_proxy_meshes'))
    parser.add_argument('--max-planes', type=int, default=22,
                        help='普通分区支撑平面数，默认 22；复杂过渡区增加 10 个')
    parser.add_argument('--support-error-mm', type=float, default=.8,
                        help='相对该分区精确凸包的支撑面距离目标，默认 0.8 毫米')
    args = parser.parse_args()
    build(ROOT / 'src/onero_a1_description/meshes', args.output, args.mesh_output,
          args.max_planes, args.support_error_mm / 1000.)


if __name__ == '__main__':
    main()
