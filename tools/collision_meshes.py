"""Select verified collision meshes or primitives, retaining the CAD fallback."""
import base64
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import sys
import xml.etree.ElementTree as ET


DESCRIPTION_DIR = Path(__file__).resolve().parents[1] / 'src/onero_a1_description'
MESH_URI_PREFIX = 'package://onero_a1_description/meshes/'
PROXY_NAME_PREFIX = '__onero_collision_proxy_v1_'


def primitives_sha256(primitives):
    """Hash the mesh-local primitive definitions stored in the manifest."""
    data = json.dumps(primitives, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')
    return hashlib.sha256(data).hexdigest()


def _collision_mesh_uri(meshes_dir, relative, expected_sha256):
    if (not isinstance(relative, str) or not relative.startswith('collision/')
            or '..' in PurePosixPath(relative).parts):
        raise ValueError('碰撞网格必须位于 meshes/collision 下')
    path = meshes_dir / relative
    if not path.resolve().is_relative_to(meshes_dir.resolve() / 'collision'):
        raise ValueError('碰撞网格必须位于 meshes/collision 下')
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
        raise ValueError(f'{relative} 的 SHA256 已变化')
    return MESH_URI_PREFIX + relative


def _vector(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError('坐标或尺寸必须包含三个数值')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError('图元参数必须是有限数值')
    return result


def _rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def _format(values):
    return ' '.join(format(0.0 if abs(value) < 1e-14 else value, '.12g') for value in values)


def _restore_proxies(root):
    # The original collision is encoded in the name because ordinary XML
    # parsers discard comments; URDF accepts names on all collision shapes.
    for parent in list(root.iter()):
        children, restored = [], {}
        for child in parent:
            name = child.get('name', '')
            if child.tag != 'collision' or not name.startswith(PROXY_NAME_PREFIX):
                children.append(child)
                continue
            group, _, payload = name[len(PROXY_NAME_PREFIX):].split('_', 2)
            if group not in restored:
                original = ET.fromstring(base64.urlsafe_b64decode(payload + '=' * (-len(payload) % 4)))
                restored[group] = original
                children.append(original)
            restored[group].tail = child.tail
        parent[:] = children


def _primitive_collisions(collision, primitives, group):
    mesh = collision.find('geometry/mesh')
    if _vector(mesh.get('scale', '1 1 1').split()) != (1.0, 1.0, 1.0):
        raise ValueError('非单位 mesh scale 不支持图元代理')
    origin = collision.find('origin')
    xyz = _vector(origin.get('xyz', '0 0 0').split()) if origin is not None else (0., 0., 0.)
    rotation = _rotation(_vector(origin.get('rpy', '0 0 0').split())
                         if origin is not None else (0., 0., 0.))
    original = deepcopy(collision)
    original.tail = None
    payload = base64.urlsafe_b64encode(ET.tostring(original)).decode('ascii').rstrip('=')
    result = []
    for index, primitive in enumerate(primitives):
        local_xyz = _vector(primitive.get('xyz', [0, 0, 0]))
        local_rotation = _rotation(_vector(primitive.get('rpy', [0, 0, 0])))
        composed_xyz = [xyz[i] + sum(rotation[i][j] * local_xyz[j] for j in range(3)) for i in range(3)]
        composed_rotation = [[sum(rotation[i][k] * local_rotation[k][j] for k in range(3))
                              for j in range(3)] for i in range(3)]
        m = composed_rotation
        pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
        if math.hypot(m[0][0], m[1][0]) > 1e-10:
            roll, yaw = math.atan2(m[2][1], m[2][2]), math.atan2(m[1][0], m[0][0])
        else:
            roll, yaw = math.atan2(-m[1][2], m[1][1]), 0.0
        item = deepcopy(collision)
        item.set('name', f'{PROXY_NAME_PREFIX}{group}_{index}_{payload}')
        item_origin = item.find('origin')
        if item_origin is None:
            item_origin = ET.Element('origin')
            item.insert(0, item_origin)
        item_origin.set('xyz', _format(composed_xyz))
        item_origin.set('rpy', _format((roll, pitch, yaw)))
        geometry = item.find('geometry')
        geometry.clear()
        kind = primitive['type']
        if kind == 'box':
            size = _vector(primitive['size'])
            if min(size) <= 0:
                raise ValueError('box size 必须为正数')
            ET.SubElement(geometry, 'box', size=_format(size))
        elif kind in ('cylinder', 'sphere'):
            dimensions = {key: float(primitive[key]) for key in
                          (('radius', 'length') if kind == 'cylinder' else ('radius',))}
            if not all(math.isfinite(value) and value > 0 for value in dimensions.values()):
                raise ValueError('图元半径和长度必须为有限正数')
            ET.SubElement(geometry, kind, {key: _format([value]) for key, value in dimensions.items()})
        elif kind == 'mesh':
            ET.SubElement(geometry, 'mesh', filename=MESH_URI_PREFIX + primitive['collision'])
        else:
            raise ValueError(f'不支持的碰撞图元：{kind}')
        result.append(item)
    return result


def apply_collision_meshes(root, description_dir=DESCRIPTION_DIR, *, source_paths=None):
    """Apply verified collision geometry; optional SDK source_paths are checked too."""
    meshes_dir = Path(description_dir) / 'meshes'
    manifest_path = meshes_dir / 'collision/manifest.json'
    records = {}
    try:
        manifest = json.loads(manifest_path.read_text())
        if manifest.get('format_version') != 1 or not isinstance(manifest.get('meshes'), dict):
            raise ValueError('不支持的清单格式')
        records = manifest['meshes']
    except FileNotFoundError:
        pass
    except (OSError, ValueError, AttributeError) as error:
        print(f'警告：无法读取碰撞网格清单，使用原始 CAD：{error}', file=sys.stderr)

    sources = {
        entry['collision']: source for source, entry in records.items()
        if isinstance(entry, dict) and isinstance(entry.get('collision'), str)
    }
    _restore_proxies(root)
    selected = {}
    for parent in list(root.iter()):
        children = []
        for group, collision in enumerate(parent):
            children.append(collision)
            if collision.tag != 'collision':
                continue
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            filename = mesh.get('filename', '')
            if not filename.startswith(MESH_URI_PREFIX):
                continue
            relative = filename[len(MESH_URI_PREFIX):]
            source = sources.get(relative, relative.removeprefix('collision/'))
            original_uri = MESH_URI_PREFIX + source
            mesh.set('filename', original_uri)
            if source not in selected:
                selected[source] = None
                entry = records.get(source)
                reason = None
                if entry is None:
                    if relative.startswith('collision/'):
                        reason = '清单中没有对应记录'
                else:
                    try:
                        if (PurePosixPath(source).is_absolute()
                                or '..' in PurePosixPath(source).parts):
                            raise ValueError('网格路径无效')
                        paths = [meshes_dir / source]
                        if source_paths and source in source_paths:
                            paths.append(Path(source_paths[source]))
                        for path in paths:
                            if hashlib.sha256(path.read_bytes()).hexdigest() != entry['source_sha256']:
                                raise ValueError(f'{path.name} 的 SHA256 已变化')
                        if 'primitives' in entry:
                            primitives = entry['primitives']
                            if not isinstance(primitives, list) or not primitives:
                                raise ValueError('primitives 必须是非空数组')
                            if primitives_sha256(primitives) != entry['primitives_sha256']:
                                raise ValueError('图元定义的 SHA256 已变化')
                            for primitive in primitives:
                                if isinstance(primitive, dict) and primitive.get('type') == 'mesh':
                                    _collision_mesh_uri(meshes_dir, primitive['collision'],
                                                        primitive['collision_sha256'])
                            selected[source] = primitives
                        else:
                            selected[source] = _collision_mesh_uri(
                                meshes_dir, entry['collision'], entry['collision_sha256'])
                    except (OSError, ValueError, TypeError, KeyError) as error:
                        reason = str(error)
                if reason:
                    print(f'警告：{source} 使用原始 CAD 碰撞网格：{reason}', file=sys.stderr)
            selection = selected[source]
            if isinstance(selection, list):
                try:
                    children[-1:] = _primitive_collisions(collision, selection, group)
                except (ValueError, TypeError, KeyError, AttributeError) as error:
                    print(f'警告：{source} 使用原始 CAD 碰撞网格：{error}', file=sys.stderr)
            elif selection:
                mesh.set('filename', selection)
        parent[:] = children
    return root
