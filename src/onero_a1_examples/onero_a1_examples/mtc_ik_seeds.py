"""Remember accepted arm configurations as IK guesses, never as executable plans."""
import json
import math
import os
from pathlib import Path
import tempfile


def _path(node):
    root = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
    return root/'tasks'/f'mtc_ik_seeds_{node.group}.json'


def _valid(seed, count):
    return (isinstance(seed, list) and len(seed) == count
            and all(type(value) in (int, float) and math.isfinite(value) for value in seed))


def load_seeds(node):
    try:
        data = json.loads(_path(node).read_text())
        if (not isinstance(data, dict) or data.get('version') != 1
                or data.get('joints') != list(node.joints) or not isinstance(data.get('seeds'), list)):
            return []
        return [[float(value) for value in seed] for seed in data['seeds']
                if _valid(seed, len(node.joints))][:16]
    except (OSError, ValueError):
        return []


def remember_solution(node, message):
    names, seeds = list(node.joints), []
    for sub in message.sub_trajectory:
        path = sub.trajectory.joint_trajectory
        if set(path.joint_names) != set(names) or not path.points:
            continue
        indices = [path.joint_names.index(name) for name in names]
        for point in (path.points[0], path.points[-1]):
            seeds.append([float(point.positions[index]) for index in indices])
    unique = []
    for seed in seeds+load_seeds(node):
        if (_valid(seed, len(names)) and not any(
                max(abs(a-b) for a, b in zip(seed, previous)) < .001 for previous in unique)):
            unique.append(seed)
        if len(unique) == 16:
            break
    path, temporary = _path(node), None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(dict(version=1, joints=names, seeds=unique), stream, allow_nan=False)
        temporary.replace(path)
    except OSError as exc:
        print(f'MTC IK 构型记录未保存：{exc}；不影响本次方案。')
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
