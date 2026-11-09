"""Remember selected grasp/place directions; all poses are generated again each run."""
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np
from moveit_msgs.msg import CollisionObject
from scipy.spatial.transform import Rotation

from .bottle_scene import pose_matrix


def _path(node, route):
    root = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
    return root/'tasks'/f'mtc_directions_{node.group}_{route[0]}_{route[1]}.json'


def load_directions(node, route):
    if route is None:
        return None
    try:
        data = json.loads(_path(node, route).read_text())
        if (not isinstance(data, dict) or data.get('version') != 1
                or data.get('joints') != list(node.joints) or data.get('route') != list(route)):
            return None
        directions = data.get('directions')
        if isinstance(directions, dict) and all(
                type(directions.get(key)) in (int, float) and math.isfinite(directions[key])
                for key in ('grasp_yaw', 'place_yaw', 'pick_yaw')):
            return {key: float(directions[key]) for key in ('grasp_yaw', 'place_yaw', 'pick_yaw')}
    except (OSError, ValueError, OverflowError):
        pass
    return None


def save_directions(node, route, directions):
    if route is None or directions is None:
        return
    path, temporary = _path(node, route), None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(dict(version=1, joints=list(node.joints), route=list(route),
                           directions=directions), stream, allow_nan=False)
        temporary.replace(path)
    except OSError as exc:
        print(f'MTC 抓放方向记录未保存：{exc}；不影响本次方案。')
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


def preferred_targets(targets, place_yaws, directions, *, pick_yaw=0.):
    if directions is not None:
        def matches(first, second):
            return abs(math.remainder(first-second, 2.*math.pi)) < 1e-6
        grasps = [item for item in targets if matches(item[0], directions['grasp_yaw'])]
        # Preserve the successful bottle turn relative to its pickup orientation.
        # The tolerance only matches hints to existing samples; it never changes
        # an IK target or a collision/placement tolerance.
        desired = directions['place_yaw'] + pick_yaw - directions['pick_yaw']
        places = [yaw for yaw in place_yaws if abs(math.remainder(yaw-desired, 2.*math.pi)) < .05]
        if grasps and places:
            return grasps, places
    return targets, place_yaws


def solution_directions(node, message, object_id, targets, place_yaws):
    """Match a selected plan's attachment and release poses to its sampled directions."""
    bottle = next((obj for obj in message.start_scene.world.collision_objects
                   if obj.id == object_id), None)
    if bottle is None or not targets or not place_yaws:
        return None
    joints = message.start_scene.robot_state.joint_state
    state = dict(zip(joints.name, joints.position))
    grasp = placed = None
    for sub in message.sub_trajectory:
        path = sub.trajectory.joint_trajectory
        if path.points:
            state.update(zip(path.joint_names, path.points[-1].positions))
        diff = sub.scene_diff.robot_state
        state.update(zip(diff.joint_state.name, diff.joint_state.position))
        for attached in diff.attached_collision_objects:
            if attached.object.id != object_id:
                continue
            if (grasp is None and attached.object.operation == CollisionObject.ADD
                    and attached.link_name == node.tip_link):
                grasp = pose_matrix(node.fk(state))
            elif grasp is not None and attached.object.operation == CollisionObject.REMOVE:
                placed = pose_matrix(node.fk(state)) @ np.linalg.inv(grasp) @ pose_matrix(bottle.pose)
        if placed is not None:
            break
    if grasp is None or placed is None:
        return None
    grasp_errors = [Rotation.from_matrix(pose_matrix(pose)[:3, :3].T @ grasp[:3, :3]).magnitude()
                    for _, pose in targets]
    place_errors = [Rotation.from_matrix(Rotation.from_euler('z', yaw).as_matrix().T
                                         @ placed[:3, :3]).magnitude() for yaw in place_yaws]
    first, second = int(np.argmin(grasp_errors)), int(np.argmin(place_errors))
    if max(grasp_errors[first], place_errors[second]) > .02:
        return None
    return dict(grasp_yaw=float(targets[first][0]), place_yaw=float(place_yaws[second]),
                pick_yaw=float(Rotation.from_matrix(pose_matrix(bottle.pose)[:3, :3]).as_euler('xyz')[2]))
