"""Shared dual-arm task state, reflection and synchronized trajectories."""
from copy import deepcopy

import numpy as np
from onero_a1_common.constants import JOINTS, LEFT_JOINTS
from onero_a1_common.moveit_client import MoveItClient, MoveItError
from onero_a1_common.trajectory_tools import (
    _curve, _scaled_limits, _synchronized_trajectory, _curve_sample_times, _curve_position_extrema,
)

from .bottle_scene import matrix_pose, pose_matrix


def arm_clients(node):
    """Use one ROS node per arm, retaining the parent for combined execution."""
    if not node.dual or node.arm_side != 'both':
        raise MoveItError('本示例需要 dual_moveit.launch.py 双臂服务')
    node.task_nodes = []
    result = {}
    for side in ('left', 'right'):
        args = deepcopy(node.args)
        args.arm = side
        child = MoveItClient(args)
        node.task_nodes.append(child)
        child.select_arm(side)
        child.execution_observer = getattr(node, 'execution_observer', None)
        result[side] = child
    return result


def close_arm_clients(node):
    errors = []
    for child in getattr(node, 'task_nodes', []):
        try:
            if not child.cleanup():
                errors.append(child.arm_side+' 请求取消失败')
        except Exception as exc:
            errors.append(str(exc))
        finally:
            child.destroy_node()
    node.task_nodes = []
    if errors:
        raise MoveItError('；'.join(errors))


class Mirror:
    """Reflect world Y; derive the flange basis parity from the actual model."""
    def __init__(self, node):
        self.reflection = np.diag([1., -1., 1.])
        zero = dict.fromkeys(LEFT_JOINTS+JOINTS, 0.)
        right = pose_matrix(node.fk(zero, link='right_Link7'))
        left = pose_matrix(node.fk(zero, link='left_Link7'))
        self.tool = right[:3, :3].T @ self.reflection @ left[:3, :3]
        right_base = pose_matrix(node.fk(zero, link='right_base_link'))
        left_base = pose_matrix(node.fk(zero, link='left_base_link'))
        if not np.allclose(left_base[:3, 3], self.point(right_base[:3, 3]), atol=1e-6):
            raise MoveItError('左右臂安装位置不关于 world Y=0 对称，无法使用镜像任务')

    def point(self, value):
        return self.reflection @ value

    def pose(self, value):
        matrix = pose_matrix(value)
        matrix[:3, 3] = self.point(matrix[:3, 3])
        matrix[:3, :3] = self.reflection @ matrix[:3, :3] @ self.tool
        return matrix_pose(matrix)


def endpoint(initial, steps):
    result = dict(initial)
    for step in steps:
        path = step.trajectory.joint_trajectory
        result.update(zip(path.joint_names, path.points[-1].positions))
    return result


def synchronize(node, start, trajectories, *, check=None):
    """Stretch both stopped splines to a common duration; validate the combined path."""
    from onero_a1_common.continuous import geometry
    names = [name for trajectory in trajectories for name in trajectory.joint_trajectory.joint_names]
    if len(names) != len(set(names)):
        raise MoveItError('同步轨迹包含重复关节')
    limits = _scaled_limits(node, names, MoveItError)
    curves, offset = [], 0
    for trajectory in trajectories:
        joints = list(trajectory.joint_trajectory.joint_names)
        curves.append(_curve(trajectory, joints, limits[offset:offset+len(joints)], MoveItError))
        offset += len(joints)
    result, controller_curve = _synchronized_trajectory(curves, names, limits, MoveItError)
    context = geometry(node, refresh=True)
    node.planning_start_state({})
    bounds = [node._joint_position_bounds[name] for name in names]
    minimum, maximum = _curve_position_extrema(controller_curve)
    if any(low-1e-9 > qmin or qmax > high+1e-9
           for qmin, qmax, (low, high) in zip(minimum, maximum, bounds)):
        raise MoveItError('双臂同步轨迹超出关节限位')
    for tick in _curve_sample_times(controller_curve):
        positions = controller_curve(tick)
        state = {**start, **dict(zip(names, positions))}
        if check is not None:
            # Only an explicit False promises unchanged scene permissions.
            # Existing callbacks returning None retain a fresh scene per check.
            if check(state) is not False:
                context = geometry(node, refresh=True)
        valid = node.validity(state, group='both_arms', context=context)
        if not valid.valid:
            pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
            raise MoveItError(f'双臂同步轨迹存在碰撞或无效状态：{pairs}')
    node.planned_start_state = node.planning_start_state(start)
    return result
