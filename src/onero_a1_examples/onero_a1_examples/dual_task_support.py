"""Shared dual-arm task state, reflection and synchronized trajectories."""
from copy import deepcopy
import math

import numpy as np
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from onero_a1_common.constants import JOINTS, LEFT_JOINTS
from onero_a1_common.moveit_client import MoveItClient, MoveItError
from onero_a1_common.trajectory_tools import _curve, _duration, _scaled_limits

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
    names = [name for trajectory in trajectories for name in trajectory.joint_trajectory.joint_names]
    if len(names) != len(set(names)):
        raise MoveItError('同步轨迹包含重复关节')
    limits = _scaled_limits(node, names, MoveItError)
    curves, offset = [], 0
    for trajectory in trajectories:
        joints = list(trajectory.joint_trajectory.joint_names)
        curves.append(_curve(trajectory, joints, limits[offset:offset+len(joints)], MoveItError))
        offset += len(joints)
    duration = max(times[-1]*scale for times, _, scale in curves)
    scales = [duration/times[-1] for times, _, _ in curves]
    knots = {0., duration}
    for (times, _, _), scale in zip(curves, scales):
        knots.update(time*scale for time in times)
    # Bound motion between checks by 20 ms and 0.02 rad, including spline overshoot.
    speed = max(float(abs(curve.derivative().c).max())/scale
                for (_, curve, _), scale in zip(curves, scales))
    count = max(1, math.ceil(duration*max(50., speed/.02)))
    knots.update(np.linspace(0., duration, count+1).tolist())
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = names
    previous = -1.
    for tick in sorted(knots):
        if tick-previous < 1e-8:
            continue
        point = JointTrajectoryPoint(time_from_start=_duration(tick))
        for (_, curve, _), scale in zip(curves, scales):
            point.positions.extend(curve(tick/scale).tolist())
            point.velocities.extend((curve(tick/scale, nu=1)/scale).tolist())
            point.accelerations.extend((curve(tick/scale, nu=2)/scale**2).tolist())
        state = {**start, **dict(zip(names, point.positions))}
        if check is not None:
            check(state)
        valid = node.validity(state, group='both_arms')
        if not valid.valid:
            pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
            raise MoveItError(f'双臂同步轨迹存在碰撞或无效状态：{pairs}')
        result.joint_trajectory.points.append(point)
        previous = tick
    node.planned_start_state = node.planning_start_state(start)
    return result
