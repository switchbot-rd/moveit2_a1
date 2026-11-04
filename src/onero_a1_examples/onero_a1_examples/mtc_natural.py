"""Keep MTC branches within comfortable axial and torso-side limits."""
import math
from copy import copy

import numpy as np

from .support import MoveItError

IK_POSTURE_WEIGHTS = [1., 1.5, .3, 2., 10., 14., 10.]


def natural_reference(node):
    reference = node.reference_seed()
    if reference is not None and len(getattr(node, 'joints', ())) == 7:
        reference = dict(reference)
        reference.update({node.joints[index]: 0. for index in (4, 5, 6)})
    return reference


class NaturalArmPolicy:
    def with_bend_limit(self, limit, *, end_limit=None):
        policy = copy(self)
        policy.limits = self.limits.copy()
        policy.limits[-1] = limit
        policy.end_bend_limit = end_limit
        return policy

    def __init__(self, node, *, wrist_limit, swivel_limit, bend_limit=math.radians(20.)):
        self.node, self.joints = node, tuple(node.joints)
        self.reference = natural_reference(node)
        self.limits = np.array([swivel_limit, math.pi, wrist_limit, wrist_limit,
                                0., 0., 0., bend_limit], dtype=float)
        self.tolerance = np.array([.01, .01, .01, .01, 1e-9, 1e-6, 1e-6, .01])
        if (len(self.joints) != 7 or self.reference is None
                or any(name not in self.reference for name in self.joints)
                or not all(math.isfinite(self.reference[name]) for name in self.joints)
                or not np.isfinite(self.limits).all()
                or any(not 0. < limit <= math.pi for limit in self.limits[:4])
                or abs(self.reference[self.joints[3]]) < .05):
            raise MoveItError('MTC 自然构型参考或轴向范围无效')
        self.elbow_sign = math.copysign(1., self.reference[self.joints[3]])
        self.prefix = node.arm_side+'_' if node.dual else ''
        self._basis = None

    def _position(self, positions, link):
        point = self.node.fk(positions, link).position
        result = np.array([point.x, point.y, point.z], dtype=float)
        if not np.isfinite(result).all():
            raise MoveItError('MTC 自然构型 FK 返回无效位置')
        return result

    def _measure(self, positions):
        if any(name not in positions or not math.isfinite(positions[name]) for name in self.joints):
            raise MoveItError('MTC 自然构型缺少完整、有限的手臂关节位置')
        if self._basis is None:
            base = self._position(self.reference, self.prefix+'base_link')
            if self.node.dual:
                other = 'right' if self.node.arm_side == 'left' else 'left'
                outward = base-self._position(self.reference, other+'_base_link')
            else:
                outward = self._position(self.reference, 'Link2')-base
            length = float(np.linalg.norm(outward))
            if not math.isfinite(length) or length < 1e-6:
                raise MoveItError('MTC 无法确定手臂向外的方向')
            self._basis = base, outward/length
        base, outward = self._basis
        elbow = self._position(positions, self.prefix+'Link4')
        wrist = self._position(positions, self.prefix+'Link6')
        axial = [abs(positions[self.joints[i]]-self.reference[self.joints[i]]) for i in (0, 2, 4, 6)]
        elbow_deficit = max(0., .05-self.elbow_sign*positions[self.joints[3]])
        inward = [max(0., -float((point-base)@outward))
                  for point in (elbow, .5*(elbow+wrist))]
        return np.array(axial+[elbow_deficit]+inward+[abs(positions[self.joints[5]])])

    def shape_cost(self, positions):
        cost = sum(IK_POSTURE_WEIGHTS[index]*(positions[self.joints[index]]**2)
                   for index in (4, 5, 6))
        shoulder = self._position(positions, self.prefix+'Link2')
        elbow = self._position(positions, self.prefix+'Link4')
        wrist = self._position(positions, self.prefix+'Link6')
        upper, forearm = np.linalg.norm(elbow-shoulder), np.linalg.norm(wrist-elbow)
        if min(upper, forearm) > .05:
            reach = np.linalg.norm(wrist-shoulder)/(upper+forearm)
            cost += 20.*max(0., .8-reach)**2
        return float(cost)

    def motion_shape_cost(self, start, trajectory):
        path = trajectory.joint_trajectory
        indices = np.unique(np.linspace(0, len(path.points)-1, min(24, len(path.points)), dtype=int))
        return float(np.mean([self.shape_cost({**start, **dict(zip(path.joint_names, path.points[i].positions))})
                              for i in indices]))

    @staticmethod
    def _finite_cost(value):
        value = float(value)
        if not math.isfinite(value):
            raise MoveItError('MTC 自然构型评分不是有限数值')
        return value

    def pose_cost(self, positions):
        if np.any(self._measure(positions) > self.limits+self.tolerance):
            raise MoveItError('MTC 构型存在轴向扭转、手肘反折或手肘、前臂越过躯干侧面')
        return self._finite_cost(self.node.posture_cost(positions)
                                 + self.node.grasp_quality(positions)[0]+self.shape_cost(positions))

    def validate_path(self, start, trajectory):
        path = trajectory.joint_trajectory
        names = list(path.joint_names)
        if (trajectory.multi_dof_joint_trajectory.points or not path.points
                or len(names) != len(self.joints) or set(names) != set(self.joints)):
            raise MoveItError('MTC 自然构型检查需要所选手臂的完整轨迹')
        allowance = np.maximum(self.limits, self._measure(start))
        for index, point in enumerate(path.points):
            if len(point.positions) != len(names):
                raise MoveItError('MTC 自然构型轨迹关节位置不完整')
            positions = {**start, **dict(zip(names, point.positions))}
            if index == 0 and any(abs(positions[name]-start[name]) > .01 for name in names):
                raise MoveItError('MTC 自然构型轨迹与阶段起点不连续')
            measured = self._measure(positions)
            if np.any(measured > allowance+self.tolerance):
                raise MoveItError('MTC 轨迹加重了轴向扭转、手肘反折或手肘、前臂向内折叠')
            allowance = np.minimum(allowance, np.maximum(self.limits, measured))
        end_limits = self.limits.copy()
        if getattr(self, 'end_bend_limit', None) is not None:
            end_limits[-1] = self.end_bend_limit
        if np.any(measured > end_limits+self.tolerance):
            raise MoveItError('MTC 轨迹终点尚未恢复自然构型')

    def stage_cost(self, solution, comment=''):
        """MTC uses a two-argument callback; infinity rejects the branch."""
        try:
            start = dict(solution.start.scene.current_state.joint_positions)
            end = dict(solution.end.scene.current_state.joint_positions)
            paths = []
            for sub in solution.toMsg().sub_trajectory:
                path = sub.trajectory.joint_trajectory
                if set(path.joint_names) & set(self.joints):
                    if set(path.joint_names) != set(self.joints):
                        raise MoveItError('MTC 阶段返回不完整的手臂轨迹')
                    paths.append(sub.trajectory)
            if not paths:
                return self.pose_cost(end)
            if len(paths) != 1:
                raise MoveItError('MTC 构型回调需要单个手臂运动阶段')
            self.validate_path(start, paths[0])
            cost = self.node.motion_posture_cost(start, paths[0])
            cost += self.motion_shape_cost(start, paths[0])
            cost += .5*(self.node.grasp_quality(start)[0]+self.node.grasp_quality(end)[0])
            return self._finite_cost(cost)
        except (MoveItError, ValueError, RuntimeError, KeyError, TypeError, AttributeError) as exc:
            solution.comment = str(exc)
            return math.inf
