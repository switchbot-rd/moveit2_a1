"""Select MTC solutions with the same natural-posture policy as other tasks."""
import math

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .support import MoveItError


def _positions(joint_state):
    names, values = joint_state.name, joint_state.position
    if (len(names) != len(values) or len(set(names)) != len(names)
            or not all(math.isfinite(value) for value in values)):
        raise MoveItError('MTC 场景包含无效关节位置')
    return dict(zip(names, values))


def rank_solution(node, message, *, wrist_limit=None, motions=None):
    """Return the shared whole-motion cost; reject invalid branches with MoveItError.

    Score every selected-arm waypoint once as a continuous geometric path.
    Gripper and attachment stages do not restart wrist recovery. The synthetic
    path is only for scoring; original MTC timings and scenes remain intact.
    """
    names = list(node.joints)
    start = _positions(message.start_scene.robot_state.joint_state)
    if not names or any(name not in start for name in names):
        raise MoveItError('MTC 起始场景缺少所选手臂的完整关节位置')
    state = dict(start)
    trajectory = RobotTrajectory()
    path = trajectory.joint_trajectory
    path.joint_names = names
    path.points = [JointTrajectoryPoint(positions=[start[name] for name in names])]
    arm_motion = False
    policy = getattr(node, 'mtc_natural_policy', None)
    for sub in message.sub_trajectory:
        source = sub.trajectory.joint_trajectory
        if sub.trajectory.multi_dof_joint_trajectory.points:
            raise MoveItError('MTC 构型评分不支持浮动基座轨迹')
        if source.joint_names or source.points:
            joints = list(source.joint_names)
            if (not joints or not source.points or len(set(joints)) != len(joints)
                    or any(len(point.positions) != len(joints)
                           or not all(math.isfinite(value) for value in point.positions)
                           for point in source.points)):
                raise MoveItError('MTC 返回空轨迹或无效关节位置')
            if set(joints) == set(names):
                if policy is not None:
                    limit = (motions or {}).get(sub.info.stage_id, {}).get('bend_limit')
                    end_limit = (motions or {}).get(sub.info.stage_id, {}).get('end_bend_limit')
                    phase = policy if limit is None else policy.with_bend_limit(limit, end_limit=end_limit)
                    phase.validate_path(state, sub.trajectory)
                indices = [joints.index(name) for name in names]
                first = source.points[0].positions
                if any(abs(first[index]-state[name]) > .01
                       for name, index in zip(names, indices)):
                    raise MoveItError('MTC 手臂阶段之间的关节位置不连续')
                for point in source.points:
                    positions = [point.positions[index] for index in indices]
                    if positions != list(path.points[-1].positions):
                        sample = JointTrajectoryPoint(positions=positions)
                        sample.time_from_start.sec = len(path.points)
                        path.points.append(sample)
                arm_motion = True
            elif joints != [node.gripper_joint]:
                raise MoveItError('MTC 轨迹必须仅包含所选手臂或夹爪')
            state.update(zip(joints, source.points[-1].positions))
        diff = _positions(sub.scene_diff.robot_state.joint_state)
        if any(abs(diff[name]-state[name]) > .01 for name in names if name in diff):
            raise MoveItError('MTC 场景变更包含未经轨迹连接的手臂跳变')
        state.update((name, value) for name, value in diff.items() if name not in names)
    if not arm_motion or len(path.points) < 2:
        raise MoveItError('MTC 解没有所选手臂的运动轨迹')
    profile = node.posture_profile()
    if not profile.enabled:
        raise MoveItError('MTC 需要启用自然构型评价，不能跳过构型筛选')
    if profile.enabled:
        if len(profile.reference) != len(names):
            raise MoveItError('MTC 构型参考与所选手臂关节不一致')
        limit = node.wrist_limit() if wrist_limit is None else float(wrist_limit)
        if not math.isfinite(limit) or not 0. < limit <= math.pi:
            raise MoveItError('MTC 腕部滚转范围必须在 (0, π] rad 内')
        wrists = [index for index in range(len(names)) if index % 7 in (4, 6)]
        allowance = {i: max(limit, abs(start[names[i]]-profile.reference[i])) for i in wrists}
        for point in path.points:
            for i in wrists:
                deviation = abs(point.positions[i]-profile.reference[i])
                if deviation > allowance[i]+.01:
                    raise MoveItError(f'MTC 轨迹途中加重了 {names[i]} 的滚转扭曲')
                allowance[i] = min(allowance[i], max(limit, deviation))
        if any(abs(path.points[-1].positions[i]-profile.reference[i]) > limit+.01 for i in wrists):
            raise MoveItError(f'MTC 轨迹终点 J5/J7 滚转超过自然构型范围 {math.degrees(limit):.1f}°')
    try:
        cost = float(node.motion_posture_cost(start, trajectory))
        # Reuse the fixed-task endpoint quality as well as the full-path
        # objective, so near-singular or cramped branches lose preference.
        selected = sorted({round(i*(len(path.points)-1)/min(23, len(path.points)-1))
                           for i in range(min(24, len(path.points)))})
        quality = [node.grasp_quality({**start, **dict(zip(names, path.points[i].positions))})[0]
                   for i in selected]
        cost += sum(quality)/len(quality)
        if policy is not None:
            cost += policy.motion_shape_cost(start, trajectory)
    except (ValueError, RuntimeError) as exc:
        raise MoveItError(f'MTC 自然构型评分失败：{exc}') from exc
    if not math.isfinite(cost):
        raise MoveItError('MTC 自然构型评分不是有限数值')
    return cost
