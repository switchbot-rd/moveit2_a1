"""Synchronize Cartesian edges without increasing either arm's speed or acceleration."""

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory

from .constants import JOINTS, LEFT_JOINTS
from .trajectory_tools import (
    _curve, _scaled_limits, _synchronized_trajectory, _curve_sample_times, _curve_position_extrema,
)


def plan_dual_cartesian_path(node, start, left_waypoints, right_waypoints, error):
    from .continuous import geometry
    if not node.dual or node.arm_side != 'both':
        raise error('双臂笛卡尔路径需要双臂服务并选择 both')
    if not left_waypoints or len(left_waypoints) != len(right_waypoints):
        raise error('左右臂必须提供相同数量的对应路径点')
    names = LEFT_JOINTS + JOINTS
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(names)
    limits = _scaled_limits(node, names, error)
    context = geometry(node, refresh=True)
    node.planning_start_state({})
    position_bounds = [node._joint_position_bounds[name] for name in names]
    current, elapsed_ns = dict(start), 0
    for edge, corners in enumerate(zip(left_waypoints, right_waypoints), 1):
        curves = []
        for side, joints, corner, bounds in zip(('left', 'right'), (LEFT_JOINTS, JOINTS), corners,
                                                (limits[:7], limits[7:])):
            response = node.cartesian_path(current, [corner], group=side+'_arm', link=side+'_Link7')
            if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 1.-1e-6:
                raise error(f'第 {edge} 条边 {side} 笛卡尔路径不完整：'
                            f'fraction={response.fraction}，code={response.error_code.val}')
            curves.append(_curve(response.solution, joints, bounds, error))
        segment, controller_curve = _synchronized_trajectory(curves, names, limits, error)
        minimum, maximum = _curve_position_extrema(controller_curve)
        if any(low-1e-9 > qmin or qmax > high+1e-9
               for qmin, qmax, (low, high) in zip(minimum, maximum, position_bounds)):
            raise error(f'双臂第 {edge} 条边超出关节限位')
        # Validate the emitted controller spline, including both arms, using
        # the same time and joint-distance sampling bounds as synchronization.
        for time in _curve_sample_times(controller_curve):
            positions = controller_curve(time)
            state = {**current, **dict(zip(names, positions))}
            valid = node.validity(state, group='both_arms', context=context)
            if not valid.valid:
                pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
                raise error(f'双臂第 {edge} 条边存在碰撞或无效状态：{pairs}；请调整起点或减小 --size')
        for point in segment.joint_trajectory.points:
            stamp = point.time_from_start.sec*1_000_000_000+point.time_from_start.nanosec
            if stamp or not trajectory.joint_trajectory.points:
                point.time_from_start.sec, point.time_from_start.nanosec = divmod(elapsed_ns+stamp, 1_000_000_000)
                trajectory.joint_trajectory.points.append(point)
        current.update(zip(names, trajectory.joint_trajectory.points[-1].positions))
        elapsed_ns = (trajectory.joint_trajectory.points[-1].time_from_start.sec*1_000_000_000
                      +trajectory.joint_trajectory.points[-1].time_from_start.nanosec)
    node.planned_start_state = node.planning_start_state(start)
    return trajectory
