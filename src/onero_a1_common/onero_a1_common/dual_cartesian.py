"""Synchronize Cartesian edges without increasing either arm's speed or acceleration."""
import math

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from .constants import JOINTS, LEFT_JOINTS
from .trajectory_tools import _curve, _duration, _scaled_limits


def plan_dual_cartesian_path(node, start, left_waypoints, right_waypoints, error):
    if not node.dual or node.arm_side != 'both':
        raise error('双臂笛卡尔路径需要双臂服务并选择 both')
    if not left_waypoints or len(left_waypoints) != len(right_waypoints):
        raise error('左右臂必须提供相同数量的对应路径点')
    names = LEFT_JOINTS + JOINTS
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list(names)
    limits = _scaled_limits(node, names, error)
    current, elapsed = dict(start), 0.
    for edge, corners in enumerate(zip(left_waypoints, right_waypoints), 1):
        curves = []
        for side, joints, corner, bounds in zip(('left', 'right'), (LEFT_JOINTS, JOINTS), corners,
                                                (limits[:7], limits[7:])):
            response = node.cartesian_path(current, [corner], group=side+'_arm', link=side+'_Link7')
            if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 1.-1e-6:
                raise error(f'第 {edge} 条边 {side} 笛卡尔路径不完整：'
                            f'fraction={response.fraction}，code={response.error_code.val}')
            curves.append(_curve(response.solution, joints, bounds, error))
        # Stretch the faster arm to the slower arm's duration. Both arms stop
        # at each corresponding corner; scaling never increases derivatives.
        duration = max(times[-1]*scale for times, _, scale in curves)
        scales = [duration/times[-1] for times, _, _ in curves]
        knots = {0., duration}
        for (times, _, _), scale in zip(curves, scales):
            knots.update(time*scale for time in times)
        # Check the combined state, including both arms and the environment.
        # Include all source knots so resampling preserves each quintic segment.
        samples = max(1, math.ceil(duration/.02))
        knots.update(duration*i/samples for i in range(samples+1))
        last = None
        for time in sorted(knots):
            if last is not None and time-last < 1e-8:
                continue
            point = JointTrajectoryPoint(time_from_start=_duration(elapsed+time))
            for (_, curve, _), scale in zip(curves, scales):
                point.positions.extend(curve(time/scale).tolist())
                point.velocities.extend((curve(time/scale, nu=1)/scale).tolist())
                point.accelerations.extend((curve(time/scale, nu=2)/scale**2).tolist())
            # Also bound the joint-space distance between collision samples.
            previous = trajectory.joint_trajectory.points[-1] if trajectory.joint_trajectory.points else point
            steps = max(1, math.ceil(max(abs(a-b) for a, b in zip(previous.positions, point.positions))/.02))
            for step in range(1, steps+1):
                sample_time = (last or 0.)+(time-(last or 0.))*step/steps
                positions = [value for (_, curve, _), scale in zip(curves, scales)
                             for value in curve(sample_time/scale).tolist()]
                state = dict(current)
                state.update(zip(names, positions))
                valid = node.validity(state, group='both_arms')
                if not valid.valid:
                    pairs = [(c.contact_body_1, c.contact_body_2) for c in valid.contacts]
                    raise error(f'双臂第 {edge} 条边存在碰撞或无效状态：{pairs}；请调整起点或减小 --size')
            if last is not None or not trajectory.joint_trajectory.points:
                trajectory.joint_trajectory.points.append(point)
            last = time
        current.update(zip(names, trajectory.joint_trajectory.points[-1].positions))
        elapsed += duration
    node.planned_start_state = node.planning_start_state(start)
    return trajectory
