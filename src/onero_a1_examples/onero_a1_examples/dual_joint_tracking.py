"""双臂十四关节角度误差测量：自动调整起点，逐对镜像往返并记录。"""
import argparse
import math

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from onero_a1_common.moveit_client import MoveItError
from onero_a1_common.trajectory_tools import _duration, _scaled_limits, validate_trajectory

from .support import positive, run_example


def _positive_integer(value):
    value = int(value)
    if value <= 0:
        raise argparse.ArgumentTypeError('必须是正整数')
    return value


def configure(parser):
    parser.add_argument('--joint-index', type=int, nargs='+', choices=range(1, 8),
                        default=list(range(1, 8)), help='测量关节编号，默认按 1～7 顺序逐对测量')
    parser.add_argument('--amplitude', type=positive, default=.1, help='往返转角幅度，rad，默认 0.1')
    parser.add_argument('--move-time', type=positive, default=3.,
                        help='每段移动时间，秒，默认 3；按速度、加速度限位自动延长')
    parser.add_argument('--hold-time', type=positive, default=1., help='每个停点保持时间，秒，默认 1')
    parser.add_argument('--repeats', type=_positive_integer, default=1, help='整组测量重复次数，默认 1')
    parser.set_defaults(record_tracking=True)


def measurement_start(start, names, bounds, joint_indices, amplitude):
    """Keep the measured pose valid and fit the full excursion inside hard limits."""
    selected = {f'joint{index}-a1_{side}' for index in joint_indices for side in ('l', 'r')}
    result = dict(start)
    for name in names:
        lower, upper = bounds[name]
        if not math.isfinite(start[name]) or not lower <= start[name] <= upper:
            raise MoveItError(f'{name} 当前角度 {start[name]:.6f} rad 超出硬限位 '
                              f'[{lower:.6f}, {upper:.6f}] rad')
    for name in names:
        if name not in selected:
            continue
        lower, upper = bounds[name]
        maximum = (upper-lower)/2.
        if amplitude > maximum:
            raise MoveItError(f'{name} 无法容纳 ±{amplitude:.6f} rad 的测量幅度；'
                              f'该关节最大允许幅度为 {maximum:.6f} rad')
        low, high = lower+amplitude, upper-amplitude
        if low > high:  # Equal half-range amplitudes can differ by floating-point rounding.
            low = high = (lower+upper)/2.
        result[name] = min(high, max(low, start[name]))
    return result


def measurement_trajectory(start, names, limits, *, joint_indices=range(1, 8), amplitude=.1,
                           move_time=3., hold_time=1., repeats=1, initial=None):
    """Build stopped quintic segments while keeping every unselected joint fixed."""
    result = RobotTrajectory()
    path = result.joint_trajectory
    path.joint_names = list(names)
    baseline = [float(start[name]) for name in names]
    tick = 0

    def append(positions, duration=0.):
        nonlocal tick
        tick += math.ceil(duration*1e9)
        path.points.append(JointTrajectoryPoint(
            positions=list(positions), velocities=[0.]*len(names), accelerations=[0.]*len(names),
            time_from_start=_duration(tick/1e9)))

    def move(target):
        previous = path.points[-1].positions
        duration = max(move_time, *(max(1.875*abs(q-p)/velocity,
                                        math.sqrt(5.7735026919*abs(q-p)/acceleration))
                                   for p, q, (velocity, acceleration)
                                   in zip(previous, target, limits)))
        append(target, duration)

    positions = [float(initial[name]) for name in names] if initial is not None else baseline
    append(positions)
    if positions != baseline:
        move(baseline)
    append(baseline, hold_time)
    indices = sorted(set(joint_indices))
    for _ in range(repeats):
        for index in indices:
            left, right = (names.index(f'joint{index}-a1_{side}') for side in ('l', 'r'))
            for sign in (1., 0., -1., 0.):
                target = list(baseline)
                target[left] += sign*amplitude
                target[right] -= sign*amplitude
                move(target)
                append(target, hold_time)
    return result


def example(node):
    if not node.dual or node.arm_side != 'both':
        raise MoveItError('本示例需要 dual_moveit.launch.py 双臂服务')
    node.planning_start_state({})
    baseline = measurement_start(node.start, node.joints, node._joint_position_bounds,
                                 node.args.joint_index, node.args.amplitude)
    adjusted = [name for name in node.joints if baseline[name] != node.start[name]]
    if adjusted:
        print('起点调整规划（其余关节保持原角度）：')
        for name in adjusted:
            print(f'  {name}: {math.degrees(node.start[name]):.3f}° → '
                  f'{math.degrees(baseline[name]):.3f}°')
    trajectory = measurement_trajectory(
        baseline, node.joints, _scaled_limits(node, node.joints, MoveItError), initial=node.start,
        joint_indices=node.args.joint_index, amplitude=node.args.amplitude,
        move_time=node.args.move_time, hold_time=node.args.hold_time, repeats=node.args.repeats)
    validate_trajectory(node, trajectory)
    node.show_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory, label='起点调整及双臂关节往返测量' if adjusted else '双臂关节往返测量')


def main(args=None):
    return run_example('dual_joint_tracking', __doc__, configure, example, args,
                       planning_pipelines=(), dual=True, tracking=True)
