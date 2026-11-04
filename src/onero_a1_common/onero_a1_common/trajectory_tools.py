"""Trajectory limits, smoothing and comparison helpers."""
from copy import deepcopy
import math

import numpy as np
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import GetParameters
from scipy.interpolate import BPoly, PPoly


def _seconds(duration):
    return duration.sec + duration.nanosec*1e-9


def _duration(seconds):
    nanoseconds = round(seconds*1e9)
    return Duration(sec=nanoseconds//10**9, nanosec=nanoseconds % 10**9)


def _scaled_limits(node, names, error):
    fields = ('has_velocity_limits', 'max_velocity', 'has_acceleration_limits', 'max_acceleration')
    values = node.call(GetParameters, '/move_group/get_parameters', GetParameters.Request(names=[
        f'robot_description_planning.joint_limits.{name}.{field}' for name in names for field in fields
    ])).values
    limits = []
    if len(values) != len(names)*len(fields):
        raise error('无法读取规划速度和加速度限位')
    for index, name in enumerate(names):
        has_v, velocity, has_a, acceleration = values[index*4:index*4+4]
        if not (has_v.type == has_a.type == 1 and has_v.bool_value and has_a.bool_value
                and velocity.type == acceleration.type == 3
                and math.isfinite(velocity.double_value) and velocity.double_value > 0
                and math.isfinite(acceleration.double_value) and acceleration.double_value > 0):
            raise error(f'{name} 缺少有效的规划速度或加速度限位')
        limits.append((velocity.double_value*node.args.velocity,
                       acceleration.double_value*node.args.acceleration))
    return limits


def trajectory_metrics(trajectory):
    path = trajectory.joint_trajectory
    if not path.joint_names or len(set(path.joint_names)) != len(path.joint_names) or len(path.points) < 2:
        raise ValueError('轨迹需要完整关节名称和至少两个路径点')
    positions = np.array([point.positions for point in path.points], dtype=float)
    if positions.shape != (len(path.points), len(path.joint_names)) or not np.isfinite(positions).all():
        raise ValueError('轨迹关节位置不完整或无效')
    times = np.array([_seconds(point.time_from_start) for point in path.points])
    if not np.isfinite(times).all() or times[0] != 0. or np.any(np.diff(times) <= 0.):
        raise ValueError('轨迹时间必须从零开始并严格递增')
    return {'points': len(path.points), 'duration': float(times[-1]),
            'length': float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())}


def _timed_curve(trajectory):
    trajectory_metrics(trajectory)
    path = trajectory.joint_trajectory
    derivatives = np.array([[p.positions, p.velocities, p.accelerations] for p in path.points], dtype=float)
    if derivatives.shape != (len(path.points), 3, len(path.joint_names)) or not np.isfinite(derivatives).all():
        raise ValueError('平滑需要完整有效的位置、速度和加速度')
    times = np.array([_seconds(p.time_from_start) for p in path.points])
    return times, BPoly.from_derivatives(times, derivatives)


def resample(trajectory, step=.01):
    """Sample the existing quintic curve; this does not remove its overshoot."""
    times, curve = _timed_curve(trajectory)
    count = max(2, math.ceil((times[-1]-times[0])/step)+1)
    result = deepcopy(trajectory)
    template = trajectory.joint_trajectory.points[0]
    points = []
    for stamp in np.linspace(times[0], times[-1], count):
        point = deepcopy(template)
        point.positions = [float(v) for v in curve(stamp)]
        point.velocities = [float(v) for v in curve.derivative()(stamp)]
        point.accelerations = [float(v) for v in curve.derivative(2)(stamp)]
        point.time_from_start = _duration(float(stamp))
        points.append(point)
    result.joint_trajectory.points = points
    return result


def _derivative_peaks(curve, order):
    """Bound each joint's actual spline, including both sides of every knot."""
    polynomial = PPoly.from_bernstein_basis(curve.derivative(order))
    edges = np.r_[curve.x, np.nextafter(curve.x[1:], curve.x[:-1])]
    peaks = abs(polynomial(edges)).max(axis=0)
    for index in range(len(peaks)):
        joint = PPoly(polynomial.c[:, :, index], polynomial.x, extrapolate=False)
        extrema = joint.derivative().roots(discontinuity=False, extrapolate=False)
        extrema = extrema[np.isfinite(extrema)]
        peaks[index] = max(peaks[index], abs(joint(extrema)).max(initial=0.))
    return peaks


def validate_trajectory(client, trajectory, *, context=None):
    """Check the controller's interpolated path without changing its timing."""
    from rclpy.serialization import serialize_message
    from .continuous import geometry
    from .moveit_client import MoveItError

    try:
        times, curve = _timed_curve(trajectory)
        names = list(trajectory.joint_trajectory.joint_names)
        if set(names) != set(client.joints):
            raise ValueError('检查轨迹的关节必须与所选手臂一致')
        if context is None:
            context = geometry(client, refresh=True)
        client.planning_start_state({})  # Load the actual position limits once.
        bounds = [client._joint_position_bounds[name] for name in names]
        constraints = getattr(client, '_motion_path_constraints', None)
        constraints = serialize_message(constraints) if constraints is not None else b''
        speeds = abs(curve.derivative().c).max(axis=(0, 2))
        for index, (begin, end) in enumerate(zip(times, times[1:])):
            steps = max(1, math.ceil((end-begin)/.02), math.ceil((end-begin)*speeds[index]/.01))
            for stamp in np.linspace(begin, end, steps+1):
                positions = curve(stamp)
                if any(not low-1e-9 <= q <= high+1e-9 for q, (low, high) in zip(positions, bounds)):
                    raise ValueError('轨迹超出关节限位')
                valid, pairs = context.validity(dict(zip(names, map(float, positions))), client.group, constraints)
                if not valid:
                    raise MoveItError(f'轨迹存在碰撞或无效状态：{pairs}')
    except (ValueError, RuntimeError) as exc:
        raise MoveItError(f'轨迹检查失败：{exc}') from exc


def smooth_trajectory(client, trajectory, *, jerk=10.):
    from rclpy.serialization import deserialize_message, serialize_message
    from moveit_msgs.msg import RobotTrajectory
    from .continuous import geometry
    from .moveit_client import MoveItError

    if not math.isfinite(jerk) or jerk <= 0:
        raise MoveItError('加速度变化率上限必须为有限正数')
    try:
        _timed_curve(trajectory)
        names = list(trajectory.joint_trajectory.joint_names)
        if set(names) != set(client.joints):
            raise ValueError('平滑轨迹的关节必须与所选手臂一致')
        limits = dict(zip(client.joints, _scaled_limits(client, client.joints, MoveItError)))
        context = geometry(client, refresh=True)
        start = client.planned_start_state or client.robot_state(client.start)
        result = deserialize_message(context.smooth(
            client.group, serialize_message(start), serialize_message(trajectory),
            {name: limits[name][0] for name in names}, {name: limits[name][1] for name in names},
            {name: jerk for name in names}), RobotTrajectory)
        if list(result.joint_trajectory.joint_names) != names:
            raise ValueError('平滑返回了不同的关节顺序')
        times, curve = _timed_curve(result)
        # JTC executes quintic segments. Bound that interpolation as well as
        # Ruckig's waypoints; uniform time scaling preserves the geometric path.
        # Exact extrema avoid conservative Bernstein bounds and spikes missed
        # by fixed-rate sampling, especially in short final intervals.
        peaks = [_derivative_peaks(curve, order) for order in (1, 2, 3)]
        velocity_ratio = float(max(peaks[0]/[limits[n][0] for n in names]))
        acceleration_ratio = float(max(peaks[1]/[limits[n][1] for n in names]))
        jerk_ratio = float(np.max(peaks[2])/jerk)
        stretch = max(1., velocity_ratio, math.sqrt(acceleration_ratio), jerk_ratio**(1./3.))
        if stretch > 1.001:
            print(f'平滑限速 {stretch:.2f}×（峰值/限值）：速度 {velocity_ratio:.2f}、'
                  f'加速度 {acceleration_ratio:.2f}、jerk {jerk_ratio:.2f}')
        if not math.isfinite(stretch):
            raise ValueError('平滑后轨迹时间无效')
        if stretch > 1.:
            stretch *= 1.001
            for point, stamp in zip(result.joint_trajectory.points, times):
                point.time_from_start = _duration(float(stamp*stretch))
                point.velocities = [float(v/stretch) for v in point.velocities]
                point.accelerations = [float(a/stretch**2) for a in point.accelerations]
        validate_trajectory(client, result, context=context)
        return result
    except (ValueError, RuntimeError) as exc:
        raise MoveItError(f'轨迹平滑失败：{exc}') from exc


def _curve(trajectory, names, limits, error):
    """Match the controller's position/velocity/acceleration interpolation."""
    joint_path = trajectory.joint_trajectory
    if set(joint_path.joint_names) != set(names) or len(joint_path.points) < 2:
        raise error('笛卡尔轨迹缺少完整的单臂关节路径')
    indices = [joint_path.joint_names.index(name) for name in names]
    times, derivatives = [], []
    for point in joint_path.points:
        values = (point.positions, point.velocities, point.accelerations)
        if any(len(values_) != len(names) or not all(map(math.isfinite, values_)) for values_ in values):
            raise error('笛卡尔轨迹缺少有效的位置、速度或加速度')
        times.append(_seconds(point.time_from_start))
        derivatives.append([[values_[index] for index in indices] for values_ in values])
    if times[0] != 0. or any(b <= a for a, b in zip(times, times[1:])):
        raise error('笛卡尔轨迹时间戳无效')
    curve = BPoly.from_derivatives(times, derivatives)
    # TOTG can return finite-difference derivatives at its endpoints. Make
    # corners stationary, confining this adjustment to the first/last 10 ms.
    times = sorted(set(times + [min(.01, times[-1]/2), max(times[-1]-.01, times[-1]/2)]))
    derivatives = [[curve(time, nu=order).tolist() for order in range(3)] for time in times]
    for boundary in (derivatives[0], derivatives[-1]):
        boundary[1:] = [[0.]*len(names), [0.]*len(names)]
    curve = BPoly.from_derivatives(times, derivatives)
    # Bernstein coefficients bound the entire spline, including between
    # samples. Stretch enough to respect the actual scaled MoveIt limits.
    velocities = abs(curve.derivative().c).max(axis=(0, 1))
    accelerations = abs(curve.derivative(2).c).max(axis=(0, 1))
    scale = max(1., *(max(v/v_limit, math.sqrt(a/a_limit))
                       for v, a, (v_limit, a_limit) in zip(velocities, accelerations, limits)))
    return times, curve, scale
