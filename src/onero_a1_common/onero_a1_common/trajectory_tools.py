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


def _curve_position_extrema(curve):
    polynomial = PPoly.from_bernstein_basis(curve)
    edges = np.r_[curve.x, np.nextafter(curve.x[1:], curve.x[:-1])]
    values = polynomial(edges)
    minimum, maximum = values.min(axis=0), values.max(axis=0)
    for index in range(len(minimum)):
        joint = PPoly(polynomial.c[:, :, index], polynomial.x, extrapolate=False)
        roots = joint.derivative().roots(discontinuity=False, extrapolate=False)
        roots = roots[np.isfinite(roots)]
        minimum[index] = min(minimum[index], joint(roots).min(initial=math.inf))
        maximum[index] = max(maximum[index], joint(roots).max(initial=-math.inf))
    if not np.isfinite(minimum).all() or not np.isfinite(maximum).all():
        raise ValueError('轨迹插值位置无效')
    return minimum, maximum


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
        # This pass uses one scene snapshot and one set of constraints. Reuse
        # exact configurations only within this call; never round nearby poses.
        checked = set()
        for index, (begin, end) in enumerate(zip(times, times[1:])):
            coefficients = curve.c[:, index, :]
            if np.all(coefficients == coefficients[0]):
                # Equal Bernstein coefficients prove the entire segment is a
                # hold. Equal endpoints alone do not prove this.
                samples = (coefficients[0],)
            else:
                steps = max(1, math.ceil((end-begin)/.02), math.ceil((end-begin)*speeds[index]/.01))
                samples = map(curve, np.linspace(begin, end, steps+1))
            for positions in samples:
                if any(not low-1e-9 <= q <= high+1e-9 for q, (low, high) in zip(positions, bounds)):
                    raise ValueError('轨迹超出关节限位')
                key = tuple(map(float, positions))
                if key in checked:
                    continue
                valid, pairs = context.validity(dict(zip(names, key)), client.group, constraints)
                if not valid:
                    raise MoveItError(f'轨迹存在碰撞或无效状态：{pairs}')
                checked.add(key)
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
    # Use actual extrema of the controller's quintic segments. Bernstein
    # coefficients are only an upper bound and can needlessly multiply timing.
    velocities = _derivative_peaks(curve, 1)
    accelerations = _derivative_peaks(curve, 2)
    scale = max(1., *(max(v/v_limit, math.sqrt(a/a_limit))
                       for v, a, (v_limit, a_limit) in zip(velocities, accelerations, limits)))
    if scale > 1.:
        scale *= 1.001  # Leave room for nanosecond timestamp rounding.
    return times, curve, scale


def _synchronized_trajectory(curves, names, limits, error):
    """Emit stable controller knots, then enforce limits on their actual spline."""
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    duration = max(times[-1]*scale for times, _, scale in curves)
    limits = np.asarray(limits, dtype=float)
    minimum_interval_ns = 1_000_000
    for _ in range(5):
        duration_ns = max(1_000_000, math.ceil(duration*1e9))
        duration = duration_ns*1e-9
        scales = [duration/times[-1] for times, _, _ in curves]
        stamps = {0, duration_ns}
        for (times, _, _), scale in zip(curves, scales):
            stamps.update(min(duration_ns, max(0, round(t*scale*1e9))) for t in times)
        count = max(1, math.ceil(duration/.02))
        stamps.update(round(duration_ns*i/count) for i in range(count+1))
        # Nearly coincident left/right knots magnify floating-point position
        # and timestamp rounding into acceleration spikes. Collision sampling
        # is performed separately on the emitted spline, at the original bounds.
        kept = [0]
        for stamp in sorted(stamps)[1:-1]:
            if stamp-kept[-1] >= minimum_interval_ns and duration_ns-stamp >= minimum_interval_ns:
                kept.append(stamp)
        kept.append(duration_ns)
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = list(names)
        for stamp in kept:
            sec, nanosec = divmod(stamp, 1_000_000_000)
            point = JointTrajectoryPoint(time_from_start=Duration(sec=sec, nanosec=nanosec))
            for (times, curve, _), scale in zip(curves, scales):
                sample = (stamp/duration_ns)*times[-1]
                point.positions.extend(curve(sample).tolist())
                point.velocities.extend((curve(sample, nu=1)/scale).tolist())
                point.accelerations.extend((curve(sample, nu=2)/scale**2).tolist())
            if stamp in (0, duration_ns):
                point.velocities = [0.]*len(names)
                point.accelerations = [0.]*len(names)
            result.joint_trajectory.points.append(point)
        _, controller_curve = _timed_curve(result)
        # Bound the geometric difference on the union of all polynomial
        # intervals. The Bernstein coefficients of their difference bound it
        # everywhere, including between the collision samples.
        comparison_times = set(controller_curve.x)
        for (times, _, _), scale in zip(curves, scales):
            comparison_times.update(min(duration, max(0., t*scale)) for t in times)
        comparison_times = sorted(comparison_times)
        differences = []
        for stamp in comparison_times:
            derivatives = []
            for order in range(3):
                reference = np.concatenate([curve(min(times[-1], max(0., stamp/scale)), nu=order)/scale**order
                    for (times, curve, _), scale in zip(curves, scales)])
                derivatives.append(controller_curve(stamp, nu=order)-reference)
            differences.append(derivatives)
        difference = BPoly.from_derivatives(comparison_times, differences)
        deviation = float(abs(difference.c).max())
        if not math.isfinite(deviation):
            raise error('同步轨迹的路径误差无效')
        if deviation > 1e-6:
            if minimum_interval_ns <= 10_000:
                raise error('同步轨迹时间点合并后偏离原路径，停止规划')
            minimum_interval_ns //= 10
            continue
        velocities = _derivative_peaks(controller_curve, 1)
        accelerations = _derivative_peaks(controller_curve, 2)
        if not np.isfinite(velocities).all() or not np.isfinite(accelerations).all():
            raise error('同步轨迹的插值导数无效')
        stretch = max(1., float(max(velocities/limits[:, 0])),
                      math.sqrt(float(max(accelerations/limits[:, 1]))))
        if stretch <= 1.+1e-9:
            return result, controller_curve
        # Re-evaluate positions/derivatives at the new published times rather
        # than just rounding a second set of scaled timestamps.
        duration *= stretch*1.001
    raise error('同步轨迹无法满足速度和加速度上限，停止规划')


def _curve_sample_times(curve):
    """Check every knot and bound intervals by 20 ms and 0.02 rad per joint."""
    duration = curve.x[-1]
    speed = float(max(_derivative_peaks(curve, 1)))*(1.+1e-9)
    count = max(1, math.ceil(duration*max(50., speed/.02)))
    times = set(curve.x)
    times.update(np.linspace(0., duration, count+1).tolist())
    previous = -1.
    for stamp in sorted(times):
        if stamp-previous > 1e-9:
            yield stamp
            previous = stamp
