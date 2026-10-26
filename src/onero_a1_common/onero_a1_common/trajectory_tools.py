"""Shared stopped splines and scaled limits for continuous Cartesian execution."""
import math

from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import GetParameters
from scipy.interpolate import BPoly


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
