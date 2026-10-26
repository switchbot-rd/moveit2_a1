"""Optional launch overrides for ros2_control arm execution tolerances."""
from copy import deepcopy
import math
from pathlib import Path
import tempfile

import yaml

TOLERANCE_ARGUMENTS = {
    'arm_trajectory_tolerance': '七关节运动过程位置误差容差（rad）；留空沿用 YAML，默认 0.20',
    'arm_goal_tolerance': '七关节终点位置误差容差（rad）；留空沿用 YAML，默认 0.1',
    'arm_goal_time_tolerance': '轨迹结束后允许额外等待的秒数；留空沿用 YAML，默认 1.0',
    'arm_stopped_velocity_tolerance': '终点停止速度容差（rad/s）；留空沿用 YAML，默认 0.02',
}


def tolerance_launch_arguments():
    from launch.actions import DeclareLaunchArgument
    return [DeclareLaunchArgument(name, default_value='', description=description)
            for name, description in TOLERANCE_ARGUMENTS.items()]


def _positive(value, name):
    try:
        if isinstance(value, bool):
            raise ValueError
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f'{name} 必须为有限正数') from None
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f'{name} 必须为有限正数')
    return number


def resolve_controller_config(config, overrides):
    """Preserve per-joint YAML values unless a uniform override is supplied."""
    result = deepcopy(config)
    parameters = result['arm_controller']['ros__parameters']
    constraints = parameters['constraints']
    for name, field in (
        ('arm_goal_time_tolerance', 'goal_time'),
        ('arm_stopped_velocity_tolerance', 'stopped_velocity_tolerance'),
    ):
        value = overrides.get(name, '')
        if str(value).strip():
            constraints[field] = _positive(value, name)
        _positive(constraints[field], f'constraints.{field}')
    for name, field in (
        ('arm_trajectory_tolerance', 'trajectory'),
        ('arm_goal_tolerance', 'goal'),
    ):
        value = overrides.get(name, '')
        for joint in parameters['joints']:
            if str(value).strip():
                constraints[joint][field] = _positive(value, name)
            _positive(constraints[joint][field], f'constraints.{joint}.{field}')
    return result


def controller_config_from_launch(context, config_file):
    from launch.substitutions import LaunchConfiguration
    config = yaml.safe_load(Path(config_file).read_text())
    if context.launch_configurations.get('arm', 'right') == 'left':
        parameters = config['arm_controller']['ros__parameters']
        parameters['joints'] = [name.replace('-a1_r', '-a1_l') for name in parameters['joints']]
        constraints = parameters['constraints']
        for name in list(constraints):
            if name.endswith('-a1_r'):
                constraints[name.replace('-a1_r', '-a1_l')] = constraints.pop(name)
    overrides = {name: LaunchConfiguration(name).perform(context) for name in TOLERANCE_ARGUMENTS}
    return resolve_controller_config(config, overrides), overrides


def execution_goal_margin(config, minimum=2.):
    """Let the controller finish its goal check before MoveIt's outer timeout."""
    goal_time = config['arm_controller']['ros__parameters']['constraints']['goal_time']
    return max(float(minimum), float(goal_time)+1.)


def prepare_controller_parameters(context, config_file):
    from launch.actions import OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
    from launch.event_handlers import OnShutdown
    config, overrides = controller_config_from_launch(context, config_file)
    if (not any(str(value).strip() for value in overrides.values())
            and context.launch_configurations.get('arm', 'right') == 'right'):
        return [SetLaunchConfiguration('onero_controllers_file', str(config_file))]
    with tempfile.NamedTemporaryFile(mode='w', prefix='onero_controllers_', suffix='.yaml',
                                     encoding='utf-8', delete=False) as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        generated = Path(stream.name)

    def cleanup(_context):
        generated.unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetLaunchConfiguration('onero_controllers_file', str(generated)),
    ]
