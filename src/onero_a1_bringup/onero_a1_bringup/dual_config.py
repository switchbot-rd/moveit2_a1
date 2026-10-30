"""Compose dual-arm settings from the shared single-arm configuration."""
from copy import deepcopy
from pathlib import Path
import yaml
from .controller_tolerances import resolve_controller_config


def joints(side):
    return [f'joint{i}-a1_{"l" if side == "left" else "r"}' for i in range(1, 8)]


def controller_config(bringup_dir, overrides, rate, enable_servo=False):
    source = yaml.safe_load((Path(bringup_dir)/'config/ros2_controllers.yaml').read_text())
    result = {'controller_manager': {'ros__parameters': {
        'update_rate': int(rate), 'joint_state_broadcaster': {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'}}}}
    for side in ('left', 'right'):
        arm = deepcopy(source['arm_controller'])
        params = arm['ros__parameters']
        old = list(params['joints'])
        params['joints'] = joints(side)
        constraints = params['constraints']
        for previous, current in zip(old, params['joints']):
            value = constraints.pop(previous)
            constraints[current] = value
        resolved = resolve_controller_config({'arm_controller': arm}, overrides)['arm_controller']
        name = side+'_arm_controller'
        result[name] = resolved
        result['controller_manager']['ros__parameters'][name] = {
            'type': 'joint_trajectory_controller/JointTrajectoryController'}
    if enable_servo:
        servo=deepcopy(result['left_arm_controller'])
        parameters=servo['ros__parameters']
        parameters['joints']=joints('left')+joints('right')
        parameters['constraints'].update(deepcopy(result['right_arm_controller']['ros__parameters']['constraints']))
        parameters.update(allow_nonzero_velocity_at_trajectory_end=True,interpolate_from_desired_state=True,cmd_timeout=.15)
        parameters['constraints']['goal_time']=0.
        result['servo_arm_controller']=servo
        result['controller_manager']['ros__parameters']['servo_arm_controller']={
            'type':'joint_trajectory_controller/JointTrajectoryController'}
    return result


def moveit_parameters(config, gripper_model):
    result = config.to_dict()
    kinematics = result['robot_description_kinematics']['arm']
    result['robot_description_kinematics'] = {side+'_arm': deepcopy(kinematics) for side in ('left', 'right')}
    limits = result['robot_description_planning']['joint_limits']
    dual_limits = {}
    for side in ('left', 'right'):
        for old, new in zip(joints('right'), joints(side)):
            dual_limits[new] = deepcopy(limits.get(new, limits[old]))
        if gripper_model == 'woan_picker':
            dual_limits[side+'_woan_picker_left_finger_joint'] = deepcopy(limits['woan_picker_left_finger_joint'])
    result['robot_description_planning']['joint_limits'] = dual_limits
    ompl = result['ompl']
    arm = ompl.pop('arm')
    ompl.pop('gripper', None)
    for side in ('left', 'right'):
        ompl[side+'_arm'] = deepcopy(arm)
        ompl[side+'_arm']['projection_evaluator'] = 'joints('+','.join(joints(side)[:2])+')'
    ompl['both_arms'] = deepcopy(arm)
    ompl['both_arms']['projection_evaluator'] = 'joints('+','.join(joints('left')[:2]+joints('right')[:2])+')'
    controllers = {'controller_names': []}
    for side in ('left', 'right'):
        name = side+'_arm_controller'
        controllers['controller_names'].append(name)
        controllers[name] = {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                             'default': True, 'joints': joints(side)}
        if gripper_model == 'woan_picker':
            name = side+'_gripper_controller'
            joint = side+'_woan_picker_left_finger_joint'
            controllers['controller_names'].append(name)
            controllers[name] = {'type': 'GripperCommand', 'action_ns': 'gripper_cmd',
                                 'default': True, 'command_joint': joint, 'joints': [joint]}
            ompl[side+'_gripper'] = {'enforce_joint_model_state_space': True,
                                    'default_planner_config': 'RRTConnect', 'planner_configs': ['RRTConnect']}
    if gripper_model == 'woan_picker':
        ompl['both_grippers'] = {'enforce_joint_model_state_space': True,
                                'default_planner_config': 'RRTConnect', 'planner_configs': ['RRTConnect']}
    result['moveit_simple_controller_manager'] = controllers
    return result
