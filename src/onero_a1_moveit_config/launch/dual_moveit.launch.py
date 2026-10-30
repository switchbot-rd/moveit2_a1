"""A1L/A1R dual-arm MoveIt: shared scene, separate SDK sessions/controllers."""
from pathlib import Path
import math
import os
import json
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from onero_a1_bringup.planning import add_online_planner
from onero_a1_bringup.hardware_mode import configure_hardware_mode, mode_default
from onero_a1_bringup.controller_tolerances import TOLERANCE_ARGUMENTS, tolerance_launch_arguments
from onero_a1_bringup.dual_config import controller_config, joints, moveit_parameters
from onero_a1_bringup.environment import load_environment
from onero_a1_bringup.payload import Payload, payload_launch_arguments


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        text = value(name).lower()
        if text not in ('true', 'false'):
            raise ValueError(f'{name} 必须为 true/false')
        return text == 'true'

    real = value('hardware_mode') == 'real'
    allow = boolean('allow_execution')
    servo_enabled=boolean('enable_servo')
    spacing = float(value('arm_spacing'))
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('arm_spacing 必须为有限正数（米）')
    channels = {side: value(side+'_can_channel').strip() for side in ('left', 'right')}
    if not all(channels.values()) or len(set(channels.values())) != 2:
        raise ValueError('双臂必须使用两个不同的 CAN 通道')
    if real:
        for channel in channels.values():
            if not (Path('/sys/class/net')/channel).exists():
                raise ValueError(f'CAN 接口不存在：{channel}；尚未连接或使能任何机械臂')
    bringup = get_package_share_directory('onero_a1_bringup')
    config_dir = get_package_share_directory('onero_a1_moveit_config')
    description = get_package_share_directory('onero_a1_description')
    gripper = value('gripper_model')
    environment = value('environment_file').strip()
    if environment:
        environment = str(Path(environment).expanduser().resolve())
        load_environment(environment, mounting='humanoid_dual', arm_spacing=spacing,
                         real_hardware=real, allow_execution=allow)
    overrides = {name: value(name) for name in TOLERANCE_ARGUMENTS}
    rate = float(value('control_rate'))
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('control_rate 必须为有限正数')
    controllers = controller_config(bringup, overrides, rate, enable_servo=servo_enabled)
    temporary = tempfile.TemporaryDirectory(prefix='onero_dual_')
    controller_file = Path(temporary.name)/'controllers.yaml'
    controller_file.write_text(yaml.safe_dump(controllers, sort_keys=False))
    config = (MoveItConfigsBuilder('onero_a1', package_name='onero_a1_moveit_config')
              .robot_description(file_path=str(Path(description)/'urdf/onero_a1_dual.urdf.xacro'),
                                 mappings={'arm_spacing': str(spacing), 'gripper_model': gripper})
              .robot_description_semantic(file_path='config/onero_a1_dual.srdf.xacro',
                                          mappings={'gripper_model': gripper})
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl', 'chomp', 'stomp'])
              .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
              .to_moveit_configs())
    add_online_planner(config)
    params = moveit_parameters(config, gripper)
    params['allow_trajectory_execution'] = allow
    goal_time = max(controllers[side+'_arm_controller']['ros__parameters']['constraints']['goal_time']
                    for side in ('left', 'right'))
    params['trajectory_execution']['allowed_goal_duration_margin'] = max(2., goal_time+1.)
    robot = {'robot_description': params['robot_description']}
    all_joints = joints('left')+joints('right')
    gripper_joints = [side+'_woan_picker_left_finger_joint' for side in ('left', 'right')] if gripper == 'woan_picker' else []
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda _: temporary.cleanup() or [])])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot], output='log'),
        Node(package='tf2_ros', executable='static_transform_publisher', name='dual_world_frame',
             arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'], output='log'),
        Node(package='controller_manager', executable='ros2_control_node',
             additional_env={'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS' if servo_enabled else os.environ.get('RMW_FASTRTPS_PUBLICATION_MODE','SYNCHRONOUS')},
             parameters=[str(controller_file), robot], output='screen')]
    initial_right = [.40, .09, -.074, 1.69, -.04, .54, -.026]
    for side, model in (('left', 'A1L'), ('right', 'A1R')):
        payload = Payload.create(float(value(side+'_payload_mass')), yaml.safe_load(value(side+'_payload_cog')))
        actions.append(Node(package='onero_a1_bringup', executable='topic_bridge',
                            namespace=side, name='onero_topic_bridge', output='screen',
                            prefix=[value('runtime_python')], parameters=[{
            'model': model, 'mounting': 'humanoid_'+side, 'can_channel': channels[side],
            'simulate': not real, 'control_rate': rate, 'state_publish_rate': min(rate, 100.),
            'joint_names': joints(side), 'joint_commands_topic': f'/{side}/robot_joint_commands',
            'joint_states_topic': f'/{side}/robot_joint_states',
            'sim_initial_positions': initial_right if side == 'right' else [-v for v in initial_right],
            'kp': float(value('kp')), 'kd': float(value('kd')),
            'gravity_compensation': boolean('gravity_compensation'), 'velocity_feedforward': boolean('velocity_feedforward'),
            'with_gripper': boolean('with_gripper'), 'gripper_model': gripper,
            'payload_mass': payload.mass, 'payload_cog': list(payload.cog),
            'payload_transition_time': float(value('payload_transition_time')),
            'allow_gripper_execution': allow, 'activate_gripper_on_start': False,
            'gripper_controller_name': side+'_gripper_controller',
            'gripper_joint_name': side+'_woan_picker_left_finger_joint',
        }]))
    actions.append(Node(package='onero_a1_bringup', executable='health_watchdog', name='dual_bridge_watchdog',
                        parameters=[{'joint_states_topic': '/left/robot_joint_states',
                                     'additional_joint_state_topics': ['/right/robot_joint_states'],
                                     'controller_names': ['left_arm_controller', 'right_arm_controller']+(['servo_arm_controller'] if servo_enabled else []),
                                     'auto_reactivate': False}], output='screen'))
    guards = {side: Node(package='onero_a1_bringup', executable='wait_for_joint_states',
                         name='wait_for_'+side+'_bridge', parameters=[{
                             'joints': joints(side), 'topic': f'/{side}/robot_joint_states'}], output='screen')
              for side in ('left', 'right')}
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '-c', '/controller_manager'], output='screen')
    complete = Node(package='onero_a1_bringup', executable='wait_for_joint_states', name='wait_for_dual_states',
                    parameters=[{'joints': all_joints+gripper_joints}], output='screen')
    spawner = Node(package='controller_manager', executable='spawner',
                   arguments=['left_arm_controller', 'right_arm_controller', '-c', '/controller_manager'], output='screen')
    servo_spawner=Node(package='controller_manager',executable='spawner',
        arguments=['servo_arm_controller','-c','/controller_manager','--inactive'],output='screen')
    servo_nodes=[]
    if servo_enabled:
        servo_config=yaml.safe_load((Path(config_dir)/'config/servo.yaml').read_text())
        manager_margins=list(servo_config['joint_limit_margins'])
        servo_config.update(move_group_name='both_arms',joint_limit_margins=manager_margins*2)
        robot_parameters={key:params[key] for key in ('robot_description','robot_description_semantic',
                                                     'robot_description_kinematics','robot_description_planning')}
        servo_nodes=[
            Node(package='onero_a1_servo',executable='servo_node',name='servo_node',output='screen',
                 additional_env={'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS'},
                 parameters=[{'moveit_servo':servo_config,'update_period':servo_config['publish_period'],
                              'planning_group_name':'both_arms'},robot_parameters]),
            Node(package='onero_a1_bringup',executable='servo_manager',output='screen',parameters=[{
                'joints':all_joints,'normal_controllers':['left_arm_controller','right_arm_controller'],
                'bridges':['/left/onero_topic_bridge','/right/onero_topic_bridge'],
                'raw_feedback_topics':['/left/robot_joint_states','/right/robot_joint_states'],
                'gripper_model':gripper,'gripper_joints':gripper_joints or ['unused'],
                'gripper_controllers':['left_gripper_controller','right_gripper_controller'],
                'joint_limit_margins':manager_margins,'max_expected_latency':servo_config['max_expected_latency']},robot_parameters])]
        if boolean('servo_debug'):
            settings={'servo':servo_config,'joint_limits':{'robot_description_planning':params['robot_description_planning']},'arm':'both'}
            servo_nodes.append(ExecuteProcess(cmd=[value('runtime_python'),'-m','tools.diagnostics.servo_debug',
                '--output',value('servo_debug_file'),'--settings-json',json.dumps(settings)],
                cwd=os.environ.get('WS_DIR',os.getcwd()),output='screen'))
    move_group = Node(package='moveit_ros_move_group', executable='move_group', parameters=[params], output='screen')
    loader = Node(package='onero_a1_bringup', executable='load_environment', parameters=[{
        'environment_file': environment, 'mounting': 'humanoid_dual', 'arm_spacing': spacing,
        'real_hardware': real, 'allow_execution': allow, 'joints': all_joints,
        'gripper_model': gripper, 'gripper_joints': gripper_joints or ['unused'], 'planning_group': 'both_arms'}], output='screen')
    rviz_config = yaml.safe_load((Path(config_dir)/'rviz/moveit.rviz').read_text())
    for display in rviz_config['Visualization Manager']['Displays']:
        if display.get('Name') == 'MotionPlanning':
            display['Planning Request']['Planning Group'] = 'right_arm'
    rviz_path = Path(temporary.name)/'dual.rviz'
    rviz_path.write_text(yaml.safe_dump(rviz_config, sort_keys=False))
    rviz = Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz_path)],
                parameters=[{key: params[key] for key in ('robot_description', 'robot_description_semantic',
                    'robot_description_kinematics', 'robot_description_planning')}, config.planning_pipelines], output='log')

    def success(target, following):
        def handle(event, _context):
            if event.returncode != 0:
                return [LogInfo(msg='双臂启动检查未通过，后续轨迹控制器保持未启动')]
            return following
        return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=handle))

    after_state = [move_group]
    if boolean('rviz'):
        after_state.append(rviz)
    after_state.extend([loader] if environment else [spawner] if allow else [])
    actions.extend([success(guards['left'], [guards['right']]), success(guards['right'], [broadcaster]),
                    success(broadcaster, [complete]), success(complete, after_state),
                    success(loader, [spawner] if allow else [])])
    if servo_enabled and allow:
        actions.extend([success(spawner,[servo_spawner]),success(servo_spawner,servo_nodes)])
    if allow and gripper == 'woan_picker':
        actions.append(success(spawner, [ExecuteProcess(cmd=['ros2', 'service', 'call',
            f'/{side}_gripper_controller/activate', 'std_srvs/srv/Trigger', '{}'], output='screen')
            for side in ('left', 'right')]))
    actions.append(guards['left'])
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=configure_hardware_mode),
        DeclareLaunchArgument('hardware_mode', default_value='sim', choices=['sim', 'real']),
        DeclareLaunchArgument('enable_servo',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('servo_debug',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('servo_debug_file',default_value=''),
        DeclareLaunchArgument('left_can_channel', default_value='can1'),
        DeclareLaunchArgument('right_can_channel', default_value='can0'),
        DeclareLaunchArgument('arm_spacing', default_value='0.07'),
        DeclareLaunchArgument('gripper_model', default_value='woan_picker', choices=['none', 'woan_picker']),
        DeclareLaunchArgument('with_gripper', default_value='true'),
        *payload_launch_arguments(dual=True),
        DeclareLaunchArgument('allow_execution', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('environment_file', default_value=''),
        DeclareLaunchArgument('runtime_python', default_value=EnvironmentVariable('ONERO_RUNTIME_PYTHON', default_value='python3')),
        DeclareLaunchArgument('kp', default_value=mode_default('kp')),
        DeclareLaunchArgument('kd', default_value='1.0'),
        DeclareLaunchArgument('control_rate', default_value=mode_default('control_rate')),
        DeclareLaunchArgument('gravity_compensation', default_value=mode_default('gravity_compensation')),
        DeclareLaunchArgument('velocity_feedforward', default_value=mode_default('velocity_feedforward')),
        *tolerance_launch_arguments(), OpaqueFunction(function=setup)])
