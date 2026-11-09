"""在仿真中运行原有规划示例，记录耗时；不执行轨迹、不调整规划参数。"""
import argparse
import functools
import importlib
import json
import os
from pathlib import Path
import threading
import time


EXAMPLES = (
    'joint_goal', 'named_pose', 'pose_goal', 'cartesian_path',
    'orientation_constraint', 'planning_scene', 'planning_compare', 'gripper',
    'arm_gripper_task', 'fixed_pick_place', 'mtc_pick_place', 'return_startup',
    'dual_joint_goal', 'dual_joint_tracking', 'dual_cartesian_square', 'dual_gripper',
)


class Timings:
    def __init__(self, output, metadata):
        self.output, self.metadata = output, metadata
        self.started = time.perf_counter()
        self.local = threading.local()
        self.metrics, self.events = {}, []
        self.lock = threading.Lock()
        self.restores = []

    def wrap(self, function, name, detail=None, observe=None):
        @functools.wraps(function)
        def measured(*args, **kwargs):
            label = name if detail is None else name + detail(args, kwargs)
            stack = getattr(self.local, 'stack', None)
            if stack is None:
                self.local.stack = stack = []
            frame = [time.perf_counter(), 0.]
            stack.append(frame)
            result, failure = None, None
            try:
                result = function(*args, **kwargs)
                return result
            except BaseException as exc:
                failure = type(exc).__name__ + ': ' + str(exc)
                raise
            finally:
                elapsed = time.perf_counter() - frame[0]
                stack.pop()
                if stack:
                    stack[-1][1] += elapsed
                with self.lock:
                    row = self.metrics.setdefault(label, dict(
                        calls=0, inclusive_s=0., exclusive_s=0., max_s=0., failures=0))
                    row['calls'] += 1
                    row['inclusive_s'] += elapsed
                    row['exclusive_s'] += max(0., elapsed-frame[1])
                    row['max_s'] = max(row['max_s'], elapsed)
                    row['failures'] += failure is not None
                if observe is not None:
                    event = dict(name=label, start_s=frame[0]-self.started,
                                 elapsed_s=elapsed, error=failure)
                    if failure is None:
                        event.update(observe(args, kwargs, result))
                    self.events.append(event)
        return measured

    def patch(self, owner, name, label=None, **kwargs):
        original = vars(owner).get(name)
        if original is None:
            return
        function = original.__func__ if isinstance(original, staticmethod) else original
        replacement = self.wrap(function, label or owner.__name__+'.'+name, **kwargs)
        if isinstance(original, staticmethod):
            replacement = staticmethod(replacement)
        setattr(owner, name, replacement)
        self.restores.append((owner, name, original))

    def save(self):
        data = dict(self.metadata, elapsed_s=time.perf_counter()-self.started,
                    metrics=self.metrics, events=self.events)
        temporary = self.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
        temporary.replace(self.output)

    def restore(self):
        for owner, name, original in reversed(self.restores):
            setattr(owner, name, original)


def install(timings):
    from onero_a1_common.moveit_client import MoveItClient, MoveItError, seconds
    from rcl_interfaces.srv import GetParameters

    def action_detail(args, kwargs):
        client, action, goal = args[:3]
        if action is client.planner:
            request = goal.request
            return f'[{request.pipeline_id}/{request.planner_id}/{request.group_name}]'
        return '[execution]'

    def action_result(args, kwargs, result):
        return dict(server_planning_s=getattr(result, 'planning_time', None))

    def show_result(args, kwargs, result):
        path = args[1].joint_trajectory
        length = sum(sum((b-a)**2 for a, b in zip(first.positions, second.positions))**.5
                     for first, second in zip(path.points, path.points[1:]))
        cost = next((event['cost'] for event in reversed(timings.events)
                     if event.get('trajectory_id') == id(args[1]) and 'cost' in event), None)
        return dict(points=len(path.points), joints=list(path.joint_names),
                    joint_path_length=length, posture_cost=cost,
                    trajectory_duration_s=seconds(path.points[-1].time_from_start) if path.points else 0.)

    for name in ('__init__', 'select_arm', 'state', 'planning_start_state',
                 'parameter', 'fk', 'validity', 'inverse_kinematics',
                 'posture_profile', 'posture_cost', 'elbow_cost', 'condition_number',
                 'grasp_quality', 'ik_candidates', 'motion_posture_cost',
                 'plan_pose_candidates', 'plan_position', 'plan_upright',
                 'plan', '_plan_with_posture', 'cartesian_path', 'dual_cartesian_path',
                 'smooth_trajectory', 'cleanup'):
        timings.patch(MoveItClient, name)
    timings.patch(MoveItClient, 'call', detail=lambda args, kw: '['+args[2]+']')
    timings.patch(MoveItClient, 'action', detail=action_detail, observe=action_result)
    timings.patch(MoveItClient, 'show_plan', observe=show_result)

    # Verify the service we actually discovered before a scenario can add objects.
    select_arm = MoveItClient.select_arm
    verified = set()

    @functools.wraps(select_arm)
    def simulation_only(node, *args, **kwargs):
        result = select_arm(node, *args, **kwargs)
        bridges = ([f'/{side}/onero_topic_bridge' for side in ('left', 'right')]
                   if node.dual else ['/onero_topic_bridge'])
        for bridge in bridges:
            if bridge in verified:
                continue
            began = time.perf_counter()
            response = node.call(GetParameters, bridge+'/get_parameters',
                                 GetParameters.Request(names=['simulate']))
            values = response.values
            if len(values) != 1 or values[0].type != 1 or not values[0].bool_value:
                raise MoveItError('耗时采集器只允许运行在 simulate=true 的仿真服务')
            verified.add(bridge)
            timings.events.append(dict(name='diagnostic_mode_check',
                                       elapsed_s=time.perf_counter()-began, bridge=bridge))
        return result
    timings.restores.append((MoveItClient, 'select_arm', select_arm))
    MoveItClient.select_arm = simulation_only

    def no_execution(*args, **kwargs):
        raise MoveItError('耗时采集器禁止执行轨迹')
    timings.restores.append((MoveItClient, 'execute', MoveItClient.execute))
    MoveItClient.execute = no_execution

    for module_name, names in (
        ('onero_a1_common.continuous', ('geometry', 'retime')),
        ('onero_a1_common.posture', ('trajectory_cost', 'load_profile')),
        ('onero_a1_common.trajectory_tools', ('_scaled_limits', '_curve',
                                            'validate_trajectory', 'smooth_trajectory')),
    ):
        module = importlib.import_module(module_name)
        for name in names:
            if name == 'trajectory_cost':
                timings.patch(module, name, observe=lambda a, k, r:
                              dict(cost=float(r), trajectory_id=id(a[1])))
            else:
                timings.patch(module, name)

    from onero_a1_kinematics._core import PlanningGeometry
    for name in ('__init__', 'set_scene', 'fk', 'validity', 'posture',
                 'motion_posture', 'position_ik', 'upright_ik', 'retime', 'smooth'):
        timings.patch(PlanningGeometry, name, 'native.geometry.'+name)

    # Install before the task modules import these functions by name.
    from onero_a1_examples import task_support
    timings.patch(task_support, 'wait_seconds', 'preview.wait_seconds')
    timings.patch(task_support, 'plan_step')
    from onero_a1_examples.mtc_planning import MtcPlanner
    from onero_a1_examples.mtc_natural import NaturalArmPolicy
    from onero_a1_examples.mtc_relay import MtcRelay
    for name in ('_build', '_plan_task', 'plan', 'plan_motion', 'plan_departure'):
        timings.patch(MtcPlanner, name, observe=lambda a, k, r: {})
    for name in ('stage_cost', 'pose_cost', 'validate_path', 'shape_cost', 'motion_shape_cost'):
        timings.patch(NaturalArmPolicy, name)
    for name in ('initialize', 'plan_leg'):
        timings.patch(MtcRelay, name, observe=lambda a, k, r: {})
    for module_name, names in (
        ('onero_a1_examples.mtc_relay', ('select_solution',)),
        ('onero_a1_examples.mtc_posture', ('rank_solution',)),
    ):
        module = importlib.import_module(module_name)
        for name in names:
            timings.patch(module, name)

    from moveit.task_constructor import core
    for name in ('SolutionBase', 'SubTrajectory', 'SolutionSequence', 'WrappedSolution'):
        owner = getattr(core, name, None)
        if owner is not None:
            timings.patch(owner, 'toMsg', 'native.mtc.'+name+'.toMsg')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='新建的 JSON 结果文件')
    parser.add_argument('--python-profile', action='store_true', help='额外记录 Python 调用统计；会增加采集开销')
    parser.add_argument('example', choices=EXAMPLES)
    parser.add_argument('example_args', nargs=argparse.REMAINDER)
    options = parser.parse_args(argv)
    supplied = options.example_args
    if supplied[:1] == ['--']:
        supplied = supplied[1:]
    if any(arg.startswith('--') and '--execute'.startswith(arg.split('=')[0]) for arg in supplied):
        parser.error('只支持规划预览，不接受 --execute')
    if any(arg in ('--reset-task', '--record-orientation') for arg in supplied):
        parser.error('耗时采集不运行任务重置或配置记录操作')
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID'] == '0':
        parser.error('请使用独立的非零 ROS_DOMAIN_ID 和仿真服务')
    if options.output.exists():
        parser.error('结果文件已存在，请指定新文件')
    options.output.parent.mkdir(parents=True, exist_ok=True)
    metadata = dict(example=options.example, arguments=supplied,
                    ros_domain_id=os.environ['ROS_DOMAIN_ID'],
                    python_profile=options.python_profile, status='running',
                    notes=['inclusive_s 包含子调用；exclusive_s 扣除被采集的子调用，不能将各层 inclusive_s 相加。',
                           'elapsed_s 包含诊断初始化及清理；app.scenario 更接近示例本体的计算时间。'])
    timings = Timings(options.output, metadata)
    profile = None
    try:
        install(timings)
        module = importlib.import_module('onero_a1_examples.'+options.example)
        timings.patch(module, 'example', 'app.scenario', observe=lambda a, k, r: {})
        timings.save()
        if options.python_profile:
            import cProfile
            profile = cProfile.Profile()
            profile.enable()
        began = time.perf_counter()
        code = module.main(supplied)
        metadata.update(status='success' if code == 0 else 'failed',
                        exit_code=code, main_s=time.perf_counter()-began)
        return code
    except BaseException as exc:
        metadata.update(status='failed', exception=type(exc).__name__+': '+str(exc))
        raise
    finally:
        if profile is not None:
            profile.disable()
            profile.dump_stats(str(options.output.with_suffix('.pstats')))
        timings.save()
        timings.restore()
        print('耗时记录：'+str(options.output), flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
