"""Mirrored bottle targets, coordinated dual-arm trajectories and contact checks."""
from copy import deepcopy
import json
import os
from pathlib import Path

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from onero_a1_common.constants import JOINTS, LEFT_JOINTS
from onero_a1_common.gains import GainClient
from onero_a1_common.trajectory_tools import _duration, _seconds

from .arm_gripper_task import build_steps, candidates
from .bottle_scene import BottleScene, PickerGeometry, matrix_pose, pose_matrix
from .dual_task_support import Mirror, arm_clients, endpoint, synchronize
from .support import MoveItError
from .task_support import (Step, confirm_opening, plan_step, preview_segment, rounded_waypoints,
                           task_state, wait_for_arm_settled, wait_seconds)


class GraspGoals:
    def __init__(self):
        self.handles = []

    def cancel_goal_async(self):
        futures = [handle.cancel_goal_async() for handle in self.handles]
        return futures[-1]


def grasp_both(node, arms, geometries):
    """Start both contact closes together; accept neither bottle without feedback."""
    clients = {side: ActionClient(node, GripperCommand, '/'+arm.gripper_controller+'/gripper_cmd')
               for side, arm in arms.items()}
    node.bottle_gripper_clients = clients
    for client in clients.values():
        if not client.wait_for_server(timeout_sec=10.):
            raise MoveItError('双夹爪动作服务未就绪')

    def command(targets):
        group = GraspGoals()
        futures = {}
        for side, client in clients.items():
            goal = GripperCommand.Goal()
            goal.command.position = float(targets[side])
            futures[side] = client.send_goal_async(goal)
        results = {}
        rejected = []
        for side, future in futures.items():
            handle = node.wait(future)
            if not handle.accepted:
                rejected.append(side)
                continue
            group.handles.append(handle)
            node.active_goal = group
            results[side] = handle.get_result_async()
        if rejected:
            raise MoveItError(f'{", ".join(rejected)} 夹爪拒绝夹取；未开始搬运')
        answers = {side: node.wait(future, 12.) for side, future in results.items()}
        node.active_goal = None
        return answers

    answers = command(dict.fromkeys(arms, 0.))
    actual, preload = task_state(node), {}
    for side, response in answers.items():
        contact, arm = response.result, arms[side]
        if (response.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
                or not contact.stalled or contact.reached_goal or not np.isfinite(contact.position)
                or not 0. < contact.position < 1.1):
            raise MoveItError(f'{side} 夹爪未检测到有效瓶身接触；未开始搬运')
        width = geometries[side].gap(contact.position)
        matched = (abs(width-arm.args.diameter) <= .008 if arm.args.grasp_opening is None else
                   abs(contact.position-arm.args.grasp_opening*1.1) <= .08)
        if not matched or abs(actual[arm.gripper_joint]-contact.position) > .015:
            raise MoveItError(
                f'{side} 夹爪接触尺寸或反馈不符；未开始搬运 '
                f'(模型宽度 {width*1000:.1f} mm，配置直径 {arm.args.diameter*1000:.1f} mm，'
                f'受阻开度 {contact.position:.4f} rad，反馈开度 {actual[arm.gripper_joint]:.4f} rad)')
        preload[side] = max(0., actual[arm.gripper_joint]-.005)
        print(f'{side} 已检测到瓶身接触，模型夹持宽度 {width*1000:.1f} mm。')
    for side, response in command(preload).items():
        if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.reached_goal:
            raise MoveItError(f'{side} 夹爪预紧未完成；未开始搬运')


class DualBottleTask:
    def __init__(self, node, arms, initial):
        self.node, self.arms = node, arms
        self.mirror = Mirror(node)
        # The right feedback defines the reference; the left arm first moves to its mirror.
        self.reference = {**initial, **dict(zip(LEFT_JOINTS, [-initial[name] for name in JOINTS]))}
        # Left/right CAD chains differ slightly. Mirrored joint signs are only
        # the seed; solve the exact mirrored flange pose in the left model.
        left_target = arms['left'].inverse_kinematics(
            self.mirror.pose(arms['right'].fk(initial)), self.reference,
            group=arms['left'].group, link=arms['left'].tip_link)
        self.reference.update({name:left_target[name] for name in LEFT_JOINTS})
        self.geometries, self.scenes, self.angles, self.offsets = {}, {}, {}, {}
        for side, arm in arms.items():
            geometry = PickerGeometry(arm, arm.args.diameter, pose_matrix(arm.fk(self.reference))[:3, :3])
            angle = geometry.angle if arm.args.grasp_opening is None else arm.args.grasp_opening*1.1
            if angle >= arm.args.open_opening*1.1:
                raise MoveItError('张开开度必须大于夹取开度')
            self.geometries[side], self.angles[side] = geometry, angle
            self.offsets[side] = geometry.center(angle) if arm.args.tcp_offset is None else np.array(arm.args.tcp_offset)
            if side == 'left' and arm.args.tcp_offset is not None:
                self.offsets[side] = self.mirror.tool.T @ self.offsets[side]
            self.scenes[side] = BottleScene(arm, geometry)
        self.steps, self.combined = {}, []
        self.holding = False

    def reset(self):
        for scene in self.scenes.values():
            if scene.bottle is not None:
                scene.world()
                scene.allow_touch(True, support=True)

    def prepare(self, initial):
        right = self.arms['right']
        errors = []
        for bottle, pregrasp, grasp, lifted in candidates(right, self.reference, self.offsets['right'], self.scenes['right']):
            try:
                left_bottle = deepcopy(bottle)
                left_bottle.position.y *= -1.
                for side, obj in (('right', bottle), ('left', left_bottle)):
                    arm, scene = self.arms[side], self.scenes[side]
                    height = scene.support_height(pose_matrix(obj)[:3, 3], arm.args.diameter/2.)
                    if height is None or abs(obj.position.z-arm.args.bottle_height/2.-height) > .001:
                        raise MoveItError(f'{side} 镜像瓶子下方没有匹配的支撑面')
                    scene.place(obj, arm.args.diameter, arm.args.bottle_height)
                    scene.allow_touch(True, support=True)
                self.steps['right'] = build_steps(right, self.reference, pregrasp, grasp, lifted, self.angles['right'])
                self.steps['left'] = build_steps(self.arms['left'], self.reference,
                    *[self.mirror.pose(pose) for pose in (pregrasp, grasp, lifted)], self.angles['left'])
                self.combined = self.run(initial)
                return
            except MoveItError as exc:
                errors.append(str(exc))
                print('镜像抓取方案不可用：'+str(exc))
            finally:
                self.reset()
        raise MoveItError('没有可行的双臂镜像取放方案：'+(errors[-1] if errors else '请调整右臂起点或加载桌面'))

    def carry(self, current):
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = LEFT_JOINTS+JOINTS
        elapsed = 0.
        for begin, end in ((4, 6), (6, 8)):
            trajectories, contacts = [], {}
            for side, arm in self.arms.items():
                scene, steps = self.scenes[side], self.steps[side]
                scene.allow_touch(True, support=True)
                poses = [pose for step in steps[begin:end] for pose in
                         (step.target if isinstance(step.target, list) else [step.target])]
                origin = arm.fk(current)
                contacts[side] = pose_matrix(origin if begin == 4 else steps[7].target)[:3, 3]
                path = rounded_waypoints(origin, poses)
                trajectories.append(plan_step(arm, current, Step('连续搬运', 'cartesian', path)))
            permitted = dict.fromkeys(self.arms, True)

            def check(state):
                for side, arm in self.arms.items():
                    position = pose_matrix(arm.fk(state))[:3, 3]
                    contact = contacts[side]
                    near = np.linalg.norm(position[:2]-contact[:2]) <= .025 and position[2] <= contact[2]+arm.args.lift+.002
                    if near != permitted[side]:
                        self.scenes[side].allow_touch(True, support=near)
                        permitted[side] = near

            try:
                trajectory = synchronize(self.node, current, trajectories, check=check)
            finally:
                for scene in self.scenes.values():
                    scene.allow_touch(True, support=True)
            for point in trajectory.joint_trajectory.points:
                if result.joint_trajectory.points and _seconds(point.time_from_start) == 0.:
                    continue
                point.time_from_start = _duration(elapsed+_seconds(point.time_from_start))
                result.joint_trajectory.points.append(point)
            elapsed = _seconds(result.joint_trajectory.points[-1].time_from_start)
            current = {**current, **dict(zip(result.joint_trajectory.joint_names, result.joint_trajectory.points[-1].positions))}
            if begin == 4:
                elapsed += self.node.args.pause
                hold = deepcopy(result.joint_trajectory.points[-1])
                hold.time_from_start = _duration(elapsed)
                result.joint_trajectory.points.append(hold)
        return result

    def run(self, initial, execute=False):
        node, current, combined = self.node, dict(initial), []
        gains = {side: GainClient(arm, f'/{side}/onero_topic_bridge') for side, arm in self.arms.items()} if execute else {}
        # A coordinated joint goal holds the right arm at its measured reference while aligning the left.
        alignment = Step('左臂到右臂起点的镜像位', 'joints', {name:self.reference[name] for name in node.joints})
        stages = [(-1, alignment)]+[(index, self.steps['right'][index]) for index in (0, 1, 2, 3, 4, 8)]
        for index, source in stages:
            step = deepcopy(source)
            print(f'{"执行" if execute else "预检查"}双臂：'+('连续提起、回起点、停顿并放回' if index == 4 else step.label))
            if execute:
                current = task_state(node)
            step.start = self.scenes['right'].state(current)
            if index == -1:
                step.trajectory = node.plan(current, node.joint_constraints(step.target))
            elif step.kind == 'gripper':
                targets = {arm.gripper_joint:self.steps[side][index].target for side, arm in self.arms.items()}
                step.trajectory = node.plan(current, node.joint_constraints(targets), group='both_grippers')
            elif index == 4:
                step.trajectory = self.carry(current)
            else:
                paths = [plan_step(arm, current, self.steps[side][index]) for side, arm in self.arms.items()]
                step.trajectory = synchronize(node, current, paths)
            if execute:
                node.show_plan(step.trajectory)
                if index == 3:
                    self.holding = True
                if index == 3 and not node.bottle_simulated:
                    grasp_both(node, self.arms, self.geometries)
                else:
                    node.execute(step.trajectory, controller=[arm.gripper_controller for arm in self.arms.values()]
                                 if step.kind == 'gripper' else None)
                    if step.kind == 'gripper':
                        for side, arm in self.arms.items():
                            confirm_opening(arm, self.steps[side][index].target)
                current = task_state(node)
            else:
                current = endpoint(current, [step])
            if index == 3:
                for side, scene in self.scenes.items():
                    scene.attach(current)
                    if execute:
                        gains[side].set_kp(120.)
                if execute:
                    wait_for_arm_settled(node)
            elif index == 8:
                self.holding = False
                for side, scene in self.scenes.items():
                    scene.world()
                    if execute:
                        gains[side].restore()
            combined.append(step)
        return combined

    def preview(self):
        try:
            for step in self.combined:
                if step.kind == 'cartesian' and step.label == '提起水瓶':
                    for scene in self.scenes.values():
                        scene.hide_world()
                trajectory = preview_segment(step.trajectory)
                self.node.display.publish(DisplayTrajectory(model_id='onero_a1', trajectory_start=step.start, trajectory=[trajectory]))
                wait_seconds(self.node, len(trajectory.joint_trajectory.points)*(.05+1./30.)+.2)
        finally:
            self.reset()


def example(node):
    if node.args.frame != 'world':
        raise MoveItError('双臂取放使用 --frame world')
    arms = arm_clients(node)
    modes = []
    devices = {}
    for side, arm in arms.items():
        values = arm.call(GetParameters, f'/{side}/onero_topic_bridge/get_parameters',
            GetParameters.Request(names=['simulate', 'model', 'can_channel', 'mounting'])).values
        if len(values) != 4 or values[0].type != 1 or any(value.type != 4 for value in values[1:]):
            raise MoveItError('无法确认双臂设备配置')
        modes.append(values[0].bool_value)
        devices[side] = [values[0].bool_value]+[value.string_value for value in values[1:]]
    if len(set(modes)) != 1:
        raise MoveItError('双臂模拟模式不一致')
    node.bottle_simulated = modes[0]
    print('运行模式：模拟。' if modes[0] else '运行模式：真机，请在左右镜像位置各摆放一个水瓶。')
    identity = {'version':'dual_mirror_v1', 'device':devices, **{name:getattr(node.args, name) for name in
        ('diameter','bottle_height','lift','reach','open_opening','grasp_opening','tcp_offset','support_height')}}
    path = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))/'tasks/arm_gripper_task_both.json'
    if node.args.execute:
        try:
            saved = json.loads(path.read_text())
        except (OSError, ValueError) as exc:
            raise MoveItError('请先预览，并按输出的左右镜像坐标各放一个水瓶') from exc
        if saved.get('config') != identity:
            raise MoveItError('设备或抓取配置变化，请重新预览')
        if node.args.bottle_position is not None and not np.allclose(node.args.bottle_position, saved['right_position'], atol=1e-6, rtol=0.):
            raise MoveItError('右瓶位置与预览不一致，请重新预览')
        arms['right'].args.bottle_position = saved['right_position']
    initial = task_state(node)
    task = DualBottleTask(node, arms, initial)
    task.prepare(initial)
    positions = {side:pose_matrix(scene.bottle.pose)[:3, 3].tolist() for side, scene in task.scenes.items()}
    print(f'水瓶中心（world，m）：右={positions["right"]}，左={positions["left"]}。')
    if node.args.execute:
        task.run(task_state(node), execute=True)
        print('双臂同步取放完成，两瓶已放回并松爪。')
    else:
        task.preview()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'config':identity, 'right_position':positions['right']}, ensure_ascii=False, indent=2))
        temporary.replace(path)
        print('预览完成；按坐标摆放两个水瓶，添加 --execute 执行。')
