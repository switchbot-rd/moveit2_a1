"""One bottle: right A→B, left B→C→B, right B→A."""
from copy import deepcopy
import json
import os
from pathlib import Path

import numpy as np
import yaml
from moveit_msgs.msg import DisplayTrajectory
from scipy.spatial.transform import Rotation
from onero_a1_common.gains import GainClient
from onero_a1_common.moveit_client import seconds

from .bottle_scene import matrix_pose, pose_matrix
from .dual_task_support import Mirror, arm_clients, endpoint
from .fixed_pick_place import FixedTask, preset_path, preview_identity, preview_settings, shifted
from .fixed_pick_place_config import TaskConfig
from .task_support import Step, plan_step, task_state, wait_seconds
from .support import MoveItError


class RelayTask:
    def __init__(self, node, arms, config):
        self.node, self.arms, self.config = node, arms, config
        self.mirror = Mirror(node)
        self.points = {'A': list(config.pick_position), 'B': list(config.place_position),
                       'C': (list(config.c_position) if config.c_position is not None
                             else self.mirror.point(config.pick_position).tolist())}
        self.route = [('right', 'A', 'B'), ('left', 'B', 'C'),
                      ('left', 'C', 'B'), ('right', 'B', 'A')]
        if not node.args.round_trip:
            self.route = self.route[:2]
        self.legs = []
        self.first = self.current = self.make_leg(0)
        self.wait_step = None
        self.executing = self.completed = False

    @property
    def scene(self):
        return self.current.scene

    @property
    def holding(self):
        return self.current.holding

    def make_leg(self, index, released=None):
        side, pick, place = self.route[index]
        cfg = deepcopy(self.config)
        cfg.arm = side
        cfg.pick_position = ([released.position.x, released.position.y, released.position.z]
                             if released is not None else self.points[pick])
        place_offset = ([0., 0., 0.] if (side != 'left' or place != 'B'
                         or cfg.left_b_place_offset is None)
                        else cfg.left_b_place_offset)
        cfg.place_position = [float(p+o) for p, o in zip(self.points[place], place_offset)]
        # A is on the tabletop; B/C use the dark plate/tray base.
        supports = {'A': self.config.pick_support, 'B': self.config.place_support,
                    'C': self.config.place_support}
        cfg.pick_support, cfg.place_support = supports[pick], supports[place]
        cfg.ready_joints = (deepcopy(self.config.ready_joints)
                            if index == len(self.route)-1 and side == 'right' else None)
        if side == 'left':
            reference = np.eye(4)
            reference[:3, :3] = Rotation.from_euler('xyz', cfg.grasp_rpy).as_matrix()
            rotation = pose_matrix(self.mirror.pose(matrix_pose(reference)))[:3, :3]
            cfg.grasp_rpy = Rotation.from_matrix(rotation).as_euler('xyz').tolist()
            cfg.approach_direction = self.mirror.point(cfg.approach_direction).tolist()
            if cfg.tcp_offset is not None:
                cfg.tcp_offset = (self.mirror.tool.T @ cfg.tcp_offset).tolist()
        cfg.validate()
        grasp_offset = None
        if side == 'left' and pick == 'B':
            grasp_offset = cfg.left_b_grasp_offset
        elif side == 'left' and pick == 'C':
            grasp_offset = cfg.left_c_grasp_offset
        elif side == 'right' and pick == 'A':
            grasp_offset = cfg.right_a_grasp_offset
        place_support_offset = (cfg.b_support_height_offset
                                if side == 'left' and place == 'B' else 0.0)
        task = FixedTask(self.arms[side], cfg, pick_pose=released,
                         object_id='onero_fixed_bottle_both', grasp_position_offset=grasp_offset,
                         tray_position=self.points['B'],
                         place_support_height_offset=place_support_offset)
        task.yield_after_place = place == 'B'
        task.pick_label, task.place_label = pick, place
        return task

    def standby_target(self, side, initial):
        arm = self.arms[side]
        origin = arm.fk(initial)
        sign = 1. if side == 'left' else -1.
        poses = []
        for back, outward, up in ((.06, .20, .05), (.10, .24, .08), (0., .24, .10)):
            target = shifted(origin, [-back, 0., up])
            target.position.y = sign*(max(sign*origin.position.y, sign*self.points['B'][1])+outward)
            poses.append(target)
        candidates = arm.ik_candidates(poses, initial)
        if candidates:
            return candidates[0][1]
        raise MoveItError(f'{side} 臂没有可行的向外让位姿态')

    def prepare(self, initial, *, first_only=False):
        current, released = dict(initial), None
        self.legs = []
        try:
            self.first.scene.place(self.first.pick_pose, self.config.diameter, self.config.bottle_height)
            self.first.reset_pick()
            print('接力预检查：左臂先向外让位。')
            self.wait_step = Step('左臂先向外让位', 'joints', self.standby_target('left', current))
            self.wait_step.start = self.first.scene.state(current)
            self.wait_step.trajectory = plan_step(self.arms['left'], current, self.wait_step)
            current = endpoint(current, [self.wait_step])
            if first_only:
                # The checked empty-arm yielding motion can start immediately.
                # The first pick is checked after yielding, from its real state.
                self.legs = [self.first]
                return
            for index, (side, pick, place) in enumerate(self.route):
                print(f'接力预检查 [{index+1}/{len(self.route)}] {side}：{pick}→{place}，松爪并撤离。')
                self.current = self.first if index == 0 else self.make_leg(index, released)
                self.legs.append(self.current)
                self.current.prepare(current)
                current = endpoint(current, self.current.steps)
                released = deepcopy(self.current.released_pose)
        finally:
            if self.first.scene.changed:
                self.first.reset_pick()

    def preview(self):
        try:
            trajectory = self.wait_step.trajectory
            self.node.display.publish(DisplayTrajectory(model_id='onero_a1',
                trajectory_start=self.wait_step.start, trajectory=[trajectory]))
            wait_seconds(self.node, seconds(trajectory.joint_trajectory.points[-1].time_from_start)+.2)
            for leg in self.legs:
                leg.reset_pick()
                leg.preview(reset=False)
        finally:
            self.legs[0].reset_pick()

    def execute(self):
        self.executing = True
        gains = {side: GainClient(arm, f'/{side}/onero_topic_bridge') for side, arm in self.arms.items()}
        print('接力执行：左臂先向外让位。')
        left = self.arms['left']
        trajectory = self.wait_step.trajectory
        # This trajectory was freshly planned in this invocation. execute()
        # verifies the measured start state and MoveIt checks the current scene.
        left.show_plan(trajectory)
        left.execute(trajectory)
        released = None
        for index, (side, pick, place) in enumerate(self.route):
            print(f'接力执行 [{index+1}/{len(self.route)}] {side}：{pick}→{place}。')
            self.current = self.legs[0] if index == 0 else self.make_leg(index, released)
            # All planning uses fresh feedback. Later legs use the measured release pose.
            self.current.prepare_grasp(task_state(self.arms[side]))
            self.current.gains, self.current.executing = gains[side], True
            self.current.run_stages(task_state(self.arms[side]), execute=True)
            if self.current.holding or self.current.scene.attached or not self.current.completed:
                raise MoveItError('当前手臂尚未完成松爪撤离，接力中止')
            released = deepcopy(self.current.released_pose)
        self.completed = True


def example(node):
    if node.args.frame != 'world':
        raise MoveItError('双臂接力使用 --frame world')
    preview_settings(node)
    arms = arm_clients(node)
    path = node.args.config or preset_path()
    config = TaskConfig.load(path, recording=node.args.record_orientation)
    if config.arm != 'right':
        raise MoveItError('双臂接力配置以右臂 A/B 点为基准，arm 请设为 right；C 点自动镜像生成')
    if node.args.record_orientation:
        if node.args.execute or node.args.config is None:
            raise MoveItError('记录朝向需要 --config，且不能同时 --execute')
        config.grasp_rpy = Rotation.from_matrix(pose_matrix(arms['right'].fk(task_state(arms['right'])))[:3, :3]).as_euler('xyz').tolist()
        path.write_text(yaml.safe_dump(config.dictionary(), allow_unicode=True, sort_keys=False))
        print('已记录右臂法兰朝向；左臂朝向自动镜像生成。')
        return
    task = RelayTask(node, arms, config)
    node.fixed_task = task
    identities = {side: preview_identity(arm, task.current, announce=side == 'right')
                  for side, arm in arms.items()}
    identity = {'version': 'dual_relay_grasp_offset_v19', 'arms': identities, 'route': task.route, 'points': task.points}
    record = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))/'tasks/fixed_pick_place_both.json'
    if node.args.execute:
        try:
            saved = json.loads(record.read_text())
        except (OSError, ValueError) as exc:
            raise MoveItError('执行前请先完成双臂接力预览') from exc
        # JSON normalizes tuples to lists. Speed and acceleration are deliberately absent.
        if saved != json.loads(json.dumps(identity)):
            raise MoveItError('设备、模型、场景或接力配置已变化，请重新预览')
    print(f'接力位置（world，m）：A={task.points["A"]}，B={task.points["B"]}，C={task.points["C"]}。')
    # Preview checks the entire relay. During execution, check each leg just
    # before use, with fresh feedback and the preceding measured bottle pose.
    task.prepare(task_state(node), first_only=node.args.execute)
    if node.args.execute:
        task.execute()
        print('双臂接力完成，瓶子已放回 A 点，两臂已松爪撤离。' if node.args.round_trip else
              '双臂接力完成，瓶子已放到 C 点，两臂已松爪撤离。')
    else:
        task.preview()
        record.parent.mkdir(parents=True, exist_ok=True)
        temporary = record.with_suffix('.tmp')
        temporary.write_text(json.dumps(identity, ensure_ascii=False, indent=2))
        temporary.replace(record)
        print('预览完成；在 A 点放一个水瓶，添加 --execute 执行。')
