"""双臂单瓶接力：右 A→B，左 B→C→B，右 B→A；每段放下并撤离。"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from moveit_msgs.msg import DisplayTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import serialize_message
from scipy.spatial.transform import Rotation

from onero_a1_common.gains import GainClient
from onero_a1_common.moveit_client import TASK_QUALITY_LIMIT, seconds
from .bottle_scene import BottleScene, PickerGeometry, clear_task_bottles, matrix_pose, pose_matrix
from .fixed_pick_place_config import TaskConfig
from .fixed_task_geometry import bottle_bottom, grasp_poses, tilt_error
from .support import MoveItError, run_example
from .task_support import (Step, confirm_opening, grasp_bottle, plan_step,
                           task_state, wait_for_arm_settled, wait_seconds)


def configure(parser):
    parser.add_argument('--config', type=Path, help='可选，自定义 YAML；省略时使用已配置的 A/B 演示位置')
    parser.add_argument('--record-orientation', action='store_true',
                        help='把当前 Link7 在 world 中的朝向写入配置，不规划、不运动')
    parser.add_argument('--round-trip', dest='round_trip', action='store_true', default=False,
                        help='在 A→B→C 后继续 C→B、B→A；默认只执行 A→B→C')


def shifted(pose, delta):
    result = pose_matrix(pose)
    result[:3, 3] += delta
    return matrix_pose(result)


def preview_settings(node):
    """Fast preview timing; leave planner choice to the shared posture selection.

    RRTConnectOnline deliberately returns one unsimplified path and bypasses
    the posture candidate comparison.  For this task the candidate comparison
    is what avoids a wound wrist, so use the normal RRTConnect path.
    """
    if not node.args.execute and not getattr(node.args, 'record_orientation', False):
        node.args.velocity = max(.5, node.args.velocity)
        node.args.acceleration = max(.5, node.args.acceleration)
        print(f'快速预览：规划速度/加速度缩放 {node.args.velocity:g}/{node.args.acceleration:g}；'
              '执行时按 --velocity/--acceleration 另行规划。')


def preset_path():
    return Path(get_package_share_directory('onero_a1_examples'))/'config'/'fixed_pick_place.yaml'


@dataclass
class Stage(Step):
    event: str = ''


class FixedTask:
    def __init__(self, node, config, pick_pose=None, object_id=None, grasp_position_offset=None,
                 tray_position=None, place_support_height_offset=0.0):
        self.node, self.config = node, config
        self.tray_position = np.array(tray_position if tray_position is not None
                                      else config.place_position, dtype=float)
        node.wrist_roll_limit = config.max_wrist_roll_deviation
        self.pick_label, self.place_label = 'A', 'B'
        self.executing = self.holding = self.completed = False
        self.gains = None
        self.yield_after_place = False
        self.grasp_position_offset = np.array(grasp_position_offset if grasp_position_offset is not None
                                              else [0., 0., 0.], dtype=float)
        rotation = Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
        self.geometry = PickerGeometry(node, config.diameter, rotation)
        self.angle = self.geometry.angle if config.grasp_opening is None else config.grasp_opening*1.1
        if self.angle >= config.open_opening:
            raise MoveItError('张开开度必须大于夹取开度')
        self.offset = self.geometry.center(self.angle) if config.tcp_offset is None else np.array(config.tcp_offset)
        self.pick_pose = deepcopy(pick_pose) if pick_pose is not None else config.object_pose()
        self.grasp, self.pregrasp = grasp_poses(config, self.offset, 0., self.grasp_position_offset)
        self.pregrasp_joints = None
        self.approach_joints = None
        self.cartesian_fallback = None
        self.scene = BottleScene(node, self.geometry, object_id or f'onero_fixed_bottle_{node.arm_side}')
        self.original = self.scene.read()
        if any(item.link_name == node.tip_link for item in self.original.robot_state.attached_collision_objects):
            raise MoveItError('所选手臂已有附着物体，请先处理上次任务的持物状态')
        self.supports, self.support_heights = [], []
        for index, (label, position, support_id) in enumerate((
                ('取料点 A', config.pick_position, config.pick_support),
                ('放料点 B', config.place_position, config.place_support))):
            height = self.scene.support_height(position, config.diameter/2., object_id=support_id)
            if (config.c_position is not None
                    and np.linalg.norm(np.array(position)-np.array(config.c_position)) < .03):
                height += config.c_support_height_offset
            if index == 1 and place_support_height_offset:
                height += place_support_height_offset
            tolerance = config.position_tolerance if pick_pose is not None and index == 0 else .001
            if height is None or abs(position[2]-config.bottle_height/2.-height) > tolerance+1e-9:
                raise MoveItError(f'{label}瓶底不在指定或已加载的水平支撑面上，请修正配置或场景')
            self.supports.append(self.scene.support_id)
            self.support_heights.append(height)
        for name in ('diameter', 'grasp_opening', 'orientation_tolerance'):
            setattr(node.args, name, getattr(config, name))
        self.steps = []

    def reset_pick(self):
        self.scene.world(self.pick_pose)
        self.scene.contacts(supports=[self.supports[0]])

    def stages(self):
        cfg = self.config
        return [Stage('张开夹爪', 'gripper', min(1., cfg.open_opening)),
                Stage(f'连续接近 {self.pick_label} 点抓取位', 'continuous', None, event='approach'),
                Stage('夹取并确认接触', 'gripper', self.angle, event='grasp'),
                Stage(f'连续离桌、搬运并直立放到 {self.place_label} 点', 'continuous', None, event='carry'),
                Stage('确认放置位置并松爪', 'gripper', min(1., cfg.open_opening), event='release'),
                Stage(f'连续撤离 {self.place_label} 点并让位', 'continuous', None, event='retreat')]

    def plan_motion(self, start, event):
        node, cfg, scene = self.node, self.config, self.scene
        if event == 'approach':
            if self.approach_joints is not None:
                # Fallback: the straight Cartesian approach stayed on a
                # near-singular/wound branch, so approach the grasp point in
                # joint space from the best-conditioned IK branch instead.
                try:
                    return node.plan_continuous(start, [('joints', self.approach_joints)])
                except MoveItError as exc:
                    if self.cartesian_fallback is None:
                        raise
                    print(f'关节空间接近不可用（{exc}），回退到直线接近。')
                    self.pregrasp_joints, self.grasp, self.pregrasp = self.cartesian_fallback
                    self.cartesian_fallback = None
                    self.approach_joints = None
            return node.plan_continuous(start, [('joints', self.pregrasp_joints), ('cartesian', self.grasp)])
        if event == 'retreat':
            tip = node.fk(start)
            direction = (pose_matrix(tip)[:3, :3] @ pose_matrix(self.grasp)[:3, :3].T
                         @ np.array(cfg.approach_direction))
            back = -direction*cfg.retreat_distance
            back[2] = 0.
            lift = lambda state: shifted(node.fk(state), [0., 0., cfg.retreat_distance])
            if self.yield_after_place:
                # Release in the B tray: back straight off the bottle, then
                # move to this arm's outward yield pose.
                goals = [('cartesian', lambda state: shifted(node.fk(state), back))]
                def outward_goal(state):
                    point = node.fk(state).position
                    sign = 1. if node.arm_side == 'left' else -1.
                    clearance = float(np.linalg.norm(self.offset))+cfg.diameter/2.+cfg.retreat_distance
                    # Only request clearance on this arm's side of the bottle.
                    # Keep X/Z local to the reached retreat, not the startup pose.
                    y = sign*max(sign*point.y+.12, sign*cfg.place_position[1]+clearance)
                    return ([point.x, y, point.z], (0., 0., 0.))
                goals.append(('position', outward_goal))
            elif cfg.ready_joints is not None:
                goals = [('cartesian', lift)]
                goals.append(('joints', dict(zip(node.joints, cfg.ready_joints))))
            else:
                # On the plate/table, lift clear first. The previous straight
                # back path can scrape the plate and stop partway.
                goals = [('cartesian', lift)]
            return node.plan_continuous(start, goals)

        def placement_contacts(enabled):
            # The C target may be intentionally lowered below the modelled plate
            # to compensate a measured physical offset; keep that contact allowed.
            supports = [self.supports[1]]
            if enabled and self.config.c_position is not None:
                supports.extend(['onero_env_dark_plate', 'onero_env_table'])
            scene.contacts(gripper=True, supports=supports if enabled else ())

        goals = [('cartesian', lambda state: shifted(node.fk(state), [0., 0., cfg.lift_distance]))]
        pick_in_tray = np.linalg.norm(np.array(cfg.pick_position[:2])-self.tray_position[:2]) < .03
        place_in_tray = np.linalg.norm(np.array(cfg.place_position[:2])-self.tray_position[:2]) < .03
        if place_in_tray or pick_in_tray:
            # 托盘有 3 cm 边框：放到托盘时在 B 上方中转；从托盘取出时在
            # 目标点上方中转，让瓶身先离开边框再走横向。
            transfer = self.tray_position if place_in_tray else np.array(cfg.place_position)
            goals.append(('upright_position',
                          ([transfer[0], transfer[1], cfg.transfer_height], scene.tip_to_object)))
        goals.append(('upright_position_contact',
                      (cfg.place_position, scene.tip_to_object, placement_contacts)))

        def contacts(index, state):
            supports = [self.supports[0]] if index == 0 else []
            if self.config.c_position is not None and index == 0:
                near_c = (np.linalg.norm(np.array(cfg.pick_position[:2])-np.array(cfg.c_position[:2])) < .03)
                if near_c:
                    supports.extend(['onero_env_dark_plate', 'onero_env_table'])
            scene.contacts(gripper=True, supports=supports)

        previous = None
        def check(state):
            nonlocal previous
            obj = self.object_at(state)
            angle = tilt_error(obj[:3, :3])
            if angle > cfg.orientation_tolerance:
                raise MoveItError(f'持物轨迹瓶身倾斜 {angle:.4f} rad，超过全程直立限值 '
                                  f'{cfg.orientation_tolerance:g} rad')
            near = [(position, support) for position, support in zip(
                (cfg.pick_position, cfg.place_position), self.supports)
                if np.linalg.norm(obj[:2, 3]-position[:2]) <= .025
                and obj[2, 3] <= position[2]+cfg.lift_distance+.002]
            supports = tuple(support for _, support in near)
            if 'onero_env_dark_plate' in supports:
                # C/B may intentionally go below the modelled plate to
                # compensate a measured offset; the fixed table is below it.
                supports = supports + ('onero_env_table',)
            if supports != previous:
                scene.contacts(gripper=True, supports=supports)
                previous = supports
            # Support contact permissions must not hide a bottle/table intersection
            # introduced by MoveIt's joint-space corner blending.
            bottom = bottle_bottom(obj, cfg.bottle_height, cfg.diameter)
            if any(bottom < position[2]-cfg.bottle_height/2.-.002 for position, _ in near):
                raise MoveItError('MoveIt 连续搬运路径使瓶身穿过支撑面')

        try:
            return node.plan_continuous(start, goals, before=contacts, check=check,
                path_constraints=node.upright_constraints(scene.tip_to_object, cfg.orientation_tolerance))
        finally:
            # This single motion includes contact at both ends. The validation
            # callback above permits support contact only near the endpoints.
            scene.contacts(gripper=True, supports=self.supports)

    def object_at(self, state):
        return pose_matrix(self.node.fk(state)) @ self.scene.tip_to_object

    def report_grasp(self, state):
        transform = pose_matrix(self.node.fk(state))
        center = transform[:3, 3]+transform[:3, :3] @ self.offset
        target = np.array(self.config.pick_position)+[0., 0., self.config.grasp_height]+self.grasp_position_offset
        delta = center-target
        joint_error = {name: state[name]-self._grasp_goal[name] for name in self.node.joints}
        worst = max(joint_error, key=lambda name: abs(joint_error[name]))
        print(f'夹取中心反馈（模型 FK，按预期接触开度）：'
              f'XYZ 偏差 [{delta[0]*1000:+.1f}, {delta[1]*1000:+.1f}, {delta[2]*1000:+.1f}] mm；'
              f'最大关节偏差 {worst}={joint_error[worst]:+.5f} rad。')

    def placement_error(self, state):
        actual = self.object_at(state)
        delta = actual[:3, 3]-self.config.place_position
        tilt = tilt_error(actual[:3, :3])
        gap = bottle_bottom(actual, self.config.bottle_height, self.config.diameter)-self.support_heights[1]
        c_place = self.place_label == 'C'
        position_tolerance = (self.config.c_position_tolerance if c_place
                              else self.config.position_tolerance)
        height_tolerance = (self.config.c_placement_height_tolerance if c_place
                            else self.config.b_placement_height_tolerance
                            if self.place_label == 'B'
                            else self.config.placement_height_tolerance)
        valid = (np.linalg.norm(delta) <= position_tolerance
                 and tilt <= self.config.orientation_tolerance
                 and abs(gap) <= height_tolerance)
        return actual, delta, tilt, float(gap), bool(valid)

    def check_place(self, state, *, label='放置反馈'):
        actual, delta, tilt, gap, valid = self.placement_error(state)
        detail = (f'位置误差 {np.linalg.norm(delta)*1000:.1f} mm，'
                  f'XYZ 偏差 [{delta[0]*1000:+.1f}, {delta[1]*1000:+.1f}, {delta[2]*1000:+.1f}] mm，'
                  f'瓶底距支撑面 {gap*1000:+.1f} mm，倾斜 {tilt:.4f} rad')
        if not valid:
            raise MoveItError(f'{label}超差（模型 FK）：{detail}；保持夹持，不松爪')
        print(f'{label}（模型 FK）：{detail}。')
        return matrix_pose(actual)

    def record_placement(self, current, *, settled):
        actual, delta, tilt, gap, valid = self.placement_error(current)
        goal = self._place_goal
        joint_error = {name: float(current[name]-goal[name]) for name in self.node.joints}
        worst = max(joint_error, key=lambda name: abs(joint_error[name]))
        print(f'放置关节跟踪：最大偏差 {worst}={joint_error[worst]:+.5f} rad。')
        report = {'arm': self.node.arm_side, 'target': self.place_label, 'settled': settled,
                  'within_task_tolerance': valid, 'position_error_xyz_m': delta.tolist(),
                  'bottle_bottom_gap_m': gap, 'tilt_rad': tilt,
                  'support_height_m': self.support_heights[1],
                  'planned_object_pose': self.object_at(goal).tolist(), 'actual_object_pose': actual.tolist(),
                  'goal_joints': {name: float(goal[name]) for name in self.node.joints},
                  'actual_joints': {name: float(current[name]) for name in self.node.joints},
                  'joint_error_rad': joint_error}
        root = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))/'tasks'
        filename = root/f'fixed_pick_place_{self.node.arm_side}_placement.json'
        try:
            root.mkdir(parents=True, exist_ok=True)
            filename.write_text(json.dumps(report, ensure_ascii=False, indent=2))
            print(f'放置记录：{filename}')
        except OSError as exc:
            self.node.get_logger().warning(f'放置记录保存失败：{exc}')

    def correct_placement(self, current):
        """Closed-loop correction of the modeled held pose before release.

        The JTC has a small steady-state joint error under load.  Commanding
        the same endpoint again cannot remove it; command the measured error
        in the opposite direction and re-check the fresh feedback.
        """
        actual, delta, tilt, gap, valid = self.placement_error(current)
        if valid:
            return current
        target = np.array(self.config.place_position)-delta
        print(f'放置闭环修正：补偿 XYZ [{(-delta[0])*1000:+.1f}, {(-delta[1])*1000:+.1f}, '
              f'{(-delta[2])*1000:+.1f}] mm；当前倾斜 {tilt:.4f} rad，瓶底间隙 {gap*1000:+.1f} mm。')
        # The real endpoint can be slightly below the support before correction.
        # Allow the bottle/table support contact so the correction can lift out
        # instead of rejecting the colliding start state.
        self.scene.contacts(gripper=True, supports=[self.supports[1], 'onero_env_table'])
        previous = getattr(self.node, '_motion_path_constraints', None)
        previous_strict = getattr(self.node, '_strict_margin', False)
        self.node._motion_path_constraints = self.node.upright_constraints(
            self.scene.tip_to_object, self.config.orientation_tolerance)
        self.node._strict_margin = True
        try:
            trajectory = self.node.plan_upright_position(current, target, self.scene.tip_to_object)
            self.node.show_plan(trajectory)
            self.node.execute(trajectory)
            wait_for_arm_settled(self.node)
        except MoveItError as exc:
            raise MoveItError(f'放置闭环修正失败：{exc}') from exc
        finally:
            self.node._strict_margin = previous_strict
            self.node._motion_path_constraints = previous
        return {name: value for name, value in task_state(self.node).items()
                if name not in self.geometry.mimic_names}

    def wait_for_placement(self, current, timeout=2.):
        """Observe the held endpoint, then correct once or twice if needed.

        JTC continues commanding the original endpoint after action success,
        so a steady-state offset is expected.  Corrections command the
        measured error in the opposite direction and are only accepted after
        fresh feedback passes the same placement checks.
        """
        node = self.node
        for attempt in range(3):
            deadline, since = time.monotonic()+timeout, None
            while time.monotonic() < deadline:
                current.update(node.state(max_age=.1,
                                          timeout=max(.001, deadline-time.monotonic())))
                if self.placement_error(current)[-1]:
                    since = time.monotonic() if since is None else since
                    if time.monotonic()-since >= .15:
                        self.record_placement(current, settled=True)
                        self.check_place(current)
                        return current
                else:
                    since = None
            if attempt < 2:
                print('放置反馈仍超差，尝试闭环修正。')
                try:
                    current = self.correct_placement(current)
                except MoveItError as exc:
                    print(f'放置闭环修正未完成：{exc}')
                    break
        self.record_placement(current, settled=False)
        self.check_place(current)
        raise MoveItError('放置反馈尚未连续稳定，保持夹持，不松爪')

    def run_stages(self, initial, *, execute=False):
        node, cfg, scene = self.node, self.config, self.scene
        current = {name: value for name, value in initial.items() if name not in self.geometry.mimic_names}
        steps = self.stages()
        for index, step in enumerate(steps, 1):
            print(f'{"执行" if execute else "预检查"} [{index}/{len(steps)}] {step.label}')
            try:
                if execute:
                    current = {name: value for name, value in task_state(node).items()
                               if name not in self.geometry.mimic_names}
                if step.event == 'approach':
                    # Keep the open fingers collision-checked during approach.
                    scene.contacts(supports=[self.supports[0]])
                elif step.event == 'grasp':
                    if execute:
                        self.report_grasp(current)
                    scene.contacts(gripper=True, supports=[self.supports[0]])
                elif step.event == 'release':
                    if execute:
                        current = self.wait_for_placement(current)
                    else:
                        self.check_place(current)
                elif step.event == 'retreat':
                    # Keep the open gripper allowed to touch the just-released
                    # bottle while lifting clear of it.
                    scene.contacts(gripper=True, supports=[self.supports[1]])
                if step.kind == 'continuous':
                    step.target = lambda state, event=step.event: self.plan_motion(state, event)
                step.start = scene.state(current)
                began = time.monotonic()
                step.trajectory = plan_step(node, current, step)
                print(f'轨迹准备耗时 {time.monotonic()-began:.2f} s。')
                if step.event in ('approach', 'carry'):
                    path = step.trajectory.joint_trajectory
                    goal = {**current, **dict(zip(path.joint_names, path.points[-1].positions))}
                    if step.event == 'approach':
                        self._grasp_goal = goal
                    else:
                        self._place_goal = goal
                        self.check_place(self._place_goal, label='规划放置终点')
                if execute:
                    node.show_plan(step.trajectory)
                    if step.event == 'grasp':
                        # Even a failed close may leave an object gripped: keep its scene on exit.
                        self.holding = True
                    if step.event == 'grasp' and not node.bottle_simulated:
                        grasp_bottle(node, self.geometry)
                    else:
                        skip_open = False
                        if (execute and index == 1 and step.kind == 'gripper'
                                and step.target is not None):
                            actual = current.get(node.gripper_joint)
                            if actual is not None and abs(actual-step.target) <= .012:
                                print(f'夹爪已在张开位（{actual:.4f}），跳过张开执行。')
                                skip_open = True
                        if not skip_open:
                            try:
                                node.execute(step.trajectory, controller='gripper_controller' if step.kind == 'gripper' else None)
                            except MoveItError:
                                if execute and index == 1 and step.kind == 'gripper':
                                    print('张爪执行失败，重试一次。')
                                    node.execute(step.trajectory, controller='gripper_controller')
                                else:
                                    raise
                            if step.kind == 'gripper':
                                confirm_opening(node, step.target)
                    current = task_state(node)
                else:
                    path = step.trajectory.joint_trajectory
                    current.update(zip(path.joint_names, path.points[-1].positions))
                if step.event == 'grasp':
                    scene.attach(current)
                    if execute:
                        self.gains.set_kp(cfg.carry_kp)
                        print(f'夹取后位置增益已生效：kp={cfg.carry_kp:g}。')
                        wait_for_arm_settled(node)
                elif step.event == 'release':
                    released = matrix_pose(self.object_at(current))
                    self.released_pose = deepcopy(released)
                    if execute:
                        self.holding = False
                    scene.world(released)
                    scene.contacts(gripper=True, supports=[self.supports[1]])
                    if execute:
                        self.gains.restore()
                        print(f'已恢复执行前位置增益：kp={self.gains.original_kp:g}。')
                        wait_for_arm_settled(node)
                elif step.event == 'retreat':
                    # Keep the open gripper allowed to touch the just-released
                    # bottle while lifting clear of it.
                    scene.contacts(gripper=True, supports=[self.supports[1]])
                    if not node.validity(current).valid:
                        raise MoveItError('撤离后仍与物体或环境碰撞')
            except (Exception, KeyboardInterrupt) as exc:
                raise MoveItError(f'阶段「{step.label}」停止，后续动作未执行：{str(exc) or "interrupted"}') from exc
        if execute:
            self.completed = True
        return steps

    def prepare_grasp(self, initial):
        """Set up the real pickup without virtually planning the entire leg twice."""
        node, cfg = self.node, self.config
        initial = {name: value for name, value in initial.items() if name not in self.geometry.mimic_names}
        self.scene.place(self.pick_pose, cfg.diameter, cfg.bottle_height)
        self.scene.contacts(supports=[self.supports[0]])
        if not node.validity(initial).valid:
            raise MoveItError('取料点与当前机器人状态碰撞')
        seed = {**initial, node.gripper_joint: min(1., cfg.open_opening)}
        if np.any(self.grasp_position_offset):
            print(f'{node.arm_side} {self.pick_label} 点夹取修正（world，mm）：'
                  f'{(self.grasp_position_offset*1000).tolist()}。')
        # A cylindrical bottle can be approached from any world yaw.  The
        # wider sweep is collision-filtered and gives the solver a chance to
        # leave an extended or wound wrist branch.
        degrees = (0., -15., 15., -30., 30., -60., 60.)
        poses = [grasp_poses(cfg, self.offset, math.radians(degree), self.grasp_position_offset)
                 for degree in degrees]
        candidates = node.ik_candidates([pregrasp for _, pregrasp in poses], seed,
                                        extra_seeds=(node.reference_seed(),))
        if not candidates:
            raise MoveItError('预抓取位没有可行的自然构型')
        best, failures, best_fraction = None, [], 0.
        for index, joints in candidates:
            grasp, pregrasp = poses[index]
            approach_start = {**seed, **joints}
            response = node.cartesian_path(approach_start, [grasp])
            if math.isfinite(response.fraction):
                best_fraction = max(best_fraction, response.fraction)
            if not (response.error_code.val == 1 and math.isfinite(response.fraction)
                    and response.fraction >= 1.-1e-6):
                continue
            path = response.solution.joint_trajectory
            endpoint = {**approach_start, **dict(zip(path.joint_names, path.points[-1].positions))}
            quality, condition, margin, wrist = node.grasp_quality(endpoint)
            try:
                path_cost = node.motion_posture_cost(approach_start, response.solution)
            except MoveItError as exc:
                failures.append(str(exc))
                continue
            score = path_cost+quality
            if best is None or score < best[0]:
                best = (score, joints, grasp, pregrasp, quality, condition, margin, wrist)
        if best is not None:
            score, joints, grasp, pregrasp, quality, condition, margin, wrist = best
            if quality <= TASK_QUALITY_LIMIT:
                self.pregrasp_joints = joints
                self.approach_joints = None
                self.cartesian_fallback = None
                self.grasp, self.pregrasp = grasp, pregrasp
                print(f'抓取候选：质量={quality:.2f}，条件数={condition:.1f}，'
                      f'关节余量={margin:.3f} rad，腕部偏差={wrist:.3f} rad。')
                return
            print(f'抓取候选末端条件偏弱（质量 {quality:.2f}），尝试关节空间接近。')
        # The straight-line approach can be valid but end on a wound branch
        # that the local Cartesian IK cannot leave.  Plan to a well-conditioned
        # grasp IK branch in joint space; MoveIt still checks the whole path.
        grasp_candidates = node.ik_candidates([grasp for grasp, _ in poses], seed,
                                              extra_seeds=(node.reference_seed(),))
        for index, joints in grasp_candidates:
            quality, condition, margin, wrist = node.grasp_quality(joints)
            if quality > TASK_QUALITY_LIMIT:
                continue
            self.grasp, self.pregrasp = poses[index]
            self.approach_joints = joints
            self.pregrasp_joints = None
            self.cartesian_fallback = ((best[1], best[2], best[3])
                                       if best is not None else None)
            print(f'改用关节空间接近：质量={quality:.2f}，条件数={condition:.1f}，'
                  f'关节余量={margin:.3f} rad，腕部偏差={wrist:.3f} rad。')
            return
        if best is not None:
            # Keep the posture-checked Cartesian result rather than failing
            # outright; the caller still applies the hard 75° wrist limit.
            score, joints, grasp, pregrasp, quality, condition, margin, wrist = best
            self.pregrasp_joints = joints
            self.approach_joints = None
            self.cartesian_fallback = None
            self.grasp, self.pregrasp = grasp, pregrasp
            print(f'警告：仍使用最佳笛卡尔接近，但末端质量 {quality:.2f} 偏弱。')
            return
        if failures:
            detail = f'；最后失败：{failures[-1]}'
        elif best_fraction > 0.:
            detail = f'；笛卡尔路径不完整（最高完成 {best_fraction:.1%}）'
        else:
            detail = ''
        raise MoveItError('自然构型候选没有可行的短距离抓取接近路径'+detail)

    def prepare(self, initial):
        self.prepare_grasp(initial)
        try:
            self.steps = self.run_stages(initial)
        finally:
            self.reset_pick()

    def preview(self, *, reset=True):
        print(f'完整预览：{self.pick_label} 点夹取 → {self.place_label} 点放置 → 撤离；不驱动硬件。')
        try:
            for step in self.steps:
                if step.event == 'carry':
                    self.scene.hide_world()
                elif step.event == 'retreat':
                    self.scene.world(self.released_pose)
                trajectory = step.trajectory
                self.node.display.publish(DisplayTrajectory(model_id='onero_a1', trajectory_start=step.start,
                                                            trajectory=[trajectory]))
                wait_seconds(self.node, seconds(trajectory.joint_trajectory.points[-1].time_from_start)+.2)
        finally:
            if reset:
                self.reset_pick()


class RoundTripTask:
    """Compose two complete pick/place legs, with a real release between them."""
    def __init__(self, node, config):
        self.node, self.config = node, config
        outbound = deepcopy(config)
        outbound.ready_joints = None  # Return to ready only after the bottle is back at A.
        self.first = self.current = FixedTask(node, outbound, tray_position=self.config.place_position)
        self.original = self.first.original
        self.executing = self.completed = False
        self.gains = None
        self.return_plan = None

    @property
    def scene(self):
        return self.current.scene

    @property
    def holding(self):
        return self.current.holding

    def return_task(self, released):
        cfg = deepcopy(self.config)
        cfg.pick_position = [released.position.x, released.position.y, released.position.z]
        cfg.place_position = list(self.config.pick_position)
        cfg.pick_support, cfg.place_support = self.config.place_support, self.config.pick_support
        cfg.validate()
        task = FixedTask(self.node, cfg, pick_pose=released,
                         tray_position=self.config.place_position)
        task.pick_label, task.place_label = 'B', 'A'
        return task

    def prepare(self, initial):
        print('往返预检查：A→B 放下并退开，再 B→A 重新抓取并放回。')
        try:
            self.first.prepare(initial)
            end = dict(initial)
            for step in self.first.steps:
                path = step.trajectory.joint_trajectory
                end.update(zip(path.joint_names, path.points[-1].positions))
            print('返程预检查：B→A。')
            self.return_plan = self.return_task(self.first.released_pose)
            self.return_plan.prepare(end)
        finally:
            if self.first.scene.changed:
                self.first.reset_pick()

    def preview(self):
        try:
            print('往返预览：A→B。')
            self.first.preview(reset=False)
            print('往返预览：B→A。')
            self.return_plan.reset_pick()
            self.return_plan.preview(reset=False)
        finally:
            self.first.reset_pick()

    def run_stages(self, initial, *, execute=False):
        if not execute:
            self.prepare(initial)
            return self.first.steps+self.return_plan.steps
        self.executing = True
        print('去程执行：A→B，松爪后退开。')
        self.current = self.first
        self.current.executing, self.current.gains = True, self.gains
        outbound = self.current.run_stages(initial, execute=True)
        if self.current.holding or self.current.scene.attached:
            raise MoveItError('去程未确认释放水瓶，返程未开始')
        # Regrasp the object where the measured release left it, rather than
        # teleporting its scene model to the nominal B coordinate.
        print('B 点已松爪并退开；按本次放置反馈重新规划返程 B→A。')
        self.current = self.return_task(deepcopy(self.first.scene.bottle.pose))
        current = task_state(self.node)
        self.current.prepare_grasp(current)
        self.current.executing, self.current.gains = True, self.gains
        inbound = self.current.run_stages(task_state(self.node), execute=True)
        self.completed = True
        return outbound+inbound


def preview_identity(node, task, *, announce=True):
    bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
    values = node.call(GetParameters, bridge+'/get_parameters',
                      GetParameters.Request(names=['simulate', 'model', 'can_channel', 'mounting'])).values
    if len(values) != 4 or values[0].type != 1 or any(value.type != 4 for value in values[1:]):
        raise MoveItError('无法确认模拟模式与设备配置')
    node.bottle_simulated = values[0].bool_value
    if announce:
        print('运行模式：模拟。' if node.bottle_simulated else '运行模式：真机，请按配置在 A 点摆放实体水瓶。')
    models = [node.parameter('/move_group', name) for name in ('robot_description', 'robot_description_semantic')]
    objects = []
    for obj in sorted(task.original.world.collision_objects, key=lambda obj: obj.id):
        if obj.id == task.scene.id:
            continue
        obj = deepcopy(obj)
        obj.header.stamp = Time()
        objects.append(hashlib.sha256(serialize_message(obj)).hexdigest())
    identity = {'version': 56, 'round_trip': getattr(node.args, 'round_trip', False),
                'config': task.config.dictionary(), 'arm': node.arm_side,
                'joints': node.joints, 'simulate': node.bottle_simulated,
                'device': [value.string_value for value in values[1:]],
                'model': hashlib.sha256('\n'.join(models).encode()).hexdigest(), 'scene': objects}
    return identity


def cleanup(node):
    from .dual_task_support import close_arm_clients
    task = getattr(node, 'fixed_task', None)
    try:
        close_arm_clients(node)
    finally:
        clear_task_bottles(node)
    if task is not None:
        task.scene.attached = False
        if task.holding:
            print('虚拟模型已清理；手臂仍保持夹持和当前增益，请确认实物状态。')


def example(node):
    if node.arm_side == 'both':
        from .fixed_pick_place_relay import example as relay_example
        return relay_example(node)
    if node.args.frame != 'world':
        raise MoveItError('固定抓放仅支持 --frame world')
    if node.args.record_orientation and node.args.execute:
        raise MoveItError('--record-orientation 不能与 --execute 一起使用')
    if node.args.config is None and node.args.record_orientation:
        raise MoveItError('默认演示已设置抓取朝向，直接运行即可；自定义记录朝向时请指定 --config')
    preview_settings(node)
    path = node.args.config or preset_path()
    initial = task_state(node)
    config = TaskConfig.load(path, recording=node.args.record_orientation)
    if config.arm != node.arm_side:
        raise MoveItError(f'配置用于 {config.arm} 臂，当前选择 {node.arm_side}；请核对 --arm 和配置')
    if node.args.record_orientation:
        config.grasp_rpy = Rotation.from_matrix(pose_matrix(node.fk(initial))[:3, :3]).as_euler('xyz').tolist()
        path.write_text(yaml.safe_dump(config.dictionary(), allow_unicode=True, sort_keys=False))
        print(f'已记录 Link7 在 world 中的固定朝向：{config.grasp_rpy} rad；未驱动硬件。')
        return
    task_type = RoundTripTask if getattr(node.args, 'round_trip', False) else FixedTask
    task = task_type(node, config)
    node.fixed_task = task
    identity = preview_identity(node, task)
    root = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
    record = root/'tasks'/f'fixed_pick_place_{node.arm_side}.json'
    if node.args.execute:
        try:
            saved = json.loads(record.read_text())
        except (OSError, ValueError) as exc:
            raise MoveItError('执行前请先使用相同配置完成预览') from exc
        # Older records included scaling factors; ignore them for compatibility.
        speed_fields = {'velocity', 'acceleration'}
        if (not isinstance(saved, dict)
                or {key: value for key, value in saved.items() if key not in speed_fields}
                != {key: value for key, value in identity.items() if key not in speed_fields}):
            raise MoveItError('配置、设备、模型、场景、往返模式或任务版本与预览不一致，请重新预览')
    if node.args.execute:
        (task.first if isinstance(task, RoundTripTask) else task).prepare_grasp(initial)
    else:
        task.prepare(initial)
    print(f'固定取料中心 A={config.pick_position}，放料中心 B={config.place_position} m；'
          f'水平搬运距离={math.dist(config.pick_position[:2], config.place_position[:2])*100:.1f} cm。')
    print('搬运目标：离桌抬升 → 以最终瓶位的 5D 直立目标一次规划；'
          '只要求瓶身全程直立，绕竖直轴转向和路径形状自由。')
    if node.args.execute:
        bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
        task.gains = GainClient(node, bridge)
        task.executing = True
        task.run_stages(task_state(node), execute=True)
        print('往返抓放完成：水瓶已从 A 点搬到 B 点，再重新夹取搬回 A 点，已松爪并撤离。'
              if getattr(node.args, 'round_trip', False) else
              '固定抓放完成：水瓶已从 A 点放到 B 点，夹爪已松开并撤离。')
    else:
        task.preview()
        record.parent.mkdir(parents=True, exist_ok=True)
        temporary = record.with_suffix('.tmp')
        temporary.write_text(json.dumps(identity, ensure_ascii=False, indent=2))
        temporary.replace(record)
        print(f'预览记录已保存：{record}；按配置摆放实物后添加 --execute，速度和加速度可另行设置。')


def main(args=None):
    return run_example('fixed_pick_place', __doc__, configure, example, args,
                       planning_pipelines=(), dual=True, cleanup=cleanup, default_planning_time=10.)
