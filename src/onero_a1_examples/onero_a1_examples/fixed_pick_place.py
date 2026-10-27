"""固定位置抓放：默认 A→B 放下并退开，再重新夹取搬回 A；--one-way 仅单程。"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import serialize_message
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

from onero_a1_common.gains import GainClient
from onero_a1_common.trajectory_tools import _curve, _duration, _scaled_limits
from .bottle_scene import BottleScene, PickerGeometry, clear_task_bottles, matrix_pose, pose_matrix
from .fixed_pick_place_config import TaskConfig
from .fixed_task_geometry import carrying_waypoints, grasp_poses, tilt_error
from .support import MoveItError, run_example
from .task_support import (Step, confirm_opening, grasp_bottle, plan_step, preview_segment,
                           task_state, wait_for_arm_settled, wait_seconds)


def configure(parser):
    parser.add_argument('--config', type=Path, help='可选，自定义 YAML；省略时使用已配置的 A/B 演示位置')
    parser.add_argument('--record-orientation', action='store_true',
                        help='把当前 Link7 在 world 中的朝向写入配置，不规划、不运动')
    parser.add_argument('--one-way', dest='round_trip', action='store_false', default=True,
                        help='仅执行 A→B；默认放下并退开后，再夹取 B 点水瓶搬回 A 点')


def shifted(pose, delta):
    result = pose_matrix(pose)
    result[:3, 3] += delta
    return matrix_pose(result)


def preset_path():
    return Path(get_package_share_directory('onero_a1_examples'))/'config'/'fixed_pick_place.yaml'


@dataclass
class Stage(Step):
    event: str = ''


class FixedTask:
    def __init__(self, node, config, pick_pose=None):
        self.node, self.config = node, config
        self.pick_label, self.place_label = 'A', 'B'
        self.executing = self.holding = self.completed = False
        self.gains = None
        rotation = Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
        self.geometry = PickerGeometry(node, config.diameter, rotation)
        self.angle = self.geometry.angle if config.grasp_opening is None else config.grasp_opening*1.1
        if self.angle >= config.open_opening*1.1:
            raise MoveItError('张开开度必须大于夹取开度')
        self.offset = self.geometry.center(self.angle) if config.tcp_offset is None else np.array(config.tcp_offset)
        self.pick_pose = deepcopy(pick_pose) if pick_pose is not None else config.object_pose()
        self.grasp = config.grasp_pose(self.offset)
        self.pregrasp_joints = None
        self.pregrasp = shifted(self.grasp, -np.array(config.approach_direction)*config.approach_distance)
        self.scene = BottleScene(node, self.geometry, f'onero_fixed_bottle_{node.arm_side}')
        self.original = self.scene.read()
        if any(item.link_name == node.tip_link for item in self.original.robot_state.attached_collision_objects):
            raise MoveItError('所选手臂已有附着物体，请先处理上次任务的持物状态')
        self.supports = []
        for index, (label, position, support_id) in enumerate((
                ('取料点 A', config.pick_position, config.pick_support),
                ('放料点 B', config.place_position, config.place_support))):
            height = self.scene.support_height(position, config.diameter/2., object_id=support_id)
            tolerance = config.position_tolerance if pick_pose is not None and index == 0 else .001
            if height is None or abs(position[2]-config.bottle_height/2.-height) > tolerance+1e-9:
                raise MoveItError(f'{label}瓶底不在指定或已加载的水平支撑面上，请修正配置或场景')
            self.supports.append(self.scene.support_id)
        for name in ('diameter', 'grasp_opening', 'orientation_tolerance'):
            setattr(node.args, name, getattr(config, name))
        self.steps = []

    def reset_pick(self):
        self.scene.world(self.pick_pose)
        self.scene.contacts(supports=[self.supports[0]])

    def stages(self):
        cfg = self.config
        stages = [Stage('张开夹爪', 'gripper', cfg.open_opening*1.1),
                  Stage('到达固定预抓取位', 'joints', self.pregrasp_joints),
                  Stage(f'直线接近 {self.pick_label} 点', 'cartesian', self.grasp, event='approach'),
                  Stage('夹取并确认接触', 'gripper', self.angle, event='grasp'),
                  Stage(f'连续抬升、搬运并下降到 {self.place_label} 点', 'carry', None),
                  Stage('确认放置位置并松爪', 'gripper', cfg.open_opening*1.1, event='release'),
                  Stage('松爪后竖直上抬', 'cartesian', None, event='raise_empty'),
                  Stage(f'高位撤离 {self.place_label} 点', 'cartesian', None, event='retreat')]
        if cfg.ready_joints is not None:
            stages.append(Stage('返回配置待机位', 'joints', dict(zip(self.node.joints, cfg.ready_joints))))
        return stages

    def object_at(self, state):
        return pose_matrix(self.node.fk(state)) @ self.scene.tip_to_object

    def check_place(self, state):
        actual = self.object_at(state)
        position_error = float(np.linalg.norm(actual[:3, 3]-self.config.place_position))
        angle_error = tilt_error(actual[:3, :3])
        if (position_error > self.config.position_tolerance
                or angle_error > self.config.orientation_tolerance):
            raise MoveItError(f'放置反馈超差：物体位置误差 {position_error*1000:.1f} mm，'
                              f'倾斜误差 {angle_error:.4f} rad；保持夹持，不松爪')
        print(f'放置反馈（模型 FK）：位置误差 {position_error*1000:.1f} mm，倾斜误差 {angle_error:.4f} rad。')
        return matrix_pose(actual)

    def plan_transfer(self, start):
        # Prefer a small turn toward the destination; retain the grasp heading
        # as a fallback. The approach direction itself stays fixed.
        tip_rotation = pose_matrix(self.node.fk(start))[:3, :3]
        reference = Rotation.from_euler('xyz', self.config.grasp_rpy).as_matrix()
        forward = tip_rotation @ reference.T @ np.array(self.config.approach_direction)
        link = self.node.arm_side+'_base_link' if self.node.dual else 'base_link'
        base = pose_matrix(self.node.fk(start, link=link))[:3, 3]
        radial = np.array(self.config.place_position)-base
        angle = math.atan2(radial[1], radial[0])-math.atan2(forward[1], forward[0])
        angle = math.atan2(math.sin(angle), math.cos(angle))
        choices = [angle, 0.] if abs(angle) <= math.pi/2 else [0.]
        errors = []
        for yaw in dict.fromkeys(choices):
            try:
                return self.plan_carry_yaw(start, yaw)
            except MoveItError as exc:
                errors.append(str(exc))
        raise MoveItError('搬运朝向候选均不可行：'+errors[-1])

    def plan_carry_yaw(self, start, yaw):
        """One controller goal; check the retimed spline with local support exceptions."""
        node, cfg, scene = self.node, self.config, self.scene
        start_object = self.object_at(start)
        path = carrying_waypoints(start_object, cfg, scene.tip_to_object, yaw)
        scene.contacts(gripper=True, supports=self.supports)
        trajectory = plan_step(node, start, Step('连续搬运', 'yaw_cartesian', path))
        names = list(node.joints)
        times, curve, scale = _curve(trajectory, names, _scaled_limits(node, names, MoveItError), MoveItError)
        count = max(1, math.ceil(float(abs(curve.derivative().c).max())*times[-1]/.02))
        samples = sorted(set(times+np.linspace(0., times[-1], count+1).tolist()))
        result = RobotTrajectory()
        result.joint_trajectory.joint_names = names
        previous_supports = tuple(self.supports)
        try:
            for tick in samples:
                points = result.joint_trajectory.points
                if points and tick-(points[-1].time_from_start.sec+points[-1].time_from_start.nanosec/1e9)/scale < 1e-8:
                    continue
                positions = curve(tick).tolist()
                state = {**start, **dict(zip(names, positions))}
                tip = pose_matrix(node.fk(state))
                obj = tip @ scene.tip_to_object
                supports = tuple(support for position, support in zip(
                    (cfg.pick_position, cfg.place_position), self.supports)
                    if np.linalg.norm(obj[:2, 3]-position[:2]) <= .025
                    and obj[2, 3] <= position[2]+cfg.lift_distance+.002)
                if supports != previous_supports:
                    scene.contacts(gripper=True, supports=supports)
                    previous_supports = supports
                if not node.validity(state).valid:
                    raise MoveItError('连续搬运插值存在碰撞或无效状态')
                if tilt_error(obj[:3, :3]) > cfg.orientation_tolerance:
                    raise MoveItError('连续搬运插值使物体倾斜超过允许范围')
                points.append(JointTrajectoryPoint(positions=positions,
                    velocities=(curve(tick, nu=1)/scale).tolist(),
                    accelerations=(curve(tick, nu=2)/scale**2).tolist(),
                    time_from_start=_duration(tick*scale)))
        finally:
            # MoveIt execution checks a single scene; stricter per-sample checks ran above.
            scene.contacts(gripper=True, supports=self.supports)
        return result

    def run_stages(self, initial, *, execute=False):
        node, cfg, scene = self.node, self.config, self.scene
        current = dict(initial)
        steps = self.stages()
        for index, step in enumerate(steps, 1):
            print(f'{"执行" if execute else "预检查"} [{index}/{len(steps)}] {step.label}')
            try:
                if execute:
                    current = task_state(node)
                if step.event == 'approach':
                    # Keep the open fingers collision-checked during approach.
                    scene.contacts(supports=[self.supports[0]])
                elif step.event == 'grasp':
                    scene.contacts(gripper=True, supports=[self.supports[0]])
                elif step.event == 'release':
                    self.check_place(current)
                elif step.event == 'raise_empty':
                    height = max(cfg.retreat_distance, cfg.transfer_height-cfg.place_position[2])
                    step.target = shifted(node.fk(current), [0., 0., height])
                elif step.event == 'retreat':
                    # Retreat above the stand rather than backing into it at table height.
                    step.target = shifted(node.fk(current), -np.array(cfg.approach_direction)*cfg.retreat_distance)
                step.start = scene.state(current)
                step.trajectory = (self.plan_transfer(current) if step.kind == 'carry'
                                   else plan_step(node, current, step))
                if execute:
                    node.show_plan(step.trajectory)
                    if step.event == 'grasp':
                        # Even a failed close may leave an object gripped: keep its scene on exit.
                        self.holding = True
                    if step.event == 'grasp' and not node.bottle_simulated:
                        grasp_bottle(node, self.geometry)
                    else:
                        node.execute(step.trajectory, controller='gripper_controller' if step.kind == 'gripper' else None)
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
                elif step.event in ('raise_empty', 'retreat'):
                    scene.contacts(supports=[self.supports[1]])
                    if not node.validity(current).valid:
                        raise MoveItError('撤离后仍与物体或环境碰撞')
            except (Exception, KeyboardInterrupt) as exc:
                raise MoveItError(f'阶段「{step.label}」停止，后续动作未执行：{str(exc) or "interrupted"}') from exc
        if execute:
            self.completed = True
        return steps

    def prepare(self, initial):
        self.scene.place(self.pick_pose, self.config.diameter, self.config.bottle_height)
        self.scene.contacts(supports=[self.supports[0]])
        try:
            if not self.node.validity(initial).valid:
                raise MoveItError('取料点 A 与当前机器人状态碰撞')
            node, cfg = self.node, self.config
            seed = {**initial, node.gripper_joint: cfg.open_opening*1.1}
            candidates, errors = [], []
            for degrees in (0., -15., 15., -30., 30., -60., 60.):
                grasp, pregrasp = grasp_poses(cfg, self.offset, math.radians(degrees))
                try:
                    target = node.inverse_kinematics(pregrasp, seed, group=node.group, link=node.tip_link)
                    target = {name: target[name] for name in node.joints}
                    if not node.validity({**seed, **target}).valid:
                        continue
                    movement = np.array([target[name]-initial[name] for name in node.joints])
                    score = float(np.linalg.norm(movement)+.5*abs(movement).max())
                    candidates.append((score, degrees, grasp, pregrasp, target))
                except MoveItError:
                    continue
            for _, degrees, grasp, pregrasp, target in sorted(candidates, key=lambda value: value[0]):
                self.grasp, self.pregrasp, self.pregrasp_joints = grasp, pregrasp, target
                print(f'检查绕 world Z 转动 {degrees:+g}° 的法兰姿态；接近方向保持 {cfg.approach_direction}。')
                try:
                    self.steps = self.run_stages(initial)
                    return
                except MoveItError as exc:
                    errors.append(str(exc))
                    print(f'该法兰姿态不可用：{exc}')
                finally:
                    self.reset_pick()
            raise MoveItError('没有找到能保持原接近方向完成抓放的法兰姿态：'
                              +(errors[-1] if errors else '预抓取逆解或碰撞检查失败'))
        finally:
            self.reset_pick()

    def preview(self, *, reset=True):
        print(f'完整预览：{self.pick_label} 点夹取 → {self.place_label} 点放置 → 撤离；不驱动硬件。')
        try:
            for step in self.steps:
                if step.kind == 'carry':
                    self.scene.hide_world()
                elif step.event == 'raise_empty':
                    self.scene.world(self.released_pose)
                trajectory = preview_segment(step.trajectory)
                self.node.display.publish(DisplayTrajectory(model_id='onero_a1', trajectory_start=step.start,
                                                            trajectory=[trajectory]))
                wait_seconds(self.node, len(trajectory.joint_trajectory.points)*(.05+1./30.)+.2)
        finally:
            if reset:
                self.reset_pick()


class RoundTripTask:
    """Compose two complete pick/place legs, with a real release between them."""
    def __init__(self, node, config):
        self.node, self.config = node, config
        outbound = deepcopy(config)
        outbound.ready_joints = None  # Return to ready only after the bottle is back at A.
        self.first = self.current = FixedTask(node, outbound)
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
        task = FixedTask(self.node, cfg, pick_pose=released)
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
        self.current.prepare(current)
        self.current.executing, self.current.gains = True, self.gains
        inbound = self.current.run_stages(task_state(self.node), execute=True)
        self.completed = True
        return outbound+inbound


def preview_identity(node, task):
    bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
    values = node.call(GetParameters, bridge+'/get_parameters',
                      GetParameters.Request(names=['simulate', 'model', 'can_channel', 'mounting'])).values
    if len(values) != 4 or values[0].type != 1 or any(value.type != 4 for value in values[1:]):
        raise MoveItError('无法确认模拟模式与设备配置')
    node.bottle_simulated = values[0].bool_value
    print('运行模式：模拟。' if node.bottle_simulated else '运行模式：真机，请按配置在 A 点摆放实体水瓶。')
    models = [node.parameter('/move_group', name) for name in ('robot_description', 'robot_description_semantic')]
    objects = []
    for obj in sorted(task.original.world.collision_objects, key=lambda obj: obj.id):
        if obj.id == task.scene.id:
            continue
        obj = deepcopy(obj)
        obj.header.stamp = Time()
        objects.append(hashlib.sha256(serialize_message(obj)).hexdigest())
    identity = {'version': 4, 'round_trip': getattr(node.args, 'round_trip', True),
                'config': task.config.dictionary(), 'arm': node.arm_side,
                'joints': node.joints, 'simulate': node.bottle_simulated,
                'device': [value.string_value for value in values[1:]],
                'model': hashlib.sha256('\n'.join(models).encode()).hexdigest(), 'scene': objects}
    return identity


def cleanup(node):
    task = getattr(node, 'fixed_task', None)
    clear_task_bottles(node)
    if task is not None:
        task.scene.attached = False
        if task.holding:
            print('虚拟模型已清理；手臂仍保持夹持和当前增益，请确认实物状态。')


def example(node):
    if node.args.frame != 'world':
        raise MoveItError('固定抓放仅支持 --frame world')
    if node.args.record_orientation and node.args.execute:
        raise MoveItError('--record-orientation 不能与 --execute 一起使用')
    if node.args.config is None and node.args.record_orientation:
        raise MoveItError('默认演示已设置抓取朝向，直接运行即可；自定义记录朝向时请指定 --config')
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
    task_type = RoundTripTask if getattr(node.args, 'round_trip', True) else FixedTask
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
    task.prepare(initial)
    print(f'固定取料中心 A={config.pick_position}，放料中心 B={config.place_position} m；'
          f'水平搬运距离={math.dist(config.pick_position[:2], config.place_position[:2])*100:.1f} cm。')
    clearance = config.transfer_height-max(config.pick_position[2], config.place_position[2])
    print(f'搬运路径：先竖直抬升 → 瓶底距较高支撑面约 {clearance*100:.1f} cm 时平移 → 到 B 点下降放置。')
    if node.args.execute:
        bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
        task.gains = GainClient(node, bridge)
        task.executing = True
        task.run_stages(task_state(node), execute=True)
        print('往返抓放完成：水瓶已从 A 点搬到 B 点，再重新夹取搬回 A 点，已松爪并撤离。'
              if getattr(node.args, 'round_trip', True) else
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
                       planning_pipelines=(), cleanup=cleanup)
