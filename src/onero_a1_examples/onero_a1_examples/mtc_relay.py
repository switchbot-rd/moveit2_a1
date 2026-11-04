"""Plan and execute one bottle's dual-arm MTC relay."""
from copy import deepcopy
import math

import numpy as np
import rclpy
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, PlanningSceneWorld, RobotTrajectory
from moveit_msgs.srv import GetPlanningScene
from scipy.spatial.transform import Rotation
from onero_a1_common.payload import PayloadClient

from .bottle_scene import BottleScene, PickerGeometry, clear_bottle, matrix_pose, pose_matrix
from .dual_task_support import Mirror
from .mtc_execution import MtcExecutor
from .mtc_natural import NaturalArmPolicy
from .mtc_planning import MtcMotionPlan, MtcPlanner, needs_rear_preparation
from .mtc_posture import rank_solution
from .support import MoveItError
from .task_support import task_state


OBJECT_ID = 'onero_mtc_bottle_both'
MTC_OBJECT_IDS = (OBJECT_ID, 'onero_mtc_bottle_left', 'onero_mtc_bottle_right')


def select_solution(planner, arms):
    groups = {frozenset(arm.joints): arm.arm_controllers for arm in arms}
    groups.update({frozenset([arm.gripper_joint]): [arm.gripper_controller] for arm in arms})
    candidates = []
    for index in range(len(planner.task.solutions)):
        solution = planner.task.solutions[index]
        message = solution.toMsg(planner.task.introspection())
        try:
            for sub in message.sub_trajectory:
                names = frozenset(sub.trajectory.joint_trajectory.joint_names)
                if names:
                    if names not in groups:
                        raise MoveItError('MTC 返回了接力手臂之外的轨迹')
                    sub.execution_info.controller_names = list(groups[names])
            score = 0.
            for arm in arms:
                if not any(set(sub.trajectory.joint_trajectory.joint_names) == set(arm.joints)
                           and sub.trajectory.joint_trajectory.points for sub in message.sub_trajectory):
                    continue
                own = (set(arm.joints), {arm.gripper_joint})
                filtered = deepcopy(message)
                for sub in filtered.sub_trajectory:
                    names = set(sub.trajectory.joint_trajectory.joint_names)
                    if names and names not in own:
                        sub.trajectory = RobotTrajectory()
                score += rank_solution(arm, filtered, wrist_limit=arm.wrist_roll_limit,
                                       motions=getattr(planner, 'motions', None))
            candidates.append((score, solution.cost, index, solution, message))
        except MoveItError as exc:
            print(f'MTC 方案 {index+1} 未通过构型检查：{exc}')
    if not candidates:
        raise MoveItError('没有符合自然构型要求的完整方案；请调整起点、朝向或增加 --max-solutions')
    score, _, index, solution, message = min(candidates, key=lambda item: item[:3])
    print(f'MTC 找到 {len(planner.task.solutions)} 条完整方案，选中方案 {index+1}'
          f'（评分 {score:.3f}），包含 {len(message.sub_trajectory)} 个阶段。')
    return solution, message


def split_initial_clearance(message, main_arm):
    """Keep a selected native empty-arm prefix separate from the grasp adapter."""
    own = (set(main_arm.joints), {main_arm.gripper_joint})
    boundary = next((index for index, sub in enumerate(message.sub_trajectory)
                     if set(sub.trajectory.joint_trajectory.joint_names) in own
                     and sub.trajectory.joint_trajectory.points), 0)
    prefix = message.sub_trajectory[:boundary]
    if not any(sub.trajectory.joint_trajectory.points for sub in prefix):
        return None, message
    first, rest = deepcopy(message), deepcopy(message)
    first.sub_trajectory = first.sub_trajectory[:boundary]
    rest.sub_trajectory = rest.sub_trajectory[boundary:]
    joint_state = rest.start_scene.robot_state.joint_state
    positions = dict(zip(joint_state.name, joint_state.position))
    for sub in prefix:
        path = sub.trajectory.joint_trajectory
        if path.points:
            positions.update(zip(path.joint_names, path.points[-1].positions))
        changed = sub.scene_diff.robot_state.joint_state
        positions.update(zip(changed.name, changed.position))
        matrix = sub.scene_diff.allowed_collision_matrix
        if matrix.entry_names:
            rest.start_scene.allowed_collision_matrix = deepcopy(matrix)
    joint_state.name, joint_state.position = list(positions), list(positions.values())
    joint_state.velocity, joint_state.effort = [], []
    return first, rest


def reset_relay(node, arms):
    if not node.args.execute:
        raise MoveItError('确认实物水瓶已取下后，使用 --reset-task --execute 恢复任务')
    for side, arm in arms.items():
        opening = task_state(arm)[arm.gripper_joint]
        if not math.isfinite(opening) or abs(opening-1.1) > .015:
            raise MoveItError(f'恢复前请托住水瓶、取下实物并完全张开两侧夹爪；{side} 夹爪尚未全开')
    for side, arm in arms.items():
        PayloadClient(arm, f'/{side}/onero_topic_bridge').restore_baseline()
    for object_id in MTC_OBJECT_IDS:
        clear_bottle(arms['right'], object_id)
    print('MTC 接力已恢复：两臂负载已恢复启动基线，水瓶模型已清理。')


class MtcRelay:
    def __init__(self, node, arms, config):
        if config.arm != 'right' or config.c_position is None:
            raise MoveItError('MTC 双臂接力配置须以右臂为基准，并填写 c_position')
        self.node, self.arms, self.config = node, arms, config
        self.mirror = Mirror(node)
        self.points = {'A': list(config.pick_position), 'B': list(config.place_position),
                       'C': list(config.c_position)}
        self.supports = {'A': config.pick_support, 'B': config.place_support, 'C': config.place_support}
        self.route = [('right', 'A', 'B'), ('left', 'B', 'C')]
        if node.args.round_trip:
            self.route += [('left', 'C', 'B'), ('right', 'B', 'A')]
        self.scene = self.executor = self.planner = None
        self.geometry, self.standby = {}, {}
        self.completed = False

    def leg_config(self, index, released=None):
        side, pick, place = self.route[index]
        config = deepcopy(self.config)
        config.arm = side
        config.pick_position = (list(self.points[pick]) if released is None else
                                (released[:3, 3]-released[:3, 2]*config.grasp_height).tolist())
        config.place_position = list(self.points[place])
        config.c_position = None  # Each MTC leg has only its own pick/place pair.
        config.pick_support, config.place_support = self.supports[pick], self.supports[place]
        if side == 'left':
            reference = np.eye(4)
            reference[:3, :3] = Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
            config.grasp_rpy = Rotation.from_matrix(
                pose_matrix(self.mirror.pose(matrix_pose(reference)))[:3, :3]).as_euler('xyz').tolist()
            config.approach_direction = self.mirror.point(config.approach_direction).tolist()
            if config.tcp_offset is not None:
                config.tcp_offset = (self.mirror.tool.T @ config.tcp_offset).tolist()
        c_offset = 0. if getattr(self, 'simulated', False) else config.left_c_height_offset
        if side == 'left' and pick == 'B':
            config.grasp_position_offset = deepcopy(config.left_b_grasp_offset)
        elif side == 'left' and pick == 'C':
            config.grasp_position_offset = [0., 0., c_offset] if c_offset else None
        elif side == 'right' and pick == 'A':
            config.grasp_position_offset = deepcopy(config.right_a_grasp_offset)
        config.place_z_offset = c_offset if side == 'left' and place == 'C' else 0.
        # A placement compensation must change the effective target itself, so
        # the placement feedback loop keeps it instead of pulling back to B.
        if not getattr(self, 'simulated', False):
            place_offset = ((config.right_b_place_offset if side == 'right' and place == 'B' else None)
                            or (config.left_c_place_offset if side == 'left' and place == 'C' else None))
            if place_offset:
                config.place_position = [value+offset for value, offset in zip(
                    config.place_position, place_offset)]
        config.validate()
        return config

    def reported_release(self, side, place, transform):
        """Remove the commanded placement compensation from the measured pose."""
        result = transform.copy()
        if getattr(self, 'simulated', False):
            return result
        offset = ((self.config.right_b_place_offset if side == 'right' and place == 'B' else None)
                  or (self.config.left_c_place_offset if side == 'left' and place == 'C' else None))
        if offset:
            result[:3, 3] = result[:3, 3]-np.asarray(offset)
        return result

    def initialize(self):
        current = self.node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
            components=PlanningSceneComponents(
                components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))).scene
        if any(item.object.id in MTC_OBJECT_IDS for item in current.robot_state.attached_collision_objects):
            raise MoveItError('场景中已有 MTC 附着水瓶；请先确认持物状态，取下实物后运行 --reset-task --execute')
        for side, arm in self.arms.items():
            index = next(i for i, leg in enumerate(self.route) if leg[0] == side)
            config = self.leg_config(index)
            arm.start = task_state(arm)
            arm.wrist_roll_limit = config.max_wrist_roll_deviation
            arm.args.diameter, arm.args.grasp_opening = config.diameter, config.grasp_opening
            self.geometry[side] = PickerGeometry(arm, config.diameter,
                Rotation.from_euler('xyz', config.grasp_rpy).as_matrix())
            arm.mtc_natural_policy = NaturalArmPolicy(arm,
                wrist_limit=config.max_wrist_roll_deviation, swivel_limit=config.max_arm_swivel_deviation,
                bend_limit=config.max_wrist_bend_deviation)
        modes = {session['identity']['simulate'] for session in self.node._startup_sessions()}
        if len(modes) != 1:
            raise MoveItError('MTC 接力需要两侧手臂使用相同的硬件模式')
        self.simulated = modes.pop()
        self.scene = BottleScene(self.node, self.geometry['right'], object_id=OBJECT_ID)
        current = self.scene.read()
        previous = ({item.id for item in current.world.collision_objects}
                    | set(current.allowed_collision_matrix.entry_names)
                    | set(current.allowed_collision_matrix.default_entry_names))
        for object_id in MTC_OBJECT_IDS[1:]:
            if object_id in previous:
                clear_bottle(self.arms['right'], object_id)
        for label, position in self.points.items():
            self.supports[label] = self.check_support(self.scene, label, position, self.supports[label])
        self.place_bottle(self.leg_config(0))
        state = task_state(self.node)
        yielding = {side for index, (side, _, _) in enumerate(self.route[:-1])
                    if self.route[index+1][0] != side}
        for side in yielding:
            self.standby[side] = self.standby_target(side, state)
        print(f'接力位置（world，m）：A={self.points["A"]}，B={self.points["B"]}，C={self.points["C"]}。')

    def check_support(self, scene, label, position, support):
        height = scene.support_height(position, self.config.diameter/2., object_id=support)
        if height is None or abs(position[2]-self.config.bottle_height/2.-height) > self.config.placement_height_tolerance:
            raise MoveItError(f'{label} 点瓶底与支撑面不匹配，请核对任务配置和 environment_file')
        return scene.support_id

    def place_bottle(self, config, released=None):
        from .mtc_pick_place import bottle_object
        self.scene.bottle = bottle_object(config, OBJECT_ID)
        if released is not None:
            self.scene.bottle.pose = matrix_pose(released)
        self.scene.changed = True
        self.scene.apply(PlanningScene(world=PlanningSceneWorld(collision_objects=[self.scene.bottle])))
        self.scene.contacts(supports=[config.pick_support])

    def standby_target(self, side, initial):
        from onero_a1_common.continuous import geometry

        arm = self.arms[side]
        sign = 1. if side == 'left' else -1.
        context = geometry(arm, refresh=True)
        seeds = [initial]
        reference = arm.reference_seed()
        if reference is not None:
            natural = {**initial, **reference}
            natural.update({arm.joints[index]: 0. for index in (4, 5, 6)})
            seeds.append(natural)
            # Yield around the natural joint posture, with no flange-frame
            # target. Small shoulder changes leave the comfortable wrist free.
            for index in (0, 1, 2):
                for delta in (-.3, -.15, .15, .3):
                    candidate = dict(natural)
                    candidate[arm.joints[index]] += delta
                    seeds.append(candidate)
        candidates = []
        for candidate in seeds:
            point = context.fk(candidate, arm.tip_link, arm.args.frame)[:3, 3]
            if sign*(point[1]-self.points['B'][1]) < .20:
                continue
            if not context.validity(candidate, arm.group, b'')[0]:
                continue
            try:
                cost = arm.mtc_natural_policy.pose_cost(candidate)
            except MoveItError:
                continue
            movement = sum((candidate[name]-initial[name])**2 for name in arm.joints)
            candidates.append((cost, movement, candidate))
        if candidates:
            return min(candidates, key=lambda item: item[:2])[2]
        raise MoveItError(f'{side} 臂没有符合自然构型的向外让位姿态')

    def close_planner(self):
        if self.planner is not None:
            self.planner.close()
            self.planner = None

    def plan_leg(self, index, released):
        side, pick, place = self.route[index]
        arm, config = self.arms[side], self.leg_config(index, released)
        self.scene = BottleScene(arm, self.geometry[side], object_id=OBJECT_ID)
        config.pick_support = self.check_support(self.scene, pick, config.pick_position, config.pick_support)
        config.place_support = self.check_support(self.scene, place, self.points[place],
                                                   config.place_support)
        self.place_bottle(config, released)
        arm.start = task_state(arm)
        self.executor = MtcExecutor(arm, config, self.geometry[side], self.scene,
                                    mass=self.node.args.mass, simulated=self.simulated)
        yield_target = (self.standby[side] if index+1 < len(self.route)
                        and self.route[index+1][0] != side else None)
        clearance = ((self.arms['left'], self.leg_config(1), self.geometry['left'])
                     if index == 0 else None)
        rear_preparation = needs_rear_preparation(side, pick)
        offset = ((self.config.right_b_place_offset if side == 'right' and place == 'B' else None)
                  or (self.config.left_c_place_offset if side == 'left' and place == 'C' else None))
        if offset and not getattr(self, 'simulated', False):
            print(f'{"左" if side == "left" else "右"}臂 {place} 点放置补偿：'
                  + ' '.join(f'{value*1000:+.1f}' for value in offset)
                  + ' mm（已计入放置目标）', flush=True)
        if released is not None:
            quaternion = Rotation.from_matrix(released[:3, :3]).as_quat().tolist()
            print(f'交接水瓶位姿（world）：XYZ={released[:3, 3].tolist()}，qxyzw={quaternion}。')
        if self.planner is None:
            self.planner = MtcPlanner(arm, config, self.geometry[side], OBJECT_ID,
                                      yield_target=yield_target, initial_clearance=clearance,
                                      rear_preparation=rear_preparation)
        else:
            self.planner.reset(arm, config, self.geometry[side], OBJECT_ID,
                                yield_target=yield_target, initial_clearance=clearance,
                                rear_preparation=rear_preparation)
        self.planner.task.name = f'{side} {pick} → {place}'
        self.planner.plan(self.node.args.max_solutions, self.node.args.task_timeout)
        solution, message = select_solution(self.planner, list(self.arms.values()))
        self.executor.planner = self.planner
        self.executor.motions = deepcopy(self.planner.motions)
        self.executor.yield_target = yield_target
        self.planner.task.publish(solution)
        return message

    def execute(self):
        released = None
        for index, (side, pick, place) in enumerate(self.route):
            print(f'MTC 接力 [{index+1}/{len(self.route)}] {side}：{pick}→{place}。')
            message = self.plan_leg(index, released)
            prefix, message = split_initial_clearance(message, self.arms[side])
            if prefix is not None:
                left = self.arms['left']
                PayloadClient(left, '/left/onero_topic_bridge').ensure_unloaded()
                adapter = MtcExecutor(left, self.leg_config(1), self.geometry['left'], self.scene,
                    mass=self.node.args.mass, simulated=self.simulated, planner=self.planner)
                print('执行已选中 MTC 方案中的左臂局部退开。')
                adapter.execute_motion(MtcMotionPlan(prefix, deepcopy(self.planner.motions)))
            self.executor.run(message)
            if self.executor.holding or self.scene.attached or not self.executor.completed:
                raise MoveItError('当前手臂尚未完成松爪撤离，接力中止')
            released = self.reported_release(side, place, self.executor._released_transform)
        self.completed = True
        target = 'A' if self.node.args.round_trip else 'C'
        print(f'MTC 双臂接力完成：水瓶已放到 {target} 点，两臂已松爪撤离。')

    def nominal_released(self, config):
        """Bottle pose after this leg's placement, used to preview the next leg."""
        from .mtc_pick_place import bottle_object
        placed = deepcopy(config)
        placed.pick_position = list(config.place_position)
        placed.pick_position[2] += getattr(config, 'place_z_offset', 0.)
        return pose_matrix(bottle_object(placed, OBJECT_ID).pose)

    def preview(self):
        released = None
        for index, (side, pick, place) in enumerate(self.route):
            print(f'MTC 预览 [{index+1}/{len(self.route)}] {side}：{pick}→{place}。')
            self.plan_leg(index, released)
            released = self.reported_release(
                side, place, self.nominal_released(self.leg_config(index)))
        print('已按手臂分别发布接力方案；在 RViz MTC Tasks 中依次选择方案播放，Ctrl+C 结束。')
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=.1)
        except KeyboardInterrupt:
            pass

    def cleanup(self):
        try:
            if self.executor is not None:
                self.executor.cleanup()
            elif self.scene is not None and self.scene.changed:
                clear_bottle(self.arms['right'], OBJECT_ID)
        finally:
            self.close_planner()
