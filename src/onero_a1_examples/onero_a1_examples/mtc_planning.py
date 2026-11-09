"""Build an MTC task with the running robot's model and planning settings."""
from pathlib import Path
from dataclasses import dataclass
import math
import tempfile
import threading
import time
import signal
import uuid
import weakref
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, GetParameterTypes, ListParameters
from rclpy.parameter import parameter_value_to_python
from scipy.spatial.transform import Rotation
import numpy as np
import yaml

from .fixed_task_geometry import grasp_poses
from .bottle_scene import matrix_pose, pose_matrix
from .support import MoveItError


@dataclass
class MtcMotionPlan:
    message: object
    motions: dict
    constraints: object = None
    restore: object = None


def grasp_targets(config, offset, angle_step, *, grasp_axis=None, forward_axis=None, reference_rotation=None):
    """Sample nearby world-yaw grasps before covering the full circle."""
    preferred = np.deg2rad([0., -15., 15., -30., 30., -60., 60.]).tolist()
    result, seen = [], set()
    for yaw in preferred+np.arange(0., 2.*np.pi, angle_step).tolist():
        key = round(yaw % (2.*np.pi), 9)
        if key in seen:
            continue
        seen.add(key)
        correction = getattr(config, 'grasp_position_offset', None) or (0., 0., 0.)
        if grasp_axis is not None and forward_axis is not None:
            jaw = np.asarray(grasp_axis, dtype=float)
            jaw = jaw/np.linalg.norm(jaw)
            forward = np.asarray(forward_axis, dtype=float)
            forward = forward-jaw*np.dot(forward, jaw)
            forward = forward/np.linalg.norm(forward)
            local = np.column_stack((forward, jaw, np.cross(forward, jaw)))
            heading = yaw+np.arctan2(config.approach_direction[1], config.approach_direction[0])
            side = np.array([-np.sin(heading), np.cos(heading), 0.])
            reference = (Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
                         if reference_rotation is None else reference_rotation)
            reference_jaw = reference @ jaw
            preferred_sign = 1. if np.dot(reference_jaw, side) >= 0. else -1.
            center = np.asarray(config.pick_position)+[0., 0., config.grasp_height]+np.asarray(correction)
            for degrees in (-30., -15., 0., 15., 30., -45., 45., 60., 75.):
                pitch = np.deg2rad(degrees)
                front = np.array([np.cos(pitch)*np.cos(heading), np.cos(pitch)*np.sin(heading), -np.sin(pitch)])
                for sign in (preferred_sign, -preferred_sign):
                    closing = sign*side
                    rotation = np.column_stack((front, closing, np.cross(front, closing))) @ local.T
                    target = np.eye(4)
                    target[:3, :3], target[:3, 3] = rotation, center-rotation @ offset
                    result.append((yaw, matrix_pose(target)))
            continue
        pose = grasp_poses(config, offset, yaw, correction)[0]
        if grasp_axis is None:
            result.append((yaw, pose))
            continue
        axis = np.asarray(grasp_axis, dtype=float)
        axis = axis/np.linalg.norm(axis)
        source = pose_matrix(pose)
        center = source[:3, 3]+source[:3, :3] @ offset
        # Turning about the pad-to-pad axis leaves both contact points fixed.
        for degrees in (0., -15., 15., -30., 30., -45., 45.):
            target = source.copy()
            target[:3, :3] = source[:3, :3] @ Rotation.from_rotvec(axis*np.deg2rad(degrees)).as_matrix()
            target[:3, 3] = center-target[:3, :3] @ offset
            result.append((yaw, matrix_pose(target)))
    if reference_rotation is not None:
        result.sort(key=lambda item: Rotation.from_matrix(
            np.asarray(reference_rotation).T @ pose_matrix(item[1])[:3, :3]).magnitude())
    return result


def axial_constraints(node, config, *, start=None, bend_limit=None, pitch_limit=None):
    from .mtc_natural import natural_reference
    reference = natural_reference(node)
    result = Constraints(name='natural axial posture')
    for index, limit in ((0, config.max_arm_swivel_deviation),
                         (4, config.max_wrist_roll_deviation), (6, config.max_wrist_roll_deviation)):
        name = node.joints[index]
        if start is not None:
            limit = max(limit, abs(start[name]-reference[name]))+.01
        result.joint_constraints.append(JointConstraint(
            joint_name=name, position=reference[name], tolerance_above=float(limit),
            tolerance_below=float(limit), weight=1.))
    if pitch_limit is not None:
        name = node.joints[1]
        if start is not None:
            pitch_limit = max(pitch_limit, abs(start[name]-reference[name]))+.01
        result.joint_constraints.append(JointConstraint(
            joint_name=name, position=reference[name], tolerance_above=float(pitch_limit),
            tolerance_below=float(pitch_limit), weight=1.))
    limit = (getattr(config, 'max_wrist_bend_deviation', math.radians(20.))
             if bend_limit is None else bend_limit)
    if start is not None:
        limit = max(limit, abs(start[node.joints[5]]))+.01
    result.joint_constraints.append(JointConstraint(joint_name=node.joints[5], position=0.,
        tolerance_above=float(limit), tolerance_below=float(limit), weight=1.))
    elbow = node.joints[3]
    lower, upper = ((.05, np.pi) if reference[elbow] > 0. else (-np.pi, -.05))
    if start is not None:
        lower, upper = min(lower, start[elbow])-.01, max(upper, start[elbow])+.01
    result.joint_constraints.append(JointConstraint(
        joint_name=elbow, position=reference[elbow], tolerance_above=float(upper-reference[elbow]),
        tolerance_below=float(reference[elbow]-lower), weight=1.))
    return result


def needs_rear_preparation(arm_side, pick):
    """A left arm cannot connect to the B-point grab from the startup pose."""
    return arm_side == 'left' and pick == 'B'


def fixed_mounting_contacts(description, object_names):
    model = ET.fromstring(description)
    links = {link.get('name') for link in model.findall('link')}
    parents = {joint.find('child').get('link'): (joint.find('parent').get('link'), joint.get('type'))
               for joint in model.findall('joint')}
    result = []
    for link, plate in (('base_link', 'onero_env_mount_plate'),
                        ('left_base_link', 'onero_env_left_mount_plate'),
                        ('right_base_link', 'onero_env_right_mount_plate')):
        if link not in links or plate not in object_names:
            continue
        current = link
        while current in parents and parents[current][1] == 'fixed':
            current = parents[current][0]
        if current not in parents:
            result.append((link, plate))
    return result


def task_parameters(node, config, grasp_angle, *, fast_ik=False):
    prefixes = ['robot_description', 'robot_description_semantic', 'robot_description_kinematics',
                'robot_description_planning', 'ompl']
    names = node.call(ListParameters, '/move_group/list_parameters',
                      ListParameters.Request(prefixes=prefixes)).result.names
    types = node.call(GetParameterTypes, '/move_group/get_parameter_types',
                      GetParameterTypes.Request(names=names)).types
    if len(names) != len(types):
        raise MoveItError(f'无法读取 MTC 参数类型：请求 {len(names)} 项，返回 {len(types)} 项')
    # MoveIt declares optional parameters without values. A single such
    # parameter makes rclcpp reject an entire GetParameters batch.
    names = [name for name, kind in zip(names, types) if kind != ParameterType.PARAMETER_NOT_SET]
    values = node.call(GetParameters, '/move_group/get_parameters', GetParameters.Request(names=names)).values
    if len(names) != len(values):
        raise MoveItError('无法读取 MTC 所需的机器人和规划参数')
    parameters = {}
    for name, value in zip(names, values):
        if value.type != 0:
            converted = parameter_value_to_python(value)
            # ROS parameter YAML cannot encode an empty array's element
            # type. Let MTC declare these optional defaults with their type.
            if value.type in (5, 6, 7, 8, 9) and not converted:
                continue
            parameters[name] = list(converted) if value.type in (5, 6, 7, 8, 9) else converted
    required = ('robot_description', 'robot_description_semantic', 'ompl.planning_plugins')
    missing = [name for name in required if name not in parameters]
    if missing:
        raise MoveItError('MTC 缺少已赋值的必要参数：'+', '.join(missing))
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    for name, angle in (('onero_mtc_open', config.open_opening*1.1), ('onero_mtc_grasp', grasp_angle)):
        for previous in semantic.findall('group_state'):
            if previous.get('name') == name and previous.get('group') == node.gripper_group:
                semantic.remove(previous)
        state = ET.SubElement(semantic, 'group_state', name=name, group=node.gripper_group)
        ET.SubElement(state, 'joint', name=node.gripper_joint, value=str(float(angle)))
    from .mtc_natural import IK_POSTURE_WEIGHTS, natural_reference
    reference = natural_reference(node)
    if reference is None:
        raise MoveItError('MTC 自然构型优选需要在 posture.yaml 中启用 enabled: true')
    if len(getattr(node, 'joints', ())) == 7:
        prefix = f'robot_description_kinematics.{node.group}.'
        parameters[prefix+'natural_reference'] = [reference[name] for name in node.joints]
        parameters[prefix+'natural_weights'] = list(IK_POSTURE_WEIGHTS)
        parameters[prefix+'natural_fast_mode'] = fast_ik
        if fast_ik:
            from .mtc_ik_seeds import load_seeds
            seeds = load_seeds(node)
            if seeds:
                parameters[prefix+'natural_seed_bank'] = [value for seed in seeds for value in seed]
            print(f'MTC IK 快速搜索：已加载 {len(seeds)} 个已通过检查的构型作为初值。')
        else:
            parameters.pop(prefix+'natural_seed_bank', None)
    state = ET.SubElement(semantic, 'group_state', name='onero_mtc_natural', group=node.group)
    for name, value in reference.items():
        ET.SubElement(state, 'joint', name=name, value=str(float(value)))
    parameters['robot_description_semantic'] = ET.tostring(semantic, encoding='unicode')
    return parameters


class MtcPlanner:
    def __init__(self, node, config, geometry, object_id, *, yield_target=None,
                 initial_clearance=None, rear_preparation=False, route=None):
        try:
            import rclcpp
            from moveit.task_constructor import core, stages
        except ImportError as exc:
            raise MoveItError('缺少 MTC Python 接口，请先执行 source setup_env.sh 安装依赖') from exc
        self.rclcpp, self.task, self.native_node = rclcpp, None, None
        self.solvers, self._context_ready = (), False
        self.directory = tempfile.TemporaryDirectory(prefix='onero_mtc_')
        self.initial_clearance = initial_clearance
        self.rear_preparation = rear_preparation
        self.route = route
        self._fast_ik = getattr(node.args, 'ik_search', 'auto') == 'auto'
        self._using_initial_clearance = False
        self._recipe = (node, config, geometry, object_id, core, stages, yield_target)
        try:
            self._build(*self._recipe)
        except (Exception, KeyboardInterrupt):
            self.close()
            raise

    def reset(self, node, config, geometry, object_id, *, yield_target=None, initial_clearance=None,
              rear_preparation=False, route=None):
        """Build the next relay leg without restarting the native ROS context."""
        from moveit.task_constructor import core, stages
        self.solvers, self.task, self.native_node = (), None, None
        self.initial_clearance, self._using_initial_clearance = initial_clearance, False
        self.rear_preparation = rear_preparation
        self.route = route
        self._fast_ik = getattr(node.args, 'ik_search', 'auto') == 'auto'
        self._recipe = (node, config, geometry, object_id, core, stages, yield_target)
        try:
            self._build(*self._recipe)
        except (Exception, KeyboardInterrupt):
            self.close()
            raise

    def _build(self, node, config, geometry, object_id, core, stages, yield_target,
               *, initial_clearance=None):
        self.motions = {}
        from .mtc_directions import load_directions
        self._preferred_directions = load_directions(node, self.route) if self._fast_ik else None
        self._using_initial_clearance = initial_clearance is not None
        rclcpp = self.rclcpp
        angle = geometry.angle if config.grasp_opening is None else config.grasp_opening*1.1
        parameters = task_parameters(node, config, angle, fast_ik=self._fast_ik)
        path = Path(self.directory.name)/'parameters.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}, sort_keys=False))
        previous = signal.getsignal(signal.SIGINT)
        try:
            if not self._context_ready:
                rclcpp.init([])
                self._context_ready = True
        finally:
            signal.signal(signal.SIGINT, previous)
        options = rclcpp.NodeOptions(arguments=['--ros-args', '--params-file', str(path)],
                                     automatically_declare_parameters_from_overrides=True)
        self.native_node = rclcpp.Node('onero_mtc_'+uuid.uuid4().hex[:8], options)
        self.task = task = core.Task('mtc_pick_place')
        task.name = f'{node.arm_side} pick and place (natural)'
        task.loadRobotModel(self.native_node)
        from .mtc_reachability import place_reach_bound
        self._place_reach_bound = place_reach_bound(parameters, node.group, node.tip_link)
        self.supports = {config.pick_support, config.place_support}
        current = stages.CurrentState('current state')
        current.timeout = 5.
        task.add(current)
        world = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
            components=PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY))).scene.world
        bottle = next((obj for obj in world.collision_objects if obj.id == object_id), None)
        self._pick_yaw = (float(Rotation.from_matrix(pose_matrix(bottle.pose)[:3, :3]).as_euler('xyz')[2])
                          if bottle is not None else 0.)
        contacts = fixed_mounting_contacts(parameters['robot_description'], {obj.id for obj in world.collision_objects})
        self.mounting_contacts = contacts
        if contacts:
            mounting = stages.ModifyPlanningScene('fixed mounting contacts')
            for base, plate in contacts:
                mounting.allowCollisions(base, plate, True)
            task.add(mounting)
        if initial_clearance is not None:
            self._add_initial_clearance(task, *initial_clearance, core, stages)
        self._add_leg(task, node, config, geometry, object_id, core, stages, yield_target,
                      rear_preparation=self.rear_preparation)
        task.init()

    def _add_initial_clearance(self, task, node, config, geometry, core, stages):
        from .mtc_natural import NaturalArmPolicy
        natural = NaturalArmPolicy(node, wrist_limit=config.max_wrist_roll_deviation,
                                   swivel_limit=config.max_arm_swivel_deviation,
                                   bend_limit=config.max_wrist_bend_deviation)
        cartesian = core.CartesianPath()
        cartesian.step_size = .003
        cartesian.max_velocity_scaling_factor = node.args.velocity
        cartesian.max_acceleration_scaling_factor = node.args.acceleration
        self.solvers += (cartesian,)
        stage = stages.MoveRelative('left local clearance', cartesian)
        stage.group = node.group
        tip = PoseStamped()
        tip.header.frame_id = node.tip_link
        tip.pose.orientation.w = 1.
        stage.ik_frame = tip
        stage.min_distance, stage.max_distance = config.retreat_distance/2., config.retreat_distance
        forward = np.asarray(geometry.center(geometry.angle), dtype=float)
        if np.linalg.norm(forward) < 1e-6:
            raise MoveItError('无法确定左臂局部退开方向')
        direction = -forward/np.linalg.norm(forward)
        vector = Vector3Stamped()
        vector.header.frame_id = node.tip_link
        vector.vector.x, vector.vector.y, vector.vector.z = map(float, direction)
        stage.setDirection(vector)
        stage.path_constraints = axial_constraints(node, config, start=node.start)
        stage.setCostTerm(self._record_motion(self.task, self.motions, natural,
            {'frame': node.tip_link, 'direction': direction.tolist(), 'arrival': False}))
        task.add(stage)

    def _add_rear_preparation(self, task, node, config, geometry, core, stages):
        """Retreat a left arm to the rear before picking at B, inside its own leg."""
        from .mtc_natural import NaturalArmPolicy
        natural = NaturalArmPolicy(node, wrist_limit=config.max_wrist_roll_deviation,
                                   swivel_limit=config.max_arm_swivel_deviation,
                                   bend_limit=config.max_wrist_bend_deviation).with_bend_limit(math.pi/2.)
        cartesian = core.CartesianPath()
        cartesian.step_size = .003
        cartesian.max_velocity_scaling_factor = node.args.velocity
        cartesian.max_acceleration_scaling_factor = node.args.acceleration
        self.solvers += (cartesian,)
        stage = stages.MoveRelative('left rear preparation', cartesian)
        stage.group = node.group
        tip = PoseStamped()
        tip.header.frame_id = node.tip_link
        tip.pose.orientation.w = 1.
        stage.ik_frame = tip
        stage.min_distance, stage.max_distance = config.retreat_distance/2., config.retreat_distance
        direction = np.array([-1., 1., 0.])/np.sqrt(2.)
        vector = Vector3Stamped()
        vector.header.frame_id = 'world'
        vector.vector.x, vector.vector.y, vector.vector.z = map(float, direction)
        stage.setDirection(vector)
        stage.path_constraints = axial_constraints(node, config, start=node.start, bend_limit=math.pi/2.)
        stage.setCostTerm(self._record_motion(self.task, self.motions, natural,
            {'frame': 'world', 'direction': direction.tolist(), 'arrival': False,
             'bend_limit': math.pi/2., 'constraints': stage.path_constraints}))
        print('MTC 左臂先向左后方腾挪，再规划夹取。')
        task.add(stage)

    @staticmethod
    def _record_motion(task, motions, natural, spec):
        owner = weakref.ref(task)
        def cost(solution, comment=''):
            value = natural.stage_cost(solution, comment)
            if np.isfinite(value):
                current = owner()
                if current is None:
                    return float('inf')
                for sub in solution.toMsg(current.introspection()).sub_trajectory:
                    if sub.trajectory.joint_trajectory.points:
                        motions[sub.info.stage_id] = spec
            return value
        return cost

    def _add_yield(self, task, node, config, target, core, stages, *, start=None):
        from .mtc_natural import NaturalArmPolicy
        natural = NaturalArmPolicy(node, wrist_limit=config.max_wrist_roll_deviation,
                                   swivel_limit=config.max_arm_swivel_deviation,
                                   bend_limit=config.max_wrist_bend_deviation)
        pipeline = core.PipelinePlanner(self.native_node, 'ompl', node.args.planner)
        pipeline.max_velocity_scaling_factor = node.args.velocity
        pipeline.max_acceleration_scaling_factor = node.args.acceleration
        self.solvers += (pipeline,)
        solver = pipeline
        if self._fast_ik:
            direct = core.JointInterpolationPlanner()
            direct.max_velocity_scaling_factor = node.args.velocity
            direct.max_acceleration_scaling_factor = node.args.acceleration
            solver = core.MultiPlanner()
            solver.add(direct)
            solver.add(pipeline)
            self.solvers += (direct, solver)
        stage = stages.MoveTo(f'{node.arm_side} arm yield', solver)
        stage.group = node.group
        stage.timeout = node.args.planning_time
        stage.path_constraints = axial_constraints(node, config, start=node.start if start is None else start)
        stage.setGoal({name: target[name] for name in node.joints})
        stage.setCostTerm(natural.stage_cost)
        task.add(stage)

    def _add_leg(self, task, node, config, geometry, object_id, core, stages, yield_target,
                 *, rear_preparation=False):
        angle = geometry.angle if config.grasp_opening is None else config.grasp_opening*1.1
        from .mtc_natural import NaturalArmPolicy
        natural = NaturalArmPolicy(node, wrist_limit=config.max_wrist_roll_deviation,
                                   swivel_limit=config.max_arm_swivel_deviation,
                                   bend_limit=config.max_wrist_bend_deviation)
        node.mtc_natural_policy = natural
        comfortable = axial_constraints(node, config,
                                        pitch_limit=getattr(config, 'max_arm_pitch_deviation', None))
        pipeline = core.PipelinePlanner(self.native_node, 'ompl', node.args.planner)
        cartesian = core.CartesianPath()
        cartesian.step_size = .003
        gripper = core.JointInterpolationPlanner()
        for solver in (pipeline, cartesian, gripper):
            solver.max_velocity_scaling_factor = node.args.velocity
            solver.max_acceleration_scaling_factor = node.args.acceleration
        self.solvers += (pipeline, cartesian, gripper)
        eef = (node.arm_side+'_' if node.dual else '')+'woan_picker'
        tip = PoseStamped()
        tip.header.frame_id = node.tip_link
        tip.pose.orientation.w = 1.
        offset = geometry.center(angle) if config.tcp_offset is None else config.tcp_offset

        def move_hand(name, target):
            stage = stages.MoveTo(name, gripper)
            stage.group = node.gripper_group
            stage.setGoal(target)
            return stage

        def relative(name, direction, frame, minimum, maximum, *, arrival=False, bend_limit=None, end_bend_limit=None):
            stage = stages.MoveRelative(name, cartesian)
            stage.group, stage.ik_frame = node.group, tip
            stage.min_distance, stage.max_distance = float(minimum), float(maximum)
            vector = Vector3Stamped()
            vector.header.frame_id = frame
            vector.vector.x, vector.vector.y, vector.vector.z = map(float, direction)
            stage.setDirection(vector)
            constraints = (comfortable if bend_limit is None else
                           axial_constraints(node, config, bend_limit=bend_limit,
                               pitch_limit=getattr(config, 'max_arm_pitch_deviation', None)))
            stage.path_constraints = constraints
            spec = {'frame': frame, 'direction': list(direction), 'arrival': arrival}
            policy = natural
            if bend_limit is not None:
                spec.update(bend_limit=bend_limit, constraints=constraints)
                if end_bend_limit is not None:
                    spec['end_bend_limit'] = end_bend_limit
                policy = natural.with_bend_limit(bend_limit, end_limit=end_bend_limit)
            stage.setCostTerm(self._record_motion(self.task, self.motions, policy, spec))
            return stage

        def inverse(name, generator, frame=tip):
            stage = stages.ComputeIK(name, generator)
            stage.group, stage.eef, stage.ik_frame = node.group, eef, frame
            stage.default_pose = 'onero_mtc_natural'
            stage.max_ik_solutions = 3 if self._fast_ik else 12
            stage.min_solution_distance = .1
            stage.timeout = min(.15 if self._fast_ik else 1., node.args.planning_time)
            stage.constraints = comfortable
            stage.properties.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ['target_pose'])
            stage.setCostTerm(natural.stage_cost)
            return stage

        pick_contacts = stages.ModifyPlanningScene('pick support contacts')
        for support in self.supports:
            pick_contacts.allowCollisions(object_id, support, support == config.pick_support)
        open_hand = move_hand('open gripper', 'onero_mtc_open')
        references = []
        approach_touch = None
        if any(getattr(config, 'grasp_position_offset', None) or ()):
            tips = [name for name in geometry.touch_links if name.endswith('finger_link_3')]
            if not tips:
                raise MoveItError('校准抓取需要 Woan Picker 指尖碰撞链接')
            reference = stages.ModifyPlanningScene('calibrated grasp contact reference')
            restore = stages.ModifyPlanningScene('restore collision checks before connection')
            approach_touch = stages.ModifyPlanningScene('allow fingertip contact during approach')
            reference.allowCollisions(object_id, tips, True)
            restore.allowCollisions(object_id, tips, False)
            approach_touch.allowCollisions(object_id, tips, True)
            references = [reference, restore]
        pick_solver = pipeline
        if self._fast_ik or node.arm_side == 'left':
            pick_solver = core.MultiPlanner()
            pick_solver.add(gripper)
            pick_solver.add(pipeline)
            self.solvers += (pick_solver,)
        connect_pick = stages.Connect('move to pick', [(node.group, pick_solver)])
        connect_pick.timeout = node.args.planning_time
        connect_pick.path_constraints = axial_constraints(node, config, start=node.start,
            bend_limit=math.pi/2. if rear_preparation else None,
            pitch_limit=getattr(config, 'max_arm_pitch_deviation', None))
        connect_pick.setCostTerm(natural.stage_cost)
        approach = relative('approach object', config.approach_direction, 'world',
                            config.approach_distance, config.approach_distance, arrival=True)
        generate = core.Alternatives('sample grasp directions')
        first, second = geometry.pads(angle)
        grasp_axis = first-second
        reference_rotation = pose_matrix(node.fk(natural.reference))[:3, :3]
        targets = list(grasp_targets(config, offset, node.args.grasp_angle_step,
                                     grasp_axis=grasp_axis, forward_axis=offset,
                                     reference_rotation=reference_rotation))
        limit = getattr(config, 'grasp_limit', None)
        if limit:
            # The list is sorted by naturalness, so the closest postures are
            # tried first and the rest only cost planning time.
            targets = targets[:limit]
        from .mtc_directions import preferred_targets
        place_yaws = np.deg2rad([0., 30., -30., 60., -60., 90., -90., 180.]).tolist()
        self._grasp_targets, self._place_yaws = targets, place_yaws
        targets, place_yaws = preferred_targets(targets, place_yaws, self._preferred_directions,
                                               pick_yaw=self._pick_yaw)
        if len(place_yaws) == 1:
            print(f'MTC 优先复用合格方案的抓放方向：抓取 {np.rad2deg(targets[0][0]):g}°，'
                  f'放置 {np.rad2deg(place_yaws[0]):g}°；按当前目标重新求解。')
        for index, (yaw, pose) in enumerate(targets):
            target = PoseStamped()
            target.header.frame_id = 'world'
            target.pose = pose
            generator = stages.GeneratePose(f'grasp {index+1} yaw {np.rad2deg(yaw):g}')
            generator.pose = target
            generator.setMonitoredStage(references[0] if references else open_hand)
            generate.add(generator)
        grasp = inverse('grasp IK', generate)
        touch = stages.ModifyPlanningScene('allow finger contact')
        touch.allowCollisions(object_id, geometry.touch_links, True)
        close_hand = move_hand('grasp object', 'onero_mtc_grasp')
        attach = stages.ModifyPlanningScene('attach object')
        attach.attachObject(object_id, node.tip_link)
        place_contacts = stages.ModifyPlanningScene('place support contact')
        place_contacts.allowCollisions(object_id, config.place_support, True)
        if getattr(config, 'place_z_offset', 0.):
            place_contacts.allowCollisions(object_id, 'onero_env_table', True)
        lift = relative('lift object', [0., 0., 1.], 'world', config.lift_distance,
                        max(config.lift_distance, config.transfer_height-config.pick_position[2]),
                        bend_limit=math.pi/2.)
        leave_support = stages.ModifyPlanningScene('leave pick support')
        for support in self.supports:
            leave_support.allowCollisions(object_id, support, False)
        if getattr(config, 'place_z_offset', 0.):
            leave_support.allowCollisions(object_id, 'onero_env_table', False)

        carry_solver = core.MultiPlanner()
        carry_solver.add(gripper)
        carry_solver.add(pipeline)
        self.solvers += (carry_solver,)
        connect_place = stages.Connect('carry to place', [(node.group, carry_solver)])
        connect_place.timeout = node.args.planning_time
        connect_place.path_constraints = axial_constraints(node, config, bend_limit=math.pi/2.,
            pitch_limit=getattr(config, 'max_arm_pitch_deviation', None))
        connect_place.setCostTerm(self._record_motion(self.task, self.motions,
            natural.with_bend_limit(math.pi/2.),
            {'bend_limit': math.pi/2., 'constraints': connect_place.path_constraints}))
        place_position = list(config.place_position)
        place_z_offset = getattr(config, 'place_z_offset', 0.)
        place_position[2] += place_z_offset
        lower = relative('lower object', [0., 0., -1.], 'world', config.lift_distance,
                         max(config.lift_distance, config.transfer_height-place_position[2]), arrival=True,
                         bend_limit=math.pi/2., end_bend_limit=config.max_wrist_bend_deviation)
        # Composite bottle shapes get no yaw freedom from GeneratePlacePose.
        # Explicit upright poses retain the full bottle collision geometry.
        generate_place = core.Alternatives('upright place directions')
        for yaw in place_yaws:
            degrees = float(np.rad2deg(yaw))
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.orientation.z = float(np.sin(np.deg2rad(degrees)/2.))
            pose.pose.orientation.w = float(np.cos(np.deg2rad(degrees)/2.))
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = place_position
            pose.pose.position.z += config.grasp_height
            generator = stages.GeneratePose(f'place yaw {degrees:g}')
            generator.pose = pose
            generator.setMonitoredStage(place_contacts)
            bound = getattr(self, '_place_reach_bound', None)
            if bound is not None:
                generator.setCostTerm(bound.object_cost(object_id, pose))
            generate_place.add(generator)
        object_frame = PoseStamped()
        object_frame.header.frame_id = object_id
        object_frame.pose.orientation.w = 1.
        place = inverse('place IK', generate_place, object_frame)
        release = move_hand('release object', 'onero_mtc_open')
        detach = stages.ModifyPlanningScene('detach object')
        detach.detachObject(object_id, node.tip_link)
        pick = core.SerialContainer('pick')
        if approach_touch is not None:
            pick.add(approach_touch)
        for stage in (approach, grasp, touch, close_hand, attach, place_contacts, lift, leave_support):
            pick.add(stage)
        place_container = core.SerialContainer('place')
        surface = stages.ModifyPlanningScene('allow place support contact')
        surface.allowCollisions(object_id, config.place_support, True)
        if place_z_offset:
            # Only the bottle's calibrated placement may cross these model
            # surfaces; robot links retain their normal collision checks.
            surface.allowCollisions(object_id, 'onero_env_table', True)
        for stage in (surface, lower, place, release, detach):
            place_container.add(stage)
        self._add_departure(place_container, node, config, geometry, object_id,
                            yield_target, core, stages, self.task, self.motions)
        if rear_preparation:
            self._add_rear_preparation(task, node, config, geometry, core, stages)
        for stage in (pick_contacts, open_hand, *references, connect_pick, pick, connect_place, place_container):
            task.add(stage)

    @staticmethod
    def _add_calibrated_release(parent, config, object_id, stages):
        if not getattr(config, 'place_z_offset', 0.):
            return
        from copy import deepcopy
        from .mtc_pick_place import bottle_object
        released = deepcopy(config)
        released.pick_position = list(config.place_position)
        normalize = stages.ModifyPlanningScene('calibrated released bottle pose')
        normalize.addObject(bottle_object(released, object_id))
        parent.add(normalize)

    def _add_departure(self, parent, node, config, geometry, object_id, yield_target,
                       core, stages, root, motions, *, start=None):
        natural = node.mtc_natural_policy
        cartesian = core.CartesianPath()
        cartesian.step_size = .003
        cartesian.max_velocity_scaling_factor = node.args.velocity
        cartesian.max_acceleration_scaling_factor = node.args.acceleration
        self.solvers += (cartesian,)
        tip = PoseStamped()
        tip.header.frame_id = node.tip_link
        tip.pose.orientation.w = 1.

        def retreat(name, direction, frame, distance, maximum=None, *, raised=False):
            stage = stages.MoveRelative(name, cartesian)
            stage.group, stage.ik_frame = node.group, tip
            stage.min_distance = float(distance)
            stage.max_distance = float(distance if maximum is None else maximum)
            vector = Vector3Stamped()
            vector.header.frame_id = frame
            vector.vector.x, vector.vector.y, vector.vector.z = map(float, direction)
            stage.setDirection(vector)
            stage.path_constraints = axial_constraints(node, config, start=node.start if start is None else start,
                                                       bend_limit=math.pi/2. if raised else None)
            policy = natural.with_bend_limit(math.pi/2.) if raised else natural
            spec = {'frame': frame, 'direction': list(direction), 'arrival': False}
            if raised:
                spec.update(bend_limit=math.pi/2., constraints=stage.path_constraints)
            stage.setCostTerm(self._record_motion(root, motions, policy, spec))
            parent.add(stage)

        if yield_target is not None:
            # Retreat along the gripper's physical forward axis with its pose
            # held fixed; permit contact only during this deterministic escape.
            allow = stages.ModifyPlanningScene('allow clearance contact')
            for base, plate in self.mounting_contacts:
                allow.allowCollisions(base, plate, True)
            allow.allowCollisions(object_id, geometry.touch_links, True)
            parent.add(allow)
            angle = geometry.angle if config.grasp_opening is None else config.grasp_opening*1.1
            forward = np.asarray(geometry.center(angle), dtype=float)
            if np.linalg.norm(forward) < 1e-6:
                raise MoveItError('无法确定夹爪后退方向')
            distance = config.retreat_distance+config.diameter/2.+config.position_tolerance
            retreat('clear bottle before yield', -forward/np.linalg.norm(forward), node.tip_link,
                    config.retreat_distance, distance)
        untouch = stages.ModifyPlanningScene('restore finger collision checks')
        for base, plate in self.mounting_contacts:
            untouch.allowCollisions(base, plate, True)
        untouch.allowCollisions(object_id, geometry.touch_links, False)
        if getattr(config, 'place_z_offset', 0.) and config.place_support != 'onero_env_table':
            untouch.allowCollisions(object_id, 'onero_env_table', False)
        calibrated_departure = bool(getattr(config, 'place_z_offset', 0.)) and yield_target is None
        if calibrated_departure:
            # The height calibration offsets the modelled bottle from the
            # gripper, so opened fingers and the palm may overlap it during the
            # retreat. Collision checks are restored right after the lift.
            clearance = stages.ModifyPlanningScene('allow calibrated release clearance')
            clearance.allowCollisions(object_id, geometry.touch_links, True)
            parent.add(clearance)
        else:
            parent.add(untouch)
        if yield_target is not None:
            self._add_yield(parent, node, config, yield_target, core, stages, start=start)
        else:
            distance = config.retreat_distance
            if calibrated_departure:
                distance += abs(config.place_z_offset)+config.position_tolerance
            retreat('retreat gripper', [0., 0., 1.], 'world', distance, raised=True)
            if calibrated_departure:
                self._add_calibrated_release(parent, config, object_id, stages)
                parent.add(untouch)

    def _motion_task(self, name):
        import moveit.core.robot_model  # Register the shared RobotModel binding.
        from moveit.task_constructor import core, stages
        task = core.Task('onero_mtc_feedback_'+uuid.uuid4().hex[:8])
        task.name = name
        task.setRobotModel(self.task.getRobotModel())
        current = stages.CurrentState('measured current state')
        current.timeout = 5.
        task.add(current)
        return task, core, stages

    def plan_departure(self, node, config, geometry, object_id, *, yield_target=None):
        from .task_support import task_state
        task, core, stages = self._motion_task('measured release departure')
        motions = {}
        current = task_state(node)
        self._add_departure(task, node, config, geometry, object_id, yield_target,
                            core, stages, task, motions, start=current)
        task.init()
        solution = self._plan_task(task, 1, node.args.planning_time*2., timeout_option='--planning-time')
        return MtcMotionPlan(solution.toMsg(task.introspection()), motions,
                             axial_constraints(node, config, start=current))

    def plan_motion(self, node, *, target=None, direction=None, frame=None, distance=None,
                    constraints=None, cartesian=False, contacts=(), object_id=None, prefer_direct=False):
        task, core, stages = self._motion_task('feedback motion')
        motions = {}
        previous = []
        if contacts:
            matrix = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
                components=PlanningSceneComponents(
                    components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))).scene.allowed_collision_matrix
            entries = {name: dict(zip(matrix.entry_names, row.enabled))
                       for name, row in zip(matrix.entry_names, matrix.entry_values)}
            defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
            allow = stages.ModifyPlanningScene('allow correction support contact')
            for base, plate in self.mounting_contacts:
                allow.allowCollisions(base, plate, True)
            for support in contacts:
                previous.append((support, entries.get(object_id, {}).get(support,
                                 defaults.get(object_id, defaults.get(support, False)))))
                allow.allowCollisions(object_id, support, True)
            task.add(allow)
        solver = core.CartesianPath() if cartesian or direction is not None else core.PipelinePlanner(
            self.native_node, 'ompl', node.args.planner)
        solver.max_velocity_scaling_factor = node.args.velocity
        solver.max_acceleration_scaling_factor = node.args.acceleration
        if prefer_direct and not cartesian and direction is None and isinstance(target, dict):
            direct = core.JointInterpolationPlanner()
            direct.max_velocity_scaling_factor = node.args.velocity
            direct.max_acceleration_scaling_factor = node.args.acceleration
            alternatives = core.MultiPlanner()
            alternatives.add(direct)
            alternatives.add(solver)
            solver = alternatives
        tip = PoseStamped()
        tip.header.frame_id = node.tip_link
        tip.pose.orientation.w = 1.
        if cartesian or direction is not None:
            solver.step_size = .003
        if direction is not None:
            stage = stages.MoveRelative('measured relative motion', solver)
            stage.ik_frame = tip
            stage.min_distance = stage.max_distance = float(distance)
            vector = Vector3Stamped()
            vector.header.frame_id = frame
            vector.vector.x, vector.vector.y, vector.vector.z = map(float, direction)
            stage.setDirection(vector)
            stage.setCostTerm(self._record_motion(task, motions, node.mtc_natural_policy,
                {'frame': frame, 'direction': list(direction), 'arrival': False}))
        else:
            stage = stages.MoveTo('measured goal motion', solver)
            if isinstance(target, dict):
                stage.setGoal({name: target[name] for name in node.joints})
            else:
                pose = PoseStamped()
                pose.header.frame_id = node.args.frame
                pose.pose = target
                stage.ik_frame = tip
                stage.setGoal(pose)
            if cartesian:
                stage.setCostTerm(self._record_motion(task, motions, node.mtc_natural_policy,
                    {'frame': node.args.frame, 'direction': [], 'arrival': True}))
            else:
                stage.setCostTerm(node.mtc_natural_policy.stage_cost)
        stage.group, stage.timeout = node.group, node.args.planning_time
        if constraints is not None:
            stage.path_constraints = constraints
        task.add(stage)
        if previous:
            restore = stages.ModifyPlanningScene('restore correction support contact')
            for support, enabled in previous:
                restore.allowCollisions(object_id, support, enabled)
            task.add(restore)
        task.init()
        solution = self._plan_task(task, 1, node.args.planning_time, timeout_option='--planning-time')
        message = solution.toMsg(task.introspection())
        restore = (PlanningScene(allowed_collision_matrix=message.sub_trajectory[-1].scene_diff.allowed_collision_matrix)
                   if previous else None)
        return MtcMotionPlan(message, motions, constraints, restore)

    def plan(self, max_solutions, timeout, *, select=None):
        if self._fast_ik:
            started = time.monotonic()
            try:
                solution = self._plan_task(self.task, max_solutions, min(8., timeout*.25))
                return select(self) if select is not None else solution
            except MoveItError as exc:
                print(f'MTC 快速搜索未得到合格完整方案，自动恢复完整搜索：{exc}', flush=True)
                exc.__traceback__ = None
            self._fast_ik = False
            task_name = self.task.name
            self.solvers, self.task, self.native_node = (), None, None
            self._build(*self._recipe)
            self.task.name = task_name
            timeout -= time.monotonic()-started
            if timeout <= 0.:
                raise MoveItError('MTC 搜索时间已到；请增加 --task-timeout')
        return self._plan_full(max_solutions, timeout, select=select)

    def remember_directions(self, message):
        from .mtc_directions import save_directions, solution_directions
        node, _, _, object_id, *_ = self._recipe
        try:
            directions = solution_directions(node, message, object_id, self._grasp_targets, self._place_yaws)
            save_directions(node, self.route, directions)
        except (ValueError, RuntimeError) as exc:
            print(f'MTC 抓放方向记录未更新：{exc}；不影响本次方案。')

    def _plan_full(self, max_solutions, timeout, *, select=None):
        def plan():
            solution = self._plan_task(self.task, max_solutions, timeout)
            return select(self) if select is not None else solution

        if self.initial_clearance is None or self._using_initial_clearance:
            return plan()
        print('MTC 优先保持左臂原位规划接力。')
        try:
            return plan()
        except MoveItError as exc:
            # No planning scene effects have been executed. Retry as another
            # complete MTC recipe, not an extra motion added by the executor.
            exc.__traceback__ = None
        print('MTC 原位方案未找到完整解，尝试左臂局部直线退开。')
        task_name = getattr(self.task, 'name', None)
        self.solvers, self.task, self.native_node = (), None, None
        self._using_initial_clearance = True
        self._build(*self._recipe, initial_clearance=self.initial_clearance)
        if task_name is not None:
            self.task.name = task_name
        return plan()

    @staticmethod
    def _plan_task(task, max_solutions, timeout, *, timeout_option='--task-timeout'):
        expired = threading.Event()
        finished = threading.Event()
        started = time.monotonic()
        diagnostics = None
        if timeout_option == '--task-timeout':
            from .mtc_diagnostics import PlanningDiagnostics
            try:
                diagnostics = PlanningDiagnostics(task.name, started)
            except Exception as exc:
                print(f'MTC 阶段统计未启动：{exc}')

        def report():
            while not finished.wait(5.):
                print(f'MTC 正在搜索方案：已用 {time.monotonic()-started:.0f}/{timeout:g} s。', flush=True)

        def stop():
            expired.set()
            task.preempt()

        timer = threading.Timer(timeout, stop)
        timer.daemon = True
        progress = threading.Thread(target=report, name='mtc-planning-progress', daemon=True)
        print(f'MTC 开始规划：最多 {max_solutions} 条方案，时限 {timeout:g} s。', flush=True)
        timer.start()
        progress.start()
        try:
            task.plan(max_solutions)
        except KeyboardInterrupt:
            task.preempt()
            raise
        finally:
            finished.set()
            progress.join()
            timer.cancel()
            timer.join()
            elapsed = time.monotonic()-started
            if diagnostics is not None:
                try:
                    diagnostics.close(elapsed)
                except Exception as exc:
                    print(f'MTC 阶段统计未完成：{exc}；不影响规划结果。')
        print(f'MTC 规划耗时 {elapsed:.1f} s。', flush=True)
        if not len(task.solutions):
            for line in MtcPlanner._failure_report(task):
                print(line, flush=True)
            detail = '搜索时间已到' if expired.is_set() else '未找到完整方案'
            raise MoveItError(f'MTC {detail}；请核对起点、瓶子位置、抓取朝向或增加 {timeout_option}')
        return task.solutions[0]

    @staticmethod
    def _failure_report(task, limit=4):
        """Name the blocked stages and their first failure comments."""
        def children(stage):
            # Task wraps its stages, so probe indexing instead of isinstance().
            result = []
            while len(result) < 64:
                try:
                    child = stage[len(result)]
                except (IndexError, TypeError):
                    break
                if child is None:
                    break
                result.append(child)
            return result

        blocked, empty = [], []

        def walk(stage, path):
            label = stage.name or ''
            name = f'{path} / {label}' if path and label else (label or path)
            if len(stage.solutions):
                return
            failures = stage.failures
            if failures:
                comments = []
                for failure in failures:
                    try:
                        value = failure.comment
                        comment = str(value() if callable(value) else value).strip()
                    except (AttributeError, TypeError):
                        continue
                    if comment and comment not in comments:
                        comments.append(comment)
                    if len(comments) == 2:
                        break
                line = f'  阻塞阶段 {name}：{len(failures)} 次尝试失败'
                if comments:
                    line += '，原因：' + '；'.join(comments)
                blocked.append(line)
                return
            sub = children(stage)
            if not sub:
                empty.append(name)
                return
            for child in sub:
                walk(child, name)
                if len(blocked) >= limit:
                    return

        walk(task, '')
        if blocked:
            return ['MTC 未找到完整方案，阻塞在：', *blocked[:limit]]
        if empty:
            return ['MTC 未找到完整方案，阻塞在：',
                    f'  阻塞阶段 {empty[0]}：无可行结果（检查碰撞、构型约束或取放点）']
        return ['MTC 未找到完整方案：未记录失败阶段，请查看上方的阶段树。']

    def close(self):
        self.solvers = ()
        self.task = None
        self.native_node = None
        self.directory.cleanup()
        if self._context_ready:
            self._context_ready = False
            self.rclcpp.shutdown()
