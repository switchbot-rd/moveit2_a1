"""Execute an MTC bottle solution with measured grasp and release checks."""
from copy import deepcopy
import math

import numpy as np
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene
from onero_a1_common.gains import GainClient
from onero_a1_common.moveit_client import seconds
from onero_a1_common.payload import PayloadClient
from onero_a1_common.trajectory_tools import validate_trajectory

from .bottle_scene import clear_bottle, matrix_pose, pose_matrix, remove_collision_entry
from .mtc_planning import MtcMotionPlan, axial_constraints
from .support import MoveItError
from .task_support import confirm_opening, grasp_bottle, task_state, wait_for_arm_settled


def placement_support_gap(scene, config, position, support):
    height = scene.support_height(position, config.diameter/2., object_id=support)
    return None if height is None else float(position[2]-config.bottle_height/2.-height)


def reset_task(node):
    """Operator-confirmed recovery after removing the physical bottle."""
    if not node.args.execute:
        raise MoveItError('确认实物水瓶已取下后，使用 --reset-task --execute 恢复任务')
    actual = task_state(node)[node.gripper_joint]
    if not math.isfinite(actual) or abs(actual-1.1) > .015:
        raise MoveItError('恢复前请先托住水瓶，将夹爪完全张开并取下实物，再运行 --reset-task --execute')
    bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
    PayloadClient(node, bridge).restore_baseline()
    clear_bottle(node, 'onero_mtc_bottle_'+node.arm_side)
    print('MTC 任务已恢复：负载已恢复启动基线，水瓶模型已清理；可重新运行抓放。')


class MtcExecutor:
    def __init__(self, node, config, geometry, scene, *, mass=.1, simulated=False,
                 planner=None, motions=None, yield_target=None, release_transform=None):
        if not math.isfinite(mass) or mass <= 0.:
            raise MoveItError('水瓶质量必须是大于零的有限数值')
        self.node, self.config, self.geometry, self.scene = node, config, geometry, scene
        self.mass, self.simulated = float(mass), simulated
        self.planner, self.motions, self.yield_target = planner, motions or {}, yield_target
        self.release_transform = release_transform
        self.holding = self.completed = False
        self.gains = self.payload = None
        self._gains_changed = self._payload_changed = False
        self._grasp_confirmed = self._release_confirmed = self._started = False
        self._tip_to_object = self._released_transform = None
        self._release_clearance_complete = False
        self._close_angle = (geometry.angle if config.grasp_opening is None
                             else config.grasp_opening*1.1)
        self._offset = np.asarray(config.tcp_offset if config.tcp_offset is not None
                                  else geometry.center(self._close_angle), dtype=float)

    def _check_pick(self):
        flange = pose_matrix(self.node.fk(task_state(self.node)))
        tcp = flange[:3, 3]+flange[:3, :3] @ self._offset
        target = np.asarray(self.config.pick_position)+[0., 0., self.config.grasp_height]
        target = target+np.asarray(getattr(self.config, 'grasp_position_offset', None) or [0., 0., 0.])
        error = float(np.linalg.norm(tcp-target))
        if error > self.config.position_tolerance:
            raise MoveItError(f'夹取位置反馈超差 {error*1000:.1f} mm，尚未闭爪')
        print(f'夹取位置反馈：TCP 偏差 {error*1000:.1f} mm。')

    def _placement(self, state):
        if self._tip_to_object is None or not self.scene.attached:
            raise MoveItError('放置前没有已确认的水瓶附着模型，保持夹持')
        transform = pose_matrix(self.node.fk(state)) @ self._tip_to_object
        # The configured correction maps the model's lowered placement back
        # to the estimated physical bottle pose used by the relay and checks.
        transform[2, 3] -= getattr(self.config, 'place_z_offset', 0.)
        center = transform[:3, 3]-transform[:3, 2]*self.config.grasp_height
        error = center-np.asarray(self.config.place_position)
        tilt = math.acos(float(np.clip(transform[2, 2], -1., 1.)))
        reported = self.release_transform(transform) if self.release_transform is not None else transform
        physical_center = reported[:3, 3]-reported[:3, 2]*self.config.grasp_height
        gap = placement_support_gap(self.scene, self.config, physical_center, self.config.place_support)
        valid = (np.linalg.norm(error) <= self.config.position_tolerance
                 and abs(error[2]) <= self.config.placement_height_tolerance
                 and gap is not None and abs(gap) <= self.config.placement_height_tolerance
                 and tilt <= self.config.orientation_tolerance)
        return transform, error, tilt, bool(valid), gap

    def _check_place(self):
        transform, error, tilt, valid, gap = self._placement(task_state(self.node))
        support = (f'未找到支撑面 {self.config.place_support}（模型缺失或瓶底不满足 5 mm 边缘余量）'
                   if gap is None else f'瓶底距支撑面 {gap*1000:+.1f} mm')
        detail = (f'XYZ 偏差 [{error[0]*1000:+.1f}, {error[1]*1000:+.1f}, '
                  f'{error[2]*1000:+.1f}] mm，倾斜 {tilt:.4f} rad，{support}')
        if not valid:
            raise MoveItError(f'放置反馈超差（模型 FK）：{detail}；保持夹持，不松爪')
        print(f'放置反馈（模型 FK）：{detail}。')
        return transform

    def _wait_for_placement(self, planned):
        """Correct bounded tracking residuals; retain all release tolerances."""
        wait_for_arm_settled(self.node)
        command = pose_matrix(self.node.fk(planned))
        command[:3, 3] -= self._placement(planned)[1]
        nominal = command[:3, 3].copy()
        previous = None
        for attempt in range(3):
            current = task_state(self.node)
            transform, error, tilt, valid, gap = self._placement(current)
            norm = float(np.linalg.norm(error))
            # A correction that does not measurably improve the fresh feedback
            # is chasing servo noise or support contact; stop instead of
            # commanding yet another offset that can end further away.
            improved = previous is None or norm < previous-.0005
            if (valid or attempt == 2 or norm > .03 or not improved
                    or tilt > self.config.orientation_tolerance):
                return self._check_place()
            if gap is None:
                # A held bottle can lose the 5 mm release margin without its
                # footprint leaving the plate. Permit only an inward correction
                # from full physical support toward a target with the full margin.
                reported = self.release_transform(transform) if self.release_transform is not None else transform
                center = reported[:3, 3]-reported[:3, 2]*self.config.grasp_height
                radius, support = self.config.diameter/2., self.config.place_support
                # Conservatively include the tilted body's horizontal projection.
                footprint = radius+self.config.bottle_height/2.*math.sin(tilt)
                if (self.scene.support_height(center, footprint, object_id=support, margin=0.) is None
                        or self.scene.support_height(center-error, footprint, object_id=support) is None):
                    return self._check_place()
            previous = norm
            # Use a half-step: full compensation can cross the support and
            # reverse the tracking error when the loaded arm settles.
            correction = -.5*error
            command[:3, 3] += correction
            if np.linalg.norm(command[:3, 3]-nominal) > .03:
                raise MoveItError('放置反馈修正累计超过 30 mm，保持夹持，不松爪')
            print(f'放置反馈修正 {attempt+1}/2：补偿 XYZ '
                  f'[{correction[0]*1000:+.1f}, {correction[1]*1000:+.1f}, '
                  f'{correction[2]*1000:+.1f}] mm。')
            policy = getattr(self.node, 'mtc_natural_policy', None)
            if policy is None:
                raise MoveItError('MTC 自然构型检查尚未就绪，停止放置修正')
            constraints = axial_constraints(self.node, self.config, start=current)
            constraints.orientation_constraints.extend(self.node.upright_constraints(
                self._tip_to_object, self.config.orientation_tolerance).orientation_constraints)
            if self.planner is None:
                raise MoveItError('放置反馈修正需要 MTC 规划器，保持夹持')
            supports = [name for name in (self.config.place_support, 'onero_env_table') if name]
            plan = self.planner.plan_motion(self.node, target=matrix_pose(command), cartesian=True,
                constraints=constraints, contacts=supports, object_id=self.scene.id)
            self.execute_motion(plan)
            wait_for_arm_settled(self.node)

    def _load(self):
        flange = pose_matrix(self.node.fk(task_state(self.node)))
        cog = flange[:3, :3].T @ (np.asarray(self.config.pick_position)-flange[:3, 3])
        self._payload_changed = True
        self.payload.load_object(self.mass, cog.tolist())
        print(f'已确认夹持，搬运 kp={self.config.carry_kp:g}，物体质量={self.mass:g} kg。')

    def _restore(self, *, restore_gains=True):
        if self.holding:
            return
        if self._payload_changed:
            self.payload.restore_baseline()
            self._payload_changed = False
        if restore_gains and self._gains_changed:
            self.gains.restore()
            self._gains_changed = False
            wait_for_arm_settled(self.node)

    def _collision_matrix(self, source):
        """Merge the task object's permissions without replacing other pairs."""
        object_id = self.scene.id
        matrix = self.scene.read().allowed_collision_matrix
        # These pairs describe the fixed physical mounting, not unrelated
        # task contacts. Preserve the native MTC assembly effects as well.
        for first, second in getattr(self.planner, 'mounting_contacts', ()):
            if first not in source.entry_names or second not in source.entry_names:
                continue
            for name in (first, second):
                if name not in matrix.entry_names:
                    for row in matrix.entry_values:
                        row.enabled.append(False)
                    matrix.entry_names.append(name)
                    matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
            allowed = source.entry_values[source.entry_names.index(first)].enabled[source.entry_names.index(second)]
            one, two = matrix.entry_names.index(first), matrix.entry_names.index(second)
            matrix.entry_values[one].enabled[two] = matrix.entry_values[two].enabled[one] = allowed
        remove_collision_entry(matrix, object_id)
        if object_id in source.default_entry_names:
            index = list(source.default_entry_names).index(object_id)
            matrix.default_entry_names.append(object_id)
            matrix.default_entry_values.append(source.default_entry_values[index])
        if object_id not in source.entry_names:
            return matrix
        own = list(source.entry_names).index(object_id)
        names = list(matrix.entry_names)
        for name in source.entry_names:
            if name == object_id or name in names:
                continue
            for row in matrix.entry_values:
                row.enabled.append(False)
            names.append(name)
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(names)))
        permitted = dict(zip(source.entry_names, source.entry_values[own].enabled))
        for name, row in zip(names, matrix.entry_values):
            row.enabled.append(permitted.get(name, False))
        matrix.entry_names = names+[object_id]
        matrix.entry_values.append(AllowedCollisionEntry(
            enabled=[permitted.get(name, False) for name in names]+[permitted.get(object_id, False)]))
        return matrix

    def _attachment(self, attached, planned):
        if attached.link_name != self.node.tip_link:
            raise MoveItError('MTC 水瓶必须附着于所选手臂的 Link7')
        result = deepcopy(attached)
        frame = result.object.header.frame_id or attached.link_name
        transform = pose_matrix(result.object.pose)
        if frame != self.node.tip_link:
            world_tip = pose_matrix(self.node.fk(planned))
            world_frame = (np.eye(4) if frame == self.node.args.frame
                           else pose_matrix(self.node.fk(planned, link=frame)))
            transform = np.linalg.inv(world_tip) @ world_frame @ transform
        result.object.header.frame_id = self.node.tip_link
        result.object.pose = matrix_pose(transform)
        return result, transform

    def _apply_diff(self, source, planned):
        # Planned end-state joints belong only to the solution. Applying them
        # to move_group would replace measured feedback and unrelated objects.
        diff = PlanningScene(is_diff=True)
        diff.robot_state.is_diff = True
        attached, detached, tip_to_object = False, False, None
        for item in source.robot_state.attached_collision_objects:
            if item.object.id != self.scene.id:
                continue
            if item.object.operation == CollisionObject.REMOVE:
                if not self._release_confirmed:
                    raise MoveItError('MTC 试图在未确认松爪时分离水瓶，保持夹持')
                diff.robot_state.attached_collision_objects.append(deepcopy(item))
                detached = True
            else:
                normalized, candidate = self._attachment(item, planned)
                if self.scene.attached:
                    # Motion diffs can repeat the current attached body,
                    # including on the opening trajectory before detachment.
                    if self._tip_to_object is None or not np.allclose(
                            candidate, self._tip_to_object, atol=1e-6, rtol=0.):
                        drift = (float('inf') if self._tip_to_object is None else
                                 float(np.max(np.abs(candidate-self._tip_to_object))))
                        raise MoveItError(
                            f'MTC 意外更改了持物变换（{drift*1000:.2f} mm），停止后续动作')
                    continue
                if not self._grasp_confirmed or self._release_confirmed:
                    raise MoveItError('MTC 试图在未确认夹取时附着水瓶')
                tip_to_object = candidate
                diff.robot_state.attached_collision_objects.append(normalized)
                attached = True
        for item in source.world.collision_objects:
            if (item.id != self.scene.id or item.operation == CollisionObject.REMOVE
                    and (attached or self.scene.attached and not detached)):
                continue
            obj = deepcopy(item)
            calibrated = bool(getattr(self.config, 'place_z_offset', 0.))
            if (self._released_transform is not None and obj.operation != CollisionObject.REMOVE
                    and (not calibrated or self._release_clearance_complete)):
                obj.header.frame_id = self.node.args.frame
                obj.pose = matrix_pose(self._released_transform)
            diff.world.collision_objects.append(obj)
        matrix = source.allowed_collision_matrix
        if matrix.entry_names or matrix.default_entry_names:
            diff.allowed_collision_matrix = self._collision_matrix(matrix)
        diff.fixed_frame_transforms = deepcopy(source.fixed_frame_transforms)
        diff.object_colors = [deepcopy(item) for item in source.object_colors if item.id == self.scene.id]
        if not (diff.robot_state.attached_collision_objects or diff.world.collision_objects
                or diff.allowed_collision_matrix.entry_names or diff.allowed_collision_matrix.default_entry_names
                or diff.fixed_frame_transforms or diff.object_colors):
            return
        self.scene.apply(diff)
        self.scene.changed = True
        if attached:
            self.scene.attached = True
            self._tip_to_object = self.scene.tip_to_object = tip_to_object
        elif detached:
            self.scene.attached = False
            if self.scene.bottle is not None and self._released_transform is not None:
                self.scene.bottle.header.frame_id = self.node.args.frame
                self.scene.bottle.pose = matrix_pose(self._released_transform)
                if getattr(self.config, 'place_z_offset', 0.) and not self._release_clearance_complete:
                    self.scene.bottle.pose.position.z += self.config.place_z_offset

    def _stopped_arm_trajectory(self, trajectory):
        """Each separately executed MTC arm segment starts and ends at rest."""
        path = trajectory.joint_trajectory
        boundaries = (path.points[0], path.points[-1])
        derivatives = [value for point in boundaries
                       for values in (point.velocities, point.accelerations) for value in values]
        if not all(map(math.isfinite, derivatives)):
            raise MoveItError('MTC 轨迹停止边界含无效速度或加速度')
        if not any(derivatives):
            print(f'本段首尾已静止：{seconds(path.points[-1].time_from_start):.2f} s。')
            return trajectory
        result = deepcopy(trajectory)
        for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
            point.velocities = [0.]*len(path.joint_names)
            point.accelerations = [0.]*len(path.joint_names)
        current = task_state(self.node)
        policy = getattr(self.node, 'mtc_natural_policy', None)
        if policy is None:
            raise MoveItError('MTC 自然构型检查尚未就绪，停止轨迹处理')
        previous = getattr(self.node, '_motion_path_constraints', None)
        self.node._motion_path_constraints = (deepcopy(previous) if previous is not None else
                                             axial_constraints(self.node, self.config, start=current))
        self.node.planned_start_state = self.node.robot_state({
            **current, **dict(zip(path.joint_names, path.points[0].positions))})
        try:
            # Recheck limits and the controller's interpolation after changing
            # derivatives; preserve the native task's geometry and endpoints.
            # Keep the planner's sampling. Resampling its quintic curve before
            # Ruckig preserves any spikes and makes duration changes at dense
            # waypoints produce much larger interpolation accelerations.
            smoothing = getattr(self.node.args, 'smoothing', 'none')
            if smoothing == 'none':
                validate_trajectory(self.node, result)
            else:
                result = self.node.smooth_trajectory(result, jerk=self.config.smoothing_jerk)
            for point in (result.joint_trajectory.points[0], result.joint_trajectory.points[-1]):
                if any(not math.isfinite(v) or abs(v) > 1e-9
                       for values in (point.velocities, point.accelerations) for v in values):
                    raise MoveItError('MTC 轨迹未保持停止边界，停止执行')
            policy.validate_path(current, result)
            self.node.motion_posture_cost(current, result)
            if smoothing == 'none':
                print(f'本段平滑已关闭：保留规划时长 '
                      f'{seconds(result.joint_trajectory.points[-1].time_from_start):.2f} s。')
            else:
                print(f'本段平滑：{seconds(path.points[-1].time_from_start):.2f} s → '
                      f'{seconds(result.joint_trajectory.points[-1].time_from_start):.2f} s'
                      f'（{len(path.points)}→{len(result.joint_trajectory.points)} 点，'
                      f'jerk={self.config.smoothing_jerk:g}）。')
            return result
        finally:
            self.node._motion_path_constraints = previous

    def _arm_trajectory(self, sub, *, motions=None, constraints=None):
        """Replan a stale segment from feedback while retaining its task goal."""
        trajectory = sub.trajectory
        path = trajectory.joint_trajectory
        current = task_state(self.node)
        error = max(abs(current[name]-position)
                    for name, position in zip(path.joint_names, path.points[0].positions))
        # MoveIt rejects execution when the start deviates beyond
        # allowed_start_tolerance (0.01 rad). Keep half of that margin so a
        # lagging measured state cannot turn a fresh trajectory into a reject.
        if error <= .005:
            return trajectory
        print(f'阶段起点跟踪偏差 {error:.4f} rad，由 MTC 按实测状态重新规划本段。')
        if self.planner is None:
            raise MoveItError('跟踪偏差需要 MTC 重新规划，停止后续动作')
        target = {**current, **dict(zip(path.joint_names, path.points[-1].positions))}
        constraints = (deepcopy(constraints) if constraints is not None else
                       axial_constraints(self.node, self.config, start=current))
        previous = getattr(self.node, '_motion_path_constraints', None)
        self.node._motion_path_constraints = constraints
        self.node.planned_start_state = self.node.robot_state(current)
        try:
            specs = self.motions if motions is None else motions
            spec = specs.get(sub.info.stage_id)
            if spec is not None and 'constraints' in spec:
                constraints = deepcopy(spec['constraints'])
            relative = spec is not None and 'direction' in spec
            if relative and not spec['arrival']:
                before = self.node.fk({**current, **dict(zip(path.joint_names, path.points[0].positions))})
                after = self.node.fk(target)
                distance = float(np.linalg.norm(pose_matrix(after)[:3, 3]-pose_matrix(before)[:3, 3]))
                plan = self.planner.plan_motion(self.node, direction=spec['direction'], frame=spec['frame'],
                                               distance=distance, constraints=constraints)
            else:
                cartesian = relative
                if not cartesian and 'CartesianPath' in sub.info.planner_id+' '+sub.info.comment:
                    raise MoveItError('MTC 相对运动信息缺失，停止重规划')
                goal = self.node.fk(target) if cartesian else target
                planner_id = sub.info.planner_id+' '+sub.info.comment
                options = ({'prefer_direct': True} if any(name in planner_id for name in
                           ('JointInterpolationPlanner', 'MultiPlanner')) else {})
                plan = self.planner.plan_motion(self.node, target=goal, cartesian=cartesian,
                                               constraints=constraints, **options)
            paths = [part.trajectory for part in plan.message.sub_trajectory
                     if part.trajectory.joint_trajectory.points]
            if len(paths) != 1 or set(paths[0].joint_trajectory.joint_names) != set(self.node.joints):
                raise MoveItError('MTC 单段重规划未返回完整的单臂轨迹')
            result = paths[0]
            self.node.planned_start_state = plan.message.start_scene.robot_state
            policy = getattr(self.node, 'mtc_natural_policy', None)
            if policy is None:
                raise MoveItError('MTC 自然构型检查尚未就绪，停止重规划')
            policy.validate_path(current, result)
            self.node.motion_posture_cost(current, result)
            return result
        finally:
            self.node._motion_path_constraints = previous

    def _execute_arm(self, sub, controller, *, motions=None, constraints=None):
        specs = self.motions if motions is None else motions
        spec = specs.get(sub.info.stage_id, {})
        previous_policy = getattr(self.node, 'mtc_natural_policy', None)
        previous_constraints = getattr(self.node, '_motion_path_constraints', None)
        constraints = spec.get('constraints', constraints)
        if 'bend_limit' in spec:
            if previous_policy is None:
                raise MoveItError('抬升阶段缺少自然构型检查')
            self.node.mtc_natural_policy = previous_policy.with_bend_limit(
                spec['bend_limit'], end_limit=spec.get('end_bend_limit'))
        if constraints is not None:
            self.node._motion_path_constraints = constraints
        try:
            trajectory = self._arm_trajectory(sub, motions=specs, constraints=constraints)
            trajectory = self._stopped_arm_trajectory(trajectory)
            self.node.tracking_label = sub.info.comment or f'MTC stage {sub.info.stage_id}'
            self.node.execute(trajectory, controller=controller)
            if self._release_confirmed and not self.scene.attached:
                self._release_clearance_complete = True
        finally:
            self.node.mtc_natural_policy = previous_policy
            self.node._motion_path_constraints = previous_constraints

    def execute_motion(self, plan: MtcMotionPlan):
        """Execute an MTC helper solution and its ordered scene effects."""
        message = plan.message
        self.node.planned_start_state = message.start_scene.robot_state
        state = message.start_scene.robot_state.joint_state
        planned = dict(zip(state.name, state.position))
        previous = getattr(self.node, '_motion_path_constraints', None)
        self.node._motion_path_constraints = plan.constraints
        try:
            for sub in message.sub_trajectory:
                path = sub.trajectory.joint_trajectory
                if path.points:
                    if (sub.trajectory.multi_dof_joint_trajectory.points
                            or set(path.joint_names) != set(self.node.joints)):
                        raise MoveItError('MTC 反馈任务返回了所选手臂之外的运动')
                    self._execute_arm(sub, self.node.arm_controllers,
                                      motions=plan.motions, constraints=plan.constraints)
                    planned.update(zip(path.joint_names, path.points[-1].positions))
                self._apply_diff(sub.scene_diff, planned)
        finally:
            self.node._motion_path_constraints = previous
            if plan.restore is not None:
                self._apply_diff(plan.restore, planned)

    def _execute(self, sub, planned):
        trajectory = sub.trajectory
        path = trajectory.joint_trajectory
        if trajectory.multi_dof_joint_trajectory.points:
            raise MoveItError('本示例不执行浮动基座轨迹')
        if not path.joint_names and not path.points:
            return None
        names = list(path.joint_names)
        if (not names or not path.points or len(set(names)) != len(names)
                or any(len(point.positions) != len(names)
                       or not all(math.isfinite(value) for value in point.positions) for point in path.points)):
            raise MoveItError('MTC 返回空轨迹或无效关节位置')
        gripper = names == [self.node.gripper_joint]
        if not gripper and set(names) != set(self.node.joints):
            raise MoveItError('MTC 轨迹必须仅包含所选手臂或夹爪')
        controllers = list(sub.execution_info.controller_names)
        controller = controllers or (self.node.gripper_controller if gripper else self.node.arm_controllers)
        self.node.planned_start_state = self.node.robot_state({
            **planned, **dict(zip(names, path.points[0].positions))})
        event = None
        if gripper:
            target = path.points[-1].positions[0]
            closing = abs(target-self._close_angle) <= .015
            opening = abs(target-min(1., self.config.open_opening)*1.1) <= .015
            if closing and not self._grasp_confirmed:
                self._check_pick()
                # A timeout or interrupt can occur after real contact. Retain
                # the model and compensation until release is confirmed.
                self.holding = True
                if self.simulated:
                    self.node.execute(trajectory, controller=controller)
                    confirm_opening(self.node, target)
                else:
                    grasp_bottle(self.node, self.geometry)
                self._grasp_confirmed = True
                event = 'grasp'
            elif opening:
                if self.holding:
                    self._released_transform = self._wait_for_placement(planned)
                if self._grasp_confirmed and not self.simulated:
                    # Contact width differs from the nominal model. The
                    # gripper controller consumes the same endpoint and time.
                    trajectory = deepcopy(trajectory)
                    trajectory.joint_trajectory.points[0].positions = [
                        task_state(self.node)[self.node.gripper_joint]]
                self.node.execute(trajectory, controller=controller)
                confirm_opening(self.node, target)
                if self.holding:
                    self.holding = False
                    self._release_confirmed = True
                    event = 'release'
            else:
                raise MoveItError('MTC 夹爪轨迹不符合本次张爪/夹取目标')
        else:
            if self.holding and not self.scene.attached:
                raise MoveItError('水瓶尚未附着到场景，未开始搬运')
            if self._release_confirmed and self.scene.attached:
                raise MoveItError('水瓶模型尚未分离，未开始撤离')
            self._execute_arm(sub, controller)
        # Scene diffs and attachment transforms refer to the original MTC
        # states, even when feedback required a newly planned motion.
        planned.update(zip(names, path.points[-1].positions))
        return event

    def run(self, solution_message):
        if self._started or not solution_message.sub_trajectory:
            raise MoveItError('MTC 执行需要一个未执行过的完整抓放解')
        print('执行前轨迹平滑：已关闭，保留规划时长。'
              if getattr(self.node.args, 'smoothing', 'none') == 'none'
              else f'执行前轨迹平滑：Ruckig，jerk={self.config.smoothing_jerk:g} rad/s³。')
        self._started = True
        if getattr(self.config, 'place_z_offset', 0.):
            print(f'C 点放置高度校准：{self.config.place_z_offset*1000:+.1f} mm。')
        bridge = (f'/{self.node.arm_side}/onero_topic_bridge' if self.node.dual
                  else '/onero_topic_bridge')
        self.gains = GainClient(self.node, bridge)
        self.payload = PayloadClient(self.node, bridge)
        self.payload.ensure_unloaded()
        # Position gain is also needed for accurate approach and alignment.
        # Waiting until after contact leaves the pick at the lower idle gain.
        self._gains_changed = True
        self.gains.set_kp(self.config.carry_kp)
        wait_for_arm_settled(self.node)
        print(f'抓放位置增益已生效：kp={self.config.carry_kp:g}。')
        state = solution_message.start_scene.robot_state.joint_state
        planned = dict(zip(state.name, state.position))
        for index, sub in enumerate(solution_message.sub_trajectory):
            label = sub.info.comment or f'MTC stage {sub.info.stage_id}'
            print(f'[{index+1}/{len(solution_message.sub_trajectory)}] {label}')
            try:
                attached_before = self.scene.attached
                event = self._execute(sub, planned)
                state = sub.scene_diff.robot_state.joint_state
                planned.update(zip(state.name, state.position))
                self._apply_diff(sub.scene_diff, planned)
                if event == 'grasp':
                    self._load()
                elif event == 'release':
                    self._restore(restore_gains=False)
                if (attached_before and not self.scene.attached and self._release_confirmed
                        and self.planner is not None):
                    print('MTC 按松爪后的实测状态重新规划撤离和让位。')
                    departure = self.planner.plan_departure(self.node, self.config, self.geometry,
                        self.scene.id, yield_target=self.yield_target)
                    self.execute_motion(departure)
                    break
            except (Exception, KeyboardInterrupt) as exc:
                raise MoveItError(f'MTC 阶段「{label}」停止，后续动作未执行：'
                                  f'{str(exc) or "interrupted"}') from exc
        if not self._release_confirmed or self.holding or self.scene.attached:
            raise MoveItError('MTC 解未完成确认夹取、松爪及分离，任务停止')
        self.completed = True
        self._restore()

    def cleanup(self):
        if self.holding:
            self.node.get_logger().warning('任务停止时可能仍夹持水瓶，已保留模型、夹持、增益和负载；'
                                           '请先支撑并人工放下水瓶，再清理模型及恢复负载。')
            return
        try:
            self._restore()
        finally:
            if self.scene.changed:
                clear_bottle(self.node, self.scene.id)
                self.scene.changed = self.scene.attached = False
