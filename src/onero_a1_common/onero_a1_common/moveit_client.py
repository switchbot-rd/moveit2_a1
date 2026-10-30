"""Small standard MoveIt ROS client, shared by applications and diagnostic tools.

This module handles ROS requests, fresh state, visualization and execution.
Reporting, pose archives, model fingerprints and precision acceptance belong
to the diagnostic tools, which can extend this client.
"""
from copy import deepcopy
from types import SimpleNamespace
from rclpy.serialization import serialize_message
import json
import math
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
    MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint, RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from .constants import JOINTS, LEFT_JOINTS, OMPL_PLANNERS, PLANNING_PIPELINES
from .startup_reference import load_reference, reference_path, store_reference

# A1 position feedback has a resolution of about 0.0003815 rad. Permit one
# feedback tick at a bounded joint's limit in planning requests only.
START_STATE_BOUNDS_EPSILON = 0.0004
# Endpoint quality above this value triggers a multi-seed joint-space branch
# search even if a Cartesian path technically completed.
TASK_QUALITY_LIMIT = 2.0


class MoveItError(RuntimeError):
    pass


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class MoveItClient(Node):
    def __init__(self, args):
        super().__init__("onero_a1_client_" + uuid.uuid4().hex[:8])
        self.args = args
        self.latest = None
        self.active_goal = None
        self.scene_id = "onero_example_" + uuid.uuid4().hex
        self.scene_added = False
        self.service_clients = {}
        self.planned_start_state = None
        self.startup_id = None
        self.wrist_roll_limit = None
        self._strict_margin = False
        self._joint_cache = {}
        self._joint_position_bounds = None
        self._configure_arm("right", dual=False)
        self.create_subscription(
            JointState, "/joint_states", self.on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.planner = ActionClient(self, MoveGroup, "/move_action")
        self.execution_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 1)

    def on_state(self, msg):
        stamp = seconds(msg.header.stamp)
        if not -.1 <= self.get_clock().now().nanoseconds*1e-9-stamp <= 1.:
            return
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(value):
                previous = self._joint_cache.get(name)
                if previous is None or stamp >= previous[1]:
                    self._joint_cache[name] = (value, stamp)
            else:
                self._joint_cache.pop(name, None)
        positions = dict(zip(msg.name, msg.position))
        if all(n in positions and math.isfinite(positions[n]) for n in self.joints):
            self.latest = msg

    def _configure_arm(self, side, dual):
        self._posture_profile = None
        self._elbow_preference = None
        self.arm_side = side
        self.dual = dual
        self.joints = LEFT_JOINTS+JOINTS if side == "both" else list(LEFT_JOINTS if side == "left" else JOINTS)
        self.group = "both_arms" if side == "both" else f"{side}_arm" if dual else "arm"
        prefix = side+"_" if dual and side != "both" else ""
        self.tip_link = prefix+"Link7"
        self.gripper_joint = prefix+"woan_picker_left_finger_joint"
        self.gripper_group = prefix+"gripper"
        self.gripper_controller = prefix+"gripper_controller"
        self.arm_controllers = (["left_arm_controller", "right_arm_controller"] if side == "both"
                                else [prefix+"arm_controller"])

    def select_arm(self, side=None):
        semantic = ET.fromstring(self.parameter("/move_group", "robot_description_semantic"))
        groups = {g.get("name") for g in semantic.findall("group")}
        dual = {"left_arm", "right_arm", "both_arms"} <= groups
        single_side = ('left' if any(j.get('name', '').endswith('-a1_l')
                                   for s in semantic.findall('group_state') for j in s.findall('joint')) else 'right')
        side = side or ('right' if dual else single_side)
        if side not in ("left", "right", "both") or (not dual and side != single_side):
            raise MoveItError(f"当前为{single_side}单臂服务；双臂请启动 dual_moveit.launch.py")
        self._configure_arm(side, dual)
        if dual and side != "both" and self.args.frame == "base_link":
            self.args.frame = side+"_base_link"

    def wait(self, future, timeout=15.):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.05)
        if not future.done():
            raise MoveItError("ROS 请求超时")
        result = future.result()
        if result is None:
            raise MoveItError("ROS 请求未返回结果")
        return result

    def call(self, srv, name, request):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(srv, name)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=10.):
            raise MoveItError(f"服务不可用：{name}")
        result = self.wait(client.call_async(request))
        context = getattr(self, '_geometry_context', None)
        if srv is ApplyPlanningScene and result.success and context is not None:
            context.set_scene(serialize_message(request.scene))
        return result

    def state(self, *, after=None, max_age=1., timeout=10.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            now = self.get_clock().now().nanoseconds*1e-9
            fresh = {name:value for name,(value,stamp) in self._joint_cache.items() if -.1 <= now-stamp <= max_age}
            if all(name in fresh and (after is None or self._joint_cache[name][1] > after)
                   for name in self.joints):
                return fresh
        raise MoveItError("没有新鲜、完整的 /joint_states")

    @staticmethod
    def robot_state(q):
        # Preserve the gripper and attached-object state in MoveIt's scene.
        return RobotState(is_diff=True, joint_state=JointState(
            name=list(q), position=[float(v) for v in q.values()]))

    def planning_start_state(self, q):
        """Project only sub-tick start errors; preserve feedback and all goals."""
        if self._joint_position_bounds is None:
            model = ET.fromstring(self.parameter('/move_group', 'robot_description'))
            bounds = {}
            for joint in model.findall('joint'):
                limit = joint.find('limit')
                if joint.get('type') == 'revolute' and limit is not None:
                    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                    if not (math.isfinite(lower) and math.isfinite(upper) and lower <= upper):
                        raise MoveItError(f"关节 {joint.get('name')} 的模型限位无效")
                    bounds[joint.get('name')] = (lower, upper)
            self._joint_position_bounds = bounds
        projected = dict(q)
        corrections = []
        for name, position in q.items():
            if name not in self._joint_position_bounds:
                continue
            lower, upper = self._joint_position_bounds[name]
            if not math.isfinite(position):
                raise MoveItError(f'规划起点 {name} 位置无效')
            bounded = min(upper, max(lower, position))
            error = abs(bounded-position)
            if error > START_STATE_BOUNDS_EPSILON + 1e-12:
                raise MoveItError(
                    f'规划起点 {name}={position:.6f} rad 超出限位 [{lower}, {upper}]，'
                    f'越界 {error:.6f} rad，大于反馈边界容差 {START_STATE_BOUNDS_EPSILON} rad')
            if error:
                projected[name] = bounded
                corrections.append(f'{name}: {position:.6f} -> {bounded:.6f} rad')
        if corrections:
            self.get_logger().warning(
                '规划起点反馈存在一个刻度内的边界偏差，仅对规划请求归界：' + '; '.join(corrections))
        return self.robot_state(projected)

    def parameter(self, node, name):
        result = self.call(GetParameters, node+"/get_parameters", GetParameters.Request(names=[name]))
        if len(result.values) != 1 or result.values[0].type != 4:
            raise MoveItError(f"无法读取 {node} 的字符串参数 {name}")
        return result.values[0].string_value

    def named_target(self, name):
        srdf = ET.fromstring(self.parameter("/move_group", "robot_description_semantic"))
        states = {s.get("name"): s for s in srdf.findall("group_state") if s.get("group") == self.group}
        if name not in states:
            raise MoveItError(f"未知姿态 {name}；可选：{list(states)}")
        return {j.get("name"): float(j.get("value")) for j in states[name].findall("joint")}

    def _startup_sessions(self):
        if self.dual and self.arm_side == "both":
            selected = [('left', LEFT_JOINTS), ('right', JOINTS)]
        else:
            selected = [(self.arm_side, self.joints)]
        sessions = []
        for side, names in selected:
            bridge = f'/{side}/onero_topic_bridge' if self.dual else '/onero_topic_bridge'
            try:
                identifier = self.parameter(bridge, 'session_id')
                identity = json.loads(self.parameter(bridge, 'recording_identity'))
            except (MoveItError, ValueError) as exc:
                raise MoveItError('无法读取服务实例标识，请更新并重启手臂服务后再记录起始位置') from exc
            if not identifier or not isinstance(identity, dict):
                raise MoveItError('服务实例或机械臂配置标识无效，请重启手臂服务')
            sessions.append({'id': identifier, 'bridge': bridge, 'arm': side,
                             'joint_names': list(names), 'identity': identity})
        return sessions

    def startup_snapshot(self):
        sessions = self._startup_sessions()
        try:
            return load_reference(reference_path(), sessions)
        except (ValueError, OSError) as exc:
            raise MoveItError(str(exc)) from exc

    def record_startup(self):
        sessions = self._startup_sessions()
        measured = self.state()
        positions = {name: measured[name] for name in self.joints}
        if self._startup_sessions() != sessions:
            raise MoveItError('记录期间手臂服务已重启，请重新记录起始位置')
        try:
            store_reference(reference_path(), sessions, positions, self.get_clock().now().nanoseconds)
        except (ValueError, OSError) as exc:
            raise MoveItError(str(exc)) from exc
        if self._startup_sessions() != sessions:
            raise MoveItError('记录期间手臂服务已重启，请重新记录起始位置')
        return self.startup_snapshot()

    def freedrive(self, *, damping=.3, duration=None, execute=False):
        from .freedrive import run_freedrive
        return run_freedrive(self, damping=damping, duration=duration,
                             execute=execute, error=MoveItError)

    def fk(self, q, link=None):
        if all(name in q for name in self.joints):
            from .continuous import geometry
            from scipy.spatial.transform import Rotation
            matrix = geometry(self).fk(q, link or self.tip_link, self.args.frame)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
                float, Rotation.from_matrix(matrix[:3, :3]).as_quat())
            return pose
        request = GetPositionFK.Request(robot_state=self.robot_state(q), fk_link_names=[link or self.tip_link])
        request.header.frame_id = self.args.frame
        response = self.call(GetPositionFK, "/compute_fk", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise MoveItError(f"FK 失败：{response.error_code.val}")
        return response.pose_stamped[0].pose

    def validity(self, q, constraints=None, group=None):
        selected_group = group or self.group
        if constraints is None and selected_group == self.group:
            constraints = getattr(self, '_motion_path_constraints', None)
        context = getattr(self, '_local_validation', None)
        if context is not None:
            valid, pairs = context.validity(q, selected_group, serialize_message(constraints) if constraints else b'')
            return SimpleNamespace(valid=valid, contacts=[SimpleNamespace(
                contact_body_1=first, contact_body_2=second) for first, second in pairs])
        request = GetStateValidity.Request(robot_state=self.robot_state(q), group_name=group or self.group)
        if constraints is not None:
            request.constraints = constraints
        return self.call(GetStateValidity, "/check_state_validity", request)

    def inverse_kinematics(self, pose, seed, *, group, link, avoid_collisions=True, timeout=1.):
        request = GetPositionIK.Request()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = self.args.frame
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state = self.robot_state(seed)
        request.ik_request.avoid_collisions = avoid_collisions
        nanoseconds = round(timeout*1e9)
        request.ik_request.timeout.sec, request.ik_request.timeout.nanosec = divmod(nanoseconds, 1_000_000_000)
        response = self.call(GetPositionIK, "/compute_ik", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItError(f"{group} 逆解失败：{response.error_code.val}")
        return dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))

    def reference_seed(self):
        profile = self.posture_profile()
        reference = getattr(profile, 'reference', ())
        if not getattr(profile, 'enabled', False) or len(reference) != len(self.joints):
            return None
        return {name: float(reference[i]) for i, name in enumerate(self.joints)}

    def wrist_limit(self):
        if self.wrist_roll_limit is not None:
            return float(self.wrist_roll_limit)
        return float(getattr(self.posture_profile(), 'max_wrist_roll_deviation', math.pi/2))

    def joint_margin(self, positions):
        """Smallest distance to the URDF joint limits, in radians."""
        bounds = getattr(self, '_joint_position_bounds', None)
        if bounds is None:
            load = getattr(self, 'planning_start_state', None)
            if load is None:
                return math.inf
            try:
                load(positions)
            except (MoveItError, AttributeError, KeyError, ValueError):
                return math.inf
            bounds = getattr(self, '_joint_position_bounds', None) or {}
        margins = []
        for name in self.joints:
            bound = bounds.get(name)
            if bound is None or name not in positions:
                continue
            margins.append(min(positions[name]-bound[0], bound[1]-positions[name]))
        return min(margins) if margins else math.inf

    def condition_number(self, positions):
        """Numerical 6x7 Jacobian condition number; large values mean near singularity."""
        import numpy as np
        from scipy.spatial.transform import Rotation
        from .continuous import geometry
        if len(self.joints) != 7 or any(name not in positions for name in self.joints):
            return math.inf
        try:
            context = geometry(self)
            base = np.asarray(context.fk({name: positions[name] for name in self.joints},
                                         self.tip_link, self.args.frame))
            jacobian = np.zeros((6, len(self.joints)))
            epsilon = 1e-5
            for index, name in enumerate(self.joints):
                perturbed = {joint: positions[joint] for joint in self.joints}
                perturbed[name] += epsilon
                pose = np.asarray(context.fk(perturbed, self.tip_link, self.args.frame))
                jacobian[:3, index] = (pose[:3, 3]-base[:3, 3])/epsilon
                jacobian[3:, index] = Rotation.from_matrix(
                    pose[:3, :3] @ base[:3, :3].T).as_rotvec()/epsilon
            singular = np.linalg.svd(jacobian, compute_uv=False)
            return float(singular[0]/max(singular[-1], 1e-9))
        except (MoveItError, RuntimeError, ValueError):
            return math.inf

    def grasp_quality(self, positions):
        """Soft endpoint score: penalize near-extension, joint limits and wrist winding."""
        profile = self.posture_profile()
        enabled = bool(getattr(profile, 'enabled', False))
        margin = self.joint_margin(positions) if enabled else math.inf
        condition = self.condition_number(positions) if enabled else math.inf
        wrist = 0.
        if enabled:
            reference = getattr(profile, 'reference', ())
            deviations = [abs(positions[name]-reference[i]) for i, name in enumerate(self.joints)
                          if i % 7 in (4, 6) and i < len(reference) and name in positions]
            wrist = max(deviations) if deviations else 0.
        condition_penalty = .02*min(condition, 500.) if math.isfinite(condition) else 0.
        margin_penalty = 50.*max(0., .12-margin)**2 if math.isfinite(margin) else 0.
        if (getattr(self, '_strict_margin', False) and math.isfinite(margin)
                and margin < .08):
            # Only final placement endpoints need the hard limit; transfer
            # poses may legitimately be closer to a joint limit.
            margin_penalty += 20.
        wrist_penalty = 8.*max(0., wrist-.35)**2
        return condition_penalty+margin_penalty+wrist_penalty, condition, margin, wrist

    def ik_candidates(self, poses, seed, *, extra_seeds=()):
        """Rank feasible task poses by endpoint quality and branch continuity.

        One MoveIt IK request returns one branch.  A single warm seed near a
        low-table target tends to keep a nearby, possibly extended/wound
        branch, so we also ask from the task reference and merge the results.
        """
        profile = self.posture_profile()
        filters = bool(getattr(profile, 'enabled', False))
        limit = self.wrist_limit()
        continuity = float(getattr(profile, 'continuity_weight', 0.))
        seeds = [seed]+[extra for extra in extra_seeds if extra is not None]
        candidates = []
        for index, pose in enumerate(poses):
            found = []
            for candidate_seed in seeds:
                try:
                    solved = self.inverse_kinematics(pose, candidate_seed, group=self.group,
                                                     link=self.tip_link, timeout=.1)
                    joints = {name: solved[name] for name in self.joints}
                except (MoveItError, KeyError):
                    continue
                if any(max(abs(joints[name]-other[name]) for name in self.joints) < .01
                       for other in found):
                    continue
                if (filters and len(profile.reference) == len(self.joints)
                        and any(abs(joints[name]-profile.reference[i]) > limit
                                for i, name in enumerate(self.joints) if i % 7 in (4, 6))):
                    continue
                found.append(joints)
                movement = sum((joints[name]-seed[name])**2 for name in self.joints)
                score = (self.posture_cost(joints)+continuity*movement
                         + self.grasp_quality(joints)[0])
                candidates.append(((score, movement), index, joints))
                if len(found) >= 2:
                    break
        return [(index, joints) for _, index, joints in sorted(candidates, key=lambda item: item[0])]

    def motion_posture_cost(self, start, trajectory):
        """Score the returned motion, including its ACTUAL redundancy branch."""
        from .continuous import geometry
        profile = self.posture_profile()
        stats = geometry(self).motion_posture(start, self.group, serialize_message(trajectory))
        if getattr(profile, 'enabled', False):
            limit = self.wrist_limit()
            if stats['end_roll'] > limit+.01:
                raise MoveItError(f'轨迹终点 J5/J7 滚转偏离参考 {math.degrees(stats["end_roll"]):.1f}°，'
                                  f'超过自然构型范围 {math.degrees(limit):.1f}°')
            if stats['peak_roll'] > max(limit, stats['start_roll'])+.01:
                raise MoveItError('轨迹途中加重了 J5/J7 的滚转扭曲')
            endpoint = self.trajectory_endpoint(start, trajectory)
            margin = self.joint_margin(endpoint)
            if (getattr(self, '_strict_margin', False) and math.isfinite(margin)
                    and margin < .08):
                raise MoveItError(f'轨迹终点关节余量 {margin:.3f} rad 过小，拒绝贴限位分支')
        return stats['cost']

    @staticmethod
    def trajectory_endpoint(start, trajectory):
        path = trajectory.joint_trajectory
        return {**start, **dict(zip(path.joint_names, path.points[-1].positions))}

    def plan_pose_candidates(self, start, poses, *, cartesian_first=False, initial=None):
        """Choose from actual feasible paths, not only nominal endpoint IK scores.

        Every complete Cartesian candidate is scored by its actual posture and
        endpoint conditioning.  If all candidates stay on a bad branch, a
        joint-space fallback is tried even when a Cartesian path technically
        completed; this is what allows the arm to leave an already wound
        redundancy branch instead of accepting the first feasible one.
        """
        best, best_state, best_cost, best_quality = None, None, math.inf, math.inf
        best_fraction, failure = 0., None

        def evaluate(path):
            cost = self.motion_posture_cost(start, path)
            endpoint = self.trajectory_endpoint(start, path)
            quality = self.grasp_quality(endpoint)[0]
            return cost+quality, quality

        if initial is not None:
            path, start_state = initial
            try:
                value, quality = evaluate(path)
                best, best_state, best_cost, best_quality = path, start_state, value, quality
            except MoveItError as exc:
                failure = exc
        if cartesian_first:
            # Independent goal IK can choose a different redundant branch from
            # continuous IK. Its failure must not veto an untried task pose.
            for pose in poses:
                response = self.cartesian_path(start, [pose])
                if math.isfinite(response.fraction):
                    best_fraction = max(best_fraction, response.fraction)
                if not self.complete_cartesian(response):
                    continue
                try:
                    value, quality = evaluate(response.solution)
                except MoveItError as exc:
                    failure = exc
                    continue
                if value < best_cost:
                    best, best_state, best_cost, best_quality = (
                        response.solution, response.start_state, value, quality)
        if best is not None and best_quality <= TASK_QUALITY_LIMIT:
            self.planned_start_state = best_state
            return best
        if best is not None:
            print(f'当前候选末端条件偏弱（质量 {best_quality:.2f}），尝试多 seed 关节空间分支。')
        elif cartesian_first:
            print(f'笛卡尔候选未通过完整性或构型检查（最高完成 {best_fraction:.1%}），尝试避障规划。')
        candidates = self.ik_candidates(poses, start, extra_seeds=(self.reference_seed(),))
        if not candidates:
            if best is not None:
                print('多 seed 逆解不可用，保留已通过完整性检查的最佳候选。')
                self.planned_start_state = best_state
                return best
            detail = str(failure) if failure else (
                f'笛卡尔候选最高完成 {best_fraction:.1%}' if cartesian_first else '目标位姿求解失败')
            raise MoveItError(f'{detail}；后备逆解未满足碰撞与自然构型要求') from failure
        selected = candidates[:4]
        budget = self.args.planning_time
        try:
            self.args.planning_time = budget/max(1, len(selected))
            for _, joints in selected:
                try:
                    path = self.plan(start, self.joint_constraints(joints))
                    value, quality = evaluate(path)
                except MoveItError as exc:
                    if self.active_goal is not None:
                        raise
                    failure = exc
                    continue
                if value < best_cost:
                    best, best_state, best_cost, best_quality = (
                        path, self.planned_start_state, value, quality)
        finally:
            self.args.planning_time = budget
        if best is not None:
            if best_quality > TASK_QUALITY_LIMIT:
                print(f'警告：仍保留最佳候选，但末端质量 {best_quality:.2f} 仍偏弱。')
            self.planned_start_state = best_state
            return best
        raise failure or MoveItError('没有可用的自然构型路径')

    def plan_continuous(self, start, goals, *, before=None, check=None, path_constraints=None):
        previous = getattr(self, '_motion_path_constraints', None)
        self._motion_path_constraints = path_constraints
        try:
            return self._plan_continuous(start, goals, before=before, check=check)
        finally:
            self._motion_path_constraints = previous

    def _plan_continuous(self, start, goals, *, before=None, check=None):
        """Plan one arm motion through task goals, with one global MoveIt timing pass.

        Logical goals do not become controller actions or intermediate stops.
        Collision checks run again after MoveIt's corner blending.
        """
        from .continuous import geometry, retime
        if not goals:
            raise MoveItError('连续运动至少需要一个目标')
        current, paths, first_state = dict(start), [], None
        budget = self.args.planning_time
        try:
            searches = sum(kind != 'cartesian' for kind, _ in goals)
            remaining = budget
            # Never let one expensive search consume the whole shared budget
            # and leave a later goal with zero seconds.  The floor is a
            # best-effort allowance, not a global time cap.
            floor = max(.5, budget*.1)
            for index, (kind, target) in enumerate(goals):
                if before is not None:
                    before(index, current)
                target = target(current) if callable(target) else target
                allowance = max(floor, remaining/max(1, searches))
                self.args.planning_time = allowance
                began = time.monotonic()
                try:
                    if kind == 'joints':
                        path = self.plan(current, self.joint_constraints(target))
                    elif kind == 'position':
                        path = self.plan_position(current, *target)
                    elif kind == 'upright_position':
                        path = self.plan_upright_position(current, *target)
                    elif kind == 'upright_position_contact':
                        position, tip_to_object, contact = target
                        if contact is not None:
                            contact(True)
                        previous_strict = getattr(self, '_strict_margin', False)
                        self._strict_margin = True
                        try:
                            path = self.plan_upright_position(current, position, tip_to_object)
                        finally:
                            self._strict_margin = previous_strict
                            if contact is not None:
                                contact(False)
                        endpoint = self.trajectory_endpoint(current, path)
                        quality, condition, margin, wrist = self.grasp_quality(endpoint)
                        print(f'最终直立放置：质量={quality:.2f}，条件数={condition:.1f}，'
                              f'关节余量={margin:.3f} rad，腕部偏差={wrist:.3f} rad。')
                    elif kind == 'upright':
                        path = self.plan_upright(current, *target)
                    elif kind == 'place':
                        prefix, path = self.plan_place_paths(current, *target)
                        paths.append(prefix)
                    elif kind == 'cartesian':
                        response = self.cartesian_path(current, [target])
                        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                                or not math.isfinite(response.fraction) or response.fraction < 1.-1e-6):
                            raise MoveItError(f'连续运动目标 {index+1} 笛卡尔路径不完整：{response.fraction}')
                        self.planned_start_state = response.start_state
                        path = response.solution
                    else:
                        raise MoveItError(f'未知连续运动目标类型：{kind}')
                except MoveItError as exc:
                    raise MoveItError(f'连续运动目标 {index+1}/{len(goals)}（{kind}）失败：{exc}') from exc
                finally:
                    if kind != 'cartesian':
                        # A short Cartesian clearance often uses almost none of
                        # its search share. Keep that time for later goals.
                        remaining = max(0., remaining-min(allowance, time.monotonic()-began))
                        searches -= 1
                if not path.joint_trajectory.points:
                    raise MoveItError('MoveIt 连续运动目标未返回路径')
                if first_state is None:
                    first_state = deepcopy(self.planned_start_state)
                paths.append(path)
                current.update(zip(path.joint_trajectory.joint_names, path.joint_trajectory.points[-1].positions))
        finally:
            self.args.planning_time = budget
        previous = getattr(self, '_local_validation', None)
        self._local_validation = geometry(self, refresh=True)
        try:
            for attempt, tolerance in enumerate((.005, .0005)):
                result = retime(self, paths, first_state, tolerance=tolerance)
                try:
                    for index, point in enumerate(result.joint_trajectory.points):
                        state = {**start, **dict(zip(result.joint_trajectory.joint_names, point.positions))}
                        if check is not None:
                            check(state)
                        validity = self.validity(state)
                        if not validity.valid:
                            pairs = ', '.join(f'{c.contact_body_1} ↔ {c.contact_body_2}' for c in validity.contacts)
                            raise MoveItError(f'连续轨迹第 {index+1} 点无效：{pairs or "违反路径约束"}')
                except MoveItError:
                    if attempt:
                        raise
                    print('圆滑路径未通过碰撞或约束检查，收紧 MoveIt 圆滑容差后重算一次。')
                    continue
                # Posture is a property of the RRT path, not of the retiming
                # tolerance. Do not retime again hoping to remove a wound wrist.
                self.motion_posture_cost(start, result)
                self.planned_start_state = first_state
                return result
        finally:
            self._local_validation = previous

    def upright_constraints(self, tip_to_object, tolerance):
        """Keep the bottle's vertical axis upright, with free world-Z yaw."""
        from scipy.spatial.transform import Rotation
        pose = Pose()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
            float, Rotation.from_matrix(tip_to_object[:3, :3].T).as_quat())
        orientation = self.orientation_constraint(pose, tolerance/math.sqrt(2.))
        orientation.absolute_z_axis_tolerance = math.pi
        return Constraints(orientation_constraints=[orientation])

    @staticmethod
    def complete_cartesian(response):
        return (response.error_code.val == MoveItErrorCodes.SUCCESS and math.isfinite(response.fraction)
                and response.fraction >= 1.-1e-6 and bool(response.solution.joint_trajectory.points))

    def plan_upright_position(self, start, position, tip_to_object):
        import numpy as np
        from .continuous import geometry
        solution = geometry(self, refresh=True).upright_ik(
            start, self.group, self.tip_link, self.args.frame, np.asarray(position, dtype=float),
            np.asarray(tip_to_object[:3, 3]), np.asarray(tip_to_object[:3, 2]))
        initial = None
        if solution:
            response = self.cartesian_path(start, [self.fk(solution)])
            if self.complete_cartesian(response):
                initial = (response.solution, response.start_state)
        return self.plan_pose_candidates(start, self.upright_poses(start, position, tip_to_object),
                                         cartesian_first=True, initial=initial)

    def plan_position(self, start, point, offset=(0., 0., 0.)):
        """Plan a tool-relative point goal with soft, naturally ranked rotations.

        Solve nearby and reference orientations before joint-space planning;
        an unconstrained random SO(3) goal sampler is costly for small retreats.
        These are endpoint alternatives, never waypoints on the returned path.
        """
        import numpy as np
        from scipy.spatial.transform import Rotation
        current_pose = self.fk(start)
        current = current_pose.orientation
        rotation = Rotation.from_quat([current.x, current.y, current.z, current.w])
        target = deepcopy(current_pose)
        target.position.x, target.position.y, target.position.z = map(float, np.array(point)-rotation.apply(offset))
        delta = np.array([target.position.x-current_pose.position.x, target.position.y-current_pose.position.y,
                          target.position.z-current_pose.position.z])
        if np.linalg.norm(delta) <= .06:
            # A short clearance motion should retain its nearby IK branch.
            # MoveIt's Cartesian solver checks collisions and joint jumps; if
            # that translation is blocked, fall back to free endpoint rotation.
            response = self.cartesian_path(start, [target])
            if self.complete_cartesian(response):
                try:
                    self.motion_posture_cost(start, response.solution)
                except MoveItError:
                    pass
                else:
                    self.planned_start_state = response.start_state
                    return response.solution
        from .continuous import geometry
        solution = geometry(self, refresh=True).position_ik(
            start, self.group, self.tip_link, self.args.frame, np.asarray(point, dtype=float),
            np.asarray(offset, dtype=float))
        reference = self.fk({**start, **dict(zip(self.joints, self.posture_profile().reference))}).orientation
        rotations = [rotation, Rotation.from_quat([reference.x, reference.y, reference.z, reference.w])]
        for axis in np.eye(3):
            rotations.extend(Rotation.from_rotvec(sign*math.pi/6.*axis)*rotation for sign in (-1., 1.))
        poses = [self.fk(solution)] if solution else []
        for candidate in rotations:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, np.array(point)-candidate.apply(offset))
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, candidate.as_quat())
            poses.append(pose)
        return self.plan_pose_candidates(start, poses, cartesian_first=True)

    def upright_poses(self, start, position, tip_to_object):
        """Upright object goals at absolute world yaws, plus the current heading.

        For a bottle, world yaw is physically free.  Sampling absolute yaws
        avoids missing the comfortable arm branch just because the carried
        object happened to end at an awkward relative yaw.
        """
        import numpy as np
        from scipy.spatial.transform import Rotation
        current = self.fk(start).orientation
        rotation = Rotation.from_quat([current.x, current.y, current.z, current.w]).as_matrix()
        obj_rotation = rotation @ tip_to_object[:3, :3]
        current_yaw = math.atan2(obj_rotation[1, 0], obj_rotation[0, 0])
        yaws = [0., -30., 30., -60., 60., -90., 90., 180.]
        if not any(abs(math.atan2(math.sin(current_yaw-math.radians(yaw)),
                                  math.cos(current_yaw-math.radians(yaw)))) < math.radians(5.)
                   for yaw in yaws):
            yaws.insert(0, math.degrees(current_yaw))
        inverse = np.linalg.inv(tip_to_object)
        poses = []
        for yaw in yaws:
            obj = np.eye(4)
            obj[:3, :3] = Rotation.from_euler('z', math.radians(yaw)).as_matrix()
            obj[:3, 3] = position
            tip = obj @ inverse
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, tip[:3, 3])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(
                float, Rotation.from_matrix(tip[:3, :3]).as_quat())
            poses.append(pose)
        return poses

    def plan_upright(self, start, position, tip_to_object):
        return self.plan_pose_candidates(start, self.upright_poses(start, position, tip_to_object),
                                         cartesian_first=True)

    def plan_place_paths(self, start, position, tip_to_object, clearance, contact=None):
        """Choose a natural upright placement, preferring joint-space branches.

        Only the bottle's vertical axis is constrained.  The descent therefore
        does not have to be a straight vertical line: first try a joint-space
        path to a well-conditioned final IK branch, then fall back to the
        Cartesian approach/descent for feasibility.
        """
        import numpy as np
        from scipy.spatial.transform import Rotation
        pre_poses = self.upright_poses(start, np.asarray(position)+[0., 0., clearance], tip_to_object)
        final_poses = self.upright_poses(start, position, tip_to_object)
        budget, failure = self.args.planning_time, None
        best, best_state, best_cost, best_quality = None, None, math.inf, math.inf
        best_final_quality = math.inf

        def descent_from(state):
            # Build a Cartesian descent from the branch actually reached.
            tip = self.fk(state)
            q = tip.orientation
            rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
            center = np.array([tip.position.x, tip.position.y, tip.position.z])+rotation.apply(tip_to_object[:3, 3])
            delta = np.asarray(position)-center
            finish = deepcopy(tip)
            finish.position.x += float(delta[0]); finish.position.y += float(delta[1]); finish.position.z += float(delta[2])
            if contact is not None:
                contact(True)
            try:
                return self.cartesian_path(state, [finish])
            finally:
                if contact is not None:
                    contact(False)

        def evaluate(prefix, descent):
            reached = self.trajectory_endpoint(start, prefix)
            final = self.trajectory_endpoint(reached, descent)
            cost = (self.motion_posture_cost(start, prefix)
                    + self.motion_posture_cost(reached, descent))
            reached_quality = self.grasp_quality(reached)[0]
            final_quality = self.grasp_quality(final)[0]
            quality = reached_quality+final_quality
            return cost+quality, quality, final_quality

        def consider(prefix, descent, state):
            nonlocal best, best_state, best_cost, best_quality, best_final_quality, failure
            try:
                value, quality, final_quality = evaluate(prefix, descent)
            except MoveItError as exc:
                failure = exc
                return False
            # The final placement posture is the primary objective.  A
            # natural final branch wins even if the transient pre-place
            # branch or the path length is worse.
            if (best is None
                    or (best_final_quality > TASK_QUALITY_LIMIT
                        and final_quality < best_final_quality-.25)
                    or value < best_cost):
                best, best_state, best_cost, best_quality = (prefix, descent), state, value, quality
                best_final_quality = final_quality
            return True

        try:
            # 1. Natural branch: keep the pre-place branch reachable from the
            #    current carry state, and keep the final branch natural.  They
            #    do not have to share a yaw; the short joint-space descent is
            #    allowed to rotate the bottle around its vertical axis.
            if contact is not None:
                contact(True)
            try:
                final_candidates = self.ik_candidates(final_poses, start,
                                                      extra_seeds=(self.reference_seed(),))[:4]
            finally:
                if contact is not None:
                    contact(False)
            pre_solutions = self.ik_candidates(pre_poses, start,
                                               extra_seeds=(self.reference_seed(),))[:6]
            if not pre_solutions:
                print('没有可用的预放置 IK 分支。')
            if not final_candidates:
                print('没有可用的最终 IK 分支。')
            self.args.planning_time = max(5., budget)
            natural_attempts = 0
            for _, pre_joints in pre_solutions:
                try:
                    prefix = self.plan(start, self.joint_constraints(pre_joints, tolerance=.005))
                    state = self.planned_start_state
                    reached = self.trajectory_endpoint(start, prefix)
                except MoveItError as exc:
                    if self.active_goal is not None:
                        raise
                    print(f'自然放置预放置前缀失败：{exc}')
                    failure = exc
                    continue
                for index, final_joints in final_candidates:
                    final_quality = self.grasp_quality(final_joints)[0]
                    if final_quality > TASK_QUALITY_LIMIT:
                        continue
                    natural_attempts += 1
                    # A short Cartesian descent to the natural final pose at
                    # this yaw; if the yaw differs it will usually fail and
                    # the joint-space descent below handles the rotation.
                    if contact is not None:
                        contact(True)
                    try:
                        checked = self.cartesian_path(reached, [final_poses[index]])
                    finally:
                        if contact is not None:
                            contact(False)
                    if self.complete_cartesian(checked):
                        if consider(prefix, checked.solution, state):
                            if best is not None and best_final_quality <= TASK_QUALITY_LIMIT:
                                final_state = self.trajectory_endpoint(reached, checked.solution)
                                _, fc, fm, fw = self.grasp_quality(final_state)
                                print(f'自然放置分支：末端质量={best_final_quality:.2f}，条件数={fc:.1f}，'
                                      f'关节余量={fm:.3f} rad，腕部偏差={fw:.3f} rad。')
                                break
                    else:
                        print('自然放置短距离 Cartesian 下降不可用，尝试关节空间下降。')
                    # Joint-space descent decouples the pre-place and final
                    # yaw; only the bottle's vertical axis is constrained.
                    try:
                        if contact is not None:
                            contact(True)
                        try:
                            descent = self.plan(reached, self.joint_constraints(final_joints, tolerance=.0001))
                        finally:
                            if contact is not None:
                                contact(False)
                    except MoveItError as exc:
                        if self.active_goal is not None:
                            raise
                        print(f'自然放置下降规划失败：{exc}')
                        failure = exc
                        continue
                    if consider(prefix, descent, state):
                        if best is not None and best_final_quality <= TASK_QUALITY_LIMIT:
                            final_state = self.trajectory_endpoint(reached, descent)
                            _, fc, fm, fw = self.grasp_quality(final_state)
                            print(f'自然放置分支：末端质量={best_final_quality:.2f}，条件数={fc:.1f}，'
                                  f'关节余量={fm:.3f} rad，腕部偏差={fw:.3f} rad。')
                            break
                    if best is None or best_final_quality > TASK_QUALITY_LIMIT:
                        print(f'自然放置候选姿态检查未通过：{failure}')
                if best is not None and best_final_quality <= TASK_QUALITY_LIMIT:
                    break
            # 2. Cartesian fallback, only when no natural branch is available
            #    or the natural branch is still poor.
            if best is None or best_final_quality > TASK_QUALITY_LIMIT:
                if best is not None:
                    print(f'自然放置末端质量 {best_final_quality:.2f} 仍偏弱，尝试笛卡尔接近/下降。')
                else:
                    print(f'没有找到自然关节空间放置分支（尝试 {natural_attempts} 个最终分支），'
                          '尝试笛卡尔接近/下降。')
                for pose in pre_poses:
                    approach = self.cartesian_path(start, [pose])
                    if not self.complete_cartesian(approach):
                        continue
                    reached = self.trajectory_endpoint(start, approach.solution)
                    checked = descent_from(reached)
                    if not self.complete_cartesian(checked):
                        continue
                    consider(approach.solution, checked.solution, approach.start_state)
                if best is None or best_final_quality > TASK_QUALITY_LIMIT:
                    candidates = self.ik_candidates(pre_poses, start,
                                                    extra_seeds=(self.reference_seed(),))[:2]
                    self.args.planning_time = budget/max(1, len(candidates))
                    for _, joints in candidates:
                        try:
                            if not self.complete_cartesian(descent_from({**start, **joints})):
                                continue
                            prefix = self.plan(start, self.joint_constraints(joints, tolerance=.0001))
                            state = self.planned_start_state
                            reached = self.trajectory_endpoint(start, prefix)
                            descent = descent_from(reached)
                            if not self.complete_cartesian(descent):
                                continue
                        except MoveItError as exc:
                            if self.active_goal is not None:
                                raise
                            failure = exc
                            continue
                        consider(prefix, descent.solution, state)
            if best is not None:
                if best_final_quality > TASK_QUALITY_LIMIT:
                    print(f'警告：仍保留最佳放置候选，但末端质量 {best_final_quality:.2f} 仍偏弱'
                          f'（路径总质量 {best_quality:.2f}）。')
                self.planned_start_state = best_state
                return best
            raise failure or MoveItError('没有找到可完整下降且腕部不过度扭曲的直立放置路径')
        finally:
            self.args.planning_time = budget

    @staticmethod
    def joint_constraints(q, tolerance=.005):
        return Constraints(joint_constraints=[JointConstraint(
            joint_name=n, position=float(v), tolerance_above=tolerance,
            tolerance_below=tolerance, weight=1.) for n, v in q.items()])

    def orientation_constraint(self, pose, tolerance):
        constraint = OrientationConstraint(
            link_name=self.tip_link, orientation=pose.orientation, weight=1.,
            absolute_x_axis_tolerance=tolerance, absolute_y_axis_tolerance=tolerance,
            absolute_z_axis_tolerance=tolerance, parameterization=OrientationConstraint.ROTATION_VECTOR)
        constraint.header.frame_id = self.args.frame
        return constraint

    def pose_constraints(self, pose):
        position = PositionConstraint(link_name=self.tip_link, weight=1.)
        position.header.frame_id = self.args.frame
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.002])]
        position.constraint_region.primitive_poses = [deepcopy(pose)]
        return Constraints(position_constraints=[position], orientation_constraints=[
            self.orientation_constraint(pose, .02)])

    def position_constraints(self, point, offset=(0., 0., 0.)):
        """Constrain a tool-relative point's position, leaving rotation free."""
        target = Pose()
        target.orientation.w = 1.
        target.position.x, target.position.y, target.position.z = map(float, point)
        position = PositionConstraint(link_name=self.tip_link, weight=1.)
        position.header.frame_id = self.args.frame
        position.target_point_offset.x, position.target_point_offset.y, position.target_point_offset.z = map(float, offset)
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.002])]
        position.constraint_region.primitive_poses = [target]
        return Constraints(position_constraints=[position])

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.):
            raise MoveItError("MoveIt Action 未就绪")
        handle = self.wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise MoveItError("MoveIt 拒绝了目标")
        self.active_goal = handle
        response = self.wait(handle.get_result_async(), timeout)
        self.active_goal = None
        self.on_action_result(client, response)
        code = response.result.error_code
        if response.status != 4 or code.val != MoveItErrorCodes.SUCCESS:
            phase = "执行" if client is self.execution_client else "规划"
            message = f"；{code.message}" if code.message else ""
            raise MoveItError(f"MoveIt {phase}失败：status={response.status}, code={code.val}{message}")
        return response.result

    def on_action_result(self, client, response):
        """Optional application hook for consuming a completed Action result."""

    def cartesian_path(self, start, waypoints, *, group=None, link=None):
        request = GetCartesianPath.Request(
            start_state=self.planning_start_state(start), group_name=group or self.group,
            link_name=link or self.tip_link,
            waypoints=waypoints, max_step=.003, revolute_jump_threshold=.2, avoid_collisions=True,
            max_velocity_scaling_factor=self.args.velocity,
            max_acceleration_scaling_factor=self.args.acceleration)
        request.header.frame_id = self.args.frame
        constraints = getattr(self, '_motion_path_constraints', None)
        if constraints is not None:
            request.path_constraints = constraints
        return self.call(GetCartesianPath, "/compute_cartesian_path", request)

    def dual_cartesian_path(self, start, left_waypoints, right_waypoints):
        from .dual_cartesian import plan_dual_cartesian_path
        return plan_dual_cartesian_path(self, start, left_waypoints, right_waypoints, MoveItError)

    def posture_profile(self):
        if self._posture_profile is None:
            from .posture import client_profile
            self._posture_profile = client_profile(self)
        return self._posture_profile

    def posture_cost(self, positions):
        if not self.posture_profile().enabled:
            return 0.
        from .continuous import geometry
        return geometry(self).posture(positions, self.group)[0]

    def elbow_cost(self, positions):
        profile = self.posture_profile()
        if not profile.enabled:
            return 0.
        if self._elbow_preference is None:
            from .posture import ElbowPreference
            self._elbow_preference = ElbowPreference(self.fk,self.joints,profile,self.dual,self.arm_side)
        return self._elbow_preference.cost(positions)

    def _plan_with_posture(self, goal, selected_group):
        request = goal.request
        profile = self.posture_profile() if request.pipeline_id == 'ompl' and selected_group == self.group else None
        if (profile is None or not profile.enabled or profile.planning_candidates == 1
                or request.planner_id == 'RRTConnectOnline'):
            return self.action(self.planner, goal, self.args.planning_time+15.)
        from .posture import trajectory_cost
        budget, attempts = request.allowed_planning_time, request.num_planning_attempts
        best, best_cost, failure = None, math.inf, None
        try:
            request.allowed_planning_time = budget/profile.planning_candidates
            request.num_planning_attempts = 1
            for _ in range(profile.planning_candidates):
                try:
                    result = self.action(self.planner, goal, request.allowed_planning_time+15.)
                except MoveItError as exc:
                    if self.active_goal is not None:
                        raise
                    failure = exc
                    continue
                try:
                    cost = trajectory_cost(profile, result.planned_trajectory, self.joints,
                                           posture_cost=self.posture_cost)
                except ValueError as exc:
                    raise MoveItError(str(exc)) from exc
                if cost < best_cost:
                    best, best_cost = result, cost
            if best is not None:
                return best
            raise failure or MoveItError('没有可用的构型规划候选')
        finally:
            request.allowed_planning_time, request.num_planning_attempts = budget, attempts

    def plan(self, start, goal_constraints, path_constraints=None, group=None):
        if path_constraints is None and (group is None or group == self.group):
            path_constraints = getattr(self, '_motion_path_constraints', None)
        pipeline = getattr(self.args, "pipeline", "ompl")
        planner = getattr(self.args, "planner", None)
        if pipeline not in PLANNING_PIPELINES:
            raise MoveItError(f"未知规划管线：{pipeline}")
        if pipeline == "ompl":
            planner = planner or "RRTConnect"
            if planner not in (*OMPL_PLANNERS, 'RRTConnectOnline'):
                raise MoveItError(f"未知 OMPL 算法：{planner}")
        elif planner:
            raise MoveItError("--planner 仅用于选择 OMPL 算法；CHOMP/STOMP 省略此参数")
        goals = goal_constraints if isinstance(goal_constraints, list) else [goal_constraints]
        if not goals:
            raise MoveItError('规划至少需要一个目标约束')
        if pipeline == "chomp" and (len(goals) != 1 or any(
                not target.joint_constraints or target.position_constraints
                or target.orientation_constraints or target.visibility_constraints for target in goals)):
            raise MoveItError("CHOMP 仅支持关节目标；位姿目标请使用 OMPL 或 STOMP")
        if pipeline != "ompl" and path_constraints is not None:
            raise MoveItError("当前 CHOMP/STOMP 接入不支持路径约束；请使用 --pipeline ompl")
        goal = MoveGroup.Goal()
        request = goal.request
        selected_group = self.gripper_group if group == "gripper" else group or self.group
        if group == 'gripper' and planner == 'RRTConnectOnline':
            planner = 'RRTConnect'
        request.group_name, request.pipeline_id, request.planner_id = selected_group, pipeline, planner or ""
        request.start_state = (self.planning_start_state(start) if selected_group == self.group
                               else self.robot_state(start))
        request.goal_constraints = goals
        request.allowed_planning_time = self.args.planning_time
        request.num_planning_attempts = 3 if pipeline == "ompl" else 1
        request.max_velocity_scaling_factor = self.args.velocity
        request.max_acceleration_scaling_factor = self.args.acceleration
        if path_constraints is not None:
            request.path_constraints = path_constraints
        goal.planning_options.plan_only = True
        print(f"规划器：{pipeline}" + (f" / {planner}" if planner else ""))
        try:
            result = self._plan_with_posture(goal, selected_group)
        except MoveItError as exc:
            # MoveGroup can collapse an adapter's collision error into the
            # generic FAILURE (99999). Recover the actionable contact pairs.
            # Leave a timed-out active goal for cleanup to cancel immediately.
            if self.active_goal is not None:
                raise
            state = dict(zip(request.start_state.joint_state.name, request.start_state.joint_state.position))
            checks = [('规划起点', state)]
            if len(goals) == 1 and goals[0].joint_constraints:
                target = {**state, **{joint.joint_name: joint.position
                                     for joint in goals[0].joint_constraints}}
                checks.append(('目标参考姿态', target))
            for label, positions in checks:
                try:
                    validity = self.validity(positions, group=selected_group)
                except Exception:
                    break
                if not validity.valid:
                    contacts = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2)))
                                       for c in validity.contacts})
                    detail = '碰撞：'+', '.join(' ↔ '.join(pair) for pair in contacts) if contacts else '无效'
                    raise MoveItError(f'{exc}；{label}{detail}') from exc
            raise
        if planner == 'RRTConnectOnline' and selected_group == self.group:
            self.motion_posture_cost(start, result.planned_trajectory)
        self.planned_start_state = result.trajectory_start
        return result.planned_trajectory

    def show_plan(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise MoveItError("MoveIt 未返回轨迹")
        self.display.publish(DisplayTrajectory(
            model_id="onero_a1", trajectory=[trajectory],
            trajectory_start=self.planned_start_state or self.robot_state(self.start)))
        print(f"规划成功：{len(points)} 点，时长 {seconds(points[-1].time_from_start):.3f}s")

    def execute(self, trajectory, controller=None):
        points = trajectory.joint_trajectory.points
        if not points or seconds(points[-1].time_from_start) <= 0:
            print("目标与当前状态重合，无需执行运动。")
            return
        if self.startup_id is not None and self.startup_snapshot()["id"] != self.startup_id:
            raise MoveItError("规划期间服务或记录的起始位置已变化，请重新运行")
        current = self.state()
        names = trajectory.joint_trajectory.joint_names
        if any(n not in current or abs(current[n]-v) > .01 for n, v in zip(names, points[0].positions)):
            raise MoveItError("规划后起始状态已变化，请重新规划")
        if controller is None:
            controllers = self.arm_controllers
        elif isinstance(controller, str):
            controllers = [self.gripper_controller if controller == "gripper_controller" else controller]
        else:
            controllers = list(controller)
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controllers=controllers)
        duration = seconds(points[-1].time_from_start)
        settings = self.call(GetParameters, "/move_group/get_parameters", GetParameters.Request(names=[
            "trajectory_execution.allowed_execution_duration_scaling",
            "trajectory_execution.allowed_goal_duration_margin",
        ])).values
        if len(settings) != 2 or any(value.type not in (2, 3) for value in settings):
            raise MoveItError("无法读取 MoveIt 轨迹执行等待参数")
        scaling, margin = [float(value.double_value if value.type == 3 else value.integer_value)
                           for value in settings]
        if not math.isfinite(scaling) or scaling <= 0 or not math.isfinite(margin) or margin < 0:
            raise MoveItError("MoveIt 轨迹执行等待参数无效")
        self.action(self.execution_client, goal, max(30., 2.*duration+5., scaling*duration+margin+5.))
        print("MoveIt 执行成功。")

    def cleanup(self):
        ok = True
        if self.active_goal is not None:
            try:
                self.wait(self.active_goal.cancel_goal_async(), 3.)
            except Exception as exc:
                self.get_logger().error(f"取消请求失败：{exc}")
                ok = False
        if self.scene_added:
            try:
                obj = CollisionObject(id=self.scene_id, operation=CollisionObject.REMOVE)
                obj.header.frame_id = self.args.frame
                scene = PlanningScene(is_diff=True)
                scene.robot_state.is_diff = True
                scene.world.collision_objects = [obj]
                if not self.call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene)).success:
                    raise MoveItError("MoveIt 拒绝删除临时对象")
                self.scene_added = False
            except Exception as exc:
                self.get_logger().error(f"清理对象 {self.scene_id} 失败：{exc}")
                ok = False
        return ok
