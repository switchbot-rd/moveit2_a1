"""Small standard MoveIt ROS client, shared by applications and diagnostic tools.

This module handles ROS requests, fresh state, visualization and execution.
Reporting, pose archives, model fingerprints and precision acceptance belong
to the diagnostic tools, which can extend this client.
"""
from copy import deepcopy
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
        return self.wait(client.call_async(request))

    def state(self):
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            now = self.get_clock().now().nanoseconds*1e-9
            fresh = {name:value for name,(value,stamp) in self._joint_cache.items() if -.1 <= now-stamp <= 1.}
            if all(name in fresh for name in self.joints):
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
        request = GetPositionFK.Request(robot_state=self.robot_state(q), fk_link_names=[link or self.tip_link])
        request.header.frame_id = self.args.frame
        response = self.call(GetPositionFK, "/compute_fk", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise MoveItError(f"FK 失败：{response.error_code.val}")
        return response.pose_stamped[0].pose

    def validity(self, q, constraints=None, group=None):
        request = GetStateValidity.Request(robot_state=self.robot_state(q), group_name=group or self.group)
        if constraints is not None:
            request.constraints = constraints
        return self.call(GetStateValidity, "/check_state_validity", request)

    def inverse_kinematics(self, pose, seed, *, group, link):
        request = GetPositionIK.Request()
        request.ik_request.group_name = group
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = self.args.frame
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.robot_state = self.robot_state(seed)
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 1
        response = self.call(GetPositionIK, "/compute_ik", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItError(f"{group} 逆解失败：{response.error_code.val}")
        return dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))

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
        return self.call(GetCartesianPath, "/compute_cartesian_path", request)

    def dual_cartesian_path(self, start, left_waypoints, right_waypoints):
        from .dual_cartesian import plan_dual_cartesian_path
        return plan_dual_cartesian_path(self, start, left_waypoints, right_waypoints, MoveItError)

    def plan(self, start, goal_constraints, path_constraints=None, group=None):
        pipeline = getattr(self.args, "pipeline", "ompl")
        planner = getattr(self.args, "planner", None)
        if pipeline not in PLANNING_PIPELINES:
            raise MoveItError(f"未知规划管线：{pipeline}")
        if pipeline == "ompl":
            planner = planner or "RRTConnect"
            if planner not in OMPL_PLANNERS:
                raise MoveItError(f"未知 OMPL 算法：{planner}")
        elif planner:
            raise MoveItError("--planner 仅用于选择 OMPL 算法；CHOMP/STOMP 省略此参数")
        if pipeline == "chomp" and (
                not goal_constraints.joint_constraints or goal_constraints.position_constraints
                or goal_constraints.orientation_constraints or goal_constraints.visibility_constraints):
            raise MoveItError("CHOMP 仅支持关节目标；位姿目标请使用 OMPL 或 STOMP")
        if pipeline != "ompl" and path_constraints is not None:
            raise MoveItError("当前 CHOMP/STOMP 接入不支持路径约束；请使用 --pipeline ompl")
        goal = MoveGroup.Goal()
        request = goal.request
        selected_group = self.gripper_group if group == "gripper" else group or self.group
        request.group_name, request.pipeline_id, request.planner_id = selected_group, pipeline, planner or ""
        request.start_state = (self.planning_start_state(start) if selected_group == self.group
                               else self.robot_state(start))
        request.goal_constraints = [goal_constraints]
        request.allowed_planning_time = self.args.planning_time
        request.num_planning_attempts = 3 if pipeline == "ompl" else 1
        request.max_velocity_scaling_factor = self.args.velocity
        request.max_acceleration_scaling_factor = self.args.acceleration
        if path_constraints is not None:
            request.path_constraints = path_constraints
        goal.planning_options.plan_only = True
        print(f"规划器：{pipeline}" + (f" / {planner}" if planner else ""))
        try:
            result = self.action(self.planner, goal, self.args.planning_time+15.)
        except MoveItError as exc:
            # MoveGroup can collapse an adapter's collision error into the
            # generic FAILURE (99999). Recover the actionable contact pairs.
            # Leave a timed-out active goal for cleanup to cancel immediately.
            if self.active_goal is not None:
                raise
            state = dict(zip(request.start_state.joint_state.name, request.start_state.joint_state.position))
            checks = [('规划起点', state)]
            if goal_constraints.joint_constraints:
                target = {**state, **{joint.joint_name: joint.position
                                     for joint in goal_constraints.joint_constraints}}
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
