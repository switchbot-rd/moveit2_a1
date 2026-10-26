"""手臂与夹爪联合任务：夹取直径 5 cm 的水瓶，带回起始姿态，停顿后放回并松爪。

先摆好手臂，运行不带 --execute 的命令生成并保存瓶子位置、预览完整取放。
按输出坐标摆放实物，再添加 --execute 执行；本次实测姿态作为起点和回位目标。
夹持中心和开度默认来自模型，
实物可用 --tcp-offset 和 --grasp-opening 指定经测量的值。
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotTrajectory
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectoryPoint
from onero_a1_common.gains import GainClient
from onero_a1_common.trajectory_tools import _curve, _duration, _scaled_limits

from .bottle_scene import BottleScene, PickerGeometry, clear_bottle, matrix_pose, pose_matrix
from .gripper import opening
from .support import MoveItError, finite, positive, run_example


def configure(parser):
    parser.add_argument('--diameter', type=positive, default=.05, help='瓶身直径，米，默认 0.05')
    parser.add_argument('--bottle-height', type=positive, default=.18, help='瓶身高度，米，默认 0.18，另含 2 cm 瓶颈')
    parser.add_argument('--reach', type=positive, default=.12, help='从起始夹持中心向前搜索的距离，米，默认 0.12')
    parser.add_argument('--lift', type=positive, default=.03, help='夹取后抬高的距离，米，默认 0.03')
    parser.add_argument('--pause', type=positive, default=2., help='带瓶回到起始姿态后的停顿，秒，默认 2')
    parser.add_argument('--open-opening', type=opening, default=1., help='张开开度 [0, 1]，默认全开')
    parser.add_argument('--grasp-opening', type=opening, help='预期接触开度 [0, 1]，省略时由瓶径和夹爪模型计算')
    parser.add_argument('--tcp-offset', type=finite, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='夹持中心相对 Link7 的偏移，米，省略时由夹爪模型计算')
    parser.add_argument('--bottle-position', type=finite, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='瓶身中心在 --frame 中的位置，米，省略时自动搜索')
    parser.add_argument('--support-height', type=finite,
                        help='核对桌面高度，米；必须与已加载桌面一致，不允许悬空放瓶')
    parser.add_argument('--orientation-tolerance', type=positive, default=.1,
                        help='每段开始时允许的末端跟踪偏差，rad，默认 0.1；目标朝向仍保持不变')


def task_state(node):
    deadline = time.monotonic() + 10.
    while time.monotonic() < deadline:
        state = node.state()
        if node.gripper_joint in state:
            return {name: state[name] for name in [*node.joints, node.gripper_joint]}
    raise MoveItError('没有新鲜的夹爪反馈，请确认 gripper_model:=woan_picker 且夹爪服务就绪')


def target_path(node):
    root = Path(os.environ.get('ROS_HOME', str(Path.home()/'.ros')))
    return root/'tasks'/f'arm_gripper_task_{node.arm_side}.json'


def rotation_distance(first, second):
    return math.acos(float(np.clip((np.trace(first.T @ second)-1.)/2., -1., 1.)))


def confirm_opening(node, target):
    deadline = time.monotonic() + 2.
    while time.monotonic() < deadline:
        actual = task_state(node)[node.gripper_joint]
        if abs(actual-target) <= .015 + 1e-9:
            return
    raise MoveItError(f'夹爪反馈未到目标开度：目标={target/1.1:.3f}，反馈={actual/1.1:.3f}')


@dataclass
class Step:
    label: str
    kind: str
    target: object
    trajectory: object = None
    start: object = None


def path_failure(node, start, waypoints, response):
    message = f'笛卡尔路径不完整：fraction={response.fraction}，code={response.error_code.val}'
    trajectory = response.solution.joint_trajectory
    if not trajectory.points:
        return message
    last = {**start, **dict(zip(trajectory.joint_names, trajectory.points[-1].positions))}
    try:
        remaining = np.linalg.norm(pose_matrix(waypoints[-1])[:3, 3]-pose_matrix(node.fk(last))[:3, 3])
        message += f'，距目标约 {remaining*1000:.1f} mm'
        target = node.inverse_kinematics(waypoints[-1], last, group=node.group, link=node.tip_link)
        validity = node.validity({**last, **{name: target[name] for name in node.joints}})
        if validity.valid:
            message += '；目标逆解有效，但当前直线路径未通过检查'
        else:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
            message += f'；目标状态无效，接触对={contacts}'
    except MoveItError as exc:
        message += f'；{exc}'
    return message


def plan_step(node, start, step):
    if step.kind == 'gripper':
        trajectory = node.plan(start, node.joint_constraints({node.gripper_joint: step.target}), group='gripper')
        expected = {node.gripper_joint}
    else:
        waypoints = step.target if isinstance(step.target, list) else [step.target]
        reference = pose_matrix(waypoints[-1])[:3, :3]
        if any(rotation_distance(reference, pose_matrix(pose)[:3, :3]) > 1e-6 for pose in waypoints):
            raise MoveItError('本任务只允许保持朝向的平移，不接受旋转目标')
        start_rotation = pose_matrix(node.fk(start))[:3, :3]
        start_error = rotation_distance(reference, start_rotation)
        if start_error > node.args.orientation_tolerance:
            raise MoveItError(f'当前末端跟踪偏差 {start_error:.4f} rad 超过 '
                              f'--orientation-tolerance={node.args.orientation_tolerance:g} rad')
        response = node.cartesian_path(start, waypoints)
        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                or not math.isfinite(response.fraction) or response.fraction < 1.-1e-6):
            raise MoveItError(path_failure(node, start, waypoints, response))
        trajectory = response.solution
        node.planned_start_state = response.start_state
        expected = set(node.joints)
    names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    if (not points or len(names) != len(expected) or set(names) != expected
            or any(len(point.positions) != len(names)
                   or not all(math.isfinite(value) for value in point.positions) for point in points)):
        raise MoveItError('MoveIt 返回了空轨迹或不完整的关节轨迹')
    if step.kind != 'gripper':
        # Cartesian planning starts at measured feedback. A previous action
        # can succeed with a small tracking residual; allow that existing
        # residual to converge, but reject new rotations introduced by a plan.
        for point in points:
            state = {**start, **dict(zip(names, point.positions))}
            rotation = pose_matrix(node.fk(state))[:3, :3]
            target_error = rotation_distance(reference, rotation)
            travelled = rotation_distance(start_rotation, rotation)
            if target_error+travelled > start_error+.01:
                raise MoveItError(f'规划产生了额外的夹爪旋转：起点跟踪偏差={start_error:.4f} rad，'
                                  f'轨迹偏差={target_error:.4f} rad')
    return trajectory


def rounded_waypoints(start, waypoints, radius=.01):
    """Round translation corners; retain the final pose and fixed orientation."""
    points = [pose_matrix(start)[:3, 3]]
    for pose in waypoints:
        position = pose_matrix(pose)[:3, 3]
        if np.linalg.norm(position-points[-1]) > 1e-8:
            points.append(position)
    # Collinear lift/approach markers need no separate stop or corner.
    index = 1
    while index < len(points)-1:
        a, b = points[index]-points[index-1], points[index+1]-points[index]
        if np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)) > 1.-1e-8:
            points.pop(index)
        else:
            index += 1
    result = []
    for index in range(1, len(points)-1):
        a, corner, b = points[index-1:index+2]
        incoming, outgoing = corner-a, b-corner
        length_a, length_b = np.linalg.norm(incoming), np.linalg.norm(outgoing)
        incoming, outgoing = incoming/length_a, outgoing/length_b
        if np.dot(incoming, outgoing) < -.99:
            result.append(corner)
            continue
        distance = min(radius, length_a/4., length_b/4.)
        controls = np.array([corner-distance*incoming, corner-.5*distance*incoming,
                             corner, corner, corner+.5*distance*outgoing, corner+distance*outgoing])
        for t in np.linspace(0., 1., max(6, math.ceil(2.*distance/.002))+1):
            result.append(sum(math.comb(5, k)*(1.-t)**(5-k)*t**k*controls[k] for k in range(6)))
    result.append(points[-1])
    poses = []
    for position in result:
        pose = deepcopy(waypoints[-1])
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        poses.append(pose)
    return poses


def plan_carry(node, scene, start, steps):
    """Plan from feedback once; preserve commanded endpoints throughout carrying."""
    names = list(node.joints)
    limits = _scaled_limits(node, names, MoveItError)
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = names
    points = result.joint_trajectory.points
    current, elapsed = dict(start), 0.
    first_state = None
    try:
        for begin, end, label in ((4, 6, '提起并回到起点'), (6, 8, '返回并放回原位')):
            # Retiming sees each complete leg, so intermediate lift/place
            # markers no longer introduce zero-velocity boundaries.
            scene.allow_touch(True, support=True)
            support_allowed = True
            leg_start = node.fk(current)
            waypoints = [pose for step in steps[begin:end] for pose in
                         (step.target if isinstance(step.target, list) else [step.target])]
            step = Step(label, 'cartesian', rounded_waypoints(leg_start, waypoints))
            print(f'连续规划 [{begin+1}–{end}/9] {label}')
            trajectory = plan_step(node, current, step)
            if first_state is None:
                first_state = deepcopy(node.planned_start_state)
            times, curve, scale = _curve(trajectory, names, limits, MoveItError)
            if points and not np.allclose(points[-1].positions, curve(0.), atol=1e-7, rtol=0.):
                raise MoveItError('连续搬运的相邻轨迹端点不一致')
            reference = pose_matrix(step.target[-1])[:3, :3]
            start_rotation = pose_matrix(node.fk(current))[:3, :3]
            start_error = rotation_distance(reference, start_rotation)
            contact = pose_matrix(leg_start if begin == 4 else steps[7].target)[:3, 3]
            # Preserve every spline knot and bound joint travel between checks.
            count = max(1, math.ceil(float(abs(curve.derivative().c).max())*times[-1]/.02))
            samples = sorted(set(times + np.linspace(0., times[-1], count+1).tolist()))
            previous = None
            for time_ in samples:
                if previous is not None and time_-previous < 1e-8:
                    continue
                positions = curve(time_).tolist()
                state = {**current, **dict(zip(names, positions))}
                tip = pose_matrix(node.fk(state))
                # Permit bottle/support contact only in the local lift/place
                # column. All other samples keep the strict carry collision rules.
                near_contact = (np.linalg.norm(tip[:2, 3]-contact[:2]) <= .025
                                and tip[2, 3] <= contact[2]+node.args.lift+.002)
                if near_contact != support_allowed:
                    scene.allow_touch(True, support=near_contact)
                    support_allowed = near_contact
                if not node.validity(state).valid:
                    raise MoveItError(f'{label}的连续插值存在碰撞或无效状态')
                rotation = tip[:3, :3]
                if (rotation_distance(reference, rotation)+rotation_distance(start_rotation, rotation)
                        > start_error+.01):
                    raise MoveItError(f'{label}的连续插值产生了额外的夹爪旋转')
                if time_ != 0. or not points:
                    points.append(JointTrajectoryPoint(positions=positions,
                        velocities=(curve(time_, nu=1)/scale).tolist(),
                        accelerations=(curve(time_, nu=2)/scale**2).tolist(),
                        time_from_start=_duration(elapsed+time_*scale)))
                previous = time_
            current.update(zip(names, points[-1].positions))
            elapsed += times[-1]*scale
            if begin == 4:
                # Hold exactly the same command during the pause, in the same
                # controller goal; restarting from lagged feedback sheds PD support.
                node.carry_pause_window = (elapsed, elapsed+node.args.pause)
                elapsed += node.args.pause
                hold = deepcopy(points[-1])
                hold.time_from_start = _duration(elapsed)
                points.append(hold)
    finally:
        # The joined goal includes contact at both ends. Carrying samples above
        # have already been checked without the bottle/support exception.
        scene.allow_touch(True, support=True)
    node.planned_start_state = first_state
    return result


class CarryFeedback:
    """Capture the actual home hold without interrupting the controller goal."""
    def __init__(self, node, seed):
        self.node, self.seed, self.sample = node, seed, None
        self.subscription = node.create_subscription(FollowJointTrajectory.Impl.FeedbackMessage,
            '/'+node.arm_controllers[0]+'/follow_joint_trajectory/_action/feedback', self.receive, 10)

    def receive(self, message):
        feedback = message.feedback
        elapsed = feedback.desired.time_from_start.sec+feedback.desired.time_from_start.nanosec/1e9
        begin, end = self.node.carry_pause_window
        if begin+.5*(end-begin) <= elapsed <= end-.05:
            if (set(feedback.joint_names) == set(self.node.joints)
                    and len(feedback.actual.positions) == len(self.node.joints)
                    and len(feedback.desired.positions) == len(self.node.joints)):
                self.sample = (dict(zip(feedback.joint_names, feedback.desired.positions)),
                               dict(zip(feedback.joint_names, feedback.actual.positions)))

    def close(self):
        self.node.destroy_subscription(self.subscription)
        if self.sample is None or getattr(self.node, 'active_goal', None) is not None:
            return
        target, actual = self.sample
        try:
            expected = self.node.fk({**self.seed, **target}).position
            measured = self.node.fk({**self.seed, **actual}).position
            worst = max(self.node.joints, key=lambda name: abs(actual[name]-target[name]))
            print(f'回位实测（模型 FK）：目标高度={expected.z:.4f} m，实测高度={measured.z:.4f} m，'
                  f'高度差={(measured.z-expected.z)*1000:+.1f} mm；'
                  f'最大关节偏差 {worst}={actual[worst]-target[worst]:+.4f} rad。')
        except MoveItError as exc:
            print(f'回位反馈已收到，但 FK 读取失败：{exc}')


def candidates(node, initial, offset, scene=None):
    tip = pose_matrix(node.fk(initial))
    forward = tip[:3, 2].copy()
    forward[2] = 0.
    if np.linalg.norm(forward) < .2:
        forward = tip[:3, 3].copy()
        forward[2] = 0.
    if np.linalg.norm(forward) < .01:
        forward = np.array([1., 0., 0.])
    forward /= np.linalg.norm(forward)
    rotation = tip[:3, :3].copy()
    center = tip[:3, 3] + tip[:3, :3] @ offset
    positions = ([np.array(node.args.bottle_position)] if node.args.bottle_position is not None else
                 [center + forward*node.args.reach*scale for scale in (1., .8, .6)])
    for position in positions:
        support = scene.support_height(position, node.args.diameter/2.) if scene is not None else None
        if support is None:
            print(f'跳过候选位置 ({position[0]:.3f}, {position[1]:.3f})：下方没有已加载的桌面。')
            continue
        if node.args.support_height is not None and abs(node.args.support_height-support) > .001:
            raise MoveItError('--support-height 与场景桌面高度不一致，请先修正桌面模型')
        table_center = support+node.args.bottle_height/2.
        if node.args.bottle_position is not None and abs(position[2]-table_center) > .001:
            raise MoveItError('指定或保存的瓶底不在桌面上，请去掉 --execute 重新预览')
        position[2] = table_center
        bottle = np.eye(4); bottle[:3, 3] = position
        # A fixed bottle can be gripped at several heights on its cylindrical
        # body. Near full extension, lifting the contact point by 1 cm can
        # restore reachability without moving the bottle or rotating the wrist.
        margin = max(0., node.args.bottle_height/2.-.025)
        heights = [height for height in (0., .01, .02, .03) if height <= margin]
        for height in heights:
            grasp = np.eye(4); grasp[:3, :3] = rotation
            grasp[:3, 3] = position - rotation @ offset
            grasp[2, 3] += height
            pregrasp = grasp.copy(); pregrasp[:3, 3] -= forward*.05
            lifted = grasp.copy(); lifted[2, 3] += node.args.lift
            yield matrix_pose(bottle), matrix_pose(pregrasp), matrix_pose(grasp), matrix_pose(lifted)


def prepare(node, scene, initial, grasp_angle, offset):
    errors = []
    for index, (bottle, pregrasp, grasp, lifted) in enumerate(candidates(node, initial, offset, scene), 1):
        contact_height = (pose_matrix(grasp) @ [*offset, 1.])[2]-bottle.position.z
        print(f'检查抓取方案 {index}：水瓶中心='
              f'({bottle.position.x:.3f}, {bottle.position.y:.3f}, {bottle.position.z:.3f}) m，'
              f'抓取点高于瓶身中心 {contact_height*1000:.0f} mm')
        scene.place(bottle, node.args.diameter, node.args.bottle_height)
        scene.allow_touch(True, support=True)
        origin = node.fk(initial)
        safe_height = max(origin.position.z, lifted.position.z)
        origin_high = deepcopy(origin); origin_high.position.z = safe_height
        pregrasp_high = deepcopy(pregrasp); pregrasp_high.position.z = safe_height
        grasp_high = deepcopy(grasp); grasp_high.position.z = safe_height
        steps = [
            Step('张开夹爪', 'gripper', node.args.open_opening*1.1),
            Step('保持朝向平移到水瓶前方', 'cartesian', [origin_high, pregrasp_high, pregrasp]),
            Step('直线接近水瓶', 'cartesian', grasp),
            Step('夹住水瓶', 'gripper', grasp_angle),
            Step('提起水瓶', 'cartesian', lifted),
            Step('保持朝向带瓶回到起始位置', 'cartesian', [grasp_high, origin_high, origin]),
            Step('保持朝向带瓶返回放置点上方', 'cartesian', [origin_high, grasp_high, lifted]),
            Step('下降并放回原位', 'cartesian', grasp),
            Step('松开夹爪', 'gripper', node.args.open_opening*1.1),
        ]
        virtual = dict(initial)
        try:
            if not node.validity(initial).valid:
                raise MoveItError('该瓶子位置与起始机器人状态碰撞')
            for step_index, step in enumerate(steps):
                if step_index == 7:
                    scene.allow_touch(True, support=True)
                print(f'预检查 [{step_index+1}/{len(steps)}] {step.label}')
                step.start = scene.state(virtual)
                step.trajectory = plan_step(node, virtual, step)
                trajectory = step.trajectory.joint_trajectory
                virtual.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
                if step_index == 3:
                    scene.attach(virtual)
                elif step_index == 4:
                    scene.allow_touch(True, support=False)
            return steps
        except MoveItError as exc:
            errors.append(str(exc))
            print(f'抓取方案不可用：{exc}')
        finally:
            # Virtual planning must never leave an object attached to the live robot.
            scene.world()
    raise MoveItError('没有找到桌面上可保持起始夹爪朝向完成取放的抓取方案：'
                      + (f'已尝试 {len(errors)} 个方案，最后失败：{errors[-1]}' if errors
                         else '请加载桌面环境，或调整手臂起始位置'))


def wait_seconds(node, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if not rclpy.ok():
            raise MoveItError('ROS 已关闭，任务停止')
        rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))


def wait_for_arm_settled(node, timeout=5.):
    """Wait for fresh feedback to settle after changing the position gain."""
    print('等待增益切换后的手臂反馈稳定。')
    earliest = node.get_clock().now().nanoseconds
    deadline = time.monotonic()+timeout
    reference, since, previous = None, None, earliest-1
    while time.monotonic() < deadline:
        if not rclpy.ok():
            raise MoveItError('ROS 已关闭，任务停止')
        rclpy.spin_once(node, timeout_sec=.02)
        message = node.latest
        if message is None:
            continue
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)/1e9
        if stamp < earliest or stamp <= previous or not -.1 <= age <= .1:
            continue
        positions = dict(zip(message.name, message.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in node.joints):
            continue
        if (reference is None or stamp-previous > 100_000_000
                or any(abs(positions[name]-reference[name]) > .003 for name in node.joints)):
            reference, since = positions, stamp
        elif stamp-since >= 300_000_000:
            return
        previous = stamp
    raise MoveItError('增益切换后未获得连续稳定的手臂反馈，尚未下发提起轨迹')


def preview_segment(trajectory):
    # Retimed copies are display-only. Controller trajectories remain intact.
    result = deepcopy(trajectory)
    points = result.joint_trajectory.points
    indices = sorted(set(np.linspace(0, len(points)-1, min(24, len(points)), dtype=int)))
    result.joint_trajectory.points = [deepcopy(points[index]) for index in indices]
    for tick, point in enumerate(result.joint_trajectory.points):
        point.time_from_start = Duration(sec=tick//20, nanosec=(tick % 20)*50_000_000)
        point.velocities = []; point.accelerations = []; point.effort = []
    return result


def preview(node, scene, steps):
    print('预览完整取放：夹取 → 回起点 → 停顿 → 放回 → 松爪（不驱动硬件）。')
    try:
        for begin, end in ((0, 4), (4, 8), (8, 9)):
            if begin == 4:
                scene.hide_world()
            trajectories = []
            for index in range(begin, end):
                trajectories.append(preview_segment(steps[index].trajectory))
                if index == 5:
                    hold = deepcopy(trajectories[-1])
                    last = deepcopy(hold.joint_trajectory.points[-1])
                    count = max(2, math.ceil(node.args.pause/.05))
                    hold.joint_trajectory.points = [deepcopy(last) for _ in range(count)]
                    for tick, point in enumerate(hold.joint_trajectory.points):
                        point.time_from_start = Duration(sec=tick//20, nanosec=(tick % 20)*50_000_000)
                    trajectories.append(hold)
            node.display.publish(DisplayTrajectory(model_id='onero_a1', trajectory_start=steps[begin].start,
                                                    trajectory=trajectories))
            # The supplied RViz config uses 0.05 s/state; also allow for render frames.
            count = sum(len(item.joint_trajectory.points) for item in trajectories)
            wait_seconds(node, count*(.05+1./30.)+.2)
    finally:
        scene.world()


def grasp_bottle(node, geometry):
    """Require contact at a bottle-sized opening, then keep a small preload.

    The standard controller reports a blocked close as ABORTED/stalled. Only
    this task interprets that specific result as contact, after checking its
    measured width. An empty full close or any other failure is rejected.
    """
    client = ActionClient(node, GripperCommand, '/'+node.gripper_controller+'/gripper_cmd')
    # Keep the client alive if a timeout leaves a goal for run_example.cleanup.
    node.bottle_gripper_client = client
    if not client.wait_for_server(timeout_sec=10.):
        raise MoveItError('夹爪动作服务不可用')

    def command(angle):
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = float(angle), 0.
        handle = node.wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise MoveItError('夹爪拒绝夹取指令')
        node.active_goal = handle
        response = node.wait(handle.get_result_async(), 12.)
        node.active_goal = None
        return response

    response = command(0.)
    contact = response.result
    print(f'夹取反馈：status={response.status}，reached_goal={contact.reached_goal}，'
          f'stalled={contact.stalled}，开度={contact.position/1.1:.4f}。')
    if (response.status == GoalStatus.STATUS_SUCCEEDED and contact.reached_goal
            and not contact.stalled and abs(contact.position) <= .015):
        raise MoveItError('夹爪已全闭，未夹到实体水瓶；当前是真机模式，RViz 水瓶模型不会产生接触阻力。'
                          '请将实物放在输出的桌面位置后重新预览和执行')
    if (response.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
            or not contact.stalled or contact.reached_goal or not math.isfinite(contact.position)
            or not 0. < contact.position < 1.1):
        raise MoveItError(f'夹取未检测到有效接触（status={response.status}，'
                          f'reached_goal={contact.reached_goal}，stalled={contact.stalled}，'
                          f'位置={contact.position:.4f} rad）；未开始搬运')
    width = geometry.gap(contact.position)
    matched = (abs(width-node.args.diameter) <= .008 if node.args.grasp_opening is None else
               abs(contact.position-node.args.grasp_opening*1.1) <= .08)
    if not matched:
        raise MoveItError(f'受阻开度对应宽度 {width*1000:.1f} mm 与瓶径不符；未开始搬运')
    actual = task_state(node)[node.gripper_joint]
    if abs(actual-contact.position) > .015:
        raise MoveItError('夹爪接触结果与最新反馈不一致；未开始搬运')
    # Within the controller's 0.01 rad success tolerance, retain a closing
    # position error instead of releasing preload with a measured-position hold.
    response = command(max(0., actual-.005))
    if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.reached_goal:
        raise MoveItError('夹爪预紧未完成；未开始搬运')
    print(f'已检测到接触：模型对应夹持宽度 {width*1000:.1f} mm，保持位置预紧。')


def execute(node, scene, steps, geometry=None, gains=None):
    scene.allow_touch(True, support=True)
    for index, step in enumerate(steps):
        if 5 <= index <= 7:
            continue
        if index == 4:
            print(f'[5–8/{len(steps)}] 连续搬运：提起 → 回起点 → 停顿 {node.args.pause:g} 秒 → 放回')
            try:
                current = task_state(node)
                trajectory = plan_carry(node, scene, current, steps)
                node.show_plan(trajectory)
                feedback = CarryFeedback(node, current)
                try:
                    node.execute(trajectory)
                finally:
                    feedback.close()
            except (Exception, KeyboardInterrupt) as exc:
                raise MoveItError(f'第 5–8 步「连续搬运」停止，后续动作未执行：'
                                  f'{str(exc) or "interrupted"}') from exc
            continue
        print(f'[{index+1}/{len(steps)}] {step.label}')
        try:
            current = task_state(node)
            trajectory = plan_step(node, current, step)
            node.show_plan(trajectory)
            if index == 3 and not node.bottle_simulated:
                grasp_bottle(node, geometry)
            else:
                node.execute(trajectory, controller='gripper_controller' if step.kind == 'gripper' else None)
            if step.kind == 'gripper' and (index != 3 or node.bottle_simulated):
                confirm_opening(node, step.target)
            if index == 3:
                measured = task_state(node)
                if gains is not None:
                    gains.set_kp(120.)
                    print('夹取后位置增益已生效：kp=120。')
                scene.attach(measured)
                if gains is not None:
                    wait_for_arm_settled(node)
            elif index == 8:
                if gains is not None:
                    gains.restore()
                    print(f'已恢复执行前位置增益：kp={gains.original_kp:g}。')
                scene.world()
        except (Exception, KeyboardInterrupt) as exc:
            raise MoveItError(f'第 {index+1} 步「{step.label}」停止，后续动作未执行：'
                              f'{str(exc) or "interrupted"}') from exc


def saved_target(node, initial, geometry, offset, angle):
    bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
    values = node.call(GetParameters, bridge+'/get_parameters',
                      GetParameters.Request(names=['simulate', 'model', 'can_channel', 'mounting'])).values
    if len(values) != 4 or values[0].type != 1 or any(value.type != 4 for value in values[1:]):
        raise MoveItError('无法确认当前机械臂的模拟模式与设备配置')
    simulated = values[0].bool_value
    node.bottle_simulated = simulated
    print('运行模式：模拟。' if simulated else '运行模式：真机，执行取放需要按桌面坐标摆放实体水瓶。')
    identity = [value.string_value for value in values[1:]]
    # Only explicit settings belong to the saved-target identity. Current
    # joint feedback and model-derived TCP/opening are runtime planning data.
    config = {'version': 3, 'frame': node.args.frame, 'joints': node.joints,
              'simulate': simulated, 'device': identity, 'diameter': node.args.diameter,
              'bottle_height': node.args.bottle_height, 'lift': node.args.lift,
              'open_opening': node.args.open_opening, 'grasp_opening': node.args.grasp_opening,
              'tcp_offset': node.args.tcp_offset, 'support_height': node.args.support_height}
    path = target_path(node)
    if node.args.execute:
        if not path.exists():
            if not simulated:
                raise MoveItError('真机执行前请先去掉 --execute，生成瓶子位置并按坐标摆放实物')
        else:
            saved = json.loads(path.read_text())
            saved_config = saved.get('config')
            if isinstance(saved_config, str):
                # Validate legacy records against their own reference pose,
                # solely to recover settings. Never use it as today's start.
                old_initial = saved.get('initial', {})
                if any(name not in old_initial or not math.isfinite(old_initial[name]) for name in node.joints):
                    raise MoveItError('旧预览记录缺少配置参考，请先重新预览')
                old_geometry = PickerGeometry(node, node.args.diameter,
                                              pose_matrix(node.fk(old_initial))[:3, :3])
                old_angle = old_geometry.angle if node.args.grasp_opening is None else node.args.grasp_opening*1.1
                old_offset = (old_geometry.center(old_angle) if node.args.tcp_offset is None
                              else np.array(node.args.tcp_offset))
                legacy = ['table_translation_v2', node.args.frame, node.joints, simulated, identity,
                          node.args.diameter, node.args.bottle_height, node.args.lift, node.args.open_opening,
                          old_angle, list(map(float, old_offset)), node.args.support_height]
                expected = hashlib.sha256(json.dumps(legacy, sort_keys=True).encode()).hexdigest()
                if saved_config != expected:
                    raise MoveItError('设备或抓取参数与旧预览记录不一致，请先重新预览')
            elif saved_config != config:
                changed = [key for key in config if not isinstance(saved_config, dict)
                           or saved_config.get(key) != config[key]]
                raise MoveItError('设备或抓取参数已变化：'+', '.join(changed)+'；请先重新预览')
            if node.args.bottle_position is not None and not np.allclose(
                    node.args.bottle_position, saved['position'], atol=1e-6, rtol=0.):
                raise MoveItError('执行位置与预览位置不同，请先重新预览')
            node.args.bottle_position = saved['position']
    return path, config


def example(node):
    if node.args.frame != 'world':
        raise MoveItError('桌面取放任务使用 --frame world，以 world Z 表示竖直方向')
    # This run's measured pose defines its start/return target. Preview only
    # fixes the physical bottle position, never the arm's starting posture.
    initial = task_state(node)
    geometry = PickerGeometry(node, node.args.diameter, pose_matrix(node.fk(initial))[:3, :3])
    angle = geometry.angle if node.args.grasp_opening is None else node.args.grasp_opening*1.1
    if angle >= node.args.open_opening*1.1:
        raise MoveItError('张开开度必须大于夹取开度')
    offset = geometry.center(angle) if node.args.tcp_offset is None else np.array(node.args.tcp_offset)
    path, digest = saved_target(node, initial, geometry, offset, angle)
    gains = None
    if node.args.execute:
        bridge = f'/{node.arm_side}/onero_topic_bridge' if node.dual else '/onero_topic_bridge'
        gains = GainClient(node, bridge)
    scene = BottleScene(node, geometry)
    steps = prepare(node, scene, initial, angle, offset)
    p = scene.bottle.pose.position
    if not node.args.execute:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'config': digest, 'initial': initial,
            'position': [p.x, p.y, p.z]}, ensure_ascii=False, indent=2))
        temporary.replace(path)
    print(f'水瓶位置：参考系={node.args.frame}，瓶身中心=({p.x:.4f}, {p.y:.4f}, {p.z:.4f}) m，'
          f'瓶底高度={p.z-node.args.bottle_height/2.:.4f} m，直径={node.args.diameter:.3f} m。')
    print(f'夹取开度={angle/1.1:.4f}，Link7 到夹持中心偏移={offset.round(5).tolist()} m。')
    contact_height = (pose_matrix(steps[2].target) @ [*offset, 1.])[2]-p.z
    print(f'选定抓取点高于瓶身中心 {contact_height*1000:.0f} mm，水瓶位置保持不变。')
    if node.args.execute:
        execute(node, scene, steps, geometry, gains)
        print('取放轨迹执行完成，已放回并松爪；回位精度见实测反馈。')
    else:
        print(f'目标已保存：{path}。按坐标摆放水瓶后，以相同参数添加 --execute。')
        preview(node, scene, steps)


def main(args=None):
    return run_example('arm_gripper_task', __doc__, configure, example, args,
                       planning_pipelines=(), cleanup=clear_bottle)
