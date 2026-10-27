"""Shared motion, feedback and grasp checks for sequential manipulation examples."""
from copy import deepcopy
from dataclasses import dataclass
import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes

from .bottle_scene import matrix_pose, pose_matrix
from .support import MoveItError


def task_state(node):
    deadline = time.monotonic() + 10.
    while time.monotonic() < deadline:
        state = node.state()
        if node.gripper_joint in state:
            return {name: state[name] for name in [*node.joints, node.gripper_joint]}
    raise MoveItError('没有新鲜的夹爪反馈，请确认 gripper_model:=woan_picker 且夹爪服务就绪')


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
    elif step.kind in ('pose', 'joints'):
        constraints = (node.pose_constraints(step.target) if step.kind == 'pose'
                       else node.joint_constraints(step.target))
        trajectory = node.plan(start, constraints)
        expected = set(node.joints)
    else:
        waypoints = step.target if isinstance(step.target, list) else [step.target]
        if step.kind != 'yaw_cartesian':
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
    if step.kind == 'cartesian':
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
