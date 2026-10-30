"""Plan and inspect a complete relay on an isolated, execution-disabled MoveIt server."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time
import threading
import xml.etree.ElementTree as ET
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from moveit.core.robot_model import RobotModel
from moveit.core.planning_scene import PlanningScene
from moveit.core.collision_detection import AllowedCollisionMatrix, CollisionRequest, CollisionResult
from moveit_msgs.msg import PlanningScene as SceneMessage
from moveit_msgs.srv import ApplyPlanningScene
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from onero_a1_common.moveit_client import MoveItClient, seconds
from onero_a1_examples.bottle_scene import pose_matrix
from onero_a1_examples.dual_task_support import arm_clients, close_arm_clients
from onero_a1_examples.fixed_pick_place_config import TaskConfig
from onero_a1_examples.fixed_pick_place_relay import RelayTask
from onero_a1_examples import fixed_pick_place, fixed_pick_place_relay


def offline_scene(model, message):
    scene = PlanningScene(model)
    world = deepcopy(message.world)
    for obj in world.collision_objects:
        if obj.header.frame_id == 'world':
            obj.header.frame_id = 'base_link'
    scene.process_planning_scene_world(world)
    scene.set_current_state(message.robot_state)
    entries = message.allowed_collision_matrix
    matrix = AllowedCollisionMatrix(entries.entry_names, False)
    for i, name in enumerate(entries.entry_names):
        for j, other in enumerate(entries.entry_names):
            matrix.set_entry(name, other, entries.entry_values[i].enabled[j])
    # Snapshot defaults must be explicit for reproducible collision checking.
    if entries.default_entry_names:
        raise ValueError('快照含默认碰撞例外，需先显式展开')
    return scene, matrix


def validate(snapshot, output, planning_time):
    snapshot, output = Path(snapshot), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    recorded = deserialize_message((snapshot/'scene.bin').read_bytes(), SceneMessage)
    saved = json.loads((snapshot/'state.json').read_text())
    tf = next(t.transform for t in recorded.fixed_frame_transforms
              if t.header.frame_id == 'world' and t.child_frame_id == 'base_link')
    np.testing.assert_allclose([tf.translation.x, tf.translation.y, tf.translation.z,
                               tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w],
                              [0., 0., 0., 0., 0., 0., 1.], atol=1e-9)
    model = RobotModel(str(snapshot/'robot.urdf'), str(snapshot/'robot.srdf'))
    args = argparse.Namespace(frame='world', pipeline='ompl', planner=None, planning_time=planning_time,
                              velocity=.2, acceleration=.2, round_trip=True)
    node = MoveItClient(args)
    node.select_arm('both')
    result = node.call(GetParameters, '/move_group/get_parameters',
                       GetParameters.Request(names=['allow_trajectory_execution'])).values
    assert len(result) == 1 and result[0].type == 1 and not result[0].bool_value, '需要禁止执行的独立 MoveIt 服务'
    for name, file in [('robot_description', 'robot.urdf'), ('robot_description_semantic', 'robot.srdf')]:
        assert node.parameter('/move_group', name) == (snapshot/file).read_text(), 'MoveIt 模型与快照不同'
    initial = dict(zip(recorded.robot_state.joint_state.name, recorded.robot_state.joint_state.position))
    initial.update(saved['joints'])
    mimic = {j.get('name') for j in ET.parse(snapshot/'robot.urdf').findall('joint') if j.find('mimic') is not None}
    initial = {name: value for name, value in initial.items() if name not in mimic}
    recorded.robot_state.joint_state = JointState(name=list(initial), position=list(initial.values()))
    source = Node('relay_snapshot_state')
    publisher = source.create_publisher(JointState, '/joint_states', 1)
    def publish_state():
        message = deepcopy(recorded.robot_state.joint_state)
        message.header.stamp = source.get_clock().now().to_msg()
        publisher.publish(message)
    source.create_timer(.05, publish_state)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    recorded.robot_state.is_diff = False
    recorded.is_diff = False
    assert node.call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=recorded)).success
    check, _ = offline_scene(model, recorded)
    for link, expected in saved['poses'].items():
        np.testing.assert_allclose(check.current_state.get_global_link_transform(link), expected, atol=1e-6)
    arms = arm_clients(node)
    config = TaskConfig.load(Path(__file__).resolve().parents[2]/'src/onero_a1_examples/config/fixed_pick_place.yaml')
    task = RelayTask(node, arms, config)
    rows, views = [], []
    original = fixed_pick_place.plan_step

    def inspect(arm, start, step):
        before = task.scene.read()
        began = time.monotonic()
        trajectory = original(arm, start, step)
        elapsed = time.monotonic()-began
        scene, matrix = offline_scene(model, before)
        scene.set_current_state(step.start)
        path = trajectory.joint_trajectory
        profile = arm.posture_profile()
        bounds = [arm._joint_position_bounds[name] for name in arm.joints]
        margin, wrist, costs, elbow = [], [], [], []
        request = CollisionRequest(); request.contacts = True
        request.joint_model_group_name = 'both_arms'
        request.max_contacts = 20
        exact_start = CollisionResult()
        scene.check_collision(request, exact_start, scene.current_state, matrix)
        if exact_start.collision:
            raise RuntimeError(f'{step.label}：原始起点碰撞')
        first = path.points[0]
        error = max(abs(start[name]-value) for name, value in zip(path.joint_names, first.positions))
        if error > .000401:
            raise RuntimeError(f'{step.label}：轨迹首点偏离任务起点 {error} rad')
        times = [seconds(p.time_from_start) for p in path.points]
        if any(t < 0. for t in times) or any(b <= a for a, b in zip(times, times[1:])):
            raise RuntimeError(f'{step.label}：轨迹时间无效')
        for p in path.points:
            for values in (p.velocities, p.accelerations):
                if len(values) != len(path.joint_names) or not all(np.isfinite(values)):
                    raise RuntimeError(f'{step.label}：缺少有效的速度或加速度')
        if times[-1] <= 0. and any(np.max(np.abs(np.array(p.positions)-first.positions)) > 1e-8 for p in path.points):
            raise RuntimeError(f'{step.label}：运动轨迹时长无效')
        poses = []
        for point in path.points:
            q = {**start, **dict(zip(path.joint_names, point.positions))}
            for side, client in arms.items():
                scene.current_state.set_joint_group_positions(client.group, np.array([q[n] for n in client.joints]))
                scene.current_state.set_joint_group_active_positions(client.gripper_group, np.array([q[client.gripper_joint]]))
            scene.current_state.update()
            result = CollisionResult()
            scene.check_collision(request, result, scene.current_state, matrix)
            if result.collision:
                validity = node.validity(q, group='both_arms')
                contacts = [(c.contact_body_1, c.contact_body_2) for c in validity.contacts]
                request.verbose = True
                scene.check_collision(request, CollisionResult(), scene.current_state, matrix)
                raise RuntimeError(f'{arm.arm_side} {step.label}：离线碰撞，服务有效={validity.valid}，接触={contacts}')
            vector = np.array([q[n] for n in arm.joints])
            margin.append(min(min(v-low, high-v) for v, (low, high) in zip(vector, bounds)))
            wrist.append(float(np.max(np.abs(vector[4:]-profile.reference[4:]))))
            costs.append(float(profile.cost(vector)))
            if arm._elbow_preference is not None:
                for link, base, outward, preferred, scale, weight in arm._elbow_preference.arms:
                    distance = float((scene.current_state.get_global_link_transform(link)[:3, 3]-base)@outward)
                    elbow.append(distance)
            poses.append({link: scene.current_state.get_global_link_transform(link)[:3, 3].tolist()
                          for link in [arm.arm_side+'_'+n for n in ('base_link','Link2','Link4','Link6','Link7')]})
        row = dict(start_error_rad=error, arm=arm.arm_side, label=step.label, event=getattr(step, 'event', ''),
                   planning_s=elapsed, duration_s=seconds(path.points[-1].time_from_start), points=len(path.points),
                   collision=False, min_joint_margin_rad=min(margin), max_wrist_reference_error_rad=max(wrist),
                   max_posture_cost=max(costs), min_elbow_outward_m=min(elbow) if elbow else None)
        rows.append(row)
        index = len(rows)
        (output/f'{index:02d}-trajectory.bin').write_bytes(serialize_message(trajectory))
        (output/f'{index:02d}-scene.bin').write_bytes(serialize_message(before))
        step.audit_index = index
        views.append(dict(arm=arm.arm_side, label=step.label, poses=poses))
        print(json.dumps(row, ensure_ascii=False), flush=True)
        return trajectory

    began = time.monotonic()
    report = {'snapshot': str(snapshot.resolve()), 'planning_budget_s': planning_time, 'stages': rows}
    try:
        with patch.object(fixed_pick_place, 'plan_step', inspect), patch.object(fixed_pick_place_relay, 'plan_step', inspect):
            task.prepare(initial)
        report.update(success=True, selected_stages=[task.wait_step.audit_index]+[step.audit_index for leg in task.legs for step in leg.steps], legs=[dict(route=route, released=pose_matrix(leg.released_pose).tolist())
                                         for route, leg in zip(task.route, task.legs)])
    except Exception as exc:
        report.update(success=False, error=str(exc))
        raise
    finally:
        report['total_s'] = time.monotonic()-began
        (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        (output/'postures.json').write_text(json.dumps(views, ensure_ascii=False))
        node.call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=recorded))
        close_arm_clients(node)
        executor.shutdown()
        thread.join()
        source.destroy_node()
        node.destroy_node()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--planning-time', type=float, default=10.)
    args = parser.parse_args()
    rclpy.init()
    try:
        report = validate(args.snapshot, args.output, args.planning_time)
        print(f'完整接力通过：{len(report["stages"])} 阶段，{report["total_s"]:.2f} s；未执行轨迹。')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
