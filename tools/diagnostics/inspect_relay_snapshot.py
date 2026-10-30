"""离线检查 5 cm 水瓶与待机手臂是否碰撞；不连接 ROS 服务或机械臂。"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from moveit.core.robot_model import RobotModel
from moveit.core.planning_scene import PlanningScene
from moveit.core.collision_detection import AllowedCollisionMatrix, CollisionRequest, CollisionResult
from moveit_msgs.msg import CollisionObject, PlanningScene as SceneMessage
from rclpy.serialization import deserialize_message
from shape_msgs.msg import SolidPrimitive


def inspect(directory, side, position, joint_override=None):
    directory = Path(directory)
    snapshot = json.loads((directory/'state.json').read_text())
    recorded = deserialize_message((directory/'scene.bin').read_bytes(), SceneMessage)
    # Dual installation uses coincident world/base_link frames. Verify the
    # recorded transform before expressing world geometry in the model root.
    transform = next(t.transform for t in recorded.fixed_frame_transforms
                     if t.header.frame_id == 'world' and t.child_frame_id == 'base_link')
    if not np.allclose([transform.translation.x, transform.translation.y, transform.translation.z,
                       transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
                      [0., 0., 0., 0., 0., 0., 1.], atol=1e-9):
        raise ValueError('快照的 world/base_link 不重合，需先转换坐标')
    model = RobotModel(str(directory/'robot.urdf'), str(directory/'robot.srdf'))
    scene = PlanningScene(model)
    world = deepcopy(recorded.world)
    for obj in world.collision_objects:
        if obj.header.frame_id == 'world':
            obj.header.frame_id = 'base_link'
    scene.process_planning_scene_world(world)
    scene.set_current_state(recorded.robot_state)
    q = dict(zip(recorded.robot_state.joint_state.name, recorded.robot_state.joint_state.position))
    q.update(snapshot['joints'])
    for arm, suffix in (('left', 'l'), ('right', 'r')):
        names = [f'joint{i}-a1_{suffix}' for i in range(1, 8)]
        scene.current_state.set_joint_group_positions(arm+'_arm', np.array([q[name] for name in names]))
        scene.current_state.set_joint_group_positions(arm+'_gripper', np.array([q[arm+'_woan_picker_left_finger_joint']]))
    scene.current_state.update()
    for link, expected in snapshot['poses'].items():
        np.testing.assert_allclose(scene.current_state.get_global_link_transform(link), expected, atol=1e-6)
    if joint_override is not None:
        q.update(joint_override)
        for arm, suffix in (('left', 'l'), ('right', 'r')):
            scene.current_state.set_joint_group_positions(arm+'_arm',
                np.array([q[f'joint{i}-a1_{suffix}'] for i in range(1, 8)]))
        scene.current_state.update()
    obj = CollisionObject(id='diagnostic_bottle', operation=CollisionObject.ADD)
    obj.header.frame_id = 'base_link'
    obj.pose.orientation.w = 1.
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = map(float, position)
    body = Pose(orientation=Quaternion(w=1.))
    neck = Pose(orientation=Quaternion(w=1.)); neck.position.z = .1
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.18, .025]),
                      SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.02, .015])]
    obj.primitive_poses = [body, neck]
    scene.apply_collision_object(obj)
    if not scene.knows_frame_transform(obj.id):
        raise RuntimeError('离线瓶子模型加载失败')
    links = [link.get('name') for link in ET.parse(directory/'robot.urdf').findall('link')]
    matrix = AllowedCollisionMatrix(links+[o.id for o in world.collision_objects]+[obj.id], True)
    results = []
    for link in links:
        if not link.startswith(side+'_'):
            continue
        matrix.set_entry(obj.id, link, False)
        request = CollisionRequest(); request.distance = True; request.joint_model_group_name = side+'_arm'
        result = CollisionResult()
        scene.check_collision(request, result, scene.current_state, matrix)
        results.append({'link': link, 'collision': result.collision,
                        'distance_m': result.distance if 0. <= result.distance < 1e10 else None})
        matrix.set_entry(obj.id, link, True)
    return {'bottle_position': list(position), 'side': side,
            'pairs': sorted(results, key=lambda item: (not item['collision'],
                item['distance_m'] if item['distance_m'] is not None else float('inf')))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--side', choices=['left', 'right'], default='left')
    parser.add_argument('--position', type=float, nargs=3, default=[.455, .1, -.24])
    parser.add_argument('--joint-state', type=Path, help='可选，离线检查指定关节位置 JSON')
    args = parser.parse_args()
    joints = json.loads(args.joint_state.read_text()) if args.joint_state else None
    print(json.dumps(inspect(args.snapshot, args.side, args.position, joints), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
