"""Woan Picker geometry and scoped collision-scene changes for the bottle task."""
from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from moveit_msgs.msg import (
    AllowedCollisionEntry, AttachedCollisionObject, CollisionObject, ObjectColor,
    DisplayTrajectory, PlanningScene, PlanningSceneComponents, PlanningSceneWorld, RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .support import MoveItError


def pose_matrix(pose):
    result = np.eye(4)
    q = pose.orientation
    result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def matrix_pose(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    return pose


def remove_collision_entry(matrix, object_id):
    if object_id in matrix.entry_names:
        index = list(matrix.entry_names).index(object_id)
        matrix.entry_names.pop(index)
        matrix.entry_values.pop(index)
        for row in matrix.entry_values:
            row.enabled.pop(index)
    if object_id in matrix.default_entry_names:
        index = list(matrix.default_entry_names).index(object_id)
        matrix.default_entry_names.pop(index)
        matrix.default_entry_values.pop(index)


def task_bottle_ids(node):
    return (f'onero_task_bottle_{node.arm_side}', f'onero_fixed_bottle_{node.arm_side}')


def clear_bottle(node, object_id=None):
    """Remove this arm's virtual task model and collision overrides on exit."""
    object_id = object_id or f'onero_task_bottle_{node.arm_side}'
    components = (PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.WORLD_OBJECT_NAMES
                  | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                  | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    current = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
        components=PlanningSceneComponents(components=components))).scene
    attached = next((item for item in current.robot_state.attached_collision_objects
                     if item.object.id == object_id), None)
    if attached is not None:
        detach = PlanningScene(is_diff=True)
        detach.robot_state.is_diff = True
        detach.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name=attached.link_name, object=CollisionObject(id=object_id, operation=CollisionObject.REMOVE))]
        if not node.call(ApplyPlanningScene, '/apply_planning_scene',
                          ApplyPlanningScene.Request(scene=detach)).success:
            raise MoveItError('MoveIt 拒绝移除附着的水瓶模型')
        # Detaching may return the geometry to the world. Read its resulting
        # state before removing it, without issuing a REMOVE for a missing ID.
        current = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
            components=PlanningSceneComponents(components=components))).scene
    present = any(item.id == object_id for item in current.world.collision_objects)
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    if present:
        scene.world.collision_objects = [CollisionObject(id=object_id, operation=CollisionObject.REMOVE)]
    remove_collision_entry(current.allowed_collision_matrix, object_id)
    scene.allowed_collision_matrix = current.allowed_collision_matrix
    if not node.call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=scene)).success:
        raise MoveItError('MoveIt 拒绝清理水瓶模型')
    # RViz's last trajectory can cache an attached bottle independently of the
    # planning scene. Replace it with a stationary, bottle-free display state.
    state = deepcopy(current.robot_state)
    state.is_diff = False
    state.attached_collision_objects = [item for item in state.attached_collision_objects
                                        if item.object.id != object_id]
    if state.joint_state.name:
        trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(
            joint_names=list(state.joint_state.name), points=[JointTrajectoryPoint(
                positions=list(state.joint_state.position))]))
        node.display.publish(DisplayTrajectory(model_id='onero_a1', trajectory_start=state, trajectory=[trajectory]))
        node.display.wait_for_all_acked(Duration(seconds=.5))
    print(f'已清理本示例的水瓶模型：{object_id}')


def clear_task_bottles(node):
    """Remove this arm's example bottles, including legacy aliases and ACM rows."""
    components = (PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                  | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
    current = node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
        components=PlanningSceneComponents(components=components))).scene
    ids = ({obj.id for obj in current.world.collision_objects}
           | {item.object.id for item in current.robot_state.attached_collision_objects}
           | set(current.allowed_collision_matrix.entry_names)
           | set(current.allowed_collision_matrix.default_entry_names))
    errors = []
    for object_id in task_bottle_ids(node):
        if object_id in ids:
            try:
                clear_bottle(node, object_id)
            except Exception as exc:
                errors.append(f'{object_id}: {exc}')
    if errors:
        raise MoveItError('任务模型清理失败：'+'；'.join(errors))


class PickerGeometry:
    """Nominal grasp frame from the imported Woan finger-pad geometry.

    The pad points are on the inner faces of Link_L3/Link_R3.STL. This is a
    model-derived setting; measured TCP/opening overrides remain available.
    """
    def __init__(self, node, diameter, rotation=None):
        model = ET.fromstring(node.parameter('/move_group', 'robot_description'))
        self.joints = {joint.find('child').get('link'): joint for joint in model.findall('joint')}
        self.tip = node.tip_link
        self.master = node.gripper_joint
        self.rotation = rotation
        self.prefix = self.master.removesuffix('woan_picker_left_finger_joint')
        self.left = self.prefix+'woan_picker_left_finger_link_3'
        self.right = self.prefix+'woan_picker_right_finger_link_3'
        self.mimic_names = {joint.get('name') for joint in self.joints.values()
                            if joint.find('mimic') is not None}
        if self.left not in self.joints or self.right not in self.joints:
            raise MoveItError('本任务需要 gripper_model:=woan_picker')
        low, high = self.gap(0.), self.gap(1.1)
        if not low < diameter < high:
            raise MoveItError(f'保持起始夹爪朝向时，水平夹持范围为 {low:.3f}..{high:.3f} m，'
                              f'无法侧夹直径 {diameter:.3f} m 的直立水瓶；请调整起始夹爪朝向')
        self.angle = brentq(lambda value: self.gap(value)-diameter, 0., 1.1)
        self.offset = self.center(self.angle)
        self.touch_links = [link for link in self.joints if link.startswith(self.prefix+'woan_picker_')]

    def transform(self, link, angle):
        if link == self.tip:
            return np.eye(4)
        joint = self.joints[link]
        origin = joint.find('origin')
        transform = np.eye(4)
        if origin is not None:
            transform[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
            transform[:3, :3] = Rotation.from_euler(
                'xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]).as_matrix()
        if joint.get('type') != 'fixed':
            mimic = joint.find('mimic')
            value = angle if joint.get('name') == self.master else 0.
            if mimic is not None:
                value = angle*float(mimic.get('multiplier', '1'))+float(mimic.get('offset', '0'))
            axis = np.array([float(v) for v in joint.find('axis').get('xyz').split()])
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(axis*value).as_matrix()
            transform = transform @ rotation
        return self.transform(joint.find('parent').get('link'), angle) @ transform

    def pads(self, angle):
        left = self.transform(self.left, angle) @ [-.0252, -.04, 0., 1.]
        right = self.transform(self.right, angle) @ [.0252, -.04, 0., 1.]
        return left[:3], right[:3]

    def gap(self, angle):
        left, right = self.pads(angle)
        delta = left-right
        if self.rotation is not None:
            delta = (self.rotation @ delta)[:2]
        return float(np.linalg.norm(delta))

    def center(self, angle):
        left, right = self.pads(angle)
        return (left+right)/2.


class BottleScene:
    def __init__(self, node, geometry, object_id=None):
        self.node, self.geometry = node, geometry
        self.id = object_id or f'onero_task_bottle_{node.arm_side}'
        self.bottle = None
        self.attached = False
        self.changed = False
        self.support_id = None
        current = self.read()
        managed = set(task_bottle_ids(node))
        if any(obj.object.id in managed | {self.id} for obj in current.robot_state.attached_collision_objects):
            raise MoveItError('场景中已有本任务的附着水瓶；先确认上次任务的持物状态')
        # Example switches reuse one physical bottle. Old temporary aliases
        # must not become a second obstacle at its current location.
        if self.id in managed:
            present = {obj.id for obj in current.world.collision_objects}
            present.update(current.allowed_collision_matrix.entry_names)
            present.update(current.allowed_collision_matrix.default_entry_names)
            for object_id in managed-{self.id}:
                if object_id in present:
                    clear_bottle(node, object_id)

    def read(self):
        components = (PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                      | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        return self.node.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
            components=PlanningSceneComponents(components=components))).scene

    def apply(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        if not self.node.call(ApplyPlanningScene, '/apply_planning_scene',
                              ApplyPlanningScene.Request(scene=scene)).success:
            raise MoveItError('MoveIt 拒绝更新水瓶场景')

    def allow_touch(self, enabled, support=False):
        self.contacts(gripper=enabled, supports=[self.support_id] if enabled and support and self.support_id else [])

    def contacts(self, *, gripper=False, supports=()):
        """Set only this object's permitted contacts, preserving other objects."""
        matrix = self.read().allowed_collision_matrix
        remove_collision_entry(matrix, self.id)
        names = list(matrix.entry_names)
        links = set(self.geometry.touch_links) if gripper else set()
        links.update(supports)
        if links:
            for link in sorted(links):
                if link not in names:
                    for row in matrix.entry_values:
                        row.enabled.append(False)
                    names.append(link)
                    matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(names)))
            for row, name in zip(matrix.entry_values, names):
                row.enabled.append(name in links)
            matrix.entry_names = names+[self.id]
            matrix.entry_values.append(AllowedCollisionEntry(enabled=[name in links for name in names]+[False]))
        self.apply(PlanningScene(allowed_collision_matrix=matrix))

    def place(self, pose, diameter, height):
        self.bottle = CollisionObject(id=self.id, operation=CollisionObject.ADD, pose=pose)
        self.bottle.header.frame_id = self.node.args.frame
        # Body and a narrower neck make the 5 cm cylinder recognizable as a bottle.
        body = Pose(); body.orientation.w = 1.
        neck = Pose(); neck.orientation.w = 1.; neck.position.z = height/2.+.01
        self.bottle.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, diameter/2.]),
                                  SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.02, diameter*.3])]
        self.bottle.primitive_poses = [body, neck]
        self.world()
        self.changed = True

    def support_height(self, position, radius, object_id=None):
        """Resolve returned scene geometry into world before testing table tops.

        MoveIt folds objects into its planning frame (base_link here), even
        when they were originally published in world.
        """
        heights = []
        frames = {self.node.args.frame: np.eye(4)}
        for obj in self.read().world.collision_objects:
            if obj.id == self.id:
                continue
            # Only use the configured tabletop/desktop plate, not stand feet
            # or arbitrary obstacle tops, as a bottle support.
            if (obj.id != object_id if object_id is not None else
                    not any(word in obj.id for word in ('table', 'desktop', 'dark_plate'))):
                continue
            frame = obj.header.frame_id or 'base_link'
            if frame not in frames:
                frames[frame] = pose_matrix(self.node.fk({}, link=frame))
            origin = frames[frame] @ pose_matrix(obj.pose)
            for primitive, pose in zip(obj.primitives, obj.primitive_poses):
                if primitive.type != SolidPrimitive.BOX:
                    continue
                transform = origin @ pose_matrix(pose)
                if transform[2, 2] < .999:
                    continue
                center = transform[:3, 3]
                point = np.array([position[0], position[1], center[2]])
                local = transform[:3, :3].T @ (point-center)
                if all(abs(local[axis])+radius+.005 < primitive.dimensions[axis]/2. for axis in (0, 1)):
                    heights.append((float(center[2]+primitive.dimensions[2]/2.), obj.id))
        if not heights:
            self.support_id = None
            return None
        height, self.support_id = max(heights)
        return height

    def world(self, pose=None):
        if pose is not None:
            self.bottle.pose = deepcopy(pose)
        detached = AttachedCollisionObject(link_name=self.node.tip_link,
            object=CollisionObject(id=self.id, operation=CollisionObject.REMOVE))
        scene = PlanningScene()
        if any(obj.object.id == self.id for obj in self.read().robot_state.attached_collision_objects):
            scene.robot_state.attached_collision_objects = [detached]
        scene.world.collision_objects = [deepcopy(self.bottle)]
        scene.object_colors = [ObjectColor(id=self.id, color=ColorRGBA(r=.12, g=.45, b=.95, a=1.))]
        self.apply(scene)
        self.attached = False

    def attach(self, state):
        tip = self.node.fk(state)
        obj = deepcopy(self.bottle)
        obj.header.frame_id = self.node.tip_link
        self.tip_to_object = np.linalg.inv(pose_matrix(tip)) @ pose_matrix(obj.pose)
        obj.pose = matrix_pose(self.tip_to_object)
        attached = AttachedCollisionObject(link_name=self.node.tip_link, object=obj,
                                           touch_links=self.geometry.touch_links)
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        # MoveIt 2 removes an existing world object when attaching the same ID.
        # A second REMOVE in this diff would fail after that implicit removal.
        self.apply(scene)
        self.attached = True

    def hide_world(self):
        self.apply(PlanningScene(world=PlanningSceneWorld(collision_objects=[
            CollisionObject(id=self.id, operation=CollisionObject.REMOVE)])))

    def state(self, positions):
        state = self.read().robot_state
        values = {name: value for name, value in zip(state.joint_state.name, state.joint_state.position)
                  if name not in self.geometry.mimic_names}
        values.update(positions)
        state.joint_state.name = list(values)
        state.joint_state.position = list(values.values())
        state.joint_state.velocity = []
        state.joint_state.effort = []
        state.is_diff = False
        return state
