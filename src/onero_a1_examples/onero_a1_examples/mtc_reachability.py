"""Conservative full-pose reach bound for MTC's attached-object placement IK."""
import math
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .bottle_scene import pose_matrix


class FlangeReachBound:
    def __init__(self, urdf, base_link, tip_link):
        self.base_link, self.tip_link = base_link, tip_link
        self.radius, self.offset = math.inf, np.zeros(3)
        parents = {joint.find('child').get('link'): joint
                   for joint in ET.fromstring(urdf).findall('joint')}
        joint = parents.get(tip_link)
        if joint is None or joint.get('type') != 'revolute' or joint.find('axis') is None:
            return
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        axis = np.fromstring(joint.find('axis').get('xyz', ''), sep=' ')
        if (any(vector.shape != (3,) or not np.isfinite(vector).all() for vector in (xyz, rpy, axis))
                or np.linalg.norm(axis) < 1e-12):
            return
        axis /= np.linalg.norm(axis)
        translated = Rotation.from_euler('xyz', rpy).as_matrix().T @ xyz
        offset = axis*np.dot(axis, translated)
        # p_tip - R_tip*offset removes the last translation's component along
        # its rotation axis. Its perpendicular component retains this radius.
        radius = float(np.linalg.norm(translated-offset))
        link = joint.find('parent').get('link')
        while link != base_link:
            joint = parents.get(link)
            if joint is None or joint.get('type') not in ('revolute', 'fixed'):
                return
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
            if xyz.shape != (3,) or not np.isfinite(xyz).all():
                return
            radius += float(np.linalg.norm(xyz))
            link = joint.find('parent').get('link')
        self.radius, self.offset = radius, offset
        # Covers NaturalPostureIK's 0.5 mm / 0.005 rad acceptance thresholds,
        # plus numerical slack. This is a necessary condition, never an IK result.
        self.margin = max(.002, .0005+.005*float(np.linalg.norm(offset))+1e-6)

    def reachable(self, tip_in_base):
        if not math.isfinite(self.radius) or not np.isfinite(tip_in_base).all():
            return True
        center = tip_in_base[:3, 3]-tip_in_base[:3, :3] @ self.offset
        return float(np.linalg.norm(center)) <= self.radius+self.margin

    def object_cost(self, object_id, target):
        def cost(solution, comment=''):
            try:
                # This generator monitors the stage after attachment to tip_link;
                # its IK frame is the object's identity frame. Recompute for
                # every grasp scene rather than caching decisions by yaw.
                scene = solution.start.scene
                frame = target.header.frame_id or scene.planning_frame
                if not all(scene.knows_frame_transform(name)
                           for name in (frame, object_id, self.tip_link, self.base_link)):
                    return 0.
                goal = np.asarray(scene.get_frame_transform(frame)) @ pose_matrix(target.pose)
                actual_object = np.asarray(scene.get_frame_transform(object_id))
                actual_tip = np.asarray(scene.get_frame_transform(self.tip_link))
                base = np.asarray(scene.get_frame_transform(self.base_link))
                desired_tip = np.linalg.inv(base) @ goal @ np.linalg.inv(actual_object) @ actual_tip
                if not self.reachable(desired_tip):
                    solution.comment = '放置目标超过机械臂几何可达上界'
                    return math.inf
            except (AttributeError, KeyError, RuntimeError, ValueError, np.linalg.LinAlgError):
                # If the bound cannot be established, let the existing IK and
                # validity checks decide; never reject on missing information.
                pass
            return 0.
        return cost


def place_reach_bound(parameters, group, tip_link):
    prefix = f'robot_description_kinematics.{group}.'
    if (parameters.get(prefix+'kinematics_solver') != 'onero_a1_kinematics/NaturalPostureIK'
            or parameters.get(prefix+'position_only_ik', False)
            or parameters.get(prefix+'orientation_vs_position', 1.) != 1.):
        return None
    epsilon = float(parameters.get(prefix+'epsilon', 1e-5))
    if not math.isfinite(epsilon) or not 0. < epsilon <= .0005:
        return None
    semantic = ET.fromstring(parameters['robot_description_semantic'])
    chain = next((item.find('chain') for item in semantic.findall('group') if item.get('name') == group), None)
    if chain is None or chain.get('tip_link') != tip_link:
        return None
    bound = FlangeReachBound(parameters['robot_description'], chain.get('base_link'), tip_link)
    return bound if math.isfinite(bound.radius) else None
