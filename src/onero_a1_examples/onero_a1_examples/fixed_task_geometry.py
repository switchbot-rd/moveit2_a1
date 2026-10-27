"""Bottle geometry: fixed approach direction, optional world-Z rotation."""
import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .bottle_scene import matrix_pose, pose_matrix
from .task_support import rounded_waypoints


def yaw_rotation(angle):
    return Rotation.from_rotvec([0., 0., angle]).as_matrix()


def tilt_error(rotation):
    return math.acos(float(np.clip(rotation[2, 2], -1., 1.)))


def grasp_poses(config, offset, yaw):
    rotation = yaw_rotation(yaw) @ Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
    grasp = np.eye(4)
    grasp[:3, :3] = rotation
    grasp[:3, 3] = np.array(config.pick_position)+[0., 0., config.grasp_height]-rotation @ offset
    pregrasp = grasp.copy()
    # Rotating the flange never moves the approach to the side of the bottle.
    pregrasp[:3, 3] -= np.array(config.approach_direction)*config.approach_distance
    return matrix_pose(grasp), matrix_pose(pregrasp)


def carrying_waypoints(start_object, config, tip_to_object, yaw):
    finish = pose_matrix(config.object_pose(True))
    initial_yaw = math.atan2(start_object[1, 0], start_object[0, 0])
    finish[:3, :3] = yaw_rotation(initial_yaw+yaw)
    up, across = start_object.copy(), finish.copy()
    up[2, 3] = across[2, 3] = config.transfer_height
    objects = rounded_waypoints(matrix_pose(start_object), [matrix_pose(up), matrix_pose(across), matrix_pose(finish)])
    delta = finish[:2, 3]-start_object[:2, 3]
    length2 = float(delta @ delta)
    turn = Slerp([0., 1.], Rotation.from_matrix([start_object[:3, :3], finish[:3, :3]]))
    inverse = np.linalg.inv(tip_to_object)
    result = []
    for pose in objects:
        transform = pose_matrix(pose)
        fraction = (float(np.clip((transform[:2, 3]-start_object[:2, 3]) @ delta/length2, 0., 1.))
                    if length2 > 1e-12 else 0.)
        transform[:3, :3] = turn(fraction).as_matrix()
        result.append(matrix_pose(transform @ inverse))
    return result
