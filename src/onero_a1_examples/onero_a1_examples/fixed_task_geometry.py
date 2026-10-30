"""Bottle geometry: fixed approach direction, optional world-Z rotation."""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .bottle_scene import matrix_pose, pose_matrix


def yaw_rotation(angle):
    return Rotation.from_rotvec([0., 0., angle]).as_matrix()


def tilt_error(rotation):
    return math.acos(float(np.clip(rotation[2, 2], -1., 1.)))


def grasp_poses(config, offset, yaw, position_offset=(0., 0., 0.)):
    rotation = yaw_rotation(yaw) @ Rotation.from_euler('xyz', config.grasp_rpy).as_matrix()
    grasp = np.eye(4)
    grasp[:3, :3] = rotation
    grasp[:3, 3] = (np.array(config.pick_position)+[0., 0., config.grasp_height]
                   +np.asarray(position_offset)-rotation @ offset)
    pregrasp = grasp.copy()
    # Rotating the flange never moves the approach to the side of the bottle.
    pregrasp[:3, 3] -= np.array(config.approach_direction)*config.approach_distance
    return matrix_pose(grasp), matrix_pose(pregrasp)


def bottle_bottom(transform, height, diameter):
    """Lowest point of the bottle body/neck collision geometry in world Z."""
    cosine = float(np.clip(transform[2, 2], -1., 1.))
    sine = math.sqrt(max(0., 1.-cosine*cosine))
    return float(transform[2, 3]+min(-height/2.*abs(cosine)-diameter/2.*sine,
        (height/2.+.01)*cosine-.01*abs(cosine)-diameter*.3*sine))
