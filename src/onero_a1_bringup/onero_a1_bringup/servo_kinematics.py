"""Position-priority teleoperation velocities for the official Servo JointJog input."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from onero_a1_kinematics import load_profile, project_nullspace


class ServoKinematics:
    def __init__(self, urdf, srdf, joints, max_velocity, max_acceleration,
                 margins=(.05,.05,.05,.2,.05,.05,.05), latency=.1, group='arm'):
        with tempfile.TemporaryDirectory(prefix='onero-servo-model-') as directory:
            u, s = Path(directory)/'robot.urdf', Path(directory)/'robot.srdf'
            u.write_text(urdf); s.write_text(srdf)
            self.model = RobotModel(str(u), str(s))
        self.state = RobotState(self.model)
        # Populate fixed mounting transforms before the dual-arm wrapper caches
        # its common-frame rotations; a new state's transform cache is identity.
        self.state.set_to_default_values()
        self.state.update()
        self.group = group
        chain = ET.fromstring(srdf).find(f"group[@name='{group}']/chain")
        self.base = chain.get('base_link')
        self.tip = chain.get('tip_link')
        limits = {j.get('name'): j.find('limit') for j in ET.fromstring(urdf).findall('joint')}
        self.lower = np.array([float(limits[j].get('lower')) for j in joints])
        self.upper = np.array([float(limits[j].get('upper')) for j in joints])
        self.max_velocity, self.max_acceleration = np.array(max_velocity), np.array(max_acceleration)
        self.margins = np.broadcast_to(np.asarray(margins), self.lower.shape)+.01
        self.latency = latency+.02
        self.reference = None
        self.posture = load_profile(list(joints))
        self.posture.set_bounds(self.lower,self.upper)
        self.posture_cost = 0.
        self.condition = self.orientation_deviation = 0.

    def pose(self, q):
        self.state.set_joint_group_positions(self.group, np.asarray(q))
        self.state.update()
        return np.linalg.inv(np.array(self.state.get_global_link_transform(self.base))) @ np.array(self.state.get_global_link_transform(self.tip))

    def jacobian(self, q):
        self.state.set_joint_group_positions(self.group, np.asarray(q))
        self.state.update()
        return np.array(self.state.get_jacobian(self.group, np.zeros(3)))

    def quality(self, q):
        jacobian = self.jacobian(q)
        # A characteristic length puts rotation and translation on comparable scales.
        jacobian[3:] *= .3
        singular = np.linalg.svd(jacobian, compute_uv=False)
        return float(np.sum(np.log(singular*singular+.02**2)))

    def reset(self):
        self.reference = None

    def solve(self, q, twist, *, posture_scale=1.):
        q, twist = np.asarray(q), np.asarray(twist).copy()
        if not np.any(twist):
            self.reset()
            return np.zeros(len(q))
        pose = self.pose(q)
        if self.reference is None:
            self.reference = pose[:3,:3].copy()
        jacobian = self.jacobian(q)
        self.condition = min(float(np.linalg.cond(jacobian)), 1e12)
        if self.condition > 30. and np.linalg.norm(twist[:3]) > 0.:
            # Let posture adjustment settle before accelerating out of a poorly
            # conditioned configuration. This only limits the singular region.
            twist[:3] *= min(1., .05/np.linalg.norm(twist[:3]))
        error = Rotation.from_matrix(self.reference @ pose[:3,:3].T).as_rotvec()
        self.orientation_deviation = float(np.linalg.norm(error))

        self.posture_cost = self.posture.cost(q)
        posture = self.posture.nullspace_velocity(q,jacobian,posture_scale)
        # Singularity escape may use orientation freedom while holding position.
        # Natural-posture correction itself stays in the full pose nullspace.
        if self.condition > 30. and posture_scale > 0.:
            step = 1e-4
            gradient = np.empty(len(q))
            for index in range(len(q)):
                offset = np.eye(len(q))[index]*step
                gradient[index] = (self.quality(q+offset)-self.quality(q-offset))/(2*step)
            escape = project_nullspace(jacobian[:3],gradient)
            posture += .3*escape/max(1.,float(np.linalg.norm(escape)))

        translating = np.linalg.norm(twist[:3]) > 0. and not np.any(twist[3:])
        angular = twist[3:].copy()
        if translating:
            angular = 2.*error
            # Release some orientation stiffness before entering the singular region.
            orientation_weight = .6*(np.clip((60.-self.condition)/40., .04, 1.)
                +(self.orientation_deviation/np.deg2rad(12.))**4)
        else:
            orientation_weight = .6
            self.reference = pose[:3,:3].copy()

        # Reserve the native trajectory's 100 ms lead time and braking distance.
        clearance_lower = np.maximum(q-self.lower-self.margins, 0.)
        clearance_upper = np.maximum(self.upper-q-self.margins, 0.)
        latency = self.latency
        def allowed(clearance):
            acceleration = self.max_acceleration
            return np.minimum(self.max_velocity,
                np.sqrt((acceleration*latency)**2+2.*acceleration*clearance)-acceleration*latency)
        lower, upper = -allowed(clearance_lower), allowed(clearance_upper)
        count = len(q)
        # A bound reached on one joint leaves the remaining joints available.
        matrix = np.vstack((30.*jacobian[:3], orientation_weight*jacobian[3:], .08*np.eye(count)))
        target = np.concatenate((30.*twist[:3], orientation_weight*angular, .08*posture))
        result = lsq_linear(matrix, target, bounds=(lower-1e-12, upper+1e-12), method='bvls', tol=1e-8)
        if not result.success or not np.all(np.isfinite(result.x)):
            raise RuntimeError('遥操作运动学求解失败')
        velocity = result.x
        actual_linear = jacobian[:3]@velocity
        if np.linalg.norm(twist[:3]) > 0.:
            progress = np.clip(actual_linear@twist[:3]/(twist[:3]@twist[:3]), 0., 1.)
            expected_velocity = progress*twist[:3]
        else:
            expected_velocity = np.zeros(3)
        # A differential solution is only locally valid. Validate its finite
        # step before Servo integrates it near the workspace boundary.
        scale = 1.
        for _ in range(12):
            next_pose = self.pose(q+.01*scale*velocity)
            displacement = next_pose[:3,3]-pose[:3,3]
            expected = .01*scale*expected_velocity
            angle = Rotation.from_matrix(self.reference@next_pose[:3,:3].T).magnitude()
            if (np.linalg.norm(displacement-expected) < 3e-5 and
                    (not translating or angle <= max(np.deg2rad(12.), self.orientation_deviation)+1e-6)):
                return scale*velocity
            scale *= .5
        return np.zeros(count)


class DualServoKinematics:
    """Independent end-effector commands, checked/executed as one 14-joint motion."""
    def __init__(self, urdf, srdf, joints, max_velocity, max_acceleration,
                 margins=(.05,.05,.05,.2,.05,.05,.05), latency=.1):
        self.parts, self.rotations = {}, {}
        self.control_target='right'
        for side,offset in (('left',0),('right',7)):
            part=ServoKinematics(urdf,srdf,joints[offset:offset+7],max_velocity[offset:offset+7],
                max_acceleration[offset:offset+7],margins,latency,group=side+'_arm')
            self.parts[side]=part
            self.rotations[side]=np.array(part.state.get_global_link_transform(part.base))[:3,:3]
        self.posture=load_profile(list(joints))
        self.posture.set_bounds(np.r_[self.parts['left'].lower,self.parts['right'].lower],
                                np.r_[self.parts['left'].upper,self.parts['right'].upper])
        self.condition=self.orientation_deviation=self.posture_cost=0.

    def reset(self):
        for part in self.parts.values():part.reset()

    def solve(self,q,twist,*,posture_scale=1.):
        result=np.zeros(14)
        for side,offset in (('left',0),('right',7)):
            part=self.parts[side]
            if self.control_target not in (side,'both'):
                part.reset();continue
            rotation=self.rotations[side].T
            local=np.r_[rotation@np.asarray(twist[:3]),rotation@np.asarray(twist[3:])]
            result[offset:offset+7]=part.solve(q[offset:offset+7],local,posture_scale=posture_scale)
        self.condition=max(part.condition for part in self.parts.values())
        self.orientation_deviation=max(part.orientation_deviation for part in self.parts.values())
        self.posture_cost=self.posture.cost(np.asarray(q))
        return result
