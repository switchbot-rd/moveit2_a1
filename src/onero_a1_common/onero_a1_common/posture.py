"""Adapters for the shared compiled posture objective; never modify a trajectory."""
import numpy as np
from onero_a1_kinematics import load_profile


class ElbowPreference:
    """Soft spatial elbow preference derived from the configured reference pose."""
    def __init__(self, fk, joints, profile, dual, side):
        self.fk = fk
        self.reference = dict(zip(joints, profile.reference))
        self.arms = []
        for index, arm in enumerate(('left', 'right') if side == 'both' else (side,)):
            prefix = arm+'_' if dual else ''
            def position(link):
                p = fk(self.reference, link=prefix+link).position
                return np.array([p.x, p.y, p.z])
            base, shoulder, elbow = [position(link) for link in ('base_link','Link2','Link4')]
            outward = shoulder-base
            if dual:
                other = 'right' if arm == 'left' else 'left'
                p = fk(self.reference, link=other+'_base_link').position
                outward = base-np.array([p.x, p.y, p.z])
            length = float(np.linalg.norm(outward))
            if length < 1e-6:
                continue
            outward /= length
            self.arms.append((prefix+'Link4', base, outward, float((elbow-base)@outward),
                max(.1, float(np.linalg.norm(elbow-shoulder))), 4.*profile.weights[index*7+2]))

    def cost(self, positions):
        result = 0.
        for link, base, outward, preferred, scale, weight in self.arms:
            p = self.fk({**self.reference, **positions}, link=link).position
            distance = float((np.array([p.x,p.y,p.z])-base)@outward)
            result += weight*(max(0., preferred-distance)/scale)**2
        return result


def client_profile(client):
    profile=load_profile(list(client.joints))
    if profile.enabled:
        client.planning_start_state({})  # Reuse the validated URDF bound cache.
        bounds=[client._joint_position_bounds[name] for name in client.joints]
        profile.set_bounds(np.array([v[0] for v in bounds]),np.array([v[1] for v in bounds]))
    return profile


def trajectory_cost(profile, trajectory, joints, *, posture_cost=None):
    path=trajectory.joint_trajectory
    if set(path.joint_names)!=set(joints) or not path.points:
        raise ValueError('构型评分需要完整的手臂轨迹')
    indices=[path.joint_names.index(name) for name in joints]
    if any(len(point.positions)!=len(joints) for point in path.points):
        raise ValueError('构型评分收到不完整的关节路径点')
    positions = np.array([[point.positions[i] for i in indices] for point in path.points])
    result = profile.path_cost(positions)
    if posture_cost is not None:
        extra = [max(0., posture_cost(dict(zip(joints, q)))-profile.cost(q)) for q in positions]
        result += profile.path_weight*sum(np.linalg.norm(b-a)*.5*(ca+cb)
            for a, b, ca, cb in zip(positions, positions[1:], extra, extra[1:]))
    return float(result)


def trajectory_elbow_cost(trajectory, joints, elbow_cost):
    path = trajectory.joint_trajectory
    indices = [path.joint_names.index(name) for name in joints]
    selected = np.unique(np.linspace(0,len(path.points)-1,min(24,len(path.points)),dtype=int))
    return float(np.mean([elbow_cost(dict(zip(joints,[path.points[i].positions[j] for j in indices])))
                          for i in selected]))
