"""规划场景：添加会挡住原路径的球体，重新规划避让，再删除球体。"""
from copy import deepcopy
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from .support import MoveItError, finite, finish_arm_plan, positive, run_example


def configure(parser):
    parser.add_argument("--scene-delta", type=finite, default=.4, help="joint4 目标变化，rad")
    parser.add_argument("--obstacle-radius", type=positive, default=.01, help="球体半径，米")


def set_obstacle(node, pose=None):
    obj = CollisionObject(id=node.scene_id)
    obj.header.frame_id = node.args.frame
    obj.operation = CollisionObject.REMOVE if pose is None else CollisionObject.ADD
    if pose is not None:
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[node.args.obstacle_radius])]
        obj.primitive_poses = [pose]
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    scene.world.collision_objects = [obj]
    if pose is not None:
        # A timed-out request may already have added the object on the server.
        # Keep it eligible for final cleanup even if no response arrives.
        node.scene_added = True
    result = node.call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene))
    if not result.success:
        raise MoveItError("MoveIt 拒绝更新碰撞场景")
    node.scene_added = pose is not None


def example(node):
    start = node.start
    joint = node.joints[3]
    target = {**start, joint: start[joint]+node.args.scene_delta}
    goal = node.joint_constraints(target)
    baseline = node.plan(start, goal).joint_trajectory
    selected = False
    for fraction in (.5, .3, .7):
        if not baseline.points:
            break
        point = baseline.points[int((len(baseline.points)-1)*fraction)]
        middle = dict(zip(baseline.joint_names, point.positions))
        flange = node.fk(middle)
        q = flange.orientation
        tool_axis = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0., 0., 1.])
        # A gripper can cover the flange route even at an endpoint. Also try
        # the tool region; require the sphere to block only the middle state.
        for distance in (0., .10):
            obstacle = deepcopy(flange)
            for axis, offset in zip("xyz", tool_axis*distance):
                setattr(obstacle.position, axis, getattr(obstacle.position, axis)+float(offset))
            set_obstacle(node, obstacle)
            if node.validity(start).valid and node.validity(target).valid and not node.validity(middle).valid:
                selected = True
                break
        if selected:
            break
    if not selected:
        raise MoveItError("该起点无法布置有效中途障碍物；调整 --scene-delta 或 --obstacle-radius")
    finish_arm_plan(node, node.plan(start, goal))
    set_obstacle(node)


def main(args=None):
    return run_example("planning_scene", __doc__, configure, example, args)
