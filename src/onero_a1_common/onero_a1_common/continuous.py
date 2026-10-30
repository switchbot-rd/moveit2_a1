"""Local native MoveIt geometry and whole-motion timing; no moveit_py context."""
from rclpy.serialization import serialize_message, deserialize_message
from moveit_msgs.msg import RobotTrajectory, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

_MODELS = {}


def geometry(client, *, refresh=False):
    from onero_a1_kinematics._core import PlanningGeometry
    context = getattr(client, '_geometry_context', None)
    if context is None:
        descriptions = tuple(client.parameter('/move_group', name) for name in (
            'robot_description', 'robot_description_semantic'))
        if descriptions not in _MODELS:
            _MODELS[descriptions] = PlanningGeometry(*descriptions)
        client._geometry_context = context = _MODELS[descriptions]
        refresh = True
    if refresh:
        mask = (PlanningSceneComponents.ROBOT_STATE | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.TRANSFORMS
                | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.OCTOMAP
                | PlanningSceneComponents.LINK_PADDING_AND_SCALING)
        scene = client.call(GetPlanningScene, '/get_planning_scene', GetPlanningScene.Request(
            components=PlanningSceneComponents(components=mask))).scene
        context.set_scene(serialize_message(scene))
    return context


def retime(client, trajectories, start_state, *, tolerance=.005):
    from .moveit_client import MoveItError
    from .trajectory_tools import _scaled_limits
    limits = _scaled_limits(client, client.joints, MoveItError)
    velocity = {name: limit[0] for name, limit in zip(client.joints, limits)}
    acceleration = {name: limit[1] for name, limit in zip(client.joints, limits)}
    try:
        result = geometry(client).retime(client.group, serialize_message(start_state),
            [serialize_message(path) for path in trajectories], velocity, acceleration, tolerance)
        return deserialize_message(result, RobotTrajectory)
    except (ValueError, RuntimeError) as exc:
        raise MoveItError(f'MoveIt 连续轨迹生成失败：{exc}') from exc
