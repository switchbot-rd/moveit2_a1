"""MTC 双臂接力：右 A→B、左 B→C，可选 C→B→A 回程。"""
from pathlib import Path
from dataclasses import dataclass
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from .fixed_pick_place_config import TaskConfig
from .support import MoveItError, OMPL_PLANNERS, positive, run_example


@dataclass
class MtcTaskConfig(TaskConfig):
    max_wrist_roll_deviation: float = math.pi/4
    max_wrist_bend_deviation: float = math.radians(20.)
    max_arm_swivel_deviation: float = math.pi/3
    max_arm_pitch_deviation: float = math.radians(30.)
    grasp_position_offset: list = None
    right_b_place_offset: list = None
    left_c_place_offset: list = None
    grasp_limit: int = 24
    left_c_height_offset: float = 0.
    place_z_offset: float = 0.

    def validate(self, *, recording=False):
        super().validate(recording=recording)
        value = self.max_arm_swivel_deviation
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0. < value <= math.pi):
            raise MoveItError('max_arm_swivel_deviation 必须在 (0, π] rad 内')
        self.max_arm_swivel_deviation = float(value)
        value = self.max_arm_pitch_deviation
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0. < value <= math.pi/2):
            raise MoveItError('max_arm_pitch_deviation 必须在 (0, π/2] rad 内')
        self.max_arm_pitch_deviation = float(value)
        value = self.max_wrist_bend_deviation
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0. < value <= math.pi/2):
            raise MoveItError('max_wrist_bend_deviation 必须在 (0, π/2] rad 内')
        self.max_wrist_bend_deviation = float(value)
        for name in ('left_c_height_offset', 'place_z_offset'):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not -.05 <= value <= .05):
                raise MoveItError(f'{name} 必须在 [-0.05, 0.05] m 内')
            setattr(self, name, float(value))
        value = self.grasp_limit
        if (isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 180):
            raise MoveItError('grasp_limit 必须是 1..180 的整数')
        for name in ('grasp_position_offset', 'right_b_place_offset', 'left_c_place_offset'):
            value = getattr(self, name)
            if value is None:
                continue
            if (not isinstance(value, list) or len(value) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
                raise MoveItError(f'{name} 必须包含 3 个有限数值')
            setattr(self, name, list(map(float, value)))
        if self.c_position is not None:
            if (not isinstance(self.c_position, list) or len(self.c_position) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                    for v in self.c_position)):
                raise MoveItError('c_position 必须包含 3 个有限数值')
            self.c_position = list(map(float, self.c_position))
            if any(math.dist(self.c_position, point) < 1e-6
                   for point in (self.pick_position, self.place_position)):
                raise MoveItError('C 点必须与 A、B 点不同')


def positive_integer(value):
    import argparse
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('必须为正整数')
    return result


def configure(parser):
    parser.add_argument('--config', type=Path, help='任务 YAML；默认使用双臂 A→B→C 预设')
    parser.add_argument('--planner', choices=OMPL_PLANNERS, default='RRTConnect')
    parser.add_argument('--max-solutions', type=positive_integer, default=3, help='最多搜索的完整方案数，默认 3')
    parser.add_argument('--task-timeout', type=positive, default=60., help='每段搜索时限，秒，默认 60')
    parser.add_argument('--grasp-angle-step', type=positive, default=math.pi/2, help='抓取方向采样间隔，rad，默认 π/2')
    parser.add_argument('--mass', type=positive, default=.1, help='实物质量，kg，执行前按实物填写，默认 0.1')
    parser.add_argument('--smoothing', choices=['ruckig', 'none'], default='none',
                        help='执行前轨迹平滑；默认 none 保留规划时长，ruckig 开启额外平滑')
    parser.add_argument('--round-trip', action='store_true',
                        help='在 A→B→C 后继续 C→B→A；默认不回程')
    parser.add_argument('--reset-task', action='store_true',
                        help='实物已取下且夹爪全张开后，清理 MTC 模型并恢复负载；须加 --execute')


def bottle_object(config, object_id):
    obj = CollisionObject(id=object_id, operation=CollisionObject.ADD)
    obj.header.frame_id = 'world'
    obj.pose.orientation.w = 1.
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = config.pick_position
    obj.pose.position.z += config.grasp_height
    body, neck = Pose(), Pose()
    body.orientation.w = neck.orientation.w = 1.
    body.position.z = -config.grasp_height
    neck.position.z = config.bottle_height/2.+.01-config.grasp_height
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                     dimensions=[config.bottle_height, config.diameter/2.]),
                      SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[.02, config.diameter*.3])]
    obj.primitive_poses = [body, neck]
    return obj


def example(node):
    from .dual_task_support import arm_clients
    from .mtc_relay import MtcRelay, reset_relay
    if node.args.frame != 'world':
        raise MoveItError('MTC 双臂接力使用 --frame world')
    arms = arm_clients(node)
    if node.args.reset_task:
        reset_relay(node, arms)
        return
    if node.args.grasp_angle_step > 2*math.pi:
        raise MoveItError('--grasp-angle-step 不能超过 2π')
    path = node.args.config or Path(get_package_share_directory('onero_a1_examples'))/'config/mtc_pick_place.yaml'
    config = MtcTaskConfig.load(path)
    if node.args.execute:
        node.check_arm_controllers()
    relay = MtcRelay(node, arms, config)
    node.mtc_relay = relay
    relay.initialize()
    if node.args.execute:
        relay.execute()
    else:
        relay.preview()


def cleanup(node):
    from .dual_task_support import close_arm_clients
    relay = getattr(node, 'mtc_relay', None)
    try:
        # Execution goals belong to the arm clients, not the parent node.
        # Stop them before changing gains, payload or collision objects.
        for arm in getattr(node, 'task_nodes', []):
            if arm.active_goal is not None and not arm.cleanup():
                raise MoveItError(f'{arm.arm_side} 臂运动取消失败，保留任务场景和补偿')
        if relay is not None:
            relay.cleanup()
    finally:
        try:
            if relay is not None:
                relay.close_planner()
        finally:
            close_arm_clients(node)


def main(args=None):
    return run_example('mtc_pick_place', __doc__, configure, example, args,
                       planning_pipelines=(), dual=True, cleanup=cleanup, tracking=True)
