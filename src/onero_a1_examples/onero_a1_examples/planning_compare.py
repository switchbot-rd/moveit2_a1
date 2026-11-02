"""轨迹平滑与方案比较：同一起点和目标比较规划器，预览或执行选中的方案。"""
import time

from onero_a1_common.constants import LEFT_JOINTS, OMPL_PLANNERS, PLANNING_PIPELINES
from onero_a1_common.trajectory_tools import trajectory_metrics
from .support import JOINTS, MoveItError, finite, positive, run_example


def configure(parser):
    target = parser.add_mutually_exclusive_group()
    target.add_argument('--target', help='命名姿态，例如 ready/home；省略时使用相对关节目标')
    target.add_argument('--delta', type=finite, default=.1, help='相对关节角，rad，默认 0.1')
    parser.add_argument('--joint', choices=LEFT_JOINTS+JOINTS, help='默认所选手臂第 1 关节；不与 --target 同用')
    parser.add_argument('--pipelines', nargs='+', choices=PLANNING_PIPELINES, default=list(PLANNING_PIPELINES),
                        help='要比较的规划器，默认 ompl chomp stomp')
    parser.add_argument('--ompl-planner', choices=OMPL_PLANNERS, default='RRTConnect')
    parser.add_argument('--smoothing', choices=['ruckig', 'none'], default='ruckig', help='默认启用 Ruckig 平滑')
    parser.add_argument('--jerk', type=positive, default=10., help='加速度变化率上限，rad/s³，默认 10')
    parser.add_argument('--select', choices=['duration', 'length'], default='duration',
                        help='选预计执行时间最短或关节路程最短的方案，默认 duration')


def example(node):
    if node.args.target:
        if node.args.joint:
            raise MoveItError('--joint 不与 --target 同用')
        target = node.named_target(node.args.target)
    else:
        joint = node.args.joint or node.joints[0]
        if joint not in node.joints:
            raise MoveItError('--joint 不属于 --arm 选择的手臂')
        target = {**node.start, joint: node.start[joint]+node.args.delta}
    if all(abs(target[name]-node.start[name]) < 1e-9 for name in target):
        print('目标与当前姿态相同，无需比较或执行。')
        return
    if node.args.execute:
        node.check_arm_controllers()
    goal = node.joint_constraints(target)
    original = node.args.pipeline, node.args.planner
    rows, candidates = [], []
    try:
        for pipeline in dict.fromkeys(node.args.pipelines):
            node.args.pipeline = pipeline
            node.args.planner = node.args.ompl_planner if pipeline == 'ompl' else None
            label = pipeline+'/'+node.args.planner if node.args.planner else pipeline
            try:
                began = time.monotonic()
                path = node.plan(dict(node.start), goal)
                planning_time = time.monotonic()-began
                start = node.planned_start_state
                metrics = trajectory_metrics(path)
                rows.append((label, planning_time, 0., metrics))
                if node.args.smoothing == 'ruckig':
                    label += '+Ruckig'
                    began = time.monotonic()
                    path = node.smooth_trajectory(path, jerk=node.args.jerk)
                    smoothing_time = time.monotonic()-began
                    metrics = trajectory_metrics(path)
                    rows.append((label, planning_time, smoothing_time, metrics))
                candidates.append((metrics, label, path, start))
            except (MoveItError, ValueError) as exc:
                print(f'{label} 失败：{exc}')
                if node.active_goal is not None:
                    raise
    finally:
        node.args.pipeline, node.args.planner = original
    print('\n方案 | 规划耗时(s) | 平滑耗时(s) | 路径点数 | 关节路程(rad) | 预计执行(s)')
    for label, planning_time, smoothing_time, metrics in rows:
        print(f'{label} | {planning_time:.3f} | {smoothing_time:.3f} | {metrics["points"]} | '
              f'{metrics["length"]:.4f} | {metrics["duration"]:.3f}')
    if not candidates:
        raise MoveItError('没有可用的规划方案，请查看各规划器的失败原因')
    metric = node.args.select
    secondary = 'length' if metric == 'duration' else 'duration'
    _, label, path, start = min(candidates, key=lambda item: (item[0][metric], item[0][secondary]))
    print(f'选中方案：{label}（按{"预计执行时间" if metric == "duration" else "关节路程"}选择）')
    node.planned_start_state = start
    node.show_plan(path)
    if node.args.execute:
        node.execute(path)


def main(args=None):
    return run_example('planning_compare', __doc__, configure, example, args, planning_pipelines=())
