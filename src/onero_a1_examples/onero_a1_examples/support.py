"""Small CLI and lifecycle helpers shared by the runnable examples."""
import argparse
from copy import deepcopy
from datetime import datetime
import math
import os
from pathlib import Path
import sys
import threading

import rclpy
from rclpy.signals import SignalHandlerOptions
from onero_a1_common.constants import JOINTS, OMPL_PLANNERS, PLANNING_PIPELINES
from onero_a1_common.moveit_client import MoveItClient, MoveItError


def mirror_console(case):
    """Copy everything written to stdout/stderr into one run log.

    Python prints and the C-level ROS console share the same file
    descriptors, so a duplicate of fd 1/2 catches both. Returns the log path
    and a closer, or None when the log directory cannot be created.
    """
    try:
        root = Path(os.environ.get('ROS_LOG_DIR')
                    or Path(os.environ.get('ROS_HOME', Path.home()/'.ros'))/'log')
        root.mkdir(parents=True, exist_ok=True)
        path = root/f'onero_a1_{case}_{datetime.now():%Y%m%d_%H%M%S}_{os.getpid()}.log'
        sink = open(path, 'ab', buffering=0)
        for stream in (sys.stdout, sys.stderr):
            stream.flush()
            try:
                stream.reconfigure(line_buffering=True)
            except (AttributeError, ValueError):
                pass
        saved = (os.dup(1), os.dup(2))
        read_fd, write_fd = os.pipe()
        os.dup2(write_fd, 1)
        os.dup2(write_fd, 2)
        os.close(write_fd)
    except OSError:
        return None

    def pump():
        while True:
            try:
                chunk = os.read(read_fd, 65536)
            except OSError:
                break
            if not chunk:
                break
            for write in (sink.write, lambda data: os.write(saved[0], data)):
                try:
                    write(chunk)
                except OSError:
                    pass

    worker = threading.Thread(target=pump, name='console-log', daemon=True)
    worker.start()

    def close():
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except (OSError, ValueError):
                pass
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        worker.join(timeout=2.)
        for fd in (read_fd, *saved):
            try:
                os.close(fd)
            except OSError:
                pass
        sink.close()
    return path, close



def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return value


def positive(value):
    value = finite(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return value


def scaling(value):
    value = positive(value)
    if value > 1:
        raise argparse.ArgumentTypeError("缩放比例必须在 (0, 1] 内")
    return value


def translation_arguments(parser):
    parser.add_argument("--dx", type=finite, default=0., help="X 方向位移，米")
    parser.add_argument("--dy", type=finite, default=0., help="Y 方向位移，米")
    parser.add_argument("--dz", type=finite, default=.01, help="Z 方向位移，米")


def translated_target(node):
    current = node.fk(node.start)
    target = deepcopy(current)
    for axis in "xyz":
        setattr(target.position, axis, getattr(target.position, axis)+getattr(node.args, "d"+axis))
    return current, target


def finish_arm_plan(node, trajectory):
    node.show_plan(trajectory)
    if node.args.execute:
        node.execute(trajectory)


def run_example(name, description, configure, scenario, args=None, *, query_only=False,
                planning_pipelines=PLANNING_PIPELINES, dual=False, allow_both=False, cleanup=None,
                default_planning_time=10., tracking=False):
    # Redirection to the example log is block buffered by default, which
    # scrambles planning progress and C++ output.
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=description)
    parser.set_defaults(pipeline="ompl", planner=None)
    if dual:
        parser.set_defaults(arm="both")
    else:
        parser.add_argument("--arm", choices=["left", "right"]+(["both"] if allow_both else []),
                            help="选择手臂；单臂自动识别，双臂默认 right")
    configure(parser)
    if tracking:
        parser.add_argument('--record-tracking', action='store_true', help='执行时记录关节误差并生成曲线')
        parser.add_argument('--tracking-output', type=Path, help='误差采集输出目录，必须尚不存在；指定后自动开启采集')
    parser.add_argument("--frame", choices=["world", "base_link"], default="world", help="参考系，默认 world")
    if query_only:
        parser.set_defaults(execute=False)
    else:
        parser.add_argument("--execute", action="store_true", help="执行轨迹；默认只规划")
        parser.add_argument("--velocity", type=scaling, default=.05, help="速度缩放，默认 0.05")
        parser.add_argument("--acceleration", type=scaling, default=.05, help="加速度缩放，默认 0.05")
        parser.add_argument("--planning-time", type=positive, default=default_planning_time)
        if planning_pipelines:
            parser.add_argument("--pipeline", choices=planning_pipelines, default="ompl",
                                help="规划管线，默认 ompl")
            parser.add_argument("--planner", choices=OMPL_PLANNERS,
                                help="OMPL 算法，默认 RRTConnect；仅用于 --pipeline ompl")
    supplied = list(args) if args is not None else sys.argv[1:]
    split = supplied.index("--ros-args") if "--ros-args" in supplied else len(supplied)
    options = parser.parse_args(supplied[:split])
    if options.pipeline != "ompl" and options.planner is not None:
        parser.error("--planner 仅用于 --pipeline ompl；CHOMP/STOMP 省略此参数")
    options.case = name
    mirror = mirror_console(name)
    try:
        if mirror is not None:
            print(f'本次运行日志：{mirror[0]}')
        if not query_only:
            print(f'本次运动参数：--velocity {options.velocity:g} --acceleration {options.acceleration:g}')
        rclpy.init(args=supplied[split:], signal_handler_options=SignalHandlerOptions.NO)
        node = None
        recorder = None
        cleanup_ready = False
        code = 0
        try:
            node = MoveItClient(options)
            node.select_arm(options.arm)
            cleanup_ready = True
            node.start = {n: q for n, q in node.state().items() if n in node.joints}
            if tracking and options.execute and (options.record_tracking or options.tracking_output):
                from .joint_tracking import JointTracking
                recorder = JointTracking(node, options.tracking_output)
                node.execution_observer = recorder
                recorder.wait_ready()
            scenario(node)
        except (Exception, KeyboardInterrupt) as exc:
            print(f"示例失败：{str(exc) or 'interrupted'}", file=sys.stderr)
            code = 1
        finally:
            if node is not None:
                try:
                    if not node.cleanup():
                        code = 1
                except (Exception, KeyboardInterrupt) as exc:
                    print(f'取消或清理请求失败：{str(exc) or "interrupted"}', file=sys.stderr)
                    code = 1
                finally:
                    if cleanup_ready and cleanup is not None:
                        try:
                            cleanup(node)
                        except (Exception, KeyboardInterrupt) as exc:
                            print(f'清理示例场景失败：{str(exc) or "interrupted"}', file=sys.stderr)
                            code = 1
                    try:
                        if recorder is not None:
                            recorder.close(code == 0)
                    except (Exception, KeyboardInterrupt) as exc:
                        print(f'误差记录导出失败：{str(exc) or "interrupted"}', file=sys.stderr)
                        code = 1
                    finally:
                        node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return code
    finally:
        if mirror is not None:
            mirror[1]()
