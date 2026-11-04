"""零重力拖动：通过重力补偿手动引导机械臂，退出后保持当前位置。"""
from .support import finite, positive, run_example


def damping(value):
    value = finite(value)
    if not 0. <= value <= 5.:
        import argparse
        raise argparse.ArgumentTypeError('阻尼必须在 [0, 5] 内')
    return value


def configure(parser):
    parser.add_argument('--execute', action='store_true', help='进入拖动模式；省略时只检查条件')
    parser.add_argument('--damping', type=damping, default=.3, help='拖动阻尼 kd，默认 0.3')
    parser.add_argument('--duration', type=positive, help='拖动持续时间，秒；省略时按 Ctrl+C 退出')
    parser.add_argument('--keep-payload', action='store_true', help='带物拖动时保留当前负载；默认恢复启动基线')


def example(node):
    node.freedrive(damping=node.args.damping, duration=node.args.duration,
                   execute=node.args.execute, keep_payload=node.args.keep_payload)


def main(args=None):
    return run_example('freedrive', __doc__, configure, example, args,
                       query_only=True, allow_both=True)
