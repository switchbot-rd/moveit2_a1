"""返回起始位置：读取 record_startup 保存的位置，从当前状态规划返回。"""
from .support import finish_arm_plan, run_example


def example(node):
    # record_startup explicitly saves this target; it survives service restarts.
    snapshot = node.startup_snapshot()
    target = snapshot["joint_positions"]
    node.startup_id = snapshot["id"]
    trajectory = node.plan(node.start, node.joint_constraints(target))
    finish_arm_plan(node, trajectory)


def main(args=None):
    return run_example("return_startup", __doc__, lambda _: None, example, args, allow_both=True)
