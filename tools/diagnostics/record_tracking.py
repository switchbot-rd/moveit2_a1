#!/usr/bin/env python3
"""只读记录一次 Plan & Execute；不发送运动目标、不修改参数、不连接 CAN。

在点击 Plan & Execute 之前启动，动作结束后再等 5 秒并按 Ctrl-C：
    source setup_env.sh
    python3 -m tools.diagnostics.record_tracking

原始目标来自 MoveGroup 的 GetResult 响应，绝不使用停止后的 controller_state
reference 代替。display_planned_path 仅保留作辅助；预览并不证明轨迹已执行。
记录中 null 表示非有限数值；所有样本包含接收时刻，消息原始时间戳也会保留。
"""

import argparse
from collections import Counter
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import DisplayTrajectory
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class TrackingRecorder(Node):
    def __init__(self, output, observe_only=False):
        super().__init__("onero_tracking_recorder")
        self.output = output
        self.file = output.open("x", encoding="utf-8")
        self.started_ns = self.get_clock().now().nanoseconds
        self.counts = Counter()
        self.statuses = {}
        self.result_requests = {}
        self.latest_raw = None
        self.last_plan = None
        self.subscriptions_kept = []
        self.parameter_clients = []
        self._write("session", {"started_ns": self.started_ns, "read_only": True})

        live_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        topics = [
            (JointTrajectoryControllerState, "/arm_controller/controller_state"),
            (JointState, "/robot_joint_commands"),
            (JointState, "/robot_joint_states"),
            (JointState, "/joint_states"),
            (DisplayTrajectory, "/display_planned_path"),
            (String, "/onero_topic_bridge/control_trace"),
            (String, "/onero_topic_bridge/payload_state"),
            (String, "/onero_topic_bridge/gain_state"),
            (FollowJointTrajectory.Impl.FeedbackMessage,
             "/arm_controller/follow_joint_trajectory/_action/feedback"),
            (Log, "/rosout"),
        ]
        for msg_type, topic in topics:
            self.subscriptions_kept.append(self.create_subscription(
                msg_type, topic, lambda msg, topic=topic: self._on_message(topic, msg), live_qos
            ))

        status_qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.actions = {}
        for action_type, name in (
            (MoveGroup, "/move_action"),
            (ExecuteTrajectory, "/execute_trajectory"),
            (FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"),
        ):
            result_type = action_type.Impl.GetResultService
            client = self.create_client(result_type, name + "/_action/get_result")
            self.actions[name] = (result_type, client)
            self.subscriptions_kept.append(self.create_subscription(
                GoalStatusArray, name + "/_action/status",
                lambda msg, name=name: self._on_status(name, msg), status_qos,
            ))

        self.parameter_names = {
            "/controller_manager": ["update_rate"],
            "/arm_controller": [
                "command_interfaces", "state_interfaces", "constraints.goal_time",
                "constraints.stopped_velocity_tolerance",
                *[f"constraints.joint{i}-a1_r.{kind}"
                  for i in range(1, 8) for kind in ("trajectory", "goal")],
            ],
            "/onero_topic_bridge": [
                "control_rate", "state_publish_rate", "kp", "kd", "simulate",
                "can_channel", "model", "gravity_compensation", "trace_control",
                "mounting", "with_gripper",
                'payload_mass', 'payload_cog', 'payload_transition_time',
                "velocity_feedforward", "velocity_feedforward_scale", "velocity_feedforward_timeout",
            ],
        }
        for name in self.parameter_names:
            self.parameter_clients.append((
                name, self.create_client(GetParameters, name + "/get_parameters")
            ))
        self.maintenance = self.create_timer(1.0, self._maintain)
        print(f"正在只读记录：{output}", flush=True)
        if observe_only:
            print("静止观察记录：保持现有目标，不需要发起运动。", flush=True)
        else:
            print("准备好后执行一次小范围、低速运动；结束后等 5 秒再按 Ctrl-C。", flush=True)

    def _write(self, source, data):
        self.counts[source] += 1
        entry = {"source": source, "received_ros_ns": self.get_clock().now().nanoseconds,
                 "received_monotonic_ns": time.monotonic_ns(), "data": data}
        self.file.write(json.dumps(json_safe(entry), ensure_ascii=False, allow_nan=False) + "\n")

    def _on_message(self, topic, msg):
        if topic == "/rosout" and msg.level < Log.WARN:
            return
        if topic.endswith(("/control_trace", "/payload_state", "/gain_state")):
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                data = {"invalid_trace_json": msg.data}
        else:
            data = message_to_ordereddict(msg)
        self._write(topic, data)
        if topic == "/robot_joint_states":
            self.latest_raw = (time.monotonic(), dict(zip(msg.name, msg.position)))

    def _on_status(self, action_name, msg):
        self._write(action_name + "/_action/status", message_to_ordereddict(msg))
        for entry in msg.status_list:
            goal_id = bytes(entry.goal_info.goal_id.uuid).hex()
            key = (action_name, goal_id)
            stamp = entry.goal_info.stamp
            started_before_recorder = stamp.sec * 1_000_000_000 + stamp.nanosec < self.started_ns
            # Transient-local status includes old completed goals. Ignore those,
            # but allow recording an in-flight goal that started before us.
            if (key not in self.statuses and started_before_recorder
                    and entry.status in (GoalStatus.STATUS_SUCCEEDED,
                                         GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)):
                continue
            if self.statuses.get(key) != entry.status:
                label = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED",
                         5: "CANCELED", 6: "ABORTED"}.get(entry.status, str(entry.status))
                print(f"{action_name}: {label}  goal={goal_id}", flush=True)
            self.statuses[key] = entry.status
            if key in self.result_requests:
                continue
            result_type, client = self.actions[action_name]
            if not client.service_is_ready():
                continue  # Retry on the next status update.
            request = result_type.Request()
            request.goal_id = entry.goal_info.goal_id
            future = client.call_async(request)  # GetResult only; never SendGoal.
            self.result_requests[key] = future
            future.add_done_callback(
                lambda future, key=key: self._on_result(key, future)
            )

    def _on_result(self, key, future):
        try:
            response = future.result()
            self._write(key[0] + "/result", {
                "goal_id": key[1], "response": message_to_ordereddict(response),
            })
            if key[0] == "/move_action":
                trajectory = response.result.planned_trajectory.joint_trajectory
                if trajectory.points:
                    self.last_plan = {
                        "goal_id": key[1], "action_status": response.status,
                        "moveit_error_code": response.result.error_code.val,
                        "planned_endpoint": dict(zip(
                            trajectory.joint_names, trajectory.points[-1].positions)),
                    }
                    print("已保存 MoveGroup 返回的原始规划终点。", flush=True)
            elif key[0] == "/execute_trajectory":
                print(f"MoveIt 执行结果：status={response.status}, "
                      f"code={response.result.error_code.val}", flush=True)
            else:
                print(f"控制器结果：{response.result.error_code} "
                      f"{response.result.error_string}", flush=True)
        except Exception as exc:
            self._write("result_read_error", {"action": key[0], "error": str(exc)})

    def _maintain(self):
        self.file.flush()
        pending = []
        for name, client in self.parameter_clients:
            if not client.service_is_ready():
                pending.append((name, client))
                continue
            request = GetParameters.Request(names=self.parameter_names[name])
            future = client.call_async(request)
            future.add_done_callback(lambda future, name=name: self._on_parameters(name, future))
        self.parameter_clients = pending

    def _on_parameters(self, name, future):
        try:
            self._write(name + "/parameters", {
                "names": self.parameter_names[name],
                "response": message_to_ordereddict(future.result()),
            })
        except Exception as exc:
            self._write("parameter_read_error", {"node": name, "error": str(exc)})

    def finish(self):
        summary = {"topic_counts": dict(self.counts), "last_move_group_plan": self.last_plan}
        if self.last_plan and self.latest_raw:
            received, positions = self.latest_raw
            age = time.monotonic() - received
            summary["latest_feedback_receipt_age_s"] = age
            summary["latest_feedback"] = positions
            # Receipt age alone does not establish CAN freshness; inspect control_trace misses.
            if age <= 1.0:
                errors = {name: target - positions[name]
                          for name, target in self.last_plan["planned_endpoint"].items()
                          if name in positions and math.isfinite(positions[name])}
                summary["planned_endpoint_minus_latest_feedback_rad"] = errors
                print("原始规划终点减去最新反馈（rad，需结合 CAN 丢帧记录判断反馈有效性）：")
                for name, error in errors.items():
                    print(f"  {name}: {error:+.6f} rad / {math.degrees(error):+.3f}°")
        if not self.last_plan:
            print("未取得 MoveGroup 原始轨迹；笛卡尔示例请结合 .logs/diagnostics 报告中的原始目标。"
                  "不能用 controller_state.reference 代替原始目标。")
        for topic in ("/robot_joint_commands", "/robot_joint_states",
                      "/arm_controller/controller_state", "/onero_topic_bridge/control_trace"):
            if not self.counts[topic]:
                print(f"缺少记录：{topic}")
        self._write("summary", summary)
        self.file.close()
        summary_path = self.output.with_suffix(".summary.json")
        summary_path.write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False,
                                           allow_nan=False) + "\n", encoding="utf-8")
        print(f"记录已保存：{self.output}\n摘要：{summary_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="JSONL 输出文件（必须不存在）")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="记录秒数；0 表示直到 Ctrl-C（默认）")
    parser.add_argument("--observe-only", action="store_true",
                        help="显示静止观察提示；仅影响提示文本，不会改变机械臂状态")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration 必须是有限非负数")
    output = args.output or (Path(__file__).resolve().parent.parent / ".logs" / "tracking"
                            / f"{datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}.jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        parser.error(f"输出文件已存在：{output}")
    rclpy.init()
    node = TrackingRecorder(output, observe_only=args.observe_only)
    started = time.monotonic()
    try:
        while rclpy.ok() and (not args.duration or time.monotonic() - started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
