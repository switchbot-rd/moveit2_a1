"""记录 Servo 输入、输出轨迹和关节反馈，不发送运动命令。"""
from collections import Counter
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from control_msgs.msg import JointJog, JointTrajectoryControllerState
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


def json_values(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_values(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_values(item) for item in value]
    return value


class ServoTrace:
    def __init__(self, path, metadata):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open('x', buffering=65536)
        self.metadata = metadata
        self.counts, self.statuses = Counter(), Counter()
        self.last_time, self.max_gap, self.max_error = {}, {}, {}
        self._write('metadata', metadata, 0, time.monotonic_ns())

    def _write(self, topic, data, ros_time_ns, steady_time_ns):
        self.stream.write(json.dumps({'topic': topic, 'ros_time_ns': ros_time_ns,
            'steady_time_ns': steady_time_ns, 'data': json_values(data)},
            ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n')

    def record(self, topic, data, ros_time_ns, steady_time_ns):
        self._write(topic, data, ros_time_ns, steady_time_ns)
        if topic in self.last_time:
            gap = (steady_time_ns-self.last_time[topic])/1e6
            self.max_gap[topic] = max(self.max_gap.get(topic, 0.), gap)
        self.last_time[topic] = steady_time_ns
        self.counts[topic] += 1
        if topic == '/servo_node/status':
            self.statuses[str(data['code'])] += 1
        if topic == '/servo_arm_controller/controller_state':
            for name, error in zip(data['joint_names'], data['error']['positions']):
                if math.isfinite(error):
                    self.max_error[name] = max(self.max_error.get(name, 0.), abs(error))

    def close(self):
        self.stream.close()
        summary = {'metadata': self.metadata, 'streams': {
            name: {'count': count, 'max_gap_ms': self.max_gap.get(name, 0.)}
            for name, count in self.counts.items()}, 'status_counts': dict(self.statuses),
            'max_joint_position_error': self.max_error}
        with self.path.with_suffix('.summary.json').open('x') as output:
            output.write(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        return summary


class ServoDebug(Node):
    def __init__(self, output_file='', settings=None):
        super().__init__('servo_debug')
        name = output_file
        if not name:
            directory = Path(os.environ.get('ROS_LOG_DIR', '.ros/log'))/'servo_debug'
            name = directory/(datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}.jsonl')
        package = Path(get_package_share_directory('moveit_servo'))/'package.xml'
        metadata = dict(settings or {})
        metadata['native_servo_version'] = ET.parse(package).getroot().findtext('version')
        metadata['servo_extension_version'] = ET.parse(
            Path(get_package_share_directory('onero_a1_servo'))/'package.xml').getroot().findtext('version')
        metadata['native_servo_executable'] = str(Path(get_package_prefix('onero_a1_servo'))/'lib/onero_a1_servo/servo_node')
        metadata['ros_domain_id'] = os.environ.get('ROS_DOMAIN_ID', '0')
        metadata['telemetry_qos'] = 'best_effort'
        self.trace = ServoTrace(name, metadata)
        self.token = self.input_sub = self.gripper_sub = self.selection_sub = None
        self.telemetry_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        for kind, topic in ((TwistStamped, '/servo_node/delta_twist_cmds'),
                            (JointJog, '/servo_node/delta_joint_cmds'),
                            (ServoStatus, '/servo_node/status'),
                            (JointTrajectory, '/servo_arm_controller/joint_trajectory'),
                            (JointTrajectoryControllerState, '/servo_arm_controller/controller_state'),
                            (JointTrajectoryControllerState, '/arm_controller/controller_state'),
                            (JointState, '/robot_joint_commands'),
                            (JointState, '/robot_joint_states'), (JointState, '/joint_states'),
                            (String, '/onero_topic_bridge/control_trace'),
                            (String, '/onero_topic_bridge/gain_state')):
            self.create_subscription(kind, topic, lambda msg, topic=topic: self.record(topic, msg), self.telemetry_qos)
        if metadata.get('arm') == 'both':
            for side in ('left', 'right'):
                for kind, topic in ((JointState, f'/{side}/robot_joint_states'),
                                    (JointState, f'/{side}/robot_joint_commands'),
                                    (String, f'/{side}/onero_topic_bridge/gain_state'),
                                    (String, f'/{side}/onero_topic_bridge/control_trace'),
                                    (JointTrajectoryControllerState, f'/{side}_arm_controller/controller_state')):
                    self.create_subscription(kind, topic, lambda msg, topic=topic: self.record(topic, msg), self.telemetry_qos)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/servo_manager/state', self.session, transient)
        self.create_timer(1., self.trace.stream.flush)
        self.get_logger().info('Servo 调试记录：'+str(self.trace.path.resolve()))

    def record(self, topic, msg):
        data = json.loads(msg.data) if isinstance(msg, String) else message_to_ordereddict(msg)
        self.trace.record(topic, data, self.get_clock().now().nanoseconds,
                          time.monotonic_ns())

    def session(self, msg):
        data = json.loads(msg.data)
        self.trace.record('/servo_manager/state', data, self.get_clock().now().nanoseconds, time.monotonic_ns())
        token = data.get('session_id')
        if token == self.token:
            return
        if self.input_sub is not None:
            self.destroy_subscription(self.input_sub)
            self.input_sub = None
        if self.gripper_sub is not None:
            self.destroy_subscription(self.gripper_sub)
            self.gripper_sub = None
        if self.selection_sub is not None:
            self.destroy_subscription(self.selection_sub)
            self.selection_sub = None
        self.token = token
        if token:
            topic = '/servo_manager/input/'+token
            self.input_sub = self.create_subscription(TwistStamped, topic,
                lambda msg: self.record(topic, msg), self.telemetry_qos)
            if len(data.get('available_targets', [])) > 1:
                selection = topic+'/selection'
                self.selection_sub = self.create_subscription(String, selection,
                    lambda msg: self.record(selection, msg), self.telemetry_qos)
            if data.get('gripper_available'):
                gripper_topic = topic+'/gripper'
                self.gripper_sub = self.create_subscription(JointJog, gripper_topic,
                    lambda msg: self.record(gripper_topic, msg), self.telemetry_qos)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='', help='JSONL 路径，默认保存在 ROS 日志目录')
    parser.add_argument('--settings-json', default='{}', help=argparse.SUPPRESS)
    options, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = ServoDebug(options.output, json.loads(options.settings_json))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
