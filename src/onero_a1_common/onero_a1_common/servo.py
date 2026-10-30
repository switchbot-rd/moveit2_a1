"""Keyboard command state and a leased client for the Servo manager."""
import json
import math

import numpy as np
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .moveit_client import MoveItError


class KeyboardState:
    AXES = {'w': (0, 1.), 's': (0, -1.), 'a': (1, 1.), 'd': (1, -1.),
            'r': (2, 1.), 'f': (2, -1.), 'u': (3, 1.), 'o': (3, -1.),
            'i': (4, 1.), 'k': (4, -1.), 'j': (5, 1.), 'l': (5, -1.)}

    def __init__(self):
        self.pressed = set()
        self.paused = False
        self.focused = True

    def stop(self, *, pause=True):
        self.pressed.clear()
        self.paused = pause

    def velocity(self, linear, angular):
        result = np.zeros(6)
        if self.paused or not self.focused or not self.pressed & {'shift_l', 'shift_r'}:
            return result
        for key in self.pressed:
            if key in self.AXES:
                index, sign = self.AXES[key]
                result[index] += sign
        # Diagonal input must not increase the configured speed.
        for begin, speed in ((0, linear), (3, angular)):
            vector = result[begin:begin+3]
            vector *= speed/max(1., float(np.linalg.norm(vector)))
        return result

    def gripper_direction(self):
        if not self.paused and self.focused and self.pressed & {'shift_l','shift_r'}:
            return int('z' in self.pressed)-int('x' in self.pressed)
        return 0


class ServoClient:
    def __init__(self, node, frame='world'):
        self.node, self.frame = node, frame
        self.dimension = 6
        self.session = self.publisher = None
        self.gripper_publisher = None
        self.selection_publisher = None
        self.requested_target = None
        self.selection_stamp = 0
        self.state = {}
        self.state_after = 0
        self.clients = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = node.create_subscription(String, '/servo_manager/state', self.receive, qos)

    def receive(self, msg):
        try:
            state = json.loads(msg.data)
            if state.get('stamp_ns', 0) >= self.state_after:
                self.state = state
        except ValueError:
            pass

    def call(self, service, timeout=4.):
        if service not in self.clients:
            self.clients[service] = self.node.create_client(Trigger, service)
        client = self.clients[service]
        if not client.wait_for_service(timeout_sec=3.):
            raise MoveItError('Servo 服务未就绪；请在 moveit.launch.py 启动参数中添加 enable_servo:=true')
        return self.node.wait(client.call_async(Trigger.Request()), timeout)

    def check(self):
        result = self.call('/servo_manager/check')
        if not result.success:
            raise MoveItError(result.message)
        return result.message

    def start(self):
        result = self.call('/servo_manager/start', 12.)
        if not result.success:
            raise MoveItError(result.message)
        self.session = json.loads(result.message)
        self.state_after = self.node.get_clock().now().nanoseconds
        self.state = {**self.state, 'active': True, 'session_id': self.session['session_id'], 'error': ''}
        self.publisher = self.node.create_publisher(TwistStamped, self.session['command_topic'], 1)
        if self.session.get('gripper_topic'):
            self.gripper_publisher = self.node.create_publisher(JointJog, self.session['gripper_topic'], 1)
        if self.session.get('selection_topic'):
            self.selection_publisher = self.node.create_publisher(String, self.session['selection_topic'], 1)
        self.send(np.zeros(self.dimension))
        # Discover the per-session stop service while heartbeats are running.
        stop = self.session['stop_service']
        self.clients[stop] = self.node.create_client(Trigger, stop)
        if self.selection_publisher is not None and self.node.arm_side != self.session['control_target']:
            self.select_target(self.node.arm_side)

    @property
    def switching(self):
        return self.requested_target is not None and not (
            self.state.get('session_id') == self.session['session_id']
            and self.state.get('control_target') == self.requested_target
            and self.state.get('pending_target') is None
            and self.state.get('selection_stamp_ns', 0) >= self.selection_stamp)

    @property
    def control_target(self):
        return self.requested_target or self.state.get('control_target', self.node.arm_side)

    def select_target(self, target):
        if self.selection_publisher is None or target not in self.session['available_targets']:
            raise MoveItError('当前服务不支持切换到该手臂')
        self.send(np.zeros(self.dimension))
        self.selection_stamp = self.node.get_clock().now().nanoseconds
        self.requested_target = target
        self.selection_publisher.publish(String(data=json.dumps({
            'target': target, 'stamp_ns': self.selection_stamp})))

    def cycle_target(self):
        targets = self.session.get('available_targets', [])
        if len(targets) > 1 and not self.switching:
            self.select_target(targets[(targets.index(self.control_target)+1) % len(targets)])

    def send(self, values, gripper=0):
        if self.publisher is None:
            return
        if len(values) != self.dimension or not all(math.isfinite(v) for v in values) or gripper not in (-1,0,1):
            raise MoveItError(f'遥操作速度必须是 {self.dimension} 个有限数值')
        if self.switching:
            values, gripper = np.zeros(self.dimension), 0
        msg = TwistStamped()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, values[:3])
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, values[3:])
        msg.header.frame_id = self.frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self.publisher.publish(msg)
        if self.gripper_publisher is not None:
            names = self.session.get('gripper_joints', [self.node.gripper_joint])
            command = JointJog(joint_names=names, velocities=[float(gripper)]*len(names))
            command.header = msg.header
            self.gripper_publisher.publish(command)

    def close(self):
        if self.session is None:
            return
        if self.state.get('session_id') == self.session['session_id']:
            self.send(np.zeros(self.dimension))
            result = self.call(self.session['stop_service'], 12.)
            if not result.success:
                raise MoveItError(result.message)
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
        if self.gripper_publisher is not None:
            self.node.destroy_publisher(self.gripper_publisher)
            self.gripper_publisher = None
        if self.selection_publisher is not None:
            self.node.destroy_publisher(self.selection_publisher)
            self.selection_publisher = None
        self.requested_target = None
        self.session = None
