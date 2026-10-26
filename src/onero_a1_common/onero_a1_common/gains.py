"""Runtime kp/kd requests acknowledged by the bridge's actual control cycles."""
import json
import math
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String

from .moveit_client import MoveItError


class GainClient:
    def __init__(self, node, bridge):
        self.node, self.bridge, self.latest, self.session = node, bridge, None, None
        self.subscription = node.create_subscription(String, bridge+'/gain_state', self._receive, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        state = self.read()
        if state['transitioning']:
            raise MoveItError('手臂已有增益切换请求，请等待完成后重新运行')
        self.session, self.original_kp = state['session_id'], state['kp']
        self.original_kd = state['kd']
        self.changed = set()

    def _receive(self, message):
        try:
            state = json.loads(message.data)
            if all(key in state for key in ('kp', 'kd', 'target_kp', 'target_kd', 'revision', 'applied_revision',
                    'transitioning', 'transition_time', 'stamp_ns', 'session_id', 'error')):
                self.latest = state
        except (ValueError, TypeError):
            pass

    def read(self):
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest is not None:
                state = dict(self.latest)
                age = (self.node.get_clock().now().nanoseconds-state['stamp_ns'])/1e9
                if -.2 <= age <= 1.:
                    if self.session is not None and state['session_id'] != self.session:
                        raise MoveItError('增益切换期间手臂服务已重启')
                    if state['error']:
                        raise MoveItError('手臂控制异常：'+state['error'])
                    return state
            rclpy.spin_once(self.node, timeout_sec=.05)
        raise MoveItError('没有新鲜的增益状态，请重新构建并重启手臂服务')

    def set_kp(self, kp):
        self.set_gains(kp=kp)

    def set_kd(self, kd):
        self.set_gains(kd=kd)

    def set_gains(self, *, kp=None, kd=None):
        values = {name: value for name, value in (('kp', kp), ('kd', kd)) if value is not None}
        if not values:
            raise MoveItError('至少指定 kp 或 kd')
        for name, value in values.items():
            maximum = 500. if name == 'kp' else 5.
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0. <= value <= maximum):
                raise MoveItError(f'{name} 必须是 [0, {maximum:g}] 范围内的有限数值')
        previous = self.read()
        response = self.node.call(SetParametersAtomically, self.bridge+'/set_parameters_atomically',
            SetParametersAtomically.Request(parameters=[
                Parameter(name, value=float(value)).to_parameter_msg() for name, value in values.items()]))
        if not response.result.successful:
            raise MoveItError('增益设置被拒绝：'+response.result.reason)
        self.changed.update(values)
        deadline = time.monotonic()+previous['transition_time']+5.
        while time.monotonic() < deadline:
            state = self.read()
            matches = all(state['target_'+name] == value for name, value in values.items())
            if state['revision'] > previous['revision'] and not matches:
                raise MoveItError('增益目标已被其他请求更改，停止后续动作')
            if (matches and all(state[name] == value for name, value in values.items())
                    and not state['transitioning']
                    and state['applied_revision'] == state['revision']):
                return
            rclpy.spin_once(self.node, timeout_sec=.05)
        raise MoveItError('等待增益在控制循环中生效超时')

    def restore(self):
        if self.changed:
            self.set_gains(**{name: getattr(self, 'original_'+name) for name in self.changed})
