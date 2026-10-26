"""Atomic payload requests with control-loop acknowledgement through ROS."""
import json
import math
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import String

from .moveit_client import MoveItError


def same_payload(mass, cog, other_mass, other_cog):
    return (math.isclose(mass, other_mass, abs_tol=1e-8, rel_tol=0.) and len(cog) == len(other_cog) == 3
            and all(math.isclose(a, b, abs_tol=1e-8, rel_tol=0.) for a, b in zip(cog, other_cog)))


class PayloadClient:
    def __init__(self, node, bridge):
        self.node, self.bridge = node, bridge
        self.latest = None
        self.subscription = node.create_subscription(String, bridge+'/payload_state', self._receive, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        state = self.read()
        self.session = state['session_id']
        self.baseline_mass = state['baseline_mass']
        self.baseline_cog = state['baseline_cog']

    def _receive(self, message):
        try:
            value = json.loads(message.data)
            if all(key in value for key in ('stamp_ns', 'session_id', 'baseline_mass', 'baseline_cog',
                                            'mass', 'cog', 'target_mass', 'target_cog', 'revision',
                                            'applied_revision', 'transitioning', 'error', 'limited',
                                            'transition_time', 'simulate', 'gravity_compensation')):
                self.latest = value
        except (ValueError, TypeError):
            pass

    def read(self, timeout=5.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest is not None:
                age = (self.node.get_clock().now().nanoseconds-self.latest['stamp_ns'])/1e9
                if -.2 <= age <= 1.:
                    return dict(self.latest)
            rclpy.spin_once(self.node, timeout_sec=.05)
        raise MoveItError('没有新鲜的负载状态，请更新并重启手臂服务')

    def checked_state(self):
        state = self.read()
        if state['session_id'] != self.session:
            raise MoveItError('负载切换期间手臂服务已重启')
        if state['error']:
            raise MoveItError('负载补偿异常：'+state['error'])
        return state

    def ensure_unloaded(self):
        state = self.checked_state()
        if not state['simulate'] and not state['gravity_compensation']:
            raise MoveItError('真机负载补偿需要 gravity_compensation:=true')
        if (state['transitioning'] or not same_payload(state['mass'], state['cog'],
                                                      self.baseline_mass, self.baseline_cog)):
            raise MoveItError('当前仍有附加载荷或正在切换；确认已放下物体后调用 '
                              +self.bridge+'/reset_payload 恢复基线')

    def set_payload(self, mass, cog):
        if (not math.isfinite(mass) or mass < 0 or len(cog) != 3
                or not all(math.isfinite(v) for v in cog)):
            raise MoveItError('负载质量或重心无效')
        previous = self.checked_state()
        parameters = [Parameter('payload_mass', value=float(mass)),
                      Parameter('payload_cog', value=list(map(float, cog)))]
        response = self.node.call(SetParametersAtomically, self.bridge+'/set_parameters_atomically',
            SetParametersAtomically.Request(parameters=[parameter.to_parameter_msg() for parameter in parameters]))
        if not response.result.successful:
            raise MoveItError('负载设置被拒绝：'+response.result.reason)
        deadline = time.monotonic()+previous['transition_time']+5.
        while time.monotonic() < deadline:
            state = self.checked_state()
            target_matches = same_payload(state['target_mass'], state['target_cog'], mass, cog)
            if state['revision'] > previous['revision'] and not target_matches:
                raise MoveItError('负载目标被其他请求更改，停止后续动作')
            if (target_matches and not state['transitioning']
                    and state['applied_revision'] == state['revision']
                    and same_payload(state['mass'], state['cog'], mass, cog)):
                if state['limited']:
                    self.node.get_logger().warning('负载补偿已生效，力矩前馈触发限幅；'
                                                   '将按现有 SDK 与适配层上限继续执行')
                return state
            rclpy.spin_once(self.node, timeout_sec=.05)
        raise MoveItError('等待负载补偿生效超时')

    def load_object(self, mass, cog):
        total = self.baseline_mass+mass
        combined = [(self.baseline_mass*a+mass*b)/total for a, b in zip(self.baseline_cog, cog)] if total else [0.]*3
        return self.set_payload(total, combined)

    def restore_baseline(self):
        return self.set_payload(self.baseline_mass, self.baseline_cog)
