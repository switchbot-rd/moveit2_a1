"""MoveIt GripperCommand interface using the SDK's shared-bus OneroGripper.

Position is the master finger joint angle: 0 rad closed, 1.1 rad open.
Feedback is measured opening, never the requested target. No force calibration
is available, so only max_effort=0 (SDK's existing position-control settings) is
accepted. This node never opens a CAN transport itself.
"""
import math
import json
import threading
import time
from collections import deque
from pathlib import Path
import statistics

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from .gripper_coordinates import GripperCalibration, load_calibration, save_calibration

JOINT = 'woan_picker_left_finger_joint'
STROKE = 1.1
OPENING_SPEED = .5  # normalized opening/s; matches the imported URDF rate cap
# MoveIt's joint goal region can place a planned endpoint within 0.005 rad
# of full open/close. Preserve the calibrated endpoint command in that band.
ENDPOINT_TOLERANCE = .005


def _target_opening(position_rad):
    """Convert a model goal to opening, including its endpoint goal region."""
    if not math.isfinite(position_rad) or not 0 <= position_rad <= STROKE:
        raise ValueError(f'夹爪目标必须在 0..{STROKE} rad 内')
    if position_rad <= ENDPOINT_TOLERANCE:
        return 0.
    if STROKE-position_rad <= ENDPOINT_TOLERANCE:
        return 1.
    return position_rad / STROKE


def _feedback_after_command(device, previous_stamp):
    """Recover one missing command reply with a passive query, never a resend."""
    state = device.state
    if state.timestamp <= previous_stamp:
        state = device.refresh()
    if state.timestamp <= previous_stamp:
        raise RuntimeError('夹爪目标发送后未取得新反馈')
    return state


class WoanGripperController(Node):
    def __init__(self, *, device=None, simulate=True, allow_execution=True,
                 activate_on_start=True, embedded=False,
                 closed_sdk_opening=0., open_sdk_opening=1.,
                 calibration_file='', arm_model='A1R', can_channel='simulation',
                 controller_name='gripper_controller', joint_name=JOINT):
        super().__init__(controller_name, use_global_arguments=not embedded)
        self._joint_name = joint_name
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('simulate', simulate, descriptor=readonly)
        self.declare_parameter('allow_execution', allow_execution, descriptor=readonly)
        self.declare_parameter('activate_on_start', activate_on_start, descriptor=readonly)
        self.declare_parameter('allow_stalling', False, descriptor=readonly)
        self.declare_parameter('closed_sdk_opening', closed_sdk_opening)
        self.declare_parameter('open_sdk_opening', open_sdk_opening)
        self.declare_parameter('calibration_file', calibration_file, descriptor=readonly)
        self.declare_parameter('arm_model', arm_model, descriptor=readonly)
        self.declare_parameter('can_channel', can_channel, descriptor=readonly)
        self._calibration = GripperCalibration(
            float(self.get_parameter('closed_sdk_opening').value),
            float(self.get_parameter('open_sdk_opening').value))
        self._simulate = bool(self.get_parameter('simulate').value)
        self._identity = {'model': str(self.get_parameter('arm_model').value),
                          'channel': str(self.get_parameter('can_channel').value),
                          'simulate': self._simulate}
        filename = str(self.get_parameter('calibration_file').value)
        self._calibration_file = Path(filename).expanduser() if filename and filename != 'none' else None
        if self._calibration_file is not None:
            loaded = load_calibration(self._calibration_file, **self._identity)
            if loaded is not None:
                self._calibration = loaded
                self.set_parameters_atomically([Parameter('closed_sdk_opening', value=float(loaded.closed)),
                                               Parameter('open_sdk_opening', value=float(loaded.opened))])
                self.get_logger().info(f'已加载夹爪标定：{self._calibration_file}')
                if json.loads(self._calibration_file.read_text()).get('schema_version') == 1:
                    self.get_logger().warning('旧版夹爪标定未区分指令与反馈端点，请空载重新执行 calibrate')
        self._permitted = bool(self.get_parameter('allow_execution').value)
        self._active = self._permitted and bool(self.get_parameter('activate_on_start').value)
        self._allow_stalling = bool(self.get_parameter('allow_stalling').value)
        if not self._simulate and device is None:
            raise ValueError('Real Woan Picker requires the Arm-owned SDK transport')
        self._device = device
        self._lock = threading.RLock()
        self._stopping = False
        self._reserved = False
        self._calibrating = False
        self._stop_generation = 0
        self._parameter_update_owner = None
        self.add_on_set_parameters_callback(self._guard_calibration_parameters)
        self._position = None
        self._velocity = 0.
        self._last_feedback = 0.
        self._previous_sdk_stamp = 0.
        self._target = None
        self._sent = None
        self._sdk_opening = None
        self._sdk_raw_position = None
        self._native_target = self._native_sent = None
        self._commanding = False
        self._error = '等待首次夹爪反馈'
        self._failures = 0
        self._last_error_log = 0.
        self._last_step = time.monotonic()
        self._publisher = self.create_publisher(JointState, '/joint_states', 10)
        self._diagnostics = self.create_publisher(String, '~/state', 10)
        self._activate_service = self.create_service(Trigger, '~/activate', self._activate)
        self._calibration_service = self.create_service(Trigger, '~/calibrate', self._calibrate,
                                                       callback_group=ReentrantCallbackGroup())
        self._stop_service = self.create_service(Trigger, '~/stop', self._stop_motion,
                                                callback_group=ReentrantCallbackGroup())
        self._server = ActionServer(self, GripperCommand, '~/gripper_cmd',
                                    execute_callback=self._execute,
                                    goal_callback=self._goal,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=ReentrantCallbackGroup())
        self._timer = self.create_timer(.05, self._update)
        self.get_logger().info('Woan Picker | 0=闭合、1.1 rad=全开 | '
                               f'simulate={self._simulate} | 可执行={self._active} | '
                               f'SDK端点={self._calibration.closed:.7f}..{self._calibration.opened:.7f}；启动只读取开度')

    def _guard_calibration_parameters(self, parameters):
        protected = any(p.name in ('closed_sdk_opening', 'open_sdk_opening') for p in parameters)
        allowed = not protected or self._parameter_update_owner == threading.get_ident()
        return SetParametersResult(successful=allowed,
                                   reason='' if allowed else '端点由自动标定服务更新；手工值需在启动时指定')

    def _install_calibration(self, calibration):
        """Caller holds the controller lock, preserving the physical holding target."""
        self._parameter_update_owner = threading.get_ident()
        try:
            result = self.set_parameters_atomically([
                Parameter('closed_sdk_opening', value=float(calibration.closed)),
                Parameter('open_sdk_opening', value=float(calibration.opened))])
            if not result.successful:
                raise RuntimeError(result.reason)
        finally:
            self._parameter_update_owner = None
        self._calibration = calibration
        if self._sdk_opening is not None:
            self._position = calibration.from_sdk(self._sdk_opening)
            self._target = self._position
            self._sent = calibration.command_from_sdk(self._sdk_opening)
            self._velocity = 0.

    def _stop_motion(self, _request, response):
        with self._lock:
            self._stop_generation += 1
            self._hold_measured()
        response.success = True
        response.message = '已请求停止夹爪动作/标定，保持反馈开度'
        return response

    def _probe_endpoint(self, target, generation, failures):
        """Drive the public SDK range, stop on fresh stable endpoint feedback."""
        with self._lock:
            self._native_target = target
            self._native_sent = self._sdk_opening
            self._commanding = True
        started = time.monotonic()
        samples = deque(maxlen=32)
        last_stamp = None
        endpoint_reached_at = None
        while rclpy.ok():
            with self._lock:
                if (self._stopping or not self._active or self._stop_generation != generation):
                    raise RuntimeError('夹爪标定已停止')
                if not self._fresh() or self._failures != failures:
                    raise RuntimeError(self._error or '标定期间反馈异常')
                stamp, value, raw, sent = (self._last_feedback, self._sdk_opening,
                                           self._sdk_raw_position, self._native_sent)
            if stamp != last_stamp:
                samples.append((stamp, value, raw))
                last_stamp = stamp
            # A plateau while ramping can be static friction, not an endpoint.
            # First finish sending the public SDK endpoint, then collect a
            # complete stable window under that same command.
            if abs(sent-target) <= 1e-9 and endpoint_reached_at is None:
                endpoint_reached_at = stamp
            recent = [s for s in samples if endpoint_reached_at is not None
                      and s[0] >= endpoint_reached_at and stamp-s[0] <= .40]
            if len(recent) >= 6 and recent[-1][0]-recent[0][0] >= .30:
                spread = max(s[1] for s in recent)-min(s[1] for s in recent)
                raw_values = [s[2] for s in recent if s[2] is not None]
                raw_stable = not raw_values or max(raw_values)-min(raw_values) <= .05
                if spread <= .0015 and raw_stable:
                    measured = statistics.median(s[1] for s in recent)
                    if (target == 1. and measured < .8) or (target == 0. and measured > .2):
                        raise RuntimeError('在行程中段提前停止，请检查阻挡或设备端点；未保存标定')
                    with self._lock:
                        self._hold_measured()
                        hold_after_feedback = self._last_feedback
                    return {'sdk_opening': measured,
                            'command_sdk_opening': target,
                            'sdk_raw_position': statistics.median(raw_values) if raw_values else None,
                            'spread': spread, 'samples': len(recent), 'last_feedback': hold_after_feedback}
            if time.monotonic()-started > 7.:
                raise RuntimeError('标定端点在 7 秒内未稳定；未保存标定')
            time.sleep(.02)
        raise RuntimeError('节点已退出，标定停止')

    def _calibrate(self, _request, response):
        with self._lock:
            if not self._active or self._reserved or not self._fresh() or self._stopping:
                response.success = False
                response.message = '标定未开始：检查执行开关、反馈和动作占用'
                return response
            self._reserved = self._calibrating = True
            old = self._calibration
            generation, failures = self._stop_generation, self._failures
        try:
            self.get_logger().info('开始 Woan Picker 空载自动标定：全开 → 全闭 → 保存')
            opened = self._probe_endpoint(1., generation, failures)
            closed = self._probe_endpoint(0., generation, failures)
            calibration = GripperCalibration(closed['sdk_opening'], opened['sdk_opening'],
                                             closed['command_sdk_opening'], opened['command_sdk_opening'])
            if calibration.opened-calibration.closed < .5:
                raise RuntimeError('测得行程不足 SDK 范围的一半；检查夹物/阻挡，未保存标定')
            # Let the native measured-position hold reach the device before disk I/O.
            deadline = time.monotonic()+.2
            while time.monotonic()<deadline:
                with self._lock:
                    if self._last_feedback > closed['last_feedback']:
                        break
                time.sleep(.01)
            with self._lock:
                if (not self._fresh() or self._failures != failures
                        or self._stop_generation != generation or self._stopping):
                    raise RuntimeError('标定提交前状态已变化；未保存标定')
                self._install_calibration(calibration)
                if self._calibration_file is not None:
                    save_calibration(self._calibration_file, calibration, **self._identity,
                                     endpoints={'open': opened, 'closed': closed})
                self._native_target = self._sdk_opening
                self._native_sent = self._sdk_opening
            response.success = True
            response.message = json.dumps({'closed_sdk_opening': calibration.closed,
                                           'open_sdk_opening': calibration.opened,
                                           'command_closed_sdk_opening': calibration.command_closed,
                                           'command_open_sdk_opening': calibration.command_opened,
                                           'file': str(self._calibration_file) if self._calibration_file else None,
                                           'simulate': self._simulate}, ensure_ascii=False)
            self.get_logger().info(f'夹爪自动标定完成：{response.message}')
        except Exception as exc:
            with self._lock:
                self._install_calibration(old)
                self._hold_measured()
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'夹爪标定失败，保留原标定：{exc}')
        finally:
            with self._lock:
                self._reserved = self._calibrating = False
        return response

    def _activate(self, _request, response):
        with self._lock:
            self._active = self._permitted
            response.success = self._active
        response.message = '夹爪动作接口已激活；尚未发送开合目标' if response.success else 'allow_execution=false，夹爪动作保持关闭'
        return response

    def _fresh(self):
        return (self._position is not None and self._error is None
                and time.monotonic() - self._last_feedback < .2)

    def _goal(self, request):
        command = request.command
        with self._lock:
            reasons = []
            if not self._active or self._stopping:
                reasons.append('动作接口未激活或正在停止')
            if self._reserved:
                reasons.append('已有动作或标定占用')
            if not self._fresh():
                reasons.append(f'反馈不可用：{self._error or "反馈过期"}')
            if not math.isfinite(command.position) or not 0 <= command.position <= STROKE:
                reasons.append(f'目标 {command.position} 超出 0..{STROKE} rad')
            if not math.isfinite(command.max_effort) or command.max_effort != 0.:
                reasons.append('max_effort 当前只支持 0')
            if not reasons:
                self._reserved = True
        if reasons:
            self.get_logger().warning('拒绝夹爪目标：' + '；'.join(reasons))
        return GoalResponse.REJECT if reasons else GoalResponse.ACCEPT

    def _hold_measured(self):
        with self._lock:
            if self._position is not None:
                self._target = self._position
                self._sent = self._calibration.command_from_sdk(self._sdk_opening)
                if self._sdk_opening is not None:
                    self._native_target = self._native_sent = self._sdk_opening

    def _execute(self, goal):
        result = GripperCommand.Result()
        reached = False
        try:
            with self._lock:
                target = _target_opening(goal.request.command.position)
                self._target = self._calibration.command_from_sdk(
                    self._calibration.target_to_sdk(target))
                self._sent = self._calibration.command_from_sdk(self._sdk_opening)
                self._commanding = True
                self._native_target = self._native_sent = None
                mark = self._position
                failures = self._failures
                generation = self._stop_generation
            started = last_motion = time.monotonic()
            while rclpy.ok() and not self._stopping:
                with self._lock:
                    position, velocity = self._position, self._velocity
                    ready = (self._active and self._fresh() and self._failures == failures
                             and self._stop_generation == generation)
                    error = self._error
                    sent_command = (self._native_sent if self._native_target is not None
                                    else self._calibration.to_sdk(self._sent)
                                    if self._sent is not None else None)
                    command_complete = (self._sent is not None
                                        and abs(self._sent-self._target) <= 1e-9)
                    if self._stop_generation != generation:
                        error = '收到夹爪停止请求'
                result.position = position * STROKE
                feedback = GripperCommand.Feedback(position=result.position, effort=0.)
                goal.publish_feedback(feedback)
                if goal.is_cancel_requested:
                    self._hold_measured()
                    goal.canceled()
                    return result
                if not ready:
                    self.get_logger().error(f'夹爪动作中止：{error or "反馈过期或执行开关关闭"}')
                    goal.abort()
                    return result
                if command_complete and abs(position - target) * STROKE <= .01:
                    reached = result.reached_goal = True
                    goal.succeed()
                    return result
                now = time.monotonic()
                if abs(position - mark) > .002:
                    mark, last_motion = position, now
                if now - last_motion > 1.:
                    result.stalled = True
                    self._hold_measured()
                    if self._allow_stalling:
                        goal.succeed()
                    else:
                        self.get_logger().error(
                            f'夹爪停止运动但尚未到位；标定后目标开度={target:.6f}，'
                            f'反馈开度={position:.6f}（0=全闭，1=全开），SDK反馈={self._sdk_opening:.7f}，'
                            f'SDK指令={sent_command:.7f}；allow_stalling=false')
                        goal.abort()
                    return result
                if now - started > 8.:
                    self.get_logger().error('夹爪动作超过 8 秒，停止推进并保持反馈开度')
                    goal.abort()
                    return result
                time.sleep(.02)
            if goal.is_active:
                goal.abort()
            return result
        finally:
            if not reached:
                self._hold_measured()
            with self._lock:
                self._reserved = False

    def _update(self):
        with self._lock:
            self._update_locked()

    def _update_locked(self):
        now = time.monotonic()
        dt = min(.1, max(0., now - self._last_step))
        self._last_step = now
        with self._lock:
            if self._stopping:
                return
            target, sent, commanding = self._target, self._sent, self._commanding
            previous = self._position
        try:
            command_speed = OPENING_SPEED * min(
                1., (self._calibration.opened-self._calibration.closed)
                / (self._calibration.command_opened-self._calibration.command_closed))
            if self._simulate:
                position = 0. if previous is None else previous
                if commanding and self._native_target is not None:
                    native = self._sdk_opening if self._sdk_opening is not None else self._calibration.to_sdk(position)
                    speed = min(.4, OPENING_SPEED*(self._calibration.opened-self._calibration.closed))
                    native += max(-speed*dt, min(speed*dt, self._native_target-native))
                    self._native_sent = native
                    position = self._calibration.from_sdk(native)
                elif commanding:
                    sent += max(-command_speed*dt, min(command_speed*dt, target-sent))
                    native = self._calibration.to_sdk(sent)
                    position = self._calibration.from_sdk(native)
                else:
                    native = self._sdk_opening if self._sdk_opening is not None else self._calibration.to_sdk(position)
                if self._native_target is not None or not commanding:
                    sent = self._calibration.command_from_sdk(native)
                sdk_stamp = now
            else:
                if commanding:
                    if not self._device.enabled:
                        self._device.enable()
                        state = self._device.state
                        if state.fault_code or state.motion_state != 1:
                            raise RuntimeError('夹爪使能反馈异常')
                        sent = self._calibration.command_from_sdk(state.position)
                    sent = self._calibration.command_from_sdk(self._device.state.position) if sent is None else sent
                    before = self._device.state.timestamp
                    if self._native_target is not None:
                        native = self._native_sent if self._native_sent is not None else self._device.state.position
                        speed = min(.4, OPENING_SPEED*(self._calibration.opened-self._calibration.closed))
                        native += max(-speed*dt, min(speed*dt, self._native_target-native))
                        self._native_sent = native
                        self._device.set_position(native)
                    else:
                        sent += max(-command_speed*dt, min(command_speed*dt, target-sent))
                        self._device.set_position(self._calibration.to_sdk(sent))
                    state = _feedback_after_command(self._device, before)
                else:
                    state = self._device.refresh()  # Passive register query; no enable or position command.
                if state.fault_code:
                    raise RuntimeError(f'夹爪故障码 0x{state.fault_code:X}')
                if commanding and state.motion_state != 1:
                    raise RuntimeError('夹爪运动期间上报失能')
                position, sdk_stamp = self._calibration.from_sdk(state.position), state.timestamp
                native = state.position
            if not math.isfinite(position) or not 0 <= position <= 1 or sdk_stamp <= self._previous_sdk_stamp:
                raise RuntimeError('夹爪反馈无效或重复')
            elapsed = sdk_stamp - self._previous_sdk_stamp
            velocity = (position-previous)/elapsed if previous is not None and elapsed > 0 else 0.
            with self._lock:
                self._position, self._velocity = position, velocity
                self._sent = sent
                self._previous_sdk_stamp = sdk_stamp
                self._last_feedback = time.monotonic()
                self._error = None
                self._sdk_opening = native
                self._sdk_raw_position = None if self._simulate else getattr(state, 'raw_position', None)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self._joint_name]
            msg.position = [position * STROKE]
            msg.velocity = [velocity * STROKE]
            self._publisher.publish(msg)
            self._diagnostics.publish(String(data=json.dumps({
                'stamp_ros_ns': self.get_clock().now().nanoseconds,
                'position_rad': position * STROKE,
                'sdk_opening': self._sdk_opening,
                'sdk_raw_position': self._sdk_raw_position,
                'calibrating': self._calibrating,
                'closed_sdk_opening': self._calibration.closed,
                'open_sdk_opening': self._calibration.opened,
                'command_closed_sdk_opening': self._calibration.command_closed,
                'command_open_sdk_opening': self._calibration.command_opened,
                'sdk_command_target': (self._native_target if self._native_target is not None
                                       else self._calibration.to_sdk(self._target)
                                       if self._target is not None else None),
                'sdk_command_sent': (self._native_sent if self._native_target is not None
                                     else self._calibration.to_sdk(self._sent)
                                     if self._sent is not None else None),
            }, allow_nan=False)))
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._failures += 1
                self._commanding = False
                self._target = self._sent = None
                self._native_target = self._native_sent = None
            if now-self._last_error_log >= 5.:
                self.get_logger().error(f'Woan Picker 反馈异常：{exc}；停止推进目标')
                self._last_error_log = now

    def stop(self):
        with self._lock:
            self._stopping = True
        self._timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = WoanGripperController()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
