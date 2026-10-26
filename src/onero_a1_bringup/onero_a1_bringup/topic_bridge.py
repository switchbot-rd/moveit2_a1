"""Onero A1 话题桥适配节点。

职责（只做这一件事）：
    把 ros2_control 的 ``joint_state_topic_hardware_interface`` 接到 armsdk。

数据流::

    joint_trajectory_controller
        │  /robot_joint_commands   (sensor_msgs/JointState)   ← 订阅
        ▼
    本节点 ──── 200Hz 控制循环 ────▶ armsdk Arm.servoj()  ──▶ CAN ──▶ 电机
        │
        │  /robot_joint_states     (sensor_msgs/JointState)   → 发布
        ▼
    joint_state_topic_hardware_interface  →  controller_manager

设计边界（有意为之）：
  * **不做轨迹处理** —— 插值、容差判断、取消、反馈全部由
    ``joint_trajectory_controller`` 负责。本节点只接收逐周期目标点。
  * **自己拥有控制循环** —— 以固定频率向硬件发命令，与 ros2_control 的
    update_rate 解耦。
  * **关节按名字映射**，不假设话题里的顺序。
"""

from __future__ import annotations

import math
import json
import os
from collections import deque
from pathlib import Path
import threading
import time
from typing import Optional
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .velocity_feedforward import VelocityFeedforward
from .payload import Payload, PayloadTransition
from .gains import GainTransition, validate_gain

DEFAULT_JOINT_NAMES = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]


def _trace_json_safe(value):
    """Keep diagnostics serializable even when hardware feedback is invalid."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, list):
        return [_trace_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {key: _trace_json_safe(item) for key, item in value.items()}
    return value


class OneroTopicBridge(Node):
    """把 JointState 话题桥接到 armsdk 的适配节点。"""

    def __init__(self) -> None:
        super().__init__("onero_topic_bridge")

        # ─── 参数 ────────────────────────────────────────────────────
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_commands_topic", "/robot_joint_commands")
        self.declare_parameter("joint_states_topic", "/robot_joint_states")
        self.declare_parameter("sim_initial_positions", [0.0] * 7)
        self.declare_parameter("gripper_controller_name", "gripper_controller")
        self.declare_parameter("gripper_joint_name", "woan_picker_left_finger_joint")
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("state_publish_rate", 100.0)
        self.declare_parameter("trace_control", False)
        # Resolve simulation before choosing the default for direct node users.
        self.declare_parameter("simulate", True)
        self.declare_parameter("velocity_feedforward", not bool(self.get_parameter("simulate").value),
                               descriptor=ParameterDescriptor(read_only=True,
                                   description="统一控制七关节速度前馈；真机默认开启，模拟默认关闭"))
        self.declare_parameter("velocity_feedforward_scale", [1.0] * 7,
                               descriptor=ParameterDescriptor(read_only=True,
                                   description="兼容参数，固定为全 1；使用 velocity_feedforward 统一开关"))
        self.declare_parameter("velocity_feedforward_timeout", 0.1,
                               descriptor=ParameterDescriptor(read_only=True))

        # simulate=true 时不连硬件，用一阶跟随模拟机械臂（用于无真机联调）
        self.declare_parameter("sim_time_constant", 0.02)

        # 真机参数（simulate=false 时生效）
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("model", "A1R")
        # Runtime gain edits are ramped and acknowledged by the control thread.
        self.declare_parameter("kp", 50.0, descriptor=ParameterDescriptor(
            description="位置比例增益，支持运行时平滑切换"))
        self.declare_parameter("kd", 1.0, descriptor=ParameterDescriptor(
            description="速度阻尼增益，支持运行时平滑切换"))
        self.declare_parameter("mounting", "upright")
        self.declare_parameter("with_gripper", True)
        self.declare_parameter("gripper_model", "none", descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("allow_gripper_execution", True, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("activate_gripper_on_start", True, descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("gripper_closed_sdk_opening", 0., descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("gripper_open_sdk_opening", 1., descriptor=ParameterDescriptor(read_only=True))
        self.declare_parameter("gripper_calibration_file", "", descriptor=ParameterDescriptor(read_only=True))
        from .gripper_coordinates import GripperCalibration, default_calibration_path, load_calibration
        GripperCalibration(float(self.get_parameter("gripper_closed_sdk_opening").value),
                           float(self.get_parameter("gripper_open_sdk_opening").value))
        self._gripper_calibration_file = str(self.get_parameter("gripper_calibration_file").value)
        if self.get_parameter("gripper_model").value == "woan_picker":
            simulated = bool(self.get_parameter("simulate").value)
            model, channel = str(self.get_parameter("model").value), str(self.get_parameter("can_channel").value)
            if not self._gripper_calibration_file and not simulated:
                ros_home = os.environ.get("ROS_HOME", str(Path.home()/".ros"))
                self._gripper_calibration_file = str(default_calibration_path(ros_home, model, channel))
            if self._gripper_calibration_file and self._gripper_calibration_file != "none":
                load_calibration(self._gripper_calibration_file, model=model, channel=channel, simulate=simulated)

        # CAN 健康与故障处置
        #   log     —— 仅记录（默认，最保守）
        #   hold    —— 冻结目标，保持当前位置
        #   disable —— 断使能电机（⚠️ 带载时会掉落，仅在机械臂有支撑时使用）
        self.declare_parameter("on_fault_action", "log")
        self.declare_parameter("max_consecutive_misses", 10)

        # 使能/失能服务（~/enable、~/disable）
        #   失能会让电机卸力 —— 带载且无支撑时机械臂会掉落。
        #   因此 ~/disable 默认【拒绝执行】，必须显式开启：
        #       -p allow_disable_service:=true
        self.declare_parameter("allow_disable_service", False)

        # 重力补偿前馈（真机默认开启，模拟模式不计算）
        #   需要 onero-sdk 的 dynamics 依赖（pinocchio）。开启前请确认负载参数正确，
        #   安装方向、夹爪标定和重力计算均由 SDK Arm 负责。
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter("max_gravity_torque", 15.0)
        self.declare_parameter("freedrive_damping", .3)
        self.declare_parameter('payload_mass', 0.)
        self.declare_parameter('payload_cog', [0., 0., 0.])
        self.declare_parameter('payload_transition_time', .5, descriptor=ParameterDescriptor(read_only=True))

        gp = self.get_parameter
        self._joint_names: list[str] = list(gp("joint_names").value)
        self._n = len(self._joint_names)
        self._idx = {n: i for i, n in enumerate(self._joint_names)}
        self._velocity_ff = VelocityFeedforward(
            self._joint_names, enabled=bool(gp("velocity_feedforward").value),
            scale=gp("velocity_feedforward_scale").value,
            timeout=gp("velocity_feedforward_timeout").value,
        )

        self._control_rate = float(gp("control_rate").value)
        if not math.isfinite(self._control_rate) or self._control_rate <= 0:
            raise ValueError("control_rate 必须是有限正数")
        self._control_dt = 1.0 / self._control_rate
        self._publish_every = max(
            1, int(round(self._control_rate / float(gp("state_publish_rate").value)))
        )

        self._simulate = bool(gp("simulate").value)
        self._sim_tau = float(gp("sim_time_constant").value)

        # 故障处置策略 —— 在启动时就校验，避免真机跑到一半才发现配置写错
        self._on_fault_action = str(gp("on_fault_action").value).lower()
        self._max_misses = int(gp("max_consecutive_misses").value)
        if self._on_fault_action not in ("log", "hold", "disable"):
            raise RuntimeError(
                f"on_fault_action 取值非法: '{self._on_fault_action}'"
                "（可选 log / hold / disable）"
            )

        # ─── 状态（由控制线程写，锁内交换）────────────────────────────
        self._lock = threading.Lock()
        self._hardware_lock = threading.RLock()
        self._freedrive = False
        self._freedrive_deadline = 0.
        self._freedrive_guard = False
        self.add_on_set_parameters_callback(self._validate_freedrive_parameters)
        self._cmd_pos = [0.0] * self._n          # 最新目标位置
        self._state_pos = [0.0] * self._n        # 当前关节位置
        if self._simulate:
            initial = list(gp("sim_initial_positions").value)
            if len(initial) != self._n or not all(map(math.isfinite, initial)):
                raise ValueError("sim_initial_positions 必须包含全部关节的有限角度")
            self._state_pos = list(initial)
            self._cmd_pos = list(initial)
        self._state_vel = [0.0] * self._n        # 当前关节速度
        self._last_cmd_stamp: Optional[float] = None   # 收到命令的单调时刻
        # Optional diagnosis: collect in the control thread, serialize/publish
        # in the ROS executor so file I/O never runs in the CAN control loop.
        self._trace_control = bool(gp("trace_control").value)
        self._trace_queue = deque(maxlen=1000)
        self._trace_dropped = 0
        self._cycle_missed_joints = []

        # CAN 健康计数（仅真机模式会增长）
        self._can_misses = 0
        self._can_faults = 0
        self._hold = False          # on_fault_action=hold 时置位，冻结目标

        # 电机使能状态。模拟模式下恒为 True（没有真实电机可控制）。
        # 失能后控制循环不再下发命令，只继续发布（冻结的）状态。
        self._enabled = True
        self._allow_disable = bool(gp("allow_disable_service").value)

        # SDK 重力补偿（可选）
        self._gravity_compensation = bool(gp("gravity_compensation").value)
        self._max_gravity_torque = abs(float(gp("max_gravity_torque").value))
        self._payload = PayloadTransition(gp('payload_mass').value, gp('payload_cog').value,
                                          gp('payload_transition_time').value)
        self._payload_control_error = ''
        self._kp = validate_gain('kp', gp('kp').value)
        self._kd = validate_gain('kd', gp('kd').value)
        self._gains = GainTransition(self._kp, self._kd)

        # ─── 硬件后端 ────────────────────────────────────────────────
        self._backend = None
        if not self._simulate:
            self._init_hardware()
        else:
            self.get_logger().info(
                "模拟模式：不连接硬件，用一阶跟随模拟机械臂。"
                "真机启动请选择 hardware_mode:=real"
            )
            if gp("gravity_compensation").value:
                self.get_logger().info(
                    "模拟模式不计算重力补偿；该开关仅在真机模式生效"
                )

        # Identify this service instance without recording a return position.
        # Only the record_startup example captures a return reference.
        self.declare_parameter(
            "session_id", uuid.uuid4().hex,
            descriptor=ParameterDescriptor(read_only=True), ignore_override=True,
        )
        self.declare_parameter(
            "recording_identity", json.dumps({
                "model": str(gp("model").value),
                "can_channel": str(gp("can_channel").value),
                "mounting": str(gp("mounting").value),
                "simulate": self._simulate,
                "gripper_model": str(gp("gripper_model").value),
                "zero_offsets": list(self._backend.offsets) if self._backend is not None else [0.] * self._n,
            }, ensure_ascii=False, allow_nan=False),
            descriptor=ParameterDescriptor(read_only=True), ignore_override=True,
        )

        # ─── ROS 接口 ────────────────────────────────────────────────
        self.add_on_set_parameters_callback(self._validate_payload_parameters)
        self.add_post_set_parameters_callback(self._request_payload)
        self.add_on_set_parameters_callback(self._validate_gain_parameters)
        self.add_post_set_parameters_callback(self._request_gain)
        # 命令用 reliable + depth 1：永远只关心最新目标，不积压
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # 状态用 reliable：
        #   硬件插件用 SensorDataQoS (best_effort) 订阅 —— reliable 发布者可以兼容；
        #   若发 best_effort，则所有默认 reliable 的订阅者（ros2 topic echo 等）
        #   都会收不到消息。reliable 是严格更优的选择。
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cmd_sub = self.create_subscription(
            JointState,
            gp("joint_commands_topic").value,
            self._on_command,
            cmd_qos,
        )
        self._state_pub = self.create_publisher(
            JointState, gp("joint_states_topic").value, state_qos
        )
        self._payload_pub = self.create_publisher(String, '~/payload_state', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._payload_timer = self.create_timer(.1, self._publish_payload_state)
        self._publish_payload_state()
        self._reset_payload_srv = self.create_service(Trigger, '~/reset_payload', self._on_reset_payload)
        self._gain_pub = self.create_publisher(String, '~/gain_state', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._gain_timer = self.create_timer(.1, self._publish_gain_state)
        if self._trace_control:
            self._trace_pub = self.create_publisher(
                String, "~/control_trace",
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self._trace_timer = self.create_timer(0.05, self._publish_control_trace)

        # ─── 使能 / 失能服务 ─────────────────────────────────────────
        # 只在真机模式创建：模拟模式下没有电机可使能，创建了反而误导。
        if self._backend is not None:
            self._enable_srv = self.create_service(
                Trigger, "~/enable", self._on_enable
            )
            self._disable_srv = self.create_service(
                Trigger, "~/disable", self._on_disable
            )
        self._freedrive_srv = self.create_service(SetBool, '~/freedrive', self._on_freedrive)
        self._freedrive_heartbeat_srv = self.create_service(
            Trigger, '~/freedrive_heartbeat', self._on_freedrive_heartbeat)

        # ─── 控制线程 ────────────────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"话题桥已启动 | 关节 {self._n} 个 | 控制 {self._control_rate:.0f}Hz "
            f"| 状态发布 {gp('state_publish_rate').value}Hz | "
            f"simulate={self._simulate}"
        )
        self.get_logger().info(
            f"  订阅 {gp('joint_commands_topic').value}"
            f"  发布 {gp('joint_states_topic').value}"
        )
        self.get_logger().info(
            f"速度前馈={self._velocity_ff.enabled} | 比例={list(self._velocity_ff.scale)} "
            f"| 过期归零={self._velocity_ff.timeout:.3f}s"
        )

    # ─── 硬件初始化 ──────────────────────────────────────────────────

    def _init_hardware(self) -> None:
        """Configure SDK Arm before enabling; fail if requested dynamics are unavailable."""
        try:
            from .sdk_session import SDKArmSession
        except ImportError as e:
            raise RuntimeError(
                "无法导入 onero-sdk。请 source setup_env.sh 安装依赖并加载项目运行环境。"
            ) from e

        gp = self.get_parameter
        model_name = str(gp("model").value)
        channel = str(gp("can_channel").value)
        try:
            self._backend = SDKArmSession(
                model_name=model_name, channel=channel,
                mounting=str(gp("mounting").value),
                with_gripper=bool(gp("with_gripper").value),
                gravity_compensation=self._gravity_compensation,
                control_rate=self._control_rate,
                max_gravity_torque=self._max_gravity_torque,
                on_miss=self._on_can_miss, on_fault=self._on_can_fault,
                gripper_model=str(gp("gripper_model").value),
                payload_mass=self._payload.baseline.mass,
                payload_cog=self._payload.baseline.cog,
            )
            self._model = self._backend.model
            if self._model.num_joints != self._n:
                raise ValueError("SDK 模型与 joint_names 关节数不匹配")
            # SDK preflight above has not opened CAN or enabled any motors.
            state = self._backend.enable()
            self._state_pos = list(state.joints)
            self._cmd_pos = list(state.joints)
            self._enabled = True
        except Exception:
            if self._backend is not None:
                self._backend.close(disable=False)
            raise

        self.get_logger().info(
            f"已连接 CAN '{channel}' | 模型 {model_name} | 电机已使能 | "
            f"SDK Arm.servoj | 故障处置={self._on_fault_action}"
        )
        self.get_logger().info(
            f"SDK 安装={gp('mounting').value} | 原装夹爪={gp('with_gripper').value} "
            f"| 重力补偿={self._gravity_compensation} "
            f"| 夹爪标定系数={self._backend.arm.gravity_profile_scales} "
            f"| 零位偏移={self._backend.offsets}"
        )
        self.get_logger().info(
            f"SDK servoj 增益 | kp={self._kp} | kd={self._kd} | 运行时平滑切换 | 作用于全部 {self._n} 个关节"
        )

    # ─── 使能 / 失能 ─────────────────────────────────────────────────

    def _validate_gain_parameters(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        if not {'kp', 'kd'}.intersection(values):
            return SetParametersResult(successful=True)
        try:
            if not set(values) <= {'kp', 'kd'} or len(parameters) != len(values):
                raise ValueError('kp/kd 需单独或一起提交，不能与其他参数混合设置')
            for name, value in values.items():
                validate_gain(name, value)
            if self._freedrive or not self._enabled:
                raise ValueError('拖动或失能期间不能切换增益')
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _request_gain(self, parameters):
        values = {p.name: p.value for p in parameters if p.name in ('kp', 'kd')}
        if values:
            self._gains.request(values, time.monotonic())

    def _publish_gain_state(self):
        state = self._gains.snapshot()
        state.update({'session_id': self.get_parameter('session_id').value,
                      'error': self._payload_control_error})
        self._gain_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def _validate_payload_parameters(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        names = {'payload_mass', 'payload_cog'}
        if not names.intersection(values):
            return SetParametersResult(successful=True)
        if set(values) != names or len(parameters) != 2:
            return SetParametersResult(successful=False, reason='负载质量和重心必须单独一次原子提交')
        try:
            payload = Payload.create(values['payload_mass'], values['payload_cog'])
            if not self._simulate and not self._gravity_compensation and payload.mass:
                raise ValueError('启用负载补偿需要 gravity_compensation:=true')
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _request_payload(self, parameters):
        values = {parameter.name: parameter.value for parameter in parameters}
        if {'payload_mass', 'payload_cog'} <= values.keys():
            self._payload.request(Payload.create(values['payload_mass'], values['payload_cog']), time.monotonic())

    def _publish_payload_state(self):
        state = self._payload.snapshot()
        state.update({'session_id': self.get_parameter('session_id').value,
                      'stamp_ns': self.get_clock().now().nanoseconds, 'simulate': self._simulate,
                      'gravity_compensation': self._gravity_compensation})
        state['error'] = state['error'] or self._payload_control_error
        self._payload_pub.publish(String(data=json.dumps(state, allow_nan=False)))

    def _on_reset_payload(self, request, response):
        baseline = self._payload.baseline
        result = self.set_parameters_atomically([
            Parameter('payload_mass', value=baseline.mass),
            Parameter('payload_cog', value=list(baseline.cog)),
        ])
        response.success = result.successful
        response.message = ('已请求恢复启动负载基线，以 payload_state 为准' if result.successful else result.reason)
        return response

    def _validate_freedrive_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'freedrive_damping':
                if (type(parameter.value) not in (int, float) or not math.isfinite(parameter.value)
                        or not 0. <= parameter.value <= 5.):
                    return SetParametersResult(successful=False, reason='拖动阻尼必须在 [0, 5] 内')
                if self._freedrive:
                    return SetParametersResult(successful=False, reason='拖动期间不能修改阻尼')
        return SetParametersResult(successful=True)

    def _on_freedrive(self, request, response):
        try:
            with self._hardware_lock:
                if request.data:
                    with self._lock:
                        if self._backend is None:
                            raise RuntimeError('零重力拖动仅支持 real 模式')
                        if not self._enabled or self._hold or not self._gravity_compensation:
                            raise RuntimeError('需要电机已使能、无故障保持且重力补偿已开启')
                        if self._freedrive:
                            raise RuntimeError('该手臂已被其他拖动示例占用')
                        if any(abs(v) > .02 for v in self._state_vel):
                            raise RuntimeError('请等待手臂停止运动后再进入拖动模式')
                        self._freedrive_damping = float(self.get_parameter('freedrive_damping').value)
                        if not math.isfinite(self._freedrive_damping) or not 0. <= self._freedrive_damping <= 5.:
                            raise RuntimeError('拖动阻尼必须在 [0, 5] 内')
                        self._freedrive = True
                        self._freedrive_deadline = time.monotonic()+2.
                        self._freedrive_guard = True
                        self._velocity_ff.clear('freedrive', reset_order=True)
                else:
                    self._finish_freedrive()
            response.success = True
            response.message = '已进入重力补偿拖动' if request.data else '已退出拖动并保持当前位置'
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_freedrive_heartbeat(self, _request, response):
        with self._lock:
            response.success = self._freedrive and self._enabled and not self._hold
            if response.success:
                self._freedrive_deadline = time.monotonic()+2.
        response.message = '拖动中' if response.success else '拖动已结束'
        return response

    def _finish_freedrive(self):
        """Called with the hardware lock; never release a stale controller target."""
        if not self._freedrive:
            return
        state = self._backend.hold_current(self._kp, self._kd) if self._enabled else None
        with self._lock:
            if state is not None:
                self._state_pos = list(state.joints)
                self._state_vel = list(state.velocities)
            self._cmd_pos = list(self._state_pos)
            self._last_cmd_stamp = None
            self._velocity_ff.clear('freedrive_exit', reset_order=True)
            self._freedrive = False
            self._freedrive_guard = True
        self._publish_state()

    def _on_enable(self, _request, response):
        """重新使能电机。"""
        if self._enabled:
            response.success = True
            response.message = "电机已经使能"
            return response
        try:
            with self._hardware_lock:
                state = self._backend.enable()
            with self._lock:
                self._state_pos = list(state.joints)
                self._cmd_pos = list(state.joints)   # 目标对齐当前位置，避免跳变
                self._velocity_ff.clear("reenabled", reset_order=True)
                self._last_cmd_stamp = None
                self._enabled = True
            response.success = True
            response.message = "电机已使能"
            self.get_logger().warn("电机已使能（目标已对齐当前位置）")
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f"使能失败: {e}"
            self.get_logger().error(f"使能失败: {e}")
        return response

    def _on_disable(self, _request, response):
        """断使能电机。

        ⚠️ 这是危险操作：卸力后带载的机械臂会掉落。
        因此默认**拒绝执行**，必须显式开启 allow_disable_service 参数。
        """
        if not self._allow_disable:
            response.success = False
            response.message = (
                "已拒绝：断使能会让机械臂卸力，带载且无支撑时会掉落。"
                "确认机械臂已由物理支撑托住后，用启动参数开启本服务："
                " -p allow_disable_service:=true"
            )
            self.get_logger().error(
                "收到 ~/disable 请求但已拒绝（allow_disable_service=false）。"
                "这是防止带载掉臂的保护。"
            )
            return response

        try:
            with self._lock:
                self._enabled = False             # 控制循环随即停止下发
                self._freedrive = False
                self._velocity_ff.clear("disabled")
            with self._hardware_lock:
                self._backend.disable()
            response.success = True
            response.message = "电机已断使能 —— 机械臂已卸力"
            self.get_logger().warn(
                "电机已断使能（~/disable）—— ⚠️ 机械臂已卸力，请确认其处于支撑状态。"
                "注意：arm_controller 仍处于 active，MoveIt 仍会下发轨迹"
                "（本节点会忽略），如需彻底阻断请停用控制器。"
            )
        except Exception as e:  # noqa: BLE001
            response.success = False
            response.message = f"断使能失败: {e}"
            self.get_logger().error(f"断使能失败: {e}")
        return response

    # ─── CAN 健康回调（仅真机模式）────────────────────────────────────

    def _on_can_miss(self, joint_index: int, consecutive: int) -> None:
        """CAN 帧丢失。偶尔丢帧正常，连续丢帧说明链路有问题。"""
        with self._lock:
            self._can_misses += 1
            if self._trace_control:
                self._cycle_missed_joints.append(joint_index)
        if consecutive == self._max_misses:
            self.get_logger().error(
                f"关节 {joint_index + 1} 连续丢帧 {consecutive} 次 —— CAN 链路可能异常"
            )
        elif consecutive < self._max_misses:
            self.get_logger().debug(
                f"关节 {joint_index + 1} 丢帧（连续 {consecutive} 次）"
            )

    def _on_can_fault(self, joint_index: int, error_code: int) -> None:
        """电机上报硬件故障。这是需要立即处置的事件。"""
        with self._lock:
            self._can_faults += 1
        self.get_logger().error(
            f"关节 {joint_index + 1} 上报故障码 0x{int(error_code):X}"
            f"（累计 {self._can_faults} 次）| 处置={self._on_fault_action}"
        )

        if self._on_fault_action == "hold":
            # 冻结目标：把目标锁到当前实际位置，后续命令一律忽略
            with self._lock:
                self._hold = True
                self._cmd_pos = list(self._state_pos)
                self._velocity_ff.clear("fault_hold")
            self.get_logger().warn("已进入保持模式，忽略后续轨迹命令")
        elif self._on_fault_action == "disable":
            self.get_logger().warn(
                "正在断使能电机 —— ⚠️ 若机械臂带载且无支撑，可能会掉落"
            )
            try:
                with self._hardware_lock:
                    self._backend.disable()
                with self._lock:
                    self._enabled = False
                    self._freedrive = False
                    self._velocity_ff.clear("fault_disabled")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"断使能失败: {e}")

    # ─── 命令回调 ────────────────────────────────────────────────────

    def _on_command(self, msg: JointState) -> None:
        """接收逐周期目标点。按名字映射关节，不假设顺序。

        ⚠️ 必须过滤非有限值（NaN / inf）：
        ros2_control 对【没有 initial_value 的接口】会用 NaN 初始化
        （见 hardware_interface/handle.hpp），因此控制器认领命令接口之前，
        话题里可能短暂出现 NaN。若原样转发给电机，后果不可预期。
        """
        if not msg.name:
            with self._lock:
                self._velocity_ff.clear("invalid_command")
            self.get_logger().warn("收到无关节名的命令，已忽略", throttle_duration_sec=5.0)
            return

        accepted = 0
        with self._lock:
            if self._freedrive:
                return
            if self._freedrive_guard:
                positions = dict(zip(msg.name, msg.position))
                # Resume only after the controller has aligned to the guided
                # pose. Old idle or queued trajectory commands stay blocked.
                if any(name not in positions or not math.isfinite(positions[name])
                       or abs(positions[name]-self._cmd_pos[index]) > .01
                       for index, name in enumerate(self._joint_names)):
                    return
                self._freedrive_guard = False
            if self._hold:
                # 已因硬件故障进入保持模式：忽略一切新目标
                return
            if not self._enabled:
                # 已失能：不接受新目标，避免 _cmd_pos 悄悄漂到别处，
                # 使能时再对齐当前位置
                return
            received_s = time.monotonic()
            self._velocity_ff.capture(
                msg.name, msg.position, msg.velocity,
                stamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                now_s=received_s, now_ros_ns=self.get_clock().now().nanoseconds,
            )
            if self._velocity_ff.reason in ("out_of_order", "stale_header"):
                self.get_logger().warn(
                    "命令时间戳过期或乱序，已忽略该位置帧并将速度前馈归零",
                    throttle_duration_sec=5.0,
                )
                return
            for name, pos in zip(msg.name, msg.position):
                i = self._idx.get(name)
                if i is None:
                    self.get_logger().warn(
                        f"命令中有未知关节 '{name}'，已忽略",
                        throttle_duration_sec=5.0,
                    )
                    continue
                value = float(pos)
                if not math.isfinite(value):
                    self.get_logger().warn(
                        f"命令中关节 '{name}' 的值非有限（{value}），已忽略",
                        throttle_duration_sec=5.0,
                    )
                    continue
                self._cmd_pos[i] = value
                accepted += 1
            if accepted:
                self._last_cmd_stamp = received_s

    # ─── 控制循环 ────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        cycle = 0
        next_t = time.monotonic()
        previous_start = None

        while self._running:
            cycle_start = time.monotonic()
            ros_start_ns = self.get_clock().now().nanoseconds if self._trace_control else None
            # 1) 取最新目标
            with self._lock:
                cmd = list(self._cmd_pos)
                command_stamp = self._last_cmd_stamp
                input_velocity, velocity, velocity_reason = self._velocity_ff.sample(
                    now_s=time.monotonic(), now_ros_ns=self.get_clock().now().nanoseconds,
                )
                self._cycle_missed_joints = []

            state = None
            failure = None
            applied = None
            io_start = cycle_start

            # 2) 与硬件交互（真机 or 模拟）
            #    失能后不再下发命令 —— 电机已卸力，继续发没有意义，
            #    而且 MIT 帧可能会把电机重新拉起来。状态照常发布。
            if self._backend is not None:
                if not self._enabled:
                    time.sleep(self._control_dt)
                    continue
                try:
                    io_start = time.monotonic()
                    with self._hardware_lock:
                        if not self._enabled:
                            continue
                        payload_revision = self._payload.advance(self._backend.set_payload, time.monotonic())
                        if self._freedrive and (time.monotonic() >= self._freedrive_deadline or self._hold):
                            self._finish_freedrive()
                            cmd = list(self._cmd_pos)
                            velocity = [0.]*self._n
                            self.get_logger().warning('拖动会话已结束，保持当前位置')
                        if self._freedrive:
                            state = self._backend.freedrive_step(self._freedrive_damping)
                        else:
                            # A mode switch can occur after the snapshot at the
                            # start of this cycle; guarded holds use the new pose.
                            with self._lock:
                                cmd = list(self._cmd_pos)
                                _, velocity, _ = self._velocity_ff.sample(
                                    now_s=time.monotonic(), now_ros_ns=self.get_clock().now().nanoseconds)
                                if self._freedrive_guard:
                                    velocity = [0.]*self._n
                            gains, gain_revision = self._gains.advance(time.monotonic())
                            self._kp, self._kd = gains['kp'], gains['kd']
                            state = self._backend.servoj(
                                cmd, kp=self._kp, kd=self._kd, velocities=velocity
                            )
                            self._gains.confirm(gains, gain_revision, self.get_clock().now().nanoseconds)
                        applied = self._backend.last_command
                        self._payload.confirm(payload_revision, limited=bool(applied and (
                            applied.get('sdk_ff_limited') or applied.get('bridge_ff_limited'))))
                        self._payload_control_error = ''
                    io_end = time.monotonic()
                    with self._lock:
                        self._state_pos = list(state.joints)
                        self._state_vel = list(state.velocities)
                except Exception as e:  # noqa: BLE001 — 控制循环不能死
                    io_end = time.monotonic()
                    failure = str(e)
                    self._payload_control_error = failure
                    applied = self._backend.last_command
                    self.get_logger().error(
                        f"控制周期失败: {e}", throttle_duration_sec=1.0
                    )
                    if self._freedrive:
                        try:
                            with self._hardware_lock:
                                self._finish_freedrive()
                        except Exception as hold_error:
                            with self._lock:
                                self._freedrive = False
                                self._freedrive_guard = True
                                self._cmd_pos = list(self._state_pos)
                                self._velocity_ff.clear('freedrive_failed', reset_order=True)
                            self.get_logger().error(f'拖动中断且持位请求失败: {hold_error}')
            else:
                payload_revision = self._payload.advance(lambda mass, cog: None, time.monotonic())
                gains, gain_revision = self._gains.advance(time.monotonic())
                self._kp, self._kd = gains['kp'], gains['kd']
                self._step_simulation(cmd)
                self._payload.confirm(payload_revision)
                self._gains.confirm(gains, gain_revision, self.get_clock().now().nanoseconds)
                io_end = time.monotonic()

            if self._trace_control:
                with self._lock:
                    if len(self._trace_queue) == self._trace_queue.maxlen:
                        self._trace_dropped += 1
                    self._trace_queue.append({
                        "cycle": cycle,
                        "ros_start_ns": ros_start_ns,
                        "monotonic_start": cycle_start,
                        "period_s": (cycle_start - previous_start)
                        if previous_start is not None else None,
                        "command_age_s": max(0.0, io_start - command_stamp)
                        if command_stamp is not None else None,
                        "io_s": io_end - io_start,  # SDK servo incl. gravity, limits and CAN I/O
                        "joint_names": self._joint_names,
                        "sdk_input_position": cmd,
                        "input_velocity": input_velocity,
                        "sdk_input_velocity": velocity,
                        "velocity_feedforward": self._velocity_ff.enabled,
                        "velocity_feedforward_scale": list(self._velocity_ff.scale),
                        "velocity_feedforward_status": velocity_reason,
                        "command_position": applied["position"] if applied else (
                            cmd if self._simulate else None),
                        "raw_command_position": applied["raw_position"] if applied else None,
                        "command_velocity": applied["velocity"] if applied else (
                            velocity if self._simulate else None),
                        "command_kp": applied["kp"] if applied else None,
                        "command_kd": applied["kd"] if applied else None,
                        "command_torque_ff": applied["torque_ff"] if applied else None,
                        "requested_torque_ff": applied["requested_torque_ff"] if applied else None,
                        "sdk_torque_ff": applied["sdk_torque_ff"] if applied else None,
                        "sdk_ff_limited": applied["sdk_ff_limited"] if applied else None,
                        "bridge_ff_limited": applied["bridge_ff_limited"] if applied else None,
                        "gravity_compensation": self._gravity_compensation,
                        "payload": self._payload.snapshot(),
                        "kp": getattr(self, "_kp", None),
                        "kd": getattr(self, "_kd", None),
                        "feedback_position": list(self._state_pos),
                        "feedback_velocity": list(self._state_vel),
                        "feedback_torque": list(state.torques) if state is not None else [],
                        # Missing responses use cached feedback in CanBackend.
                        # Do not treat that joint's values as fresh measurements.
                        "missed_joint_indices": list(self._cycle_missed_joints),
                        "can_misses_total": self._can_misses,
                        "can_faults_total": self._can_faults,
                        "trace_dropped_total": self._trace_dropped,
                        "simulate": self._simulate,
                        "exception": failure,
                    })
            previous_start = cycle_start

            # 3) 发布状态
            cycle += 1
            if cycle % self._publish_every == 0:
                self._publish_state()

            # 4) 定时（用绝对时刻累加，避免漂移）
            next_t += self._control_dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # 落后了就重置基准，避免雪崩式追赶
                next_t = time.monotonic()

    def _publish_control_trace(self) -> None:
        with self._lock:
            samples = list(self._trace_queue)
            self._trace_queue.clear()
        for sample in samples:
            self._trace_pub.publish(String(data=json.dumps(
                _trace_json_safe(sample), allow_nan=False
            )))

    def _step_simulation(self, cmd: list[float]) -> None:
        """一阶跟随：state += (cmd - state) * dt / tau

        带 NaN 恢复：若状态已是非有限值（例如启动期被 NaN 命令污染过），
        直接重置到目标值，避免 NaN 永久传播。
        """
        alpha = min(1.0, self._control_dt / max(self._sim_tau, 1e-6))
        with self._lock:
            for i in range(self._n):
                target = cmd[i]
                current = self._state_pos[i]

                if not math.isfinite(current):
                    # 从污染中恢复
                    self._state_pos[i] = target if math.isfinite(target) else 0.0
                    self._state_vel[i] = 0.0
                    continue

                delta = (target - current) * alpha
                self._state_vel[i] = delta / self._control_dt
                self._state_pos[i] = current + delta

    def _publish_state(self) -> None:
        with self._lock:
            pos = list(self._state_pos)
            vel = list(self._state_vel)

        # 最后一道防线：绝不发布非有限值。
        # NaN 一旦进入 /robot_joint_states，会污染硬件接口的状态接口，
        # 进而让 JTC 的到位判断失效（它可能误判为"已到位"）。
        for i in range(self._n):
            if not math.isfinite(pos[i]):
                pos[i] = 0.0
            if not math.isfinite(vel[i]):
                vel[i] = 0.0

        msg = JointState()
        # 必须用节点时钟，不能用 time.monotonic —— 否则 use_sim_time 下 TF 会崩
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = pos
        msg.velocity = vel
        self._state_pub.publish(msg)

    # ─── 生命周期 ────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._backend is not None:
            # 不自动 disable：带载的机械臂突然卸力会掉落
            try:
                with self._hardware_lock:
                    self._backend.close(disable=False)
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    rclpy.init(args=args)
    node = OneroTopicBridge()
    gripper = None
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        if node.get_parameter("gripper_model").value == "woan_picker":
            from .woan_gripper import WoanGripperController
            gripper = WoanGripperController(
                device=node._backend.gripper if node._backend is not None else None,
                simulate=node._simulate,
                allow_execution=bool(node.get_parameter("allow_gripper_execution").value),
                activate_on_start=bool(node.get_parameter("activate_gripper_on_start").value),
                closed_sdk_opening=float(node.get_parameter("gripper_closed_sdk_opening").value),
                open_sdk_opening=float(node.get_parameter("gripper_open_sdk_opening").value),
                calibration_file=node._gripper_calibration_file,
                arm_model=str(node.get_parameter("model").value),
                can_channel=str(node.get_parameter("can_channel").value),
                embedded=True,
                controller_name=str(node.get_parameter("gripper_controller_name").value),
                joint_name=str(node.get_parameter("gripper_joint_name").value),
            )
            executor.add_node(gripper)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if gripper is not None:
            gripper.stop()
        executor.shutdown(timeout_sec=2.)
        if gripper is not None:
            gripper.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
