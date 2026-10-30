"""Own Servo sessions, command freshness and strict controller handoff."""
import json
import math
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.msg import JointJog, JointTrajectoryControllerState
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from trajectory_msgs.msg import JointTrajectory

from .servo_kinematics import ServoKinematics, DualServoKinematics
from .servo_gripper import ServoGripper, ServoGripperGroup


class ServoManager(Node):
    def __init__(self):
        super().__init__('servo_manager')
        self.declare_parameter('joints', [f'joint{i}-a1_r' for i in range(1, 8)])
        self.declare_parameter('input_timeout', .15)
        self.joints = list(self.get_parameter('joints').value)
        self.timeout = float(self.get_parameter('input_timeout').value)
        self.dual = len(self.joints)==14
        for name,default in (
            ('normal_controllers',['arm_controller']),('bridges',['/onero_topic_bridge']),
            ('raw_feedback_topics',['/robot_joint_states']),
            ('gripper_joints',['woan_picker_left_finger_joint']),('gripper_controllers',['gripper_controller'])):
            self.declare_parameter(name,default)
        self.normal_controllers=list(self.get_parameter('normal_controllers').value)
        self.bridges=list(self.get_parameter('bridges').value)
        self.raw_feedback_topics=list(self.get_parameter('raw_feedback_topics').value)
        self.bridge_times={topic:0. for topic in self.raw_feedback_topics}
        self.available_targets=['left','right','both'] if self.dual else ['left' if self.joints[0].endswith('_l') else 'right']
        self.control_target='right' if self.dual else self.available_targets[0]
        self.pending_target=None
        self.selection_stamp=0
        self.selection_worker=None
        self.declare_parameter('robot_description', '')
        self.declare_parameter('robot_description_semantic', '')
        self.declare_parameter('joint_limit_margins', [.05,.05,.05,.2,.05,.05,.05])
        self.declare_parameter('max_expected_latency', .1)
        velocity, acceleration = [], []
        for joint in self.joints:
            for field, values in [('max_velocity', velocity), ('max_acceleration', acceleration)]:
                name = f'robot_description_planning.joint_limits.{joint}.{field}'
                self.declare_parameter(name, 0.)
                value = float(self.get_parameter(name).value)
                if not math.isfinite(value) or value <= 0.:
                    raise RuntimeError('Servo 缺少有效关节限位：'+name)
                values.append(value)
        solver = DualServoKinematics if self.dual else ServoKinematics
        self.kinematics = solver(self.get_parameter('robot_description').value,
            self.get_parameter('robot_description_semantic').value, self.joints, velocity, acceleration,
            self.get_parameter('joint_limit_margins').value, self.get_parameter('max_expected_latency').value)
        self.io = ReentrantCallbackGroup()
        self.operations = MutuallyExclusiveCallbackGroup()
        self.command_timer_group = MutuallyExclusiveCallbackGroup()
        self.declare_parameter('gripper_model', 'none')
        self.gripper_joints=list(self.get_parameter('gripper_joints').value)
        self.gripper_joint=self.gripper_joints[0]
        controllers=list(self.get_parameter('gripper_controllers').value)
        self.gripper=None
        if self.get_parameter('gripper_model').value == 'woan_picker':
            helpers=[ServoGripper(ActionClient(self,GripperCommand,'/'+name+'/gripper_cmd',callback_group=self.io)) for name in controllers]
            self.gripper=ServoGripperGroup(dict(zip(('left','right'),helpers))) if self.dual else helpers[0]
        self.lock, self.transition = threading.RLock(), threading.Lock()
        self.service_clients, self.stops = {}, []
        self.ready = self.stopping = False
        self.token = None
        self.error, self.servo_message = '', '等待 Servo 就绪'
        self.servo_code = ServoStatus.INVALID
        self.latest = self.input = None
        self.target, self.target_time = None, 0.
        self.feedback_time = self.bridge_time = self.input_time = self.status_time = 0.
        self.sequence = 0
        self.input_stamp = 0
        self.busy = set()
        self.started = 0.
        self.input_sub = None
        self.gripper_sub = None
        self.selection_sub = None
        self.gripper_input, self.gripper_time, self.gripper_stamp = 0, 0., 0
        self.worker = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '~/state', transient)
        self.joint_command_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 1)
        self.create_subscription(JointTrajectory, '/servo_arm_controller/joint_trajectory',
                                 self.native_target, 1, callback_group=self.io)
        self.create_subscription(JointState, '/joint_states', self.feedback, 10, callback_group=self.io)
        for topic in self.raw_feedback_topics:
            self.create_subscription(JointState,topic,
                lambda msg,topic=topic:self.bridge_feedback(msg,topic),10,callback_group=self.io)
        self.create_subscription(ServoStatus, '/servo_node/status', self.status, 10, callback_group=self.io)
        self.create_subscription(JointTrajectoryControllerState, '/servo_arm_controller/controller_state',
                                 self.tracking, 10, callback_group=self.io)
        for topic in tuple('/'+name+'/follow_joint_trajectory/_action/status' for name in self.normal_controllers)+('/execute_trajectory/_action/status',):
            self.create_subscription(GoalStatusArray, topic,
                lambda msg, topic=topic: self.action_status(topic, msg), transient, callback_group=self.io)
        self.create_service(Trigger, '~/start', self.start_session, callback_group=self.operations)
        self.create_service(Trigger, '~/check', self.check, callback_group=self.operations)
        self.create_timer(.01, self.tick, callback_group=self.command_timer_group)
        self.create_timer(.1, self.publish_state, callback_group=self.io)
        threading.Thread(target=self.initialize, daemon=True).start()

    def call(self, kind, name, request, timeout=4.):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(kind, name, callback_group=self.io)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('服务未就绪：'+name)
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            time.sleep(.005)
        if not future.done() or future.result() is None:
            raise RuntimeError('服务请求超时：'+name)
        return future.result()

    def pause(self, paused):
        result = self.call(SetBool, '/servo_node/pause_servo', SetBool.Request(data=paused))
        if not result.success:
            raise RuntimeError('Servo 暂停切换失败：'+result.message)

    def bridge_mode(self, enabled):
        applied=[]
        try:
            for bridge in self.bridges:
                result=self.call(SetBool,bridge+'/servo_mode',SetBool.Request(data=enabled))
                if not result.success:raise RuntimeError(result.message)
                applied.append(bridge)
        except Exception:
            if enabled:
                for bridge in applied:
                    self.call(SetBool,bridge+'/servo_mode',SetBool.Request(data=False))
            raise

    def resume(self):
        self.pause(False)
        resumed = time.monotonic()
        deadline = resumed+2.
        while rclpy.ok() and time.monotonic() < deadline:
            if self.status_time > resumed:
                return
            time.sleep(.01)
        raise RuntimeError('Servo 未产生状态反馈，未授予键盘控制')

    def controllers(self):
        result = self.call(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        return {item.name: item.state for item in result.controller}

    def switch(self, servo):
        states = self.controllers()
        activate,deactivate=(['servo_arm_controller'],self.normal_controllers) if servo else (self.normal_controllers,['servo_arm_controller'])
        request = SwitchController.Request(
            activate_controllers=[name for name in activate if states.get(name)!='active'],
            deactivate_controllers=[name for name in deactivate if states.get(name)=='active'],
            strictness=SwitchController.Request.STRICT, activate_asap=True, timeout=DurationMsg(sec=3))
        if request.activate_controllers or request.deactivate_controllers:
            if not self.call(SwitchController, '/controller_manager/switch_controller', request).ok:
                raise RuntimeError('控制器切换失败')
        states = self.controllers()
        if any(states.get(name)!='active' for name in activate) or any(states.get(name)!='inactive' for name in deactivate):
            raise RuntimeError('控制器互斥状态未确认')

    def initialize(self):
        try:
            self.pause(True)
            self.ready = True
            self.get_logger().info('Servo 已暂停待命，键盘 --execute 后接管控制。')
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error(self.error)

    def feedback(self, msg):
        age = (self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        values = dict(zip(msg.name, msg.position))
        if -.1 <= age <= .2 and all(name in values and math.isfinite(values[name]) for name in self.joints):
            with self.lock:
                self.latest = {name: values[name] for name in self.joints}
                self.feedback_time = time.monotonic()
                self.sequence += 1

    def bridge_feedback(self, msg, topic=None):
        age = (self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        values = dict(zip(msg.name, msg.position))
        topic=topic or self.raw_feedback_topics[0]
        index=self.raw_feedback_topics.index(topic)
        names=self.joints[index*7:index*7+7] if self.dual else self.joints
        if -.1 <= age <= .2 and all(n in values and math.isfinite(values[n]) for n in names):
            self.bridge_times[topic]=time.monotonic()
            self.bridge_time=min(self.bridge_times.values())

    def action_status(self, topic, msg):
        with self.lock:
            if any(item.status in (1, 2, 3) for item in msg.status_list):
                self.busy.add(topic)
            else:
                self.busy.discard(topic)

    def status(self, msg):
        self.status_time, self.servo_message = time.monotonic(), msg.message or str(msg.code)
        if msg.message == 'Checked collision retreat':
            self.servo_message = '正在低速撤离障碍物'
        self.servo_code = msg.code

    def tracking(self, msg):
        if self.token and not self.stopping and time.monotonic()-self.started > .3 and msg.error.positions:
            if any(not math.isfinite(v) or abs(v) > .20 for v in msg.error.positions):
                self.request_stop('Servo 关节跟踪误差超过 0.20 rad')

    def native_target(self, msg):
        if list(msg.joint_names) == self.joints and msg.points:
            values = list(msg.points[-1].positions)
            if len(values) == len(self.joints) and all(map(math.isfinite, values)):
                with self.lock:
                    self.target, self.target_time = values, time.monotonic()

    def wait_still(self, timeout=2.):
        deadline = time.monotonic()+timeout
        reference, since, previous = None, 0., -1
        while time.monotonic() < deadline and rclpy.ok():
            with self.lock:
                values, sequence, stamp = self.latest, self.sequence, self.feedback_time
            now = time.monotonic()
            if values is not None and now-stamp <= .2 and now-self.bridge_time <= .2 and sequence != previous:
                if reference is None or max(abs(values[n]-reference[n]) for n in self.joints) > .002:
                    reference, since = dict(values), now
                elif now-since >= .2:
                    return
                previous = sequence
            time.sleep(.01)
        raise RuntimeError('未获得连续稳定且新鲜的关节反馈')

    def check_ready(self):
        if not self.ready:
            raise RuntimeError(self.error or 'Servo 尚未就绪')
        states = self.controllers()
        if any(states.get(name)!='active' for name in self.normal_controllers) or states.get('servo_arm_controller')!='inactive':
            raise RuntimeError('需普通控制器 active、Servo 控制器 inactive；异常退出后请重启服务')
        if self.latest is None or min(self.feedback_time, self.bridge_time) < time.monotonic()-.2:
            raise RuntimeError('关节反馈不新鲜')

    def check(self, _request, response):
        try:
            if self.token:
                response.success, response.message = False, '手臂已被其他 Servo 会话占用'
                return response
            self.check_ready()
            response.success, response.message = True, 'Servo、控制器和关节反馈已就绪'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def start_session(self, request, response):
        if not self.transition.acquire(blocking=False):
            response.success, response.message = False, '控制模式正在切换'
            return response
        bridged = switched = False
        try:
            if self.token:
                response.success, response.message = False, '手臂已被其他 Servo 会话占用'
                return response
            self.check_ready()
            if self.busy:
                raise RuntimeError('已有轨迹正在执行，请先结束轨迹')
            self.wait_still()
            if self.busy:
                raise RuntimeError('已有轨迹正在执行，请先结束轨迹')
            bridged = True  # Service timeouts can occur after the bridge applied the request.
            self.bridge_mode(True)
            self.pause(True)
            switched = True
            self.switch(True)
            result = self.call(ServoCommandType, '/servo_node/switch_command_type',
                ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG))
            if not result.success:
                raise RuntimeError('Servo 未接受输入模式')
            token = 's_'+uuid.uuid4().hex
            topic, stop = f'/servo_manager/input/{token}', f'/servo_manager/stop/{token}'
            self.kinematics.reset()
            self.input_sub = self.create_subscription(TwistStamped, topic,
                lambda msg, token=token: self.command(msg, token), 1, callback_group=self.io)
            gripper_topic = topic+'/gripper' if self.gripper else None
            if gripper_topic:
                self.gripper_sub = self.create_subscription(JointJog, gripper_topic,
                    lambda msg, token=token: self.gripper_command(msg, token), 1, callback_group=self.io)
            selection_topic=topic+'/selection' if self.dual else None
            if selection_topic:
                self.selection_sub=self.create_subscription(String,selection_topic,
                    lambda msg,token=token:self.select_target(msg,token),1,callback_group=self.io)
            self.stops.append(self.create_service(Trigger, stop,
                lambda request, answer, token=token: self.stop_session(token, answer), callback_group=self.operations))
            self.resume()
            with self.lock:
                self.token, self.stopping, self.input, self.input_time = token, False, None, 0.
                self.input_stamp = 0
                self.gripper_input, self.gripper_time, self.gripper_stamp = 0, 0., 0
                self.pending_target=None
                self.selection_stamp=0
                self.control_target='right' if self.dual else self.available_targets[0]
                if self.dual:
                    self.kinematics.control_target=self.control_target
                    if self.gripper:self.gripper.set_target(self.control_target)
                self.started, self.error = time.monotonic(), ''
            response.success = True
            response.message = json.dumps({'session_id': token, 'command_topic': topic, 'stop_service': stop,
                'gripper_topic':gripper_topic,'gripper_joints':self.gripper_joints,
                'selection_topic':selection_topic,'available_targets':self.available_targets,
                'control_target':self.control_target})
        except Exception as exc:
            self.error = str(exc)
            try:
                if bridged or switched:
                    self.pause(True)
                if switched:
                    self.switch(False)
                if bridged:
                    self.bridge_mode(False)
            except Exception as rollback:
                self.error += '；恢复失败：'+str(rollback)
            self.token = None
            if self.input_sub is not None:
                self.destroy_subscription(self.input_sub)
                self.input_sub = None
            if self.gripper_sub is not None:
                self.destroy_subscription(self.gripper_sub)
                self.gripper_sub = None
            if self.selection_sub is not None:
                self.destroy_subscription(self.selection_sub)
                self.selection_sub=None
            response.success, response.message = False, self.error
        finally:
            self.transition.release()
        return response

    def command(self, msg, token):
        if token != self.token or self.stopping:
            return
        values = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                  msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (msg.header.frame_id not in ('world', 'base_link') or not -.1 <= age <= self.timeout
                or not all(map(math.isfinite, values))):
            self.request_stop('键盘命令无效或已过期')
            return
        try:
            rotation = np.eye(3)
            if msg.header.frame_id != 'base_link':
                transform = self.tf.lookup_transform('base_link', msg.header.frame_id, Time(), timeout=Duration(seconds=.03))
                q = transform.transform.rotation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            linear, angular = rotation @ values[:3], rotation @ values[3:]
            out = TwistStamped()
            out.header.frame_id = 'base_link'
            out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = map(float, linear)
            out.twist.angular.x, out.twist.angular.y, out.twist.angular.z = map(float, angular)
            with self.lock:
                if token != self.token or self.stopping or stamp <= max(self.input_stamp,self.selection_stamp):
                    return
                if (self.get_clock().now().nanoseconds-stamp)/1e9 > self.timeout:
                    return
                self.input, self.input_time = out, time.monotonic()
                self.input_stamp = stamp
        except Exception as exc:
            self.request_stop('命令参考系转换失败：'+str(exc))

    def gripper_command(self, msg, token):
        if token != self.token or self.stopping or self.gripper is None:
            return
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (list(msg.joint_names) != self.gripper_joints or len(msg.velocities) != len(self.gripper_joints)
                or msg.displacements or not -.1 <= age <= self.timeout
                or any(value not in (-1.,0.,1.) for value in msg.velocities)
                or any(value!=msg.velocities[0] for value in msg.velocities)):
            self.request_stop('夹爪按键命令无效或已过期')
            return
        with self.lock:
            if token != self.token or self.stopping or stamp <= max(self.gripper_stamp,self.selection_stamp):
                return
            if (self.get_clock().now().nanoseconds-stamp)/1e9 > self.timeout:
                return
            self.gripper_input, self.gripper_time, self.gripper_stamp = int(msg.velocities[0]), time.monotonic(), stamp

    def select_target(self,msg,token):
        if token!=self.token or self.stopping:return
        try:
            value=json.loads(msg.data);target=value['target'];stamp=int(value['stamp_ns'])
            age=(self.get_clock().now().nanoseconds-stamp)/1e9
            if target not in self.available_targets or not -.1<=age<=self.timeout:
                raise ValueError('选臂请求无效或已过期')
            with self.lock:
                if token!=self.token or self.stopping or stamp<=self.selection_stamp:return
                self.pending_target=target;self.selection_stamp=stamp
                self.input=None;self.gripper_input=0
                if self.selection_worker is None or not self.selection_worker.is_alive():
                    self.selection_worker=threading.Thread(target=self.finish_selection,daemon=True)
                    self.selection_worker.start()
        except Exception as exc:self.request_stop('选臂切换失败：'+str(exc))

    def finish_selection(self):
        with self.transition:
            try:
                while self.token and not self.stopping:
                    with self.lock:stamp=self.selection_stamp
                    self.wait_still()
                    with self.lock:
                        if self.stopping or not self.token: return
                        if stamp!=self.selection_stamp:continue
                        target=self.pending_target
                        if target is None:return
                    if self.gripper:self.gripper.set_target(target)
                    with self.lock:
                        if self.stopping or not self.token:return
                        if stamp!=self.selection_stamp:continue
                        self.kinematics.reset();self.kinematics.control_target=target
                        self.control_target=target;self.pending_target=None;self.input=None
                        self.input_stamp=self.get_clock().now().nanoseconds
                        self.selection_worker=None
                    return
            except Exception as exc:self.request_stop('选臂切换失败：'+str(exc))

    def tick(self):
        with self.lock:
            if not self.token:
                return
            now = time.monotonic()
            if not self.stopping:
                if min(self.feedback_time, self.bridge_time) < now-.2:
                    self.request_stop(f'关节反馈失联（广播 {now-self.feedback_time:.3f}s，原始 {now-self.bridge_time:.3f}s）')
                elif self.input_time and now-self.input_time > self.timeout:
                    self.request_stop('键盘输入超时，已停止并释放控制')
                elif not self.input_time and now-self.started > .5:
                    self.request_stop('未收到键盘心跳')
                elif now-self.started > .5 and now-self.status_time > .5:
                    self.request_stop('Servo 状态失联')
            token = self.token
            incoming = self.input if not self.stopping and self.pending_target is None else None
            q = (list(self.target) if self.target is not None and self.target_time > self.started
                 and now-self.target_time < .15 else [self.latest[n] for n in self.joints])
            gripper_direction = self.gripper_input if not self.stopping and self.pending_target is None and now-self.gripper_time <= self.timeout else 0
        command = JointJog(joint_names=list(self.joints), velocities=[0.]*len(self.joints))
        twist = np.zeros(6) if incoming is None else np.array([
            incoming.twist.linear.x, incoming.twist.linear.y, incoming.twist.linear.z,
            incoming.twist.angular.x, incoming.twist.angular.y, incoming.twist.angular.z])
        try:
            posture_scale = 0. if self.servo_code in (ServoStatus.DECELERATE_FOR_COLLISION, ServoStatus.HALT_FOR_COLLISION) else 1.
            command.velocities = list(map(float, self.kinematics.solve(q, twist, posture_scale=posture_scale)))
        except Exception as exc:
            self.request_stop('遥操作运动学停止：'+str(exc))
        with self.lock:
            if self.token != token or self.stopping:
                command.velocities = [0.]*len(self.joints)
                gripper_direction = 0
        if self.gripper:
            self.gripper.set_direction(gripper_direction)
        command.header.frame_id = 'base_link'
        command.header.stamp = self.get_clock().now().to_msg()
        # Reliable DDS publication may wait; keep input/feedback callbacks free.
        self.joint_command_pub.publish(command)

    def request_stop(self, reason):
        with self.lock:
            if not self.token or self.stopping:
                return
            self.stopping, self.error, self.input = True, reason, None
            self.get_logger().warning(reason)
            self.worker = threading.Thread(target=self.finish_session, daemon=True)
            self.worker.start()

    def finish_session(self):
        with self.transition:
            try:
                if self.gripper:
                    self.gripper.set_direction(0)
                    deadline = time.monotonic()+2.
                    while not self.gripper.idle and time.monotonic() < deadline:
                        time.sleep(.01)
                    if not self.gripper.idle:
                        self.error += '；夹爪停止尚未确认'
                # tick() continues streaming zeros while feedback settles.
                self.wait_still()
                self.pause(True)
                self.switch(False)
                self.bridge_mode(False)
            except Exception as exc:
                self.error += '；停止/恢复失败：'+str(exc)
                self.get_logger().error(self.error)
                try:
                    self.pause(True)
                except Exception:
                    pass
            finally:
                with self.lock:
                    self.token, self.input = None, None
                    if self.input_sub is not None:
                        self.destroy_subscription(self.input_sub)
                        self.input_sub = None
                    if self.gripper_sub is not None:
                        self.destroy_subscription(self.gripper_sub)
                        self.gripper_sub = None
                    if self.selection_sub is not None:
                        self.destroy_subscription(self.selection_sub)
                        self.selection_sub=None

    def stop_session(self, token, response):
        if token != self.token:
            response.success, response.message = False, '该会话已经结束'
            return response
        try:
            self.request_stop('键盘退出')
            if self.worker is not None:
                self.worker.join(timeout=10.)
            states=self.controllers()
            response.success = (self.token is None and all(states.get(name)=='active' for name in self.normal_controllers)
                                and (self.gripper is None or self.gripper.idle))
            response.message = '已停稳并恢复普通控制器' if response.success else self.error
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps({'ready': self.ready, 'active': self.token is not None,
            'stopping': self.stopping, 'session_id': self.token, 'error': self.error,
            'servo_status': self.servo_message, 'servo_code': self.servo_code,
            'gripper_available': self.gripper is not None,
            'gripper_direction': self.gripper.direction if self.gripper else 0,
            'gripper_holding': self.gripper.holding if self.gripper else False,
            'gripper_error': self.gripper.error if self.gripper else '',
            'available_targets':self.available_targets,'control_target':self.control_target,
            'pending_target':self.pending_target,'selection_stamp_ns':self.selection_stamp,
            'cartesian_solver': 'position_priority', 'kinematic_condition': self.kinematics.condition,
            'orientation_deviation_rad': self.kinematics.orientation_deviation,
            'command_posture_cost': self.kinematics.posture_cost,
            'posture_cost': (self.kinematics.posture.cost([self.latest[n] for n in self.joints])
                             if self.latest else None),
            'stamp_ns': self.get_clock().now().nanoseconds}, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ServoManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
