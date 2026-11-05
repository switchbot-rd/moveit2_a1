"""Record paired controller targets/feedback while task clients plan and execute."""
from collections import Counter
from datetime import datetime
import csv
import json
import math
import os
from pathlib import Path
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.interpolate import BPoly
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from onero_a1_common.constants import JOINTS, LEFT_JOINTS
from onero_a1_common.moveit_client import MoveItError, seconds


def state_values(message, names):
    if (len(message.joint_names) != len(set(message.joint_names))
            or len(message.reference.positions) != len(message.joint_names)
            or len(message.feedback.positions) != len(message.joint_names)):
        return None
    indices = {name: index for index, name in enumerate(message.joint_names)}
    if any(name not in indices for name in names):
        return None
    target = [message.reference.positions[indices[name]] for name in names]
    actual = [message.feedback.positions[indices[name]] for name in names]
    return (target, actual) if all(map(math.isfinite, target+actual)) else None


def trajectory_curve(path):
    times = [seconds(point.time_from_start) for point in path.points]
    if (len(times) < 2 or times[0] != 0. or any(not math.isfinite(t) for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))):
        raise ValueError('测量轨迹的时间必须从零开始并严格递增')
    derivatives = []
    for point in path.points:
        values = [list(point.positions)]
        if len(point.velocities) == len(path.joint_names):
            values.append(list(point.velocities))
            if len(point.accelerations) == len(path.joint_names):
                values.append(list(point.accelerations))
        derivatives.append(values)
    return times[-1], BPoly.from_derivatives(times, derivatives)


def summarize(samples, names):
    selected = [sample for sample in samples if sample['matched']]
    result = []
    for index, name in enumerate(names):
        moving = [sample for sample in selected if sample['moving'][index]]
        errors = [math.degrees(s['target'][index]-s['actual'][index]) for s in selected]
        motion_errors = [math.degrees(s['target'][index]-s['actual'][index]) for s in moving]
        result.append({'joint': name, 'samples': len(errors), 'moving_samples': len(motion_errors),
                       'rmse_deg': math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
                       'moving_rmse_deg': math.sqrt(sum(e*e for e in motion_errors)/len(motion_errors))
                       if motion_errors else None,
                       'max_abs_error_deg': max(map(abs, errors)) if errors else None,
                       'mean_error_deg': sum(errors)/len(errors) if errors else None})
    return result


class JointTracking(Node):
    def __init__(self, client, output=None):
        super().__init__('onero_joint_tracking_'+uuid.uuid4().hex[:8])
        self.output = Path(output) if output else Path('.logs/joint_tracking')/(
            datetime.now().strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
        self.output.mkdir(parents=True, exist_ok=False)
        sides = ('left', 'right') if client.dual else (client.arm_side,)
        self.names = {side: list(LEFT_JOINTS if side == 'left' else JOINTS) for side in sides}
        self.metadata = {'example': client.args.case, 'error_definition': 'target - feedback',
                         'angle_unit': 'deg', 'modes': {}, 'sources': {}}
        self.lock = threading.Lock()
        self.origin_ns = None
        self.motions, self.current, self.latest_raw, self.latest_state = [], {}, {}, {}
        self.samples = {side: [] for side in sides}
        self.invalid = Counter()
        self.streams, self.writers = {}, {}
        self.events = (self.output/'motions.jsonl').open('x', encoding='utf-8')
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in sides:
            bridge = f'/{side}/onero_topic_bridge' if client.dual else '/onero_topic_bridge'
            values = client.call(GetParameters, bridge+'/get_parameters',
                                 GetParameters.Request(names=['simulate'])).values
            if len(values) != 1 or values[0].type != 1:
                raise MoveItError('无法确认误差采集的真机/模拟模式')
            self.metadata['modes'][side] = 'sim' if values[0].bool_value else 'real'
            controller = side+'_arm_controller' if client.dual else 'arm_controller'
            raw_topic = f'/{side}/robot_joint_states' if client.dual else '/robot_joint_states'
            state_topic = f'/{controller}/controller_state'
            self.metadata['sources'][side] = {'controller': controller, 'state': state_topic,
                                              'raw_feedback': raw_topic}
            stream = (self.output/f'{side}.csv').open('x', newline='', encoding='utf-8')
            self.streams[side] = stream
            writer = self.writers[side] = csv.writer(stream)
            writer.writerow(['time_s', 'ros_stamp_ns', 'trajectory_time_s', 'motion_id',
                             'matched_trajectory']+[
                column for index in range(1, 8) for column in (
                    f'joint{index}_target_deg', f'joint{index}_actual_deg',
                    f'joint{index}_error_deg', f'joint{index}_moving')])
            self.create_subscription(JointState, raw_topic,
                                     lambda msg, side=side: self._raw(side, msg), qos)
            self.create_subscription(JointTrajectoryControllerState, state_topic,
                                     lambda msg, side=side: self._state(side, msg), qos)
        self.create_timer(1., self._flush)
        self._recording_executor = SingleThreadedExecutor(context=self.context)
        self._recording_executor.add_node(self)
        self.worker = threading.Thread(target=self._recording_executor.spin, daemon=True)
        self.worker.start()
        print(f'关节误差采集目录：{self.output.resolve()}')

    def _flush(self):
        with self.lock:
            for stream in self.streams.values():
                stream.flush()
            self.events.flush()

    def _raw(self, side, message):
        values = dict(zip(message.name, message.position))
        if any(name not in values or not math.isfinite(values[name]) for name in self.names[side]):
            return
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        with self.lock:
            self.latest_raw[side] = (stamp, values)

    def _state(self, side, message):
        values = state_values(message, self.names[side])
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if values is None or not -.1 <= (self.get_clock().now().nanoseconds-stamp)*1e-9 <= 1.:
            self.invalid[side] += 1
            return
        target, actual = values
        with self.lock:
            if stamp <= self.latest_state.get(side, -1):
                return
            self.latest_state[side] = stamp
            if self.origin_ns is None:
                return
            trajectory_time = seconds(message.reference.time_from_start)
            motion = self.current.get(side)
            matched = False
            moving = [False]*len(self.names[side])
            if motion is not None and motion['status'] != 'failed' and trajectory_time >= 0.:
                names = motion['joint_names']
                indices = [names.index(name) for name in self.names[side]]
                expected = motion['_curve'](min(trajectory_time, motion['duration_s']))
                matched = max(abs(target[i]-expected[j]) for i, j in enumerate(indices)) < 1e-4
                if matched and motion['status'] == 'running' and trajectory_time < motion['duration_s']:
                    velocity = motion['_curve'](trajectory_time, nu=1)
                    moving = [abs(velocity[j]) > 1e-6 for j in indices]
            sample = {'stamp_ns': stamp, 'time': (stamp-self.origin_ns)*1e-9,
                      'trajectory_time': trajectory_time, 'motion': motion['id'] if motion else 0,
                      'matched': bool(matched), 'moving': moving, 'target': target, 'actual': actual}
            self.samples[side].append(sample)
            row = [sample['time'], stamp, trajectory_time, sample['motion'], int(matched)]
            for index, (q, a) in enumerate(zip(target, actual)):
                row.extend([math.degrees(q), math.degrees(a), math.degrees(q-a), int(moving[index])])
            self.writers[side].writerow(row)

    def wait_ready(self, timeout=5.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            now = self.get_clock().now().nanoseconds
            with self.lock:
                ready = all(side in self.latest_raw and side in self.latest_state
                            and -.1 <= (now-self.latest_raw[side][0])*1e-9 <= .5
                            and -.1 <= (now-self.latest_state[side])*1e-9 <= .5 for side in self.names)
            if ready:
                return
            time.sleep(.02)
        raise MoveItError('误差采集缺少新鲜的关节反馈或控制器状态，尚未开始运动')

    def begin(self, trajectory, controllers, label=None):
        path = trajectory.joint_trajectory
        arm_sides = [side for side, names in self.names.items() if set(names) <= set(path.joint_names)]
        duration, curve = trajectory_curve(path) if arm_sides else (seconds(path.points[-1].time_from_start), None)
        with self.lock:
            now = self.get_clock().now().nanoseconds
            if self.origin_ns is None:
                self.origin_ns = now
            motion = {'id': len(self.motions)+1, 'label': label or ' / '.join(controllers),
                      'controllers': list(controllers), 'joint_names': list(path.joint_names),
                      'begin_ros_ns': now, 'begin_s': (now-self.origin_ns)*1e-9,
                      'duration_s': duration, 'status': 'running', '_curve': curve,
                      'points': [{'time_s': seconds(p.time_from_start), 'positions_rad': list(p.positions),
                                  'velocities_rad_s': list(p.velocities),
                                  'accelerations_rad_s2': list(p.accelerations)} for p in path.points]}
            self.motions.append(motion)
            for side in arm_sides:
                self.current[side] = motion
            self.events.write(json.dumps({k: v for k, v in motion.items() if not k.startswith('_')},
                                         ensure_ascii=False, allow_nan=False)+'\n')
            self.events.flush()
            return motion['id']

    def end(self, identifier, succeeded, detail=''):
        with self.lock:
            motion = self.motions[identifier-1]
            now = self.get_clock().now().nanoseconds
            motion.update(status='succeeded' if succeeded else 'failed', detail=detail,
                          end_ros_ns=now, end_s=(now-self.origin_ns)*1e-9)
            endpoint = dict(zip(motion['joint_names'], motion['points'][-1]['positions_rad']))
            errors = {}
            for side, (stamp, positions) in self.latest_raw.items():
                if -.1 <= (now-stamp)*1e-9 <= .5:
                    errors.update({name: math.degrees(endpoint[name]-positions[name])
                                   for name in self.names[side] if name in endpoint})
            motion['planned_endpoint_error_deg'] = errors
            self.events.write(json.dumps({'id': identifier, 'event': 'result',
                                          **{k: motion[k] for k in ('status', 'detail', 'end_s',
                                                                   'planned_endpoint_error_deg')}},
                                         ensure_ascii=False, allow_nan=False)+'\n')
            self.events.flush()

    def close(self, succeeded):
        self._recording_executor.shutdown()
        self.worker.join()
        self._recording_executor.remove_node(self)
        self.destroy_node()
        for stream in self.streams.values():
            stream.close()
        self.events.close()
        metrics = {side: summarize(self.samples[side], names) for side, names in self.names.items()}
        report = {**self.metadata, 'example_succeeded': succeeded, 'invalid_samples': dict(self.invalid),
                  'joint_metrics': metrics, 'motions': [
                      {k: v for k, v in motion.items() if not k.startswith('_')} for motion in self.motions]}
        for side, samples in self.samples.items():
            stamps = [s['stamp_ns'] for s in samples]
            intervals = np.diff(stamps)*1e-9
            report.setdefault('sampling', {})[side] = {
                'samples': len(stamps), 'rate_hz': float(1./np.median(intervals)) if len(intervals) else None,
                'max_gap_s': float(max(intervals)) if len(intervals) else None}
        (self.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False,
                                                           indent=2, allow_nan=False)+'\n', encoding='utf-8')
        with (self.output/'metrics.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=['arm']+list(next(iter(metrics.values()))[0]))
            writer.writeheader()
            for side, rows in metrics.items():
                writer.writerows({'arm': side, **row} for row in rows)
        self._plot(succeeded)
        missing = [side for side, rows in metrics.items() if not any(row['moving_samples'] for row in rows)]
        if missing:
            print('以下手臂没有匹配到正在执行的关节轨迹，请检查采集文件：'+', '.join(missing))
        print(f'运动曲线、误差曲线及 CSV 已保存：{self.output.resolve()}')

    def _plot(self, succeeded):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        for side, samples in self.samples.items():
            if not samples:
                continue
            figure = Figure(figsize=(15, 18), constrained_layout=True)
            FigureCanvasAgg(figure)
            axes = figure.subplots(7, 2, sharex=True)
            times = np.array([s['time'] for s in samples])
            target = np.rad2deg([s['target'] for s in samples])
            actual = np.rad2deg([s['actual'] for s in samples])
            for index in range(7):
                angle, error = axes[index]
                angle.plot(times, target[:, index], '--', color='#2563eb', label='Target', linewidth=1.)
                angle.plot(times, actual[:, index], color='#ea580c', label='Feedback', linewidth=.9)
                error.plot(times, target[:, index]-actual[:, index], color='#dc2626', linewidth=.9)
                error.axhline(0., color='#64748b', linewidth=.6)
                angle.set_ylabel(f'J{index+1} angle (deg)')
                error.set_ylabel('Target - feedback (deg)')
                for axis in (angle, error):
                    axis.grid(alpha=.25)
                    for motion in self.motions:
                        if 'end_s' in motion:
                            axis.axvline(motion['end_s'], color='#dc2626' if motion['status'] == 'failed'
                                         else '#94a3b8', alpha=.4, linewidth=.6)
            for motion in self.motions:
                if 'end_s' in motion:
                    axes[0, 0].text((motion['begin_s']+motion['end_s'])/2., .98, f'#{motion["id"]}',
                                    transform=axes[0, 0].get_xaxis_transform(), va='top',
                                    ha='center', fontsize=7,
                                    color='#dc2626' if motion['status'] == 'failed' else '#475569')
            axes[0, 0].set_title('Target and feedback (# = motion ID)')
            axes[0, 1].set_title('Tracking error')
            axes[0, 0].legend(loc='best')
            axes[-1, 0].set_xlabel('Time (s)')
            axes[-1, 1].set_xlabel('Time (s)')
            status = 'completed' if succeeded else 'interrupted / failed'
            figure.suptitle(f'{side.title()} arm | {self.metadata["example"]} | '
                            f'{self.metadata["modes"][side]} | {status}')
            figure.savefig(self.output/f'{side}_tracking.png', dpi=120)
