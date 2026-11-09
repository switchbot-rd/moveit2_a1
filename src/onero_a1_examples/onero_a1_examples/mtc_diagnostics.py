"""Observe MTC's native statistics without changing its search or cost terms."""
import os
import re
import socket
import threading
import time
import uuid


class PlanningStatistics:
    def __init__(self, name, *, pid=None, hostname=None):
        self.name, self.pid = name, os.getpid() if pid is None else pid
        hostname = socket.gethostname() if hostname is None else hostname
        self.prefix = re.sub(r'[^A-Za-z0-9_/]', '_', hostname)+f'_{self.pid}_'
        self.task_id, self.root = None, None
        self.stages, self.latest, self.first = {}, {}, {}

    def local(self, task_id):
        return re.fullmatch(re.escape(self.prefix)+r'\d+', task_id) is not None

    def description(self, message):
        if not self.local(message.task_id):
            return
        if not message.stages:
            self.latest.pop(message.task_id, None)
            self.first.pop(message.task_id, None)
            if self.task_id == message.task_id:
                self.task_id, self.root, self.stages = None, None, {}
            return
        root = next((stage for stage in message.stages
                     if stage.parent_id == 0 and stage.name == self.name), None)
        if root is not None:
            self.task_id, self.root = message.task_id, root.id
            self.stages = {stage.id: stage for stage in message.stages}

    def statistics(self, message, elapsed):
        if not self.local(message.task_id):
            return
        self.latest[message.task_id] = message
        first = self.first.setdefault(message.task_id, {})
        for stage in message.stages:
            if stage.solved:
                first.setdefault(stage.id, elapsed)

    @property
    def first_solution_s(self):
        return self.first.get(self.task_id, {}).get(self.root)

    def report(self):
        message = self.latest.get(self.task_id)
        rows, solutions = [], 0
        parents = {stage.parent_id for stage in self.stages.values()}
        for stat in message.stages if message is not None else ():
            stage = self.stages.get(stat.id)
            if stat.id == self.root:
                solutions = len(stat.solved)
            elif stage is not None and (stat.id not in parents or stage.name.endswith(' IK')):
                rows.append(dict(name=stage.name, seconds=stat.total_compute_time,
                                 solved=len(stat.solved), failed=stat.num_failed))
        return dict(first_solution_s=self.first_solution_s, solutions=solutions,
                    stages=sorted(rows, key=lambda row: row['seconds'], reverse=True))


class PlanningDiagnostics:
    def __init__(self, name, started):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from moveit_task_constructor_msgs.msg import TaskDescription, TaskStatistics

        self.data, self.started = PlanningStatistics(name), started
        self.stopped = threading.Event()
        self.error = None
        self.node = rclpy.create_node('onero_mtc_timing_'+uuid.uuid4().hex[:8])
        self.executor = None
        try:
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.node.create_subscription(TaskDescription, '/mtc_pick_place/description', self.data.description, qos)
            self.node.create_subscription(TaskStatistics, '/mtc_pick_place/statistics',
                lambda message: self.data.statistics(message, time.monotonic()-started), qos)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self._receive, name='mtc-statistics', daemon=True)
            self.thread.start()
        except BaseException:
            try:
                if self.executor is not None:
                    self.executor.shutdown()
            finally:
                self.node.destroy_node()
            raise

    def _receive(self):
        try:
            while not self.stopped.is_set():
                self.executor.spin_once(timeout_sec=.02)
            # Consume the final native statistic, including a solution found
            # immediately before plan() returned. Do not delay motion indefinitely.
            deadline = time.monotonic()+.05
            while time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=.005)
        except Exception as exc:
            self.error = str(exc)

    def close(self, elapsed):
        self.stopped.set()
        try:
            self.thread.join()
        finally:
            try:
                self.executor.shutdown()
            finally:
                self.node.destroy_node()
        report = self.data.report()
        first = report['first_solution_s']
        if first is not None:
            print(f'MTC 首条完整方案观测于 {first:.2f} s，随后继续搜索约 '
                  f'{max(0., elapsed-first):.2f} s（按统计消息接收时间）。')
        if report['stages']:
            print('MTC 阶段累计计算（含各自子阶段，耗时不相加）：')
            for stage in report['stages'][:6]:
                print(f'  {stage["name"]}：{stage["seconds"]:.2f} s，'
                      f'成功 {stage["solved"]}，失败 {stage["failed"]}。')
        else:
            print('MTC 本次未收到匹配的阶段统计，以上总规划耗时仍有效。')
        if self.error:
            print(f'MTC 阶段统计中断：{self.error}')
        return report
