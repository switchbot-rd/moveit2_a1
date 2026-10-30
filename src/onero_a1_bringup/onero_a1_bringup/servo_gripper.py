"""One cancellable GripperCommand action for each held open/close key."""
import threading
import math
from control_msgs.action import GripperCommand


class ServoGripper:
    def __init__(self, client):
        self.client = client
        self.lock = threading.RLock()
        self.direction = self.generation = 0
        self.attempted = self.goal_generation = -1
        self.pending = self.goal = None
        self.canceling = False
        self.error = ''
        self.holding = False
        self.preload_target = None
        self.goal_is_preload = False
        self.goal_direction = 0

    @property
    def idle(self):
        with self.lock:
            return self.pending is None and self.goal is None

    def set_direction(self, direction):
        with self.lock:
            if direction != self.direction:
                self.direction = direction
                self.generation += 1
                if direction:
                    self.preload_target = None
                if direction > 0:
                    self.holding = False
            self._advance()

    def _advance(self):
        if self.pending is not None:
            return
        if self.goal is not None:
            if (self.goal_generation != self.generation and not self.canceling
                    and not (self.goal_is_preload and self.direction == 0)):
                self.canceling = True
                self.goal.cancel_goal_async()
            return
        preload = self.preload_target
        if preload is None and (not self.direction or self.attempted == self.generation):
            return
        self.attempted = self.goal_generation = self.generation
        self.error = ''
        if not self.client.server_is_ready():
            self.error = '夹爪接口未就绪'
            return
        goal = GripperCommand.Goal()
        self.goal_is_preload = preload is not None
        self.goal_direction = self.direction
        self.preload_target = None
        goal.command.position = preload if self.goal_is_preload else 1.1 if self.direction > 0 else 0.
        goal.command.max_effort = 0.
        self.pending = self.client.send_goal_async(goal)
        self.pending.add_done_callback(self._accepted)

    def _accepted(self, future):
        with self.lock:
            self.pending = None
            try:
                goal = future.result()
                if not goal.accepted:
                    self.error = '夹爪拒绝动作，请检查设备状态或动作占用'
                    self._advance()
                    return
                self.goal = goal
                goal.get_result_async().add_done_callback(self._finished)
                self._advance()  # A release/reversal may precede goal acceptance.
            except Exception as exc:
                self.error = '夹爪动作失败：'+str(exc)

    def _finished(self, future):
        with self.lock:
            try:
                result = future.result()
                contact = result.result
                closing = self.goal_direction < 0 and not self.goal_is_preload
                if (closing and self.direction <= 0 and (contact.stalled or result.status == 5)
                        and math.isfinite(contact.position) and 0. < contact.position < 1.1):
                    # Same small position preload as the validated pick/place task.
                    # Releasing a key must not remove the closing position error.
                    self.preload_target = max(0., contact.position-.005)
                elif self.goal_is_preload and result.status == 4 and contact.reached_goal:
                    self.holding = True
                elif closing and result.status == 4 and contact.position <= .015:
                    self.error = '夹爪全闭，未检测到接触'
                elif result.status not in (4, 5):
                    self.error = '夹爪受阻，保持开度' if result.result.stalled else '夹爪动作中止'
            except Exception as exc:
                self.error = '夹爪反馈失败：'+str(exc)
            self.goal, self.canceling = None, False
            self._advance()


class ServoGripperGroup:
    """Route held-key commands to the selected side, preserving inactive grips."""
    def __init__(self, grippers):
        self.grippers=grippers
        self.target='right'
        self.direction=0

    def set_target(self,target):
        self.set_direction(0)
        self.target=target

    def set_direction(self,direction):
        self.direction=direction
        for side,gripper in self.grippers.items():
            gripper.set_direction(direction if self.target in (side,'both') else 0)

    @property
    def idle(self):return all(g.idle for g in self.grippers.values())
    @property
    def holding(self):
        return all(g.holding for side,g in self.grippers.items() if self.target in (side,'both'))
    @property
    def error(self):
        return '；'.join(side+': '+g.error for side,g in self.grippers.items() if g.error and self.target in (side,'both'))
