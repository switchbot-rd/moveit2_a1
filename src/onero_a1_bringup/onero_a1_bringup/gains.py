"""Control-cycle-owned kp/kd changes with acknowledgement."""
import math
import threading


def validate_gain(name, value):
    maximum = {'kp': 500., 'kd': 5.}[name]
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 0. <= value <= maximum):
        raise ValueError(f'{name} 必须是 [0, {maximum:g}] 范围内的有限数值')
    return float(value)


class GainTransition:
    def __init__(self, kp, kd, duration=.5):
        self.value = self.origin = self.target = self.applied = {
            'kp': validate_gain('kp', kp), 'kd': validate_gain('kd', kd)}
        self.duration, self.started = duration, 0.
        self.revision = self.applied_revision = 0
        self.stamp_ns = 0
        self._lock = threading.RLock()

    def request(self, values, now):
        updates = {name: validate_gain(name, value) for name, value in values.items()}
        with self._lock:
            value = {**self.target, **updates}
            if value != self.target:
                self.origin, self.target, self.started = self.value, value, now
                self.revision += 1

    def advance(self, now):
        with self._lock:
            fraction = max(0., min(1., (now-self.started)/self.duration))
            smooth = fraction*fraction*(3.-2.*fraction)
            self.value = {name: self.origin[name]+(self.target[name]-self.origin[name])*smooth
                          for name in ('kp', 'kd')}
            if fraction == 1.:
                self.value = dict(self.target)
            return dict(self.value), self.revision

    def confirm(self, gains, revision, stamp_ns):
        """Called only after the SDK command and feedback cycle succeeded."""
        with self._lock:
            self.applied, self.stamp_ns = dict(gains), stamp_ns
            if revision == self.revision and gains == self.target:
                self.applied_revision = revision

    def snapshot(self):
        with self._lock:
            return {**self.applied, 'target_kp': self.target['kp'], 'target_kd': self.target['kd'],
                    'revision': self.revision, 'applied_revision': self.applied_revision,
                    'transitioning': self.applied_revision != self.revision,
                    'transition_time': self.duration, 'stamp_ns': self.stamp_ns}
