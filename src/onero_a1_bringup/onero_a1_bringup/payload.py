"""Payload validation and control-cycle-owned, acknowledged mass transitions."""
from dataclasses import dataclass
from collections.abc import Sequence
import math
import threading


@dataclass(frozen=True)
class Payload:
    mass: float
    cog: tuple

    @classmethod
    def create(cls, mass, cog):
        if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass < 0:
            raise ValueError('payload_mass 必须为有限非负数，单位 kg')
        if (not isinstance(cog, Sequence) or len(cog) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in cog)):
            raise ValueError('payload_cog 必须是末端坐标系下的三个有限数值，单位 m')
        return cls(float(mass), tuple(map(float, cog)))


def blend_payloads(first, second, fraction):
    if fraction <= 0:
        return first
    if fraction >= 1:
        return second
    # Interpolate mass and first moment, so gravity feedforward changes
    # linearly even when starting from zero payload mass.
    mass = first.mass*(1-fraction)+second.mass*fraction
    cog = tuple((first.mass*a*(1-fraction)+second.mass*b*fraction)/mass
                for a, b in zip(first.cog, second.cog)) if mass else first.cog
    return Payload.create(mass, cog)


class PayloadTransition:
    def __init__(self, mass=0., cog=(0., 0., 0.), duration=.5):
        if (isinstance(duration, bool) or not isinstance(duration, (int, float))
                or not math.isfinite(duration) or duration <= 0):
            raise ValueError('payload_transition_time 必须为有限正数，单位 s')
        self.baseline = Payload.create(mass, cog)
        self.current = self.target = self.origin = self.baseline
        self.duration = float(duration)
        self.started = 0.
        self.revision = self.applied_revision = 0
        self.error = ''
        self.limited = False
        self._needs_apply = False
        self._lock = threading.RLock()

    def request(self, payload, now):
        with self._lock:
            if payload != self.target or self.error:
                self._needs_apply = self._needs_apply or bool(self.error)
                self.origin, self.target, self.started = self.current, payload, now
                self.revision += 1
                self.error = ''
            return self.revision

    def advance(self, setter, now):
        """Called under the bridge's hardware lock, before gravity calculation."""
        with self._lock:
            if self.error:
                raise RuntimeError('负载设置失败：'+self.error)
            if self.applied_revision == self.revision:
                return self.revision
            fraction = max(0., min(1., (now-self.started)/self.duration))
            value = blend_payloads(self.origin, self.target, fraction)
            if value != self.current or self._needs_apply:
                try:
                    setter(value.mass, list(value.cog))
                except Exception as exc:
                    self.error = str(exc)
                    self._needs_apply = True
                    raise
                self.current = value
                self._needs_apply = False
            return self.revision

    def confirm(self, revision, limited=False):
        """Acknowledge only after a successful command/feedback cycle."""
        with self._lock:
            self.limited = bool(limited)
            if revision == self.revision and self.current == self.target and not self.error:
                self.applied_revision = revision

    def snapshot(self):
        with self._lock:
            return {'baseline_mass': self.baseline.mass, 'baseline_cog': list(self.baseline.cog),
                    'mass': self.current.mass, 'cog': list(self.current.cog),
                    'target_mass': self.target.mass, 'target_cog': list(self.target.cog),
                    'revision': self.revision, 'applied_revision': self.applied_revision,
                    'transitioning': self.applied_revision != self.revision,
                    'transition_time': self.duration, 'limited': self.limited, 'error': self.error}


def payload_launch_arguments(dual=False):
    from launch.actions import DeclareLaunchArgument
    result = []
    for prefix in (('left_', 'right_') if dual else ('',)):
        result.extend([
            DeclareLaunchArgument(prefix+'payload_mass', default_value='0.0',
                                  description='空夹爪基线的额外负载质量，kg'),
            DeclareLaunchArgument(prefix+'payload_cog', default_value='[0.0, 0.0, 0.0]',
                                  description='基线负载重心，SDK 末端坐标系，m'),
        ])
    result.append(DeclareLaunchArgument('payload_transition_time', default_value='0.5',
                                        description='运行时负载平滑切换时长，s'))
    return result
