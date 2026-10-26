"""Own one SDK Arm; leave gravity, mounting and joint safety to the SDK.

The transport opens only on enable(), after SDK/dynamics configuration has
been checked. This module is imported only in real-hardware mode.
"""

import math

from onero_sdk import Arm
from onero_sdk.backend import ArmBackend
from onero_sdk.can_backend import CanBackend
from onero_sdk.model_registry import get_model
from onero_sdk.safety import SafetyConfig
from onero_sdk.zero_calibration import load_zero_offset, validate_offsets
from onero_sdk.zero_offset_backend import ZeroOffsetBackend
from .can_ownership import ChannelOwnership
from .payload import Payload


# Gripper register/MIT replies share the arm's transport and lock, but need
# their own response budget rather than the arm's 3 ms batch deadline.
GRIPPER_FEEDBACK_TIMEOUT = .01


class _DeferredBackend(ArmBackend):
    """Lazy CAN ownership plus observation of commands after SDK limiting."""

    def __init__(self, model, channel, max_ff_torque, on_miss, on_fault, factory, gripper=None):
        self._model = model
        self._channel = channel
        self._max_ff_torque = max_ff_torque
        self._on_miss = on_miss
        self._on_fault = on_fault
        self._factory = factory
        self._inner = None
        self._gripper = gripper
        self.last_command = None
        self._ownership = ChannelOwnership(channel)

    @property
    def num_joints(self):
        return self._model.num_joints

    def enable(self):
        if self._inner is None:
            self._ownership.acquire()
            try:
                self._inner = self._factory(
                    channel=self._channel, joint_specs=self._model.joints,
                    recv_timeout=0.003, on_miss=self._on_miss, on_fault=self._on_fault,
                )
                if self._gripper is not None:
                    self._inner.bind_can_device(self._gripper)
            except BaseException:
                try:
                    if self._inner is not None:
                        self._inner.close(disable=False)
                finally:
                    self._inner = None
                    self._ownership.close()
                raise
        return self._inner.enable()

    def disable(self):
        if self._inner is not None:
            self._inner.disable()

    def command(self, positions, velocities, kps, kds, torques):
        if self._inner is None:
            raise RuntimeError("CAN transport has not been enabled")
        # Preserve the bridge's existing feedforward ceiling in addition to
        # the SDK's model-specific limits. This does not limit the motor's PD output.
        limited = [max(-self._max_ff_torque, min(self._max_ff_torque, t)) for t in torques]
        self.last_command = {
            "raw_position": list(positions), "velocity": list(velocities),
            "kp": list(kps), "kd": list(kds), "torque_ff": limited,
            "sdk_torque_ff": list(torques),
            "bridge_ff_limited": limited != list(torques),
        }
        return self._inner.command(positions, velocities, kps, kds, limited)

    def command_velocity(self, velocities):
        raise NotImplementedError("The ROS bridge uses Arm.servoj() only")

    def command_torque(self, torques):
        raise NotImplementedError("The ROS bridge uses Arm.servoj() only")

    def get_state(self):
        if self._inner is None:
            raise RuntimeError("CAN transport has not been enabled")
        return self._inner.get_state()

    def close(self, disable=False):
        try:
            if self._inner is not None:
                self._inner.close(disable=disable)
                self._inner = None
        finally:
            self._ownership.close()


class SDKArmSession:
    def __init__(self, *, model_name, channel, mounting, with_gripper,
                 gravity_compensation, control_rate, max_gravity_torque,
                 on_miss=None, on_fault=None, backend_factory=CanBackend,
                 gripper_model="none", payload_mass=0., payload_cog=(0., 0., 0.)):
        self.model = get_model(model_name)
        self._gravity = gravity_compensation
        self._requested_torque_ff = None
        if not math.isfinite(max_gravity_torque) or max_gravity_torque <= 0:
            raise ValueError("max_gravity_torque must be positive and finite")
        if not math.isfinite(control_rate) or control_rate <= 0:
            raise ValueError("control_rate must be positive and finite")

        # An injected backend has no channel in Arm's constructor. Load via
        # the SDK helper here so channel-specific calibration is still honored.
        self.offsets = load_zero_offset(self.model.name, channel, self.model.num_joints)
        validate_offsets(self.offsets, self.model.joints)
        self.gripper = None
        if gripper_model == "woan_picker":
            from onero_sdk.grippers import OneroGripper
            self.gripper = OneroGripper(model=model_name, feedback_timeout=GRIPPER_FEEDBACK_TIMEOUT)
        elif gripper_model != "none":
            raise ValueError(f"Unsupported gripper_model: {gripper_model}")
        self._transport = _DeferredBackend(
            self.model, channel, max_gravity_torque, on_miss, on_fault, backend_factory,
            gripper=self.gripper,
        )
        calibrated = ZeroOffsetBackend(self._transport, self.offsets)
        safety = SafetyConfig(
            control_freq=control_rate,
            # JTC owns tracking-error stopping. Do not introduce the SDK's
            # additional tracking watchdog that disables a loaded arm.
            # Joint/rate/velocity/torque limiting stays enabled in SafeBackend;
            # CAN faults keep the ROS bridge's explicit log/hold/disable policy.
            tracking_error_hard_threshold=0.0,
            tracking_error_soft_threshold=0.0,
        )
        self.arm = Arm(
            model=model_name, backend=calibrated, mounting=mounting,
            with_gripper=with_gripper, safety=safety,
        )
        self.set_payload(payload_mass, payload_cog)
        if self._gravity:
            # Public SDK API: load dynamics and validate mounting/profile before
            # opening CAN or enabling a motor. Unlike servoj's internal fallback,
            # this call raises if the SDK dynamics dependency cannot be loaded.
            self._checked_gravity([0.0] * self.model.num_joints)

    def _checked_gravity(self, positions=None):
        torques = list(self.arm.gravity_torque(positions))
        if len(torques) != self.model.num_joints or not all(map(math.isfinite, torques)):
            raise RuntimeError("SDK gravity_torque returned invalid torques")
        return torques

    def set_payload(self, mass, cog):
        payload = Payload.create(mass, cog)
        self.arm.set_payload(payload.mass, list(payload.cog))
        self.payload = payload

    @property
    def last_command(self):
        command = self._transport.last_command
        if command is None:
            return None
        return {
            **command,
            "position": [p - o for p, o in zip(command["raw_position"], self.offsets)],
            "requested_torque_ff": self._requested_torque_ff,
            "sdk_ff_limited": (any(abs(a - b) > 1e-9 for a, b in
                                   zip(self._requested_torque_ff, command["sdk_torque_ff"]))
                               if self._requested_torque_ff is not None else None),
        }

    def enable(self):
        state = self.arm.enable()
        if (len(state.joints) != self.model.num_joints
                or not all(map(math.isfinite, state.joints))):
            raise RuntimeError("SDK enable did not return finite measured joint positions")
        return state

    def servoj(self, positions, kp, kd, velocities):
        self._transport.last_command = None
        self._requested_torque_ff = None
        # Evaluate at the SDK's latest measured pose, not at the desired target.
        # Explicitly reading this public SDK API also prevents silent zero-FF
        # fallback after a dynamics error. All dynamics remain inside the SDK.
        torques = self._checked_gravity() if self._gravity else [0.0] * self.model.num_joints
        self._requested_torque_ff = list(torques)
        return self.arm.servoj(positions, kp=kp, kd=kd, velocities=velocities, tau_ff=torques)

    def freedrive_step(self, damping):
        if not self._gravity:
            raise RuntimeError('Freedrive requires gravity compensation')
        self._transport.last_command = None
        self._requested_torque_ff = None
        return self.arm.freedrive_step(damping=damping)

    def hold_current(self, kp, kd):
        # This public SDK hold operation sends current-pose gravity feedforward
        # and reconciles its rate limiter after externally guided movement.
        self.arm.set_gains(kp, kd)
        self.arm.force_mode_disable()
        return self.arm.state

    def disable(self):
        self.arm.disable()

    def close(self, disable=False):
        self.arm.close(disable=disable)
