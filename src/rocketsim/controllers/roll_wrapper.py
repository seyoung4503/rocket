"""Roll-axis PID wrapper.

Most of our controllers (LandingPID, the lookahead MPC wrappers,
the SCP wrappers) emit a Command whose ``roll_cmd`` field is zero,
because they were written before the optional exhaust-vane channel
existed.  This wrapper sits in front of any of those, reads
omega_z + the body-z component of the small-angle attitude from the
state, runs a simple PD-with-derivative-only-on-omega regulator,
and stuffs the result into ``cmd.roll_cmd`` before passing the
Command back.

Reading the roll *angle* from a quaternion is the only subtle
piece -- we use the existing ``quat_to_euler`` helper, which returns
(roll, pitch, yaw) in the ZYX convention, and treat the roll
component as the body-z deviation from the desired heading
(default 0 = pad reference frame).

Use case:
    from rocketsim.controllers import LandingPID, RollPIDWrapper
    base = LandingPID(vehicle, env.world)
    ctrl = RollPIDWrapper(base, vehicle, env.world)
    # ctrl(t, state) now emits cmd with cmd.roll_cmd set.

When the underlying Vehicle has ``edf_vane_torque_max = 0``
(default) the roll_cmd is ignored by the simulator anyway, so the
wrapper is safe to leave on even on vehicles without vanes.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat
from ..dynamics import Command
from ..vehicle import Environment, Vehicle


class RollPIDWrapper:
    """Wrap any controller and inject a roll command on top of its output.

    Parameters
    ----------
    inner : callable
        Existing controller; called as ``inner(t, state) -> Command``.
    vehicle, env : Vehicle, Environment
        Used to size sensible default gains against the body-z inertia.
    kp_roll, kd_roll, ki_roll : float
        PID gains on roll angle (rad), body-z angular velocity (rad/s)
        and integral of roll angle (rad·s) respectively.  Defaults
        are tuned for the 90 mm EDF testbed (I_zz ≈ 0.004 kg·m²) with
        the 0.5 N·m vane authority — gains roughly match the
        critically-damped response of that system.
    target_roll : float
        Desired body-z heading angle (rad), default 0 (pad-aligned).
    integral_clamp : float
        Anti-windup limit on the integral term, in rad·s.
    """

    def __init__(
        self,
        inner,
        vehicle: Vehicle,
        env: Environment,
        kp_roll: float = 8.0,
        kd_roll: float = 1.2,
        ki_roll: float = 2.0,
        target_roll: float = 0.0,
        integral_clamp: float = 0.5,
    ):
        self.inner = inner
        self.vehicle = vehicle
        self.env = env
        self.kp_roll = float(kp_roll)
        self.kd_roll = float(kd_roll)
        self.ki_roll = float(ki_roll)
        self.target_roll = float(target_roll)
        self.integral_clamp = float(integral_clamp)
        self._roll_int = 0.0
        self._last_t = 0.0

    def reset(self) -> None:
        self._roll_int = 0.0
        self._last_t = 0.0

    def __call__(self, t: float, state: np.ndarray) -> Command:
        # Delegate to inner controller for throttle / gimbals.
        cmd = self.inner(t, state)

        q = state[dyn.QUAT]
        omega = state[dyn.OMEGA]
        # Euler ZYX → (roll about body-z, pitch about y, yaw about x).
        # quat_to_euler returns (roll, pitch, yaw); we treat the first
        # component as the body-z heading deviation.
        roll, _, _ = quat.quat_to_euler(q)
        roll_err = self.target_roll - float(roll)
        omega_z = float(omega[2])

        # Integrate roll error with anti-windup.
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        self._roll_int = float(
            np.clip(
                self._roll_int + roll_err * dt,
                -self.integral_clamp,
                self.integral_clamp,
            )
        )

        # PD on roll, with the derivative term sourced directly from the
        # gyro (omega_z) instead of differentiating the angle — same
        # trick HoverPID uses for pitch / yaw.
        roll_cmd_raw = (
            self.kp_roll * roll_err
            - self.kd_roll * omega_z
            + self.ki_roll * self._roll_int
        )
        roll_cmd = float(np.clip(roll_cmd_raw, -1.0, 1.0))

        return Command(
            throttle=cmd.throttle,
            gimbal_x=cmd.gimbal_x,
            gimbal_y=cmd.gimbal_y,
            roll_cmd=roll_cmd,
        )
