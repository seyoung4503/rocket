"""Attitude controller: thrust-acceleration vector → (throttle, gimbal_x,
gimbal_y).

Separated from the planner / tracker so each layer has a single
responsibility:

  * **Planner (MPC)** decides where the body should go.
  * **Tracker** decides what thrust acceleration would close the error.
  * **Attitude controller** (this file) decides how to point the gimbal
    so the body's thrust vector aligns with the requested direction.

The algorithm is a small-angle PID on the alignment error between the
current body-z (world frame) and the desired thrust direction, mapped
to a gimbal angle through the engine offset / lever arm.  It is the same
algorithm as the existing ``_command_from_thrust_accel`` helper, extracted
into a clean class so the SpaceX stack has no implicit dependency on the
old MPC class hierarchy.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat
from ..dynamics import Command
from ..vehicle import Environment, Vehicle


class AttitudeController:
    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        max_tilt: float = np.deg2rad(20.0),
        kp_att: float = 130.0,
        kd_att: float = 24.0,
        ki_att: float = 55.0,
    ):
        self.vehicle = vehicle
        self.env = env
        self.max_tilt = float(max_tilt)
        self.kp_att = float(kp_att)
        self.kd_att = float(kd_att)
        self.ki_att = float(ki_att)
        self._att_int = np.zeros(3)
        self._last_t = 0.0

    def reset(self, t: float = 0.0) -> None:
        self._att_int[:] = 0.0
        self._last_t = float(t)

    def command(self, t: float, state: np.ndarray, a_des: np.ndarray) -> Command:
        v = self.vehicle
        q = state[dyn.QUAT]
        omega = state[dyn.OMEGA]
        a_des = np.asarray(a_des, dtype=float)

        # Desired body-z direction (= unit vector of desired thrust).
        f_des = v.mass * a_des
        f_mag = float(np.linalg.norm(f_des))
        if f_mag < 1e-6:
            z_des = np.array([0.0, 0.0, 1.0])
            throttle = 0.0
        else:
            z_des = f_des / f_mag
            tilt = float(np.arccos(np.clip(z_des[2], -1.0, 1.0)))
            if tilt > self.max_tilt:
                horiz = z_des[:2]
                horiz_norm = float(np.linalg.norm(horiz))
                if horiz_norm > 1e-9:
                    horiz = horiz / horiz_norm * np.sin(self.max_tilt)
                z_des = np.array([horiz[0], horiz[1], np.cos(self.max_tilt)])
            throttle = float(np.clip(f_mag / v.max_thrust, 0.0, 1.0))

        # Attitude error (body-z alignment).
        dt = max(float(t) - self._last_t, 0.0)
        self._last_t = float(t)
        bz_world = quat.quat_rotate(q, np.array([0.0, 0.0, 1.0]))
        err_world = np.cross(bz_world, z_des)
        R = quat.quat_to_rotmat(q)
        err_body = R.T @ err_world

        self._att_int += err_body * dt
        self._att_int = np.clip(self._att_int, -0.5, 0.5)
        torque_des = v.inertia @ (
            self.kp_att * err_body - self.kd_att * omega + self.ki_att * self._att_int
        )

        # Map torque to gimbal (clip).
        thrust = max(throttle * v.max_thrust, 0.25 * v.max_thrust)
        lever = v.engine_offset * thrust
        gx = float(np.clip(-torque_des[0] / lever, -v.gimbal_limit, v.gimbal_limit))
        gy = float(np.clip(-torque_des[1] / lever, -v.gimbal_limit, v.gimbal_limit))
        return Command(throttle=throttle, gimbal_x=gx, gimbal_y=gy)
