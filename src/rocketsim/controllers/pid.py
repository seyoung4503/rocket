"""Cascaded PID hover/landing controller (the baseline to beat with RL).

Structure (outer -> inner):
  1. Position/altitude PD  ->  desired world thrust vector F_des
  2. throttle             = |F_des| / max_thrust
  3. desired body-z axis  = F_des / |F_des|
  4. Attitude PD on the body-z alignment error -> desired body torque
  5. gimbal angles        = inverse of (gimbal torque model)

Roll about the body z-axis is *not* controllable with a single gimbal, so this
controller only stabilizes tilt (pitch/yaw), which is the relevant degree of
freedom for an upright EDF hover testbed.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat
from ..dynamics import Command
from ..vehicle import Environment, Vehicle


class HoverPID:
    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        target=(0.0, 0.0, 5.0),
        # position (horizontal) gains
        kp_pos: float = 1.2,
        kd_pos: float = 2.0,
        # altitude gains
        kp_z: float = 6.0,
        kd_z: float = 4.0,
        ki_z: float = 1.5,
        # attitude gains
        kp_att: float = 80.0,
        kd_att: float = 16.0,
        max_tilt: float = np.deg2rad(20.0),
    ):
        self.v = vehicle
        self.env = env
        self.target = np.asarray(target, dtype=float)
        self.kp_pos, self.kd_pos = kp_pos, kd_pos
        self.kp_z, self.kd_z, self.ki_z = kp_z, kd_z, ki_z
        self.kp_att, self.kd_att = kp_att, kd_att
        self.max_tilt = max_tilt
        self._z_int = 0.0
        self._last_t = 0.0

    def __call__(self, t: float, state: np.ndarray) -> Command:
        v = self.v
        pos = state[dyn.POS]
        vel = state[dyn.VEL]
        q = state[dyn.QUAT]
        omega = state[dyn.OMEGA]

        dt = max(t - self._last_t, 0.0)
        self._last_t = t

        # --- Outer loop: desired world-frame thrust vector ---
        pos_err = self.target - pos
        # altitude with integral term
        self._z_int += pos_err[2] * dt
        self._z_int = float(np.clip(self._z_int, -5.0, 5.0))
        a_z = self.kp_z * pos_err[2] - self.kd_z * vel[2] + self.ki_z * self._z_int

        a_xy = self.kp_pos * pos_err[:2] - self.kd_pos * vel[:2]

        f_des = v.mass * np.array([a_xy[0], a_xy[1], a_z + self.env.gravity])

        # Clamp the commanded lean so the vehicle never tips past max_tilt.
        f_mag = np.linalg.norm(f_des)
        if f_mag < 1e-6:
            f_des = np.array([0.0, 0.0, v.mass * self.env.gravity])
            f_mag = np.linalg.norm(f_des)
        z_des = f_des / f_mag
        tilt = np.arccos(np.clip(z_des[2], -1.0, 1.0))
        if tilt > self.max_tilt:
            # blend toward vertical to respect the tilt budget
            horiz = z_des[:2]
            horiz_norm = np.linalg.norm(horiz)
            if horiz_norm > 1e-9:
                horiz = horiz / horiz_norm * np.sin(self.max_tilt)
            z_des = np.array([horiz[0], horiz[1], np.cos(self.max_tilt)])

        throttle = float(np.clip(f_mag / v.max_thrust, 0.0, 1.0))

        # --- Inner loop: attitude PD on body-z alignment ---
        bz_world = quat.quat_rotate(q, np.array([0.0, 0.0, 1.0]))
        err_world = np.cross(bz_world, z_des)  # ~ axis * sin(angle), world frame
        R = quat.quat_to_rotmat(q)
        err_body = R.T @ err_world
        torque_des = v.inertia @ (self.kp_att * err_body - self.kd_att * omega)

        # --- Map desired torque to gimbal angles ---
        # torque_x ≈ -L*T*gx,  torque_y ≈ -L*T*gy   (small-angle gimbal model)
        thrust = max(throttle * v.max_thrust, 0.25 * v.max_thrust)
        lever = v.engine_offset * thrust
        gx = -torque_des[0] / lever
        gy = -torque_des[1] / lever
        gx = float(np.clip(gx, -v.gimbal_limit, v.gimbal_limit))
        gy = float(np.clip(gy, -v.gimbal_limit, v.gimbal_limit))

        return Command(throttle=throttle, gimbal_x=gx, gimbal_y=gy)
