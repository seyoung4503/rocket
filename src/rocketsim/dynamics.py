"""6-DOF rigid-body dynamics for a gimbaled-thrust vehicle.

State (flat vector, length 14):
    [0:3]   position in world frame      (m)
    [3:6]   velocity in world frame      (m/s)
    [6:10]  attitude quaternion [w,x,y,z] (body -> world)
    [10:13] angular velocity in body frame (rad/s)
    [13]    actual thrust magnitude       (N)   -- lags the commanded value

Command:
    throttle in [0, 1], gimbal_x (rad), gimbal_y (rad)
The gimbal deflects the thrust vector away from the body +z axis.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import quaternion as quat
from .vehicle import Environment, Vehicle

POS = slice(0, 3)
VEL = slice(3, 6)
QUAT = slice(6, 10)
OMEGA = slice(10, 13)
THRUST = 13
STATE_DIM = 14


@dataclass
class Command:
    throttle: float = 0.0  # [0, 1]
    gimbal_x: float = 0.0  # rad, deflection about body x-axis
    gimbal_y: float = 0.0  # rad, deflection about body y-axis
    # Optional fourth channel — roll vane deflection in [-1, 1].
    # Defaults to 0 so existing controllers stay backwards compatible.
    # Only takes effect when Vehicle.edf_vane_torque_max > 0.
    roll_cmd: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array(
            [self.throttle, self.gimbal_x, self.gimbal_y, self.roll_cmd]
        )


def initial_state(
    position=(0.0, 0.0, 0.0),
    velocity=(0.0, 0.0, 0.0),
    quaternion=(1.0, 0.0, 0.0, 0.0),
    omega=(0.0, 0.0, 0.0),
    thrust=0.0,
) -> np.ndarray:
    s = np.zeros(STATE_DIM)
    s[POS] = position
    s[VEL] = velocity
    s[QUAT] = quat.quat_normalize(np.asarray(quaternion, dtype=float))
    s[OMEGA] = omega
    s[THRUST] = thrust
    return s


def thrust_direction_body(gimbal_x: float, gimbal_y: float) -> np.ndarray:
    """Unit thrust direction in body frame: Rx(gx) @ Ry(gy) @ [0,0,1]."""
    cx, sx = np.cos(gimbal_x), np.sin(gimbal_x)
    cy, sy = np.cos(gimbal_y), np.sin(gimbal_y)
    return np.array([sy, -sx * cy, cx * cy])


def state_derivative(
    state: np.ndarray,
    cmd: np.ndarray,
    vehicle: Vehicle,
    env: Environment,
    wind: np.ndarray | None = None,
    external_force: np.ndarray | None = None,
) -> np.ndarray:
    """Continuous-time derivative of the state given a (clamped) command.

    ``wind`` (world-frame air velocity, m/s) makes drag act on the airspeed
    relative to the moving air. ``external_force`` (world frame, N) is an extra
    disturbance (e.g. a gust load). Both default to zero.
    """
    wind = np.zeros(3) if wind is None else wind
    external_force = np.zeros(3) if external_force is None else external_force

    q = state[QUAT]
    v_world = state[VEL]
    omega = state[OMEGA]
    thrust = state[THRUST]

    throttle = float(np.clip(cmd[0], 0.0, 1.0))
    # Constant thrust/CG misalignment adds to the commanded gimbal (unknown to
    # the controller), then the physical gimbal limit clamps the total.
    gx = float(np.clip(cmd[1] + vehicle.thrust_misalign[0], -vehicle.gimbal_limit, vehicle.gimbal_limit))
    gy = float(np.clip(cmd[2] + vehicle.thrust_misalign[1], -vehicle.gimbal_limit, vehicle.gimbal_limit))
    # Optional 4th command channel: roll vane deflection in [-1, 1].
    # Older controllers emit a 3-element array; default to zero so the
    # vane torque term below is a no-op when no roll command is given.
    roll_cmd = float(np.clip(cmd[3], -1.0, 1.0)) if len(cmd) > 3 else 0.0

    # --- Forces ---
    dir_body = thrust_direction_body(gx, gy)
    f_thrust_body = thrust * dir_body
    f_thrust_world = quat.quat_rotate(q, f_thrust_body)

    f_gravity_world = np.array([0.0, 0.0, -vehicle.mass * env.gravity])

    # Drag acts on airspeed = ground velocity - wind. Use the larger side area
    # for the crosswind component so wind is actually felt at low speed.
    v_air = v_world - wind
    air_speed = np.linalg.norm(v_air)
    if air_speed > 1e-6:
        area = vehicle.ref_area + (vehicle.side_area - vehicle.ref_area) * (
            np.linalg.norm(v_air[:2]) / air_speed
        )
        drag_mag = 0.5 * env.air_density * vehicle.drag_coeff * area * air_speed * air_speed
        f_drag_world = -drag_mag * (v_air / air_speed)
    else:
        f_drag_world = np.zeros(3)

    accel = (f_thrust_world + f_gravity_world + f_drag_world + external_force) / vehicle.mass

    # --- Torque (body frame) ---
    r_engine = np.array([0.0, 0.0, -vehicle.engine_offset])
    torque_body = np.cross(r_engine, f_thrust_body)

    # --- Optional EDF roll physics (see docs/2026-05-29_2108) ---
    if vehicle.edf_roll_coeff > 0.0:
        # Fan reaction torque: spinning fan pushes air down, body feels
        # Newton's 3rd-law reaction about body-z, opposing fan spin.
        torque_body = torque_body + np.array(
            [0.0, 0.0, -vehicle.edf_roll_coeff * thrust]
        )
    if vehicle.edf_fan_inertia > 0.0 and vehicle.edf_fan_omega_max > 0.0:
        # Gyroscopic precession: tau_gyro = -omega_body × H_fan_body.
        # Fan spin scales with sqrt(thrust / max_thrust) because thrust
        # ∝ omega_fan^2 in the basic momentum-disk model.
        omega_fan = vehicle.edf_fan_omega_max * np.sqrt(
            max(thrust, 0.0) / vehicle.max_thrust
        )
        H_fan_body = np.array(
            [0.0, 0.0, vehicle.edf_fan_inertia * omega_fan]
        )
        torque_body = torque_body - np.cross(omega, H_fan_body)

    # --- Optional exhaust-vane roll control ---
    # Authority scales with current thrust (exhaust momentum flux).
    if vehicle.edf_vane_torque_max > 0.0 and abs(roll_cmd) > 1e-9:
        vane_tau_z = (
            roll_cmd
            * vehicle.edf_vane_torque_max
            * (max(thrust, 0.0) / vehicle.max_thrust)
        )
        torque_body = torque_body + np.array([0.0, 0.0, vane_tau_z])

    ang_accel = vehicle.inertia_inv @ (torque_body - np.cross(omega, vehicle.inertia @ omega))

    # --- Thrust spool-up (first-order lag) ---
    thrust_cmd = throttle * vehicle.max_thrust
    dthrust = (thrust_cmd - thrust) / vehicle.thrust_time_constant

    deriv = np.zeros(STATE_DIM)
    deriv[POS] = v_world
    deriv[VEL] = accel
    deriv[QUAT] = quat.quat_derivative(q, omega)
    deriv[OMEGA] = ang_accel
    deriv[THRUST] = dthrust
    return deriv


def rk4_step(
    state: np.ndarray,
    cmd: np.ndarray,
    dt: float,
    vehicle: Vehicle,
    env: Environment,
    wind: np.ndarray | None = None,
    external_force: np.ndarray | None = None,
) -> np.ndarray:
    """One RK4 integration step (command and disturbances held constant over dt)."""
    k1 = state_derivative(state, cmd, vehicle, env, wind, external_force)
    k2 = state_derivative(state + 0.5 * dt * k1, cmd, vehicle, env, wind, external_force)
    k3 = state_derivative(state + 0.5 * dt * k2, cmd, vehicle, env, wind, external_force)
    k4 = state_derivative(state + dt * k3, cmd, vehicle, env, wind, external_force)
    new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    new_state[QUAT] = quat.quat_normalize(new_state[QUAT])
    return new_state
