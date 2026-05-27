"""Small MPC experiments for landing guidance/control.

This is intentionally a quick, dependency-free first pass: a sampled 1D
vertical MPC plans throttle over a short horizon, while the existing HoverPID
still handles horizontal position and attitude. It is not a full 6-DOF convex
powered-descent solver yet.
"""

from __future__ import annotations

from itertools import product

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat
from ..dynamics import Command
from ..vehicle import Environment, Vehicle
from .pid import HoverPID


class SampledVerticalMPC:
    """Grid-sampled 1D MPC for vertical landing throttle.

    State model:
        z_dot = vz
        vz_dot = thrust / mass - g
        thrust_dot = (throttle * max_thrust - thrust) / tau

    The planner evaluates a small library of piecewise-constant throttle
    sequences and returns the first throttle from the lowest-cost sequence.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        horizon: float = 4.0,
        dt: float = 0.1,
        segments: int = 4,
        touchdown_v: float = 0.25,
        max_touchdown_v: float = 1.0,
    ):
        self.vehicle = vehicle
        self.env = env
        self.horizon = horizon
        self.dt = dt
        self.steps = max(2, int(round(horizon / dt)))
        self.segments = max(1, segments)
        self.touchdown_v = touchdown_v
        self.max_touchdown_v = max_touchdown_v
        self.hover = np.clip(vehicle.mass * env.gravity / vehicle.max_thrust, 0.0, 1.0)
        self._seqs = self._build_sequences()

    def _build_sequences(self) -> np.ndarray:
        levels = np.unique(
            np.clip(
                np.array(
                    [
                        0.35 * self.hover,
                        0.60 * self.hover,
                        0.85 * self.hover,
                        1.05 * self.hover,
                        1.25 * self.hover,
                        1.0,
                    ]
                ),
                0.0,
                1.0,
            )
        )
        segment_values = np.array(list(product(levels, repeat=self.segments)), dtype=float)
        seg_len = int(np.ceil(self.steps / self.segments))
        seqs = np.repeat(segment_values, seg_len, axis=1)[:, : self.steps]
        # Include a few smooth, hand-shaped profiles that the coarse grid misses.
        ramps = []
        for start in (0.45 * self.hover, 0.70 * self.hover, self.hover):
            for end in (self.hover, 1.0):
                ramps.append(np.linspace(start, end, self.steps))
        return np.vstack([seqs, np.clip(np.array(ramps), 0.0, 1.0)])

    def throttle(self, z: float, vz: float, thrust: float) -> float:
        n = len(self._seqs)
        z_pred = np.full(n, max(float(z), 0.0))
        vz_pred = np.full(n, float(vz))
        thrust_pred = np.full(n, max(float(thrust), 0.0))
        cost = np.zeros(n)
        alive = np.ones(n, dtype=bool)
        hit = np.zeros(n, dtype=bool)
        hit_speed = np.zeros(n)
        prev_u = np.full(n, self.hover)

        for k in range(self.steps):
            u = self._seqs[:, k]
            accel = thrust_pred / self.vehicle.mass - self.env.gravity
            z_next = z_pred + vz_pred * self.dt
            vz_next = vz_pred + accel * self.dt
            a = np.exp(-self.dt / self.vehicle.thrust_time_constant)
            thrust_next = u * self.vehicle.max_thrust + (
                thrust_pred - u * self.vehicle.max_thrust
            ) * a
            thrust_next = np.clip(thrust_next, 0.0, self.vehicle.max_thrust)

            downward = np.maximum(0.0, -vz_next)
            speed_limit = self.touchdown_v + 0.55 * np.sqrt(np.maximum(z_next, 0.0))
            speed_limit = np.clip(speed_limit, self.max_touchdown_v, 3.5)
            overspeed = np.maximum(0.0, downward - speed_limit)

            running = (
                0.08 * z_next * z_next
                + 0.50 * overspeed * overspeed
                + 0.015 * (u - self.hover) * (u - self.hover)
                + 0.025 * (u - prev_u) * (u - prev_u)
            )
            cost += np.where(alive, running, 0.0)

            crossed = alive & (z_next <= 0.0)
            if np.any(crossed):
                hit[crossed] = True
                hit_speed[crossed] = downward[crossed]
                soft_excess = np.maximum(0.0, downward[crossed] - self.max_touchdown_v)
                cost[crossed] += 450.0 * soft_excess * soft_excess
                cost[crossed] += 80.0 * (downward[crossed] - self.touchdown_v) ** 2
                # Prefer reaching the pad softly over hovering high in the horizon.
                cost[crossed] -= 30.0
                alive[crossed] = False

            z_pred = np.where(alive, np.maximum(z_next, 0.0), z_pred)
            vz_pred = np.where(alive, vz_next, vz_pred)
            thrust_pred = np.where(alive, thrust_next, thrust_pred)
            prev_u = u

        terminal_downward = np.maximum(0.0, -vz_pred)
        cost += np.where(
            alive,
            7.0 * z_pred * z_pred + 20.0 * (terminal_downward - self.touchdown_v) ** 2,
            0.0,
        )
        # Hard impacts are worse than not reaching the ground this horizon.
        cost += np.where(hit, 120.0 * np.maximum(0.0, hit_speed - self.max_touchdown_v) ** 2, 0.0)
        best = int(np.argmin(cost))
        return float(self._seqs[best, 0])


class LandingVerticalMPC:
    """Landing controller using vertical MPC throttle plus HoverPID TVC."""

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.08,
        gate_offset: float = 0.7,
        gate_speed: float = 0.7,
        gate_tilt_deg: float = 8.0,
        gate_alt: float = 2.5,
        tight_frac: float = 0.5,
        slow_descent_margin: float = 0.03,
    ):
        self.vehicle = vehicle
        self.env = env
        self.pid = HoverPID(vehicle, env, target=(0.0, 0.0, 0.0))
        self.mpc = SampledVerticalMPC(vehicle, env)
        self.replan_dt = replan_dt
        self.gate_offset = gate_offset
        self.gate_speed = gate_speed
        self.gate_tilt = np.deg2rad(gate_tilt_deg)
        self.gate_alt = gate_alt
        self.tight_frac = tight_frac
        self.slow_descent_margin = slow_descent_margin
        self._last_plan_t = -np.inf
        self._last_throttle = self.mpc.hover

    def _readiness(self, state: np.ndarray, alt: float) -> float:
        scale = self.tight_frac + (1.0 - self.tight_frac) * min(alt / self.gate_alt, 1.0)
        horiz = np.linalg.norm(state[dyn.POS][:2])
        lat_speed = np.linalg.norm(state[dyn.VEL][:2])
        tilt = quat.tilt_angle(state[dyn.QUAT])
        return float(
            np.exp(-((horiz / (self.gate_offset * scale)) ** 2))
            * np.exp(-((lat_speed / (self.gate_speed * scale)) ** 2))
            * np.exp(-((tilt / (self.gate_tilt * scale)) ** 2))
        )

    def __call__(self, t: float, state: np.ndarray) -> Command:
        z = float(state[dyn.POS][2])
        vz = float(state[dyn.VEL][2])

        # Reuse HoverPID for horizontal centering and body-z alignment. Its
        # throttle is replaced by MPC below.
        self.pid.target = np.array([0.0, 0.0, max(z, 0.0)])
        cmd = self.pid(t, state)

        if t - self._last_plan_t >= self.replan_dt:
            self._last_plan_t = t
            self._last_throttle = self.mpc.throttle(z, vz, float(state[dyn.THRUST]))

        throttle = self._last_throttle
        readiness = self._readiness(state, max(z, 0.0))
        if z < self.gate_alt and readiness < 0.35:
            # Slow the final descent while horizontal/attitude errors settle,
            # but keep a slight descent bias so it does not hover indefinitely.
            throttle = max(throttle, self.mpc.hover - self.slow_descent_margin)

        cmd.throttle = float(np.clip(throttle, 0.0, 1.0))
        return cmd
