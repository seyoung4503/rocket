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
from ..guidance import LandingGuidance, LandingGuidanceConfig
from ..vehicle import Environment, Vehicle
from .landing import LandingPID
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


class CvxpyPointMassMPC:
    """Convex 3D point-mass MPC for powered descent guidance.

    The optimization state is position/velocity only. The control is a world
    thrust-acceleration vector ``u``; gravity is added in the dynamics. The
    thrust magnitude and maximum tilt are convex second-order-cone constraints:

        ||u|| <= max_thrust / mass
        ||u_xy|| <= tan(max_tilt) * u_z,  u_z >= 0

    This is the small "SpaceX-style" step: plan a constrained powered-descent
    acceleration vector, then let a lower-level attitude/TVC controller track it.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        horizon: float = 4.0,
        dt: float = 0.2,
        max_tilt: float = np.deg2rad(12.0),
        du_max: float | None = 3.0,
    ):
        # ``du_max`` (m/s^2 per step) caps how fast the planned thrust-acceleration
        # vector may rotate/grow between steps. This makes the MPC actuator-aware:
        # without it, the optimizer pays only a soft penalty for jumping the
        # thrust direction and ends up planning slews the 6-DOF body cannot
        # actually realize (diagnosis: gimbal saturates 87-94% of steps). Set to
        # None to recover the original point-mass behavior.
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover - exercised by environment setup
            raise RuntimeError("cvxpy is required for CvxpyPointMassMPC") from exc

        self.cp = cp
        self.vehicle = vehicle
        self.env = env
        self.dt = dt
        self.n = max(2, int(round(horizon / dt)))
        self.max_tilt = max_tilt
        self.max_acc = vehicle.max_thrust / vehicle.mass
        self.hover_acc = np.array([0.0, 0.0, env.gravity])
        self._last_u = self.hover_acc.copy()

        self.p0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.accel_bias = cp.Parameter(3, value=np.zeros(3))
        self.prev_u = cp.Parameter(3, value=self.hover_acc)
        self.p = cp.Variable((3, self.n + 1))
        self.v = cp.Variable((3, self.n + 1))
        self.u = cp.Variable((3, self.n))

        g = np.array([0.0, 0.0, -env.gravity])
        constraints = [self.p[:, 0] == self.p0, self.v[:, 0] == self.v0]
        cost = 0

        q_pos = np.array([1.2, 1.2, 0.08])
        q_vel = np.array([4.0, 4.0, 3.0])
        q_final_pos = np.array([45.0, 45.0, 18.0])
        q_final_vel = np.array([80.0, 80.0, 180.0])
        r_u = 0.02
        r_du = 0.25
        tan_tilt = float(np.tan(max_tilt))

        for k in range(self.n):
            acc = self.u[:, k] + g + self.accel_bias
            constraints += [
                self.p[:, k + 1]
                == self.p[:, k] + self.dt * self.v[:, k] + 0.5 * self.dt * self.dt * acc,
                self.v[:, k + 1] == self.v[:, k] + self.dt * acc,
                self.p[2, k] >= 0.0,
                self.u[2, k] >= 0.0,
                cp.norm(self.u[:, k], 2) <= self.max_acc,
                cp.norm(self.u[:2, k], 2) <= tan_tilt * self.u[2, k],
                self.v[2, k] >= -4.0,
                self.v[2, k] + 0.45 * self.p[2, k] >= -0.45,
            ]
            if du_max is not None:
                prev = self.prev_u if k == 0 else self.u[:, k - 1]
                constraints += [cp.norm(self.u[:, k] - prev, 2) <= du_max]
            cost += cp.sum(cp.multiply(q_pos, cp.square(self.p[:, k])))
            cost += cp.sum(cp.multiply(q_vel, cp.square(self.v[:, k])))
            cost += r_u * cp.sum_squares(self.u[:, k] - self.hover_acc)
            if k == 0:
                cost += r_du * cp.sum_squares(self.u[:, k] - self.prev_u)
            else:
                cost += r_du * cp.sum_squares(self.u[:, k] - self.u[:, k - 1])

        constraints += [self.p[2, self.n] >= 0.0, self.v[2, self.n] >= -0.35]
        cost += cp.sum(cp.multiply(q_final_pos, cp.square(self.p[:, self.n])))
        cost += cp.sum(cp.multiply(q_final_vel, cp.square(self.v[:, self.n])))

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def plan(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        accel_bias: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        self.p0.value = np.asarray(pos, dtype=float)
        self.v0.value = np.asarray(vel, dtype=float)
        if accel_bias is None:
            self.accel_bias.value = np.zeros(3)
        else:
            self.accel_bias.value = np.asarray(accel_bias, dtype=float)
        self.prev_u.value = self._last_u
        try:
            self.problem.solve(solver="CLARABEL", warm_start=True, verbose=False)
        except Exception:
            self.problem.solve(solver="SCS", warm_start=True, verbose=False, max_iters=500)

        if self.problem.status not in ("optimal", "optimal_inaccurate") or self.u.value is None:
            return None

        u0 = np.asarray(self.u.value[:, 0], dtype=float)
        if not np.all(np.isfinite(u0)):
            return None
        self._last_u = u0
        return (
            np.asarray(self.p.value, dtype=float).copy(),
            np.asarray(self.v.value, dtype=float).copy(),
            np.asarray(self.u.value, dtype=float).copy(),
        )

    def acceleration(self, pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        plan = self.plan(pos, vel)
        if plan is None:
            return self._last_u.copy()
        _, _, u = plan
        u0 = np.asarray(u[:, 0], dtype=float)
        self._last_u = u0
        return u0.copy()


class LandingCvxpyMPC:
    """3D convex point-mass MPC guidance plus attitude/TVC tracking."""

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(12.0),
        kp_att: float = 130.0,
        kd_att: float = 24.0,
        ki_att: float = 55.0,
        use_bias_estimator: bool = False,
    ):
        self.vehicle = vehicle
        self.env = env
        self.replan_dt = replan_dt
        self.max_tilt = max_tilt
        self.kp_att = kp_att
        self.kd_att = kd_att
        self.ki_att = ki_att
        self._att_int = np.zeros(3)
        self._last_t = 0.0
        self._last_plan_t = -np.inf
        self._u = np.array([0.0, 0.0, env.gravity])
        self.mpc = CvxpyPointMassMPC(vehicle, env, max_tilt=max_tilt)
        self._bias = np.zeros(3)
        self._last_vel_for_bias: np.ndarray | None = None
        self._last_bias_t: float | None = None
        self._last_thrust_accel = np.array([0.0, 0.0, env.gravity])
        self.use_bias_estimator = use_bias_estimator

    def _update_bias_estimate(self, t: float, state: np.ndarray) -> None:
        vel = np.asarray(state[dyn.VEL], dtype=float)
        if self._last_vel_for_bias is None or self._last_bias_t is None:
            self._last_vel_for_bias = vel.copy()
            self._last_bias_t = t
            return
        dt = t - self._last_bias_t
        if dt <= 1e-6:
            return
        measured_accel = (vel - self._last_vel_for_bias) / dt
        expected_accel = self._last_thrust_accel + np.array([0.0, 0.0, -self.env.gravity])
        residual = measured_accel - expected_accel
        residual = np.clip(residual, [-3.0, -3.0, -0.8], [3.0, 3.0, 0.8])
        alpha = 0.06
        self._bias = (1.0 - alpha) * self._bias + alpha * residual
        self._last_vel_for_bias = vel.copy()
        self._last_bias_t = t

    def _command_from_thrust_accel(self, t: float, state: np.ndarray, thrust_accel: np.ndarray) -> Command:
        self._last_thrust_accel = np.asarray(thrust_accel, dtype=float)
        v = self.vehicle
        q = state[dyn.QUAT]
        omega = state[dyn.OMEGA]
        f_des = v.mass * np.asarray(thrust_accel, dtype=float)
        f_mag = float(np.linalg.norm(f_des))
        if f_mag < 1e-6:
            z_des = np.array([0.0, 0.0, 1.0])
            throttle = 0.0
        else:
            z_des = f_des / f_mag
            tilt = np.arccos(np.clip(z_des[2], -1.0, 1.0))
            if tilt > self.max_tilt:
                horiz = z_des[:2]
                horiz_norm = np.linalg.norm(horiz)
                if horiz_norm > 1e-9:
                    horiz = horiz / horiz_norm * np.sin(self.max_tilt)
                z_des = np.array([horiz[0], horiz[1], np.cos(self.max_tilt)])
            throttle = float(np.clip(f_mag / v.max_thrust, 0.0, 1.0))

        dt = max(t - self._last_t, 0.0)
        self._last_t = t

        bz_world = quat.quat_rotate(q, np.array([0.0, 0.0, 1.0]))
        err_world = np.cross(bz_world, z_des)
        err_body = quat.quat_to_rotmat(q).T @ err_world
        self._att_int += err_body * dt
        self._att_int = np.clip(self._att_int, -0.5, 0.5)
        torque_des = v.inertia @ (
            self.kp_att * err_body - self.kd_att * omega + self.ki_att * self._att_int
        )

        thrust = max(throttle * v.max_thrust, 0.25 * v.max_thrust)
        lever = v.engine_offset * thrust
        gx = float(np.clip(-torque_des[0] / lever, -v.gimbal_limit, v.gimbal_limit))
        gy = float(np.clip(-torque_des[1] / lever, -v.gimbal_limit, v.gimbal_limit))
        return Command(throttle=throttle, gimbal_x=gx, gimbal_y=gy)

    def __call__(self, t: float, state: np.ndarray) -> Command:
        if self.use_bias_estimator:
            self._update_bias_estimate(t, state)
        if t - self._last_plan_t >= self.replan_dt:
            self._last_plan_t = t
            bias = self._bias if self.use_bias_estimator else None
            plan = self.mpc.plan(state[dyn.POS], state[dyn.VEL], accel_bias=bias)
            if plan is not None:
                _, _, u = plan
                self._u = np.asarray(u[:, 0], dtype=float)
        u = self._u.copy()
        pos, vel = state[dyn.POS], state[dyn.VEL]
        tilt = quat.tilt_angle(state[dyn.QUAT])
        touchdown_ready = (
            pos[2] < 0.35
            and np.linalg.norm(pos[:2]) < 0.45
            and np.linalg.norm(vel[:2]) < 0.40
            and vel[2] > -0.60
            and np.rad2deg(tilt) < 7.0
        )
        if touchdown_ready:
            # Convex MPC keeps z >= 0, so it tends to asymptotically hover just
            # above the pad. Once inside the landing box, bias thrust below
            # hover to commit to contact.
            u[:2] *= 0.5
            u[2] = min(u[2], 0.90 * self.env.gravity)
        return self._command_from_thrust_accel(t, state, u)


class LandingCvxpyGuidancePID(LandingCvxpyMPC):
    """Convex MPC as guidance, with feedback tracking underneath.

    This is closer to a real GNC stack than ``LandingCvxpyMPC``:

    * MPC plans a feasible point-mass trajectory.
    * A tracking controller converts the next waypoint plus feed-forward
      acceleration into a thrust vector.
    * The inherited attitude/TVC loop tracks that thrust vector.

    The MPC no longer directly commands the vehicle's instantaneous thrust
    vector, which reduces the mismatch caused by attitude and actuator lag.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        lookahead: int = 3,
        use_bias_estimator: bool = False,
    ):
        super().__init__(
            vehicle,
            env,
            replan_dt=replan_dt,
            max_tilt=max_tilt,
            use_bias_estimator=use_bias_estimator,
        )
        self.lookahead = max(1, lookahead)
        self.kp_track = np.array([0.9, 0.9, 1.4])
        self.kd_track = np.array([1.8, 1.8, 2.2])
        self._p_ref = np.zeros(3)
        self._v_ref = np.zeros(3)
        self._u_ff = np.array([0.0, 0.0, env.gravity])
        self._has_plan = False
        self._z_gate_set: float | None = None
        self._last_guidance_t = 0.0

    def _readiness(self, state: np.ndarray) -> float:
        z = max(float(state[dyn.POS][2]), 0.0)
        scale = 0.5 + 0.5 * min(z / 2.5, 1.0)
        horiz = np.linalg.norm(state[dyn.POS][:2])
        lat_speed = np.linalg.norm(state[dyn.VEL][:2])
        tilt = quat.tilt_angle(state[dyn.QUAT])
        return float(
            np.exp(-((horiz / (0.7 * scale)) ** 2))
            * np.exp(-((lat_speed / (0.7 * scale)) ** 2))
            * np.exp(-((tilt / (np.deg2rad(8.0) * scale)) ** 2))
        )

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos, vel = state[dyn.POS], state[dyn.VEL]
        if self.use_bias_estimator:
            self._update_bias_estimate(t, state)
        if t - self._last_plan_t >= self.replan_dt or not self._has_plan:
            self._last_plan_t = t
            bias = self._bias if self.use_bias_estimator else None
            plan = self.mpc.plan(pos, vel, accel_bias=bias)
            if plan is not None:
                p, v, u = plan
                k = min(self.lookahead, p.shape[1] - 1, u.shape[1] - 1)
                self._p_ref = p[:, k]
                self._v_ref = v[:, k]
                self._u_ff = u[:, 0]
                self._has_plan = True

        if not self._has_plan:
            return self._command_from_thrust_accel(t, state, self._u)

        p_ref = self._p_ref.copy()
        v_ref = self._v_ref.copy()
        u_ff = self._u_ff.copy()

        readiness = self._readiness(state)
        if self._z_gate_set is None:
            self._z_gate_set = float(pos[2])
        dt = max(t - self._last_guidance_t, 0.0)
        self._last_guidance_t = t

        # Terminal state machine: MPC may plan an optimistic descent; the real
        # 6-DOF vehicle must be centered, slow and upright before committing.
        v_desc = float(np.clip(0.45 * self._z_gate_set, 0.25, 3.5))
        gate = 0.35 + 0.65 * readiness
        self._z_gate_set = max(0.0, self._z_gate_set - v_desc * gate * dt)
        if pos[2] < 4.0 or self._z_gate_set < p_ref[2]:
            p_ref[2] = max(p_ref[2], self._z_gate_set)
            if p_ref[2] > pos[2] - 0.15:
                v_ref[2] = min(v_ref[2], 0.0)

        accel = u_ff + self.kp_track * (p_ref - pos) + self.kd_track * (v_ref - vel)

        touchdown_ready = (
            pos[2] < 0.35
            and np.linalg.norm(pos[:2]) < 0.45
            and np.linalg.norm(vel[:2]) < 0.40
            and vel[2] > -0.60
            and np.rad2deg(quat.tilt_angle(state[dyn.QUAT])) < 7.0
        )
        if touchdown_ready:
            accel[:2] *= 0.5
            accel[2] = min(accel[2], 0.88 * self.env.gravity)

        return self._command_from_thrust_accel(t, state, accel)


class LandingCvxpyWaypointPID:
    """Convex MPC waypoint guidance tracked by the existing HoverPID.

    This is the most conservative GNC split here:

    * Convex MPC proposes a near-future 3D waypoint.
    * The proven PID landing gate owns vertical commit timing.
    * HoverPID tracks the waypoint and keeps its integral disturbance rejection.

    It deliberately gives up some optimizer authority to test whether MPC
    guidance can help without breaking the robust PID inner/outer loops.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        lookahead: int = 4,
        v_max: float = 3.5,
        v_min: float = 0.25,
        flare_gain: float = 0.45,
        gate_offset: float = 0.7,
        gate_speed: float = 0.7,
        gate_tilt_deg: float = 8.0,
        gate_alt: float = 2.5,
        tight_frac: float = 0.5,
        creep: float = 0.35,
    ):
        self.vehicle = vehicle
        self.env = env
        self.pid = HoverPID(vehicle, env, target=(0.0, 0.0, 0.0))
        self.mpc = CvxpyPointMassMPC(vehicle, env, max_tilt=max_tilt)
        self.replan_dt = replan_dt
        self.lookahead = max(1, lookahead)
        self.guidance = LandingGuidance(
            LandingGuidanceConfig(
                v_max=v_max,
                v_min=v_min,
                flare_gain=flare_gain,
                gate_offset=gate_offset,
                gate_speed=gate_speed,
                gate_tilt_deg=gate_tilt_deg,
                gate_alt=gate_alt,
                tight_frac=tight_frac,
                creep=creep,
            )
        )
        self._last_plan_t = -np.inf
        self._xy_ref = np.zeros(2)
        self._has_plan = False

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos, vel = state[dyn.POS], state[dyn.VEL]
        if t - self._last_plan_t >= self.replan_dt or not self._has_plan:
            self._last_plan_t = t
            plan = self.mpc.plan(pos, vel)
            if plan is not None:
                p, _v, _u = plan
                k = min(self.lookahead, p.shape[1] - 1)
                self._xy_ref = p[:2, k]
                self._has_plan = True

        z = float(pos[2])
        # The optimizer's XY waypoint can be overly timid high up and overly
        # optimistic near the pad. Blend toward the actual pad target as we get
        # low so touchdown precision stays PID-like.
        gate_alt = self.guidance.config.gate_alt
        low_blend = np.clip((gate_alt - z) / gate_alt, 0.0, 1.0)
        xy_target = (1.0 - low_blend) * self._xy_ref
        self.pid.target = self.guidance.update(t, state, xy_target=xy_target).target
        return self.pid(t, state)


class LandingFeasibleWaypointMPC(LandingCvxpyWaypointPID):
    """Convex waypoint guidance with a simple actuator/attitude feasibility gate.

    The point-mass optimizer may propose lateral motion the real gimbaled body
    cannot track near touchdown. This controller projects the MPC waypoint back
    into a conservative lateral-acceleration/tilt envelope before PID sees it.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        max_required_tilt: float = np.deg2rad(22.0),
        near_ground_tilt: float = np.deg2rad(10.0),
        max_lateral_accel: float = 6.0,
        min_accept_readiness: float = 0.12,
        **kwargs,
    ):
        super().__init__(vehicle, env, **kwargs)
        self.max_required_tilt = max_required_tilt
        self.near_ground_tilt = near_ground_tilt
        self.max_lateral_accel = max_lateral_accel
        self.min_accept_readiness = min_accept_readiness
        self._accepted_xy_ref = np.zeros(2)

    def _project_xy(self, state: np.ndarray, xy_ref: np.ndarray) -> np.ndarray:
        pos = state[dyn.POS]
        vel = state[dyn.VEL]
        alt = max(float(pos[2]), 0.0)
        horizon = max(self.lookahead * self.mpc.dt, self.mpc.dt)
        required_accel = 2.0 * (xy_ref - pos[:2] - vel[:2] * horizon) / (horizon * horizon)
        accel_norm = float(np.linalg.norm(required_accel))

        gate_alt = self.guidance.config.gate_alt
        low = np.clip((gate_alt - alt) / gate_alt, 0.0, 1.0)
        tilt_limit = (1.0 - low) * self.max_required_tilt + low * self.near_ground_tilt

        current_tilt = quat.tilt_angle(state[dyn.QUAT])
        readiness = self.guidance.readiness(state, self.guidance.z_setpoint or alt)
        if alt < gate_alt and readiness < self.min_accept_readiness:
            return np.zeros(2)

        margin_tilt = max(0.0, np.deg2rad(24.0) - current_tilt)
        allowed_tilt = min(tilt_limit, margin_tilt)
        max_acc_from_tilt = self.env.gravity * np.tan(allowed_tilt)
        max_accel = max(0.0, min(self.max_lateral_accel, max_acc_from_tilt))

        if accel_norm > max_accel and accel_norm > 1e-9:
            required_accel = required_accel * (max_accel / accel_norm)
        projected = pos[:2] + vel[:2] * horizon + 0.5 * required_accel * horizon * horizon

        # Near touchdown, hand authority back toward the pad center so the
        # attitude loop is not still leaning to chase a lateral waypoint.
        ground_blend = np.clip((gate_alt - alt) / gate_alt, 0.0, 1.0)
        projected = (1.0 - 0.5 * ground_blend) * projected
        return projected

    def _feasible_xy(self, state: np.ndarray, xy_ref: np.ndarray) -> bool:
        projected = self._project_xy(state, xy_ref)
        if not np.all(np.isfinite(projected)):
            return False
        if np.linalg.norm(projected - xy_ref) > 1.0:
            return False
        pos = state[dyn.POS]
        vel = state[dyn.VEL]
        alt = max(float(pos[2]), 0.0)
        gate_alt = self.guidance.config.gate_alt
        readiness = self.guidance.readiness(state, self.guidance.z_setpoint or alt)
        current_tilt = quat.tilt_angle(state[dyn.QUAT])
        horizon = max(self.lookahead * self.mpc.dt, self.mpc.dt)
        required_accel = 2.0 * (projected - pos[:2] - vel[:2] * horizon) / (horizon * horizon)
        accel_norm = float(np.linalg.norm(required_accel))
        tilt_req = float(np.arctan2(accel_norm, self.env.gravity))
        if alt < gate_alt and readiness < self.min_accept_readiness:
            return False
        if current_tilt + tilt_req > np.deg2rad(24.0):
            return False
        return True

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos, vel = state[dyn.POS], state[dyn.VEL]
        if t - self._last_plan_t >= self.replan_dt or not self._has_plan:
            self._last_plan_t = t
            plan = self.mpc.plan(pos, vel)
            if plan is not None:
                p, _v, _u = plan
                k = min(self.lookahead, p.shape[1] - 1)
                candidate = p[:2, k]
                self._accepted_xy_ref = self._project_xy(state, candidate)
                self._has_plan = True

        z = float(pos[2])
        gate_alt = self.guidance.config.gate_alt
        low_blend = np.clip((gate_alt - z) / gate_alt, 0.0, 1.0)
        xy_target = (1.0 - low_blend) * self._accepted_xy_ref
        self.pid.target = self.guidance.update(t, state, xy_target=xy_target).target
        return self.pid(t, state)


class LandingFullDynamicsMPC:
    """Sampled nonlinear MPC using the same 6-DOF dynamics as the simulator.

    This is intentionally not convex. It is the model-fidelity counterpart to
    ``CvxpyPointMassMPC``: candidates are rolled forward through
    ``dyn.rk4_step`` with rigid-body attitude, inertia, thrust lag and gimbal
    rate limits. Disturbances are still unmeasured, so the rollout is a nominal
    prediction rather than an oracle future.

    The planner samples bounded residual command sequences around the existing
    LandingPID command and returns the first command from the lowest-cost
    rollout. That makes it a practical hybrid: PID supplies a robust stabilizing
    prior, while MPC rejects candidates that look bad under the full vehicle
    model.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.08,
        horizon: float = 1.2,
        dt: float = 0.08,
        candidates: int = 64,
        segments: int = 4,
        seed: int = 0,
        gate_offset: float = 0.7,
        gate_speed: float = 0.7,
        gate_tilt_deg: float = 8.0,
        gate_alt: float = 2.5,
        tight_frac: float = 0.5,
    ):
        self.vehicle = vehicle
        self.env = env
        self.pid = LandingPID(vehicle, env)
        self.replan_dt = replan_dt
        self.horizon = horizon
        self.dt = dt
        self.steps = max(4, int(round(horizon / dt)))
        self.candidates = max(2, candidates)
        self.segments = max(1, segments)
        self.rng = np.random.default_rng(seed)
        self.hover = float(np.clip(vehicle.mass * env.gravity / vehicle.max_thrust, 0.0, 1.0))
        self.gate_offset = gate_offset
        self.gate_speed = gate_speed
        self.gate_tilt = np.deg2rad(gate_tilt_deg)
        self.gate_alt = gate_alt
        self.tight_frac = tight_frac
        self._last_plan_t = -np.inf
        self._last_t = 0.0
        self._last_cmd = Command(throttle=self.hover)
        self._gimbal_est = np.zeros(2)

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

    def _update_gimbal_estimate(self, t: float) -> None:
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        target = np.array([self._last_cmd.gimbal_x, self._last_cmd.gimbal_y])
        max_delta = self.vehicle.gimbal_rate_limit * dt
        delta = np.clip(target - self._gimbal_est, -max_delta, max_delta)
        self._gimbal_est = self._gimbal_est + delta

    def _sample_sequences(self, base: np.ndarray) -> np.ndarray:
        seg_len = int(np.ceil(self.steps / self.segments))
        seqs = np.empty((self.candidates, self.steps, 3), dtype=float)
        seqs[0, :, :] = base

        # A few deterministic command biases make the optimizer less dependent
        # on random luck during short horizons.
        deterministic = [
            np.array([0.02, 0.0, 0.0]),
            np.array([-0.02, 0.0, 0.0]),
            np.array([0.00, np.deg2rad(1.0), 0.0]),
            np.array([0.00, -np.deg2rad(1.0), 0.0]),
            np.array([0.00, 0.0, np.deg2rad(1.0)]),
            np.array([0.00, 0.0, -np.deg2rad(1.0)]),
        ]
        for i, delta in enumerate(deterministic, start=1):
            if i >= self.candidates:
                break
            seqs[i, :, :] = base + delta

        start = 1 + min(len(deterministic), self.candidates - 1)
        for i in range(start, self.candidates):
            seg_delta = np.column_stack(
                [
                    self.rng.normal(0.0, 0.04, self.segments),
                    self.rng.normal(0.0, np.deg2rad(1.5), self.segments),
                    self.rng.normal(0.0, np.deg2rad(1.5), self.segments),
                ]
            )
            seq = np.repeat(base[None, :] + seg_delta, seg_len, axis=0)[: self.steps]
            seqs[i] = seq

        seqs[:, :, 0] = np.clip(seqs[:, :, 0], 0.0, 1.0)
        g = self.vehicle.gimbal_limit
        seqs[:, :, 1:] = np.clip(seqs[:, :, 1:], -g, g)
        return seqs

    def _touchdown_cost(self, state: np.ndarray, step: int) -> float:
        pos = state[dyn.POS]
        vel = state[dyn.VEL]
        offset = float(np.linalg.norm(pos[:2]))
        vspeed = float(max(0.0, -vel[2]))
        hspeed = float(np.linalg.norm(vel[:2]))
        tilt = float(quat.tilt_angle(state[dyn.QUAT]))
        cost = 0.0
        cost += 900.0 * max(0.0, offset - 0.50) ** 2
        cost += 900.0 * max(0.0, vspeed - 1.00) ** 2
        cost += 900.0 * max(0.0, hspeed - 0.50) ** 2
        cost += 900.0 * max(0.0, tilt - np.deg2rad(8.0)) ** 2
        cost += 35.0 * offset * offset
        cost += 20.0 * vspeed * vspeed
        cost += 20.0 * hspeed * hspeed
        cost += 35.0 * tilt * tilt
        if offset <= 0.50 and vspeed <= 1.00 and hspeed <= 0.50 and tilt <= np.deg2rad(8.0):
            cost -= 80.0 + 0.25 * (self.steps - step)
        return float(cost)

    def _rollout_cost(
        self,
        state: np.ndarray,
        seq: np.ndarray,
        z0: float,
        z_terminal_ref: float,
        initial_gimbal: np.ndarray,
    ) -> float:
        s = state.copy()
        gimbal = initial_gimbal.copy()
        cost = 0.0
        touched = False
        for k in range(self.steps):
            target = seq[k]
            max_delta = self.vehicle.gimbal_rate_limit * self.dt
            gimbal = gimbal + np.clip(target[1:] - gimbal, -max_delta, max_delta)
            applied = np.array([target[0], gimbal[0], gimbal[1]])
            s = dyn.rk4_step(s, applied, self.dt, self.vehicle, self.env)

            pos = s[dyn.POS]
            vel = s[dyn.VEL]
            omega = s[dyn.OMEGA]
            tilt = quat.tilt_angle(s[dyn.QUAT])
            progress = (k + 1) / self.steps
            z_ref = (1.0 - progress) * z0 + progress * z_terminal_ref
            desired_down = float(np.clip(0.45 * max(pos[2], 0.0), 0.25, 3.0))

            cost += 2.8 * float(np.dot(pos[:2], pos[:2]))
            cost += 1.2 * float(np.dot(vel[:2], vel[:2]))
            cost += 0.7 * float((pos[2] - z_ref) ** 2)
            cost += 0.25 * float((max(0.0, -vel[2]) - desired_down) ** 2)
            cost += 28.0 * float(tilt * tilt)
            cost += 0.20 * float(np.dot(omega, omega))
            cost += 0.02 * float((target[0] - self.hover) ** 2)
            cost += 2.00 * float(np.dot(target[1:], target[1:]))

            if tilt > np.deg2rad(28.0):
                cost += 2000.0 * float((tilt - np.deg2rad(28.0)) ** 2)
            if np.linalg.norm(pos[:2]) > 12.0 or pos[2] > 20.0:
                cost += 1000.0
                break
            if pos[2] <= 0.0:
                cost += self._touchdown_cost(s, k)
                touched = True
                break

        if not touched:
            pos = s[dyn.POS]
            vel = s[dyn.VEL]
            tilt = quat.tilt_angle(s[dyn.QUAT])
            omega = s[dyn.OMEGA]
            speed_limit = 0.25 + 0.55 * np.sqrt(max(pos[2], 0.0))
            cost += 18.0 * float(np.dot(pos[:2], pos[:2]))
            cost += 5.0 * float(np.dot(vel[:2], vel[:2]))
            cost += 8.0 * float((pos[2] - z_terminal_ref) ** 2)
            cost += 25.0 * float(max(0.0, -vel[2] - speed_limit) ** 2)
            cost += 90.0 * float(tilt * tilt)
            cost += 0.8 * float(np.dot(omega, omega))
        return float(cost)

    def __call__(self, t: float, state: np.ndarray) -> Command:
        self._update_gimbal_estimate(t)
        nominal = self.pid(t, state)
        if t - self._last_plan_t < self.replan_dt:
            return self._last_cmd

        self._last_plan_t = t
        base = np.array([nominal.throttle, nominal.gimbal_x, nominal.gimbal_y], dtype=float)
        z0 = max(float(state[dyn.POS][2]), 0.0)
        readiness = self._readiness(state, z0)
        v_desc = float(np.clip(0.45 * z0, 0.25, 3.5))
        z_terminal_ref = max(0.0, z0 - v_desc * (0.35 + 0.65 * readiness) * self.horizon)

        seqs = self._sample_sequences(base)
        best_i = 0
        baseline_cost = self._rollout_cost(state, seqs[0], z0, z_terminal_ref, self._gimbal_est)
        best_cost = baseline_cost
        for i, seq in enumerate(seqs[1:], start=1):
            cost = self._rollout_cost(state, seq, z0, z_terminal_ref, self._gimbal_est)
            if cost < best_cost:
                best_cost = cost
                best_i = i

        required_gain = max(5.0, 0.15 * abs(baseline_cost))
        if best_i == 0 or best_cost > baseline_cost - required_gain:
            first = base
        else:
            # Treat full-dynamics MPC as a residual safety layer. The nominal
            # PID command remains the stabilizing backbone; sampled MPC can only
            # make a small first-step correction when the rollout advantage is
            # large enough to justify trusting the nonlinear prediction.
            first = 0.75 * base + 0.25 * seqs[best_i, 0]
        self._last_cmd = Command(
            throttle=float(first[0]),
            gimbal_x=float(first[1]),
            gimbal_y=float(first[2]),
        )
        return self._last_cmd


# ============================================================================
# Step 1 of actuator-aware MPC (2026-05-29 v1).
#
# See docs/2026-05-29_0213_v1_hierarchical_mpc_plan.md for the design rationale
# and the four caveats from the codex literature check that this class
# explicitly addresses:
#   1. slew is an EFFECTIVE JERK limit, not just the gimbal rate spec
#   2. a slack variable absorbs infeasibility during disturbance recovery
#   3. world-frame slew is still an APPROXIMATION of body attitude dynamics
#   4. slew authority must scale with the current thrust magnitude
# ============================================================================


class CvxpyActuatorAwareMPC:
    """Convex MPC that treats the thrust-acceleration VECTOR as a STATE with a
    hard slew-rate constraint, so the planned trajectory is realizable by a
    finite-bandwidth thrust vectoring system (i.e. the body can't rotate fast
    enough to teleport the thrust direction).

    Difference from ``CvxpyPointMassMPC``:
      * ``u`` (world-frame thrust acceleration) is a STATE, not a control.
      * The control is the RATE ``du`` with a hard SOC constraint.
      * The slew bound is proportional to the current vertical thrust
        (``||du[k]|| <= slew_factor * u[2,k] + slack[k]``), reflecting that
        lateral acceleration authority is limited by thrust magnitude.
      * A slack variable absorbs infeasibility under wind/disturbance gusts.

    The interface (``plan(pos, vel, accel_bias=None)``) and return shape
    (``(p, v, u)``) match ``CvxpyPointMassMPC`` so existing wrappers can
    swap to this class without changes.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        horizon: float = 4.0,
        dt: float = 0.2,
        max_tilt: float = np.deg2rad(12.0),
        # Slew factor: ||du[k]|| <= slew_factor * u_z[k] + slack[k].
        # At hover (u_z = g = 9.8 m/s^2), 0.6 gives ~5.9 m/s^2 of allowable
        # thrust-vector change per dt=0.2s step, derived from an EDF-class
        # effective jerk J_eff = (T/m) * achievable_omega ~ 9.8 * 3 rad/s.
        slew_factor: float = 0.6,
        slack_weight: float = 50.0,
        # 2026-05-29 diagnostic showed gimbal sat 96% -> plan demands
        # infeasible lateral motion. q_pos_xy / q_final_pos_xy parameterize the
        # horizontal-tracking aggressiveness so we can sweep gentler weights.
        q_pos_xy: float = 1.2,
        q_final_pos_xy: float = 45.0,
    ):
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("cvxpy is required for CvxpyActuatorAwareMPC") from exc

        self.cp = cp
        self.vehicle = vehicle
        self.env = env
        self.dt = dt
        self.n = max(2, int(round(horizon / dt)))
        self.max_tilt = max_tilt
        self.max_acc = vehicle.max_thrust / vehicle.mass
        self.hover_acc = np.array([0.0, 0.0, env.gravity])
        self.slew_factor = float(slew_factor)
        self.slack_weight = float(slack_weight)
        self._last_u = self.hover_acc.copy()

        self.p0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.u0 = cp.Parameter(3, value=self.hover_acc)
        self.accel_bias = cp.Parameter(3, value=np.zeros(3))

        self.p = cp.Variable((3, self.n + 1))
        self.v = cp.Variable((3, self.n + 1))
        self.u = cp.Variable((3, self.n + 1))  # thrust accel as a STATE
        self.du = cp.Variable((3, self.n))     # rate of change is the CONTROL
        self.slack = cp.Variable(self.n, nonneg=True)

        g = np.array([0.0, 0.0, -env.gravity])
        tan_tilt = float(np.tan(max_tilt))

        constraints = [
            self.p[:, 0] == self.p0,
            self.v[:, 0] == self.v0,
            self.u[:, 0] == self.u0,
        ]
        cost = 0

        q_pos = np.array([float(q_pos_xy), float(q_pos_xy), 0.08])
        q_vel = np.array([4.0, 4.0, 3.0])
        q_final_pos = np.array([float(q_final_pos_xy), float(q_final_pos_xy), 18.0])
        q_final_vel = np.array([80.0, 80.0, 180.0])
        r_u = 0.02
        r_du = 0.05

        for k in range(self.n):
            acc = self.u[:, k] + g + self.accel_bias
            constraints += [
                # position/velocity dynamics
                self.p[:, k + 1]
                == self.p[:, k] + dt * self.v[:, k] + 0.5 * dt * dt * acc,
                self.v[:, k + 1] == self.v[:, k] + dt * acc,
                # thrust-vector dynamics: u evolves via its rate du
                self.u[:, k + 1] == self.u[:, k] + dt * self.du[:, k],
                # ground / one-sided thrust
                self.p[2, k] >= 0.0,
                self.u[2, k] >= 0.0,
                # thrust magnitude + tilt cone
                cp.norm(self.u[:, k], 2) <= self.max_acc,
                cp.norm(self.u[:2, k], 2) <= tan_tilt * self.u[2, k],
                # descent rate + glideslope (mirrors point-mass MPC)
                self.v[2, k] >= -4.0,
                self.v[2, k] + 0.45 * self.p[2, k] >= -0.45,
                # ★ hard slew constraint with thrust-magnitude scaling + slack.
                #   ||du[k]||_2 <= slew_factor * u_z[k] + slack[k]
                #   This is a second-order cone constraint (convex).
                cp.norm(self.du[:, k], 2)
                <= self.slew_factor * self.u[2, k] + self.slack[k],
            ]
            cost += cp.sum(cp.multiply(q_pos, cp.square(self.p[:, k])))
            cost += cp.sum(cp.multiply(q_vel, cp.square(self.v[:, k])))
            cost += r_u * cp.sum_squares(self.u[:, k] - self.hover_acc)
            cost += r_du * cp.sum_squares(self.du[:, k])

        constraints += [
            self.p[2, self.n] >= 0.0,
            self.v[2, self.n] >= -0.35,
            self.u[2, self.n] >= 0.0,
            cp.norm(self.u[:, self.n], 2) <= self.max_acc,
            cp.norm(self.u[:2, self.n], 2) <= tan_tilt * self.u[2, self.n],
        ]
        cost += cp.sum(cp.multiply(q_final_pos, cp.square(self.p[:, self.n])))
        cost += cp.sum(cp.multiply(q_final_vel, cp.square(self.v[:, self.n])))
        cost += self.slack_weight * cp.sum_squares(self.slack)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def plan(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        accel_bias: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        self.p0.value = np.asarray(pos, dtype=float)
        self.v0.value = np.asarray(vel, dtype=float)
        # Initial thrust-acceleration state: warm-started from the previous plan.
        self.u0.value = np.asarray(self._last_u, dtype=float)
        if accel_bias is None:
            self.accel_bias.value = np.zeros(3)
        else:
            self.accel_bias.value = np.asarray(accel_bias, dtype=float)
        try:
            self.problem.solve(solver="CLARABEL", warm_start=True, verbose=False)
        except Exception:
            self.problem.solve(solver="SCS", warm_start=True, verbose=False, max_iters=500)

        if self.problem.status not in ("optimal", "optimal_inaccurate") or self.u.value is None:
            return None

        u_traj = np.asarray(self.u.value, dtype=float)
        if not np.all(np.isfinite(u_traj)):
            return None
        # Warm-start: next u0 is what we planned for time step 1.
        self._last_u = u_traj[:, 1].copy() if u_traj.shape[1] > 1 else u_traj[:, 0].copy()
        # Return the first n columns of u to match the point-mass MPC signature
        # (control-shaped (3, n) so downstream wrappers don't have to change).
        return (
            np.asarray(self.p.value, dtype=float).copy(),
            np.asarray(self.v.value, dtype=float).copy(),
            u_traj[:, : self.n].copy(),
        )

    def acceleration(self, pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        plan = self.plan(pos, vel)
        if plan is None:
            return self._last_u.copy()
        _, _, u = plan
        return np.asarray(u[:, 0], dtype=float).copy()


# ============================================================================
# Step 2 of actuator-aware MPC (2026-05-29 v1).
#
# Extends Step 1: the *vector slew* limit stays, but now thrust *magnitude*
# is also a 1st-order lagged state matching the simulator's EDF spool-up
# (``vehicle.thrust_time_constant``).  The plan is still a point-mass model;
# Step 3 (SCP 6-DOF, separate class) is the first step that leaves point
# mass entirely.  See docs/2026-05-29_0213_v1_hierarchical_mpc_plan.md.
# ============================================================================


class CvxpyActuatorAwareMagLagMPC:
    """Step 2: Step 1 + thrust-magnitude 1st-order lag.

    Adds two scalar quantities on top of ``CvxpyActuatorAwareMPC``:
      * ``T[k]`` (state)   — actual thrust-accel magnitude.
      * ``T_cmd[k]`` (control) — commanded magnitude, bounded ``[0, max_acc]``.
    Dynamics: ``T[k+1] = a*T[k] + (1-a)*T_cmd[k]`` with
    ``a = exp(-dt / tau_spool)`` (exact ZOH).  Default ``tau_spool`` reads
    ``vehicle.thrust_time_constant`` so the MPC's internal lag matches the
    simulator.

    The Step-1 bound ``||u||_2 <= max_acc`` is replaced with the lossless-
    convex relaxation ``||u||_2 <= T[k]``, with ``T_cmd`` driving ``T``.
    Direction (tilt cone) and Step-1 slew constraints carry over unchanged.

    Still point-mass (no attitude state); Step 3 is the structural fix.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        horizon: float = 4.0,
        dt: float = 0.2,
        max_tilt: float = np.deg2rad(12.0),
        slew_factor: float = 0.9,
        slack_weight: float = 50.0,
        q_pos_xy: float = 1.2,
        q_final_pos_xy: float = 45.0,
        # EDF spool-up lag time constant. ``None`` -> read vehicle attribute so
        # the internal lag matches the simulator's actual lag.
        tau_spool: float | None = None,
        # Effort cost on commanded magnitude, nudging T_cmd toward hover.
        # Same magnitude as r_u (0.02). Needed so the lossless-convex
        # relaxation ||u||<=T doesn't park T at an arbitrarily large value.
        r_Tcmd: float = 0.02,
    ):
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "cvxpy is required for CvxpyActuatorAwareMagLagMPC"
            ) from exc

        self.cp = cp
        self.vehicle = vehicle
        self.env = env
        self.dt = dt
        self.n = max(2, int(round(horizon / dt)))
        self.max_tilt = max_tilt
        self.max_acc = vehicle.max_thrust / vehicle.mass
        self.hover_acc = np.array([0.0, 0.0, env.gravity])
        self.hover_mag = float(env.gravity)
        self.slew_factor = float(slew_factor)
        self.slack_weight = float(slack_weight)
        self.tau_spool = float(
            tau_spool if tau_spool is not None
            else getattr(vehicle, "thrust_time_constant", 0.08)
        )
        self._lag_a = float(np.exp(-self.dt / self.tau_spool))
        self.r_Tcmd = float(r_Tcmd)

        self._last_u = self.hover_acc.copy()
        self._last_T = self.hover_mag

        self.p0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.u0 = cp.Parameter(3, value=self.hover_acc)
        self.T0 = cp.Parameter(value=self.hover_mag, nonneg=True)
        self.accel_bias = cp.Parameter(3, value=np.zeros(3))

        self.p = cp.Variable((3, self.n + 1))
        self.v = cp.Variable((3, self.n + 1))
        self.u = cp.Variable((3, self.n + 1))            # thrust accel (state)
        self.du = cp.Variable((3, self.n))               # thrust-accel rate
        self.T = cp.Variable(self.n + 1, nonneg=True)    # thrust-mag state
        self.T_cmd = cp.Variable(self.n, nonneg=True)    # thrust-mag control
        self.slack = cp.Variable(self.n, nonneg=True)

        g = np.array([0.0, 0.0, -env.gravity])
        tan_tilt = float(np.tan(max_tilt))
        a = self._lag_a

        constraints = [
            self.p[:, 0] == self.p0,
            self.v[:, 0] == self.v0,
            self.u[:, 0] == self.u0,
            self.T[0] == self.T0,
        ]
        cost = 0

        q_pos = np.array([float(q_pos_xy), float(q_pos_xy), 0.08])
        q_vel = np.array([4.0, 4.0, 3.0])
        q_final_pos = np.array(
            [float(q_final_pos_xy), float(q_final_pos_xy), 18.0]
        )
        q_final_vel = np.array([80.0, 80.0, 180.0])
        r_u = 0.02
        r_du = 0.05

        for k in range(self.n):
            acc = self.u[:, k] + g + self.accel_bias
            constraints += [
                # position/velocity dynamics
                self.p[:, k + 1]
                == self.p[:, k] + dt * self.v[:, k] + 0.5 * dt * dt * acc,
                self.v[:, k + 1] == self.v[:, k] + dt * acc,
                # thrust-vector dynamics: u evolves via its rate du (Step 1)
                self.u[:, k + 1] == self.u[:, k] + dt * self.du[:, k],
                # ★ thrust-magnitude 1st-order lag (Step 2 addition)
                #   T[k+1] = a*T[k] + (1-a)*T_cmd[k]   (exact ZOH)
                self.T[k + 1]
                == a * self.T[k] + (1.0 - a) * self.T_cmd[k],
                # commanded-magnitude upper bound (T_cmd >= 0 via nonneg flag)
                self.T_cmd[k] <= self.max_acc,
                # ground / one-sided thrust
                self.p[2, k] >= 0.0,
                self.u[2, k] >= 0.0,
                # ★ thrust-vector magnitude bounded by lagged actual T
                #   (lossless-convex relaxation à la Açıkmeşe G-FOLD)
                cp.norm(self.u[:, k], 2) <= self.T[k],
                # tilt cone (gimbal direction limit)
                cp.norm(self.u[:2, k], 2) <= tan_tilt * self.u[2, k],
                # descent rate + glideslope (mirrors point-mass MPC)
                self.v[2, k] >= -4.0,
                self.v[2, k] + 0.45 * self.p[2, k] >= -0.45,
                # Step 1 slew on thrust-vector rate
                cp.norm(self.du[:, k], 2)
                <= self.slew_factor * self.u[2, k] + self.slack[k],
            ]
            cost += cp.sum(cp.multiply(q_pos, cp.square(self.p[:, k])))
            cost += cp.sum(cp.multiply(q_vel, cp.square(self.v[:, k])))
            cost += r_u * cp.sum_squares(self.u[:, k] - self.hover_acc)
            cost += r_du * cp.sum_squares(self.du[:, k])
            cost += self.r_Tcmd * cp.square(self.T_cmd[k] - self.hover_mag)

        constraints += [
            self.p[2, self.n] >= 0.0,
            self.v[2, self.n] >= -0.35,
            self.u[2, self.n] >= 0.0,
            cp.norm(self.u[:, self.n], 2) <= self.T[self.n],
            cp.norm(self.u[:2, self.n], 2) <= tan_tilt * self.u[2, self.n],
        ]
        cost += cp.sum(cp.multiply(q_final_pos, cp.square(self.p[:, self.n])))
        cost += cp.sum(cp.multiply(q_final_vel, cp.square(self.v[:, self.n])))
        cost += self.slack_weight * cp.sum_squares(self.slack)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def plan(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        accel_bias: np.ndarray | None = None,
        thrust_mag: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        self.p0.value = np.asarray(pos, dtype=float)
        self.v0.value = np.asarray(vel, dtype=float)
        self.u0.value = np.asarray(self._last_u, dtype=float)
        if thrust_mag is None:
            self.T0.value = float(self._last_T)
        else:
            self.T0.value = float(np.clip(thrust_mag, 1e-6, self.max_acc))
        if accel_bias is None:
            self.accel_bias.value = np.zeros(3)
        else:
            self.accel_bias.value = np.asarray(accel_bias, dtype=float)
        try:
            self.problem.solve(solver="CLARABEL", warm_start=True, verbose=False)
        except Exception:
            self.problem.solve(
                solver="SCS", warm_start=True, verbose=False, max_iters=500
            )

        if (
            self.problem.status not in ("optimal", "optimal_inaccurate")
            or self.u.value is None
        ):
            return None

        u_traj = np.asarray(self.u.value, dtype=float)
        T_traj = np.asarray(self.T.value, dtype=float)
        if not np.all(np.isfinite(u_traj)) or not np.all(np.isfinite(T_traj)):
            return None
        # Warm-start the next call from what we planned for step 1.
        self._last_u = (
            u_traj[:, 1].copy() if u_traj.shape[1] > 1 else u_traj[:, 0].copy()
        )
        self._last_T = (
            float(T_traj[1]) if T_traj.shape[0] > 1 else float(T_traj[0])
        )
        return (
            np.asarray(self.p.value, dtype=float).copy(),
            np.asarray(self.v.value, dtype=float).copy(),
            u_traj[:, : self.n].copy(),
        )

    def acceleration(self, pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        plan = self.plan(pos, vel)
        if plan is None:
            return self._last_u.copy()
        _, _, u = plan
        return np.asarray(u[:, 0], dtype=float).copy()


class LandingActuatorAwareMagLagWaypointPID(LandingCvxpyWaypointPID):
    """Step 2 sibling of ``LandingActuatorAwareWaypointPID``: same waypoint /
    PID landing gate / HoverPID tracking, but the underlying MPC is the Step 2
    magnitude-lag variant."""

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        lookahead: int = 10,  # see LandingCvxpyWaypointPID for rationale
        slew_factor: float = 0.9,
        slack_weight: float = 50.0,
        q_pos_xy: float = 1.2,
        q_final_pos_xy: float = 45.0,
        tau_spool: float | None = None,
        r_Tcmd: float = 0.02,
        **gate_kwargs,
    ):
        super().__init__(
            vehicle,
            env,
            replan_dt=replan_dt,
            max_tilt=max_tilt,
            lookahead=lookahead,
            **gate_kwargs,
        )
        self.mpc = CvxpyActuatorAwareMagLagMPC(
            vehicle,
            env,
            max_tilt=np.deg2rad(12.0),
            slew_factor=slew_factor,
            slack_weight=slack_weight,
            q_pos_xy=q_pos_xy,
            q_final_pos_xy=q_final_pos_xy,
            tau_spool=tau_spool,
            r_Tcmd=r_Tcmd,
        )


class LandingActuatorAwareWaypointPID(LandingCvxpyWaypointPID):
    """Step 1 sibling of ``LandingCvxpyWaypointPID`` that uses the
    actuator-aware (slew-limited) MPC underneath. Everything else (the PID
    landing gate, HoverPID tracking, vertical commit logic) is identical, so
    the only A/B difference is the underlying MPC model."""

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        lookahead: int = 10,  # see LandingCvxpyWaypointPID for rationale
        # 2026-05-29 bumped 0.6 -> 0.9: the n=50 sweep with 0.6 (46% hard,
        # 74% noisy) showed the MPC was too conservative — its plan was too
        # sluggish to catch wind disturbances. Higher beta = less conservative.
        slew_factor: float = 0.9,
        slack_weight: float = 50.0,
        # Horizontal tracking aggressiveness — see CvxpyActuatorAwareMPC.
        # Defaults match the original (aggressive) weights; A/B variants in
        # evaluate_navigation pass softer values.
        q_pos_xy: float = 1.2,
        q_final_pos_xy: float = 45.0,
        **gate_kwargs,
    ):
        super().__init__(
            vehicle,
            env,
            replan_dt=replan_dt,
            max_tilt=max_tilt,
            lookahead=lookahead,
            **gate_kwargs,
        )
        # Swap to the actuator-aware MPC. The MPC's max_tilt should match the
        # cone tilt used for waypoint generation, not the controller's overall
        # tilt budget; keep it at the 12° gimbal cone like the point-mass MPC.
        self.mpc = CvxpyActuatorAwareMPC(
            vehicle,
            env,
            max_tilt=np.deg2rad(12.0),
            slew_factor=slew_factor,
            slack_weight=slack_weight,
            q_pos_xy=q_pos_xy,
            q_final_pos_xy=q_final_pos_xy,
        )
