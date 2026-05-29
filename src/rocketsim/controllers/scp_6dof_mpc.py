"""Step 3 — Linearized 6-DOF landing MPC.

The previous Step 1 / Step 2 MPCs treat the rocket as a point mass with
extra actuator-aware constraints layered on top.  Step 3 is the first
variant that actually leaves the point-mass assumption: attitude
(small-angle rotation vector ``φ``) and body angular velocity ``ω``
are *states* of the MPC, and gimbal torque is the control through which
the optimizer steers attitude and therefore thrust direction.

Full nonlinear 6-DOF MPC would require Successive Convex Programming
(SCP) — iteratively linearizing R(q)·ê_z and ω×Iω around a reference
trajectory until convergence.  See Szmuk-Açıkmeşe arXiv 1811.10803 for
the canonical formulation.

This file implements a *single-shot linearization* around
**(vertical attitude, hover thrust)**, which is exact at hover and
accurate while the vehicle stays near the small-angle regime
(|φ| ≲ 15°).  For the short EDF hop scenarios we run, the vehicle
spends most of its time inside that envelope, so a single linearization
is a reasonable trade-off between fidelity and simplicity.  The class
is named ``CvxpyScp6DofMPC`` to signal the family it belongs to and
to leave room for a true SCP variant later (which can subclass this
and add the iteration loop).

State (12-D)
------------
    p   ∈ R³   world-frame position
    v   ∈ R³   world-frame velocity
    φ   ∈ R³   small-angle rotation vector (body frame, ≈ 2·q_v)
    ω   ∈ R³   body-frame angular velocity

Control (3-D)
-------------
    T   ∈ [0, T_max]      thrust force magnitude (N)
    g_x ∈ [-g_lim, g_lim] gimbal angle about body-x (rad)
    g_y ∈ [-g_lim, g_lim] gimbal angle about body-y (rad)

Linearized dynamics (around q_bar = identity, T_bar = m·g)
----------------------------------------------------------
With body-z = (0, 0, 1) and small-angle vector φ, the rotated body-z
in the world frame is approximately
    R(q) ê_z ≈ ê_z + (φ_y, -φ_x, 0)

so the world-frame thrust acceleration is
    a_thrust ≈ (T/m) · (φ_y, -φ_x, 1)

Discrete dynamics (Euler integration, step dt):
    p[k+1] = p[k] + dt·v[k] + ½·dt²·(g·φ_y[k], -g·φ_x[k], T[k]/m − g)
    v[k+1] = v[k] + dt·(g·φ_y[k], -g·φ_x[k], T[k]/m − g)
    φ[k+1] = φ[k] + dt·ω[k]
    ω[k+1] = ω[k] + dt·I⁻¹·τ_body
where τ_body = m·g·L · (−g_y, g_x, 0) ∈ R³ comes from the linearized
gimbal-torque model (thrust ≈ hover, lever arm L = engine_offset).

The full quadratic cost (regulator + terminal landing) keeps the
problem an SOCP that cvxpy solves quickly.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from ..vehicle import Environment, Vehicle


class CvxpyScp6DofMPC:
    """Linearized 6-DOF landing MPC.

    Returns plans in the same ``(p, v, u)`` shape as
    ``CvxpyPointMassMPC`` so the existing
    ``LandingCvxpyWaypointPID``-style wrappers can drive it without
    modification (apart from passing ``q`` and ``omega`` through to
    ``plan(...)``).

    The synthesized ``u`` is the *linearized world-frame thrust
    acceleration*:
        u_x = g·φ_y,   u_y = −g·φ_x,   u_z = T/m
    which is exact at hover and a first-order approximation elsewhere.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        horizon: float = 4.0,
        dt: float = 0.2,
        max_tilt: float = np.deg2rad(30.0),  # safety bound on φ
        v_max_desc: float = 4.0,
        # Cost weights.  Tuned to match the regulator scale of the
        # existing point-mass MPC (q_pos[xy]=1.2, q_pos[z]=0.08,
        # q_final[xy]=45, q_final[z]=18) plus new attitude penalties.
        q_pos_xy: float = 1.2,
        q_pos_z: float = 0.08,
        q_vel_xy: float = 4.0,
        q_vel_z: float = 3.0,
        q_phi: float = 5.0,
        q_omega: float = 0.5,
        q_final_pos_xy: float = 45.0,
        q_final_pos_z: float = 18.0,
        q_final_vel_xy: float = 80.0,
        q_final_vel_z: float = 180.0,
        q_final_phi: float = 100.0,
        q_final_omega: float = 50.0,
        r_thrust: float = 0.02,
        r_gimbal: float = 0.05,
        # Soft slack on descent rate at k > 0 (initial vz may already
        # exceed the bound and we don't want strict infeasibility).
        w_desc_slack: float = 1000.0,
    ):
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("cvxpy is required for CvxpyScp6DofMPC") from exc

        self.cp = cp
        self.vehicle = vehicle
        self.env = env
        self.dt = dt
        self.n = max(2, int(round(horizon / dt)))
        self.max_tilt = float(max_tilt)
        self.v_max_desc = float(v_max_desc)

        m = vehicle.mass
        g_val = env.gravity
        L = vehicle.engine_offset
        # Diagonal inertia is a good approximation for slender bodies.
        I = np.asarray(vehicle.inertia, dtype=float)
        if I.ndim == 2:
            I_diag = np.diag(I)
        else:
            I_diag = I
        I_diag = np.asarray(I_diag, dtype=float)
        # Avoid divide-by-zero on tiny roll inertia.
        I_diag_safe = np.where(I_diag > 1e-9, I_diag, 1.0)

        self.m = float(m)
        self.g = float(g_val)
        self.L = float(L)
        self.I_diag = I_diag.copy()
        self._I_inv_safe = 1.0 / I_diag_safe
        self.T_max = float(vehicle.max_thrust)
        self.gimbal_limit = float(vehicle.gimbal_limit)
        # Linearized torque-from-gimbal coefficient (Newton·meter per rad).
        # τ_body ≈ T_bar·L · (−g_y, g_x, 0) at T_bar = m·g.
        self._tau_coeff = m * g_val * L  # = T_bar * L

        # Parameters (set per call).
        self.p0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.phi0 = cp.Parameter(3)
        self.omega0 = cp.Parameter(3)

        # Variables.
        self.p = cp.Variable((3, self.n + 1))
        self.v = cp.Variable((3, self.n + 1))
        self.phi = cp.Variable((3, self.n + 1))
        self.omega = cp.Variable((3, self.n + 1))
        self.T = cp.Variable(self.n)                      # thrust force (N)
        self.gx = cp.Variable(self.n)                     # gimbal x (rad)
        self.gy = cp.Variable(self.n)                     # gimbal y (rad)
        self.desc_slack = cp.Variable(self.n, nonneg=True)

        g_world_z = -g_val  # gravity z component
        T_bar = m * g_val   # hover thrust
        # Inertia-scaled torque coefficients for ω̇.
        cw_x = self._tau_coeff / I_diag_safe[0]   # multiplies -g_y
        cw_y = self._tau_coeff / I_diag_safe[1]   # multiplies  g_x

        constraints = [
            self.p[:, 0] == self.p0,
            self.v[:, 0] == self.v0,
            self.phi[:, 0] == self.phi0,
            self.omega[:, 0] == self.omega0,
        ]

        cost = 0
        for k in range(self.n):
            # Linearized world-frame thrust acceleration.
            ax = g_val * self.phi[1, k]
            ay = -g_val * self.phi[0, k]
            az = self.T[k] / m - g_val

            # Position / velocity dynamics (Euler with ½·dt² term).
            constraints += [
                self.p[0, k + 1] == self.p[0, k] + dt * self.v[0, k] + 0.5 * dt * dt * ax,
                self.p[1, k + 1] == self.p[1, k] + dt * self.v[1, k] + 0.5 * dt * dt * ay,
                self.p[2, k + 1] == self.p[2, k] + dt * self.v[2, k] + 0.5 * dt * dt * az,
                self.v[0, k + 1] == self.v[0, k] + dt * ax,
                self.v[1, k + 1] == self.v[1, k] + dt * ay,
                self.v[2, k + 1] == self.v[2, k] + dt * az,
            ]

            # Attitude kinematics + linearized rotational dynamics.
            constraints += [
                self.phi[:, k + 1] == self.phi[:, k] + dt * self.omega[:, k],
                # ω̇_x ≈ (T_bar·L/I_xx) · (-g_y)
                self.omega[0, k + 1]
                == self.omega[0, k] + dt * cw_x * (-self.gy[k]),
                # ω̇_y ≈ (T_bar·L/I_yy) · g_x
                self.omega[1, k + 1]
                == self.omega[1, k] + dt * cw_y * self.gx[k],
                # ω_z is not actuated by xy gimbals in this linearization,
                # but keep it on the state with zero plan dynamics so the
                # terminal cost can penalize a roll component imported from
                # initial conditions.
                self.omega[2, k + 1] == self.omega[2, k],
            ]

            # Control + state bounds.
            constraints += [
                self.T[k] >= 0,
                self.T[k] <= self.T_max,
                self.gx[k] >= -self.gimbal_limit,
                self.gx[k] <= self.gimbal_limit,
                self.gy[k] >= -self.gimbal_limit,
                self.gy[k] <= self.gimbal_limit,
                # Safety: keep the linearization valid by capping φ_xy.
                self.phi[0, k] >= -self.max_tilt,
                self.phi[0, k] <= self.max_tilt,
                self.phi[1, k] >= -self.max_tilt,
                self.phi[1, k] <= self.max_tilt,
                # Ground constraint.
                self.p[2, k] >= 0.0,
            ]
            if k > 0:
                # Soft descent-rate cap (skip k=0 because v[:,0] is fixed
                # by the boundary condition and may already exceed).
                constraints.append(
                    self.v[2, k] + self.v_max_desc + self.desc_slack[k] >= 0
                )

            # Running cost — regulator + actuator effort.
            cost = cost + q_pos_xy * (
                cp.square(self.p[0, k]) + cp.square(self.p[1, k])
            )
            cost = cost + q_pos_z * cp.square(self.p[2, k])
            cost = cost + q_vel_xy * (
                cp.square(self.v[0, k]) + cp.square(self.v[1, k])
            )
            cost = cost + q_vel_z * cp.square(self.v[2, k])
            cost = cost + q_phi * cp.sum_squares(self.phi[:, k])
            cost = cost + q_omega * cp.sum_squares(self.omega[:, k])
            cost = cost + r_thrust * cp.square(self.T[k] - T_bar)
            cost = cost + r_gimbal * (
                cp.square(self.gx[k]) + cp.square(self.gy[k])
            )
            cost = cost + w_desc_slack * cp.square(self.desc_slack[k])

        # Terminal cost.
        cost = cost + q_final_pos_xy * (
            cp.square(self.p[0, self.n]) + cp.square(self.p[1, self.n])
        )
        cost = cost + q_final_pos_z * cp.square(self.p[2, self.n])
        cost = cost + q_final_vel_xy * (
            cp.square(self.v[0, self.n]) + cp.square(self.v[1, self.n])
        )
        cost = cost + q_final_vel_z * cp.square(self.v[2, self.n])
        cost = cost + q_final_phi * cp.sum_squares(self.phi[:, self.n])
        cost = cost + q_final_omega * cp.sum_squares(self.omega[:, self.n])

        # Terminal state — gentle descent at touchdown.
        constraints += [
            self.p[2, self.n] >= 0.0,
            self.v[2, self.n] >= -0.35,
        ]

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_phi(q: np.ndarray) -> np.ndarray:
        """Convert a unit quaternion to its small-angle rotation vector.

        Uses the exact axis-angle formula ``φ = θ · n̂`` so the
        conversion stays accurate at moderate tilts; for q very near
        identity it reduces to ``2·q_vec``.
        """
        q = np.asarray(q, dtype=float)
        q_w = float(q[0])
        q_v = np.asarray(q[1:], dtype=float)
        nv = float(np.linalg.norm(q_v))
        if nv < 1e-9:
            return np.zeros(3)
        # θ_half = atan2(|q_v|, q_w); full angle = 2·θ_half.
        theta = 2.0 * float(np.arctan2(nv, q_w))
        return theta * q_v / nv

    # ------------------------------------------------------------------
    def plan(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        q: np.ndarray | None = None,
        omega: np.ndarray | None = None,
        **_ignored,  # accept and ignore accel_bias from wrappers
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        cp = self.cp
        self.p0.value = np.asarray(pos, dtype=float)
        self.v0.value = np.asarray(vel, dtype=float)
        if q is None:
            self.phi0.value = np.zeros(3)
        else:
            self.phi0.value = self._quat_to_phi(q)
        if omega is None:
            self.omega0.value = np.zeros(3)
        else:
            self.omega0.value = np.asarray(omega, dtype=float)

        try:
            self.problem.solve(solver="CLARABEL", warm_start=True, verbose=False)
        except Exception:
            try:
                self.problem.solve(
                    solver="SCS", warm_start=True, verbose=False, max_iters=500
                )
            except Exception:
                return None

        if self.problem.status not in ("optimal", "optimal_inaccurate"):
            return None
        if self.p.value is None or self.v.value is None or self.T.value is None:
            return None

        p_arr = np.asarray(self.p.value, dtype=float)
        v_arr = np.asarray(self.v.value, dtype=float)
        phi_arr = np.asarray(self.phi.value, dtype=float)
        T_arr = np.asarray(self.T.value, dtype=float)

        if not (
            np.all(np.isfinite(p_arr))
            and np.all(np.isfinite(v_arr))
            and np.all(np.isfinite(phi_arr))
            and np.all(np.isfinite(T_arr))
        ):
            return None

        # Reconstruct the world-frame thrust-acceleration trajectory the
        # wrapper expects: u_k = (g·φ_y, −g·φ_x, T_k/m).
        u_arr = np.zeros((3, self.n))
        u_arr[0, :] = self.g * phi_arr[1, :self.n]
        u_arr[1, :] = -self.g * phi_arr[0, :self.n]
        u_arr[2, :] = T_arr / self.m

        return p_arr.copy(), v_arr.copy(), u_arr.copy()

    def acceleration(self, pos: np.ndarray, vel: np.ndarray) -> np.ndarray:
        plan = self.plan(pos, vel)
        if plan is None:
            return np.array([0.0, 0.0, self.g])
        _, _, u = plan
        return np.asarray(u[:, 0], dtype=float).copy()


# ============================================================================
# Wrapper — Step 3 sibling of LandingActuatorAwareWaypointPID etc.
# Reuses the same lookahead trick + HoverPID landing gate the
# point-mass / Step 1 / Step 2 wrappers ride on; the only new piece is
# that __call__ forwards the current attitude (quaternion + body
# angular velocity) into the MPC plan call so the 6-DOF MPC knows
# where it is starting from.
# ============================================================================


from .mpc import LandingCvxpyWaypointPID  # noqa: E402 (avoids circular at top)


class LandingScp6DofWaypointPID(LandingCvxpyWaypointPID):
    """Step 3 wrapper: linearized 6-DOF MPC underneath the existing
    waypoint / HoverPID landing-gate machinery."""

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        lookahead: int = 10,
        # 6-DOF MPC tunables (forwarded).
        mpc_max_tilt: float = np.deg2rad(30.0),
        v_max_desc: float = 4.0,
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
        self.mpc = CvxpyScp6DofMPC(
            vehicle,
            env,
            max_tilt=mpc_max_tilt,
            v_max_desc=v_max_desc,
        )

    def __call__(self, t: float, state: np.ndarray):
        pos, vel = state[dyn.POS], state[dyn.VEL]
        q, omega = state[dyn.QUAT], state[dyn.OMEGA]
        if t - self._last_plan_t >= self.replan_dt or not self._has_plan:
            self._last_plan_t = t
            plan = self.mpc.plan(pos, vel, q=q, omega=omega)
            if plan is not None:
                p, _v, _u = plan
                k = min(self.lookahead, p.shape[1] - 1)
                self._xy_ref = p[:2, k]
                self._has_plan = True

        z = float(pos[2])
        gate_alt = self.guidance.config.gate_alt
        low_blend = np.clip((gate_alt - z) / gate_alt, 0.0, 1.0)
        xy_target = (1.0 - low_blend) * self._xy_ref
        self.pid.target = self.guidance.update(t, state, xy_target=xy_target).target
        return self.pid(t, state)
