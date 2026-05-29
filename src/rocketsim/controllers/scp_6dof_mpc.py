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


# ============================================================================
# Step 3 v2 — Warm-start SCP: linearize R(q)·ê_z around the *previous
# plan's attitude trajectory*, not around the fixed vertical hover.
#
# Implementation pattern (a single SCP iteration per replan, with the
# previous plan acting as the linearization reference):
#
#   1. Store q_bar[0..N] (per-step reference quaternion) between solves.
#   2. At each replan, recompute n̂_bar[k] = R(q̄[k])·ê_z and
#      M_φ_bar[k] = R(q̄[k]) · skew_ez (the sensitivity of u to φ).
#   3. φ in the cvxpy model is the *rotation from q̄[k] to the actual
#      q[k]*, so it stays small even when the absolute attitude is large.
#   4. After the solve, compose q_bar_new[k] = q̄[k] ⊗ quat_from_phi(φ*[k])
#      and shift forward one step for the receding-window warm start of
#      the next replan.
#   5. First call: q̄ = identity (matches single-shot Step 3); subsequent
#      calls track the plan's actual attitude trajectory.
#
# Compared to single-shot Step 3, only the R(q)·ê_z linearization is
# updated.  The torque model is kept simple (T·gimbal linearized around
# the hover reference) so the cvxpy structure changes minimally.
# ============================================================================


class CvxpyScpWarm6DofMPC(CvxpyScp6DofMPC):
    """Step 3 with warm-start linearization around the previous plan.

    Extends ``CvxpyScp6DofMPC`` by promoting the body-z direction n̂_bar
    and its sensitivity matrix M_φ_bar from compile-time constants to
    cvxpy parameters, updated each ``plan()`` call from the previous
    iteration's planned quaternion trajectory.  When the predicted plan
    matches reality the linearization is exact; when it doesn't,
    repeated solves drift it toward the true non-linear dynamics
    (full SCP is just this with an inner-iteration loop; here we do a
    single iteration per replan and let the receding window carry the
    fixed point).
    """

    # ------------------------------------------------------------------
    # Quaternion helpers (kept local so this class has no extra deps).
    @staticmethod
    def _quat_conj(q: np.ndarray) -> np.ndarray:
        return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

    @staticmethod
    def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        # Hamilton product, [w, x, y, z] convention.
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ],
            dtype=float,
        )

    @staticmethod
    def _phi_to_quat(phi: np.ndarray) -> np.ndarray:
        """Axis-angle rotation vector → unit quaternion."""
        phi = np.asarray(phi, dtype=float)
        theta = float(np.linalg.norm(phi))
        if theta < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])
        axis = phi / theta
        half = 0.5 * theta
        return np.array(
            [np.cos(half), axis[0] * np.sin(half), axis[1] * np.sin(half),
             axis[2] * np.sin(half)],
            dtype=float,
        )

    @staticmethod
    def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=float,
        )

    # ------------------------------------------------------------------
    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        # Build the cvxpy problem with parametric linearization
        # coefficients, replacing the parent's compile-time
        # ``g·φ_y, −g·φ_x, T/m − g`` substitutions.
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError(
                "cvxpy is required for CvxpyScpWarm6DofMPC"
            ) from exc

        # Pull the configurable knobs out so we can override the parent's
        # __init__ entirely.  Keep argument order/defaults aligned with
        # CvxpyScp6DofMPC.__init__.
        horizon = kwargs.pop("horizon", 4.0)
        dt = kwargs.pop("dt", 0.2)
        max_tilt = kwargs.pop("max_tilt", np.deg2rad(30.0))
        v_max_desc = kwargs.pop("v_max_desc", 4.0)
        q_pos_xy = kwargs.pop("q_pos_xy", 1.2)
        q_pos_z = kwargs.pop("q_pos_z", 0.08)
        q_vel_xy = kwargs.pop("q_vel_xy", 4.0)
        q_vel_z = kwargs.pop("q_vel_z", 3.0)
        q_phi = kwargs.pop("q_phi", 5.0)
        q_omega = kwargs.pop("q_omega", 0.5)
        q_final_pos_xy = kwargs.pop("q_final_pos_xy", 45.0)
        q_final_pos_z = kwargs.pop("q_final_pos_z", 18.0)
        q_final_vel_xy = kwargs.pop("q_final_vel_xy", 80.0)
        q_final_vel_z = kwargs.pop("q_final_vel_z", 180.0)
        q_final_phi = kwargs.pop("q_final_phi", 100.0)
        q_final_omega = kwargs.pop("q_final_omega", 50.0)
        r_thrust = kwargs.pop("r_thrust", 0.02)
        r_gimbal = kwargs.pop("r_gimbal", 0.05)
        w_desc_slack = kwargs.pop("w_desc_slack", 1000.0)
        if kwargs:
            raise TypeError(f"unexpected kwargs: {list(kwargs)}")

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
        I_diag = np.asarray(vehicle.inertia, dtype=float)
        if I_diag.ndim == 2:
            I_diag = np.diag(I_diag)
        I_diag_safe = np.where(I_diag > 1e-9, I_diag, 1.0)
        self.m = float(m)
        self.g = float(g_val)
        self.L = float(L)
        self.I_diag = I_diag.copy()
        self._I_inv_safe = 1.0 / I_diag_safe
        self.T_max = float(vehicle.max_thrust)
        self.gimbal_limit = float(vehicle.gimbal_limit)
        self._tau_coeff = m * g_val * L

        # Skew matrix that, applied to a body-frame small-angle vector φ,
        # produces the body-frame perturbation of ê_z:  [φ]×·ê_z =
        # (φ_y, −φ_x, 0).  In matrix form this is constant.
        self._skew_ez = np.array(
            [[0.0, 1.0, 0.0],
             [-1.0, 0.0, 0.0],
             [0.0, 0.0, 0.0]],
            dtype=float,
        )
        T_bar = m * g_val
        cw_x = self._tau_coeff / I_diag_safe[0]
        cw_y = self._tau_coeff / I_diag_safe[1]

        # Cvxpy variables.
        self.p0 = cp.Parameter(3)
        self.v0 = cp.Parameter(3)
        self.phi0 = cp.Parameter(3)
        self.omega0 = cp.Parameter(3)

        # Time-varying linearization references.  Each step has its own
        # parameter so the optimizer can be re-used across calls with a
        # single cvxpy compile.
        self.n_hat_bar = [cp.Parameter(3) for _ in range(self.n)]
        self.M_phi_bar = [cp.Parameter((3, 3)) for _ in range(self.n)]

        self.p = cp.Variable((3, self.n + 1))
        self.v = cp.Variable((3, self.n + 1))
        self.phi = cp.Variable((3, self.n + 1))
        self.omega = cp.Variable((3, self.n + 1))
        self.T = cp.Variable(self.n)
        self.gx = cp.Variable(self.n)
        self.gy = cp.Variable(self.n)
        self.desc_slack = cp.Variable(self.n, nonneg=True)

        constraints = [
            self.p[:, 0] == self.p0,
            self.v[:, 0] == self.v0,
            self.phi[:, 0] == self.phi0,
            self.omega[:, 0] == self.omega0,
        ]
        g_world = np.array([0.0, 0.0, -g_val])

        cost = 0
        for k in range(self.n):
            # World-frame thrust acceleration, linearized around q̄(t):
            #     u = (T/m)·n̂_bar + (T_bar/m)·M_φ_bar·φ
            # Plus gravity to get total acceleration.
            acc = (
                self.T[k] / m * self.n_hat_bar[k]
                + (T_bar / m) * (self.M_phi_bar[k] @ self.phi[:, k])
                + g_world
            )

            constraints += [
                self.p[:, k + 1]
                == self.p[:, k] + dt * self.v[:, k] + 0.5 * dt * dt * acc,
                self.v[:, k + 1] == self.v[:, k] + dt * acc,
                # Attitude kinematics.  In the rotating reference frame
                # of q̄(t), the body angular velocity ω drives φ — for
                # short horizons this stays a good first-order approx
                # even though q̄ itself is time varying.
                self.phi[:, k + 1] == self.phi[:, k] + dt * self.omega[:, k],
                self.omega[0, k + 1]
                == self.omega[0, k] + dt * cw_x * (-self.gy[k]),
                self.omega[1, k + 1]
                == self.omega[1, k] + dt * cw_y * self.gx[k],
                self.omega[2, k + 1] == self.omega[2, k],
                self.T[k] >= 0,
                self.T[k] <= self.T_max,
                self.gx[k] >= -self.gimbal_limit,
                self.gx[k] <= self.gimbal_limit,
                self.gy[k] >= -self.gimbal_limit,
                self.gy[k] <= self.gimbal_limit,
                self.phi[0, k] >= -self.max_tilt,
                self.phi[0, k] <= self.max_tilt,
                self.phi[1, k] >= -self.max_tilt,
                self.phi[1, k] <= self.max_tilt,
                self.p[2, k] >= 0.0,
            ]
            if k > 0:
                constraints.append(
                    self.v[2, k] + self.v_max_desc + self.desc_slack[k] >= 0
                )

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

        constraints += [
            self.p[2, self.n] >= 0.0,
            self.v[2, self.n] >= -0.35,
        ]

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

        # Reference quaternion trajectory (N+1 entries; identity = vertical
        # for the very first solve).
        self._ref_q = np.tile(
            np.array([1.0, 0.0, 0.0, 0.0]), (self.n + 1, 1)
        )

    # ------------------------------------------------------------------
    def _update_linearization(self) -> None:
        """Recompute n̂_bar[k] and M_φ_bar[k] from the stored q̄ trajectory."""
        for k in range(self.n):
            R = self._quat_to_rotmat(self._ref_q[k])
            self.n_hat_bar[k].value = R @ np.array([0.0, 0.0, 1.0])
            self.M_phi_bar[k].value = R @ self._skew_ez

    # ------------------------------------------------------------------
    def plan(
        self,
        pos: np.ndarray,
        vel: np.ndarray,
        q: np.ndarray | None = None,
        omega: np.ndarray | None = None,
        **_ignored,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        cp = self.cp
        self.p0.value = np.asarray(pos, dtype=float)
        self.v0.value = np.asarray(vel, dtype=float)
        if q is None:
            q_actual = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            q_actual = np.asarray(q, dtype=float)
        if omega is None:
            self.omega0.value = np.zeros(3)
        else:
            self.omega0.value = np.asarray(omega, dtype=float)

        # Initial φ = rotation FROM q_bar[0] TO actual q.
        q_delta0 = self._quat_mul(self._quat_conj(self._ref_q[0]), q_actual)
        phi0_val = self._quat_to_phi(q_delta0)

        # ★ Trust region: if the reference quaternion at the head of the
        # receding window has drifted too far from reality (e.g., the
        # divert event just teleported the pad and the old plan is
        # describing a totally different trajectory), the linearization
        # around q̄ becomes inaccurate.  Reset the whole reference
        # trajectory to the current attitude so the next solve starts
        # from a fresh small-angle expansion.  Threshold tuned slightly
        # below max_tilt so we trip before the linearization error
        # becomes large enough to corrupt the plan.
        if float(np.linalg.norm(phi0_val)) > 0.30:  # ~17 deg
            self._ref_q[:] = q_actual
            phi0_val = np.zeros(3)
        self.phi0.value = phi0_val

        # Push the current q̄ trajectory into the cvxpy parameters.
        self._update_linearization()

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

        # Reconstruct world-frame thrust acceleration.  In this class the
        # linearization is around (T_bar=m·g, q̄(t)) so the world thrust
        # direction at step k is n̂_bar[k] + M_φ_bar[k]·φ[k]:
        u_arr = np.zeros((3, self.n))
        for k in range(self.n):
            n_hat = np.asarray(self.n_hat_bar[k].value, dtype=float)
            M_phi = np.asarray(self.M_phi_bar[k].value, dtype=float)
            dir_world = n_hat + M_phi @ phi_arr[:, k]
            u_arr[:, k] = T_arr[k] / self.m * dir_world

        # Update q̄ trajectory for the next solve.
        new_ref_q = np.empty_like(self._ref_q)
        for k in range(self.n + 1):
            q_delta = self._phi_to_quat(phi_arr[:, k])
            new_ref_q[k] = self._quat_mul(self._ref_q[k], q_delta)
        # Receding-window shift: drop step 0 (which is now in the past)
        # and pad with the terminal state for the tail.
        self._ref_q[:-1] = new_ref_q[1:]
        self._ref_q[-1] = new_ref_q[-1]

        return p_arr.copy(), v_arr.copy(), u_arr.copy()


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


class LandingScpWarm6DofWaypointPID(LandingScp6DofWaypointPID):
    """Wrapper for the warm-start SCP variant.  Identical structure to
    the single-shot wrapper (lookahead=10 + HoverPID landing gate);
    differs only in that the MPC underneath is the iterative-
    linearization variant ``CvxpyScpWarm6DofMPC``."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, **kwargs)
        # Swap the MPC built by the parent for the warm-start variant.
        mpc_kwargs = {}
        if "mpc_max_tilt" in kwargs:
            mpc_kwargs["max_tilt"] = kwargs["mpc_max_tilt"]
        if "v_max_desc" in kwargs:
            mpc_kwargs["v_max_desc"] = kwargs["v_max_desc"]
        self.mpc = CvxpyScpWarm6DofMPC(vehicle, env, **mpc_kwargs)
