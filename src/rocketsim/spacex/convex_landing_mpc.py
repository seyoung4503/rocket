"""Convex (G-FOLD style) point-mass landing MPC.

This is the new, *from-scratch* MPC for the SpaceX-style stack.  It departs
from the regulator-style ``CvxpyPointMassMPC`` in three important ways
documented in ``docs/2026-05-29_1753_v1_spacex_style_design.md``:

  1. **Min-fuel objective** — minimize ``sum_k ||u[k]||_2`` (with strong
     terminal landing penalty), not ``q_pos * ||p||^2``.  Removes the
     smoothness bias that made the previous plans hover.

  2. **Shrinking horizon** — ``T_final`` is *fixed* at controller construction
     time. Each replan plans over the *remaining* mission duration
     ``T_final - sim_t``, so the optimizer's "land in X seconds" deadline
     does **not** recede with time.  Vehicle is forced to commit.

  3. **Glideslope position constraint** — ``||p_xy[k]|| <= tan(gamma) * p_z[k]``
     keeps the planned trajectory inside a cone from the pad, the classic
     G-FOLD safety constraint that prevents orbiting / bouncing.

The state-update is the same RK1 (Euler) integration as the existing
MPC, so the *dynamics model* itself is unchanged; only the cost function
and the constraint set differ.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..vehicle import Environment, Vehicle


@dataclass
class MpcPlan:
    """Output of the MPC: full discrete trajectory + the wall-clock time
    at which the plan was solved.  The tracker uses ``start_t`` and ``dt``
    to interpolate the reference at the current sim-time."""

    p: np.ndarray  # (3, N+1) — planned positions
    v: np.ndarray  # (3, N+1) — planned velocities
    u: np.ndarray  # (3, N)   — planned thrust accelerations (control)
    start_t: float  # sim time at which the plan was solved
    dt: float       # plan step size (s)

    @property
    def duration(self) -> float:
        return self.dt * self.u.shape[1]

    def at_time(
        self, tau: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Linearly interpolate (p, v, u) at plan time ``tau`` (= sim_t -
        start_t). Past the end of the plan, returns the terminal state +
        zero thrust deviation."""
        n_u = self.u.shape[1]
        idx_f = max(0.0, tau) / self.dt
        idx = int(idx_f)
        alpha = idx_f - idx
        # Position / velocity have N+1 columns (one extra terminal).
        if idx >= self.p.shape[1] - 1:
            return self.p[:, -1].copy(), self.v[:, -1].copy(), self.u[:, -1].copy()
        p_ref = (1.0 - alpha) * self.p[:, idx] + alpha * self.p[:, idx + 1]
        v_ref = (1.0 - alpha) * self.v[:, idx] + alpha * self.v[:, idx + 1]
        if idx >= n_u - 1:
            u_ref = self.u[:, -1].copy()
        else:
            u_ref = (1.0 - alpha) * self.u[:, idx] + alpha * self.u[:, idx + 1]
        return p_ref, v_ref, u_ref


class ConvexLandingMPC:
    """G-FOLD-style convex landing planner.

    Variables (cvxpy):
      * ``p[:, 0..N]``    — position state (m)
      * ``v[:, 0..N]``    — velocity state (m/s)
      * ``u[:, 0..N-1]``  — thrust-accel control (m/s^2, world frame)
      * ``s[0..N-1]``     — slack on ||u|| (lossless min-fuel relaxation)
      * ``soft[0..N-1]``  — *small* slack on glideslope to keep problem feasible
                            under noisy initial conditions; heavily penalized.

    Cost:
        w_fuel * sum_k s[k]                                   (min-fuel)
      + w_term_p * ||p[N]||_2^2                               (land on pad)
      + w_term_v * ||v[N]||_2^2                               (zero terminal vel)
      + w_soft * sum_k soft[k]^2                              (slack penalty)

    Constraints (per step k = 0..N-1):
        p[:, k+1] = p[:, k] + dt * v[:, k] + 0.5 * dt^2 * (u[:, k] + g)
        v[:, k+1] = v[:, k] + dt * (u[:, k] + g)
        ||u[:, k]||_2     <= s[k]               # lossless conv on magnitude
        s[k]              in [a_min, a_max]     # thrust bounds (acceleration)
        u[2, k]           >= 0                  # one-sided thrust
        ||u_xy[k]||       <= tan(tilt) * u_z    # gimbal-cone (tilt limit)
        ||p_xy[k]||       <= tan(glide) * p_z + soft[k]   # glideslope ★
        p[2, k]           >= 0                  # don't go underground
        v[2, k]           >= -v_max_desc        # max descent rate
        soft[k]           >= 0

    Boundary:
        p[:, 0] = p0,  v[:, 0] = v0     # current state (parameters)
        p[2, N] >= 0  (terminal landing penalized softly)
        v[2, N] >= -0.35  (gentle terminal vz)

    The horizon ``N`` is the integer number of steps remaining until
    ``T_final``, computed at each ``solve(t, state)`` call. The cvxpy
    problem is built once at the *maximum* horizon ``N_max`` (= ``T_final
    / dt``) and reused; the ``effective`` N is parametrized via a mask on
    cost terms that don't make sense beyond the deadline.

    Implementation simplification: instead of dynamically sizing the
    problem (which would force cvxpy to recompile), we rebuild the
    problem on each ``solve()`` call.  cvxpy's first-solve compile cost
    dominates only for very small N; for our 5 Hz replan and N≤50
    this is acceptable (the solve itself is the bottleneck).
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        T_final: float = 10.0,
        dt: float = 0.2,
        max_tilt: float = np.deg2rad(12.0),
        glideslope_deg: float = 30.0,
        v_max_desc: float = 4.0,
        # EDF can shut off (unlike SpaceX engines that require ≥40% throttle).
        # a_min=0 lets the optimizer plan free-fall segments, which is the
        # mathematically correct min-fuel solution for our TWR>1 vehicle.
        a_min_frac: float = 0.0,
        # Cost weights. Smaller terminal weights so min-fuel dominates the
        # in-flight choice; the optimizer should still arrive near the pad
        # because the constraints (glideslope + descent-rate + N steps to
        # reach z=0) force convergence.
        w_fuel: float = 1.0,
        w_term_p: float = 200.0,
        w_term_v: float = 200.0,
        w_soft: float = 1000.0,
        # Small running cost on z position so the optimizer prefers
        # descending earlier rather than bouncing up. This encodes
        # "land sooner = better" without sacrificing fuel optimality.
        w_running_z: float = 0.5,
    ):
        try:
            import cvxpy as cp
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("cvxpy is required for ConvexLandingMPC") from exc

        self.cp = cp
        self.vehicle = vehicle
        self.env = env
        self.T_final = float(T_final)
        self.dt = float(dt)
        self.max_tilt = float(max_tilt)
        self.glideslope_deg = float(glideslope_deg)
        self.v_max_desc = float(v_max_desc)

        # Acceleration bounds (m/s^2).  Note: a_min should leave room for
        # gravity cancellation, but for landing we let the lower bound
        # approach zero so descent at terminal velocity is allowed.
        a_max = vehicle.max_thrust / vehicle.mass
        a_min = a_min_frac * a_max
        self.a_max = float(a_max)
        self.a_min = float(a_min)

        self.w_fuel = float(w_fuel)
        self.w_term_p = float(w_term_p)
        self.w_term_v = float(w_term_v)
        self.w_soft = float(w_soft)
        self.w_running_z = float(w_running_z)

        # Hover acceleration (world frame) used as feed-forward floor.
        self._g_world = np.array([0.0, 0.0, -env.gravity])
        self._hover_acc = np.array([0.0, 0.0, env.gravity])
        self._tan_tilt = float(np.tan(max_tilt))
        self._tan_glide = float(np.tan(np.deg2rad(glideslope_deg)))

    # ------------------------------------------------------------------
    def solve(self, t: float, state: np.ndarray) -> MpcPlan | None:
        """Solve the convex landing MPC over the remaining mission time.

        Returns an ``MpcPlan`` (full (p, v, u) trajectory + start_t) on
        success, or ``None`` if the solver fails.
        """
        cp = self.cp
        remaining = self.T_final - t
        N = max(2, int(round(remaining / self.dt)))
        # Cap by a sensible upper bound so the problem doesn't grow if
        # the controller is called before t=0 etc.
        N = min(N, int(round(self.T_final / self.dt)) + 2)

        # ---- variables ----
        p = cp.Variable((3, N + 1))
        v = cp.Variable((3, N + 1))
        u = cp.Variable((3, N))
        s = cp.Variable(N, nonneg=True)          # ||u[:,k]|| <= s[k]
        soft = cp.Variable(N, nonneg=True)       # glideslope slack

        pos0 = np.asarray(state[:3], dtype=float)
        vel0 = np.asarray(state[3:6], dtype=float)

        dt = self.dt
        g = self._g_world

        constraints = [
            p[:, 0] == pos0,
            v[:, 0] == vel0,
        ]
        cost = 0
        for k in range(N):
            acc = u[:, k] + g
            constraints += [
                # discrete dynamics (Euler / midpoint position)
                p[:, k + 1] == p[:, k] + dt * v[:, k] + 0.5 * dt * dt * acc,
                v[:, k + 1] == v[:, k] + dt * acc,
                # lossless-conv magnitude bound
                cp.norm(u[:, k], 2) <= s[k],
                s[k] >= self.a_min,
                s[k] <= self.a_max,
                # one-sided thrust + tilt-cone
                u[2, k] >= 0,
                cp.norm(u[:2, k], 2) <= self._tan_tilt * u[2, k],
                # glideslope (with small slack)
                cp.norm(p[:2, k], 2) <= self._tan_glide * p[2, k] + soft[k],
                # ground & descent-rate
                p[2, k] >= 0.0,
                v[2, k] >= -self.v_max_desc,
            ]
            cost = cost + self.w_fuel * s[k] + self.w_soft * cp.square(soft[k])
            # Running cost on altitude — encourages descent throughout
            # the trajectory instead of climbing-then-falling.
            cost = cost + self.w_running_z * cp.square(p[2, k])

        # Terminal state — soft landing on pad.
        constraints += [
            p[2, N] >= 0.0,
            v[2, N] >= -0.35,
        ]
        cost = cost + self.w_term_p * cp.sum_squares(p[:, N])
        cost = cost + self.w_term_v * cp.sum_squares(v[:, N])

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver="CLARABEL", verbose=False)
        except Exception:
            try:
                prob.solve(solver="SCS", verbose=False, max_iters=500)
            except Exception:
                return None

        if prob.status not in ("optimal", "optimal_inaccurate"):
            return None
        if p.value is None or v.value is None or u.value is None:
            return None
        p_arr = np.asarray(p.value, dtype=float)
        v_arr = np.asarray(v.value, dtype=float)
        u_arr = np.asarray(u.value, dtype=float)
        if not (
            np.all(np.isfinite(p_arr))
            and np.all(np.isfinite(v_arr))
            and np.all(np.isfinite(u_arr))
        ):
            return None
        return MpcPlan(p=p_arr, v=v_arr, u=u_arr, start_t=float(t), dt=self.dt)
