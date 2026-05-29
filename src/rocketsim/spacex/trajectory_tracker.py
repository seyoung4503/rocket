"""Time-indexed trajectory tracker.

Consumes the MPC's full ``MpcPlan`` (p(t), v(t), u(t)) and at every
controller step computes a desired thrust-acceleration via

    a_des(t) = u_ref(tau) + K_p @ (p_ref(tau) - pos)
                          + K_v @ (v_ref(tau) - vel)
                          + K_i @ ∫ (p_ref - pos) dt   (anti-windup clipped)

where ``tau = t - plan.start_t`` is the current plan time. This is the
classical reference-tracking pattern that the lookahead/setpoint hack was
short-circuiting.

No safety overrides, no descent ladder, no `lookahead` / `anticipation`
knobs — those used to live here in the old wrappers; in this stack the
*MPC's constraints* (glideslope, descent-rate, terminal landing) take
care of safety, and the *tracker* just closes the loop on the plan.

Gains default to the HoverPID-tuned values for compatibility; they can
be retuned per scenario if needed.
"""

from __future__ import annotations

import numpy as np

from .convex_landing_mpc import MpcPlan


class TrajectoryTracker:
    def __init__(
        self,
        kp_pos: tuple[float, float, float] = (1.6, 1.6, 6.0),
        kd_pos: tuple[float, float, float] = (2.4, 2.4, 4.0),
        ki_pos: tuple[float, float, float] = (0.6, 0.6, 1.5),
        xy_int_limit: float = 3.0,
        z_int_limit: float = 5.0,
    ):
        self.kp = np.asarray(kp_pos, dtype=float)
        self.kd = np.asarray(kd_pos, dtype=float)
        self.ki = np.asarray(ki_pos, dtype=float)
        self.xy_int_limit = float(xy_int_limit)
        self.z_int_limit = float(z_int_limit)
        self._int = np.zeros(3)
        self._last_t = 0.0

    def reset(self, t: float = 0.0) -> None:
        self._int[:] = 0.0
        self._last_t = float(t)

    def step(
        self,
        t: float,
        pos: np.ndarray,
        vel: np.ndarray,
        plan: MpcPlan,
    ) -> np.ndarray:
        """Compute desired thrust acceleration (m/s^2, world frame) at sim
        time ``t`` given the current ``plan``.

        The reference is the plan interpolated at ``tau = t - plan.start_t``.
        Past the plan's horizon, the reference is the terminal state +
        terminal thrust (which should be near hover by construction).
        """
        tau = max(0.0, t - plan.start_t)
        p_ref, v_ref, u_ref = plan.at_time(tau)

        # Integrator update with anti-windup.
        dt = max(t - self._last_t, 0.0)
        self._last_t = t
        err = p_ref - np.asarray(pos, dtype=float)
        self._int += err * dt
        self._int[:2] = np.clip(self._int[:2], -self.xy_int_limit, self.xy_int_limit)
        self._int[2] = float(np.clip(self._int[2], -self.z_int_limit, self.z_int_limit))

        v_err = v_ref - np.asarray(vel, dtype=float)
        a_des = (
            u_ref
            + self.kp * err
            + self.kd * v_err
            + self.ki * self._int
        )
        return a_des
