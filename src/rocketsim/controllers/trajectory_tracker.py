"""SpaceX-style time-indexed trajectory tracker for MPC plans.

Unlike ``LandingCvxpyWaypointPID`` (which samples the MPC plan at a single
``lookahead`` step and passes only XY position to a setpoint-tracking PID),
this controller closes the loop on the **current plan time**: at each step
it interpolates ``(p_ref, v_ref, u_ff)`` from the MPC plan at
``tau = sim_t - plan_t0``, applies a PD correction on position and
velocity error, and uses the MPC's planned thrust acceleration as
feedforward.

Architecturally this is closer to a literature G-FOLD / convex landing
controller. The ``lookahead`` knob disappears: the simulator clock decides
where on the trajectory the inner loop tracks. See
``docs/2026-05-29_1653_v1_lookahead_vs_spacex_design.md``.

The class accepts any planner with the ``CvxpyPointMassMPC.plan(pos, vel)``
shape (returning ``(p, v, u)`` arrays of size ``(3, n+1)`` and ``(3, n)``),
so the same wrapper can drive point-mass, Step 1, Step 2, or (later)
Step 3 MPCs.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from .. import dynamics as dyn
from ..dynamics import Command
from ..vehicle import Environment, Vehicle
from ..guidance.landing import LandingGuidance, LandingGuidanceConfig
from .mpc import (
    CvxpyActuatorAwareMagLagMPC,
    CvxpyActuatorAwareMPC,
    CvxpyPointMassMPC,
    LandingCvxpyMPC,
)


class LandingTrajectoryTrackingMPC(LandingCvxpyMPC):
    """Time-indexed reference tracker around a configurable convex planner.

    State per instance:
      * ``_plan_p, _plan_v, _plan_u`` — most recent MPC plan
      * ``_plan_t0`` — simulator time at which the plan was made
      * ``_last_plan_t`` (inherited) — replan timing

    Per step the controller interpolates the plan at ``tau = t - plan_t0``
    (clipped to the plan horizon) and computes
        ``a_des = u_ff + K_p * (p_ref - pos) + K_d * (v_ref - vel)``
    then converts ``a_des`` to a ``Command`` via the inherited attitude
    PID + thrust-vector inverse map.

    No landing gate / hover override — the MPC plan owns the descent
    schedule. (If the MPC plan is wrong, the tracker will faithfully follow
    it; this is intentional for a clean MPC-vs-PID comparison.)
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        planner: Callable[[Vehicle, Environment], object] | None = None,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        # PD gains on position/velocity error in world frame. The defaults
        # match LandingCvxpyGuidancePID, which has been tuned on the
        # existing scenarios; tune from here if needed.
        kp_track: tuple[float, float, float] = (0.9, 0.9, 1.4),
        kd_track: tuple[float, float, float] = (1.8, 1.8, 2.2),
        # Forward-time offset (s) applied to the reference index. Small
        # positive values give the tracker a slight lead time on the plan,
        # which can compensate for actuator lag. 0.0 = exact current time.
        anticipation: float = 0.0,
        # Maximum tilt the underlying MPC plans for (separate from the
        # attitude-PID tilt limit). 12° matches the gimbal cone.
        mpc_max_tilt: float = np.deg2rad(12.0),
        use_bias_estimator: bool = False,
    ):
        super().__init__(
            vehicle,
            env,
            replan_dt=replan_dt,
            max_tilt=max_tilt,
            use_bias_estimator=use_bias_estimator,
        )
        # Replace the parent's default point-mass planner with the one we
        # were given (or keep point-mass if planner is None).
        if planner is None:
            self.mpc = CvxpyPointMassMPC(vehicle, env, max_tilt=mpc_max_tilt)
        else:
            self.mpc = planner(vehicle, env)
        self.kp_track = np.asarray(kp_track, dtype=float)
        self.kd_track = np.asarray(kd_track, dtype=float)
        self.anticipation = float(anticipation)
        self._plan_p: np.ndarray | None = None
        self._plan_v: np.ndarray | None = None
        self._plan_u: np.ndarray | None = None
        self._plan_t0: float = -np.inf

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos, vel = state[dyn.POS], state[dyn.VEL]
        if self.use_bias_estimator:
            self._update_bias_estimate(t, state)

        # Replan periodically. The plan's t=0 is "now" so we record plan_t0.
        if t - self._last_plan_t >= self.replan_dt or self._plan_p is None:
            self._last_plan_t = t
            bias = self._bias if self.use_bias_estimator else None
            plan = self.mpc.plan(pos, vel, accel_bias=bias)
            if plan is not None:
                self._plan_p, self._plan_v, self._plan_u = plan
                self._plan_t0 = t

        # No plan yet: hover.
        if self._plan_p is None:
            hover = np.array([0.0, 0.0, self.env.gravity])
            return self._command_from_thrust_accel(t, state, hover)

        # ★ Time-indexed reference. The plan was made at plan_t0 with the
        # vehicle at p[:,0]. tau seconds later, the plan says we should be
        # at p[:, tau/dt] with velocity v[:, tau/dt] and thrust u[:, tau/dt].
        tau = max(0.0, t - self._plan_t0 + self.anticipation)
        plan_dt = self.mpc.dt
        idx_f = tau / plan_dt
        idx = int(idx_f)
        alpha = idx_f - idx
        n_p = self._plan_p.shape[1] - 1  # last valid index in p (and v)
        n_u = self._plan_u.shape[1] - 1  # last valid index in u

        if idx >= n_p:
            p_ref = self._plan_p[:, -1]
            v_ref = self._plan_v[:, -1]
        else:
            p_ref = (1.0 - alpha) * self._plan_p[:, idx] + alpha * self._plan_p[:, idx + 1]
            v_ref = (1.0 - alpha) * self._plan_v[:, idx] + alpha * self._plan_v[:, idx + 1]

        if idx >= n_u:
            u_ff = self._plan_u[:, -1] if self._plan_u.shape[1] > 0 \
                else np.array([0.0, 0.0, self.env.gravity])
        else:
            u_ff = (1.0 - alpha) * self._plan_u[:, idx] + alpha * self._plan_u[:, idx + 1]

        # Feedforward thrust + PD feedback. Note that u_ff is already in
        # world-frame thrust acceleration (m/s^2), matching what
        # _command_from_thrust_accel expects.
        accel = u_ff + self.kp_track * (p_ref - pos) + self.kd_track * (v_ref - vel)
        return self._command_from_thrust_accel(t, state, accel)


# ----------------------------------------------------------------------------
# Convenience wrappers for the three MPC variants we want to A/B against the
# corresponding lookahead-based wrappers (LandingCvxpyWaypointPID,
# LandingActuatorAwareWaypointPID, LandingActuatorAwareMagLagWaypointPID).
# ----------------------------------------------------------------------------


def _pointmass_factory(vehicle: Vehicle, env: Environment):
    return CvxpyPointMassMPC(vehicle, env, max_tilt=np.deg2rad(12.0))


def _actuator_factory(vehicle: Vehicle, env: Environment):
    return CvxpyActuatorAwareMPC(vehicle, env, max_tilt=np.deg2rad(12.0))


def _actuator2_factory(vehicle: Vehicle, env: Environment):
    return CvxpyActuatorAwareMagLagMPC(vehicle, env, max_tilt=np.deg2rad(12.0))


class LandingPointMassTrackingMPC(LandingTrajectoryTrackingMPC):
    """Time-indexed tracker over the base point-mass MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_pointmass_factory, **kwargs)


class LandingActuatorTrackingMPC(LandingTrajectoryTrackingMPC):
    """Time-indexed tracker over the Step 1 actuator-aware MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_actuator_factory, **kwargs)


class LandingActuatorMagLagTrackingMPC(LandingTrajectoryTrackingMPC):
    """Time-indexed tracker over the Step 2 actuator-aware + mag-lag MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_actuator2_factory, **kwargs)


# ============================================================================
# Full SpaceX-style tracker = time-indexed reference (anticipation=0) +
# position integrator (anti-windup) + LandingGuidance descent gate +
# touchdown commit.  This is the "no-hack" version that proper GNC stacks
# look like in literature (Blackmore, Açıkmeşe G-FOLD).
#
# Compared to LandingTrajectoryTrackingMPC (bare) and to the
# `lookahead`-based wrappers, this adds:
#   * xy / z position integrator with anti-windup (steady-state wind /
#     plant-mismatch rejection)
#   * LandingGuidance z setpoint override near the pad (forces descent
#     commit when the MPC plan is too smooth)
#   * touchdown-ready logic (lateral damping + sub-hover thrust at final
#     contact to soften impact)
#
# See docs/2026-05-29_1712_v1_trajectory_tracking_analysis.md for why the
# bare tracker (without these layers) failed — turns out the lookahead
# wrapper was bundling several pieces of GNC machinery, not just sampling.
# ============================================================================


class LandingTrajectoryTrackingFullMPC(LandingCvxpyMPC):
    """Full trajectory tracker = bare tracker + integrator + landing gate.

    Architectural layers:
      1. **Guidance** — MPC plan (time-indexed) + ``LandingGuidance`` descent
         ladder near the pad.
      2. **Outer control** — PD + I on (p_ref - pos, v_ref - vel) error with
         anti-windup clamps on the integrator state.
      3. **Inner control** — attitude PID (inherited from ``LandingCvxpyMPC``).
      4. **Touchdown commit** — when ``guidance.touchdown_ready()`` fires,
         dampen lateral acceleration and command sub-hover thrust for a
         soft contact.

    No ``lookahead`` knob and no ``anticipation`` knob: tracking is true
    current-time. Anti-windup limits match ``HoverPID`` defaults.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        planner: Callable[[Vehicle, Environment], object] | None = None,
        replan_dt: float = 0.2,
        max_tilt: float = np.deg2rad(20.0),
        # 2026-05-29 raised from (0.9, 0.9, 1.4)/(1.8, 1.8, 2.2) to match
        # HoverPID's tuned values. The earlier defaults (which came from
        # LandingCvxpyGuidancePID) were too soft: with a time-indexed
        # reference the error is small per step, so soft gains never
        # commanded enough descent → vehicle hovered → timeout.
        # HoverPID: kp_pos=1.6, kd_pos=2.4, kp_z=6.0, kd_z=4.0.
        kp_track: tuple[float, float, float] = (1.6, 1.6, 6.0),
        kd_track: tuple[float, float, float] = (2.4, 2.4, 4.0),
        # Position integrator gains. Match HoverPID: ki_pos=0.6, ki_z=1.5.
        ki_xy: float = 0.6,
        ki_z: float = 1.5,
        xy_int_limit: float = 3.0,
        z_int_limit: float = 5.0,
        # MPC plan's tilt cone (separate from attitude PID's tilt budget).
        mpc_max_tilt: float = np.deg2rad(12.0),
        # Reuse LandingGuidance for the descent ladder + touchdown commit.
        guidance_config: LandingGuidanceConfig | None = None,
        use_bias_estimator: bool = False,
    ):
        super().__init__(
            vehicle,
            env,
            replan_dt=replan_dt,
            max_tilt=max_tilt,
            use_bias_estimator=use_bias_estimator,
        )
        if planner is None:
            self.mpc = CvxpyActuatorAwareMPC(vehicle, env, max_tilt=mpc_max_tilt)
        else:
            self.mpc = planner(vehicle, env)
        self.kp_track = np.asarray(kp_track, dtype=float)
        self.kd_track = np.asarray(kd_track, dtype=float)
        self.ki_xy = float(ki_xy)
        self.ki_z = float(ki_z)
        self.xy_int_limit = float(xy_int_limit)
        self.z_int_limit = float(z_int_limit)
        self.guidance = LandingGuidance(guidance_config or LandingGuidanceConfig())
        # State
        self._plan_p: np.ndarray | None = None
        self._plan_v: np.ndarray | None = None
        self._plan_u: np.ndarray | None = None
        self._plan_t0: float = -np.inf
        self._xy_int = np.zeros(2)
        self._z_int = 0.0
        self._last_track_t = 0.0

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos, vel = state[dyn.POS], state[dyn.VEL]
        if self.use_bias_estimator:
            self._update_bias_estimate(t, state)

        # Replan periodically.
        if t - self._last_plan_t >= self.replan_dt or self._plan_p is None:
            self._last_plan_t = t
            bias = self._bias if self.use_bias_estimator else None
            plan = self.mpc.plan(pos, vel, accel_bias=bias)
            if plan is not None:
                self._plan_p, self._plan_v, self._plan_u = plan
                self._plan_t0 = t

        # Always update guidance (state machine keeps running even if MPC
        # planning fails on a step).
        guidance_out = self.guidance.update(t, state, xy_target=np.zeros(2))

        if self._plan_p is None:
            hover = np.array([0.0, 0.0, self.env.gravity])
            return self._command_from_thrust_accel(t, state, hover)

        # Time-indexed reference at current plan time. NO anticipation.
        tau = max(0.0, t - self._plan_t0)
        plan_dt = self.mpc.dt
        idx_f = tau / plan_dt
        idx = int(idx_f)
        alpha = idx_f - idx
        n_p = self._plan_p.shape[1] - 1
        n_u = self._plan_u.shape[1] - 1

        if idx >= n_p:
            p_ref = self._plan_p[:, -1].copy()
            v_ref = self._plan_v[:, -1].copy()
        else:
            p_ref = (1.0 - alpha) * self._plan_p[:, idx] + alpha * self._plan_p[:, idx + 1]
            v_ref = (1.0 - alpha) * self._plan_v[:, idx] + alpha * self._plan_v[:, idx + 1]

        if idx >= n_u:
            u_ff = self._plan_u[:, -1] if self._plan_u.shape[1] > 0 \
                else np.array([0.0, 0.0, self.env.gravity])
        else:
            u_ff = (1.0 - alpha) * self._plan_u[:, idx] + alpha * self._plan_u[:, idx + 1]

        # ★ z setpoint override. Below 4 m, or whenever the guidance ladder is
        # asking for a lower z than the MPC plan is providing, take the lower
        # of the two. This forces the tracker to commit to descent even when
        # the MPC plan is smoothly hovering. Mirrors LandingCvxpyGuidancePID.
        if pos[2] < 4.0 or guidance_out.z_setpoint < p_ref[2]:
            p_ref[2] = max(p_ref[2], guidance_out.z_setpoint)
            # Cap upward velocity reference so a bouncing ref doesn't push
            # the vehicle back up after it's committed.
            if p_ref[2] > pos[2] - 0.15:
                v_ref[2] = min(v_ref[2], 0.0)

        # Position integrator with anti-windup.
        dt = max(t - self._last_track_t, 0.0)
        self._last_track_t = t
        p_err = p_ref - pos
        v_err = v_ref - vel
        self._xy_int = np.clip(
            self._xy_int + p_err[:2] * dt, -self.xy_int_limit, self.xy_int_limit
        )
        self._z_int = float(
            np.clip(self._z_int + p_err[2] * dt, -self.z_int_limit, self.z_int_limit)
        )

        accel = (
            u_ff
            + self.kp_track * p_err
            + self.kd_track * v_err
            + np.array(
                [
                    self.ki_xy * self._xy_int[0],
                    self.ki_xy * self._xy_int[1],
                    self.ki_z * self._z_int,
                ]
            )
        )

        # Touchdown commit: gentle lateral + sub-hover thrust so the vehicle
        # settles into contact instead of bouncing.
        if self.guidance.touchdown_ready(state):
            accel[:2] *= 0.5
            accel[2] = min(accel[2], 0.88 * self.env.gravity)

        return self._command_from_thrust_accel(t, state, accel)


class LandingPointMassTrackingFullMPC(LandingTrajectoryTrackingFullMPC):
    """Full tracker over the base point-mass MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_pointmass_factory, **kwargs)


class LandingActuatorTrackingFullMPC(LandingTrajectoryTrackingFullMPC):
    """Full tracker over the Step 1 actuator-aware MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_actuator_factory, **kwargs)


class LandingActuatorMagLagTrackingFullMPC(LandingTrajectoryTrackingFullMPC):
    """Full tracker over the Step 2 actuator-aware + mag-lag MPC."""

    def __init__(self, vehicle: Vehicle, env: Environment, **kwargs):
        super().__init__(vehicle, env, planner=_actuator2_factory, **kwargs)
