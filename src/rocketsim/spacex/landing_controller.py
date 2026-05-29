"""Top-level SpaceX-style landing controller.

Composes the three independent layers:

    ┌─────────────────────────────────────────────────────────────────┐
    │  ConvexLandingMPC     (planner — min-fuel, glideslope, terminal)│
    │            │                                                    │
    │            ▼ MpcPlan(p, v, u, start_t)                          │
    │  TrajectoryTracker    (closed-loop — PD+I on time-indexed ref)  │
    │            │                                                    │
    │            ▼ a_des (m/s^2, world frame)                         │
    │  AttitudeController   (thrust vector → throttle + gimbal)       │
    │            │                                                    │
    │            ▼ Command(throttle, gimbal_x, gimbal_y)              │
    └─────────────────────────────────────────────────────────────────┘

This is the *only* class evaluation scripts need to instantiate; the
internals stay clean and independent so each layer can be unit-tested
or swapped out (e.g., Step 3 SCP 6-DOF MPC could replace the planner
without touching the tracker or attitude controller).
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from ..dynamics import Command
from ..vehicle import Environment, Vehicle
from .attitude_controller import AttitudeController
from .convex_landing_mpc import ConvexLandingMPC, MpcPlan
from .trajectory_tracker import TrajectoryTracker


class LandingControllerSpaceX:
    """SpaceX-style landing controller (planner + tracker + attitude).

    Public API mirrors the existing landing controllers — instances are
    callable ``ctrl(t, state) -> Command`` — so this drops into the
    existing ``evaluate_navigation.py`` infrastructure.
    """

    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        # Mission duration upper bound. Plan shrinks toward 0 as sim_t
        # approaches T_final, forcing commit. Tuned to be a bit longer
        # than the simulator's `timeout` so plans don't run out before
        # the env decides the episode is over.
        T_final: float = 12.0,
        replan_dt: float = 0.2,
        mpc_dt: float = 0.2,
        mpc_max_tilt: float = np.deg2rad(12.0),
        att_max_tilt: float = np.deg2rad(20.0),
        glideslope_deg: float = 30.0,
        # 2026-05-29 lowered 4.0 -> 2.0: our EDF has noticeable actuator
        # lag (thrust τ=0.08s + gimbal rate limit) so the plan's
        # hoverslam-style "coast at -4 m/s, kill at the end" cannot be
        # tracked precisely — vehicle slams the ground at vspeed > 3 m/s.
        # 2.0 m/s leaves the tracker enough time-margin to decelerate.
        v_max_desc: float = 2.0,
        # Tracker gains — HoverPID-tuned defaults, with extra vz damping.
        kp_pos: tuple[float, float, float] = (1.6, 1.6, 6.0),
        kd_pos: tuple[float, float, float] = (2.4, 2.4, 6.0),
        ki_pos: tuple[float, float, float] = (0.6, 0.6, 1.5),
        # ★ 2026-05-29 — swap in actuator-aware planners.
        # planner_cls is called with the same kwargs that get forwarded
        # to ConvexLandingMPC (T_final, dt, max_tilt, glideslope_deg,
        # v_max_desc).  Defaults to the base ConvexLandingMPC; use
        # ActuatorAwareLandingMPC for Step 1 (slew) or
        # ActuatorMagLagLandingMPC for Step 1+2 (slew + thrust-magnitude lag).
        planner_cls=ConvexLandingMPC,
        planner_kwargs: dict | None = None,
    ):
        self.vehicle = vehicle
        self.env = env
        self.replan_dt = float(replan_dt)
        self.mpc = planner_cls(
            vehicle,
            env,
            T_final=T_final,
            dt=mpc_dt,
            max_tilt=mpc_max_tilt,
            glideslope_deg=glideslope_deg,
            v_max_desc=v_max_desc,
            **(planner_kwargs or {}),
        )
        self.tracker = TrajectoryTracker(
            kp_pos=kp_pos, kd_pos=kd_pos, ki_pos=ki_pos
        )
        self.attitude = AttitudeController(
            vehicle, env, max_tilt=att_max_tilt
        )
        self._plan: MpcPlan | None = None
        self._last_plan_t = -np.inf

    def __call__(self, t: float, state: np.ndarray) -> Command:
        pos = np.asarray(state[dyn.POS], dtype=float)
        vel = np.asarray(state[dyn.VEL], dtype=float)

        # Periodic replan. If solver fails, keep the most recent plan.
        if (
            t - self._last_plan_t >= self.replan_dt
            or self._plan is None
        ):
            plan = self.mpc.solve(t, state)
            if plan is not None:
                self._plan = plan
                self._last_plan_t = t

        # Fallback if we don't yet have a plan — hover.
        if self._plan is None:
            hover_a = np.array([0.0, 0.0, self.env.gravity])
            return self.attitude.command(t, state, hover_a)

        a_des = self.tracker.step(t, pos, vel, self._plan)
        return self.attitude.command(t, state, a_des)
