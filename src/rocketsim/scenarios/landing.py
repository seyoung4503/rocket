"""Retro-thrust landing scenario — the shared task definition for PID and RL.

This module defines the *task*, not the controller:
  - initial condition sampling (start high, descending, slightly off-axis),
  - termination (touchdown / crash / out-of-bounds / timeout),
  - success criteria (soft, upright, on-target touchdown),
  - a reward function (used later by the RL environment),
  - trajectory evaluation metrics (used to compare PID vs RL fairly).

Ground is the plane z = 0; the pad is the origin. The vehicle must descend and
touch down softly and upright.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat


@dataclass
class TouchdownResult:
    landed: bool
    crashed: bool
    timed_out: bool
    t: float
    horizontal_offset: float  # m from pad
    vertical_speed: float  # m/s at touchdown (positive = downward)
    horizontal_speed: float  # m/s at touchdown
    tilt_deg: float
    success: bool
    energy: float  # integral of throttle (proxy for energy/fuel use)


@dataclass
class LandingScenario:
    # --- start conditions (sampled) ---
    start_alt: tuple[float, float] = (8.0, 12.0)  # m
    start_descent: tuple[float, float] = (-1.0, -3.0)  # m/s (negative = down)
    start_offset: float = 1.5  # m, max horizontal offset
    start_tilt_deg: float = 8.0  # deg, max initial tilt
    start_lateral_vel: float = 0.5  # m/s, max horizontal velocity

    # --- success thresholds at touchdown ---
    max_touchdown_vspeed: float = 1.0  # m/s
    max_touchdown_hspeed: float = 0.5  # m/s
    max_touchdown_offset: float = 0.5  # m
    max_touchdown_tilt_deg: float = 8.0  # deg

    # --- termination ---
    crash_tilt_deg: float = 45.0  # tip-over -> crash
    bounds_radius: float = 15.0  # m, horizontal fly-away -> abort
    max_altitude: float = 20.0  # m, shot up -> abort
    timeout: float = 20.0  # s

    pad_alt: float = 0.0  # m

    @classmethod
    def hard(cls) -> "LandingScenario":
        """Aggressive start: higher, faster descent, big offset/tilt/lateral vel.
        Success thresholds are unchanged (a soft landing is a soft landing)."""
        return cls(
            start_alt=(10.0, 14.0),
            start_descent=(-2.0, -5.0),
            start_offset=3.0,
            start_tilt_deg=20.0,
            start_lateral_vel=2.0,
            timeout=35.0,  # starts higher + may hover to re-center before commit
        )

    def sample_initial_state(self, rng: np.random.Generator | None = None) -> np.ndarray:
        rng = rng or np.random.default_rng()
        alt = rng.uniform(*self.start_alt)
        vz = rng.uniform(min(self.start_descent), max(self.start_descent))
        ang = rng.uniform(0, 2 * np.pi)
        r = rng.uniform(0, self.start_offset)
        px, py = r * np.cos(ang), r * np.sin(ang)
        vxy = rng.uniform(-self.start_lateral_vel, self.start_lateral_vel, size=2)

        # small random tilt
        tilt = np.deg2rad(rng.uniform(0, self.start_tilt_deg))
        axis_ang = rng.uniform(0, 2 * np.pi)
        half = tilt / 2
        q = np.array(
            [np.cos(half), np.sin(half) * np.cos(axis_ang), np.sin(half) * np.sin(axis_ang), 0.0]
        )
        return dyn.initial_state(
            position=(px, py, alt),
            velocity=(vxy[0], vxy[1], vz),
            quaternion=q,
        )

    # --- termination & success ---------------------------------------------

    def check_done(self, state: np.ndarray, t: float) -> tuple[bool, str]:
        z = state[dyn.POS][2]
        tilt = np.rad2deg(quat.tilt_angle(state[dyn.QUAT]))
        horiz = np.linalg.norm(state[dyn.POS][:2])
        if z <= self.pad_alt:
            return True, "touchdown"
        if tilt > self.crash_tilt_deg:
            return True, "crash_tilt"
        if horiz > self.bounds_radius:
            return True, "out_of_bounds"
        if z > self.max_altitude:
            return True, "too_high"
        if t >= self.timeout:
            return True, "timeout"
        return False, ""

    def is_soft_landing(self, state: np.ndarray) -> bool:
        vel = state[dyn.VEL]
        return (
            -vel[2] <= self.max_touchdown_vspeed
            and np.linalg.norm(vel[:2]) <= self.max_touchdown_hspeed
            and np.linalg.norm(state[dyn.POS][:2]) <= self.max_touchdown_offset
            and np.rad2deg(quat.tilt_angle(state[dyn.QUAT])) <= self.max_touchdown_tilt_deg
        )

    # --- RL reward (shaped) --------------------------------------------------

    def reward(self, state: np.ndarray, cmd: np.ndarray, reason: str) -> float:
        """Dense shaping + sparse terminal bonus. Used by the RL env later."""
        pos, vel = state[dyn.POS], state[dyn.VEL]
        tilt = quat.tilt_angle(state[dyn.QUAT])
        horiz = np.linalg.norm(pos[:2])

        # per-step shaping: stay near the pad axis, upright, descend gently
        r = 0.0
        r -= 0.30 * horiz  # off-axis penalty
        r -= 0.50 * tilt  # tilt penalty
        r -= 0.05 * np.linalg.norm(vel[:2])  # lateral velocity
        r -= 0.02 * cmd[0]  # mild energy penalty
        r -= 0.02  # time penalty (encourage finishing)

        if reason == "touchdown":
            if self.is_soft_landing(state):
                r += 100.0
                r -= 10.0 * max(0.0, -vel[2])  # softer is better
            else:
                r -= 30.0  # hard landing
        elif reason in ("crash_tilt", "out_of_bounds", "too_high"):
            r -= 100.0
        return float(r)

    # --- evaluation ----------------------------------------------------------

    def evaluate(self, traj) -> TouchdownResult:
        t, states, cmds = traj.as_arrays()
        energy = float(np.trapezoid(cmds[:, 0], t)) if len(t) > 1 else 0.0
        final, reason = self._final_index(states, t)
        s = states[final]
        vel = s[dyn.VEL]
        landed = reason == "touchdown"
        return TouchdownResult(
            landed=landed,
            crashed=reason in ("crash_tilt", "out_of_bounds", "too_high"),
            timed_out=reason == "timeout",
            t=float(t[final]),
            horizontal_offset=float(np.linalg.norm(s[dyn.POS][:2])),
            vertical_speed=float(-vel[2]),
            horizontal_speed=float(np.linalg.norm(vel[:2])),
            tilt_deg=float(np.rad2deg(quat.tilt_angle(s[dyn.QUAT]))),
            success=landed and self.is_soft_landing(s),
            energy=energy,
        )

    def _final_index(self, states: np.ndarray, t: np.ndarray) -> tuple[int, str]:
        for i in range(len(states)):
            done, reason = self.check_done(states[i], t[i])
            if done:
                return i, reason
        return len(states) - 1, "timeout"
