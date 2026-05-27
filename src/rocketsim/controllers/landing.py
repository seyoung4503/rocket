"""PID landing (retro-thrust) controller — the baseline to beat with RL.

Reuses the cascaded HoverPID, but instead of a fixed setpoint it runs a simple
descent *guidance*: lower the altitude setpoint over time with a descent rate
that shrinks near the ground (a "flare"), so the vehicle decelerates into a soft
touchdown over the pad. Horizontal target stays at the origin.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat
from ..dynamics import Command
from ..vehicle import Environment, Vehicle
from .pid import HoverPID


class LandingPID:
    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        v_max: float = 3.5,  # m/s, max descent rate (high altitude)
        v_min: float = 0.25,  # m/s, descent rate near the ground (flare)
        flare_gain: float = 0.45,  # 1/s, descent rate per metre of altitude
        # landing gate (at altitude): error scales that shrink the descent rate
        gate_offset: float = 0.7,  # m, horizontal-error scale
        gate_speed: float = 0.7,  # m/s, lateral-speed scale
        gate_tilt_deg: float = 8.0,  # deg, tilt scale
        # the gate tightens toward the ground so touchdown happens only when the
        # vehicle is well centered, level and slow (avoids gust-pushed contact).
        gate_alt: float = 2.5,  # m, above this the gate is at full (loose) width
        tight_frac: float = 0.5,  # gate width multiplier at the pad
        creep: float = 0.35,  # min fraction of descent kept when off-nominal
    ):
        self.pid = HoverPID(vehicle, env, target=(0.0, 0.0, 0.0))
        self.v_max, self.v_min, self.flare_gain = v_max, v_min, flare_gain
        self.gate_offset = gate_offset
        self.gate_speed = gate_speed
        self.gate_tilt = np.deg2rad(gate_tilt_deg)
        self.gate_alt, self.tight_frac = gate_alt, tight_frac
        self.creep = creep
        self._zset: float | None = None
        self._last_t = 0.0

    def _readiness(self, state: np.ndarray, alt: float) -> float:
        """0..1: how ready the vehicle is to descend (centered, level, slow).
        Tolerances shrink with altitude so the final commit demands precision."""
        scale = self.tight_frac + (1.0 - self.tight_frac) * min(alt / self.gate_alt, 1.0)
        horiz = np.linalg.norm(state[dyn.POS][:2])
        lat_speed = np.linalg.norm(state[dyn.VEL][:2])
        tilt = quat.tilt_angle(state[dyn.QUAT])
        g = (
            np.exp(-((horiz / (self.gate_offset * scale)) ** 2))
            * np.exp(-((lat_speed / (self.gate_speed * scale)) ** 2))
            * np.exp(-((tilt / (self.gate_tilt * scale)) ** 2))
        )
        return float(g)

    def __call__(self, t: float, state: np.ndarray) -> Command:
        z = state[dyn.POS][2]
        if self._zset is None:
            self._zset = float(z)
        dt = max(t - self._last_t, 0.0)
        self._last_t = t

        # descent rate shrinks as the setpoint approaches the pad ...
        v_desc = float(np.clip(self.flare_gain * self._zset, self.v_min, self.v_max))
        # ... and is gated by readiness: hover to re-center/level when off-nominal,
        # keeping only a slow creep so progress never fully stalls.
        gate = self.creep + (1.0 - self.creep) * self._readiness(state, self._zset)
        self._zset = max(0.0, self._zset - v_desc * gate * dt)

        self.pid.target = np.array([0.0, 0.0, self._zset])
        return self.pid(t, state)
