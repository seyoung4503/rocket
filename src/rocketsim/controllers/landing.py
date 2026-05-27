"""PID landing (retro-thrust) controller — the baseline to beat with RL.

Reuses the cascaded HoverPID, but instead of a fixed setpoint it runs a simple
descent *guidance*: lower the altitude setpoint over time with a descent rate
that shrinks near the ground (a "flare"), so the vehicle decelerates into a soft
touchdown over the pad. Horizontal target stays at the origin.
"""

from __future__ import annotations

import numpy as np

from .. import dynamics as dyn
from ..dynamics import Command
from ..vehicle import Environment, Vehicle
from .pid import HoverPID


class LandingPID:
    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment,
        v_max: float = 2.5,  # m/s, max descent rate (high altitude)
        v_min: float = 0.3,  # m/s, descent rate near the ground (flare)
        flare_gain: float = 0.4,  # 1/s, descent rate per metre of altitude
    ):
        self.pid = HoverPID(vehicle, env, target=(0.0, 0.0, 0.0))
        self.v_max, self.v_min, self.flare_gain = v_max, v_min, flare_gain
        self._zset: float | None = None
        self._last_t = 0.0

    def __call__(self, t: float, state: np.ndarray) -> Command:
        z = state[dyn.POS][2]
        if self._zset is None:
            self._zset = float(z)
        dt = max(t - self._last_t, 0.0)
        self._last_t = t

        # descent rate shrinks as the setpoint approaches the pad
        v_desc = float(np.clip(self.flare_gain * self._zset, self.v_min, self.v_max))
        self._zset = max(0.0, self._zset - v_desc * dt)

        self.pid.target = np.array([0.0, 0.0, self._zset])
        return self.pid(t, state)
