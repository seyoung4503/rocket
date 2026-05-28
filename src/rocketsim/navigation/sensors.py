"""Simple sensor models for GNC experiments.

The landing env already has a noisy measurement path. This module makes that
idea reusable outside the env, so controller/evaluator code can distinguish
true state, raw measurement and estimated state explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat


@dataclass
class SensorNoise:
    """Measurement standard deviations used by the simple sensor model."""

    pos: float = 0.05
    vel: float = 0.20
    omega: float = 0.08
    attitude_deg: float = 1.5
    scale: float = 1.0


class SensorModel:
    """Noisy full-state measurement model.

    This intentionally mirrors ``LandingEnv._measure``. It is not a real IMU
    mechanization yet; it is a small transition step that lets us test whether
    a controller improves when it consumes an estimated state instead of raw
    noisy measurements.
    """

    def __init__(self, noise: SensorNoise | None = None):
        self.noise = noise or SensorNoise()

    def measure(self, true_state: np.ndarray, rng: np.random.Generator) -> np.ndarray:
        s = np.asarray(true_state, dtype=float).copy()
        k = self.noise.scale
        if k <= 0.0:
            return s

        s[dyn.POS] += rng.normal(0.0, self.noise.pos * k, 3)
        s[dyn.VEL] += rng.normal(0.0, self.noise.vel * k, 3)
        s[dyn.OMEGA] += rng.normal(0.0, self.noise.omega * k, 3)

        a = rng.normal(0.0, np.deg2rad(self.noise.attitude_deg) * k, 3)
        dq = quat.quat_normalize(np.array([1.0, a[0] / 2, a[1] / 2, a[2] / 2]))
        s[dyn.QUAT] = quat.quat_normalize(quat.quat_mult(s[dyn.QUAT], dq))
        return s
