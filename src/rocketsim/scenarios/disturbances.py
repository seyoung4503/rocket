"""Disturbances and per-episode randomization — what makes the task *hard*.

Two pieces:
  * ``DisturbanceModel`` produces time-varying wind (mean + gusts) and a small
    random force load, advanced step-by-step during a rollout.
  * ``Randomization`` samples a perturbed Vehicle per episode (unknown mass,
    thrust, and a constant thrust/CG misalignment). This is exactly the
    domain-randomization an RL policy would later train against.

Together they create the regime where a controller must reject disturbances it
cannot measure and cope with a plant it does not know exactly.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from ..vehicle import Vehicle


@dataclass
class DisturbanceModel:
    wind_mean: float = 0.0  # m/s, steady horizontal wind magnitude
    wind_dir: float = 0.0  # rad, horizontal wind direction
    gust_std: float = 0.0  # m/s, std of gust velocity (per horizontal axis)
    gust_tau: float = 1.5  # s, gust correlation time (Ornstein-Uhlenbeck)
    force_std: float = 0.0  # N, std of a direct random force (per horizontal axis)
    force_tau: float = 0.8  # s, force correlation time

    def reset(self, rng: np.random.Generator) -> None:
        self._gust = np.zeros(3)
        self._force = np.zeros(3)
        d = self.wind_dir if self.wind_dir else rng.uniform(0, 2 * np.pi)
        self._mean = np.array([self.wind_mean * np.cos(d), self.wind_mean * np.sin(d), 0.0])

    def _ou(self, prev: np.ndarray, std: float, tau: float, dt: float, rng) -> np.ndarray:
        # Discrete Ornstein-Uhlenbeck toward zero (only horizontal axes excited)
        if std <= 0.0:
            return np.zeros(3)
        a = np.exp(-dt / tau)
        noise = rng.normal(0.0, std * np.sqrt(1 - a * a), size=2)
        out = prev.copy()
        out[:2] = a * prev[:2] + noise
        out[2] = 0.0
        return out

    def step(self, t: float, dt: float, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
        """Advance and return (wind world m/s, external force world N)."""
        self._gust = self._ou(self._gust, self.gust_std, self.gust_tau, dt, rng)
        self._force = self._ou(self._force, self.force_std, self.force_tau, dt, rng)
        return self._mean + self._gust, self._force


@dataclass
class Randomization:
    mass_scale: tuple[float, float] = (1.0, 1.0)
    thrust_scale: tuple[float, float] = (1.0, 1.0)
    misalign_deg: float = 0.0  # max constant thrust/CG misalignment (per axis)
    cg_offset_scale: tuple[float, float] = (1.0, 1.0)  # scales engine_offset
    inertia_scale: tuple[float, float] = (1.0, 1.0)  # scales the inertia tensor
    twr_range: tuple[float, float] | None = None  # if set, draw thrust from TWR
    # (overrides thrust_scale; guarantees a feasible thrust-to-weight while mass
    # varies widely, so episodes stay solvable)

    def sample_vehicle(self, base: Vehicle, rng: np.random.Generator) -> Vehicle:
        v = copy.deepcopy(base)
        v.mass = base.mass * rng.uniform(*self.mass_scale)
        if self.twr_range is not None:
            v.max_thrust = v.mass * 9.80665 * rng.uniform(*self.twr_range)
        else:
            v.max_thrust = base.max_thrust * rng.uniform(*self.thrust_scale)
        v.engine_offset = base.engine_offset * rng.uniform(*self.cg_offset_scale)
        v.inertia = base.inertia * rng.uniform(*self.inertia_scale)
        m = np.deg2rad(self.misalign_deg)
        v.thrust_misalign = rng.uniform(-m, m, size=2) if m > 0 else np.zeros(2)
        v.__post_init__()  # refresh inertia_inv and array casts
        return v


# --- presets ---------------------------------------------------------------

def calm() -> tuple[DisturbanceModel, Randomization]:
    """Nominal, fully-known plant (the original easy regime)."""
    return DisturbanceModel(), Randomization()


def moderate() -> tuple[DisturbanceModel, Randomization]:
    """Realistic disturbances a well-tuned PID should handle convincingly:
    a steady breeze with mild gusts, small plant uncertainty & misalignment."""
    dist = DisturbanceModel(
        wind_mean=2.5,  # ~9 km/h steady breeze
        gust_std=1.0,
        gust_tau=1.5,
        force_std=1.0,  # N, ~4% of weight
        force_tau=0.9,
    )
    rand = Randomization(
        mass_scale=(0.92, 1.08),
        thrust_scale=(0.95, 1.05),
        misalign_deg=1.5,
        cg_offset_scale=(0.92, 1.08),
    )
    return dist, rand


def hard() -> tuple[DisturbanceModel, Randomization]:
    """RL-benchmark-grade difficulty: gusty wind, unknown plant, misalignment."""
    dist = DisturbanceModel(
        wind_mean=4.0,  # ~14 km/h steady wind
        gust_std=2.0,  # strong gusts
        gust_tau=1.2,
        force_std=2.0,  # N, ~8% of weight random load
        force_tau=0.7,
    )
    # Plant uncertainty kept hard-but-feasible: worst-case thrust-to-weight
    # stays ~1.3 (40*0.9 / (2.5*1.15*9.8)), so a soft landing remains possible.
    rand = Randomization(
        mass_scale=(0.85, 1.15),  # +/- ~15% mass unknown
        thrust_scale=(0.9, 1.1),  # weaker / stronger motor
        misalign_deg=2.5,  # constant torque disturbance
        cg_offset_scale=(0.85, 1.15),
    )
    return dist, rand


def model_unknown() -> tuple[DisturbanceModel, Randomization]:
    """The plant changes drastically every episode (mass, thrust, inertia, CG all
    vary 2-3x); wind is mild so the *model uncertainty* is the dominant challenge.

    PID is given a single fixed nominal model and fixed gains, so it cannot be
    right for every episode. RL (trained across this range, with memory) can
    infer the current plant from the state history and adapt — its home turf.
    """
    dist = DisturbanceModel(
        wind_mean=1.0,  # mild — isolate the model-uncertainty effect
        gust_std=0.5,
        gust_tau=1.5,
        force_std=0.5,
        force_tau=0.9,
    )
    rand = Randomization(
        mass_scale=(0.6, 1.8),  # 3x mass spread -> wrong hover-throttle feedforward
        twr_range=(1.4, 2.6),  # feasible but varied -> hover throttle unknown
        misalign_deg=3.0,
        cg_offset_scale=(0.6, 1.5),
        inertia_scale=(0.5, 2.0),  # 4x spread -> fixed attitude gains mismatched
    )
    return dist, rand
