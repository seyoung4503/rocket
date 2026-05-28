"""PID landing controller using the shared landing guidance layer."""

from __future__ import annotations

from ..dynamics import Command
from ..guidance import LandingGuidance, LandingGuidanceConfig
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
        self.guidance = LandingGuidance(
            LandingGuidanceConfig(
                v_max=v_max,
                v_min=v_min,
                flare_gain=flare_gain,
                gate_offset=gate_offset,
                gate_speed=gate_speed,
                gate_tilt_deg=gate_tilt_deg,
                gate_alt=gate_alt,
                tight_frac=tight_frac,
                creep=creep,
            )
        )

    def __call__(self, t: float, state: np.ndarray) -> Command:
        guidance = self.guidance.update(t, state)
        self.pid.target = guidance.target
        return self.pid(t, state)
