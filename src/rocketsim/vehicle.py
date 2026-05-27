"""Vehicle and environment configuration.

Body frame: z = thrust axis pointing toward the nose ("up" when upright),
x = forward, y = left. The engine (gimbaled EDF) sits at the tail, a distance
``engine_offset`` below the center of gravity along -z.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Vehicle:
    # Mass properties
    mass: float = 2.5  # kg  (small EDF testbed)
    inertia: np.ndarray = field(
        # Slender body: large about x/y (pitch/yaw), small about z (roll).
        default_factory=lambda: np.diag([0.12, 0.12, 0.004])
    )

    # Propulsion (EDF)
    max_thrust: float = 40.0  # N  (~4 kgf, thrust-to-weight ~1.6)
    thrust_time_constant: float = 0.08  # s, first-order spool-up lag
    engine_offset: float = 0.45  # m, distance from CG to gimbal pivot along -z
    gimbal_limit: float = np.deg2rad(12.0)  # rad, max deflection per axis
    gimbal_rate_limit: float = np.deg2rad(200.0)  # rad/s, actuator slew rate

    # Aerodynamics (simple quadratic drag opposing airspeed)
    drag_coeff: float = 0.5
    ref_area: float = 0.0079  # m^2, ~10 cm diameter tube (axial)
    side_area: float = 0.045  # m^2, ~10 cm dia x 45 cm body (catches crosswind)

    # Constant thrust/CG misalignment, modeled as a fixed gimbal bias (rad).
    # The controller does NOT know about this -> a steady torque disturbance.
    thrust_misalign: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def __post_init__(self) -> None:
        self.thrust_misalign = np.asarray(self.thrust_misalign, dtype=float)
        self.inertia = np.asarray(self.inertia, dtype=float)
        self.inertia_inv = np.linalg.inv(self.inertia)


@dataclass
class Environment:
    gravity: float = 9.80665  # m/s^2
    air_density: float = 1.225  # kg/m^3 (sea level)


# Convenience factory for the EDF hover testbed used in the early phase.
def edf_testbed() -> Vehicle:
    return Vehicle()
