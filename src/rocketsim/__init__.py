"""rocketsim: a 6-DOF gimbaled-thrust rocket simulator.

Phase 1 target: an EDF (electric ducted fan) thrust-vectored hover testbed,
used as a safe, low-speed stage to develop and compare PID and RL controllers
for attitude stabilization and propulsive (retro-thrust) landing.
"""

from . import dynamics, quaternion, simulator, vehicle
from .dynamics import Command, initial_state
from .simulator import Simulator, Trajectory
from .vehicle import Environment, Vehicle, edf_testbed

__all__ = [
    "dynamics",
    "quaternion",
    "simulator",
    "vehicle",
    "Command",
    "initial_state",
    "Simulator",
    "Trajectory",
    "Environment",
    "Vehicle",
    "edf_testbed",
]
