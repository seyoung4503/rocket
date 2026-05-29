"""SpaceX-style landing GNC stack.

Built from scratch (independent of the existing PID/MPC wrappers) to
faithfully realize the literature pattern:

  * Convex min-fuel MPC with glideslope + thrust-cone constraints,
    SHRINKING horizon (T_final fixed, not receding).
  * Time-indexed trajectory tracker with PD + position integrator. No
    setpoint hack; consumes the full (p, v, u) plan.
  * Quaternion-based attitude controller that maps a desired
    thrust-acceleration vector into (throttle, gimbal_x, gimbal_y).

See ``docs/2026-05-29_1753_v1_spacex_style_design.md`` for the algorithm-
level differences vs the lookahead/wrapper approach we used previously.
"""

from .attitude_controller import AttitudeController
from .convex_landing_mpc import (
    ActuatorAwareLandingMPC,
    ActuatorMagLagLandingMPC,
    ConvexLandingMPC,
    MpcPlan,
)
from .landing_controller import LandingControllerSpaceX
from .trajectory_tracker import TrajectoryTracker

__all__ = [
    "AttitudeController",
    "ConvexLandingMPC",
    "ActuatorAwareLandingMPC",
    "ActuatorMagLagLandingMPC",
    "MpcPlan",
    "TrajectoryTracker",
    "LandingControllerSpaceX",
]
