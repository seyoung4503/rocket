from .landing import LandingPID
from .mpc import (
    CvxpyActuatorAwareMagLagMPC,
    CvxpyActuatorAwareMPC,
    CvxpyPointMassMPC,
    LandingActuatorAwareMagLagWaypointPID,
    LandingActuatorAwareWaypointPID,
    LandingCvxpyGuidancePID,
    LandingCvxpyMPC,
    LandingCvxpyWaypointPID,
    LandingFeasibleWaypointMPC,
    LandingFullDynamicsMPC,
    LandingVerticalMPC,
    SampledVerticalMPC,
)
from .pid import HoverPID
from .trajectory_tracker import (
    LandingActuatorMagLagTrackingFullMPC,
    LandingActuatorMagLagTrackingMPC,
    LandingActuatorTrackingFullMPC,
    LandingActuatorTrackingMPC,
    LandingPointMassTrackingFullMPC,
    LandingPointMassTrackingMPC,
    LandingTrajectoryTrackingFullMPC,
    LandingTrajectoryTrackingMPC,
)

__all__ = [
    "HoverPID",
    "LandingPID",
    "LandingVerticalMPC",
    "SampledVerticalMPC",
    "LandingCvxpyMPC",
    "LandingCvxpyGuidancePID",
    "LandingCvxpyWaypointPID",
    "LandingFeasibleWaypointMPC",
    "LandingFullDynamicsMPC",
    "CvxpyPointMassMPC",
    # Step 1 of actuator-aware MPC (see docs/2026-05-29_0213_v1_*).
    "CvxpyActuatorAwareMPC",
    "LandingActuatorAwareWaypointPID",
    # Step 2: Step 1 + thrust-magnitude 1st-order lag.
    "CvxpyActuatorAwareMagLagMPC",
    "LandingActuatorAwareMagLagWaypointPID",
    # SpaceX-style time-indexed trajectory tracking (no `lookahead` knob);
    # see docs/2026-05-29_1653_v1_lookahead_vs_spacex_design.md.
    "LandingTrajectoryTrackingMPC",
    "LandingPointMassTrackingMPC",
    "LandingActuatorTrackingMPC",
    "LandingActuatorMagLagTrackingMPC",
    # Full SpaceX-style: time-indexed + position integrator + landing gate
    # + touchdown commit. Closer to the proper GNC stack.
    "LandingTrajectoryTrackingFullMPC",
    "LandingPointMassTrackingFullMPC",
    "LandingActuatorTrackingFullMPC",
    "LandingActuatorMagLagTrackingFullMPC",
]
