"""Small state estimators for controller-facing GNC experiments."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat


def _alpha(dt: float, tau: float) -> float:
    if tau <= 0.0:
        return 1.0
    return float(1.0 - np.exp(-max(dt, 0.0) / tau))


def _nlerp_quat(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = quat.quat_normalize(np.asarray(q0, dtype=float))
    q1 = quat.quat_normalize(np.asarray(q1, dtype=float))
    if float(np.dot(q0, q1)) < 0.0:
        q1 = -q1
    return quat.quat_normalize((1.0 - alpha) * q0 + alpha * q1)


@dataclass
class LowPassEstimatorConfig:
    # Keep attitude/rate almost raw; over-filtering them removes attitude
    # damping and causes tip-over. The main noise problem for LandingPID is
    # translational velocity, so filter position/velocity lightly.
    #
    # These tau values are tuned for the 2x sensor-noise "noisy" regime. They
    # are scaled by ``noise_scale`` at update time so clean inputs are passed
    # nearly raw (scale -> 0 means no filtering, which avoids the lag penalty
    # that hurt PID on the hard regime where obs_noise=0).
    pos_tau: float = 0.04
    vel_tau: float = 0.12
    quat_tau: float = 0.001
    omega_tau: float = 0.003
    derived_vel_mix: float = 0.0
    noise_scale: float = 1.0  # multiplies all taus; 0 = passthrough, 1 = full

    @classmethod
    def for_obs_noise(cls, obs_noise: float, reference: float = 2.0) -> "LowPassEstimatorConfig":
        """Build a config whose filter strength scales with the input noise.

        Defaults are tuned for obs_noise=2 (the ``noisy`` regime), so
        ``noise_scale = obs_noise / reference``: obs_noise=0 -> no filtering,
        obs_noise=2 -> the full tuned filter.
        """
        scale = float(max(obs_noise, 0.0) / max(reference, 1e-6))
        return cls(noise_scale=scale)


class LowPassStateEstimator:
    """Low-pass/complementary estimator over the existing full-state measurement.

    It is deliberately simpler than an EKF. The immediate goal is to test the
    GNC split: controllers consume estimated state, while success metrics still
    use true state.
    """

    def __init__(self, config: LowPassEstimatorConfig | None = None):
        self.config = config or LowPassEstimatorConfig()
        self.state: np.ndarray | None = None
        self._prev_pos: np.ndarray | None = None

    def reset(self, measurement: np.ndarray) -> np.ndarray:
        self.state = np.asarray(measurement, dtype=float).copy()
        self.state[dyn.QUAT] = quat.quat_normalize(self.state[dyn.QUAT])
        self._prev_pos = self.state[dyn.POS].copy()
        return self.state.copy()

    def update(self, measurement: np.ndarray, dt: float) -> np.ndarray:
        measurement = np.asarray(measurement, dtype=float)
        if self.state is None:
            return self.reset(measurement)

        cfg = self.config
        out = self.state.copy()

        # Scale all filter time constants by noise_scale: clean inputs pass
        # through (scale -> 0 collapses tau -> 0 -> alpha -> 1.0).
        s = cfg.noise_scale
        a_pos = _alpha(dt, cfg.pos_tau * s)
        a_vel = _alpha(dt, cfg.vel_tau * s)
        a_quat = _alpha(dt, cfg.quat_tau * s)
        a_omega = _alpha(dt, cfg.omega_tau * s)

        prev_pos = out[dyn.POS].copy()
        out[dyn.POS] = (1.0 - a_pos) * out[dyn.POS] + a_pos * measurement[dyn.POS]

        derived_vel = (out[dyn.POS] - prev_pos) / max(dt, 1e-6)
        measured_vel = measurement[dyn.VEL]
        vel_input = (1.0 - cfg.derived_vel_mix) * measured_vel + cfg.derived_vel_mix * derived_vel
        out[dyn.VEL] = (1.0 - a_vel) * out[dyn.VEL] + a_vel * vel_input

        out[dyn.QUAT] = _nlerp_quat(out[dyn.QUAT], measurement[dyn.QUAT], a_quat)
        out[dyn.OMEGA] = (1.0 - a_omega) * out[dyn.OMEGA] + a_omega * measurement[dyn.OMEGA]
        out[dyn.THRUST] = measurement[dyn.THRUST]

        self.state = out
        self._prev_pos = out[dyn.POS].copy()
        return out.copy()
