"""Shared landing guidance for PID, MPC and later RL-residual controllers.

The guidance layer decides the target position the tracking controller should
follow. It does not generate throttle or gimbal commands directly.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .. import dynamics as dyn
from .. import quaternion as quat


@dataclass
class LandingGuidanceConfig:
    v_max: float = 3.5
    v_min: float = 0.25
    flare_gain: float = 0.45
    gate_offset: float = 0.7
    gate_speed: float = 0.7
    gate_tilt_deg: float = 8.0
    gate_alt: float = 2.5
    tight_frac: float = 0.5
    creep: float = 0.35
    settle_readiness: float = 0.45
    commit_alt: float = 0.35
    commit_offset: float = 0.45
    commit_hspeed: float = 0.40
    commit_vspeed: float = 0.60
    commit_tilt_deg: float = 7.0


@dataclass
class LandingGuidanceOutput:
    target: np.ndarray
    readiness: float
    gate: float
    mode: str
    z_setpoint: float
    descent_rate: float


class LandingGuidance:
    """Stateful landing target generator.

    This is the landing gate that used to live inside ``LandingPID``. Keeping it
    here lets PID and MPC variants share the same descent timing and touchdown
    commit logic.
    """

    def __init__(self, config: LandingGuidanceConfig | None = None):
        self.config = config or LandingGuidanceConfig()
        self._zset: float | None = None
        self._last_t = 0.0

    def reset(self, state: np.ndarray, t: float = 0.0) -> None:
        self._zset = float(max(state[dyn.POS][2], 0.0))
        self._last_t = float(t)

    @property
    def z_setpoint(self) -> float | None:
        return self._zset

    def readiness(self, state: np.ndarray, alt: float | None = None) -> float:
        cfg = self.config
        alt = max(float(state[dyn.POS][2] if alt is None else alt), 0.0)
        scale = cfg.tight_frac + (1.0 - cfg.tight_frac) * min(alt / cfg.gate_alt, 1.0)
        horiz = np.linalg.norm(state[dyn.POS][:2])
        lat_speed = np.linalg.norm(state[dyn.VEL][:2])
        tilt = quat.tilt_angle(state[dyn.QUAT])
        gate_tilt = np.deg2rad(cfg.gate_tilt_deg)
        return float(
            np.exp(-((horiz / (cfg.gate_offset * scale)) ** 2))
            * np.exp(-((lat_speed / (cfg.gate_speed * scale)) ** 2))
            * np.exp(-((tilt / (gate_tilt * scale)) ** 2))
        )

    def touchdown_ready(self, state: np.ndarray) -> bool:
        cfg = self.config
        pos = state[dyn.POS]
        vel = state[dyn.VEL]
        return bool(
            pos[2] < cfg.commit_alt
            and np.linalg.norm(pos[:2]) < cfg.commit_offset
            and np.linalg.norm(vel[:2]) < cfg.commit_hspeed
            and vel[2] > -cfg.commit_vspeed
            and np.rad2deg(quat.tilt_angle(state[dyn.QUAT])) < cfg.commit_tilt_deg
        )

    def mode(self, state: np.ndarray, readiness: float) -> str:
        z = float(state[dyn.POS][2])
        if self.touchdown_ready(state):
            return "commit"
        if z < self.config.gate_alt:
            return "flare" if readiness >= self.config.settle_readiness else "settle"
        return "approach"

    def update(self, t: float, state: np.ndarray, xy_target: np.ndarray | None = None) -> LandingGuidanceOutput:
        cfg = self.config
        if self._zset is None:
            self.reset(state, t)

        dt = max(float(t) - self._last_t, 0.0)
        self._last_t = float(t)

        assert self._zset is not None
        ready = self.readiness(state, self._zset)
        v_desc = float(np.clip(cfg.flare_gain * self._zset, cfg.v_min, cfg.v_max))
        gate = cfg.creep + (1.0 - cfg.creep) * ready
        mode = self.mode(state, ready)

        self._zset = max(0.0, self._zset - v_desc * gate * dt)

        xy = np.zeros(2) if xy_target is None else np.asarray(xy_target, dtype=float)
        target = np.array([xy[0], xy[1], self._zset], dtype=float)
        return LandingGuidanceOutput(
            target=target,
            readiness=ready,
            gate=gate,
            mode=mode,
            z_setpoint=self._zset,
            descent_rate=v_desc * gate,
        )
