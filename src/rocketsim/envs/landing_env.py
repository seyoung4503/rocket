"""Gymnasium environment for retro-thrust landing.

Wraps the shared landing scenario, disturbances and 6-DOF dynamics so an RL
agent can be trained and compared against the PID baseline *in the same world*.

  observation (13): [ pos/10 (3), vel/5 (3), body_z_world (3), omega/5 (3),
                      thrust/max_thrust (1) ]
      body_z_world is the body up-axis in world coords (upright -> [0,0,1]);
      used instead of a quaternion to avoid the double-cover for the network.
  action (3): [ throttle_cmd, gimbal_x_cmd, gimbal_y_cmd ] in [-1, 1]
      throttle = (a0+1)/2 -> [0,1];  gimbal = a*gimbal_limit

Control runs at ``control_hz``; physics is integrated with ``sim_dt`` substeps,
applying gimbal rate limits and disturbances per substep (matching the PID sim).
"""

from __future__ import annotations

from collections import deque

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .. import dynamics as dyn
from ..dynamics import Command
from ..vehicle import Environment, edf_testbed
from ..scenarios import LandingScenario, calm
from ..scenarios.disturbances import DisturbanceModel, Randomization


class LandingEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        scenario: LandingScenario | None = None,
        disturbance: DisturbanceModel | None = None,
        randomization: Randomization | None = None,
        control_hz: float = 50.0,
        sim_dt: float = 0.002,
        step_penalty: float = 0.05,
        init_scale: float = 1.0,
        residual: bool = False,
        residual_scale: float = 0.4,
        n_stack: int = 1,
        seed: int | None = None,
    ):
        super().__init__()
        self.step_penalty = step_penalty
        self.init_scale = init_scale
        # Residual RL: the PID baseline produces the action; the policy only adds
        # a bounded correction (residual_scale * action). The policy thus starts
        # from PID's behavior and learns just the gust-precision it misses.
        self.residual = residual
        self.residual_scale = residual_scale
        self._base_ctrl = None
        self.scenario = scenario or LandingScenario()
        if disturbance is None or randomization is None:
            d, r = calm()
            disturbance = disturbance or d
            randomization = randomization or r
        self.disturbance = disturbance
        self.randomization = randomization

        self.base = edf_testbed()  # nominal model (defines action scaling)
        self.world = Environment()
        self.control_dt = 1.0 / control_hz
        self.sim_dt = sim_dt
        self.n_sub = max(1, int(round(self.control_dt / self.sim_dt)))

        # Frame stacking ("memory"): the policy sees the last n_stack raw obs,
        # so it can infer the unmeasured wind/gust from how the state evolves
        # (acceleration vs commanded thrust) and pre-compensate — something a
        # memoryless policy (and PID's slow integral) can only do weakly.
        self.n_stack = max(1, int(n_stack))
        self._frames: deque | None = None

        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        high = np.full(13 * self.n_stack, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self._rng = np.random.default_rng(seed)
        self.vehicle = self.base
        self.state = dyn.initial_state()
        self.t = 0.0
        self._prev_gimbal = np.zeros(2)

    # --- helpers ------------------------------------------------------------

    def _raw_obs(self) -> np.ndarray:
        s = self.state
        from .. import quaternion as quat

        body_z = quat.quat_rotate(s[dyn.QUAT], np.array([0.0, 0.0, 1.0]))
        return np.concatenate(
            [
                s[dyn.POS] / 10.0,
                s[dyn.VEL] / 5.0,
                body_z,
                s[dyn.OMEGA] / 5.0,
                [s[dyn.THRUST] / self.base.max_thrust],
            ]
        ).astype(np.float32)

    def _obs(self) -> np.ndarray:
        """Push the current raw obs and return the stacked (memory) observation."""
        raw = self._raw_obs()
        if self._frames is None or self.n_stack == 1:
            self._frames = deque([raw] * self.n_stack, maxlen=self.n_stack)
        else:
            self._frames.append(raw)
        return np.concatenate(list(self._frames)).astype(np.float32)

    def _reset_frames(self) -> np.ndarray:
        raw = self._raw_obs()
        self._frames = deque([raw] * self.n_stack, maxlen=self.n_stack)
        return np.concatenate(list(self._frames)).astype(np.float32)

    def action_to_command(self, action: np.ndarray) -> Command:
        a = np.clip(action, -1.0, 1.0)
        throttle = float((a[0] + 1.0) / 2.0)
        gx = float(a[1] * self.base.gimbal_limit)
        gy = float(a[2] * self.base.gimbal_limit)
        return Command(throttle=throttle, gimbal_x=gx, gimbal_y=gy)

    def command_to_action(self, cmd: Command) -> np.ndarray:
        """Inverse map, so a PID controller can be evaluated through this env."""
        a0 = 2.0 * cmd.throttle - 1.0
        a1 = cmd.gimbal_x / self.base.gimbal_limit
        a2 = cmd.gimbal_y / self.base.gimbal_limit
        return np.clip(np.array([a0, a1, a2], dtype=np.float32), -1.0, 1.0)

    # --- gym API ------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self.vehicle = self.randomization.sample_vehicle(self.base, self._rng)
        self.disturbance.reset(self._rng)
        self.state = self.scenario.sample_initial_state(self._rng, scale=self.init_scale)
        self.t = 0.0
        self._prev_gimbal = np.zeros(2)
        self._prev_phi = self.scenario.potential(self.state)
        if self.residual:
            from ..controllers import LandingPID

            self._base_ctrl = LandingPID(self.base, self.world)  # fresh per episode
        return self._reset_frames(), {}

    def step(self, action: np.ndarray):
        if self.residual:
            # base PID action for the current state + bounded policy correction
            base_a = self.command_to_action(self._base_ctrl(self.t, self.state))
            action = np.clip(base_a + self.residual_scale * np.asarray(action), -1.0, 1.0)
        cmd = self.action_to_command(action)
        target_gimbal = np.array([cmd.gimbal_x, cmd.gimbal_y])
        max_delta = self.vehicle.gimbal_rate_limit * self.sim_dt

        reason = ""
        for _ in range(self.n_sub):
            delta = np.clip(target_gimbal - self._prev_gimbal, -max_delta, max_delta)
            self._prev_gimbal = self._prev_gimbal + delta
            applied = np.array([cmd.throttle, self._prev_gimbal[0], self._prev_gimbal[1]])

            wind, fext = self.disturbance.step(self.t, self.sim_dt, self._rng)
            self.state = dyn.rk4_step(
                self.state, applied, self.sim_dt, self.vehicle, self.world, wind, fext
            )
            self.t += self.sim_dt
            done, reason = self.scenario.check_done(self.state, self.t)
            if done:
                break

        # Potential-difference shaping (Phi' - Phi): a fixed point gives ZERO
        # shaping, so hovering can't farm reward. A small step penalty actively
        # discourages dithering. Terminal reward supplies the landing objective.
        phi = self.scenario.potential(self.state)
        reward = (phi - self._prev_phi) - self.step_penalty
        self._prev_phi = phi
        if reason:
            reward += self.scenario.terminal_reward(self.state, reason)

        terminated = reason in ("touchdown", "crash_tilt", "out_of_bounds", "too_high")
        truncated = reason == "timeout"
        info = {"reason": reason}
        if terminated or truncated:
            info["success"] = bool(reason == "touchdown" and self.scenario.is_soft_landing(self.state))
        return self._obs(), float(reward), terminated, truncated, info


def make_landing_env(difficulty: str = "calm", **kwargs) -> LandingEnv:
    """Factory: difficulty in {calm, moderate, hard, unknown}."""
    from ..scenarios import disturbances as D

    presets = {"calm": D.calm, "moderate": D.moderate, "hard": D.hard, "unknown": D.model_unknown}
    dist, rand = presets[difficulty]()
    scenario = LandingScenario.hard() if difficulty == "hard" else LandingScenario()
    return LandingEnv(scenario=scenario, disturbance=dist, randomization=rand, **kwargs)
