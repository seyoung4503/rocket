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
        obs_noise: float = 0.0,
        seed: int | None = None,
    ):
        super().__init__()
        self.step_penalty = step_penalty
        self.init_scale = init_scale
        # Sensor noise / partial observability: the controller sees a noisy
        # MEASUREMENT, not the true state. PID's derivative (damping) terms
        # amplify velocity/rate noise -> jitter; a memory policy can average it
        # out over frames. obs_noise scales the baseline measurement stds.
        self.obs_noise = obs_noise
        self.measured = None
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

        # 4-dim action: [throttle (-1..1 -> 0..1), gimbal_x, gimbal_y, roll_cmd]
        # The 4th channel feeds the optional exhaust-vane roll torque
        # (Vehicle.edf_vane_torque_max); when the vane is disabled the
        # value is ignored, so 4-dim works on vanilla EDFs too.
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        high = np.full(13 * self.n_stack, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self._rng = np.random.default_rng(seed)
        self.vehicle = self.base
        self.state = dyn.initial_state()
        self.t = 0.0
        self._prev_gimbal = np.zeros(2)

    # --- helpers ------------------------------------------------------------

    def _measure(self) -> np.ndarray:
        """Noisy measurement of the true state (what the controller is allowed
        to see). Returns the true state when obs_noise == 0."""
        from .. import quaternion as quat

        s = self.state.copy()
        k = self.obs_noise
        if k <= 0.0:
            return s
        rng = self._rng
        s[dyn.POS] += rng.normal(0.0, 0.05 * k, 3)
        s[dyn.VEL] += rng.normal(0.0, 0.20 * k, 3)  # velocity is the noisy one
        s[dyn.OMEGA] += rng.normal(0.0, 0.08 * k, 3)
        # attitude: perturb by a small random rotation
        a = rng.normal(0.0, np.deg2rad(1.5) * k, 3)
        dq = quat.quat_normalize(np.array([1.0, a[0] / 2, a[1] / 2, a[2] / 2]))
        s[dyn.QUAT] = quat.quat_normalize(quat.quat_mult(s[dyn.QUAT], dq))
        return s

    def _raw_obs(self) -> np.ndarray:
        from .. import quaternion as quat

        s = self.measured if self.measured is not None else self.state
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
        # The 4th channel is the raw roll vane command in [-1, 1].  Older
        # 3-element actions (e.g. from RL models trained before vane
        # support) default to roll 0 so they still work.
        roll_cmd = float(a[3]) if a.shape[0] > 3 else 0.0
        return Command(
            throttle=throttle, gimbal_x=gx, gimbal_y=gy, roll_cmd=roll_cmd
        )

    def command_to_action(self, cmd: Command) -> np.ndarray:
        """Inverse map, so a PID controller can be evaluated through this env."""
        a0 = 2.0 * cmd.throttle - 1.0
        a1 = cmd.gimbal_x / self.base.gimbal_limit
        a2 = cmd.gimbal_y / self.base.gimbal_limit
        a3 = float(cmd.roll_cmd)
        return np.clip(
            np.array([a0, a1, a2, a3], dtype=np.float32), -1.0, 1.0
        )

    # --- gym API ------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self.vehicle = self.randomization.sample_vehicle(self.base, self._rng)
        self.disturbance.reset(self._rng)
        self.state = self.scenario.sample_initial_state(self._rng, scale=self.init_scale)
        self.t = 0.0
        self._prev_gimbal = np.zeros(2)
        # Divert one-shot guard. Set True after the pad-shift event fires so
        # subsequent steps don't re-translate the state.
        self._pad_shifted = False
        self.measured = self._measure()
        self._prev_phi = self.scenario.potential(self.state)
        if self.residual:
            from ..controllers import LandingPID

            self._base_ctrl = LandingPID(self.base, self.world)  # fresh per episode
        return self._reset_frames(), {}

    def step(self, action: np.ndarray):
        if self.residual:
            # base PID action (on the measured state) + bounded policy correction
            base_a = self.command_to_action(self._base_ctrl(self.t, self.measured))
            action = np.clip(base_a + self.residual_scale * np.asarray(action), -1.0, 1.0)
        cmd = self.action_to_command(action)
        target_gimbal = np.array([cmd.gimbal_x, cmd.gimbal_y])
        max_delta = self.vehicle.gimbal_rate_limit * self.sim_dt

        reason = ""
        for _ in range(self.n_sub):
            delta = np.clip(target_gimbal - self._prev_gimbal, -max_delta, max_delta)
            self._prev_gimbal = self._prev_gimbal + delta
            applied = np.array([
                cmd.throttle,
                self._prev_gimbal[0],
                self._prev_gimbal[1],
                cmd.roll_cmd,
            ])

            wind, fext = self.disturbance.step(self.t, self.sim_dt, self._rng)
            prev_t = self.t
            self.state = dyn.rk4_step(
                self.state, applied, self.sim_dt, self.vehicle, self.world, wind, fext
            )
            self.t += self.sim_dt
            # In-flight pad shift (divert scenario). If t just crossed the
            # shift moment, rigidly translate the stored POS so subsequent
            # pad-relative checks treat the new pad as the origin. Dynamics
            # are inertial-translation invariant, so this is a pure
            # change-of-origin.
            shift_t = self.scenario.pad_shift_time
            if (
                shift_t is not None
                and not self._pad_shifted
                and prev_t < shift_t <= self.t
            ):
                self.state[dyn.POS][:2] -= np.asarray(
                    self.scenario.pad_shift_delta_xy, dtype=float
                )
                self._pad_shifted = True
            done, reason = self.scenario.check_done(self.state, self.t)
            if done:
                break

        self.measured = self._measure()  # noisy measurement for obs + next-step PID

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


def make_landing_env(
    difficulty: str = "calm",
    edf_roll: bool = False,
    edf_roll_scale: float = 1.0,
    edf_vane: bool = False,
    edf_vane_torque_max: float = 0.5,
    **kwargs,
) -> LandingEnv:
    """Factory: difficulty in
    {calm, moderate, hard, unknown, recovery, noisy, divert, divert_hard}.

    ``edf_roll=True`` toggles the EDF fan reaction torque + gyroscopic
    precession on the vehicle (see docs/2026-05-29_2108).  Default off so
    existing experiments stay backward compatible.  ``edf_roll_scale``
    multiplies both the reaction-torque coefficient and the gyroscopic
    angular momentum (= modeling partial counter-rotation cancellation
    or an EDF that is intrinsically less roll-noisy); 1.0 = single-fan
    full strength, 0.1 = ~90 % cancelled by a counter-rotating fan.
    """
    from ..scenarios import disturbances as D

    presets = {"calm": D.calm, "moderate": D.moderate, "hard": D.hard, "unknown": D.model_unknown}
    if difficulty == "recovery":
        dist, rand = D.moderate()  # mild wind/plant — isolate the attitude recovery
        scenario = LandingScenario.recovery()
    elif difficulty == "noisy":
        dist, rand = D.moderate()  # mild wind/plant — isolate the sensor-noise effect
        scenario = LandingScenario()
        kwargs.setdefault("obs_noise", 2.0)  # heavy noise: PID (no filter) drops to ~21%
    elif difficulty == "divert":
        # In-flight pad shift on top of the moderate IC + moderate disturbance.
        # Designed to exercise receding-horizon replanning; see
        # docs/2026-05-29_1531_v1_divert_scenario_plan.md.
        dist, rand = D.moderate()
        scenario = LandingScenario.divert()
    elif difficulty == "divert_hard":
        # divert composed with the hard IC + hard disturbance preset.
        dist, rand = D.hard()
        scenario = LandingScenario.divert_hard()
    elif difficulty == "spin":
        # Vehicle starts with ~286 deg/s body-z spin on top of an
        # otherwise moderate IC.  Designed to be combined with
        # edf_roll=True + edf_vane=True so the roll-PID wrapper has
        # something to damp.
        dist, rand = D.moderate()
        scenario = LandingScenario.spin()
    else:
        dist, rand = presets[difficulty]()
        scenario = LandingScenario.hard() if difficulty == "hard" else LandingScenario()
    env = LandingEnv(scenario=scenario, disturbance=dist, randomization=rand, **kwargs)
    if edf_roll:
        # Enable EDF fan reaction torque + gyroscopic precession on both
        # the controller-facing base vehicle and the simulator's vehicle.
        # Values follow the design doc estimates for a small 90 mm EDF,
        # scaled by edf_roll_scale to model counter-rotating-fan cancellation.
        scale = float(edf_roll_scale)
        for v in (env.base, env.vehicle):
            v.edf_roll_coeff = 0.012 * scale  # N·m per N thrust
            v.edf_fan_inertia = 1.0e-4 * scale  # kg·m² (effective net spin)
            v.edf_fan_omega_max = 4000.0  # rad/s at max thrust (per fan)
    if edf_vane:
        # Equip the vehicle with V-2-style exhaust vanes for active
        # roll control.  Authority = edf_vane_torque_max * (thrust/Tmax)
        # in body-z when the controller drives Command.roll_cmd.
        for v in (env.base, env.vehicle):
            v.edf_vane_torque_max = float(edf_vane_torque_max)
    return env
