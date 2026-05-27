"""Fixed-step simulator that closes the loop around a controller.

A controller is any callable ``controller(t, state) -> Command``. The simulator
applies actuator rate limits to the gimbal before integrating, so the controller
sees realistic actuation lag (thrust lag is handled inside the dynamics).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from . import dynamics as dyn
from .dynamics import Command
from .vehicle import Environment, Vehicle


class Controller(Protocol):
    def __call__(self, t: float, state: np.ndarray) -> Command: ...


@dataclass
class Trajectory:
    t: list[float] = field(default_factory=list)
    states: list[np.ndarray] = field(default_factory=list)
    commands: list[np.ndarray] = field(default_factory=list)

    def as_arrays(self):
        return np.asarray(self.t), np.asarray(self.states), np.asarray(self.commands)


class Simulator:
    def __init__(
        self,
        vehicle: Vehicle,
        env: Environment | None = None,
        dt: float = 0.002,
        disturbance=None,
        rng: np.random.Generator | None = None,
    ):
        self.vehicle = vehicle
        self.env = env or Environment()
        self.dt = dt
        self.disturbance = disturbance
        self.rng = rng or np.random.default_rng()
        self._prev_gimbal = np.array([0.0, 0.0])

    def _rate_limit_gimbal(self, target: np.ndarray) -> np.ndarray:
        max_delta = self.vehicle.gimbal_rate_limit * self.dt
        delta = np.clip(target - self._prev_gimbal, -max_delta, max_delta)
        self._prev_gimbal = self._prev_gimbal + delta
        return self._prev_gimbal

    def run(
        self,
        controller: Controller,
        initial_state: np.ndarray,
        duration: float,
        terminate: Callable[[float, np.ndarray], bool] | None = None,
    ) -> Trajectory:
        """Run the closed loop. If ``terminate(t, state)`` returns True, stop
        after recording that step (used for landing/episode termination)."""
        state = initial_state.copy()
        self._prev_gimbal = np.array([0.0, 0.0])
        if self.disturbance is not None:
            self.disturbance.reset(self.rng)
        traj = Trajectory()
        n_steps = int(round(duration / self.dt))
        for i in range(n_steps):
            t = i * self.dt
            cmd = controller(t, state)
            gimbal = self._rate_limit_gimbal(np.array([cmd.gimbal_x, cmd.gimbal_y]))
            applied = np.array([cmd.throttle, gimbal[0], gimbal[1]])

            traj.t.append(t)
            traj.states.append(state.copy())
            traj.commands.append(applied.copy())

            if terminate is not None and terminate(t, state):
                break

            if self.disturbance is not None:
                wind, fext = self.disturbance.step(t, self.dt, self.rng)
            else:
                wind = fext = None
            state = dyn.rk4_step(state, applied, self.dt, self.vehicle, self.env, wind, fext)
        return traj
