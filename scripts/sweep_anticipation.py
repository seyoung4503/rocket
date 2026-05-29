"""Sweep `anticipation` for the trajectory tracker.

If our `lookahead=10` trick is just "look 2s into the plan", then setting
`anticipation=2.0` on the time-indexed tracker should reproduce its
behavior. This sweep tests that hypothesis: if tracking_actuator at
anticipation=2.0 matches actuator (la=10) ~ 60-90%, then the difference
between the two architectures collapses to "where on the plan do you
sample".
"""

from __future__ import annotations

import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim.controllers import LandingActuatorTrackingMPC  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402
from rocketsim.navigation.estimator import LowPassEstimatorConfig  # noqa: E402


def run_one(args):
    difficulty, seed, anticipation = args
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = LandingActuatorTrackingMPC(
        env.base, env.world, anticipation=anticipation
    )
    estimator = LowPassStateEstimator(
        LowPassEstimatorConfig.for_obs_noise(getattr(env, "obs_noise", 0.0))
    )
    est = estimator.reset(env.measured)
    done = False
    while not done:
        cmd = ctrl(env.t, est)
        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        est = estimator.update(env.measured, env.control_dt)
        done = term or trunc
    return {
        "anticipation": anticipation,
        "difficulty": difficulty,
        "seed": seed,
        "success": bool(info.get("success", False)),
        "reason": info.get("reason", ""),
    }


def main():
    n = 50
    difficulties = ["hard", "noisy", "divert", "divert_hard"]
    anticipations = [0.0, 0.5, 1.0, 2.0]
    tasks = []
    for d in difficulties:
        for a in anticipations:
            for s in range(n):
                tasks.append((d, s, a))
    with ProcessPoolExecutor(max_workers=6) as ex:
        results = list(ex.map(run_one, tasks, chunksize=1))

    for d in difficulties:
        print(f"\n=== {d} (n={n}, estimated) ===")
        for a in anticipations:
            rows = [r for r in results if r["difficulty"] == d and r["anticipation"] == a]
            succ = sum(r["success"] for r in rows)
            reasons = {}
            for r in rows:
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
            print(f"  anticipation={a:>3.1f}s: {succ}/{n} ({100*succ/n:.0f}%)  reasons={reasons}")


if __name__ == "__main__":
    main()
