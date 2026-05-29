"""Sweep `lookahead` for the actuator-aware waypoint wrapper.

The hover-bug diagnostic in
`docs/2026-05-29_15..._v1_hover_bug_diagnosis.md` identified that the wrapper
samples the MPC plan at `lookahead=4` steps (= 0.8s) which sits in the slow
early portion of an MPC plan that ITSELF is correct (terminal at the pad).
This script tests whether simply sampling further along the plan fixes the
mild-divert hover-and-timeout failure mode.

We sweep lookahead in {4 (baseline), 10, 19} for `actuator` and report
success rate on divert (mild), divert_hard, hard, noisy.
"""

from __future__ import annotations

import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim.controllers import LandingActuatorAwareWaypointPID  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402
from rocketsim.navigation.estimator import LowPassEstimatorConfig  # noqa: E402


def run_one(args):
    difficulty, seed, lookahead = args
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = LandingActuatorAwareWaypointPID(
        env.base, env.world, lookahead=lookahead
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
        "lookahead": lookahead,
        "difficulty": difficulty,
        "seed": seed,
        "success": bool(info.get("success", False)),
        "reason": info.get("reason", ""),
    }


def main():
    n = 50
    difficulties = ["divert", "divert_hard", "hard", "noisy"]
    lookaheads = [4, 10, 19]  # 4 = current, 10 = 2s, 19 = horizon end
    tasks = []
    for d in difficulties:
        for la in lookaheads:
            for s in range(n):
                tasks.append((d, s, la))
    with ProcessPoolExecutor(max_workers=6) as ex:
        results = list(ex.map(run_one, tasks, chunksize=1))

    for d in difficulties:
        print(f"\n=== {d} (n={n}, estimated) ===")
        for la in lookaheads:
            rows = [r for r in results if r["difficulty"] == d and r["lookahead"] == la]
            succ = sum(r["success"] for r in rows)
            reasons = {}
            for r in rows:
                reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
            print(f"  lookahead={la:>3}: {succ}/{n} ({100 * succ / n:.0f}%)  reasons={reasons}")


if __name__ == "__main__":
    main()
