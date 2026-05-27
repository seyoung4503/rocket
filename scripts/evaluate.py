"""Evaluate a controller in the landing env — PID baseline or a trained RL model.

    python scripts/evaluate.py --policy pid --difficulty hard --episodes 100
    python scripts/evaluate.py --policy models/ppo_hard.zip --difficulty hard

Both go through the SAME environment (same disturbances, randomization, control
rate), so the success rates are directly comparable.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import LandingPID  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402


def eval_pid(env, n):
    succ, landed, vsp = 0, 0, []
    for ep in range(n):
        env.reset(seed=ep)
        pid = LandingPID(env.base, env.world)  # nominal model only
        done = False
        while not done:
            cmd = pid(env.t, env.state)
            obs, r, term, trunc, info = env.step(env.command_to_action(cmd))
            done = term or trunc
        succ += int(info.get("success", False))
        if info.get("reason") == "touchdown":
            landed += 1
            vsp.append(-env.state[3 + 2])
    return succ, landed, vsp


def eval_model(env, model, n):
    succ, landed, vsp = 0, 0, []
    for ep in range(n):
        obs, _ = env.reset(seed=ep)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(action)
            done = term or trunc
        succ += int(info.get("success", False))
        if info.get("reason") == "touchdown":
            landed += 1
            vsp.append(-env.state[3 + 2])
    return succ, landed, vsp


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="pid", help="'pid' or path to an SB3 .zip model")
    ap.add_argument("--difficulty", default="hard", choices=["calm", "moderate", "hard"])
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--residual", action="store_true", help="evaluate a residual-RL model (PID + correction)")
    ap.add_argument("--residual-scale", type=float, default=0.4)
    args = ap.parse_args()

    env = make_landing_env(
        args.difficulty, residual=args.residual, residual_scale=args.residual_scale
    )

    if args.policy == "pid":
        label = "PID"
        succ, landed, vsp = eval_pid(env, args.episodes)
    else:
        from stable_baselines3 import PPO, SAC

        loader = SAC if "sac" in args.policy.lower() else PPO
        model = loader.load(args.policy)
        label = os.path.basename(args.policy)
        succ, landed, vsp = eval_model(env, model, args.episodes)

    n = args.episodes
    print(f"=== {label} @ {args.difficulty}  (n={n}, in landing env @ 50 Hz) ===")
    print(f"  soft landings : {succ}/{n}  ({100*succ/n:.0f}%)")
    print(f"  reached ground: {landed}/{n}")
    if vsp:
        print(f"  touchdown vspeed: mean {np.mean(vsp):.2f}  worst {np.max(vsp):.2f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
