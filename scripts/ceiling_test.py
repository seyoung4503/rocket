"""Is the ~60% hard-landing rate a physical ceiling or a strategy limit?

Runs the PID baseline on 'hard' while progressively relaxing the actuator limits
(gimbal travel, gimbal slew rate, thrust spool-up lag). If success jumps, the
ceiling is set by actuators (no controller — PID or RL — beats it on the stock
vehicle). If it stays flat, the limit is strategy/observation and RL has room.

    python scripts/ceiling_test.py [episodes]
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim.controllers import LandingPID  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402


def eval_pid(env, n):
    succ = 0
    vsp = []
    for ep in range(n):
        env.reset(seed=ep)
        pid = LandingPID(env.base, env.world)
        done = False
        while not done:
            obs, r, term, trunc, info = env.step(env.command_to_action(pid(env.t, env.state)))
            done = term or trunc
        succ += int(info.get("success", False))
        if info.get("reason") == "touchdown":
            vsp.append(-env.state[5])
    return succ, (np.mean(vsp) if vsp else float("nan"))


def run(label, n, **overrides):
    env = make_landing_env("hard")
    for k, v in overrides.items():  # override the base vehicle the env/PID use
        setattr(env.base, k, v)
    succ, vsp = eval_pid(env, n)
    print(f"  {label:42s}: {succ:3d}/{n}  ({100*succ/n:3.0f}%)  td_vspeed {vsp:.2f}")


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    d2r = np.deg2rad
    print(f"PID on 'hard' with relaxed actuators (n={n} each)\n")
    print("  config                                     : success      detail")
    run("stock (gimbal12 rate200 tau0.08)", n)
    run("gimbal 30deg", n, gimbal_limit=d2r(30))
    run("gimbal rate 2000deg/s", n, gimbal_rate_limit=d2r(2000))
    run("thrust lag tau 0.02", n, thrust_time_constant=0.02)
    run("ALL idealized", n, gimbal_limit=d2r(30), gimbal_rate_limit=d2r(2000), thrust_time_constant=0.02)
    print("\nIf 'ALL idealized' >> stock -> ceiling is actuator-limited.")
    print("If it stays ~stock     -> limit is strategy/observation (RL has room).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
