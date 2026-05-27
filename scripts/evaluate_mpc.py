"""Quick PID vs sampled vertical-MPC comparison in the landing env.

The MPC controller is intentionally small: it plans only vertical throttle with
a 1D model, while HoverPID still handles horizontal centering and TVC attitude.

    python scripts/evaluate_mpc.py --difficulty calm --episodes 20
    python scripts/evaluate_mpc.py --difficulty hard --episodes 50
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import LandingPID, LandingVerticalMPC  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402


def eval_controller(env, factory, n: int):
    succ = 0
    landed = 0
    reasons: dict[str, int] = {}
    fail = {"offset": 0, "vspeed": 0, "hspeed": 0, "tilt": 0}
    touchdown = []
    energy = []

    for ep in range(n):
        env.reset(seed=ep)
        ctrl = factory(env)
        done = False
        ep_energy = 0.0
        while not done:
            cmd = ctrl(env.t, env.measured)
            ep_energy += cmd.throttle * env.control_dt
            _, _, term, trunc, info = env.step(env.command_to_action(cmd))
            done = term or trunc

        reason = info.get("reason", "")
        reasons[reason] = reasons.get(reason, 0) + 1
        ok = bool(info.get("success", False))
        succ += int(ok)
        energy.append(ep_energy)

        if reason == "touchdown":
            landed += 1
            pos = env.state[dyn.POS]
            vel = env.state[dyn.VEL]
            metrics = {
                "offset": float(np.linalg.norm(pos[:2])),
                "vspeed": float(max(0.0, -vel[2])),
                "hspeed": float(np.linalg.norm(vel[:2])),
                "tilt": float(np.rad2deg(quat.tilt_angle(env.state[dyn.QUAT]))),
            }
            touchdown.append(metrics)
            if not ok:
                if metrics["offset"] > env.scenario.max_touchdown_offset:
                    fail["offset"] += 1
                if metrics["vspeed"] > env.scenario.max_touchdown_vspeed:
                    fail["vspeed"] += 1
                if metrics["hspeed"] > env.scenario.max_touchdown_hspeed:
                    fail["hspeed"] += 1
                if metrics["tilt"] > env.scenario.max_touchdown_tilt_deg:
                    fail["tilt"] += 1

    return {
        "success": succ,
        "landed": landed,
        "reasons": reasons,
        "fail": fail,
        "touchdown": touchdown,
        "energy": energy,
    }


def print_result(label: str, result, n: int) -> None:
    print(f"=== {label} (n={n}) ===")
    print(f"  soft landings : {result['success']}/{n} ({100 * result['success'] / n:.0f}%)")
    print(f"  reached ground: {result['landed']}/{n}")
    print(f"  reasons       : {result['reasons']}")
    print(f"  throttle int  : mean {np.mean(result['energy']):.2f}")
    if result["touchdown"]:
        arr = np.array(
            [
                [m["offset"], m["vspeed"], m["hspeed"], m["tilt"]]
                for m in result["touchdown"]
            ]
        )
        print(
            "  touchdown mean: "
            f"offset {arr[:, 0].mean():.2f} m, "
            f"vspeed {arr[:, 1].mean():.2f} m/s, "
            f"hspeed {arr[:, 2].mean():.2f} m/s, "
            f"tilt {arr[:, 3].mean():.1f} deg"
        )
    if result["success"] < result["landed"]:
        print(f"  landed fail   : {result['fail']}")
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--difficulty",
        default="calm",
        choices=["calm", "moderate", "hard", "unknown", "recovery", "noisy"],
    )
    ap.add_argument("--episodes", type=int, default=20)
    args = ap.parse_args()

    print(f"PID vs sampled vertical MPC @ {args.difficulty}")
    print("MPC controls vertical throttle only; HoverPID still handles TVC attitude.\n")

    env = make_landing_env(args.difficulty)
    pid = eval_controller(env, lambda e: LandingPID(e.base, e.world), args.episodes)
    print_result("PID", pid, args.episodes)

    env = make_landing_env(args.difficulty)
    mpc = eval_controller(env, lambda e: LandingVerticalMPC(e.base, e.world), args.episodes)
    print_result("PID + VerticalMPC", mpc, args.episodes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
