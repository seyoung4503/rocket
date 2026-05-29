"""Diagnose *why* Step 2 (`actuator2`) closed-loop beats Step 1 (`actuator`)
when the open-loop plan-mismatch numbers are essentially identical.

Hypothesis from `docs/2026-05-29_1530_v1_step2_maglag_analysis.md`:

    The Step-2 gains do NOT come from a more realizable plan (the open-loop
    diagnostic shows the same gimbal saturation). They come from a *smoother*
    reference signal that the PID waypoint tracker can follow more cleanly.

This script tests that hypothesis by running BOTH controllers on the SAME
seeds and recording, per step:

  * commanded throttle (the actual command sent to the simulator)
  * commanded gimbal x/y
  * vehicle tilt over time

For each seed it then reports the RMS rate-of-change of throttle and gimbal
commands (a smoothness metric). The hypothesis predicts that actuator2's
throttle rate-of-change RMS is significantly lower than actuator's, even
though their plan-vs-rollout numbers match.

It also identifies per-seed outcome flips (S1 succeeds vs S2 succeeds) and
compares smoothness between flip cases and non-flip cases.
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402
from rocketsim.navigation.estimator import LowPassEstimatorConfig  # noqa: E402


CONTROLLERS = {
    "actuator": "LandingActuatorAwareWaypointPID",
    "actuator2": "LandingActuatorAwareMagLagWaypointPID",
}


def make_controller(name: str, env):
    from rocketsim import controllers as ctrl_mod
    cls = getattr(ctrl_mod, CONTROLLERS[name])
    return cls(env.base, env.world)


def rms_rate(arr: np.ndarray, dt: float) -> float:
    if len(arr) < 2:
        return 0.0
    d = np.diff(arr) / dt
    return float(np.sqrt(np.mean(d * d)))


def run_one(args):
    difficulty, seed, controller_name = args
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = make_controller(controller_name, env)
    estimator = LowPassStateEstimator(
        LowPassEstimatorConfig.for_obs_noise(getattr(env, "obs_noise", 0.0))
    )
    est = estimator.reset(env.measured)

    throttle, gx, gy, tilt = [], [], [], []
    done = False
    while not done:
        cmd = ctrl(env.t, est)
        throttle.append(cmd.throttle)
        gx.append(cmd.gimbal_x)
        gy.append(cmd.gimbal_y)
        tilt.append(float(np.rad2deg(quat.tilt_angle(env.state[dyn.QUAT]))))
        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        est = estimator.update(env.measured, env.control_dt)
        done = term or trunc

    dt = env.control_dt
    return {
        "seed": seed,
        "controller": controller_name,
        "success": bool(info.get("success", False)),
        "reason": info.get("reason", ""),
        "n_steps": len(throttle),
        "throttle_rms_rate": rms_rate(np.array(throttle), dt),
        "gimbal_x_rms_rate": rms_rate(np.array(gx), dt),
        "gimbal_y_rms_rate": rms_rate(np.array(gy), dt),
        "throttle_std": float(np.std(throttle)),
        "max_tilt": float(np.max(tilt)) if tilt else 0.0,
        "mean_tilt": float(np.mean(tilt)) if tilt else 0.0,
    }


def summarize(rows: list[dict], label: str) -> None:
    a = np.array([r["throttle_rms_rate"] for r in rows])
    gx = np.array([r["gimbal_x_rms_rate"] for r in rows])
    gy = np.array([r["gimbal_y_rms_rate"] for r in rows])
    tmax = np.array([r["max_tilt"] for r in rows])
    tmean = np.array([r["mean_tilt"] for r in rows])
    n_succ = sum(r["success"] for r in rows)
    print(f"  {label} (n={len(rows)}, success={n_succ})")
    print(
        f"    throttle |d/dt| RMS  : mean {a.mean():.3f}  median {np.median(a):.3f}  max {a.max():.3f}"
    )
    print(
        f"    gimbal_x |d/dt| RMS  : mean {gx.mean():.3f}  median {np.median(gx):.3f}"
    )
    print(
        f"    gimbal_y |d/dt| RMS  : mean {gy.mean():.3f}  median {np.median(gy):.3f}"
    )
    print(
        f"    tilt max (deg)       : mean {tmax.mean():.1f}  median {np.median(tmax):.1f}"
    )
    print(
        f"    tilt mean (deg)      : mean {tmean.mean():.1f}  median {np.median(tmean):.1f}"
    )


def analyze_flips(rows_a: list[dict], rows_b: list[dict]) -> None:
    by_seed_a = {r["seed"]: r for r in rows_a}
    by_seed_b = {r["seed"]: r for r in rows_b}
    seeds = sorted(set(by_seed_a) & set(by_seed_b))

    only_a = [s for s in seeds if by_seed_a[s]["success"] and not by_seed_b[s]["success"]]
    only_b = [s for s in seeds if by_seed_b[s]["success"] and not by_seed_a[s]["success"]]
    both = [s for s in seeds if by_seed_a[s]["success"] and by_seed_b[s]["success"]]
    none = [s for s in seeds if not by_seed_a[s]["success"] and not by_seed_b[s]["success"]]

    print(f"  flip analysis (n={len(seeds)} seeds):")
    print(f"    both succeed         : {len(both)}")
    print(f"    only Step1 (actuator): {len(only_a)} {only_a if only_a else ''}")
    print(f"    only Step2 (actuator2): {len(only_b)} {only_b if only_b else ''}")
    print(f"    both fail            : {len(none)}")

    def diff_rms(seeds_sub: list[int], key: str) -> tuple[float, float]:
        if not seeds_sub:
            return 0.0, 0.0
        a_vals = np.array([by_seed_a[s][key] for s in seeds_sub])
        b_vals = np.array([by_seed_b[s][key] for s in seeds_sub])
        return float((a_vals - b_vals).mean()), float(
            np.median(a_vals - b_vals)
        )

    print("    paired diff (S1 − S2), positive = S2 smoother / smaller:")
    for key in ("throttle_rms_rate", "gimbal_x_rms_rate", "gimbal_y_rms_rate", "max_tilt", "mean_tilt"):
        m, med = diff_rms(seeds, key)
        print(f"      ALL  {key:>22}: mean Δ {m:+.3f}  median Δ {med:+.3f}")
    if only_b:
        for key in ("throttle_rms_rate", "gimbal_x_rms_rate", "gimbal_y_rms_rate"):
            m, med = diff_rms(only_b, key)
            print(f"      ONLY_S2_WIN {key:>15}: mean Δ {m:+.3f}  median Δ {med:+.3f}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--difficulties", default="hard,noisy")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--workers", type=int, default=6)
    args = ap.parse_args()

    difficulties = [d.strip() for d in args.difficulties.split(",") if d.strip()]
    for difficulty in difficulties:
        print(f"\n=== {difficulty} (n={args.episodes}) ===")
        tasks = []
        for ctrl in CONTROLLERS:
            for ep in range(args.episodes):
                tasks.append((difficulty, ep, ctrl))
        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            results = list(ex.map(run_one, tasks, chunksize=1))
        rows_a = [r for r in results if r["controller"] == "actuator"]
        rows_b = [r for r in results if r["controller"] == "actuator2"]
        summarize(rows_a, "Step 1 (actuator)")
        summarize(rows_b, "Step 2 (actuator2)")
        analyze_flips(rows_a, rows_b)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
