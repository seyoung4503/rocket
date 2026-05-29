"""Diagnose Step 3 (linearized 6-DOF) linearization error vs Step 1.

The single-shot linearization around hover is exact at small tilt but
gets sloppy as the vehicle tilts away from vertical:
    R(q) ê_z      = (sin θ · n̂_y, -sin θ · n̂_x, cos θ)         (exact)
    linearized    = (θ · n̂_y,    -θ · n̂_x,    1)              (Step 3 sees this)

We want quantitative answers to two questions before committing to a
full iterative SCP implementation:

  1. How often does the vehicle actually tilt past the small-angle
     regime (10°, 15°, 20°) in each scenario?  This tells us how often
     the linearization error matters.

  2. Does Step 3's plan track the realized 6-DOF rollout *better* than
     Step 1's plan does?  We compare predicted next-step position
     (from the MPC plan) against the actual next-step position over
     n=20 episodes per (controller, difficulty).

Output is a single compact table — no plots.
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
    "scp": "LandingScp6DofWaypointPID",
}


def _make_ctrl(name: str, env):
    from rocketsim import controllers as ctrl_mod
    cls = getattr(ctrl_mod, CONTROLLERS[name])
    return cls(env.base, env.world)


def _tilt_deg(q: np.ndarray) -> float:
    return float(np.rad2deg(quat.tilt_angle(np.asarray(q, dtype=float))))


def run_one(args):
    difficulty, seed, controller_name = args
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = _make_ctrl(controller_name, env)
    estimator = LowPassStateEstimator(
        LowPassEstimatorConfig.for_obs_noise(getattr(env, "obs_noise", 0.0))
    )
    est = estimator.reset(env.measured)

    tilts: list[float] = []
    pred_errors: list[float] = []  # ‖p_plan_next − p_actual_next‖ per step

    done = False
    last_plan_p_next: np.ndarray | None = None
    while not done:
        # Record tilt of the *actual* vehicle state (truth, not estimate).
        tilts.append(_tilt_deg(env.state[dyn.QUAT]))

        # If we have a plan-predicted next-step position from the
        # previous loop iteration, score it against the actual current
        # position (= the next-step position from that vantage).
        if last_plan_p_next is not None:
            err = float(np.linalg.norm(last_plan_p_next - env.state[dyn.POS]))
            pred_errors.append(err)

        cmd = ctrl(env.t, est)

        # Grab the MPC's planned next-step position right after this
        # controller call.  The wrapper stores plans on different
        # attributes; sniff for either the cvxpy variable .value or
        # the trajectory-tracker-style stored arrays.
        plan_p = None
        if hasattr(ctrl, "mpc"):
            mpc = ctrl.mpc
            if hasattr(mpc, "p") and hasattr(mpc.p, "value") and mpc.p.value is not None:
                plan_p = np.asarray(mpc.p.value, dtype=float)
        if plan_p is not None and plan_p.shape[1] > 1:
            # plan_p[:,0] is the planning-time state, plan_p[:,1] is the
            # planner's prediction one mpc.dt into the future.  The
            # controller steps the env by control_dt which equals
            # mpc.dt in our setup (0.2 s).
            last_plan_p_next = plan_p[:, 1].copy()
        else:
            last_plan_p_next = None

        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        est = estimator.update(env.measured, env.control_dt)
        done = term or trunc

    return {
        "difficulty": difficulty,
        "controller": controller_name,
        "seed": seed,
        "n_steps": len(tilts),
        "max_tilt": float(np.max(tilts)) if tilts else 0.0,
        "mean_tilt": float(np.mean(tilts)) if tilts else 0.0,
        "p50_tilt": float(np.percentile(tilts, 50)) if tilts else 0.0,
        "p90_tilt": float(np.percentile(tilts, 90)) if tilts else 0.0,
        "p99_tilt": float(np.percentile(tilts, 99)) if tilts else 0.0,
        "frac_tilt_gt10": float(np.mean(np.array(tilts) > 10.0)) if tilts else 0.0,
        "frac_tilt_gt20": float(np.mean(np.array(tilts) > 20.0)) if tilts else 0.0,
        "med_pred_err": float(np.median(pred_errors)) if pred_errors else 0.0,
        "p90_pred_err": float(np.percentile(pred_errors, 90)) if pred_errors else 0.0,
        "max_pred_err": float(np.max(pred_errors)) if pred_errors else 0.0,
        "success": bool(info.get("success", False)),
    }


def summarize(rows: list[dict], controller: str, difficulty: str) -> None:
    sub = [r for r in rows if r["controller"] == controller and r["difficulty"] == difficulty]
    if not sub:
        return
    arr = lambda key: np.array([r[key] for r in sub])
    print(
        f"  {controller:10s}  succ {sum(r['success'] for r in sub):>2}/{len(sub):>2}  "
        f"tilt(deg) mean={arr('mean_tilt').mean():5.2f}  "
        f"p50={arr('p50_tilt').mean():5.2f}  "
        f"p90={arr('p90_tilt').mean():5.2f}  "
        f"p99={arr('p99_tilt').mean():5.2f}  "
        f"max={arr('max_tilt').mean():5.2f}  "
        f"|  frac>10°={arr('frac_tilt_gt10').mean()*100:4.1f}%  "
        f">20°={arr('frac_tilt_gt20').mean()*100:4.1f}%  "
        f"|  pred err(m) med={arr('med_pred_err').mean():.3f}  "
        f"p90={arr('p90_pred_err').mean():.3f}  "
        f"max={arr('max_pred_err').mean():.3f}"
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--difficulties", default="hard,noisy,divert,divert_hard")
    ap.add_argument("--controllers", default="actuator,scp")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--workers", type=int, default=6)
    args = ap.parse_args()

    difficulties = [d.strip() for d in args.difficulties.split(",") if d.strip()]
    controllers = [c.strip() for c in args.controllers.split(",") if c.strip()]

    tasks = [
        (d, ep, ctrl)
        for d in difficulties
        for ctrl in controllers
        for ep in range(args.episodes)
    ]
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        rows = list(ex.map(run_one, tasks, chunksize=1))

    print(
        "Tilt distribution and 1-step plan-vs-rollout prediction error.  "
        f"n={args.episodes} episodes per (controller, difficulty).\n"
    )
    for d in difficulties:
        print(f"=== {d} ===")
        for c in controllers:
            summarize(rows, c, d)
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
