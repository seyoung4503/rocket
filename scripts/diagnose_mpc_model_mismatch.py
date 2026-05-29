"""Diagnose point-mass MPC plan mismatch against the 6-DOF simulator.

This does not close the loop for a whole landing. It asks a narrower question:

    If the convex point-mass MPC plans a thrust-acceleration trajectory from the
    current state, how far does the real 6-DOF vehicle drift when a TVC tracker
    tries to follow that plan?

The rollout intentionally uses no future wind/external force. Remaining error
therefore comes from model structure mismatch, actuator/thrust lag, attitude
dynamics and plant parameter mismatch.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import (  # noqa: E402
    CvxpyActuatorAwareMagLagMPC,
    CvxpyActuatorAwareMPC,
    CvxpyPointMassMPC,
    LandingCvxpyMPC,
)
from rocketsim.envs import make_landing_env  # noqa: E402


PLANNERS = {
    "pointmass": CvxpyPointMassMPC,
    "actuator": CvxpyActuatorAwareMPC,
    "actuator2": CvxpyActuatorAwareMagLagMPC,
}


def run_episode(difficulty: str, seed: int, plant_model: str, mpc_name: str = "pointmass") -> dict:
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    model_vehicle = env.vehicle if plant_model == "oracle" else env.base
    PlannerCls = PLANNERS[mpc_name]
    planner = PlannerCls(model_vehicle, env.world)
    tracker = LandingCvxpyMPC(model_vehicle, env.world)

    plan = planner.plan(env.state[dyn.POS], env.state[dyn.VEL])
    if plan is None:
        return {"planned": False}
    p_pred, v_pred, u_pred = plan

    state = env.state.copy()
    gimbal = np.zeros(2)
    max_tilt = 0.0
    throttle_sat = 0
    gimbal_sat = 0
    steps = u_pred.shape[1]
    dt = planner.dt

    for k in range(steps):
        t = k * dt
        cmd = tracker._command_from_thrust_accel(t, state, u_pred[:, k])
        target_gimbal = np.array([cmd.gimbal_x, cmd.gimbal_y])
        max_delta = env.vehicle.gimbal_rate_limit * dt
        gimbal = gimbal + np.clip(target_gimbal - gimbal, -max_delta, max_delta)
        applied = np.array([cmd.throttle, gimbal[0], gimbal[1]])
        state = dyn.rk4_step(state, applied, dt, env.vehicle, env.world)

        max_tilt = max(max_tilt, float(np.rad2deg(quat.tilt_angle(state[dyn.QUAT]))))
        throttle_sat += int(cmd.throttle <= 1e-4 or cmd.throttle >= 1.0 - 1e-4)
        gimbal_sat += int(np.any(np.abs(target_gimbal) >= env.vehicle.gimbal_limit - 1e-5))
        if state[dyn.POS][2] <= 0.0:
            break

    p_final = p_pred[:, min(k + 1, p_pred.shape[1] - 1)]
    v_final = v_pred[:, min(k + 1, v_pred.shape[1] - 1)]
    pos_err = state[dyn.POS] - p_final
    vel_err = state[dyn.VEL] - v_final
    return {
        "planned": True,
        "pos_err": float(np.linalg.norm(pos_err)),
        "xy_err": float(np.linalg.norm(pos_err[:2])),
        "z_err": float(abs(pos_err[2])),
        "vel_err": float(np.linalg.norm(vel_err)),
        "final_alt": float(state[dyn.POS][2]),
        "pred_alt": float(p_final[2]),
        "final_tilt": float(np.rad2deg(quat.tilt_angle(state[dyn.QUAT]))),
        "max_tilt": max_tilt,
        "throttle_sat_frac": throttle_sat / max(1, k + 1),
        "gimbal_sat_frac": gimbal_sat / max(1, k + 1),
    }


def summarize(rows: list[dict]) -> dict:
    valid = [r for r in rows if r.get("planned")]
    if not valid:
        return {"planned": 0}
    keys = [
        "pos_err",
        "xy_err",
        "z_err",
        "vel_err",
        "final_alt",
        "pred_alt",
        "final_tilt",
        "max_tilt",
        "throttle_sat_frac",
        "gimbal_sat_frac",
    ]
    out = {"planned": len(valid)}
    for key in keys:
        vals = np.array([r[key] for r in valid], dtype=float)
        out[key] = float(vals.mean())
    return out


def print_summary(label: str, summary: dict, episodes: int) -> None:
    print(f"=== {label} (n={episodes}) ===")
    print(f"  planned     : {summary['planned']}/{episodes}")
    if summary["planned"] == 0:
        print()
        return
    print(f"  pos err     : {summary['pos_err']:.2f} m")
    print(f"  xy / z err  : {summary['xy_err']:.2f} m / {summary['z_err']:.2f} m")
    print(f"  vel err     : {summary['vel_err']:.2f} m/s")
    print(f"  alt actual  : {summary['final_alt']:.2f} m")
    print(f"  alt planned : {summary['pred_alt']:.2f} m")
    print(f"  tilt final  : {summary['final_tilt']:.1f} deg")
    print(f"  tilt max    : {summary['max_tilt']:.1f} deg")
    print(f"  throttle sat: {100 * summary['throttle_sat_frac']:.0f}%")
    print(f"  gimbal sat  : {100 * summary['gimbal_sat_frac']:.0f}%")
    print()


def write_markdown(path: Path, records: list[tuple[str, str, dict]], episodes: int) -> None:
    lines = [
        "# MPC Model Mismatch Diagnostic",
        "",
        "Date: 2026-05-28",
        "",
        "This diagnostic compares the convex point-mass MPC's planned trajectory against a no-disturbance 6-DOF rollout using the same rigid-body simulator as the landing environment.",
        "",
        f"Episodes per row: {episodes}",
        "",
        "| difficulty | plant model | planned | pos err | xy err | z err | vel err | actual alt | planned alt | final tilt | max tilt | throttle sat | gimbal sat |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for difficulty, plant_model, summary in records:
        if summary["planned"] == 0:
            lines.append(f"| {difficulty} | {plant_model} | 0/{episodes} | - | - | - | - | - | - | - | - | - | - |")
            continue
        lines.append(
            "| "
            f"{difficulty} | {plant_model} | {summary['planned']}/{episodes} | "
            f"{summary['pos_err']:.2f} m | "
            f"{summary['xy_err']:.2f} m | "
            f"{summary['z_err']:.2f} m | "
            f"{summary['vel_err']:.2f} m/s | "
            f"{summary['final_alt']:.2f} m | "
            f"{summary['pred_alt']:.2f} m | "
            f"{summary['final_tilt']:.1f} deg | "
            f"{summary['max_tilt']:.1f} deg | "
            f"{100 * summary['throttle_sat_frac']:.0f}% | "
            f"{100 * summary['gimbal_sat_frac']:.0f}% |"
        )
    lines.extend(
        [
            "",
            "Interpretation: large error here means the optimizer's point-mass plan is not dynamically trackable by the actual gimbaled rigid body, even before future wind is added.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--difficulties", default="moderate,hard,noisy")
    ap.add_argument("--plant-models", default="nominal,oracle")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--output", default=None)
    ap.add_argument(
        "--mpcs",
        default="pointmass",
        help=f"comma-separated MPC planners: {', '.join(PLANNERS)}",
    )
    args = ap.parse_args()

    records = []
    difficulties = [d.strip() for d in args.difficulties.split(",") if d.strip()]
    plant_models = [m.strip() for m in args.plant_models.split(",") if m.strip()]
    mpcs = [m.strip() for m in args.mpcs.split(",") if m.strip()]
    for mpc_name in mpcs:
        for difficulty in difficulties:
            for plant_model in plant_models:
                rows = [
                    run_episode(difficulty, seed, plant_model, mpc_name)
                    for seed in range(args.episodes)
                ]
                summary = summarize(rows)
                label = f"{mpc_name} / {difficulty} / {plant_model}"
                print_summary(label, summary, args.episodes)
                records.append((f"{mpc_name}|{difficulty}", plant_model, summary))

    if args.output:
        write_markdown(Path(args.output), records, args.episodes)
        print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
