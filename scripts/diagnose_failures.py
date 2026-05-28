"""Per-seed failure diagnosis for landing controllers.

The evaluation scripts report aggregate success rates. This script writes the
missing detail: which seeds failed, which touchdown criterion failed, how
controllers differ on the same episode seeds, and per-step trace CSVs for every
episode by default.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import (  # noqa: E402
    LandingCvxpyWaypointPID,
    LandingFeasibleWaypointMPC,
    LandingPID,
)
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402


CONTROLLERS = ("pid", "waypoint", "feasible")
MODES = ("true", "measured", "estimated")
TRACE_POLICIES = ("none", "failed", "all")
DEFAULT_TRACE_DIR = Path("out/failure_traces")


def make_controller(name: str, env, plant_model: str):
    vehicle = env.vehicle if plant_model == "oracle" else env.base
    if name == "pid":
        return LandingPID(vehicle, env.world)
    if name == "waypoint":
        return LandingCvxpyWaypointPID(vehicle, env.world)
    if name == "feasible":
        return LandingFeasibleWaypointMPC(vehicle, env.world)
    raise ValueError(f"unknown controller: {name}")


def touchdown_metrics(env) -> dict[str, float]:
    pos = env.state[dyn.POS]
    vel = env.state[dyn.VEL]
    return {
        "offset": float(np.linalg.norm(pos[:2])),
        "vspeed": float(max(0.0, -vel[2])),
        "hspeed": float(np.linalg.norm(vel[:2])),
        "tilt": float(np.rad2deg(quat.tilt_angle(env.state[dyn.QUAT]))),
        "alt": float(pos[2]),
    }


def fail_flags(env, metrics: dict[str, float], success: bool, reason: str) -> dict[str, int]:
    flags = {
        "fail_offset": 0,
        "fail_vspeed": 0,
        "fail_hspeed": 0,
        "fail_tilt": 0,
        "fail_timeout": int(reason == "timeout"),
        "fail_crash": int(reason in ("crash_tilt", "out_of_bounds", "too_high")),
    }
    if reason == "touchdown" and not success:
        flags["fail_offset"] = int(metrics["offset"] > env.scenario.max_touchdown_offset)
        flags["fail_vspeed"] = int(metrics["vspeed"] > env.scenario.max_touchdown_vspeed)
        flags["fail_hspeed"] = int(metrics["hspeed"] > env.scenario.max_touchdown_hspeed)
        flags["fail_tilt"] = int(metrics["tilt"] > env.scenario.max_touchdown_tilt_deg)
    return flags


def _add_state(row: dict, prefix: str, state: np.ndarray | None) -> None:
    if state is None:
        for key in (
            "x",
            "y",
            "z",
            "vx",
            "vy",
            "vz",
            "qw",
            "qx",
            "qy",
            "qz",
            "wx",
            "wy",
            "wz",
            "thrust",
            "offset",
            "hspeed",
            "vspeed",
            "tilt",
        ):
            row[f"{prefix}_{key}"] = np.nan
        return

    pos = state[dyn.POS]
    vel = state[dyn.VEL]
    q = state[dyn.QUAT]
    omega = state[dyn.OMEGA]
    row.update(
        {
            f"{prefix}_x": float(pos[0]),
            f"{prefix}_y": float(pos[1]),
            f"{prefix}_z": float(pos[2]),
            f"{prefix}_vx": float(vel[0]),
            f"{prefix}_vy": float(vel[1]),
            f"{prefix}_vz": float(vel[2]),
            f"{prefix}_qw": float(q[0]),
            f"{prefix}_qx": float(q[1]),
            f"{prefix}_qy": float(q[2]),
            f"{prefix}_qz": float(q[3]),
            f"{prefix}_wx": float(omega[0]),
            f"{prefix}_wy": float(omega[1]),
            f"{prefix}_wz": float(omega[2]),
            f"{prefix}_thrust": float(state[dyn.THRUST]),
            f"{prefix}_offset": float(np.linalg.norm(pos[:2])),
            f"{prefix}_hspeed": float(np.linalg.norm(vel[:2])),
            f"{prefix}_vspeed": float(max(0.0, -vel[2])),
            f"{prefix}_tilt": float(np.rad2deg(quat.tilt_angle(q))),
        }
    )


def controller_debug(ctrl, ctrl_state: np.ndarray) -> dict:
    out = {
        "target_x": np.nan,
        "target_y": np.nan,
        "target_z": np.nan,
        "guidance_mode": "",
        "guidance_readiness": np.nan,
        "guidance_z_setpoint": np.nan,
        "guidance_touchdown_ready": "",
        "mpc_xy_ref_x": np.nan,
        "mpc_xy_ref_y": np.nan,
        "mpc_accepted_xy_ref_x": np.nan,
        "mpc_accepted_xy_ref_y": np.nan,
        "last_plan_t": np.nan,
    }

    pid = getattr(ctrl, "pid", None)
    target = getattr(pid, "target", None)
    if target is not None:
        target = np.asarray(target, dtype=float)
        out["target_x"] = float(target[0])
        out["target_y"] = float(target[1])
        out["target_z"] = float(target[2])

    guidance = getattr(ctrl, "guidance", None)
    if guidance is not None:
        z_set = guidance.z_setpoint
        out["guidance_z_setpoint"] = np.nan if z_set is None else float(z_set)
        ready = guidance.readiness(ctrl_state, z_set)
        out["guidance_readiness"] = float(ready)
        out["guidance_mode"] = guidance.mode(ctrl_state, ready)
        out["guidance_touchdown_ready"] = str(guidance.touchdown_ready(ctrl_state))

    xy_ref = getattr(ctrl, "_xy_ref", None)
    if xy_ref is not None:
        xy_ref = np.asarray(xy_ref, dtype=float)
        out["mpc_xy_ref_x"] = float(xy_ref[0])
        out["mpc_xy_ref_y"] = float(xy_ref[1])

    accepted_xy_ref = getattr(ctrl, "_accepted_xy_ref", None)
    if accepted_xy_ref is not None:
        accepted_xy_ref = np.asarray(accepted_xy_ref, dtype=float)
        out["mpc_accepted_xy_ref_x"] = float(accepted_xy_ref[0])
        out["mpc_accepted_xy_ref_y"] = float(accepted_xy_ref[1])

    last_plan_t = getattr(ctrl, "_last_plan_t", None)
    if last_plan_t is not None:
        out["last_plan_t"] = float(last_plan_t)
    return out


def write_trace_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def trace_filename(
    trace_dir: Path,
    difficulty: str,
    controller_name: str,
    mode: str,
    plant_model: str,
    seed: int,
    success: bool,
) -> Path:
    status = "success" if success else "fail"
    name = f"{difficulty}_{controller_name}_{mode}_{plant_model}_seed{seed:04d}_{status}.csv"
    return trace_dir / name


def run_episode(
    difficulty: str,
    controller_name: str,
    mode: str,
    plant_model: str,
    seed: int,
    trace_dir: Path | None = DEFAULT_TRACE_DIR,
    trace_policy: str = "all",
    trace_seeds: set[int] | None = None,
) -> dict:
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = make_controller(controller_name, env, plant_model)
    estimator = LowPassStateEstimator()
    est = estimator.reset(env.measured)

    done = False
    energy = 0.0
    max_tilt = 0.0
    max_hspeed = 0.0
    min_alt = float(env.state[dyn.POS][2])
    steps = 0
    info = {}
    trace_rows: list[dict] = []
    trace_seeds = trace_seeds or set()
    collect_trace = trace_dir is not None and (trace_policy in ("failed", "all") or seed in trace_seeds)

    while not done:
        t0 = float(env.t)
        if mode == "true":
            ctrl_state = env.state
        elif mode == "measured":
            ctrl_state = env.measured
        elif mode == "estimated":
            ctrl_state = est
        else:
            raise ValueError(f"unknown mode: {mode}")

        cmd = ctrl(env.t, ctrl_state)
        debug = controller_debug(ctrl, ctrl_state) if collect_trace else {}
        energy += cmd.throttle * env.control_dt
        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        if mode == "estimated":
            est = estimator.update(env.measured, env.control_dt)

        metrics_now = touchdown_metrics(env)
        max_tilt = max(max_tilt, metrics_now["tilt"])
        max_hspeed = max(max_hspeed, metrics_now["hspeed"])
        min_alt = min(min_alt, metrics_now["alt"])
        steps += 1

        if collect_trace:
            row = {
                "difficulty": difficulty,
                "controller": controller_name,
                "mode": mode,
                "plant_model": plant_model,
                "seed": seed,
                "step": steps,
                "t0": t0,
                "t1": float(env.t),
                "cmd_throttle": float(cmd.throttle),
                "cmd_gimbal_x": float(cmd.gimbal_x),
                "cmd_gimbal_y": float(cmd.gimbal_y),
                "applied_gimbal_x": float(env._prev_gimbal[0]),
                "applied_gimbal_y": float(env._prev_gimbal[1]),
                "reason": str(info.get("reason", "")),
                "success": int(bool(info.get("success", False))),
                **debug,
            }
            _add_state(row, "true", env.state)
            _add_state(row, "measured", env.measured)
            _add_state(row, "estimated", est if mode == "estimated" else None)
            _add_state(row, "controller_input", ctrl_state)
            trace_rows.append(row)
        done = term or trunc

    reason = str(info.get("reason", ""))
    success = bool(info.get("success", False))
    metrics = touchdown_metrics(env)
    flags = fail_flags(env, metrics, success, reason)
    if trace_dir is not None and trace_rows:
        should_write = (
            trace_policy == "all"
            or (trace_policy == "failed" and not success)
            or seed in trace_seeds
        )
        if should_write:
            write_trace_csv(
                trace_filename(trace_dir, difficulty, controller_name, mode, plant_model, seed, success),
                trace_rows,
            )
    return {
        "difficulty": difficulty,
        "controller": controller_name,
        "mode": mode,
        "plant_model": plant_model,
        "seed": seed,
        "success": int(success),
        "reason": reason,
        "t": float(env.t),
        "steps": steps,
        "energy": float(energy),
        "offset": metrics["offset"],
        "vspeed": metrics["vspeed"],
        "hspeed": metrics["hspeed"],
        "tilt": metrics["tilt"],
        "max_tilt": max_tilt,
        "max_hspeed": max_hspeed,
        "min_alt": min_alt,
        **flags,
    }


def primary_failure(row: dict) -> str:
    if row["success"]:
        return "success"
    if row["reason"] != "touchdown":
        return row["reason"]
    failures = []
    for key, label in (
        ("fail_offset", "offset"),
        ("fail_vspeed", "vspeed"),
        ("fail_hspeed", "hspeed"),
        ("fail_tilt", "tilt"),
    ):
        if row[key]:
            failures.append(label)
    return "+".join(failures) if failures else "touchdown_not_soft"


def summarize(rows: list[dict]) -> list[str]:
    lines: list[str] = []
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for row in rows:
        groups[(row["difficulty"], row["controller"])].append(row)

    lines.append("| difficulty | controller | success | reasons | primary failures | touchdown mean |")
    lines.append("|---|---|---:|---|---|---|")
    for (difficulty, controller), group in sorted(groups.items()):
        n = len(group)
        succ = sum(int(r["success"]) for r in group)
        reasons = Counter(r["reason"] for r in group)
        failures = Counter(primary_failure(r) for r in group if not r["success"])
        touched = [r for r in group if r["reason"] == "touchdown"]
        if touched:
            offset = np.mean([r["offset"] for r in touched])
            vspeed = np.mean([r["vspeed"] for r in touched])
            hspeed = np.mean([r["hspeed"] for r in touched])
            tilt = np.mean([r["tilt"] for r in touched])
            touchdown = f"offset {offset:.2f}, vspeed {vspeed:.2f}, hspeed {hspeed:.2f}, tilt {tilt:.1f}"
        else:
            touchdown = "-"
        lines.append(
            "| "
            f"{difficulty} | {controller} | {succ}/{n} ({100 * succ / n:.0f}%) | "
            f"{dict(reasons)} | {dict(failures)} | {touchdown} |"
        )
    return lines


def seed_comparison(rows: list[dict], controllers: list[str]) -> list[str]:
    by_seed: dict[tuple[str, int], dict[str, dict]] = defaultdict(dict)
    for row in rows:
        by_seed[(row["difficulty"], int(row["seed"]))][row["controller"]] = row

    lines = [
        "| difficulty | seed | outcome pattern | details |",
        "|---|---:|---|---|",
    ]
    interesting = []
    for (difficulty, seed), result_by_controller in sorted(by_seed.items()):
        if not all(c in result_by_controller for c in controllers):
            continue
        outcomes = {c: bool(result_by_controller[c]["success"]) for c in controllers}
        if len(set(outcomes.values())) == 1:
            continue
        pattern = ", ".join(f"{c}:{'ok' if outcomes[c] else primary_failure(result_by_controller[c])}" for c in controllers)
        details = ", ".join(
            f"{c}(off={result_by_controller[c]['offset']:.2f}, "
            f"hs={result_by_controller[c]['hspeed']:.2f}, "
            f"tilt={result_by_controller[c]['tilt']:.1f})"
            for c in controllers
        )
        interesting.append((difficulty, seed, pattern, details))

    for difficulty, seed, pattern, details in interesting[:40]:
        lines.append(f"| {difficulty} | {seed} | {pattern} | {details} |")
    if len(interesting) > 40:
        lines.append(f"| ... | ... | {len(interesting) - 40} more differing seeds omitted | ... |")
    return lines


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(path: Path, rows: list[dict], controllers: list[str]) -> None:
    lines = [
        "# Failure Diagnosis",
        "",
        "Date: 2026-05-28",
        "",
        "This report logs per-seed landing outcomes. It is meant to answer where each controller fails, not only the aggregate success rate.",
        "",
        "## Detailed Trace Logs",
        "",
        f"Per-step trajectory CSV logs are saved by default under `{DEFAULT_TRACE_DIR}`.",
        "",
        "One file is written per difficulty/controller/mode/plant-model/seed, with the final success/fail status in the filename. Each row includes commanded throttle/gimbal, applied gimbal, guidance/MPC debug fields, and true/measured/estimated/controller-input state snapshots.",
        "",
        "## Summary",
        "",
        *summarize(rows),
        "",
        "## Seeds Where Controllers Disagree",
        "",
        *seed_comparison(rows, controllers),
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--difficulties", default="hard")
    ap.add_argument("--controllers", default="pid,waypoint,feasible")
    ap.add_argument("--mode", default="estimated", choices=MODES)
    ap.add_argument("--plant-model", default="nominal", choices=["nominal", "oracle"])
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--csv", default="out/failure_diagnosis.csv")
    ap.add_argument("--markdown", default="docs/failure_diagnosis.md")
    ap.add_argument("--trace-dir", default=str(DEFAULT_TRACE_DIR), help="directory for per-step trajectory CSV logs")
    ap.add_argument(
        "--trace",
        default="all",
        choices=TRACE_POLICIES,
        help="write per-step logs: none, failed, or all; default writes every episode",
    )
    ap.add_argument(
        "--trace-seeds",
        default="",
        help="comma-separated seed ids to trace regardless of success/failure",
    )
    args = ap.parse_args()

    difficulties = [d.strip() for d in args.difficulties.split(",") if d.strip()]
    controllers = [c.strip() for c in args.controllers.split(",") if c.strip()]
    unknown = sorted(set(controllers) - set(CONTROLLERS))
    if unknown:
        raise ValueError(f"unknown controllers: {unknown}")
    trace_seeds = {int(s.strip()) for s in args.trace_seeds.split(",") if s.strip()}
    trace_dir = Path(args.trace_dir) if args.trace_dir else None

    rows = []
    for difficulty in difficulties:
        for controller in controllers:
            for seed in range(args.episodes):
                rows.append(
                    run_episode(
                        difficulty,
                        controller,
                        args.mode,
                        args.plant_model,
                        seed,
                        trace_dir=trace_dir,
                        trace_policy=args.trace,
                        trace_seeds=trace_seeds,
                    )
                )

    write_csv(Path(args.csv), rows)
    write_markdown(Path(args.markdown), rows, controllers)
    print(f"Wrote {args.csv}")
    print(f"Wrote {args.markdown}")
    if trace_dir is not None:
        print(f"Wrote detailed traces under {trace_dir}")
    for line in summarize(rows):
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
