"""Compare controllers on true, raw measured and estimated state.

This isolates the next GNC layer: Navigation. Physics and success criteria still
use the true state, but controllers can be fed:

  * true      -- ideal, not hardware-realistic
  * measured  -- raw noisy measurement
  * estimated -- low-pass/complementary estimated state

Examples:
    python scripts/evaluate_navigation.py --difficulties noisy --episodes 100
    python scripts/evaluate_navigation.py --difficulties hard,noisy --controllers pid,waypoint
    python scripts/evaluate_navigation.py --output docs/navigation_experiments.md
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
    LandingCvxpyGuidancePID,
    LandingCvxpyMPC,
    LandingCvxpyWaypointPID,
    LandingFeasibleWaypointMPC,
    LandingFullDynamicsMPC,
    LandingPID,
    LandingVerticalMPC,
)
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402
from rocketsim.navigation.estimator import LowPassEstimatorConfig  # noqa: E402


CONTROLLERS = (
    "pid", "vertical", "cvxpy", "guidance", "waypoint", "feasible", "full",
    "actuator",  # Step 1 actuator-aware MPC + PID waypoint (2026-05-29 v1)
    # A/B sweep on horizontal-tracking aggressiveness in the actuator-aware MPC
    # (q_pos[xy] / q_final_pos[xy]). Default is 1.2 / 45 which causes ~96%
    # gimbal saturation; A and B relax those weights 4x / 8x respectively.
    "actuator_a",
    "actuator_b",
    # Step 2: Step 1 + thrust-magnitude 1st-order lag matching the EDF
    # spool-up tau (vehicle.thrust_time_constant). Still point-mass.
    "actuator2",
    # SpaceX-style time-indexed trajectory tracking variants — drop the
    # single-point lookahead in favor of plan-time-interpolated
    # (p_ref, v_ref, u_ff). See
    # docs/2026-05-29_1653_v1_lookahead_vs_spacex_design.md.
    "tracking_pointmass",
    "tracking_actuator",
    "tracking_actuator2",
    # Full SpaceX-style: bare tracker + position integrator + landing gate
    # + touchdown commit. See trajectory_tracker.LandingTrajectoryTrackingFullMPC.
    "full_pointmass",
    "full_actuator",
    "full_actuator2",
    # Proper SpaceX-style stack built from scratch in src/rocketsim/spacex:
    # min-fuel convex MPC + glideslope + shrinking horizon + time-indexed
    # PD+I tracker. See docs/2026-05-29_1753_v1_spacex_style_design.md.
    "spacex",
    # SpaceX stack with Step 1 (slew) / Step 1+2 (slew + thrust-mag lag)
    # actuator-aware constraints ported into the convex landing MPC.
    "spacex_actuator",
    "spacex_actuator2",
)
MODES = ("true", "measured", "estimated")


def make_controller(name: str, env, plant_model: str):
    vehicle = env.vehicle if plant_model == "oracle" else env.base
    if name == "pid":
        return LandingPID(vehicle, env.world)
    if name == "vertical":
        return LandingVerticalMPC(vehicle, env.world)
    if name == "cvxpy":
        return LandingCvxpyMPC(vehicle, env.world)
    if name == "guidance":
        return LandingCvxpyGuidancePID(vehicle, env.world)
    if name == "waypoint":
        return LandingCvxpyWaypointPID(vehicle, env.world)
    if name == "feasible":
        return LandingFeasibleWaypointMPC(vehicle, env.world)
    if name == "full":
        return LandingFullDynamicsMPC(vehicle, env.world)
    if name == "actuator":
        from rocketsim.controllers import LandingActuatorAwareWaypointPID
        return LandingActuatorAwareWaypointPID(vehicle, env.world)
    if name == "actuator_a":
        from rocketsim.controllers import LandingActuatorAwareWaypointPID
        return LandingActuatorAwareWaypointPID(
            vehicle, env.world, q_pos_xy=0.3, q_final_pos_xy=15.0
        )
    if name == "actuator_b":
        from rocketsim.controllers import LandingActuatorAwareWaypointPID
        return LandingActuatorAwareWaypointPID(
            vehicle, env.world, q_pos_xy=0.15, q_final_pos_xy=8.0
        )
    if name == "actuator2":
        from rocketsim.controllers import LandingActuatorAwareMagLagWaypointPID
        return LandingActuatorAwareMagLagWaypointPID(vehicle, env.world)
    if name == "tracking_pointmass":
        from rocketsim.controllers import LandingPointMassTrackingMPC
        return LandingPointMassTrackingMPC(vehicle, env.world)
    if name == "tracking_actuator":
        from rocketsim.controllers import LandingActuatorTrackingMPC
        return LandingActuatorTrackingMPC(vehicle, env.world)
    if name == "tracking_actuator2":
        from rocketsim.controllers import LandingActuatorMagLagTrackingMPC
        return LandingActuatorMagLagTrackingMPC(vehicle, env.world)
    if name == "full_pointmass":
        from rocketsim.controllers import LandingPointMassTrackingFullMPC
        return LandingPointMassTrackingFullMPC(vehicle, env.world)
    if name == "full_actuator":
        from rocketsim.controllers import LandingActuatorTrackingFullMPC
        return LandingActuatorTrackingFullMPC(vehicle, env.world)
    if name == "full_actuator2":
        from rocketsim.controllers import LandingActuatorMagLagTrackingFullMPC
        return LandingActuatorMagLagTrackingFullMPC(vehicle, env.world)
    if name == "spacex":
        from rocketsim.spacex import LandingControllerSpaceX
        return LandingControllerSpaceX(vehicle, env.world)
    if name == "spacex_actuator":
        from rocketsim.spacex import (
            ActuatorAwareLandingMPC,
            LandingControllerSpaceX,
        )
        return LandingControllerSpaceX(
            vehicle, env.world, planner_cls=ActuatorAwareLandingMPC
        )
    if name == "spacex_actuator2":
        from rocketsim.spacex import (
            ActuatorMagLagLandingMPC,
            LandingControllerSpaceX,
        )
        return LandingControllerSpaceX(
            vehicle, env.world, planner_cls=ActuatorMagLagLandingMPC
        )
    raise ValueError(f"unknown controller: {name}")


def summarize_touchdown(env):
    pos = env.state[dyn.POS]
    vel = env.state[dyn.VEL]
    return {
        "offset": float(np.linalg.norm(pos[:2])),
        "vspeed": float(max(0.0, -vel[2])),
        "hspeed": float(np.linalg.norm(vel[:2])),
        "tilt": float(np.rad2deg(quat.tilt_angle(env.state[dyn.QUAT]))),
    }


def _run_one_episode(args):
    """Top-level worker function for ProcessPoolExecutor (must be picklable).

    Each call constructs a FRESH env/controller/estimator inside the worker
    process, runs one episode, and returns a small dict. All work is local to
    the worker, so episodes can run in true parallel across CPU cores.
    """
    ep, difficulty, controller_name, mode, plant_model = args
    env = make_landing_env(difficulty)
    env.reset(seed=ep)
    ctrl = make_controller(controller_name, env, plant_model)
    estimator = LowPassStateEstimator(
        LowPassEstimatorConfig.for_obs_noise(getattr(env, "obs_noise", 0.0))
    )
    est = estimator.reset(env.measured)

    done = False
    ep_energy = 0.0
    while not done:
        if mode == "true":
            ctrl_state = env.state
        elif mode == "measured":
            ctrl_state = env.measured
        elif mode == "estimated":
            ctrl_state = est
        else:
            raise ValueError(f"unknown mode: {mode}")
        cmd = ctrl(env.t, ctrl_state)
        ep_energy += cmd.throttle * env.control_dt
        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        if mode == "estimated":
            est = estimator.update(env.measured, env.control_dt)
        done = term or trunc

    reason = info.get("reason", "")
    out = {
        "ep": ep,
        "reason": reason,
        "success": bool(info.get("success", False)),
        "energy": ep_energy,
        "touchdown": summarize_touchdown(env) if reason == "touchdown" else None,
    }
    return out


def _aggregate_results(results, env_for_thresholds):
    """Combine per-episode dicts into the shape eval_mode used to return."""
    succ = 0
    landed = 0
    reasons: dict[str, int] = {}
    fail = {"offset": 0, "vspeed": 0, "hspeed": 0, "tilt": 0}
    touchdown = []
    energy = []
    sc = env_for_thresholds.scenario
    for r in sorted(results, key=lambda d: d["ep"]):
        reason = r["reason"]
        reasons[reason] = reasons.get(reason, 0) + 1
        succ += int(r["success"])
        energy.append(r["energy"])
        if reason == "touchdown" and r["touchdown"] is not None:
            landed += 1
            metrics = r["touchdown"]
            touchdown.append(metrics)
            if not r["success"]:
                if metrics["offset"] > sc.max_touchdown_offset:
                    fail["offset"] += 1
                if metrics["vspeed"] > sc.max_touchdown_vspeed:
                    fail["vspeed"] += 1
                if metrics["hspeed"] > sc.max_touchdown_hspeed:
                    fail["hspeed"] += 1
                if metrics["tilt"] > sc.max_touchdown_tilt_deg:
                    fail["tilt"] += 1
    return {
        "success": succ,
        "landed": landed,
        "reasons": reasons,
        "fail": fail,
        "touchdown": touchdown,
        "energy": energy,
    }


def eval_mode(
    difficulty: str,
    controller_name: str,
    mode: str,
    n: int,
    plant_model: str,
    n_workers: int = 1,
):
    if n_workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        args_list = [
            (ep, difficulty, controller_name, mode, plant_model) for ep in range(n)
        ]
        with ProcessPoolExecutor(max_workers=n_workers) as ex:
            results = list(ex.map(_run_one_episode, args_list, chunksize=1))
        env_ref = make_landing_env(difficulty)
        env_ref.reset(seed=0)
        return _aggregate_results(results, env_ref)

    env = make_landing_env(difficulty)
    succ = 0
    landed = 0
    reasons: dict[str, int] = {}
    fail = {"offset": 0, "vspeed": 0, "hspeed": 0, "tilt": 0}
    touchdown = []
    energy = []

    for ep in range(n):
        env.reset(seed=ep)
        ctrl = make_controller(controller_name, env, plant_model)
        # Adaptive estimator: filter strength scales with input noise so
        # clean inputs (hard, obs_noise=0) aren't unnecessarily lagged.
        estimator = LowPassStateEstimator(
            LowPassEstimatorConfig.for_obs_noise(getattr(env, "obs_noise", 0.0))
        )
        est = estimator.reset(env.measured)

        done = False
        ep_energy = 0.0
        while not done:
            if mode == "true":
                ctrl_state = env.state
            elif mode == "measured":
                ctrl_state = env.measured
            elif mode == "estimated":
                ctrl_state = est
            else:
                raise ValueError(f"unknown mode: {mode}")

            cmd = ctrl(env.t, ctrl_state)
            ep_energy += cmd.throttle * env.control_dt
            _, _, term, trunc, info = env.step(env.command_to_action(cmd))
            if mode == "estimated":
                est = estimator.update(env.measured, env.control_dt)
            done = term or trunc

        reason = info.get("reason", "")
        reasons[reason] = reasons.get(reason, 0) + 1
        succ += int(info.get("success", False))
        energy.append(ep_energy)
        if reason == "touchdown":
            landed += 1
            metrics = summarize_touchdown(env)
            touchdown.append(metrics)
            if not info.get("success", False):
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


def touchdown_mean(result) -> tuple[float, float, float, float] | None:
    if not result["touchdown"]:
        return None
    arr = np.array(
        [
            [m["offset"], m["vspeed"], m["hspeed"], m["tilt"]]
            for m in result["touchdown"]
        ],
        dtype=float,
    )
    return (
        float(arr[:, 0].mean()),
        float(arr[:, 1].mean()),
        float(arr[:, 2].mean()),
        float(arr[:, 3].mean()),
    )


def print_result(label: str, result, n: int) -> None:
    print(f"=== {label} (n={n}) ===")
    print(f"  soft landings : {result['success']}/{n} ({100 * result['success'] / n:.0f}%)")
    print(f"  reached ground: {result['landed']}/{n}")
    print(f"  reasons       : {result['reasons']}")
    print(f"  throttle int  : mean {np.mean(result['energy']):.2f}")
    mean = touchdown_mean(result)
    if mean is not None:
        offset, vspeed, hspeed, tilt = mean
        print(
            "  touchdown mean: "
            f"offset {offset:.2f} m, "
            f"vspeed {vspeed:.2f} m/s, "
            f"hspeed {hspeed:.2f} m/s, "
            f"tilt {tilt:.1f} deg"
        )
    if result["success"] < result["landed"]:
        print(f"  landed fail   : {result['fail']}")
    print()


def format_reasons(reasons: dict[str, int]) -> str:
    if not reasons:
        return ""
    return ", ".join(f"{k}:{v}" for k, v in sorted(reasons.items()))


def append_markdown_table(lines: list[str], records: list[dict]) -> None:
    lines.append("| difficulty | controller | plant model | input | success | landed | throttle int | touchdown mean | reasons | landed fail |")
    lines.append("|---|---|---|---:|---:|---:|---:|---|---|---|")
    for rec in records:
        result = rec["result"]
        n = rec["episodes"]
        mean = touchdown_mean(result)
        if mean is None:
            touchdown = "-"
        else:
            offset, vspeed, hspeed, tilt = mean
            touchdown = (
                f"offset {offset:.2f} m, "
                f"vspeed {vspeed:.2f} m/s, "
                f"hspeed {hspeed:.2f} m/s, "
                f"tilt {tilt:.1f} deg"
            )
        fail = "-"
        if result["success"] < result["landed"]:
            fail = format_reasons({k: v for k, v in result["fail"].items() if v})
        lines.append(
            "| "
            f"{rec['difficulty']} | "
            f"{rec['controller']} | "
            f"{rec['plant_model']} | "
            f"{rec['mode']} | "
            f"{result['success']}/{n} ({100 * result['success'] / n:.0f}%) | "
            f"{result['landed']}/{n} | "
            f"{np.mean(result['energy']):.2f} | "
            f"{touchdown} | "
            f"{format_reasons(result['reasons'])} | "
            f"{fail} |"
        )


def write_markdown(path: Path, records: list[dict], episodes: int) -> None:
    lines = [
        "# Navigation / GNC Controller Experiments",
        "",
        "Date: 2026-05-28",
        "",
        "Purpose: compare the same landing controllers when they consume ideal true state, raw noisy measured state, or estimated state. Physics and touchdown scoring always use the true simulated vehicle state.",
        "",
        f"Episodes per row: {episodes}",
        "",
        "Controller labels:",
        "",
        "- `pid`: cascaded LandingPID baseline.",
        "- `vertical`: PID attitude/XY with sampled 1D vertical MPC throttle.",
        "- `cvxpy`: direct 3D point-mass convex MPC thrust-vector guidance plus TVC attitude tracking.",
        "- `guidance`: convex MPC waypoint/feed-forward acceleration tracked by a feedback layer.",
        "- `waypoint`: convex MPC XY waypoint guidance with the existing PID landing gate.",
        "- `feasible`: convex MPC XY waypoint guidance projected into a simple attitude/actuator feasibility envelope before PID tracking.",
        "- `full`: sampled nonlinear MPC that rolls candidates through the same 6-DOF rigid-body dynamics used by the simulator.",
        "",
        "Input labels:",
        "",
        "- `true`: non-hardware upper bound; controller receives exact simulator state.",
        "- `measured`: raw sensor-like measurement.",
        "- `estimated`: controller receives the navigation estimator output.",
        "",
        "Plant model labels:",
        "",
        "- `nominal`: controller uses the nominal EDF model while the episode may randomize the actual vehicle.",
        "- `oracle`: controller is given the randomized vehicle model for that episode. This is not hardware-realistic by itself, but isolates whether failures come from model mismatch versus controller structure.",
        "",
    ]
    append_markdown_table(lines, records)
    lines.extend(
        [
            "",
            "## Readout",
            "",
            "- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.",
            "- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.",
            "- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--difficulty",
        default="noisy",
        choices=[
            "calm", "moderate", "hard", "unknown", "recovery", "noisy",
            "divert", "divert_hard",
        ],
        help="single difficulty; kept for compatibility",
    )
    ap.add_argument(
        "--difficulties",
        default=None,
        help="comma-separated difficulties; overrides --difficulty",
    )
    ap.add_argument("--controller", default="pid", choices=CONTROLLERS)
    ap.add_argument(
        "--controllers",
        default=None,
        help=f"comma-separated controllers: {', '.join(CONTROLLERS)}; overrides --controller",
    )
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument(
        "--plant-model",
        default="nominal",
        choices=["nominal", "oracle"],
        help="controller model: nominal base vehicle or oracle episode vehicle",
    )
    ap.add_argument(
        "--modes",
        default="true,measured,estimated",
        help="comma-separated: true, measured, estimated",
    )
    ap.add_argument("--output", default=None, help="optional markdown output path")
    ap.add_argument(
        "--workers",
        type=int,
        default=1,
        help="number of parallel worker processes (1 = sequential)",
    )
    args = ap.parse_args()

    difficulties = (
        [d.strip() for d in args.difficulties.split(",") if d.strip()]
        if args.difficulties
        else [args.difficulty]
    )
    controllers = (
        [c.strip() for c in args.controllers.split(",") if c.strip()]
        if args.controllers
        else [args.controller]
    )
    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    unknown_controllers = sorted(set(controllers) - set(CONTROLLERS))
    unknown_modes = sorted(set(modes) - set(MODES))
    if unknown_controllers:
        raise ValueError(f"unknown controllers: {unknown_controllers}")
    if unknown_modes:
        raise ValueError(f"unknown modes: {unknown_modes}")

    records = []
    for difficulty in difficulties:
        for controller in controllers:
            print(
                f"Navigation evaluation @ {difficulty}, "
                f"controller={controller}, plant_model={args.plant_model}\n"
            )
            for mode in modes:
                result = eval_mode(
                    difficulty,
                    controller,
                    mode,
                    args.episodes,
                    args.plant_model,
                    n_workers=args.workers,
                )
                print_result(mode, result, args.episodes)
                records.append(
                    {
                        "difficulty": difficulty,
                        "controller": controller,
                        "plant_model": args.plant_model,
                        "mode": mode,
                        "episodes": args.episodes,
                        "result": result,
                    }
                )

    if args.output:
        write_markdown(Path(args.output), records, args.episodes)
        print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
