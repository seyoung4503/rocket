"""Automated hyperparameter tuning for the landing controllers.

Uses Optuna's TPE sampler to search the controller's parameter space
against a multi-scenario success-rate objective.  The objective is
biased toward worst-case success so the search finds *robust*
configurations rather than ones that win on a single easy scenario.

Each trial:
  1. Build the controller with the trial's suggested parameters.
  2. Run ``n_episodes`` per scenario in parallel
     (ProcessPoolExecutor, matching evaluate_navigation.py).
  3. Aggregate to a scalar score (see ``objective_score`` below).

Trials are stored in a local SQLite database so a sweep can be
interrupted and resumed.

Usage examples
--------------
    # Tune the scp_warm MPC weights, 60 trials, 30 episodes/scenario.
    python scripts/tune_controller.py scp_warm \\
        --trials 60 --episodes 30 --workers 6

    # Resume an existing study (same name).
    python scripts/tune_controller.py scp_warm --trials 60

    # Quick smoke test with very small budget.
    python scripts/tune_controller.py scp_warm \\
        --trials 5 --episodes 10 --workers 3
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.navigation import LowPassStateEstimator  # noqa: E402
from rocketsim.navigation.estimator import LowPassEstimatorConfig  # noqa: E402


# --------------------------------------------------------------------------
# Tuning targets — registry of (controller name -> param space + builder).
# Adding a new target = adding one entry here.  The builder receives a
# ``trial`` and the (vehicle, env.world) tuple, and must return a callable
# controller compatible with ``evaluate_navigation``.
# --------------------------------------------------------------------------


@dataclass
class TuningTarget:
    name: str
    suggest: Callable[["optuna.trial.Trial"], dict]
    build: Callable[[dict, object, object], object]


def _build_scp_warm(params: dict, vehicle, world):
    from rocketsim.controllers import LandingScpWarm6DofWaypointPID
    from rocketsim.controllers.scp_6dof_mpc import CvxpyScpWarm6DofMPC

    ctrl = LandingScpWarm6DofWaypointPID(vehicle, world)
    # Rebuild the underlying MPC with the suggested weights.  All the
    # cost-weight knobs of CvxpyScpWarm6DofMPC are exposed here.
    ctrl.mpc = CvxpyScpWarm6DofMPC(vehicle, world, **params)
    return ctrl


def _suggest_scp_warm(trial: "optuna.trial.Trial") -> dict:
    # Logspace on cost weights so we cover orders of magnitude with the
    # same trial budget.
    return {
        "q_pos_xy": trial.suggest_float("q_pos_xy", 0.3, 5.0, log=True),
        "q_pos_z": trial.suggest_float("q_pos_z", 0.02, 1.0, log=True),
        "q_vel_xy": trial.suggest_float("q_vel_xy", 1.0, 20.0, log=True),
        "q_vel_z": trial.suggest_float("q_vel_z", 0.5, 15.0, log=True),
        "q_phi": trial.suggest_float("q_phi", 1.0, 50.0, log=True),
        "q_omega": trial.suggest_float("q_omega", 0.1, 5.0, log=True),
        "q_final_pos_xy": trial.suggest_float("q_final_pos_xy", 10.0, 200.0, log=True),
        "q_final_pos_z": trial.suggest_float("q_final_pos_z", 5.0, 80.0, log=True),
        "q_final_vel_xy": trial.suggest_float("q_final_vel_xy", 20.0, 300.0, log=True),
        "q_final_vel_z": trial.suggest_float("q_final_vel_z", 50.0, 500.0, log=True),
        "q_final_phi": trial.suggest_float("q_final_phi", 20.0, 400.0, log=True),
        "q_final_omega": trial.suggest_float("q_final_omega", 10.0, 200.0, log=True),
        "r_thrust": trial.suggest_float("r_thrust", 0.005, 0.2, log=True),
        "r_gimbal": trial.suggest_float("r_gimbal", 0.01, 0.3, log=True),
        "v_max_desc": trial.suggest_float("v_max_desc", 2.0, 6.0),
    }


def _build_actuator(params: dict, vehicle, world):
    from rocketsim.controllers import LandingActuatorAwareWaypointPID

    return LandingActuatorAwareWaypointPID(vehicle, world, **params)


def _suggest_actuator(trial: "optuna.trial.Trial") -> dict:
    return {
        "slew_factor": trial.suggest_float("slew_factor", 0.3, 1.4),
        "slack_weight": trial.suggest_float("slack_weight", 10.0, 200.0, log=True),
        "q_pos_xy": trial.suggest_float("q_pos_xy", 0.3, 5.0, log=True),
        "q_final_pos_xy": trial.suggest_float("q_final_pos_xy", 10.0, 200.0, log=True),
        "xy_ref_alpha": trial.suggest_float("xy_ref_alpha", 0.3, 1.0),
        "lookahead": trial.suggest_int("lookahead", 5, 15),
    }


TARGETS: dict[str, TuningTarget] = {
    "scp_warm": TuningTarget(
        name="scp_warm",
        suggest=_suggest_scp_warm,
        build=_build_scp_warm,
    ),
    "actuator": TuningTarget(
        name="actuator",
        suggest=_suggest_actuator,
        build=_build_actuator,
    ),
}


# --------------------------------------------------------------------------
# Episode runner.  Mirrors evaluate_navigation._run_one_episode so a worker
# process can rebuild env + controller without sharing state.
# --------------------------------------------------------------------------


def _run_one_episode(args):
    target_name, params, difficulty, seed = args
    target = TARGETS[target_name]
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    ctrl = target.build(params, env.base, env.world)
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
    return bool(info.get("success", False))


# --------------------------------------------------------------------------
# Objective: robust scalar score from per-scenario success rates.
# --------------------------------------------------------------------------


def objective_score(
    success_by_scenario: dict[str, float],
    worst_case_weight: float = 2.0,
) -> float:
    """Combine per-scenario rates into a single score.

    score = mean(success_rates) + worst_case_weight * min(success_rates)

    The worst-case term dominates as the trial improves the minimum,
    pulling Optuna toward configurations that are universally good
    rather than ones that ace one scenario.
    """
    rates = np.asarray(list(success_by_scenario.values()), dtype=float)
    return float(rates.mean() + worst_case_weight * rates.min())


# --------------------------------------------------------------------------
# Tuning loop.
# --------------------------------------------------------------------------


def run_study(
    target_name: str,
    n_trials: int,
    n_episodes: int,
    scenarios: list[str],
    n_workers: int,
    storage_path: Path,
):
    import optuna

    target = TARGETS[target_name]

    # Persistent storage so a sweep can be resumed.  Use a per-target
    # SQLite database so different targets don't collide.
    storage_path.parent.mkdir(parents=True, exist_ok=True)
    storage_url = f"sqlite:///{storage_path}"
    study = optuna.create_study(
        study_name=f"tune-{target_name}",
        direction="maximize",
        storage=storage_url,
        load_if_exists=True,
        sampler=optuna.samplers.TPESampler(seed=0, n_startup_trials=10),
    )

    print(
        f"Tuning {target_name}.  Storage: {storage_path}\n"
        f"Trials so far: {len(study.trials)}.  Budget: +{n_trials} trials.\n"
        f"Episodes per scenario: {n_episodes} ({len(scenarios)} scenarios)."
    )

    def objective(trial: "optuna.trial.Trial") -> float:
        params = target.suggest(trial)
        success_by_scenario: dict[str, float] = {}
        # Parallelize across (scenario, seed) pairs in one big pool.
        tasks = [
            (target_name, params, d, seed)
            for d in scenarios
            for seed in range(n_episodes)
        ]
        with ProcessPoolExecutor(max_workers=n_workers) as ex:
            results = list(ex.map(_run_one_episode, tasks, chunksize=1))
        for i, d in enumerate(scenarios):
            start = i * n_episodes
            end = start + n_episodes
            success_by_scenario[d] = float(np.mean(results[start:end]))
        # Report per-scenario rates so the dashboard can show them.
        for d, rate in success_by_scenario.items():
            trial.set_user_attr(f"rate_{d}", rate)
        return objective_score(success_by_scenario)

    study.optimize(objective, n_trials=n_trials, show_progress_bar=False)

    print("\n=== Best so far ===")
    best = study.best_trial
    print(f"  score = {best.value:.4f}")
    print("  rates per scenario:")
    for d in scenarios:
        rate = best.user_attrs.get(f"rate_{d}", float("nan"))
        print(f"    {d:12s}: {rate * 100:.1f}%")
    print("  params:")
    for k, v in best.params.items():
        if isinstance(v, float):
            print(f"    {k:18s} = {v:.6f}")
        else:
            print(f"    {k:18s} = {v}")
    return study


# --------------------------------------------------------------------------
# CLI.
# --------------------------------------------------------------------------


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("target", choices=sorted(TARGETS), help="controller to tune")
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--episodes", type=int, default=30, help="seeds per scenario")
    ap.add_argument(
        "--scenarios",
        default="hard,noisy,divert,divert_hard",
        help="comma-separated scenarios in the objective",
    )
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument(
        "--storage",
        default=None,
        help="sqlite path; defaults to .tuning/<target>.db",
    )
    args = ap.parse_args()

    scenarios = [s.strip() for s in args.scenarios.split(",") if s.strip()]
    storage = (
        Path(args.storage)
        if args.storage
        else Path(".tuning") / f"{args.target}.db"
    )
    run_study(
        args.target,
        args.trials,
        args.episodes,
        scenarios,
        args.workers,
        storage,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
