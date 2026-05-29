"""Diagnose the MPC hover-and-timeout bug on `divert` (mild).

Hypothesis from `docs/2026-05-29_1550_v1_divert_baseline_analysis.md`:

    MPC's waypoint at `lookahead=4` steps (0.8s ahead) is too close to the
    current vehicle position, so the inner PID gets only a small xy_target
    nudge. The vehicle moves laterally too slowly to satisfy the guidance
    layer's readiness gate; descent stays at the floor (~0.35*v_min), so
    the rocket never gets below `gate_alt=2.5m` before the episode times
    out.

This script logs, for ONE seed of `divert` running `actuator` (Step 1 MPC),
the per-step time series of:

  * vehicle z, xy_offset (from current pad)
  * MPC's xy waypoint at k=lookahead vs MPC's xy at k=horizon-end
  * guidance's z setpoint, readiness, descent rate floor
  * PID's commanded throttle and tilt

We print a compact table and report whether the descent rate stalls at the
floor while xy_offset stays large — that would confirm the hypothesis.
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import dynamics as dyn  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.envs import make_landing_env  # noqa: E402
from rocketsim.controllers import (  # noqa: E402
    LandingActuatorAwareWaypointPID,
    LandingPID,
)


def run_logged(difficulty: str, seed: int, controller_name: str):
    env = make_landing_env(difficulty)
    env.reset(seed=seed)
    if controller_name == "actuator":
        ctrl = LandingActuatorAwareWaypointPID(env.base, env.world)
    elif controller_name == "pid":
        ctrl = LandingPID(env.base, env.world)
    else:
        raise ValueError(controller_name)

    rows = []
    done = False
    while not done:
        # Sample BEFORE controller call (so we capture inputs)
        z = float(env.state[dyn.POS][2])
        xy_off = float(np.linalg.norm(env.state[dyn.POS][:2]))
        vz = float(env.state[dyn.VEL][2])

        cmd = ctrl(env.t, env.state)

        # Inspect controller internal state if available
        if hasattr(ctrl, "mpc") and hasattr(ctrl.mpc, "p"):
            p_plan = (
                ctrl.mpc.p.value if ctrl.mpc.p.value is not None else None
            )
        else:
            p_plan = None

        if hasattr(ctrl, "guidance"):
            g = ctrl.guidance
            zset = float(getattr(g, "_zset", np.nan))
            readiness = float(g.readiness(env.state, getattr(g, "_zset", z)))
            mode = g.mode(env.state, getattr(g, "_zset", z))
        else:
            zset = np.nan
            readiness = np.nan
            mode = "n/a"

        if hasattr(ctrl, "_xy_ref"):
            xy_ref = ctrl._xy_ref.copy()
        else:
            xy_ref = np.array([np.nan, np.nan])

        # Step the env
        _, _, term, trunc, info = env.step(env.command_to_action(cmd))
        done = term or trunc

        # Stash row
        plan_xy_horizon_end = (
            p_plan[:2, -1] if p_plan is not None else np.array([np.nan, np.nan])
        )
        plan_xy_lookahead = (
            p_plan[:2, min(4, p_plan.shape[1] - 1)] if p_plan is not None
            else np.array([np.nan, np.nan])
        )
        rows.append({
            "t": env.t,
            "z": z,
            "xy_off": xy_off,
            "vz": vz,
            "zset": zset,
            "readiness": readiness,
            "mode": mode,
            "xy_ref_norm": float(np.linalg.norm(xy_ref)),
            "plan_la_norm": float(np.linalg.norm(plan_xy_lookahead)),
            "plan_end_norm": float(np.linalg.norm(plan_xy_horizon_end)),
            "thr": cmd.throttle,
            "tilt_deg": float(np.rad2deg(quat.tilt_angle(env.state[dyn.QUAT]))),
        })

    print(f"reason={info.get('reason')}  success={info.get('success')}  steps={len(rows)}")
    print(
        f"{'t':>5} {'z':>6} {'xy':>6} {'vz':>6} {'zset':>6} {'rd':>5} "
        f"{'mode':>8} {'xyref':>6} {'plk':>6} {'pend':>6} {'thr':>5} {'tilt':>5}"
    )
    # Sample every Nth row to keep output readable
    sample = max(1, len(rows) // 30)
    for i, r in enumerate(rows):
        if i % sample == 0 or r["mode"] == "commit" or i == len(rows) - 1:
            print(
                f"{r['t']:5.2f} {r['z']:6.2f} {r['xy_off']:6.2f} {r['vz']:6.2f} "
                f"{r['zset']:6.2f} {r['readiness']:5.2f} {r['mode']:>8} "
                f"{r['xy_ref_norm']:6.2f} {r['plan_la_norm']:6.2f} "
                f"{r['plan_end_norm']:6.2f} {r['thr']:5.2f} {r['tilt_deg']:5.1f}"
            )


def main():
    # Pick a divert seed where actuator known to timeout (we saw 40-42 timeouts
    # out of 50; seed 0 is a reasonable representative).
    print("\n========== mild divert, controller=actuator, seed=0 ==========")
    run_logged("divert", 0, "actuator")
    print("\n========== mild divert, controller=pid, seed=0 ==========")
    run_logged("divert", 0, "pid")


if __name__ == "__main__":
    main()
