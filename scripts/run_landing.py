"""Demo: retro-thrust landing with the PID baseline on the landing scenario.

Runs one detailed episode (-> out/landing.csv) and then a small batch to report
the PID baseline success rate. This is the number RL will be compared against.

    python scripts/run_landing.py
"""

from __future__ import annotations

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import Environment, Simulator, edf_testbed  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import LandingPID  # noqa: E402
from rocketsim.scenarios import LandingScenario  # noqa: E402


def run_episode(seed: int, write_csv: bool = False):
    vehicle = edf_testbed()
    env = Environment()
    sim = Simulator(vehicle, env, dt=0.002)
    scenario = LandingScenario()

    rng = np.random.default_rng(seed)
    state0 = scenario.sample_initial_state(rng)
    controller = LandingPID(vehicle, env)

    traj = sim.run(
        controller,
        state0,
        duration=scenario.timeout,
        terminate=lambda t, s: scenario.check_done(s, t)[0],
    )
    result = scenario.evaluate(traj)

    if write_csv:
        t, states, cmds = traj.as_arrays()
        os.makedirs("out", exist_ok=True)
        with open("out/landing.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "tilt_deg", "throttle", "gx_deg", "gy_deg"])
            for i in range(len(t)):
                tilt = np.rad2deg(quat.tilt_angle(states[i, 6:10]))
                w.writerow(
                    [
                        f"{t[i]:.4f}",
                        *[f"{x:.4f}" for x in states[i, 0:3]],
                        *[f"{x:.4f}" for x in states[i, 3:6]],
                        f"{tilt:.3f}",
                        f"{cmds[i,0]:.3f}",
                        f"{np.rad2deg(cmds[i,1]):.3f}",
                        f"{np.rad2deg(cmds[i,2]):.3f}",
                    ]
                )
    return result


def main() -> int:
    r = run_episode(seed=0, write_csv=True)
    print("=== Retro-thrust landing (PID baseline), seed=0 ===")
    print(f"  outcome        : {'LANDED' if r.landed else ('CRASHED' if r.crashed else 'TIMEOUT')}")
    print(f"  touchdown time : {r.t:.2f} s")
    print(f"  offset         : {r.horizontal_offset*100:.1f} cm")
    print(f"  vertical speed : {r.vertical_speed:.2f} m/s  (limit {LandingScenario().max_touchdown_vspeed})")
    print(f"  lateral speed  : {r.horizontal_speed:.2f} m/s  (limit {LandingScenario().max_touchdown_hspeed})")
    print(f"  tilt           : {r.tilt_deg:.2f} deg")
    print(f"  energy (∫thr)  : {r.energy:.2f}")
    print(f"  SOFT LANDING   : {'✅' if r.success else '❌'}")
    print("  wrote out/landing.csv")

    # batch success rate
    n = 50
    results = [run_episode(seed=s) for s in range(n)]
    succ = sum(x.success for x in results)
    landed = sum(x.landed for x in results)
    vsp = np.mean([x.vertical_speed for x in results if x.landed]) if landed else float("nan")
    print(f"\n=== PID baseline over {n} randomized episodes ===")
    print(f"  reached ground : {landed}/{n}")
    print(f"  soft landings  : {succ}/{n}  ({100*succ/n:.0f}%)")
    print(f"  mean touchdown vspeed (landed): {vsp:.2f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
