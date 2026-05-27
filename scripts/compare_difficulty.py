"""PID landing under two regimes: calm (known plant) vs hard (RL-benchmark grade).

The controller is always given the NOMINAL vehicle model, while the simulator
runs a per-episode RANDOMIZED plant (unknown mass/thrust, thrust misalignment)
under gusty wind and random force loads. This model mismatch + unmeasured
disturbance is the regime where the PID baseline is actually tested.

    python scripts/compare_difficulty.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import Environment, Simulator, edf_testbed  # noqa: E402
from rocketsim.controllers import LandingPID  # noqa: E402
from rocketsim.scenarios import LandingScenario, calm, hard, moderate  # noqa: E402


def run_episode(scenario, dist, rand, seed):
    env = Environment()
    base = edf_testbed()  # nominal model the controller is allowed to know
    rng = np.random.default_rng(seed)

    true_vehicle = rand.sample_vehicle(base, rng)  # what physically flies
    sim = Simulator(true_vehicle, env, dt=0.002, disturbance=dist, rng=rng)

    controller = LandingPID(base, env)  # built on the NOMINAL model
    state0 = scenario.sample_initial_state(rng)

    traj = sim.run(
        controller,
        state0,
        duration=scenario.timeout,
        terminate=lambda t, s: scenario.check_done(s, t)[0],
    )
    return scenario.evaluate(traj)


def summarize(name, scenario, dist, rand, n):
    results = [run_episode(scenario, dist, rand, seed=s) for s in range(n)]
    succ = sum(r.success for r in results)
    landed = sum(r.landed for r in results)
    crashed = sum(r.crashed for r in results)
    vsp = [r.vertical_speed for r in results if r.landed]
    # which soft-landing criterion is violated among ground-reaching episodes?
    fail = {"offset": 0, "vspeed": 0, "hspeed": 0, "tilt": 0}
    for r in results:
        if r.landed and not r.success:
            if r.horizontal_offset > scenario.max_touchdown_offset:
                fail["offset"] += 1
            if r.vertical_speed > scenario.max_touchdown_vspeed:
                fail["vspeed"] += 1
            if r.horizontal_speed > scenario.max_touchdown_hspeed:
                fail["hspeed"] += 1
            if r.tilt_deg > scenario.max_touchdown_tilt_deg:
                fail["tilt"] += 1
    print(f"=== {name}  (n={n}) ===")
    print(f"  soft landings : {succ}/{n}  ({100*succ/n:.0f}%)")
    print(f"  reached ground: {landed}/{n}   crashed/aborted: {crashed}/{n}")
    if vsp:
        print(f"  touchdown vspeed: mean {np.mean(vsp):.2f}  worst {np.max(vsp):.2f} m/s")
    if succ < landed:
        print(f"  failure causes (landed but not soft): {fail}")
    print()
    return succ / n


def main() -> int:
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    calm_d, calm_r = calm()
    mod_d, mod_r = moderate()
    hard_d, hard_r = hard()

    print("PID landing baseline — difficulty curve\n")
    summarize("CALM      (known plant, no wind)", LandingScenario(), calm_d, calm_r, n)
    summarize("MODERATE  (breeze, small uncertainty)", LandingScenario(), mod_d, mod_r, n)
    summarize("HARD      (turbulent gusts, unknown plant)", LandingScenario.hard(), hard_d, hard_r, n)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
