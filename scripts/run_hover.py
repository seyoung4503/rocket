"""Demo: stabilize the EDF testbed to a hover with the PID baseline.

Starts tilted and off-target, then holds a 5 m hover. Prints a summary and
writes the trajectory to out/hover.csv.

    python scripts/run_hover.py
"""

from __future__ import annotations

import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from rocketsim import Environment, Simulator, edf_testbed, initial_state  # noqa: E402
from rocketsim import quaternion as quat  # noqa: E402
from rocketsim.controllers import HoverPID  # noqa: E402


def small_tilt_quat(roll_deg: float, pitch_deg: float) -> np.ndarray:
    r, p = np.deg2rad(roll_deg) / 2, np.deg2rad(pitch_deg) / 2
    qr = np.array([np.cos(r), np.sin(r), 0.0, 0.0])
    qp = np.array([np.cos(p), 0.0, np.sin(p), 0.0])
    return quat.quat_mult(qr, qp)


def main() -> int:
    vehicle = edf_testbed()
    env = Environment()
    sim = Simulator(vehicle, env, dt=0.002)

    target = np.array([0.0, 0.0, 5.0])
    controller = HoverPID(vehicle, env, target=target)

    state0 = initial_state(
        position=(1.5, -1.0, 4.0),
        quaternion=small_tilt_quat(roll_deg=10.0, pitch_deg=-8.0),
    )

    traj = sim.run(controller, state0, duration=12.0)
    t, states, cmds = traj.as_arrays()

    final_pos = states[-1, 0:3]
    pos_err = np.linalg.norm(final_pos - target)
    final_tilt = np.rad2deg(quat.tilt_angle(states[-1, 6:10]))

    # settle time: last time the position error exceeds 0.25 m
    errs = np.linalg.norm(states[:, 0:3] - target, axis=1)
    above = np.where(errs > 0.25)[0]
    settle_t = t[above[-1]] if len(above) else 0.0

    print("=== EDF hover (PID baseline) ===")
    print(f"  start pos   : {states[0,0:3]}  tilt {np.rad2deg(quat.tilt_angle(states[0,6:10])):.1f} deg")
    print(f"  target      : {target}")
    print(f"  final pos   : {final_pos}")
    print(f"  final error : {pos_err*100:.1f} cm")
    print(f"  final tilt  : {final_tilt:.2f} deg")
    print(f"  settle time : {settle_t:.2f} s (|err| < 25 cm)")
    print(f"  mean throttle: {cmds[:,0].mean():.2f}")

    os.makedirs("out", exist_ok=True)
    with open("out/hover.csv", "w", newline="") as f:
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
    print("  wrote out/hover.csv")

    ok = pos_err < 0.25 and final_tilt < 2.0
    print("  RESULT:", "STABILIZED ✅" if ok else "did not settle ⚠️")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
