# Navigation / GNC Controller Experiments

Date: 2026-05-28

Purpose: compare the same landing controllers when they consume ideal true state, raw noisy measured state, or estimated state. Physics and touchdown scoring always use the true simulated vehicle state.

Episodes per row: 50

Controller labels:

- `pid`: cascaded LandingPID baseline.
- `vertical`: PID attitude/XY with sampled 1D vertical MPC throttle.
- `cvxpy`: direct 3D point-mass convex MPC thrust-vector guidance plus TVC attitude tracking.
- `guidance`: convex MPC waypoint/feed-forward acceleration tracked by a feedback layer.
- `waypoint`: convex MPC XY waypoint guidance with the existing PID landing gate.
- `feasible`: convex MPC XY waypoint guidance projected into a simple attitude/actuator feasibility envelope before PID tracking.
- `full`: sampled nonlinear MPC that rolls candidates through the same 6-DOF rigid-body dynamics used by the simulator.

Input labels:

- `true`: non-hardware upper bound; controller receives exact simulator state.
- `measured`: raw sensor-like measurement.
- `estimated`: controller receives the navigation estimator output.

Plant model labels:

- `nominal`: controller uses the nominal EDF model while the episode may randomize the actual vehicle.
- `oracle`: controller is given the randomized vehicle model for that episode. This is not hardware-realistic by itself, but isolates whether failures come from model mismatch versus controller structure.

| difficulty | controller | plant model | input | success | landed | throttle int | touchdown mean | reasons | landed fail |
|---|---|---|---:|---:|---:|---:|---|---|---|
| hard | pid | nominal | estimated | 29/50 (58%) | 50/50 | 12.94 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.33 m/s, tilt 6.0 deg | touchdown:50 | hspeed:8, offset:6, tilt:11 |
| hard | actuator | nominal | estimated | 34/50 (68%) | 50/50 | 13.72 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.31 m/s, tilt 5.2 deg | touchdown:50 | hspeed:7, offset:7, tilt:9 |
| hard | scp_warm | nominal | estimated | 35/50 (70%) | 50/50 | 13.78 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.32 m/s, tilt 5.4 deg | touchdown:50 | hspeed:7, offset:8, tilt:6 |
| hard | actuator_tuned | nominal | estimated | 30/50 (60%) | 50/50 | 13.29 | offset 0.35 m, vspeed 0.09 m/s, hspeed 0.35 m/s, tilt 5.7 deg | touchdown:50 | hspeed:9, offset:7, tilt:12 |
| hard | scp_warm_tuned | nominal | estimated | 30/50 (60%) | 50/50 | 13.56 | offset 0.35 m, vspeed 0.09 m/s, hspeed 0.32 m/s, tilt 5.4 deg | touchdown:50 | hspeed:11, offset:10, tilt:11 |
| noisy | pid | nominal | estimated | 42/50 (84%) | 50/50 | 10.57 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.1 deg | touchdown:50 | hspeed:4, tilt:6 |
| noisy | actuator | nominal | estimated | 39/50 (78%) | 49/50 | 10.87 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 4.9 deg | timeout:1, touchdown:49 | hspeed:4, offset:1, tilt:6 |
| noisy | scp_warm | nominal | estimated | 37/50 (74%) | 49/50 | 10.92 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 5.2 deg | timeout:1, touchdown:49 | hspeed:3, offset:1, tilt:10 |
| noisy | actuator_tuned | nominal | estimated | 39/50 (78%) | 49/50 | 10.71 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.25 m/s, tilt 5.1 deg | timeout:1, touchdown:49 | hspeed:3, offset:1, tilt:9 |
| noisy | scp_warm_tuned | nominal | estimated | 39/50 (78%) | 49/50 | 10.88 | offset 0.22 m, vspeed 0.20 m/s, hspeed 0.25 m/s, tilt 5.1 deg | timeout:1, touchdown:49 | hspeed:4, offset:1, tilt:8 |
| divert | pid | nominal | estimated | 39/50 (78%) | 40/50 | 9.17 | offset 0.18 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | too_high:10, touchdown:40 | offset:1 |
| divert | actuator | nominal | estimated | 45/50 (90%) | 45/50 | 11.39 | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.5 deg | timeout:5, touchdown:45 | - |
| divert | scp_warm | nominal | estimated | 42/50 (84%) | 42/50 | 11.57 | offset 0.19 m, vspeed 0.14 m/s, hspeed 0.16 m/s, tilt 2.5 deg | timeout:8, touchdown:42 | - |
| divert | actuator_tuned | nominal | estimated | 50/50 (100%) | 50/50 | 10.86 | offset 0.21 m, vspeed 0.13 m/s, hspeed 0.17 m/s, tilt 3.2 deg | touchdown:50 | - |
| divert | scp_warm_tuned | nominal | estimated | 48/50 (96%) | 48/50 | 11.33 | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.4 deg | timeout:2, touchdown:48 | - |
| divert_hard | pid | nominal | estimated | 9/50 (18%) | 13/50 | 6.69 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 4.3 deg | too_high:37, touchdown:13 | hspeed:3, offset:1, tilt:2 |
| divert_hard | actuator | nominal | estimated | 26/50 (52%) | 49/50 | 14.62 | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.2 deg | crash_tilt:1, touchdown:49 | hspeed:7, offset:9, tilt:15 |
| divert_hard | scp_warm | nominal | estimated | 30/50 (60%) | 50/50 | 14.97 | offset 0.32 m, vspeed 0.10 m/s, hspeed 0.34 m/s, tilt 6.4 deg | touchdown:50 | hspeed:7, offset:9, tilt:14 |
| divert_hard | actuator_tuned | nominal | estimated | 29/50 (58%) | 50/50 | 14.47 | offset 0.35 m, vspeed 0.10 m/s, hspeed 0.34 m/s, tilt 5.5 deg | touchdown:50 | hspeed:9, offset:9, tilt:12 |
| divert_hard | scp_warm_tuned | nominal | estimated | 32/50 (64%) | 49/50 | 14.55 | offset 0.32 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.2 deg | too_high:1, touchdown:49 | hspeed:9, offset:6, tilt:10 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
