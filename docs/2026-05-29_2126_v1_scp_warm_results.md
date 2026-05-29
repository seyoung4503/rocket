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
| hard | scp | nominal | estimated | 29/50 (58%) | 50/50 | 13.85 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.35 m/s, tilt 5.5 deg | touchdown:50 | hspeed:12, offset:7, tilt:11 |
| hard | scp_warm | nominal | estimated | 33/50 (66%) | 50/50 | 13.31 | offset 0.29 m, vspeed 0.10 m/s, hspeed 0.34 m/s, tilt 5.5 deg | touchdown:50 | hspeed:9, offset:3, tilt:10 |
| noisy | pid | nominal | estimated | 42/50 (84%) | 50/50 | 10.57 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.1 deg | touchdown:50 | hspeed:4, tilt:6 |
| noisy | actuator | nominal | estimated | 39/50 (78%) | 49/50 | 10.87 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 4.9 deg | timeout:1, touchdown:49 | hspeed:4, offset:1, tilt:6 |
| noisy | scp | nominal | estimated | 37/50 (74%) | 49/50 | 10.99 | offset 0.21 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 5.5 deg | timeout:1, touchdown:49 | hspeed:3, offset:1, tilt:8 |
| noisy | scp_warm | nominal | estimated | 39/50 (78%) | 49/50 | 10.74 | offset 0.22 m, vspeed 0.18 m/s, hspeed 0.26 m/s, tilt 4.8 deg | timeout:1, touchdown:49 | hspeed:3, offset:1, tilt:7 |
| divert | pid | nominal | estimated | 39/50 (78%) | 40/50 | 9.17 | offset 0.18 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | too_high:10, touchdown:40 | offset:1 |
| divert | actuator | nominal | estimated | 45/50 (90%) | 45/50 | 11.39 | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.5 deg | timeout:5, touchdown:45 | - |
| divert | scp | nominal | estimated | 42/50 (84%) | 42/50 | 11.63 | offset 0.18 m, vspeed 0.14 m/s, hspeed 0.17 m/s, tilt 2.5 deg | timeout:8, touchdown:42 | - |
| divert | scp_warm | nominal | estimated | 35/50 (70%) | 35/50 | 10.64 | offset 0.23 m, vspeed 0.12 m/s, hspeed 0.19 m/s, tilt 3.1 deg | timeout:9, too_high:6, touchdown:35 | - |
| divert_hard | pid | nominal | estimated | 9/50 (18%) | 13/50 | 6.69 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 4.3 deg | too_high:37, touchdown:13 | hspeed:3, offset:1, tilt:2 |
| divert_hard | actuator | nominal | estimated | 26/50 (52%) | 49/50 | 14.62 | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.2 deg | crash_tilt:1, touchdown:49 | hspeed:7, offset:9, tilt:15 |
| divert_hard | scp | nominal | estimated | 28/50 (56%) | 50/50 | 15.05 | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.4 deg | touchdown:50 | hspeed:7, offset:9, tilt:14 |
| divert_hard | scp_warm | nominal | estimated | 9/50 (18%) | 26/50 | 9.72 | offset 0.38 m, vspeed 0.09 m/s, hspeed 0.42 m/s, tilt 5.9 deg | too_high:24, touchdown:26 | hspeed:7, offset:8, tilt:7 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
