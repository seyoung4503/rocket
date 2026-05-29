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
| hard | waypoint | nominal | estimated | 27/50 (54%) | 50/50 | 13.47 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.35 m/s, tilt 5.6 deg | touchdown:50 | hspeed:9, offset:11, tilt:12 |
| hard | actuator | nominal | estimated | 34/50 (68%) | 50/50 | 13.72 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.31 m/s, tilt 5.2 deg | touchdown:50 | hspeed:7, offset:7, tilt:9 |
| hard | actuator2 | nominal | estimated | 32/50 (64%) | 50/50 | 13.68 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.32 m/s, tilt 5.1 deg | touchdown:50 | hspeed:9, offset:7, tilt:9 |
| hard | tracking_pointmass | nominal | estimated | 13/50 (26%) | 30/50 | 10.86 | offset 2.02 m, vspeed 2.22 m/s, hspeed 1.08 m/s, tilt 3.1 deg | timeout:20, touchdown:30 | hspeed:11, offset:17, tilt:3, vspeed:11 |
| hard | tracking_actuator | nominal | estimated | 5/50 (10%) | 30/50 | 10.38 | offset 2.16 m, vspeed 2.28 m/s, hspeed 1.20 m/s, tilt 3.6 deg | timeout:20, touchdown:30 | hspeed:18, offset:22, tilt:3, vspeed:13 |
| hard | tracking_actuator2 | nominal | estimated | 2/50 (4%) | 26/50 | 11.93 | offset 2.42 m, vspeed 2.60 m/s, hspeed 1.30 m/s, tilt 3.4 deg | timeout:24, touchdown:26 | hspeed:17, offset:19, tilt:2, vspeed:15 |
| noisy | pid | nominal | estimated | 42/50 (84%) | 50/50 | 10.57 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.1 deg | touchdown:50 | hspeed:4, tilt:6 |
| noisy | waypoint | nominal | estimated | 36/50 (72%) | 49/50 | 10.77 | offset 0.22 m, vspeed 0.18 m/s, hspeed 0.24 m/s, tilt 5.2 deg | timeout:1, touchdown:49 | hspeed:2, offset:1, tilt:12 |
| noisy | actuator | nominal | estimated | 39/50 (78%) | 49/50 | 10.87 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 4.9 deg | timeout:1, touchdown:49 | hspeed:4, offset:1, tilt:6 |
| noisy | actuator2 | nominal | estimated | 36/50 (72%) | 49/50 | 10.86 | offset 0.21 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 5.1 deg | timeout:1, touchdown:49 | hspeed:5, offset:1, tilt:8 |
| noisy | tracking_pointmass | nominal | estimated | 24/50 (48%) | 27/50 | 9.51 | offset 0.22 m, vspeed 0.21 m/s, hspeed 0.27 m/s, tilt 4.6 deg | timeout:23, touchdown:27 | hspeed:2, offset:1, tilt:1 |
| noisy | tracking_actuator | nominal | estimated | 20/50 (40%) | 27/50 | 8.90 | offset 0.35 m, vspeed 0.26 m/s, hspeed 0.23 m/s, tilt 3.8 deg | timeout:23, touchdown:27 | offset:7 |
| noisy | tracking_actuator2 | nominal | estimated | 13/50 (26%) | 18/50 | 10.05 | offset 0.35 m, vspeed 0.28 m/s, hspeed 0.25 m/s, tilt 3.3 deg | timeout:32, touchdown:18 | hspeed:1, offset:3, tilt:1 |
| divert | pid | nominal | estimated | 39/50 (78%) | 40/50 | 9.17 | offset 0.18 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | too_high:10, touchdown:40 | offset:1 |
| divert | waypoint | nominal | estimated | 47/50 (94%) | 47/50 | 11.37 | offset 0.20 m, vspeed 0.14 m/s, hspeed 0.15 m/s, tilt 2.5 deg | timeout:3, touchdown:47 | - |
| divert | actuator | nominal | estimated | 45/50 (90%) | 45/50 | 11.39 | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.5 deg | timeout:5, touchdown:45 | - |
| divert | actuator2 | nominal | estimated | 48/50 (96%) | 48/50 | 10.90 | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.6 deg | timeout:2, touchdown:48 | - |
| divert | tracking_pointmass | nominal | estimated | 23/50 (46%) | 26/50 | 9.88 | offset 0.27 m, vspeed 0.16 m/s, hspeed 0.19 m/s, tilt 2.9 deg | timeout:24, touchdown:26 | offset:3 |
| divert | tracking_actuator | nominal | estimated | 10/50 (20%) | 25/50 | 9.25 | offset 0.70 m, vspeed 0.22 m/s, hspeed 0.36 m/s, tilt 3.1 deg | timeout:25, touchdown:25 | hspeed:7, offset:14 |
| divert | tracking_actuator2 | nominal | estimated | 9/50 (18%) | 20/50 | 9.82 | offset 0.66 m, vspeed 0.23 m/s, hspeed 0.43 m/s, tilt 3.2 deg | timeout:30, touchdown:20 | hspeed:7, offset:9 |
| divert_hard | pid | nominal | estimated | 9/50 (18%) | 13/50 | 6.69 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 4.3 deg | too_high:37, touchdown:13 | hspeed:3, offset:1, tilt:2 |
| divert_hard | waypoint | nominal | estimated | 32/50 (64%) | 50/50 | 14.75 | offset 0.31 m, vspeed 0.10 m/s, hspeed 0.32 m/s, tilt 6.0 deg | touchdown:50 | hspeed:8, offset:8, tilt:9 |
| divert_hard | actuator | nominal | estimated | 26/50 (52%) | 49/50 | 14.62 | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.2 deg | crash_tilt:1, touchdown:49 | hspeed:7, offset:9, tilt:15 |
| divert_hard | actuator2 | nominal | estimated | 26/50 (52%) | 49/50 | 14.56 | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.4 deg | crash_tilt:1, touchdown:49 | hspeed:8, offset:9, tilt:12 |
| divert_hard | tracking_pointmass | nominal | estimated | 6/50 (12%) | 26/50 | 11.04 | offset 2.12 m, vspeed 1.81 m/s, hspeed 1.01 m/s, tilt 4.0 deg | out_of_bounds:4, timeout:20, touchdown:26 | hspeed:15, offset:15, tilt:3, vspeed:7 |
| divert_hard | tracking_actuator | nominal | estimated | 0/50 (0%) | 25/50 | 10.82 | offset 2.67 m, vspeed 1.92 m/s, hspeed 1.30 m/s, tilt 4.9 deg | out_of_bounds:4, timeout:21, touchdown:25 | hspeed:21, offset:23, tilt:4, vspeed:8 |
| divert_hard | tracking_actuator2 | nominal | estimated | 1/50 (2%) | 22/50 | 11.87 | offset 2.96 m, vspeed 2.16 m/s, hspeed 1.33 m/s, tilt 4.2 deg | out_of_bounds:4, timeout:24, touchdown:22 | hspeed:17, offset:21, tilt:3, vspeed:9 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
