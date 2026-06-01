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
| hard | pid_roll | nominal | estimated | 21/50 (42%) | 47/50 | 13.05 | offset 0.37 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 5.7 deg | crash_tilt:3, touchdown:47 | hspeed:12, offset:10, tilt:11 |
| hard | actuator_roll | nominal | estimated | 24/50 (48%) | 47/50 | 13.86 | offset 0.36 m, vspeed 0.09 m/s, hspeed 0.35 m/s, tilt 5.7 deg | crash_tilt:3, touchdown:47 | hspeed:10, offset:8, tilt:13 |
| hard | scp_warm_roll | nominal | estimated | 23/50 (46%) | 48/50 | 13.94 | offset 0.36 m, vspeed 0.10 m/s, hspeed 0.35 m/s, tilt 5.9 deg | crash_tilt:2, touchdown:48 | hspeed:12, offset:9, tilt:12 |
| noisy | pid_roll | nominal | estimated | 43/50 (86%) | 50/50 | 10.83 | offset 0.23 m, vspeed 0.19 m/s, hspeed 0.27 m/s, tilt 4.8 deg | touchdown:50 | hspeed:4, tilt:4 |
| noisy | actuator_roll | nominal | estimated | 40/50 (80%) | 48/50 | 11.06 | offset 0.23 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.0 deg | timeout:2, touchdown:48 | hspeed:4, tilt:6 |
| noisy | scp_warm_roll | nominal | estimated | 40/50 (80%) | 47/50 | 11.21 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.26 m/s, tilt 5.2 deg | timeout:3, touchdown:47 | hspeed:3, tilt:4 |
| divert | pid_roll | nominal | estimated | 38/50 (76%) | 39/50 | 9.13 | offset 0.19 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | too_high:11, touchdown:39 | offset:1 |
| divert | actuator_roll | nominal | estimated | 43/50 (86%) | 43/50 | 11.49 | offset 0.20 m, vspeed 0.14 m/s, hspeed 0.17 m/s, tilt 2.5 deg | timeout:7, touchdown:43 | - |
| divert | scp_warm_roll | nominal | estimated | 42/50 (84%) | 42/50 | 11.60 | offset 0.19 m, vspeed 0.14 m/s, hspeed 0.17 m/s, tilt 2.6 deg | timeout:8, touchdown:42 | - |
| divert_hard | pid_roll | nominal | estimated | 5/50 (10%) | 11/50 | 6.48 | offset 0.37 m, vspeed 0.09 m/s, hspeed 0.47 m/s, tilt 6.2 deg | crash_tilt:5, too_high:34, touchdown:11 | hspeed:5, offset:2, tilt:2 |
| divert_hard | actuator_roll | nominal | estimated | 22/50 (44%) | 46/50 | 14.93 | offset 0.33 m, vspeed 0.09 m/s, hspeed 0.35 m/s, tilt 6.4 deg | crash_tilt:4, touchdown:46 | hspeed:9, offset:10, tilt:13 |
| divert_hard | scp_warm_roll | nominal | estimated | 22/50 (44%) | 42/50 | 14.45 | offset 0.33 m, vspeed 0.09 m/s, hspeed 0.34 m/s, tilt 5.9 deg | crash_tilt:7, too_high:1, touchdown:42 | hspeed:7, offset:9, tilt:10 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
