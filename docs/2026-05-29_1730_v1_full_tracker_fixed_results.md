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
| hard | full_pointmass | nominal | estimated | 1/50 (2%) | 12/50 | 17.35 | offset 4.42 m, vspeed 5.00 m/s, hspeed 2.25 m/s, tilt 1.2 deg | timeout:38, touchdown:12 | hspeed:11, offset:11, vspeed:11 |
| hard | full_actuator | nominal | estimated | 0/50 (0%) | 11/50 | 16.54 | offset 4.80 m, vspeed 5.42 m/s, hspeed 2.45 m/s, tilt 0.9 deg | timeout:36, too_high:3, touchdown:11 | hspeed:11, offset:11, vspeed:11 |
| hard | full_actuator2 | nominal | estimated | 0/50 (0%) | 11/50 | 15.51 | offset 4.80 m, vspeed 5.42 m/s, hspeed 2.45 m/s, tilt 0.9 deg | timeout:33, too_high:6, touchdown:11 | hspeed:11, offset:11, vspeed:11 |
| noisy | full_pointmass | nominal | estimated | 15/50 (30%) | 15/50 | 12.00 | offset 0.26 m, vspeed 0.51 m/s, hspeed 0.23 m/s, tilt 2.8 deg | timeout:35, touchdown:15 | - |
| noisy | full_actuator | nominal | estimated | 1/50 (2%) | 1/50 | 12.57 | offset 0.21 m, vspeed 0.89 m/s, hspeed 0.26 m/s, tilt 2.5 deg | timeout:49, touchdown:1 | - |
| noisy | full_actuator2 | nominal | estimated | 5/50 (10%) | 5/50 | 12.46 | offset 0.17 m, vspeed 0.53 m/s, hspeed 0.17 m/s, tilt 3.2 deg | timeout:45, touchdown:5 | - |
| divert | full_pointmass | nominal | estimated | 2/50 (4%) | 2/50 | 12.49 | offset 0.31 m, vspeed 0.46 m/s, hspeed 0.23 m/s, tilt 4.6 deg | timeout:48, touchdown:2 | - |
| divert | full_actuator | nominal | estimated | 1/50 (2%) | 1/50 | 12.50 | offset 0.42 m, vspeed 0.59 m/s, hspeed 0.39 m/s, tilt 2.9 deg | timeout:49, touchdown:1 | - |
| divert | full_actuator2 | nominal | estimated | 0/50 (0%) | 0/50 | 11.98 | - | timeout:46, too_high:4 | - |
| divert_hard | full_pointmass | nominal | estimated | 0/50 (0%) | 7/50 | 17.18 | offset 6.09 m, vspeed 5.72 m/s, hspeed 2.30 m/s, tilt 0.9 deg | out_of_bounds:4, timeout:38, too_high:1, touchdown:7 | hspeed:7, offset:7, vspeed:7 |
| divert_hard | full_actuator | nominal | estimated | 0/50 (0%) | 7/50 | 16.54 | offset 6.09 m, vspeed 5.72 m/s, hspeed 2.30 m/s, tilt 0.9 deg | out_of_bounds:4, timeout:36, too_high:3, touchdown:7 | hspeed:7, offset:7, vspeed:7 |
| divert_hard | full_actuator2 | nominal | estimated | 1/50 (2%) | 9/50 | 16.07 | offset 4.86 m, vspeed 4.50 m/s, hspeed 1.88 m/s, tilt 2.0 deg | out_of_bounds:4, timeout:33, too_high:4, touchdown:9 | hspeed:7, offset:8, tilt:1, vspeed:7 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
