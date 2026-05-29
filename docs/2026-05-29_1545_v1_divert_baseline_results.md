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
| divert | pid | nominal | estimated | 39/50 (78%) | 40/50 | 9.17 | offset 0.18 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | too_high:10, touchdown:40 | offset:1 |
| divert | waypoint | nominal | estimated | 21/50 (42%) | 21/50 | 12.24 | offset 0.19 m, vspeed 0.15 m/s, hspeed 0.17 m/s, tilt 2.6 deg | timeout:29, touchdown:21 | - |
| divert | actuator | nominal | estimated | 8/50 (16%) | 8/50 | 12.36 | offset 0.18 m, vspeed 0.15 m/s, hspeed 0.16 m/s, tilt 2.9 deg | timeout:42, touchdown:8 | - |
| divert | actuator2 | nominal | estimated | 10/50 (20%) | 10/50 | 12.28 | offset 0.17 m, vspeed 0.16 m/s, hspeed 0.16 m/s, tilt 2.1 deg | timeout:40, touchdown:10 | - |
| divert_hard | pid | nominal | estimated | 9/50 (18%) | 13/50 | 6.69 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 4.3 deg | too_high:37, touchdown:13 | hspeed:3, offset:1, tilt:2 |
| divert_hard | waypoint | nominal | estimated | 29/50 (58%) | 50/50 | 15.73 | offset 0.38 m, vspeed 0.10 m/s, hspeed 0.37 m/s, tilt 6.8 deg | touchdown:50 | hspeed:10, offset:13, tilt:14 |
| divert_hard | actuator | nominal | estimated | 28/50 (56%) | 50/50 | 15.95 | offset 0.39 m, vspeed 0.10 m/s, hspeed 0.32 m/s, tilt 6.8 deg | touchdown:50 | hspeed:7, offset:16, tilt:14 |
| divert_hard | actuator2 | nominal | estimated | 27/50 (54%) | 49/50 | 15.65 | offset 0.39 m, vspeed 0.10 m/s, hspeed 0.34 m/s, tilt 7.0 deg | too_high:1, touchdown:49 | hspeed:11, offset:14, tilt:13 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
