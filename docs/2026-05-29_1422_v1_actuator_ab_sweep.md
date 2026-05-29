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
| hard | waypoint | nominal | estimated | 26/50 (52%) | 50/50 | 14.38 | offset 0.36 m, vspeed 0.10 m/s, hspeed 0.34 m/s, tilt 6.5 deg | touchdown:50 | hspeed:13, offset:8, tilt:18 |
| hard | actuator | nominal | estimated | 23/50 (46%) | 50/50 | 15.11 | offset 0.35 m, vspeed 0.10 m/s, hspeed 0.39 m/s, tilt 5.8 deg | touchdown:50 | hspeed:16, offset:12, tilt:15 |
| hard | actuator_a | nominal | estimated | 23/50 (46%) | 50/50 | 15.27 | offset 0.38 m, vspeed 0.10 m/s, hspeed 0.38 m/s, tilt 6.8 deg | touchdown:50 | hspeed:13, offset:15, tilt:17 |
| hard | actuator_b | nominal | estimated | 22/50 (44%) | 50/50 | 15.33 | offset 0.39 m, vspeed 0.10 m/s, hspeed 0.39 m/s, tilt 6.9 deg | touchdown:50 | hspeed:13, offset:14, tilt:18 |
| noisy | pid | nominal | estimated | 42/50 (84%) | 50/50 | 10.57 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.1 deg | touchdown:50 | hspeed:4, tilt:6 |
| noisy | waypoint | nominal | estimated | 38/50 (76%) | 47/50 | 11.02 | offset 0.20 m, vspeed 0.20 m/s, hspeed 0.26 m/s, tilt 4.8 deg | timeout:3, touchdown:47 | hspeed:4, tilt:5 |
| noisy | actuator | nominal | estimated | 37/50 (74%) | 42/50 | 11.39 | offset 0.21 m, vspeed 0.20 m/s, hspeed 0.25 m/s, tilt 4.4 deg | timeout:8, touchdown:42 | hspeed:4, tilt:1 |
| noisy | actuator_a | nominal | estimated | 32/50 (64%) | 40/50 | 11.50 | offset 0.21 m, vspeed 0.21 m/s, hspeed 0.28 m/s, tilt 4.6 deg | timeout:10, touchdown:40 | hspeed:5, tilt:3 |
| noisy | actuator_b | nominal | estimated | 29/50 (58%) | 38/50 | 11.57 | offset 0.22 m, vspeed 0.20 m/s, hspeed 0.27 m/s, tilt 4.8 deg | timeout:12, touchdown:38 | hspeed:6, tilt:3 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
