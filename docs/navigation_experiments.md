# Navigation / GNC Controller Experiments

Date: 2026-05-28

Purpose: compare the same landing controllers when they consume ideal true state, raw noisy measured state, or estimated state. Physics and touchdown scoring always use the true simulated vehicle state.

Episodes per row: 10

Note: the `full` rows in this file are the first unconservative sampled full-dynamics MPC run. After this failed through attitude crashes, the controller was retuned into a conservative PID-residual safety filter. The updated `full`-only result is stored in `docs/full_dynamics_mpc_experiments.md`.

Controller labels:

- `pid`: cascaded LandingPID baseline.
- `vertical`: PID attitude/XY with sampled 1D vertical MPC throttle.
- `cvxpy`: direct 3D point-mass convex MPC thrust-vector guidance plus TVC attitude tracking.
- `guidance`: convex MPC waypoint/feed-forward acceleration tracked by a feedback layer.
- `waypoint`: convex MPC XY waypoint guidance with the existing PID landing gate.
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
| noisy | pid | nominal | estimated | 9/10 (90%) | 10/10 | 10.53 | offset 0.23 m, vspeed 0.17 m/s, hspeed 0.20 m/s, tilt 4.7 deg | touchdown:10 | tilt:1 |
| noisy | waypoint | nominal | estimated | 7/10 (70%) | 10/10 | 11.04 | offset 0.20 m, vspeed 0.17 m/s, hspeed 0.23 m/s, tilt 4.9 deg | touchdown:10 | hspeed:1, tilt:2 |
| noisy | full | nominal | estimated | 0/10 (0%) | 3/10 | 6.71 | offset 0.33 m, vspeed 1.12 m/s, hspeed 0.58 m/s, tilt 15.9 deg | crash_tilt:7, touchdown:3 | hspeed:1, tilt:3, vspeed:2 |
| hard | pid | nominal | estimated | 4/10 (40%) | 10/10 | 13.02 | offset 0.31 m, vspeed 0.08 m/s, hspeed 0.39 m/s, tilt 6.9 deg | touchdown:10 | hspeed:3, tilt:3 |
| hard | waypoint | nominal | estimated | 6/10 (60%) | 10/10 | 14.51 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.34 m/s, tilt 5.5 deg | touchdown:10 | hspeed:2, offset:2, tilt:2 |
| hard | full | nominal | estimated | 0/10 (0%) | 1/10 | 5.53 | offset 1.18 m, vspeed 0.75 m/s, hspeed 0.12 m/s, tilt 36.2 deg | crash_tilt:9, touchdown:1 | offset:1, tilt:1 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
