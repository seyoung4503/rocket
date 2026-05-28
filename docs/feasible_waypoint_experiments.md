# Navigation / GNC Controller Experiments

Date: 2026-05-28

Purpose: compare the same landing controllers when they consume ideal true state, raw noisy measured state, or estimated state. Physics and touchdown scoring always use the true simulated vehicle state.

Episodes per row: 100

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
| noisy | pid | nominal | estimated | 85/100 (85%) | 100/100 | 10.59 | offset 0.21 m, vspeed 0.19 m/s, hspeed 0.26 m/s, tilt 4.9 deg | touchdown:100 | hspeed:6, tilt:11 |
| noisy | waypoint | nominal | estimated | 75/100 (75%) | 95/100 | 11.02 | offset 0.21 m, vspeed 0.21 m/s, hspeed 0.26 m/s, tilt 5.1 deg | timeout:5, touchdown:95 | hspeed:7, tilt:13 |
| noisy | feasible | nominal | estimated | 79/100 (79%) | 98/100 | 10.91 | offset 0.21 m, vspeed 0.21 m/s, hspeed 0.25 m/s, tilt 5.4 deg | timeout:2, touchdown:98 | hspeed:5, tilt:15 |
| hard | pid | nominal | estimated | 34/100 (34%) | 100/100 | 13.52 | offset 0.37 m, vspeed 0.09 m/s, hspeed 0.46 m/s, tilt 6.8 deg | touchdown:100 | hspeed:39, offset:20, tilt:35 |
| hard | waypoint | nominal | estimated | 33/100 (33%) | 100/100 | 14.72 | offset 0.37 m, vspeed 0.09 m/s, hspeed 0.43 m/s, tilt 7.5 deg | touchdown:100 | hspeed:37, offset:29, tilt:40 |
| hard | feasible | nominal | estimated | 34/100 (34%) | 100/100 | 14.51 | offset 0.35 m, vspeed 0.09 m/s, hspeed 0.41 m/s, tilt 7.5 deg | touchdown:100 | hspeed:36, offset:26, tilt:41 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.
- This run confirms the cleaned-up MPC+PID structure, but not a performance win. At n=100, `feasible` improves over raw `waypoint` in noisy stability, but both MPC+PID variants are below PID in noisy and roughly tied with PID in hard.
- The remaining hard failures are not vertical-speed failures. They are mostly horizontal speed, offset and tilt at touchdown, so the next real improvement has to change terminal guidance/trajectory timing, not just clip the MPC waypoint.
- The current cvxpy evaluation is slow: n=100 over PID/waypoint/feasible and noisy/hard takes several minutes. This matters for later onboard feasibility.
