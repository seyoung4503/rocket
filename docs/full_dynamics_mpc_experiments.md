# Navigation / GNC Controller Experiments

Date: 2026-05-28

Purpose: compare the same landing controllers when they consume ideal true state, raw noisy measured state, or estimated state. Physics and touchdown scoring always use the true simulated vehicle state.

Episodes per row: 10

Note: this run uses the conservative retuned `full` controller: sampled full-dynamics MPC acts only as a small residual around PID when the 6-DOF rollout predicts a clear improvement. It is still too slow for a 50 Hz hardware loop in this Python implementation.

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
| noisy | full | nominal | estimated | 0/10 (0%) | 4/10 | 10.84 | offset 0.23 m, vspeed 0.25 m/s, hspeed 0.67 m/s, tilt 18.0 deg | crash_tilt:2, timeout:4, touchdown:4 | hspeed:2, offset:1, tilt:4 |
| hard | full | nominal | estimated | 5/10 (50%) | 9/10 | 12.53 | offset 0.37 m, vspeed 0.08 m/s, hspeed 0.42 m/s, tilt 6.8 deg | crash_tilt:1, touchdown:9 | hspeed:2, offset:2, tilt:2 |

## Readout

- The comparison is only meaningful within each difficulty because `hard` changes initial conditions and disturbances, while `noisy` isolates sensor noise on top of the moderate disturbance preset.
- A controller that improves under `true` but collapses under `measured` is not ready for hardware; it needs a stronger navigation/estimation layer before controller tuning is meaningful.
- `landed fail` breaks down soft-landing misses after reaching the ground: pad offset, vertical speed, horizontal speed, and tilt.

## Readout

- In `hard`, the conservative full-dynamics residual reached 5/10 soft landings. That is better than the earlier direct full-dynamics run, and near the 6/10 waypoint result from the broader comparison.
- In `noisy`, it still failed 0/10. The failures are not mainly vertical speed; they are horizontal speed, tilt and timeout. That points back to navigation noise plus sampled residual selection, not raw point-mass mismatch.
- This class is useful as an offline diagnostic and research controller. It is not a practical onboard MPC form yet; the next practical version should be a lower-dimensional tracker or SQP/iLQR-style short-horizon controller with explicit attitude and actuator constraints.
