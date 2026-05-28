# MPC Model Mismatch Diagnostic

Date: 2026-05-28

This diagnostic compares the convex point-mass MPC's planned trajectory against a no-disturbance 6-DOF rollout using the same rigid-body simulator as the landing environment.

Episodes per row: 5

| difficulty | plant model | planned | pos err | xy err | z err | vel err | actual alt | planned alt | final tilt | max tilt | throttle sat | gimbal sat |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moderate | nominal | 5/5 | 4.86 m | 1.73 m | 4.46 m | 4.76 m/s | -0.35 m | 4.11 m | 30.0 deg | 46.2 deg | 0% | 87% |
| moderate | oracle | 5/5 | 5.01 m | 1.61 m | 4.70 m | 4.49 m/s | -0.59 m | 4.11 m | 15.3 deg | 38.2 deg | 0% | 92% |
| hard | nominal | 4/5 | 6.25 m | 2.87 m | 5.19 m | 4.56 m/s | -0.68 m | 4.51 m | 27.4 deg | 42.8 deg | 0% | 93% |
| hard | oracle | 4/5 | 6.24 m | 1.54 m | 6.03 m | 6.24 m/s | -0.76 m | 5.27 m | 35.3 deg | 55.3 deg | 0% | 94% |
| noisy | nominal | 5/5 | 4.86 m | 1.73 m | 4.46 m | 4.76 m/s | -0.35 m | 4.11 m | 30.0 deg | 46.2 deg | 0% | 87% |
| noisy | oracle | 5/5 | 5.01 m | 1.61 m | 4.70 m | 4.49 m/s | -0.59 m | 4.11 m | 15.3 deg | 38.2 deg | 0% | 92% |

Interpretation: large error here means the optimizer's point-mass plan is not dynamically trackable by the actual gimbaled rigid body, even before future wind is added.

## Readout

- The point-mass plan predicts the vehicle should still be about 4-5 m above the pad, while the 6-DOF rollout has already crossed below the ground plane. That is a structural mismatch, not just bad controller tuning.
- The `oracle` plant model does not remove the mismatch. Knowing the exact episode mass/thrust/inertia helps some attitude numbers, but the plan is still dynamically untrackable because the optimizer ignores thrust spool-up, rigid-body attitude response, gimbal rate limits and the fact that lateral acceleration requires tilting the vehicle.
- Gimbal saturation is the strongest signal: 87-94% of rollout steps hit the gimbal limit. The point-mass solver is asking for lateral acceleration that the slender gimbaled body cannot realize on that time scale.
- Conclusion: the convex point-mass MPC can remain as high-level guidance, but it should not directly own landing authority in this simulator. It needs an attitude/actuator feasibility layer, a tracking controller, and a safety filter.
