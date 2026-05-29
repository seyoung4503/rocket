# SpaceX MPC + Step 1/Step 1+2 — Raw Results

**Date**: 2026-05-29 19:03 KST  
**Episodes per row**: 50, mode=estimated, plant=nominal, 6 workers parallel.

5 controllers × 4 difficulties.  Note: the eval was finalized in two
batches (the main batch was interrupted before the last two cells; those
were re-run separately and merged here).

| difficulty | controller | success | landed | throttle int | touchdown mean | reasons |
|---|---|---:|---:|---:|---|---|
| hard | pid | 29/50 (58%) | 50/50 | 12.94 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.33 m/s, tilt 6.0 deg | touchdown:50 |
| hard | actuator | 34/50 (68%) | 50/50 | 13.72 | offset 0.34 m, vspeed 0.10 m/s, hspeed 0.31 m/s, tilt 5.2 deg | touchdown:50 |
| hard | spacex | 7/50 (14%) | 14/50 | — | offset 0.11 m, vspeed 0.48 m/s, hspeed 0.34 m/s, tilt 6.5 deg | touchdown:14, timeout:36 |
| hard | spacex_actuator | 5/50 (10%) | 27/50 | — | offset 0.30 m, vspeed 0.64 m/s, hspeed 0.54 m/s, tilt 8.5 deg | touchdown:27, timeout:23 |
| hard | spacex_actuator2 | 4/50 (8%) | 29/50 | — | offset 0.34 m, vspeed 0.59 m/s, hspeed 0.51 m/s, tilt 8.5 deg | touchdown:29, timeout:21 |
| noisy | pid | 42/50 (84%) | 50/50 | 10.57 | offset 0.21 m, vspeed 0.18 m/s, hspeed 0.27 m/s, tilt 5.1 deg | touchdown:50 |
| noisy | actuator | 39/50 (78%) | 49/50 | 10.87 | offset 0.22 m, vspeed 0.19 m/s, hspeed 0.25 m/s, tilt 4.9 deg | touchdown:49, timeout:1 |
| noisy | spacex | 14/50 (28%) | 48/50 | — | offset 0.15 m, vspeed 0.97 m/s, hspeed 0.61 m/s, tilt 7.3 deg | touchdown:48, crash_tilt:1, timeout:1 |
| noisy | spacex_actuator | 5/50 (10%) | 31/50 | — | offset 0.34 m, vspeed 0.66 m/s, hspeed 0.75 m/s, tilt 6.5 deg | touchdown:31, timeout:19 |
| noisy | spacex_actuator2 | 7/50 (14%) | 32/50 | — | offset 0.34 m, vspeed 0.64 m/s, hspeed 0.71 m/s, tilt 6.7 deg | touchdown:32, timeout:18 |
| divert | pid | 39/50 (78%) | 40/50 | 9.17 | offset 0.18 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.9 deg | touchdown:40, too_high:10 |
| divert | actuator | 45/50 (90%) | 45/50 | — | offset 0.20 m, vspeed 0.13 m/s, hspeed 0.16 m/s, tilt 2.5 deg | touchdown:45, timeout:5 |
| divert | spacex | 16/50 (32%) | 17/50 | — | offset 0.04 m, vspeed 0.15 m/s, hspeed 0.15 m/s, tilt 3.5 deg | touchdown:17, timeout:30, crash_tilt:3 |
| divert | spacex_actuator | 6/50 (12%) | 12/50 | — | offset 0.13 m, vspeed 0.42 m/s, hspeed 0.37 m/s, tilt 6.3 deg | touchdown:12, timeout:38 |
| divert | spacex_actuator2 | 4/50 (8%) | 12/50 | — | offset 0.14 m, vspeed 0.57 m/s, hspeed 0.52 m/s, tilt 5.6 deg | touchdown:12, timeout:38 |
| divert_hard | pid | 9/50 (18%) | 13/50 | 6.69 | offset 0.34 m, vspeed 0.09 m/s, hspeed 0.40 m/s, tilt 4.3 deg | touchdown:13, too_high:37 |
| divert_hard | actuator | 26/50 (52%) | 49/50 | — | offset 0.33 m, vspeed 0.10 m/s, hspeed 0.33 m/s, tilt 6.2 deg | touchdown:49, crash_tilt:1 |
| divert_hard | spacex | 12/50 (24%) | 17/50 | — | offset 0.09 m, vspeed 0.35 m/s, hspeed 0.36 m/s, tilt 5.2 deg | touchdown:17, timeout:30, crash_tilt:3 |
| divert_hard | spacex_actuator | 5/50 (10%) | 26/50 | — | offset 0.70 m, vspeed 0.30 m/s, hspeed 0.36 m/s, tilt 6.1 deg | touchdown:26, timeout:24 |
| divert_hard | spacex_actuator2 | 5/50 (10%) | 26/50 | — | offset 0.65 m, vspeed 0.32 m/s, hspeed 0.40 m/s, tilt 7.0 deg | touchdown:26, timeout:24 |

Analysis: `docs/2026-05-29_2045_v1_spacex_actuator_analysis.md`.
