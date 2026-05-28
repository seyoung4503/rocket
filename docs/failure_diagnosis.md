# Failure Diagnosis

Date: 2026-05-28

This report logs per-seed landing outcomes. It is meant to answer where each controller fails, not only the aggregate success rate.

## Detailed Trace Logs

Per-step trajectory CSV logs are saved by default under `out/failure_traces`.

One file is written per difficulty/controller/mode/plant-model/seed, with the final success/fail status in the filename. Each row includes commanded throttle/gimbal, applied gimbal, guidance/MPC debug fields, and true/measured/estimated/controller-input state snapshots.

## Summary

| difficulty | controller | success | reasons | primary failures | touchdown mean |
|---|---|---:|---|---|---|
| hard | feasible | 12/30 (40%) | {'touchdown': 30} | {'tilt': 5, 'hspeed': 3, 'offset+tilt': 6, 'hspeed+tilt': 2, 'offset+hspeed+tilt': 1, 'offset+hspeed': 1} | offset 0.35, vspeed 0.10, hspeed 0.37, tilt 7.5 |
| hard | pid | 13/30 (43%) | {'touchdown': 30} | {'tilt': 6, 'hspeed': 8, 'offset+tilt': 3} | offset 0.33, vspeed 0.09, hspeed 0.41, tilt 6.3 |
| hard | waypoint | 14/30 (47%) | {'touchdown': 30} | {'hspeed': 2, 'offset': 2, 'offset+tilt': 8, 'hspeed+tilt': 2, 'tilt': 1, 'offset+hspeed': 1} | offset 0.39, vspeed 0.10, hspeed 0.37, tilt 7.5 |

## Seeds Where Controllers Disagree

| difficulty | seed | outcome pattern | details |
|---|---:|---|---|
| hard | 1 | pid:tilt, waypoint:ok, feasible:ok | pid(off=0.33, hs=0.27, tilt=8.5), waypoint(off=0.29, hs=0.15, tilt=4.4), feasible(off=0.31, hs=0.13, tilt=7.7) |
| hard | 2 | pid:hspeed, waypoint:ok, feasible:ok | pid(off=0.12, hs=0.70, tilt=7.7), waypoint(off=0.14, hs=0.36, tilt=6.1), feasible(off=0.10, hs=0.38, tilt=4.4) |
| hard | 3 | pid:hspeed, waypoint:ok, feasible:ok | pid(off=0.27, hs=0.77, tilt=6.2), waypoint(off=0.31, hs=0.21, tilt=2.5), feasible(off=0.25, hs=0.27, tilt=1.9) |
| hard | 4 | pid:ok, waypoint:ok, feasible:hspeed | pid(off=0.43, hs=0.39, tilt=7.1), waypoint(off=0.27, hs=0.16, tilt=2.0), feasible(off=0.22, hs=0.72, tilt=6.2) |
| hard | 8 | pid:ok, waypoint:hspeed+tilt, feasible:ok | pid(off=0.11, hs=0.13, tilt=6.8), waypoint(off=0.23, hs=0.54, tilt=9.7), feasible(off=0.33, hs=0.40, tilt=1.8) |
| hard | 9 | pid:ok, waypoint:ok, feasible:tilt | pid(off=0.31, hs=0.32, tilt=1.9), waypoint(off=0.08, hs=0.35, tilt=2.4), feasible(off=0.09, hs=0.34, tilt=9.2) |
| hard | 11 | pid:ok, waypoint:tilt, feasible:tilt | pid(off=0.45, hs=0.21, tilt=7.5), waypoint(off=0.37, hs=0.42, tilt=13.7), feasible(off=0.43, hs=0.28, tilt=13.3) |
| hard | 13 | pid:ok, waypoint:offset+tilt, feasible:hspeed+tilt | pid(off=0.22, hs=0.41, tilt=0.3), waypoint(off=0.62, hs=0.42, tilt=13.4), feasible(off=0.25, hs=0.92, tilt=10.4) |
| hard | 14 | pid:ok, waypoint:offset+hspeed, feasible:ok | pid(off=0.11, hs=0.50, tilt=3.4), waypoint(off=0.57, hs=0.70, tilt=5.5), feasible(off=0.32, hs=0.18, tilt=1.7) |
| hard | 22 | pid:tilt, waypoint:ok, feasible:tilt | pid(off=0.29, hs=0.39, tilt=8.1), waypoint(off=0.27, hs=0.43, tilt=7.0), feasible(off=0.25, hs=0.21, tilt=9.6) |
| hard | 24 | pid:hspeed, waypoint:ok, feasible:hspeed | pid(off=0.12, hs=0.52, tilt=1.7), waypoint(off=0.22, hs=0.32, tilt=4.7), feasible(off=0.15, hs=0.53, tilt=3.9) |
| hard | 27 | pid:ok, waypoint:offset+tilt, feasible:offset+hspeed | pid(off=0.33, hs=0.48, tilt=4.4), waypoint(off=0.65, hs=0.16, tilt=9.3), feasible(off=0.51, hs=0.55, tilt=2.6) |
| hard | 28 | pid:tilt, waypoint:ok, feasible:ok | pid(off=0.47, hs=0.41, tilt=8.3), waypoint(off=0.29, hs=0.49, tilt=5.5), feasible(off=0.30, hs=0.50, tilt=6.3) |
