# Failure Diagnosis

Date: 2026-05-28

This report logs per-seed landing outcomes. It is meant to answer where each controller fails, not only the aggregate success rate.

## Detailed Trace Logs

Per-step trajectory CSV logs are saved by default under `out/failure_traces`.

One file is written per difficulty/controller/mode/plant-model/seed, with the final success/fail status in the filename. Each row includes commanded throttle/gimbal, applied gimbal, guidance/MPC debug fields, and true/measured/estimated/controller-input state snapshots.

## Summary

| difficulty | controller | success | reasons | primary failures | touchdown mean |
|---|---|---:|---|---|---|
| hard | feasible | 1/2 (50%) | {'touchdown': 2} | {'tilt': 1} | offset 0.30, vspeed 0.09, hspeed 0.18, tilt 9.0 |
| hard | pid | 0/2 (0%) | {'touchdown': 2} | {'tilt': 2} | offset 0.35, vspeed 0.08, hspeed 0.28, tilt 9.2 |

## Seeds Where Controllers Disagree

| difficulty | seed | outcome pattern | details |
|---|---:|---|---|
| hard | 1 | pid:tilt, feasible:ok | pid(off=0.33, hs=0.27, tilt=8.5), feasible(off=0.31, hs=0.13, tilt=7.7) |
