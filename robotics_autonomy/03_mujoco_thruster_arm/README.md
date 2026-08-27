# 03 — Basilisk + MuJoCo: thruster-arm control

## What this is

Two cases on a 6-DOF hub with two 4-DOF thruster arms (`sat_w_thruster_arms.xml`):

1. **Fixed-offset hover** — `scenarioThrArmControl.py`, 320 s, hub reaction-torque compensation on. Position settles in ~5 min.
2. **Moving inspection path** — `ThrArmPaper.py` tracks ρ*(t) in the YZ plane (amplitude 2 m, period 28800 s). Last-60 s RMS tracking 0.055 m; ~162 arm reconfigurations over 8 h.

## Official example this started from

**Basilisk** `examples/mujoco/scenarioThrArmControl.py` and `sat_w_thruster_arms.xml` (AVS Lab). The official hover demo is the starting point. This benchmark **keeps that hover** and adds the long inspection-path script `ThrArmPaper.py` / `sat_w_thruster_arms_paper.xml` with τ_pre disabled, full-period tracking metrics, and report plots.

## What changed

- Documented hover gains/timing and CoM/MRP plots for the default 320 s run.
- 8 h inspection path (wall ~56 min) with path-span and joint-reconfiguration counts.
- Results plots for both phases (`bsk_thrarm_*.png`, `real_ThrArm_*.png`).

## How to run

```bash
cd scripts
python3 scenarioThrArmControl.py    # hover
python3 ThrArmPaper.py              # long path; hours of wall time
```

Place the XML files beside the scripts in a Basilisk MuJoCo examples directory if needed.

## Results in this repo

Hover and inspection PNGs under `results/`, plus `run_manifest.json` from the robotics experiment folder.
