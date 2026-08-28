# 03 — Basilisk + MuJoCo: thruster-arm control

## What this is

Two cases on a 6-DOF hub with two 4-DOF thruster arms (`sat_w_thruster_arms.xml`):

1. **Fixed-offset hover** — `scenarioThrArmControl.py`, 320 s, hub reaction-torque compensation on. Position settles in ~5 min.
2. **Moving inspection path** — `ThrArmPaper.py` tracks ρ*(t) in the YZ plane (amplitude 2 m, period 28800 s). Last-60 s RMS tracking 0.055 m; ~162 arm reconfigurations over 8 h.

## Official upstream (AVSLab Basilisk)

| Official file | Role |
|---------------|------|
| `examples/mujoco/scenarioThrArmControl.py` | 6-DOF hub + two 4-DOF thruster arms; fixed-offset hover |
| `examples/mujoco/sat_w_thruster_arms.xml` | MuJoCo model for the hover demo |

Upstream includes **only** the hover case. There is no `ThrArmPaper.py` or `sat_w_thruster_arms_paper.xml` in official Basilisk.

## Files in this repo

| File | Status |
|------|--------|
| `scripts/scenarioThrArmControl.py` | **Copied from official** (hover demo) |
| `scripts/sat_w_thruster_arms.xml` | **Copied from official** |
| `scripts/ThrArmPaper.py` | **Custom** — long inspection-path scenario (Schwend & Schaub 2026 style) |
| `scripts/sat_w_thruster_arms_paper.xml` | **Custom** — XML variant for the inspection path |

## What changed vs official Basilisk

- Hover: documented gains/timing and CoM/MRP plots for the default 320 s run (official demo only; metrics archived here).
- **New** 8 h inspection path with τ_pre disabled, full-period tracking metrics, path-span and joint-reconfiguration counts.
- Results plots for both phases (`bsk_thrarm_*.png`, `real_ThrArm_*.png`).

## How to run

```bash
cd scripts
python3 scenarioThrArmControl.py    # hover (~5 min wall time)
python3 ThrArmPaper.py              # long path (~56 min wall time for one period)
```

Place the XML files beside the scripts in a Basilisk MuJoCo examples directory if needed.

## Results in this repo

Hover and inspection PNGs under `results/`, plus `run_manifest.json`.
