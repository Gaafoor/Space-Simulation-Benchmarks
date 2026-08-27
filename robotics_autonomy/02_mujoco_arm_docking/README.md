# 02 — Basilisk + MuJoCo: arm docking with capture

## What this is

Chaser (Hub 2) carries a four-link impedance arm. The tip approaches Hub 1 through mesh docking interfaces (receptacle + probe). Capture arms a weld when dock-site gap ≤ 0.008 m and hub–arm contact force ≥ 0.05 N (weld at t = 1.34 s). A short LEO co-orbit segment includes J2, Moon/Sun, drag, and SRP (common-mode on a few-second docking timeline).

## Official example this started from

**Basilisk** `examples/mujoco/scenarioSimpleDocking.py` and `sats_dock.xml` / `sats_cubesat_arm_docking.xml`. The official docking demo uses primitive geoms and a weld. It does not use custom probe/receptacle meshes, LEO environment modules, per-contact logging in a hub-centered frame, or the capture criteria used here.

## What changed

- `scenarioArmDockingRealistic.py` + `sats_cubesat_arm_docking_realistic.xml` + `scripts/meshes/*.obj`.
- Approach 0.5 N / 0.76 s, brake −1.0 N / 0.30 s, then capture search.
- Per-contact forces and free-joint wrench; weld `dock_capture`.
- Relative motion evaluated in a Hub 1-centered frame (LEO inertial centimetre-scale contact is unreliable).

## How to run

Keep `meshes/` next to the XML (`compiler meshdir="meshes"`):

```bash
cd scripts
python3 scenarioArmDockingRealistic.py
```

Run from a Basilisk MuJoCo examples tree if imports fail.

## Results in this repo

`results/scenarioArmDockingRealistic_results.json` and arm-docking PNGs.

| Quantity | Value |
|----------|-------|
| Peak hub–arm free-joint wrench | 5.84 N at t = 1.341 s |
| Peak mjData hub–arm contact | 7.30 N |
| Capture weld | yes, t = 1.34 s |
| Final dock-site gap | ~6×10⁻⁷ m |
