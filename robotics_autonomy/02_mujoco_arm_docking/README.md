# 02 — Basilisk + MuJoCo: arm docking with capture

## What this is

Chaser (Hub 2) carries a four-link impedance arm. The tip approaches Hub 1 through mesh docking interfaces (receptacle + probe). Capture arms a weld when dock-site gap ≤ 0.008 m and hub–arm contact force ≥ 0.05 N (weld at t = 1.34 s). A short LEO co-orbit segment includes J2, Moon/Sun, drag, and SRP (common-mode on a few-second docking timeline).

## Official upstream (AVSLab Basilisk)

| Official file | Role |
|---------------|------|
| `examples/mujoco/scenarioSimpleDocking.py` + `sats_dock.xml` | Two CubeSat hubs; primitive geoms; **weld equality**; contact disabled |
| `examples/mujoco/scenarioArmWithThrusters.py` + `sat_w_deployable_thruster.xml` | Branching 4-link deployable arm with tip thrusters |

There is **no** `scenarioArmDockingRealistic.py`, `sats_cubesat_arm_docking.xml`, or probe/receptacle OBJ meshes in upstream Basilisk.

## Files in this repo (custom)

| File | Status |
|------|--------|
| `scripts/scenarioArmDockingRealistic.py` | **Custom** — LEO co-orbit, arm impedance, capture logic, per-contact logging |
| `scripts/sats_cubesat_arm_docking_realistic.xml` | **Custom** — mesh docking interfaces, extra contact geoms, inactive weld |
| `scripts/meshes/docking_probe.obj`, `docking_receptacle.obj` | **Custom** — docking interface meshes |

Inspired by the official simple-docking and deployable-arm demos, but implemented as a new scenario rather than a patch to `scenarioSimpleDocking.py`.

## What changed vs official Basilisk

- Contact **enabled** with mesh probe/receptacle and pad/ring geoms (official docking disables contact and uses a weld from the start).
- Short LEO environment: J2, SPICE third bodies, drag, SRP on both hubs.
- Approach 0.5 N / 0.76 s, brake −1.0 N / 0.30 s, then capture search.
- Per-contact forces and free-joint wrench; weld `dock_capture` armed after criteria met.
- Relative motion evaluated in a Hub 1-centered frame.

## How to run

Keep `meshes/` next to the XML (`compiler meshdir="meshes"`):

```bash
cd scripts
python3 scenarioArmDockingRealistic.py
```

Run from a Basilisk MuJoCo examples tree if imports fail.

## Results in this repo

`results/scenarioArmDockingRealistic_results.json`, arm-docking PNGs, and demo MP4s (`arm_docking_demo_*.mp4`).

| Quantity | Value |
|----------|-------|
| Peak hub–arm free-joint wrench | 5.84 N at t = 1.341 s |
| Peak mjData hub–arm contact | 7.30 N |
| Capture weld | yes, t = 1.34 s |
| Final dock-site gap | ~6×10⁻⁷ m |
