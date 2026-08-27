# 04 — NASA 42 docking (`Demo_docking`)

## What this is

CubeSat face-to-face approach with `BodyBodyContactFrcTrq` (polygon-centroid penalty: pressure, viscosity, friction on the OBJ mesh). Observed: contact can fire before faces look closed; rebound was often oblique (~90° from the approach line). Thruster/attitude alignment sets closing speed.

## Official example this started from

**NASA 42** `Demo/` docking / two-spacecraft proximity (chaser + target CubeSats, `SC_Chaser_Cube.txt`, `SC_Target_Cube.txt`, contact enabled in `Inp_Sim.txt`). The stock Demo is a visual multi-body showcase with default `PressCoef` / `ViscCoef` / `FricCoef` and default `DTSIM`.

## What changed

- Tuned `Inp_Sim.txt`: contact ON, reduced `DTSIM`, pulsed thrusters (`PulseWidthCmd`), contact-normal guards against zero-vector `UNITV` warnings.
- Alignment trials: Euler–Hill short approach; thruster axis vs relative velocity.
- Relative-motion exports and plots (`nasa42_relative.png`, ~90° rebound screenshot).
- Only **custom/needed decks** are stored here. Copy them on top of a full 42 `Demo` tree (maps, remaining `Orb_*.txt` listed in `Inp_Sim.txt` as TRUE still need the stock 42 files, or disable unused orbs).

## How to run

```bash
export NASA42_ROOT=/path/to/42
# Overlay scripts/ onto a copy of Demo, or pass this folder as InOut
cd "$NASA42_ROOT"
./42 /path/to/robotics_autonomy/04_nasa42_docking/scripts
```

Graphics: `Inp_Graphics.txt` with the 42 OpenGL front end.

## Results in this repo

`results/run_manifest.json`, `nasa42_relative.png`, `nasa42_rebound_view.png`, `angled90rebound.png`.

Basilisk+MuJoCo vs 42 contact comparison: internship report §2.5.
