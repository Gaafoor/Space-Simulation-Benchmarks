# 01 — Basilisk + MuJoCo: hub-to-hub chassis rebound

## What this is

Two 1U buses face to face. An open-loop thruster approach/brake sequence drives chassis geoms into contact; post-impact motion is MuJoCo contact only. Internal wheels and tanks stay at zero command. Peak contact ~252 N, impact ~3 ms, estimated restitution 1.07 (uncalibrated soft contact).

## Official upstream (AVSLab Basilisk)

There is **no** `scenarioCubeSatContactRebound.py` or `sats_cubesat_contact.xml` in the public Basilisk `examples/mujoco/` tree. The closest official demos are:

| Official file | Role |
|---------------|------|
| `examples/mujoco/scenarioUnbalancedThrusters.py` + `sat_w_thrusters.xml` | 1U CubeSat with four thrusters and tank bodies |
| `examples/mujoco/scenarioReactionWheel.py` + `sat_w_wheel.xml` | 1U CubeSat with a reaction wheel |
| `examples/mujoco/scenarioSimpleDocking.py` + `sats_dock.xml` | Two CubeSat hubs; **contact disabled**, docking via MuJoCo weld equality |

Those examples do not run a hub-to-hub impact/rebound test, log contact impulse/restitution, or use the contact XML in this repo.

## Files in this repo (custom)

| File | Status |
|------|--------|
| `scripts/scenarioCubeSatContactRebound_realistic.py` | **Custom** — contact-only rebound with JSON archive |
| `scripts/sats_cubesat_contact_realistic.xml` | **Custom** — 1U buses with RW + four tank sub-bodies, contact enabled |

Built by combining patterns from the official CubeSat/thruster/RW MuJoCo examples above into a new scenario focused on calibrated soft-contact rebound metrics.

## What changed vs official Basilisk

- New scenario + XML (not a fork of a named upstream script).
- Approach 1.0 N for 0.72 s, brake −2.0 N for 0.18 s, then hold.
- Contact: `solref = (0.01, 0.1)`, `solimp = (0.99, 0.99, 0.001)`.
- Logged peak force, impulse, pre/post relative speed, site separation → `scenarioCubeSatContactReboundRealistic_results.json`.

## How to run

Requires Basilisk built with MuJoCo. Copy the script and XML into `basilisk/examples/mujoco/` (or set `PYTHONPATH`) so Basilisk example imports resolve:

```bash
cd scripts
python3 scenarioCubeSatContactRebound_realistic.py
```

## Results in this repo

`results/scenarioCubeSatContactReboundRealistic_results.json` and hub-hub PNGs.

| Quantity | Value |
|----------|-------|
| Peak contact force | 252.3 N at t = 0.922 s |
| Contact impulse | 0.396 N·s |
| Pre / post relative speed | −0.810 / +0.863 m/s |
| Estimated restitution | 1.07 |
