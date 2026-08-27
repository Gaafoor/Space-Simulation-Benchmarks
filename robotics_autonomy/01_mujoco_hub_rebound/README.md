# 01 — Basilisk + MuJoCo: hub-to-hub chassis rebound

## What this is

Two 1U buses face to face. An open-loop thruster approach/brake sequence drives chassis geoms into contact; post-impact motion is MuJoCo contact only. Internal wheels and tanks stay at zero command. Peak contact ~252 N, impact ~3 ms, estimated restitution 1.07 (uncalibrated soft contact).

## Official example this started from

**Basilisk** `examples/mujoco/scenarioCubeSatContactRebound.py` and `sats_cubesat_contact.xml`. The official demo collides two simple chassis geoms. It does not add Basilisk-style RW and tank sub-bodies, the realistic XML, or the JSON contact-force archive used in the report.

## What changed

- `scenarioCubeSatContactRebound_realistic.py` + `sats_cubesat_contact_realistic.xml`.
- Approach 1.0 N for 0.72 s, brake −2.0 N for 0.18 s, then hold.
- Contact: `solref = (0.01, 0.1)`, `solimp = (0.99, 0.99, 0.001)`.
- Logged peak force, impulse, pre/post relative speed, site separation.

## How to run

Requires Basilisk built with MuJoCo. Copy the script and XML into `basilisk/examples/mujoco/` (or set Python path) so Basilisk example imports resolve:

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
