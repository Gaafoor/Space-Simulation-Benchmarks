# 05 — Basilisk rendezvous (`scenarioRendezVous`)

## What this is

Servicer and debris in LEO. Attitude guidance switches Hill pointing → debris pointing → sun pointing. Relative orbit uses scripted Hill-frame velocity sets (`relOrbDrift`, `relativeEllipse`). This benchmark **ran the official example** and archived ADCS/Vizard evidence for the tool comparison.

## Official example this started from

**Basilisk** `examples/scenarioRendezVous.py` (AVS Lab). Phases: along-track drift (~1/8 orbit), drifting circumnavigation (~2 orbits), 2:1 bounded park (~1/2 orbit).

## What changed

The scenario file is **not** forked in this repo (keep using your Basilisk tree). What was added is the evaluation package: attitude error, RW torque/speed, rate tracking, and Vizard stills used in the report.

If you later patch the official script (logging paths, run length), keep those diffs next to `results/` rather than copying the whole upstream file.

## How to run

```bash
cd /path/to/basilisk-develop/examples
python3 scenarioRendezVous.py
```

## Results in this repo

`results/bsk_rdv_*.png`, Vizard screenshot, `run_manifest.json`.
