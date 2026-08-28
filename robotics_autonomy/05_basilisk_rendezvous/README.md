# 05 — Basilisk rendezvous (`scenarioRendezVous`)

## What this is

Servicer and debris in LEO. Attitude guidance switches Hill pointing → debris pointing → sun pointing. Relative orbit uses scripted Hill-frame velocity sets (`relOrbDrift`, `relativeEllipse`). This benchmark **ran the official example** and archived ADCS/Vizard evidence for the tool comparison.

## Official upstream (AVSLab Basilisk)

| Official file | Role |
|---------------|------|
| `examples/scenarioRendezVous.py` | Servicer + debris; Hill / debris / sun pointing; relative drift and ellipse phases |

This scenario **exists unchanged** in upstream Basilisk. No custom scenario file is stored in this repo.

## Files in this repo

| Content | Status |
|---------|--------|
| `results/bsk_rdv_*.png`, Vizard screenshot | **Custom** — report artifacts from an official run |
| `results/run_manifest.json` | **Custom** — run metadata |

## What changed vs official Basilisk

Nothing in the scenario source. What was added is the evaluation package: attitude error, RW torque/speed, rate tracking, and Vizard stills used in the report.

If you later patch the official script (logging paths, run length), keep those diffs next to `results/` rather than copying the whole upstream file.

## How to run

```bash
cd /path/to/basilisk/examples
python3 scenarioRendezVous.py
```

## Results in this repo

`results/bsk_rdv_*.png`, Vizard screenshot, `run_manifest.json`.
