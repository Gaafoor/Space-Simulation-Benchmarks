# 05 — Basilisk rendezvous (`scenarioRendezVous`)

Servicer and debris in LEO. Attitude: Hill pointing → debris pointing → sun pointing. Relative orbit: `relOrbDrift`, `relativeEllipse`.

## Original files

- `examples/scenarioRendezVous.py` — servicer + debris, three ADCS phases, relative ellipses

## Changed

- Copied `scripts/scenarioRendezVous.py` from Basilisk examples (official, unchanged)
- Evaluation package: attitude error, RW torque/speed, rate tracking, Vizard stills/video
- `results/run_manifest.json`

## Run

```bash
cd scripts
python3 scenarioRendezVous.py
```

Or from your Basilisk install: `basilisk/examples/scenarioRendezVous.py`.

## Results

- `results/bsk_rdv_*.png`, `results/rendezvous_vizard.mp4`, `run_manifest.json`
