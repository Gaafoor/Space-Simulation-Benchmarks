# 05 — LEO-to-GEO electric-transfer trajectory design

## What this is

GMAT low-thrust electric spiral from LEO (SMA 6878.137 km) to a GEO-size SMA target (42164 km). Event-based convergence on **SMA only**; residual eccentricity (~0.076) and inclination (~28.3°) are left for later orbit shaping. Elapsed time to SMA target: **52.477 days**. Propellant used: **3.766 kg** (start mass 17 kg).

This folder owns the **design** scripts and the exported burn schedule. Execution in Basilisk / NASA 42 is [06](../06_transfer_execution/).

## Official example this started from

- **GMAT** electric-propulsion / finite-burn / CSALT samples (for example low-thrust transfer scripts under `samples/` and `samples/CSALT/`). Official examples typically demonstrate a thruster + targeter on a shorter or different transfer, with constant power assumptions, not this 17 kg CubeSat-class bus, Madrid-era epoch, SMA-only stop, and handoff JSON/CSV for other tools.

## What changed

- Spacecraft, tank, EP polynomials, and solar power system sized for the A3 bus (`Benchmark_A3_LEO_GEO_Transfer.script`).
- SMA-target stopping condition and fuel accounting over ~52.5 days.
- Export path: `burn_schedule.json` + `reference_ephemeris.csv` via `export_gmat_handoff.py` for Basilisk and NASA 42.
- Optional optimizer script `Benchmark_A3_LEO_GEO_Opt.script`.

## How to run

Open in GMAT R2026a:

```text
scripts/gmat/Benchmark_A3_LEO_GEO_Transfer.script
```

Then, from this folder or from [06](../06_transfer_execution/scripts/basilisk/):

```bash
python3 export_gmat_handoff.py
```

(Adjust paths inside the exporter to your GMAT output directory.)

## Results in this repo

| Design metric | Value |
|---------------|-------|
| Initial / target SMA [km] | 6878.137 / 42164.000 |
| Time to SMA target [days] | 52.477 |
| Mass start / at target [kg] | 17.000 / 13.234 |
| Propellant [kg] | 3.766 |
| e / i at SMA target | 0.0765 / 28.31° |

- `results/burn_schedule.json`, `reference_ephemeris.csv`, `summary.json`
- `results/a3_leogeo_transfer_spiral.png`
