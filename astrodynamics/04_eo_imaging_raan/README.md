# 04 — Earth-observation imaging opportunity analysis

## What this is

Nadir-pointing camera (10° full cone) over a Madrid regional box. A **RAAN sweep** selects the operating point that satisfies the imaging rule. At **RAAN = 61°**, GMAT, Basilisk, and Orekit agree on opportunity count (1 or 2 depending on tool counting) and off-nadir geometry (~2.1–2.2°), with ~67% regional coverage.

## Official example this started from

- **Basilisk** `examples/scenarioGroundLocationImaging.py` and `scenarioStripImaging.py`. Official demos image a ground location or strip from **one** orbit orientation. They do not sweep RAAN, score a coverage grid, or match GMAT/Orekit camera FOV.
- **GMAT** coverage / FOV / ContactLocator samples.
- **Orekit** field-of-view and `GeographicZoneDetector` tutorials.

## What changed

- Shared camera and target-box geometry in `a3_imaging_geometry.py`.
- RAAN trade (`results/raan_trade_study.csv` and `raan_sweep/*/summary.json`); selected case archived in `results/RAAN_061/`.
- Cross-tool `a3_imaging_metrics_comparison.csv` (opportunity count, min off-nadir, coverage).
- Footprint, heatmap, and ground-track plots for the selected RAAN.

## How to run

```bash
cd scripts
python3 run_basilisk_a3_imaging_analysis.py
python3 run_orekit_a3_imaging_analysis.py
python3 run_gmat_a3_analysis.py
```

GMAT script: `scripts/gmat/benchmark_a3_gmat.script`.

## Results in this repo

- `results/A3_COMPARISON_REPORT.md`, `A3_PARAMETER_TABLE.md`, `raan_trade_study.png`
- Selected-case plots in `results/RAAN_061/` and per-tool folders
- Other RAAN JSON summaries in `results/raan_sweep/` (plots omitted to keep the repo small)
