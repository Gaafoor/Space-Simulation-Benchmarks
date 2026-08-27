# 03 — Mission analysis with Madrid contacts and eclipse windows

## What this is

24 h operational access and eclipse for a LEO spacecraft and a **Madrid** ground station: pass count, mean pass duration, minimum revisit, eclipse time fraction. GMAT and Basilisk are the strict comparison pair; FreeFlyer is treated as consistency evidence (different mission context).

Reported 24 h metrics (GMAT / Basilisk / FreeFlyer): 4 / 4 / 4 Madrid passes; eclipse ~14.8 / 15.2 / 14.6 %.

## Official example this started from

- **GMAT** `samples/` ContactLocator / eclipse-interval missions and ground-station examples. Stock scripts typically use a generic station (or GSFC-like defaults), not a frozen Madrid site with a shared 24 h window and a comparison CSV.
- **Basilisk** `examples/scenarioGroundLocationImaging.py`, `scenarioGroundTracks.py`, `scenarioSpacecraftLocation.py`. Those demos show access geometry; they do not produce the Madrid pass table used here.

## What changed

- Shared mission config in `a2_mission_config.py` (epoch, orbit, Madrid lat/lon, min elevation).
- GMAT script `benchmark_a2_gmat.script` with Madrid contact and eclipse reports.
- Basilisk runner that writes pass tables, timeline, and summary JSON.
- Side-by-side `a2_mission_metrics_comparison.csv`.

## How to run

```bash
cd scripts
python3 run_basilisk_a2_mission_analysis.py
python3 run_gmat_a2_mission_analysis.py
```

Run `scripts/gmat/benchmark_a2_gmat.script` in GMAT to produce contact/eclipse text reports if the post-processor expects them.

## Results in this repo

- `results/A2_COMPARISON_REPORT.md`, `A2_PARAMETER_TABLE.md`
- `results/a2_mission_metrics_comparison.csv`
- Pass CSVs and summaries under `results/basilisk/` and `results/gmat/`
