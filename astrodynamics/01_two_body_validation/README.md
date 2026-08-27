# 01 — Two-body orbit propagation validation

## What this is

24 h two-body propagation from a common 500 km SSO-like initial state, sampled every 600 s, compared to an independent Kepler analytical reference (WGS84 µ). Tools: GMAT and Basilisk.

Both passed sub-meter accuracy vs the reference (GMAT 0.249 m max position error, Basilisk 0.441 m). Max GMAT–Basilisk position difference at the same epochs: 0.689 m.

## Official example this started from

- **GMAT** `samples/` two-body / default propagate missions (for example `Ex_*.script` Kepler propagate). Stock samples propagate one spacecraft and plot; they do not freeze a shared IC set or score against an external analytical ephemeris.
- **Basilisk** `examples/scenarioBasicOrbit.py`. Demonstrates a point-mass Earth orbit in Python. It does not export a 145-sample comparison table against Kepler truth or against GMAT.

## What changed

- Frozen common epoch, Keplerian elements, and 600 s output grid for both tools.
- Independent Kepler reference in `a1_analytical_reference.py` (not “GMAT vs Basilisk as truth”).
- Cross-tool CSV at identical epochs; pass/fail on sub-meter position error.
- GMAT mission script `scripts/gmat/benchmark_a1_gmat.script` written for this IC set rather than a renamed sample.

## How to run

Requires a local GMAT install and a Basilisk Python environment.

```bash
cd scripts
python3 run_basilisk_a1_analytical_validation.py
python3 run_gmat_a1_analytical_validation.py   # after running the GMAT script, or as documented in the runner
```

Open `scripts/gmat/benchmark_a1_gmat.script` in GMAT if the runner expects a GUI export.

## Results in this repo

- `results/A1_COMPARISON_REPORT.md`, `A1_PARAMETER_TABLE.md`
- `results/a1_three_way_comparison.csv`, `a1_cross_tool_comparison.csv`
- `results/a1_two_body_position_error.png`
- Per-tool summaries under `results/basilisk/` and `results/gmat/`
