# 02 — Force-model fidelity build-up (perturbation ladder)

## What this is

The same LEO initial state is propagated with a **five-case force ladder**: (1) two-body, (2) J2, (3) J2+drag, (4) J2+drag+SRP, (5) J2+drag+SRP+third-body. Incremental drift, element deltas, and runtime are compared across GMAT, Basilisk, Orekit, and FreeFlyer.

Over 24 h, J2 is the dominant jump from two-body; drag, SRP, and third-body add smaller increments at this altitude. The three physics engines follow the same trend.

## Official example this started from

- **GMAT** force-model / Earth gravity samples (JGM-2, drag, SRP objects in `samples/`). Each sample usually enables a **fixed** force set for one mission, not a synchronized five-case ladder with identical ICs restored every time.
- **Basilisk** `scenarioBasicOrbit.py` plus separate examples for drag, SRP, or spherical harmonics. Official examples do not run Cases 1–5 in one driver with shared metrics.
- **Orekit** tutorials for `NumericalPropagator` and force models.
- **FreeFlyer** default mission / force-model demos.

## What changed

- Frozen orbit, spacecraft, and atmosphere parameters in `b1_physics_config.py` / `a0_physics_config.py`.
- One runner per tool that loops Cases 1–5 and writes per-case `summary.json`.
- Cross-tool incremental-drift plots (`b1_drift_cross_tool_24h.png`).
- GMAT scripts `Benchmark_A0_PhysicsFidelity.script` and `benchmark_b1_gmat.script`; FreeFlyer `benchmark_b1_physics_fidelity.MissionPlan`.

## How to run

```bash
cd scripts
python3 run_basilisk_b1_physics_fidelity.py
python3 run_orekit_b1_physics_fidelity.py
python3 run_gmat_b1_physics_fidelity.py   # post-process GMAT CSV exports
```

GMAT: open `scripts/gmat/benchmark_b1_gmat.script` (or the A0 physics script) in GMAT. FreeFlyer: `ff -mp scripts/freeflyer/benchmark_b1_physics_fidelity.MissionPlan` (headless flags as in your FreeFlyer install).

## Results in this repo

- `results/B1_COMPARISON_REPORT.md`, `B1_PARAMETER_TABLE.md`, `B1_METRICS_COMPARISON.csv`
- Per-case `summary.json` under `results/{basilisk,gmat,orekit}/`
- Drift plots in `results/`
