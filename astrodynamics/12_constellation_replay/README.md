# 12 — Constellation divergence and replay workflows

## What this is

Five-spacecraft propagation with **synchronized case progression** and **fixed ICs restored before each force-model case**, so divergence is attributable to the perturbation ladder rather than mismatched starts. GMAT, Basilisk, and FreeFlyer implement the same benchmarking intent with different executives. Common reference epoch: **02 Jun 2026 00:00:00 UTC**.

Basilisk constellation: 5 spacecraft, 72° phase spacing. FreeFlyer: 5-case ladder, 24 h default, 10 s dynamics, 600 s state export. Replay: archived state histories / visualization sessions (Vizard `.bin` files are **not** in this repo).

## Official example this started from

- **GMAT** constellation / formation samples (multiple `Spacecraft` + one propagate). Stock scripts do not reset ICs across a five-case physics ladder for five vehicles.
- **Basilisk** `examples/scenarioSatelliteConstellation.py`, `MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat.py`. Official multi-sat demos fly one force model, not the A0 case ladder.
- **FreeFlyer** constellation / formation mission plans.

## What changed

- GMAT `Benchmark_A0_Constellation_PhysicsFidelity.script` (+ 1-day smoke test).
- Basilisk `run_basilisk_a0_physics_fidelity.py` with constellation mission type.
- FreeFlyer mission plan reused from the B1 ladder; constellation screenshots `a0_freeflyer_constellation_*.png`.
- Optional FreeForm attitude snippets under `scripts/freeflyer/freeforms/` (milestone visualization, not the 5-sc ladder).

## How to run

```bash
# Basilisk constellation ladder
cd scripts/basilisk
python3 run_basilisk_a0_physics_fidelity.py
```

GMAT: open `scripts/gmat/Benchmark_A0_Constellation_PhysicsFidelity.script`.  
FreeFlyer: `scripts/freeflyer/benchmark_b1_physics_fidelity.MissionPlan` (and GUI constellation views as in the report).

## Results in this repo

- `results/a0_constellation_summary.json`, `A0_METRICS_COMPARISON.csv`, `run_log.md`
- Early/late FreeFlyer constellation PNGs
