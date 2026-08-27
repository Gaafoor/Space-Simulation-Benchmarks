# Benchmark A1 — Basilisk vs GMAT vs Analytical Reference

Independent Kepler two-body reference (WGS84, 500 km SSO-like LEO, 24 h, 600 s samples).

**Status: A1 closed for Basilisk + GMAT (2026-06-03). STK deferred.**

See also: `A1_PARAMETER_TABLE.md`, `docs/benchmarking/checklists/A1_CLOSURE_CHECKLIST.md`

## Summary

| Metric | Basilisk | GMAT |
|--------|----------|------|
| Max position error vs reference [m] | 0.441 | 0.249 |
| Final position error vs reference [m] | 0.441 | 0.249 |
| Max velocity error vs reference [m/s] | 4.89×10⁻⁴ | 2.75×10⁻⁴ |
| Wall-clock runtime [s] | 0.247 (sim only) | 0.829 (GUI + plots) |
| Runtime ratio (sim/wall) | ~349294 | ~104222 |
| Samples | 145 | 145 |
| A1 result | pass | pass |

## Cross-tool (same epochs)

- Epochs compared: 145
- Max /|r_GMAT − r_Basilisk/| [m]: 0.689
- Max /|v_GMAT − v_Basilisk/| [m/s]: 7.65×10⁻⁴
- At t = 86400 s: GMAT − Basilisk position = 0.689 m

## STK

| Item | Status |
|------|--------|
| STK A1 run | **Deferred** — STK not installed |
| Impact on A1 closure | Basilisk + GMAT sufficient for A1 sign-off per variant scope |

## Artifacts

| Artifact | Path |
|----------|------|
| Parameter table | `A1_PARAMETER_TABLE.md` |
| Basilisk vs reference | `basilisk_a1/basilisk_a1_state_comparison.csv` |
| GMAT vs reference | `gmat_a1/gmat_a1_state_comparison.csv` |
| Three-way table | `a1_three_way_comparison.csv` |
| GMAT vs Basilisk | `a1_cross_tool_comparison.csv` |
| Evaluation sheets | `basilisk_a1/basilisk_a1_evaluation_sheet.csv`, `gmat_a1/gmat_a1_evaluation_sheet.csv` |
| Run logs | `docs/benchmarking/logs/a1_basilisk_2026-06-03.md`, `a1_gmat_2026-06-03.md` |
| Closure checklist | `docs/benchmarking/checklists/A1_CLOSURE_CHECKLIST.md` |

## Interpretation

Both simulators are compared to the same analytical Kepler propagator, not to each other as truth. Sub-meter agreement with the reference over 24 h satisfies A1. GMAT–Basilisk separation (~0.7 m) reflects integrator and gravity-model differences (JGM2 degree 0 vs point-mass μ), not a failed run.

**Do not use A1 scores as full Benchmark A weighted totals** — mission-analysis categories apply in A2.

## Next step

Benchmark **A2**: J2 enabled, Madrid target, passes / revisit / eclipse (Basilisk + GMAT).
