# Benchmark A2 - GMAT vs Basilisk

Frozen scenario: 500 km SSO-like LEO, J2 on, Madrid 10 deg min elevation, 24 h.

## Gravity model policy (Path B — no harmonization yet)

| Tool | J2 implementation |
|------|-------------------|
| GMAT | Bundled **JGM-2**, degree 2, order 0 (`JGM2.cof`) |
| Basilisk | **GGM03S J2-only**, max degree 2 (`GGM03S-J2-only.txt`) |

**GMAT uses bundled JGM-2; Basilisk uses GGM03S J2-only; propagation compare is indicative only.**

Mission metrics (Madrid passes, revisit, eclipse) remain **valid for tool comparison** — they depend mainly on geometry and event definitions, not m-level state agreement.

State propagation cross-check (`a2_cross_tool_state_comparison.csv`) is **informative only** until both tools share the same harmonics file.

## Mission metrics

| Metric | GMAT A2 | Basilisk A2 |
|--------|---------|-------------|
| Pass count | 4 | 4 |
| Avg pass duration [min] | 5.53 | 5.458333333333333 |
| Min revisit [min] | 94.18 | 94.16666666666667 |
| Eclipse time [%] | 14.83 (total events) | 15.24305555555555 (shadow threshold) |
| GMAT eclipse (Umbra only) [%] | 13.37 | - |

## State propagation (same 600 s samples)

- Epochs compared: 145
- Max |r_GMAT - r_Basilisk| [m]: 146.368
- Max |v_GMAT - v_Basilisk| [m/s]: 0.164142
- |r_GMAT - r_Basilisk| at t=86400 s [m]: 139.390
- Detail: `a2_cross_tool_state_comparison.csv`

## Interpretation

Pass counts and durations align closely — suitable for **tool comparison** on mission analysis. Eclipse percentage definitions differ (GMAT SPICE Umbra/Penumbra event report vs Basilisk `shadowFactor < 0.999`). Orbit separation (~100–150 m at 600 s samples) reflects different J2 gravity files and integrators; treat `a2_cross_tool_state_comparison.csv` as **indicative propagation drift**, not a failed scenario.

## Artifacts

- `gmat_a2/gmat_a2_summary.json`
- `gmat_a2/gmat_a2_madrid_passes.csv`
- `a2_mission_metrics_comparison.csv`
- `a2_cross_tool_state_comparison.csv`
- Benchmark A interim conclusion: `BENCHMARK_A_INTERIM_CONCLUSION.md`
- Closure: `docs/benchmarking/checklists/A2_CLOSURE_CHECKLIST.md`
