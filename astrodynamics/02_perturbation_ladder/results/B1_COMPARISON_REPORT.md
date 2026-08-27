# Benchmark B1 — Physics Fidelity Comparison Report

Status: **Closed** for GMAT, Basilisk, and Orekit (Cases 1–5). STK deferred.

Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`  
Metrics CSV: `data/benchmark_b/B1_METRICS_COMPARISON.csv`

## Executive summary

Three tools propagated the same frozen 500 km SSO-like CubeSat orbit for **7 days** under a five-case perturbation ladder (two-body → J2 → drag → SRP → all supported). Conclusions:

- **J2 dominates** long-horizon drift vs Case 1 (~3.41M m position drift; ~6.92–6.93° RAAN change over 7 days).
- **Cross-tool J2 agreement is strong:** GMAT vs Orekit final-state separation ≤ **35 m** on Case 2; Basilisk vs Orekit ≤ **1.3 km** on Case 2.
- **Drag is tool-dependent at 500 km:** Orekit (NRLMSISE-00) and GMAT (NRLMSISE-00) show **~76–85 m** extra altitude decay vs J2-only; Basilisk exponential atmosphere is **negligible** (Case 3 identical to Case 2).
- **SRP and third-body** add small increments on top of drag at this altitude (~1–3 m altitude; sub-km to ~18 km cross-tool separation depending on case).
- **Runtime:** Basilisk **5.57 s** total (five cases); Orekit **6.83 s**; GMAT **15.44 s** without live plots (**171.9 s** with orbit view + ground track).

Case 5 definitions are **tool-specific** (documented per tool); cross-tool Case 5 is compared for workflow and trend, not identical force stacks.

---

## Per-tool summaries

### Orekit (7 days)

| Case | Drag | SRP | Third-body | Position drift vs Case 1 [m] | RAAN change [deg] | Altitude change [m] | Runtime [s] |
|------|------|-----|------------|-------------------------------|-------------------|---------------------|-------------|
| 1 two-body | off | off | off | 0 | ~0 | −0.07 | 0.30 |
| 2 J2 | off | off | off | 3,408,408 | 6.93 | −8,426 | 0.16 |
| 3 J2+drag | on | off | off | 3,423,615 | 6.93 | −8,502 | 1.66 |
| 4 J2+drag+SRP | on | on | off | 3,424,405 | 6.93 | −8,504 | 2.20 |
| 5 all supported | on | on | Sun+Moon | 3,424,689 | 6.93 | −8,505 | 2.50 |

**Total pipeline runtime:** 6.83 s

Artifacts: `data/benchmark_b/b1_physics_fidelity/orekit/`  
Run log: `data/benchmark_b/b1_physics_fidelity/orekit/orekit_b1_run_log.md`  
Case 5 definition: `data/benchmark_b/b1_physics_fidelity/orekit/orekit_b1_case5_definition.md`

### Basilisk 2.10.2 (7 days)

| Case | Drag | SRP | Third-body | Position drift vs Case 1 [m] | RAAN change [deg] | Altitude change [m] | Runtime [s] |
|------|------|-----|------------|-------------------------------|-------------------|---------------------|-------------|
| 1 two-body | off | off | off | 0 | ~0 | −0.02 | 0.85 |
| 2 J2 | off | off | off | 3,408,592 | 6.92 | −8,377 | 0.90 |
| 3 J2+drag | on | off | off | 3,408,592 | 6.92 | −8,377 | 0.90 |
| 4 J2+drag+SRP | on | on | off | 3,407,215 | 6.92 | −8,421 | 1.25 |
| 5 all supported | on | on | Sun+Moon | 3,407,497 | 6.92 | −8,422 | 1.68 |

**Total pipeline runtime:** 5.57 s

Artifacts: `data/benchmark_b/b1_physics_fidelity/basilisk/`  
Run log: `data/benchmark_b/b1_physics_fidelity/basilisk/basilisk_b1_run_log.md`  
Case 5 definition: `data/benchmark_b/b1_physics_fidelity/basilisk/basilisk_b1_case5_definition.md`

**Basilisk drag note:** Case 3 matches Case 2 numerically. Exponential atmosphere at 500 km with 0.01 m² drag area produces no measurable effect in this run. Documented; not a Case 1–2 failure.

### GMAT R2026a (7 days)

| Case | Drag | SRP | Third-body | Position drift vs Case 1 [m] | RAAN change [deg] | Altitude change [m] | Runtime [s]† |
|------|------|-----|------------|-------------------------------|-------------------|---------------------|-------------|
| 1 two-body | off | off | off | 0 | ~0 | +0.00002 | 3.09 |
| 2 J2 | off | off | off | 3,408,407 | 6.93 | −8,425 | 3.09 |
| 3 J2+drag | on | off | off | 3,425,259 | 6.93 | −8,510 | 3.09 |
| 4 J2+drag+SRP | on | on | off | 3,425,367 | 6.93 | −8,509 | 3.09 |
| 5 all supported | on | on | Sun+Luna | 3,425,649 | 6.93 | −8,510 | 3.09 |

† Per-case wall clock amortized from **15.44 s** total (no live plots). With OpenFrames orbit view + ground track: **171.9 s** total.

Artifacts: `data/benchmark_b/b1_physics_fidelity/gmat/`  
Run log: `data/benchmark_b/b1_physics_fidelity/gmat/gmat_b1_run_log.md`  
Case 5 definition: `data/benchmark_b/b1_physics_fidelity/gmat/gmat_b1_case5_definition.md`  
GMAT script: `benchmarks/benchmark_b/gmat/benchmark_b1_gmat.script`

---

## Cross-tool comparison

For the position-difference table below, final-epoch states are compared to **Orekit** as a documented comparison anchor (see reference policy in `B1_PARAMETER_TABLE.md`). This is a reporting choice for cross-tool tables; **no tool is treated as ground truth.**

Values are ‖r_tool − r_Orekit‖ at final epoch (t = 604800 s).

| Case | GMAT vs Orekit [m] | Basilisk vs Orekit [m] | Interpretation |
|------|-------------------|------------------------|----------------|
| 1 two-body | 34 | 20 | Initial-condition / integrator noise only |
| 2 J2 | 35 | 1,294 | GMAT–Orekit 35 m separation; Basilisk–Orekit 1.3 km |
| 3 J2+drag | 1,716 | 16,381 | Drag model divergence (NRLMSISE vs exponential) |
| 4 J2+drag+SRP | 990 | 18,243 | SRP + prior drag differences accumulate |
| 5 all supported | 988 | 18,245 | Third-body increment small vs Case 4 |

### Perturbation increment comparison (altitude change over 7 days)

| Case step | Orekit Δalt [m] | Basilisk Δalt [m] | GMAT Δalt [m] |
|-----------|-----------------|-------------------|---------------|
| 1 → 2 (add J2) | −8,426 | −8,377 | −8,425 |
| 2 → 3 (add drag) | −76 | 0 | −85 |
| 3 → 4 (add SRP) | −3 | −44 | +1 |
| 4 → 5 (add third-body) | −1 | −1 | −1 |

Path B gravity is **not harmonized** (JGM-2 vs GGM03S-J2-only vs Orekit 2×0). In this run, GMAT–Orekit Case 2 final states differ by **35 m**, despite different gravity files.

### Same-tool drift metrics (Case 5 vs Case 1)

| Tool | Position drift [m] | RAAN change [deg] | Altitude change [m] |
|------|-------------------|-------------------|---------------------|
| Orekit | 3,424,689 | 6.926 | −8,505 |
| GMAT | 3,425,649 | 6.926 | −8,510 |
| Basilisk | 3,407,497 | 6.923 | −8,422 |

Basilisk final altitude ~83 m higher than Orekit/GMAT — consistent with absent drag decay in Cases 3–5.

---

## Force-model support (B1)

| Perturbation | GMAT | Basilisk | Orekit | STK |
|--------------|------|----------|--------|-----|
| Two-body / central gravity | Native (JGM-2 0×0) | Native (point mass) | Native (0×0 harmonics) | Deferred |
| J2 | Native (JGM-2 2×0) | Native (GGM03S J2-only) | Native (2×0 harmonics) | Deferred |
| Drag | Native NRLMSISE-00 | Native exponential† | Native NRLMSISE-00 | Deferred |
| SRP | Native spherical | Native cannonball | Native isotropic | Deferred |
| Third-body Sun/Moon | Native point mass | Native SPICE grav | Native ThirdBodyAttraction | Deferred |

† Basilisk drag enabled in Case 3+ but negligible at frozen parameters.

---

## Runtime comparison (five cases, 35 sim-days total)

| Tool | Total wall clock [s] | Notes |
|------|---------------------|-------|
| Basilisk | 5.57 | Fastest; Python runner |
| Orekit | 6.83 | Includes JVM init in Case 1 |
| GMAT | 15.44 | No live plots; all cases in one script |
| GMAT | 171.9 | With OpenFrames + ground track |

---

## Experimental conclusions (B1 scope only)

These statements are limited to what was run and measured in B1. They do **not** predict performance in B2/B3 or other scenarios.

| Observation | Measured evidence |
|-------------|-------------------|
| J2 dominates 7-day drift vs two-body Case 1 | Same-tool position drift ~3.41M m; RAAN change ~6.92–6.93° (all tools) |
| GMAT and Orekit J2 propagation closely aligned | Final-state separation **35 m** on Case 2 |
| Basilisk J2 propagation close but not identical to Orekit | Final-state separation **1,294 m** on Case 2 |
| NRLMSISE-00 drag has measurable effect in Orekit and GMAT at 500 km | Case 3 vs Case 2 altitude change: **−76 m** (Orekit), **−85 m** (GMAT) |
| Basilisk exponential drag inactive at frozen parameters | Case 3 vs Case 2 altitude change: **0 m**; Case 3 metrics identical to Case 2 |
| SRP and third-body are small vs J2/drag here | Case 4→5 altitude steps **−1 m** or less per tool; Case 5 vs Case 4 cross-tool separation changes modestly |
| Fastest total B1 wall clock (five 7-day cases) | Basilisk **5.57 s**, Orekit **6.83 s**, GMAT **15.44 s** (no live plots) |
| GMAT live visualization adds GUI overhead | **171.9 s** total with OpenFrames orbit view + ground track vs **15.44 s** without |
| Attitude dynamics, GNC, and power were not tested | Out of B1 scope per frozen parameter table |

**Not tested in B1:** attitude control, battery/power models, multi-mode operations, STK, or any B2/B3 scenario.

---

## Evaluation sheets

| Tool | Sheet |
|------|-------|
| GMAT | `data/benchmark_b/b1_physics_fidelity/gmat/gmat_b1_evaluation_sheet.csv` |
| Basilisk | `data/benchmark_b/b1_physics_fidelity/basilisk/basilisk_b1_evaluation_sheet.csv` |
| Orekit | `data/benchmark_b/b1_physics_fidelity/orekit/orekit_b1_evaluation_sheet.csv` |

Rollup index: `docs/benchmarking/evaluation/B_EVALUATION_ROLLUP.md`

## Deferred

- STK B1 (not installed)
