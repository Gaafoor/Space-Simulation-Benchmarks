# Benchmark B1 — Frozen Parameter Table

Authoritative variant: **B1 Physics Fidelity** (`docs/benchmarking/benchmarks/BENCHMARK_B1_PHYSICS_FIDELITY.md`).

**Frozen:** 2026-06-09

## Orbit and epoch

Inherited from Benchmark A1/A2 baseline (not A3 RAAN 61 trade case):

| Parameter | Value | Notes |
|-----------|-------|-------|
| Sub-benchmark | B1 | Physics fidelity / perturbation ladder |
| Epoch (UTC) | 02 Jun 2026 00:00:00.000 | Same as A1/A2 |
| Coordinate frame | Earth MJ2000 Equatorial (`EarthMJ2000Eq` / EME2000) | GMAT / Orekit naming |
| Semi-major axis | 6878.137 km | 500 km altitude above WGS84 equatorial radius |
| Altitude | 500 km | `R_EARTH + altitude` with `R_EARTH = 6378137 m` |
| Eccentricity | 0 (circular) | |
| Inclination | 97.4° | SSO-like LEO |
| RAAN | 0° | A1/A2 baseline; **not** A3 RAAN 61 |
| Argument of perigee | 0° | |
| True anomaly | 0° | |
| Simulated duration | **7 days** (604800 s) | Extended from A1/A2 24 h |
| Dynamics step | 10 s | Match A1/A2 integrator cadence |
| Output sample cadence | 600 s | 1009 epochs: 0, 600, …, 604800 |

## Spacecraft physical properties

Frozen for drag and SRP cases (Cases 3–5). Values align with GMAT A2/A3 CubeSat script defaults:

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Dry mass | 12 | kg | |
| Drag coefficient Cd | 2.2 | — | |
| Reflectivity coefficient Cr | 1.8 | — | SRP |
| Drag area | 0.01 | m² | |
| SRP area | 1.0 | m² | |
| Attitude for drag/SRP | **Fixed inertial** | — | No attitude dynamics in B1; spacecraft axes do not slew. Document if a tool requires a different default. |

## Perturbation cases (frozen ladder)

| Case | ID | Two-body | J2 | Drag | SRP | Third-body (Sun/Moon) | Notes |
|------|----|----------|----|------|-----|---------------------|-------|
| 1 | `b1_case1_twobody` | On | Off | Off | Off | Off | Point-mass Earth only |
| 2 | `b1_case2_j2` | On | On | Off | Off | Off | J2-only policy (see gravity table) |
| 3 | `b1_case3_j2_drag` | On | On | On | Off | Off | Atmosphere model per tool |
| 4 | `b1_case4_j2_drag_srp` | On | On | On | On | Off | SRP model per tool |
| 5 | `b1_case5_all_supported` | On | On | On | On | **Tool max** | All natively supported perturbations in that tool |

**Case 5 policy (cross-tool):** The label `all supported` is generic in shared benchmark documents. Each tool must record its **own** Case 5 definition in tool-specific logs and results. Case 5 is **not** required to be identical across tools.

**Orekit Case 5 definition:** `data/benchmark_b/b1_physics_fidelity/orekit/orekit_b1_case5_definition.md`

**Basilisk Case 5 definition:** `data/benchmark_b/b1_physics_fidelity/basilisk/basilisk_b1_case5_definition.md`

## Gravity model policy (per tool)

Cross-tool gravity is **not** harmonized (same Path B policy as A2). Document per run.

| Tool | Case 1 | Cases 2–5 |
|------|--------|-----------|
| GMAT | JGM2, degree 0, order 0 | JGM2, degree 2, order 0 |
| Basilisk | Point-mass Earth (`mu`) | `GGM03S-J2-only.txt`, max degree 2 |
| Orekit | Central attraction only | Normalized harmonics 2×0 (J2-only style) |

## Atmosphere model policy (Cases 3–5)

| Tool | Policy |
|------|--------|
| GMAT | Native drag force with spacecraft Cd and DragArea; default GMAT atmosphere model (document version in run log) |
| Basilisk | `DragDynamicEffector` or equivalent; document atmosphere model selected |
| Orekit | `DragForce` with documented atmosphere (e.g. NRLMSISE-00 or exponential if simpler); document in run log |

If a tool cannot run native drag, mark Case 3+ as **unsupported** for that tool and record evidence — do not substitute undisclosed custom drag without documentation.

## SRP model policy (Cases 4–5)

| Tool | Policy |
|------|--------|
| GMAT | Native SRP with Cr and SRPArea |
| Basilisk | `solarRadiationPressure` or equivalent module |
| Orekit | `SolarRadiationPressure` with isotropic or documented model |

## Third-body policy (Case 5 only)

| Tool | Target policy |
|------|---------------|
| GMAT | Sun and Moon third-body gravity if supported natively in force model |
| Basilisk | Sun/Moon point-mass gravity via SPICE if supported |
| Orekit | `ThirdBodyAttraction` for Sun and Moon |

## Integrator settings

| Tool | Setting |
|------|---------|
| GMAT | Runge-Kutta 89, initial step 10 s, accuracy 1e-12 (match A1) |
| Basilisk | Dynamics task step 10 s; recorder 600 s |
| Orekit | Dormand-Prince 853; min step 0.1 s, max step 300 s (document if changed) |

## Units in exports

| Quantity | Unit |
|----------|------|
| Position | m (convert GMAT km exports before comparison) |
| Velocity | m/s |
| Time | s from epoch |
| Angles | deg in summary metrics; rad allowed in raw element CSV if documented |

## Drift metric definitions

| Metric | Definition |
|--------|------------|
| `position_drift_vs_case1_m` | ‖r_caseN(t_end) − r_case1(t_end)‖ for same tool |
| `position_drift_vs_reference_m` | ‖r_tool(t_end) − r_ref(t_end)‖; reference documented in comparison report |
| `ground_track_lon_drift_deg` | Sub-satellite longitude difference vs Case 1 at t_end |
| `raan_change_deg` | RAAN(t_end) − RAAN(t_0) |
| `altitude_change_m` | Altitude(t_end) − Altitude(t_0) from semi-major axis or radius |
| `along_track_phase_drift_deg` | Argument of latitude (or true anomaly) difference vs Case 1 at t_end |

## Run IDs and naming

Pattern: `b1_<tool>_case<N>_<short_name>`

Examples:

- `b1_gmat_case1_twobody`
- `b1_basilisk_case3_j2_drag`
- `b1_orekit_case5_all_supported`

Artifact naming follows `docs/benchmarking/README.md`:

`YYYY-MM-DD_B1_<Simulator>_<run-id>.<ext>`

## Deferred

| Item | Status |
|------|--------|
| STK B1 | Deferred — STK not installed |
| Attitude dynamics | Out of B1 scope (B2) |
| Battery / power model | Out of B1 scope (B2) |

## Reproducibility artifacts

| Artifact | Path |
|----------|------|
| Shared config | `benchmarks/benchmark_b/b1_physics_config.py` |
| Post-processor | `benchmarks/benchmark_b/b1_metrics_postprocess.py` |
| Orekit runner | `benchmarks/benchmark_b/run_orekit_b1_physics_fidelity.py` |
| Orekit outputs | `data/benchmark_b/b1_physics_fidelity/orekit/` |
| Orekit run log | `data/benchmark_b/b1_physics_fidelity/orekit/orekit_b1_run_log.md` |
| Basilisk runner | `benchmarks/benchmark_b/run_basilisk_b1_physics_fidelity.py` |
| Basilisk outputs | `data/benchmark_b/b1_physics_fidelity/basilisk/` |
| Basilisk run log | `data/benchmark_b/b1_physics_fidelity/basilisk/basilisk_b1_run_log.md` |
| GMAT runner (script) | `benchmarks/benchmark_b/gmat/benchmark_b1_gmat.script` |
| GMAT post-processor | `benchmarks/benchmark_b/run_gmat_b1_physics_fidelity.py` |
| GMAT outputs | `data/benchmark_b/b1_physics_fidelity/gmat/` |
| GMAT run log | `data/benchmark_b/b1_physics_fidelity/gmat/gmat_b1_run_log.md` |
| Comparison report | `data/benchmark_b/B1_COMPARISON_REPORT.md` |
| Metrics CSV | `data/benchmark_b/B1_METRICS_COMPARISON.csv` |
