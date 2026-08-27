# Benchmark A1 — Frozen Parameter Table

Authoritative variant: **A1 analytical two-body validation** (`docs/benchmarking/benchmarks/BENCHMARK_A_FINAL_SPECIFICATION.md`).

## Orbit and epoch

| Parameter | Value | Notes |
|-----------|-------|-------|
| Variant | A1 | Propagation validation only; no mission products |
| Epoch (UTC) | 02 Jun 2026 00:00:00.000 | GMAT spacecraft epoch |
| Coordinate frame | Earth MJ2000 Equatorial (`EarthMJ2000Eq`) | GMAT export frame |
| Semi-major axis | 6878.137 km | 500 km altitude above WGS84 equatorial radius |
| Altitude | 500 km | `R_EARTH + altitude` with `R_EARTH = 6378137 m` |
| Eccentricity | 0 (circular) | GMAT reports ~1e-16 numerical noise |
| Inclination | 97.4° | SSO-like LEO |
| RAAN | 0° | |
| Argument of perigee | 0° | |
| True anomaly | 0° | |
| Simulated duration | 24 h (86400 s) | |
| Output sample cadence | 600 s | 145 epochs: 0, 600, …, 86400 |

## Earth and dynamics (A1)

| Parameter | Value |
|-----------|-------|
| Earth model | WGS84 |
| μ (reference / Basilisk) | 3.986004418×10¹⁴ m³/s² |
| R_Earth (reference / Basilisk) | 6378137 m |
| J2 | **Off** |
| Drag | **Off** |
| SRP | **Off** |
| Third-body (Sun/Moon) | **Off** |
| SPICE (Basilisk A1) | **Not used** |
| GMAT gravity field | JGM2, Degree 0, Order 0 |
| GMAT propagator | Runge-Kutta 89, initial step 10 s, accuracy 1e-12 |

## Integrator / task settings

| Tool | Setting |
|------|---------|
| Basilisk dynamics step | 10 s |
| Basilisk output recorder | 600 s |
| GMAT propagation | Increment 600 s per loop iteration (`ElapsedSecs = stepSec`) |
| GMAT reports | Mission `Report` only; **no** `ReportFile.Add` |

## Reference model (independent truth for A1)

| Item | Value |
|------|-------|
| Model | Analytical Kepler two-body |
| Implementation | `benchmarks/benchmark_a/a1_analytical_reference.py` |
| Purpose | Validate propagators; **not** STK/GMAT as reference |

## Units in exports

| Tool | Position | Velocity |
|------|----------|----------|
| GMAT CSV | km | km/s |
| Basilisk / post-processing | m | m/s |
| Reference / comparison CSVs | m | m/s |

## Deferred (not in A1 scope)

| Item | Status |
|------|--------|
| STK A1 | Deferred — STK not installed |
| Madrid ground target / passes | A2 |
| Eclipse / revisit / coverage | A2 |
| J2 enabled | A2 |

## Reproducibility artifacts

| Artifact | Path |
|----------|------|
| Basilisk runner | `benchmarks/benchmark_a/run_basilisk_a1_analytical_validation.py` |
| GMAT mission script | `benchmarks/benchmark_a/gmat/benchmark_a1_gmat.script` |
| GMAT post-processor | `benchmarks/benchmark_a/run_gmat_a1_analytical_validation.py` |
| Analytical reference | `benchmarks/benchmark_a/a1_analytical_reference.py` |
