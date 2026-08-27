# Benchmark A3 - Parameter Table

Authoritative variant: `docs/benchmarking/benchmarks/BENCHMARK_A3_IMAGING_OPPORTUNITY_ANALYSIS.md`.

## Inherited Mission Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Variant | A3 | Imaging opportunity analysis |
| Epoch (UTC) | 02 Jun 2026 00:00:00.000 | A1/A2 |
| Orbit type | SSO-like circular LEO | A1/A2 |
| Altitude | 500 km | A1/A2 |
| Semi-major axis | 6878.137 km | A1/A2 |
| Eccentricity | 0 | A1/A2 |
| Inclination | 97.4 deg | A1/A2 |
| RAAN | 0 deg | A1/A2 |
| Argument of perigee | 0 deg | A1/A2 |
| True anomaly | 0 deg | A1/A2 |
| Duration | 24 h | A1/A2 |
| Dynamics sample step | 10 s where available | A2 |
| Sparse state sample cadence | 600 s | A1/A2 |

## Dynamics Policy

| Parameter | Basilisk | GMAT |
|-----------|----------|------|
| J2 | On - GGM03S J2-only, degree 2 | On - bundled JGM-2, degree 2, order 0 |
| Drag | Off | Off |
| SRP | Off | Off |
| Third-body gravity | Off | Off |
| SPICE | Used for Earth/Sun geometry where inherited from A2 | GMAT SolarSystem / event geometry |

Cross-tool state comparison remains indicative only under the existing A2 Path B gravity policy. A3 compares sensor-geometry workflow and resulting imaging windows, not exact state identity.

## Ground Target

| Parameter | Value |
|-----------|-------|
| Target | Madrid |
| Latitude | 40.4168 deg N |
| Longitude | 3.7038 deg W |
| Altitude | 667 m |
| Minimum elevation | 10 deg for A2 access context |

## Camera Model

| Parameter | Value | Notes |
|-----------|-------|-------|
| Camera model | Conical field of view | Geometry only |
| Full cone FOV | 10 deg | Frozen for A3 run |
| Half-angle | 5 deg | Imaging threshold |
| Default pointing | Nadir pointing | Target pointing may be tested later if a tool supports it natively |
| Rendered image output | No | Deferred to C1 |
| Footprint model | Flat-Earth small-angle estimate | Future work: spherical footprint |

## Coverage and Heatmap Implementation Note

For A3 in this repository, coverage percentage, regional grid coverage, footprint map, and heatmap plots are implemented as custom Python post-processing functions (for Basilisk, GMAT, and Orekit runners). They are not produced by a native built-in coverage module in Basilisk, GMAT script syntax, or Orekit APIs in the current A3 workflow.

## A3 Imaging Metrics

| Metric | Definition |
|--------|------------|
| imaging_opportunity_count | Count of contiguous target-in-FOV intervals |
| avg_imaging_opportunity_duration_min | Mean interval duration |
| total_imaging_time_min | Sum of all imaging opportunity durations |
| min_revisit_time_min | Minimum time between opportunity starts |
| minimum_boresight_angle_deg | Minimum target angle from camera boresight during run |
| ground_footprint_diameter_km | Approximate nadir footprint diameter |
| sensor_geometry_native_support | Whether the tool computed sensor access natively |
| rendered_image_output_supported | Whether the tool generated actual images; expected No/N/A for A3 |

## Deferred

| Item | Status |
|------|--------|
| STK A3 | Deferred until STK is installed |
| FreeFlyer | Deferred pending student license response |
| Synthetic image generation | Deferred to C1 |
| Target-pointing camera mode | Optional future extension after nadir A3 baseline |

## Planned Artifacts

| Artifact | Path |
|----------|------|
| Basilisk/Python runner | `benchmarks/benchmark_a/run_basilisk_a3_imaging_analysis.py` |
| GMAT A3 script | `benchmarks/benchmark_a/gmat/benchmark_a3_gmat.script` |
| GMAT post-processor | `benchmarks/benchmark_a/run_gmat_a3_analysis.py` |
| Orekit/Python runner | `benchmarks/benchmark_a/run_orekit_a3_imaging_analysis.py` |
| Basilisk consolidated outputs | `data/benchmark_a/basilisk_a3/` |
| GMAT outputs | `data/benchmark_a/gmat_a3/` |
| Orekit outputs | `data/benchmark_a/orekit_a3/` |
| Cross-tool report | `data/benchmark_a/A3_COMPARISON_REPORT.md` |
| Metrics comparison | `data/benchmark_a/a3_imaging_metrics_comparison.csv` |
