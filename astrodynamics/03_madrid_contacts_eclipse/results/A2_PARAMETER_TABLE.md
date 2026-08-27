# Benchmark A2 — Frozen Parameter Table

Variant **A2 realistic mission** (extends A1 orbit; J2 and mission products enabled).

## Inherited from A1

Same orbit, epoch, duration, and sample cadence as `A1_PARAMETER_TABLE.md`:

- 500 km circular LEO, i = 97.4°, epoch 02 Jun 2026 00:00:00 UTC
- 24 h, dynamics step 10 s, state samples every 600 s

## A2 dynamics

| Parameter | Basilisk | GMAT |
|-----------|----------|------|
| J2 | **On** — `GGM03S-J2-only.txt`, max degree 2 | **On** — bundled **JGM-2**, degree 2, order 0 (`JGM2.cof`) |
| Drag | Off | Off |
| SRP | Off | Off |
| Third-body | Off (Sun for eclipse via SPICE) | Off (Sun for eclipse via SPICE locators) |
| SPICE | de430, naif0012, de_403_masses, pck00010 | GMAT SolarSystem / SPICE kernels for event locators |

**Cross-tool gravity (Path B):** Models are **not** harmonized. Propagation state comparison is **indicative only**; mission metrics (passes, revisit, eclipse) are the primary A2 comparison metrics.

## Ground target — Madrid

| Parameter | Value |
|-----------|-------|
| Latitude | 40.4168° N |
| Longitude | 3.7038° W |
| Altitude | 667 m |
| Minimum elevation | 10° |

## Mission metrics (Basilisk definitions)

| Metric | Definition |
|--------|------------|
| observation_window_count | Count of contiguous `hasAccess` intervals |
| avg_observation_window_duration | Mean pass duration [min] |
| total_contact_time | Sum of pass durations [min] |
| coverage_revisit_time | Minimum time between pass start times [min] |
| eclipse_time_percent | Fraction of sim time with `shadowFactor < 0.999` |

## Artifacts

| File | Path |
|------|------|
| Runner | `benchmarks/benchmark_a/run_basilisk_a2_mission_analysis.py` |
| Config | `benchmarks/benchmark_a/a2_mission_config.py` |
| Outputs | `data/benchmark_a/basilisk_a2/` |

## Note on analytical reference

A1 Kepler two-body reference is **not** used as A2 pass/fail criterion (J2 enabled). Cross-tool A2 comparison uses GMAT and Basilisk mission products; see `A2_COMPARISON_REPORT.md`.
