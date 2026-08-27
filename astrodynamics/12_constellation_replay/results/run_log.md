# Basilisk A-0 constellation run log (2026-06-22)

## Configuration

- Spec: `benchmarks/benchmark_a/config/A0_PHYSICS_FIDELITY_1Y.md`
- Duration: 0 days
- Dynamics step: 10.0 s
- Output cadence: 60.0 s
- Basilisk version: 2.10.2

## Cases

### a0_case1_twobody

- Runtime: 49.998 s
- Output epochs: 9
- Gravity: Point-mass Earth (mu only, no harmonics)
- Drag: False
- SRP: False
- Third body: none
- Viz: companion_app / offline_postprocess

### a0_case2_j2

- Runtime: 49.998 s
- Output epochs: 9
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: False
- SRP: False
- Third body: none
- Viz: companion_app / offline_postprocess
- drift_vs_case1_m: 8085383.3489285065

### a0_case3_j2_drag

- Runtime: 49.998 s
- Output epochs: 9
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True
- SRP: False
- Third body: none
- Viz: companion_app / offline_postprocess
- drift_vs_case1_m: 13081957.438217629

### a0_case4_j2_drag_srp

- Runtime: 49.998 s
- Output epochs: 9
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True
- SRP: True
- Third body: none
- Viz: companion_app / offline_postprocess
- drift_vs_case1_m: 13082457.199239723

### a0_case5_all_supported

- Runtime: 49.998 s
- Output epochs: 9
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True
- SRP: True
- Third body: Sun, Moon
- Viz: companion_app / offline_postprocess
- drift_vs_case1_m: 6568540092.24798

## Pipeline

- Total wall-clock: 50.473 s

See `RESULTS_NOTES.md` for milestone-2 reporting notes.
