# GMAT B1 run log (2026-07-09)

## Configuration

- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`
- GMAT script: `benchmarks/benchmark_b/gmat/benchmark_b1_gmat.script`
- Input CSV directory: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1`
- Duration: 1.0 days
- Output cadence: 600.0 s
- GMAT version: R2026a GmatConsole
- Python post-processor: `benchmarks/benchmark_b/run_gmat_b1_physics_fidelity.py`

## Cases processed

### b1_case1_twobody

- Run ID: `b1_gmat_case1_twobody`
- Status: pass (145 samples)
- Source CSV: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1/gmat_b1_case1_twobody_state.csv`
- Gravity: JGM2.cof 0x0
- Drag: False (n/a)
- SRP: False (n/a)
- Third body: none

GMAT force models enabled:
- Earth central gravity via JGM-2, degree 0, order 0 (point-mass Earth)

Increment vs previous case: Baseline case. Point-mass Earth gravity only (JGM-2 deg 0).

### b1_case2_j2

- Run ID: `b1_gmat_case2_j2`
- Status: pass (145 samples)
- Source CSV: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1/gmat_b1_case2_j2_state.csv`
- Gravity: JGM2.cof 2x0
- Drag: False (n/a)
- SRP: False (n/a)
- Third body: none

GMAT force models enabled:
- Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)

Increment vs previous case: Adds J2 via JGM-2 degree 2, order 0.

Drift vs Case 1 (final epoch):
- Position drift: 474085.433 m
- Ground-track lon drift: -12.168722 deg
- RAAN change: 0.988973 deg
- Altitude change: -18607.293 m

### b1_case3_j2_drag

- Run ID: `b1_gmat_case3_j2_drag`
- Status: pass (145 samples)
- Source CSV: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1/gmat_b1_case3_j2_drag_state.csv`
- Gravity: JGM2.cof 2x0
- Drag: True (NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt))
- SRP: False (n/a)
- Third body: none

GMAT force models enabled:
- Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)
- Native drag with Cd=2.2, DragArea=0.01 m²
- NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)

Increment vs previous case: Adds native atmospheric drag (NRLMSISE-00).

Drift vs Case 1 (final epoch):
- Position drift: 474459.472 m
- Ground-track lon drift: -12.181698 deg
- RAAN change: 0.988975 deg
- Altitude change: -18612.697 m

### b1_case4_j2_drag_srp

- Run ID: `b1_gmat_case4_j2_drag_srp`
- Status: pass (145 samples)
- Source CSV: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1/gmat_b1_case4_j2_drag_srp_state.csv`
- Gravity: JGM2.cof 2x0
- Drag: True (NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt))
- SRP: True (Spherical SRP)
- Third body: none

GMAT force models enabled:
- Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)
- Native drag with Cd=2.2, DragArea=0.01 m²
- NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)
- Spherical solar radiation pressure with Cr=1.8, SRPArea=1.0 m²

Increment vs previous case: Adds spherical solar radiation pressure.

Drift vs Case 1 (final epoch):
- Position drift: 474376.651 m
- Ground-track lon drift: -12.179009 deg
- RAAN change: 0.989016 deg
- Altitude change: -18612.724 m

### b1_case5_all_supported

- Run ID: `b1_gmat_case5_all_supported`
- Status: pass (145 samples)
- Source CSV: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/benchmarks/benchmark_b/results/gmat_b1/gmat_b1_case5_all_supported_state.csv`
- Gravity: JGM2.cof 2x0
- Drag: True (NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt))
- SRP: True (Spherical SRP)
- Third body: Sun, Luna

GMAT force models enabled:
- Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)
- Native drag with Cd=2.2, DragArea=0.01 m²
- NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)
- Spherical solar radiation pressure with Cr=1.8, SRPArea=1.0 m²
- Sun and Luna point-mass third-body gravity

Increment vs previous case: Adds Sun and Luna point-mass third-body gravity. Inherits all force models from Case 4.

**Case 5 — all supported (GMAT-specific definition):**
- Inherits from: `b1_case4_j2_drag_srp`
- Added in Case 5 vs Case 4:
  - Sun point-mass gravity
  - Luna (Moon) point-mass gravity
- Not included in GMAT Case 5:
  - Higher-degree/order gravity field (beyond J2-only policy)
  - Solid Earth tides or ocean tides on gravity field
  - Additional planets as third bodies
  - Relativistic corrections
  - Earth albedo / infrared radiation pressure
  - SPAD or facet SRP models

Drift vs Case 1 (final epoch):
- Position drift: 474461.104 m
- Ground-track lon drift: -12.181882 deg
- RAAN change: 0.988791 deg
- Altitude change: -18612.676 m

## GMAT B1 physics scope

GMAT B1 demonstrates a staged perturbation ladder using native GMAT force models aligned with the frozen cross-tool benchmark. It does not claim to exercise every physics feature GMAT supports.

Covered in this B1 run:
- Numerical propagation (Runge-Kutta 89)
- JGM-2 central gravity and J2-only harmonics policy
- NRLMSISE-00 atmospheric drag with CSSI space-weather file
- Spherical solar radiation pressure
- Sun and Luna third-body point-mass gravity
- 7-day LEO state history export and drift metrics

GMAT-native features not exercised in B1:
- Full gravity field to high degree/order (e.g. EGM96, tides)
- Alternative atmosphere models (MSISE90, JacchiaRoberts, exponential)
- SPAD file-based or facet SRP models
- Third-body perturbations from additional solar-system bodies
- Relativistic correction force models
- Attitude-dependent drag and SRP (requires attitude dynamics, B2 scope)
- Maneuver modeling and event detection beyond fixed-force propagation

## Runtime

- Python post-processing: 1.355 s
- GMAT mission: all five cases run sequentially in one script execution
- GMAT GUI total (no live plots): 13.867 s
- GMAT per-case amortized (no plots): 2.773 s
- Runtime ratio per case (no plots): 31153.1
- Metrics CSV uses per-case amortized no-plots wall clock for cross-tool comparison
