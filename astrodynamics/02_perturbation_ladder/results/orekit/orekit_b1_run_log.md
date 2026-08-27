# Orekit B1 run log (2026-07-09)

## Configuration

- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`
- Duration: 1.0 days
- Output cadence: 600.0 s
- Orekit data: `/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws/orekit-data.zip`

## Cases executed

### b1_case1_twobody

- Run ID: `b1_orekit_case1_twobody`
- Gravity harmonics: 0x0
- Drag: False (n/a)
- SRP: False
- Third body: none
- Propagation runtime: 0.124 s
- Runtime ratio: 696624.6

Orekit force models enabled:
- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 0x0 (central Earth)

Increment vs previous case: Baseline case. No perturbations beyond central Earth gravity.

### b1_case2_j2

- Run ID: `b1_orekit_case2_j2`
- Gravity harmonics: 2x0
- Drag: False (n/a)
- SRP: False
- Third body: none
- Propagation runtime: 0.068 s
- Runtime ratio: 1263992.6

Orekit force models enabled:
- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)

Increment vs previous case: Adds J2 via gravity harmonics 2x0 (replaces 0x0 central-only gravity).

Drift vs Case 1 (final epoch):
- Position drift: 474085.596 m
- Ground-track lon drift: -12.168731 deg
- RAAN change: 0.988973 deg
- Altitude change: -18607.309 m

### b1_case3_j2_drag

- Run ID: `b1_orekit_case3_j2_drag`
- Gravity harmonics: 2x0
- Drag: True (NRLMSISE00 + CssiSpaceWeatherData)
- SRP: False
- Third body: none
- Propagation runtime: 0.758 s
- Runtime ratio: 113933.0

Orekit force models enabled:
- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)
- DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)
- NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)

Increment vs previous case: Adds atmospheric drag (NRLMSISE00 + isotropic drag model).

Drift vs Case 1 (final epoch):
- Position drift: 474348.807 m
- Ground-track lon drift: -12.177861 deg
- RAAN change: 0.988974 deg
- Altitude change: -18611.107 m

### b1_case4_j2_drag_srp

- Run ID: `b1_orekit_case4_j2_drag_srp`
- Gravity harmonics: 2x0
- Drag: True (NRLMSISE00 + CssiSpaceWeatherData)
- SRP: True
- Third body: none
- Propagation runtime: 0.366 s
- Runtime ratio: 236277.1

Orekit force models enabled:
- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)
- DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)
- NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)
- SolarRadiationPressure with IsotropicRadiationSingleCoefficient(Cr=1.8, area=1.0 m²)

Increment vs previous case: Adds solar radiation pressure (isotropic single-coefficient model).

Drift vs Case 1 (final epoch):
- Position drift: 474285.128 m
- Ground-track lon drift: -12.175834 deg
- RAAN change: 0.989013 deg
- Altitude change: -18611.422 m

### b1_case5_all_supported

- Run ID: `b1_orekit_case5_all_supported`
- Gravity harmonics: 2x0
- Drag: True (NRLMSISE00 + CssiSpaceWeatherData)
- SRP: True
- Third body: Sun, Moon
- Propagation runtime: 0.559 s
- Runtime ratio: 154532.3

Orekit force models enabled:
- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)
- DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)
- NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)
- SolarRadiationPressure with IsotropicRadiationSingleCoefficient(Cr=1.8, area=1.0 m²)
- ThirdBodyAttraction(Sun)
- ThirdBodyAttraction(Moon)

Increment vs previous case: Adds Sun and Moon third-body gravity via ThirdBodyAttraction. Inherits all force models from Case 4.

**Case 5 — all supported (Orekit-specific definition):**
- Inherits from: `b1_case4_j2_drag_srp`
- Added in Case 5 vs Case 4:
  - ThirdBodyAttraction(Sun)
  - ThirdBodyAttraction(Moon)
- Not included in Orekit Case 5:
  - Higher-degree/order gravity field (beyond 2x0 J2-only policy)
  - Solid Earth tides or ocean tides on gravity field
  - Additional planets as third bodies
  - Relativistic corrections
  - Earth albedo / infrared radiation pressure
  - Empirical accelerations or custom force models

Drift vs Case 1 (final epoch):
- Position drift: 474369.547 m
- Ground-track lon drift: -12.178706 deg
- RAAN change: 0.988789 deg
- Altitude change: -18611.374 m

## Orekit B1 physics scope

Orekit B1 demonstrates a staged perturbation ladder using native Orekit force models aligned with the frozen cross-tool benchmark. It does not claim to exercise every physics feature Orekit supports.

Covered in this B1 run:
- Numerical propagation (DormandPrince853Integrator)
- Central gravity and J2-only gravity harmonics policy
- NRLMSISE00 atmospheric drag with space-weather data
- Isotropic solar radiation pressure
- Sun and Moon third-body point-mass gravity
- 7-day LEO state history export and drift metrics

Orekit-native features not exercised in B1:
- Full gravity field to high degree/order (e.g. EGM2008, tides)
- Alternative atmosphere models (Harris-Priester, exponential, DTM2000, etc.)
- Box-wing or multi-panel radiation pressure models
- Third-body perturbations from additional solar-system bodies
- Relativistic correction force models
- Earth albedo and infrared radiation pressure
- Empirical acceleration / custom force models
- Semi-analytical propagators (e.g. DSST) and TLE/SGP4 workflows
- Attitude-dependent drag and SRP (requires attitude dynamics, B2 scope)
- Maneuver modeling and event detection beyond fixed-force propagation

## Runtime

- Orekit init: 0.001 s
- Total pipeline: 10.834 s
