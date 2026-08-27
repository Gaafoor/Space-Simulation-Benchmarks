# Orekit B1 Case 5 — all supported (Orekit-specific definition)

Cross-tool benchmark documents use the label **all supported** generically. This file defines exactly what that means for **Orekit** in Benchmark B1.

## Inherits from

- `b1_case4_j2_drag_srp` (full Case 4 force stack)

## Added in Case 5 vs Case 4

- ThirdBodyAttraction(Sun)
- ThirdBodyAttraction(Moon)

## Full Orekit force stack in Case 5

- HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)
- DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)
- NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)
- SolarRadiationPressure with IsotropicRadiationSingleCoefficient(Cr=1.8, area=1.0 m²)
- ThirdBodyAttraction(Sun)
- ThirdBodyAttraction(Moon)

## Explicitly not added in Orekit Case 5

- Higher-degree/order gravity field (beyond 2x0 J2-only policy)
- Solid Earth tides or ocean tides on gravity field
- Additional planets as third bodies
- Relativistic corrections
- Earth albedo / infrared radiation pressure
- Empirical accelerations or custom force models

## Does Orekit B1 cover all Orekit physics fidelity features?

Orekit B1 demonstrates a staged perturbation ladder using native Orekit force models aligned with the frozen cross-tool benchmark. It does not claim to exercise every physics feature Orekit supports.

### Covered in this B1 run

- Numerical propagation (DormandPrince853Integrator)
- Central gravity and J2-only gravity harmonics policy
- NRLMSISE00 atmospheric drag with space-weather data
- Isotropic solar radiation pressure
- Sun and Moon third-body point-mass gravity
- 7-day LEO state history export and drift metrics

### Orekit-native but not in B1

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
