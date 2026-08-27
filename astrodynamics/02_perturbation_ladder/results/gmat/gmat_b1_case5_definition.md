# GMAT B1 Case 5 — all supported (GMAT-specific definition)

Cross-tool benchmark documents use the label **all supported** generically. This file defines exactly what that means for **GMAT** in Benchmark B1.

## Inherits from

- `b1_case4_j2_drag_srp` (full Case 4 force stack)

## Added in Case 5 vs Case 4

- Sun point-mass gravity
- Luna (Moon) point-mass gravity

## Full GMAT force stack in Case 5

- Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)
- Native drag with Cd=2.2, DragArea=0.01 m²
- NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)
- Spherical solar radiation pressure with Cr=1.8, SRPArea=1.0 m²
- Sun and Luna point-mass third-body gravity

## Explicitly not added in GMAT Case 5

- Higher-degree/order gravity field (beyond J2-only policy)
- Solid Earth tides or ocean tides on gravity field
- Additional planets as third bodies
- Relativistic corrections
- Earth albedo / infrared radiation pressure
- SPAD or facet SRP models

## Does GMAT B1 cover all GMAT physics fidelity features?

GMAT B1 demonstrates a staged perturbation ladder using native GMAT force models aligned with the frozen cross-tool benchmark. It does not claim to exercise every physics feature GMAT supports.

### Covered in this B1 run

- Numerical propagation (Runge-Kutta 89)
- JGM-2 central gravity and J2-only harmonics policy
- NRLMSISE-00 atmospheric drag with CSSI space-weather file
- Spherical solar radiation pressure
- Sun and Luna third-body point-mass gravity
- 7-day LEO state history export and drift metrics

### GMAT-native but not in B1

- Full gravity field to high degree/order (e.g. EGM96, tides)
- Alternative atmosphere models (MSISE90, JacchiaRoberts, exponential)
- SPAD file-based or facet SRP models
- Third-body perturbations from additional solar-system bodies
- Relativistic correction force models
- Attitude-dependent drag and SRP (requires attitude dynamics, B2 scope)
- Maneuver modeling and event detection beyond fixed-force propagation
