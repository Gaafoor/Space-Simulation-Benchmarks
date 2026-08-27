# Basilisk B1 run log (2026-07-09)

## Configuration

- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`
- Duration: 1.0 days
- Dynamics step: 10.0 s
- Output cadence: 600.0 s
- Basilisk version: 2.11.0b0

## Cases executed

### b1_case1_twobody

- Run ID: `b1_basilisk_case1_twobody`
- Gravity: Point-mass Earth (mu only, no harmonics)
- Drag: False (n/a)
- SRP: False (n/a)
- Third body: none
- SPICE: False
- Propagation runtime: 0.138 s
- Runtime ratio: 623926.4

Basilisk force models enabled:
- simIncludeGravBody.gravBodyFactory.createEarth() with point-mass gravity (mu only)

Increment vs previous case: Baseline case. Point-mass Earth gravity only.

### b1_case2_j2

- Run ID: `b1_basilisk_case2_j2`
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: False (n/a)
- SRP: False (n/a)
- Third body: none
- SPICE: False
- Propagation runtime: 0.142 s
- Runtime ratio: 606876.1

Basilisk force models enabled:
- Earth gravity via GGM03S-J2-only.txt (max degree 2)

Increment vs previous case: Adds J2 via GGM03S-J2-only spherical harmonics.

Drift vs Case 1 (final epoch):
- Position drift: 474127.609 m
- Ground-track lon drift: -12.160025 deg
- RAAN change: 0.988737 deg
- Altitude change: -18596.143 m

### b1_case3_j2_drag

- Run ID: `b1_basilisk_case3_j2_drag`
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True (ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset))
- SRP: False (n/a)
- Third body: none
- SPICE: False
- Propagation runtime: 0.144 s
- Runtime ratio: 601610.3

Basilisk force models enabled:
- Earth gravity via GGM03S-J2-only.txt (max degree 2)
- ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')
- DragDynamicEffector (Cd=2.2, projected area=0.01 m²)

Increment vs previous case: Adds ExponentialAtmosphere + DragDynamicEffector.

Drift vs Case 1 (final epoch):
- Position drift: 474127.609 m
- Ground-track lon drift: -12.160025 deg
- RAAN change: 0.988737 deg
- Altitude change: -18596.143 m

### b1_case4_j2_drag_srp

- Run ID: `b1_basilisk_case4_j2_drag_srp`
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True (ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset))
- SRP: True (RadiationPressure cannonball model)
- Third body: none
- SPICE: True
- Propagation runtime: 0.191 s
- Runtime ratio: 453324.6

Basilisk force models enabled:
- Earth gravity via GGM03S-J2-only.txt (max degree 2)
- ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')
- DragDynamicEffector (Cd=2.2, projected area=0.01 m²)
- RadiationPressure cannonball (Cr=1.8, area=1.0 m²)
- SPICE sun ephemeris for SRP

Increment vs previous case: Adds RadiationPressure (cannonball) with SPICE Sun ephemeris.

Drift vs Case 1 (final epoch):
- Position drift: 473914.730 m
- Ground-track lon drift: -12.162907 deg
- RAAN change: 0.988640 deg
- Altitude change: -18606.265 m

### b1_case5_all_supported

- Run ID: `b1_basilisk_case5_all_supported`
- Gravity: GGM03S J2-only spherical harmonics (max degree 2)
- Drag: True (ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset))
- SRP: True (RadiationPressure cannonball model)
- Third body: Sun, Moon
- SPICE: True
- Propagation runtime: 0.263 s
- Runtime ratio: 328607.9

Basilisk force models enabled:
- Earth gravity via GGM03S-J2-only.txt (max degree 2)
- ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')
- DragDynamicEffector (Cd=2.2, projected area=0.01 m²)
- RadiationPressure cannonball (Cr=1.8, area=1.0 m²)
- SPICE ephemerides (de430, naif0012, de_403_masses, pck00010)
- Sun and Moon point-mass gravity via gravBodyFactory

Increment vs previous case: Adds Sun and Moon point-mass gravity bodies via gravBodyFactory. Inherits all force models from Case 4.

**Case 5 — all supported (Basilisk-specific definition):**
- Inherits from: `b1_case4_j2_drag_srp`
- Added in Case 5 vs Case 4:
  - Moon point-mass gravity via gravBodyFactory
  - Sun point-mass gravity via gravBodyFactory (in addition to SRP ephemeris use)
- Not included in Basilisk Case 5:
  - Higher-degree/order gravity field (beyond frozen J2-only policy)
  - MsisAtmosphere or other atmosphere models
  - FacetSRPDynamicEffector or faceted drag models
  - Additional planetary third bodies beyond Sun and Moon
  - Attitude-dependent drag/SRP (B2 scope)

Drift vs Case 1 (final epoch):
- Position drift: 473999.145 m
- Ground-track lon drift: -12.165777 deg
- RAAN change: 0.988415 deg
- Altitude change: -18606.217 m

## Basilisk B1 physics scope

Basilisk B1 demonstrates the frozen cross-tool perturbation ladder with native Basilisk modules. It does not claim to exercise every physics feature Basilisk supports.

Covered in this B1 run:
- Spacecraft hub propagation with 10 s dynamics step
- Point-mass Earth gravity (Case 1)
- J2-only GGM03S gravity harmonics (Cases 2–5)
- ExponentialAtmosphere drag with DragDynamicEffector
- RadiationPressure cannonball SRP with SPICE Sun ephemeris
- Sun and Moon point-mass third-body gravity (Case 5)
- 7-day LEO state history export and drift metrics

Basilisk-native features not exercised in B1:
- Full gravity field to high degree/order
- MsisAtmosphere and other atmosphere models
- Facet drag / facet SRP dynamic effectors
- Additional third bodies or custom gravity bodies
- Integrating custom Python force models via DynamicEffector
- Attitude-dependent drag and SRP (B2 scope)
- Maneuver and event-driven propagation workflows

## Runtime

- Total pipeline: 2.334 s
