# Basilisk B1 Case 5 — all supported (Basilisk-specific definition)

Cross-tool benchmark documents use the label **all supported** generically. This file defines exactly what that means for **Basilisk** in Benchmark B1.

## Inherits from

- `b1_case4_j2_drag_srp` (full Case 4 force stack)

## Added in Case 5 vs Case 4

- Moon point-mass gravity via gravBodyFactory
- Sun point-mass gravity via gravBodyFactory (in addition to SRP ephemeris use)

## Full Basilisk force stack in Case 5

- Earth gravity via GGM03S-J2-only.txt (max degree 2)
- ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')
- DragDynamicEffector (Cd=2.2, projected area=0.01 m²)
- RadiationPressure cannonball (Cr=1.8, area=1.0 m²)
- SPICE ephemerides (de430, naif0012, de_403_masses, pck00010)
- Sun and Moon point-mass gravity via gravBodyFactory

## Explicitly not added in Basilisk Case 5

- Higher-degree/order gravity field (beyond frozen J2-only policy)
- MsisAtmosphere or other atmosphere models
- FacetSRPDynamicEffector or faceted drag models
- Additional planetary third bodies beyond Sun and Moon
- Attitude-dependent drag/SRP (B2 scope)

## Does Basilisk B1 cover all Basilisk physics fidelity features?

Basilisk B1 demonstrates the frozen cross-tool perturbation ladder with native Basilisk modules. It does not claim to exercise every physics feature Basilisk supports.

### Covered in this B1 run

- Spacecraft hub propagation with 10 s dynamics step
- Point-mass Earth gravity (Case 1)
- J2-only GGM03S gravity harmonics (Cases 2–5)
- ExponentialAtmosphere drag with DragDynamicEffector
- RadiationPressure cannonball SRP with SPICE Sun ephemeris
- Sun and Moon point-mass third-body gravity (Case 5)
- 7-day LEO state history export and drift metrics

### Basilisk-native but not in B1

- Full gravity field to high degree/order
- MsisAtmosphere and other atmosphere models
- Facet drag / facet SRP dynamic effectors
- Additional third bodies or custom gravity bodies
- Integrating custom Python force models via DynamicEffector
- Attitude-dependent drag and SRP (B2 scope)
- Maneuver and event-driven propagation workflows
