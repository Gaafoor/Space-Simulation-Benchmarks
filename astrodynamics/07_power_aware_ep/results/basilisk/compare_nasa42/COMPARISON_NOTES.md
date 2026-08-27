# Basilisk vs NASA 42 — implementation comparison notes

For inclusion in benchmarks main PDF (sections 1.10 / 1.11).

## Solar / eclipse modelling

**Basilisk advantage (native capability).** Basilisk computes a continuous illumination factor (`eclipse_factor` ∈ [0, 1]) from dual-cone eclipse geometry (`Eclipse` module + `attach_solar_environment.py`). Partial penumbra during shadow ingress/egress reduces solar power smoothly before the minimum-usable-power cutoff.

**NASA 42 limitation (default configuration, not modified here).** `GmatAvailableThrustPowerKw` in `42actuators.c` uses a binary sun fraction when `SolarShadowActive`: `percent_sun = Eclipse ? 0 : 1`. `Eclipse.42` is therefore 0/1 only. Upgrading 42 would require custom penumbra geometry inside the actuator path; this benchmark intentionally keeps stock 42 eclipse logic.

**Impact on comparison plots:** 42 shows sharp thrust/power duty cycles during early orbit; Basilisk shows near-continuous thrust with small eclipse dips. This is a **model fidelity difference**, not a bug in either exporter.

## Gravity harmonics vs GMAT design authority

**GMAT trajectory design** (`catec_ws/benchmarks/benchmark_a/gmat/scripts/Benchmark_A3_LEO_GEO_Transfer.script`):

- `A3_FM.GravityField.Earth.Degree = 20`, `Order = 20`
- `PotentialFile = 'JGM2.cof'`
- NRLMSISE-00 drag, spherical SRP, Sun + Luna point masses

**Basilisk execution** (`force_model.py`, `gmat_match`): JGM2-equivalent `GGM2BData` 20×20 — aligned with GMAT.

**NASA 42 execution** (`Inp_Sim.txt`): Earth gravity 8×8 (42 file limit comment `<=18`). Runtime/speed choice; 42 can load higher-degree harmonics but was not configured to match GMAT 20×20 for N9.

## Fidelity tier used for Basilisk full E2E (`dynamic_solar`)

Command: `--fidelity dynamic_solar` (no `--mtb`, no `full_sensors`).

| Feature | Enabled | Notes |
|---------|---------|-------|
| Dynamic solar + polynomial thrust | Yes | `dynamic_solar=True` |
| RW closed-loop velocity pointing | Yes | `gnc_mode=rw_feedback_velocity_pointing` |
| SimpleNav + small Gauss–Markov noise | Yes | Not truth state; σ_att ≈ 0.001° |
| Hardware sensors (ST/IMU/mag + bridge) | No | Requires `full_sensors` tier |
| MTB momentum dump | No | Requires `--mtb` flag |

Tiering isolates propulsion/orbit fidelity first; full sensor stack deferred to `full_sensors` / `mission_realistic` due to runtime cost (~10 h full mission already at `dynamic_solar`).
