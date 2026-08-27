# Benchmark B2 — Frozen Parameter Table

Authoritative variant: **B2 Attitude & GNC** (`docs/benchmarking/benchmarks/BENCHMARK_B2_ATTITUDE_AND_GNC.md`).

**Frozen:** 2026-06-10

Execute only after B1 is closed (**B1 closed 2026-06-10**).

## Mission framing

| Aspect | Frozen value | Notes |
|--------|--------------|-------|
| Mission goal | Maximize battery charge (SoC) | Sun-pointing is the **means**, not the objective |
| Simulated duration | **24 h** (86400 s) | |
| Target SoC metric | **80%** | For `charging_time_to_target_soc_s` |
| Sun-pointing body axis | **+Z** | Solar panel normal |
| Eclipse model | **Cylindrical** Earth shadow | All cases |

## Case ladder

B2 is three cases on a shared spacecraft/orbit/power baseline. Each case tests a different ADCS verification tier.

| Case ID | Name | Purpose |
|---------|------|---------|
| `b2a` | Nominal GNC | Capability baseline — can the platform simulate ADCS correctly? |
| `b2b` | Realistic GNC | Operational behavior — sensors, disturbances, environment |
| `b2c` | Stress test | Robustness — failures, large tumble, Monte Carlo |

## Orbit and epoch (shared — inherit B1)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Epoch (UTC) | 02 Jun 2026 00:00:00.000 | Same as B1 / A1/A2 |
| Coordinate frame | Earth MJ2000 Equatorial (`EarthMJ2000Eq` / EME2000) | |
| Semi-major axis | 6878.137 km | 500 km altitude |
| Altitude | 500 km | `R_EARTH = 6378137 m` |
| Eccentricity | 0 (circular) | |
| Inclination | 97.4° | SSO-like LEO |
| RAAN | 0° | A1/A2 baseline |
| Argument of perigee | 0° | |
| True anomaly | 0° | |
| Dynamics step | 10 s | Match B1 integrator cadence |
| Output sample cadence | 10 s | Finer than B1 for GNC time series (8641 epochs) |

## Spacecraft (shared)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Mass | 12 | kg | Same as B1 |
| Inertia matrix | diag(0.15, 0.18, 0.12) | kg·m² | Distributed 6U-class model; not solid cuboid |
| Drag coefficient Cd | 2.2 | — | Inherited from B1; used in B2B/B2C |
| Reflectivity coefficient Cr | 1.8 | — | Inherited from B1; used in B2B/B2C |
| Drag area | 0.01 | m² | Inherited from B1; attitude-dependent in B2B/B2C |
| SRP area | 1.0 | m² | Inherited from B1; attitude-dependent in B2B/B2C |

## Power (shared)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Battery capacity | 40 | Wh | |
| Initial SoC | 40 | % | Partially depleted start |
| Solar panel effective area | 0.05 | m² | **Intentionally conservative** — benchmark should not be easy |
| Solar cell efficiency | 30 | % | Triple-junction class |
| Nominal bus load | 5 | W | Constant |

Generated power scales with Sun incidence on +Z panel. Net charging = generated − load.

## Actuators — reaction wheels (shared layout)

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Wheel count | 4 | — | Pyramid / redundant layout |
| Nominal torque limit (per wheel) | 0.005 | Nm (5 mNm) | B2A/B2B |
| Nominal momentum limit (per wheel) | 0.02 | Nms (20 mNms) | B2A/B2B |
| Stress torque limit (per wheel) | 0.003 | Nm (3 mNm) | **B2C only** — saturation stress |
| Stress momentum limit (per wheel) | 0.012 | Nms (12 mNms) | **B2C only** |

## Controller (shared type; per-tool gains)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Controller type | MRP feedback | Same control law class across tools |
| Gain policy | **Per-platform tuning** | Each tool tunes gains independently on B2A, then freezes for that tool |
| Cross-tool gain sharing | **Prohibited** | Compare mission outcome, runtime, effort, robustness — not identical gain values |

Record final per-tool gains in each tool's B2 run log.

## Initial conditions

| Parameter | B2A / B2B | B2C | Unit |
|-----------|-----------|-----|------|
| Sun misalignment | 30 | 120 | deg |
| Initial body rates | [0.5, 0.3, −0.4] | [2.0, 1.5, −1.0] | deg/s |

Attitude initialization: rotate +Z away from Sun direction by misalignment angle about a fixed body axis (document chosen axis per tool in run log; default: body +Y).

---

## Case B2A — Nominal GNC (`b2a`)

**Purpose:** Clean ADCS capability baseline.

### Force model

| Perturbation | B2A |
|--------------|-----|
| Two-body / central gravity | On |
| J2 | On |
| Drag | **Off** |
| SRP | **Off** |
| Gravity-gradient | **Off** |
| Third-body | Off |

Gravity model policy: same J2-only policy as B1 Case 2 (per-tool gravity files documented in run log).

### Sensors

| Parameter | B2A |
|-----------|-----|
| Sun sensor | **Ideal** (zero noise, zero bias) |

### Actuators

| Parameter | B2A |
|-----------|-----|
| Reaction wheels | **Ideal** within nominal torque/momentum limits |

### Disturbances

None beyond J2 orbit perturbation.

### Failures / Monte Carlo

| Parameter | B2A |
|-----------|-----|
| Actuator failures | None |
| Monte Carlo | Single deterministic run |

### Pass gate (B2A)

- Battery SoC increases during sunlit charging windows when controller is active.
- Final SoC > initial SoC (40%), or failure analyzed with evidence.
- Pointing error reported; not sole pass/fail gate.

---

## Case B2B — Realistic GNC (`b2b`)

**Purpose:** Operational spacecraft behavior.

Inherits all shared parameters and B2A initial conditions unless overridden below.

### Force model

| Perturbation | B2B |
|--------------|-----|
| Two-body / central gravity | On |
| J2 | On |
| Drag | **On** |
| SRP | **On** |
| Gravity-gradient | **On** (mandatory if tool supports; document if unsupported) |
| Third-body | Off |

Atmosphere / drag model: per-tool native policy (document in run log; same Path B non-harmonization as B1).

SRP model: per-tool native cannonball / flat-plate (attitude-dependent).

### Sensors

| Parameter | B2B |
|-----------|-----|
| Sun sensor noise | 0.15° σ per axis |
| Sun sensor bias | 0.5° constant per axis |

Literature anchor: fine sun sensor class (nanoSSOC / GOMspace FSS ±0.5° 3σ budget).

### Actuators

| Parameter | B2B |
|-----------|-----|
| Reaction wheels | Nominal limits; saturation logged |

### Disturbances

SRP torque, aerodynamic torque, and gravity-gradient torque **enabled** (attitude-dependent). Document tool-specific implementation in run log.

### Failures / Monte Carlo

| Parameter | B2B |
|-----------|-----|
| Actuator failures | None |
| Monte Carlo | Single deterministic run |

### Pass gate (B2B)

- Net positive energy over 24 h **or** documented failure (saturation, disturbances).
- Compare ΔSoC, pointing, and control effort vs B2A.

---

## Case B2C — Stress test (`b2c`)

**Purpose:** ADCS robustness under failures and large initial tumble.

Inherits B2B force model, sensors, and disturbances unless overridden below.

### Force model

Same as B2B (J2 + drag + SRP + gravity-gradient).

### Actuators

| Parameter | B2C |
|-----------|-----|
| Reaction wheels | **Stress limits** (3 mNm / 12 mNms per wheel) |
| Wheel 4 failure | **Stuck** at t = 0 (zero torque; not removed from geometry) |

Stuck wheel is an operational failure mode, not a design change.

### Initial conditions

Large tumble (see table above): 120° misalignment, [2.0, 1.5, −1.0] deg/s.

### Monte Carlo

| Parameter | Value |
|-----------|-------|
| Runs (preferred) | **100** |
| Runs (minimum) | **50** |
| Random seed | Fixed and documented per tool |
| Dispersions | ICs ±10%; sensor bias ±0.3°; initial rates ±0.2 deg/s |

Report mean ± σ for: final SoC, time-to-80%, max pointing error, saturation duty cycle, eclipse-exit settling time.

### Pass gate (B2C)

- Document robustness statistics; reaching 80% SoC is **not** required.
- Failure modes and dispersion are data.

---

## Candidate simulators

| Tier | Tools | Policy |
|------|-------|--------|
| **Primary** | Basilisk, FreeFlyer | Full B2A → B2B → B2C execution |
| **Capability probe** | GMAT, Orekit | Attempt B2A; if successful attempt B2B; if successful attempt B2C; **stop and document** when a case is unsupported |
| **Deferred** | STK | Until installed |

FreeFlyer: student license confirmed 2026-06-10.

## Metrics (all cases)

| Metric | Role |
|--------|------|
| `battery_soc_percent` | **Primary** mission outcome |
| `generated_power_w` | Charging performance |
| `eclipse_duration_s` / `eclipse_fraction` | Environmental constraint |
| `charging_time_to_target_soc_s` | Mission efficiency (target 80%) |
| `pointing_error_deg` | Contributing GNC metric |
| `control_effort_rms` | Actuator usage |
| `wheel_saturation_fraction` | B2B/B2C |
| `eclipse_exit_settling_time_s` | B2C emphasis |
| `wall_clock_runtime_s` | Performance |
| `mc_final_soc_mean` / `mc_final_soc_std` | B2C Monte Carlo |

Primary pass gate is **energy outcome**, not pointing alone.

## Artifacts

| Item | Path |
|------|------|
| Comparison report | `data/benchmark_b/B2_COMPARISON_REPORT.md` |
| Metrics CSV | `data/benchmark_b/B2_METRICS_COMPARISON.csv` |
| Tool outputs | `data/benchmark_b/b2_attitude_gnc/<tool>/b2a/|b2b/|b2c/` |
| Per-tool gains | Each tool's `*_b2_run_log.md` |
