# A3 RW GNC (sensor est full) — 42 validation summary

- Samples: **76362**
- Sim span: **0.0 – 4581660.0 s** (1272.683 hr)

## ADCS pointing (PROTOTYPE_FSW + Point commands)

| Metric | Mean | Max | Final |
|--------|------|-----|-------|
| Thrust–velocity error [deg] | 4.394 | 80.639 | 3.069 |
| Nadir error [deg] | 1.402 | 81.488 | 0.003 |
| |ω| [deg/s] | 0.0599 | 1.8046 | 0.0047 |

## Orbit (FIXED on GMAT SPLINE)

- Mean position error vs spline: **1.844 m**
- Max position error: **8.474 m**
- Mean velocity error: **0.0004 m/s**

- Thruster force: mean **50.00 mN**, max **50.00 mN** (GMAT target 50 mN)

## Environment (GMAT-aligned)

| Effect | 42 flag | GMAT / notes |
|--------|---------|--------------|
| Drag | ON (shadows) | NRLMSISE00, F10.7=150, Ap=3, Cd=2.2 |
| SRP | ON (shadows) | Flux implicit 1367 W/m²; `A3_sphere` r≈0.178 m → ~0.1 m² |
| Gravity gradient | ON | Disturbance torque on attitude |
| Orbit drag/SRP | OFF (`GravPert`) | **FIXED** SPLINE — env forces do not re-prop orbit |

## Model fidelity (what is / is not being validated)

| Aspect | Current setup | Implication |
|--------|---------------|-------------|
| Attitude control | `PROTOTYPE_FSW` + **3 RW** (10 mNm / 60 mNms) + MTB unload | Limited torque & momentum; no IdealTrq |
| Orbit | **FIXED** on GMAT SPLINE | Trajectory imposed; thrust does not change orbit |
| Thruster | PROPORTIONAL @ **+Y offset 0.15 m**, 0.05 N | Thrust torque ~7.5 mNm about −Z |
| Sensors | 3-axis gyro + ST + mag (noisy) | Estimated `qbn`/`wbn`, not truth |
| Environment | Aero + SRP + gg **ON** | Disturbance torques on attitude |
| `THR_FSW` | Not used | Demo formation controller; ignores `Point` cmds |

Pointing error reflects RW saturation, thrust misalignment torque, sensor noise, and environmental disturbances — a more realistic ADCS stress test than IdealTrq.
