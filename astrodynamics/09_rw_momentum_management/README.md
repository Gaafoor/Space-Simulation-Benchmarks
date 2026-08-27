# 09 — Reaction-wheel and momentum-management behavior

## What this is

Same actuator layout in both executors: **4-wheel pyramid**, 0.150 N m s per-wheel limit, **3-axis MTB** at 5.0 A m². On the 53-day spiral:

- Basilisk peaks at **7.7%** of the wheel envelope but hits **25%** RW torque saturation; MTB unloads at the dipole limit (keeps stored momentum low).
- NASA 42 reaches **59%** of the wheel envelope; MTB commands stay ~0 (spline-guided mesh, smaller disturbance budget).

## Official example this started from

- **Basilisk** `scenarioMtbMomentumManagement.py`, `scenarioMomentumDumping.py`, `scenarioAttitudeFeedbackRW.py`. Official examples dump momentum on a short LEO arc, not during a GMAT-scheduled EP spiral with matched 42 limits.
- **NASA 42** wheel + MTB fields on `SC_*.txt` and PrototypeFSW unload logic. Stock Demo does not use the A3 0.150 N m s / 5 A m² envelope on a 53-day transfer.

## What changed

- Matched RW/MTB limits across Basilisk and 42.
- Full-mission wheel-speed and MTB dipole histories (`a3_rw_mtb_management.png`).
- Interpretation: Basilisk rejects environmental + thrust-misalignment torque entirely on the wheels; 42 tracks a reference trajectory so wheels grow slowly and MTB barely fires.

## Scripts

[06](../06_transfer_execution/) (`gnc_stack.py`, `SC_A3_RW.txt`).

## Results in this repo

| Metric | Basilisk | NASA 42 |
|--------|----------|---------|
| Peak wheel momentum [N m s] | 0.0116 | 0.089 |
| Peak momentum / limit [%] | 7.7 | 59.3 |
| RW torque saturation (spiral) [%] | 25.0 | — |
| Peak MTB dipole [A m²] | 5.00 | 1.1×10⁻⁵ |
