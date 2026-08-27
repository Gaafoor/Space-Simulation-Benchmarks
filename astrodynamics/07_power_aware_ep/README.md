# 07 — Power-aware electric propulsion (dynamic solar and eclipse)

## What this is

Constant effective-power EP is replaced by power that depends on Sun distance, eclipse state, and panel coefficients. Thrust and mass-flow follow the same polynomials in Basilisk and NASA 42; **eclipse gating differs**.

- Basilisk: continuous dual-cone penumbra illumination → ~100% thrust duty, mean spiral thrust 17.2 mN, 0.819 kW.
- NASA 42: binary sun-on/sun-off → ~87% duty, ~12.8% eclipse-off, 15.0 mN, 0.713 kW.

## Official example this started from

- **GMAT** `SolarPowerSystem` / eclipse samples (native in the A3 design script).
- **Basilisk** `scenarioPowerDemo.py`, `scenarioAttitudeFeedbackRWPower.py`. Those demos show a power module on a short scenario, not a 53-day EP spiral with GMAT-matched polynomials and a 42 binary-eclipse counterpart.
- **NASA 42** eclipse flag on the spacecraft / CSS illumination. Stock Demo does not implement the A3 thrust–power polynomials.

## What changed

- Shared solar/thrust polynomials (`solar_power_model.py` in [06](../06_transfer_execution/scripts/basilisk/)).
- Basilisk dual-cone penumbra vs 42 binary eclipse, with paired thrust/power/eclipse time series (`a3_power_thrust_profile.png`).

## Scripts

Use the pipeline in [05](../05_leo_geo_ep_design/) and [06](../06_transfer_execution/). Dynamic-solar Basilisk artifacts come from the `gmat_gnc_handoff_dynamic_solar` run.

## Results in this repo

- `results/basilisk/summary.json` and comparison plots vs NASA 42
- `results/a3_power_thrust_profile.png` (copied under basilisk results)

| Metric | Basilisk | NASA 42 |
|--------|----------|---------|
| Mean spiral thrust [N] | 0.01722 | 0.01499 |
| Mean spiral power [kW] | 0.819 | 0.713 |
| Thrust duty fraction | 1.000 | 0.871 |
| Eclipse fraction [%] | 0.04 | 12.77 |
| Fuel [kg] | 3.969 | 3.767 |
