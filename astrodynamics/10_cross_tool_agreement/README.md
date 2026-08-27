# 10 — Cross-tool trajectory agreement against a common reference

## What this is

Basilisk (free 6-DOF + perturbations) and NASA 42 (propagated 6-DOF on a GMAT spline) are compared to the **same GMAT design ephemeris**. Semi-major-axis histories are the primary overlay (`a3_sma_compare.png`). A shorter physics-parity run reports ephemeris error vs the reference (`physics_parity_report.md`).

Basilisk continues raising SMA past the design target (free flight, ~100% duty). NASA 42 stays near the spline (final SMA 41844 km vs design 42164 km).

## Official example this started from

- **GMAT** ephemeris export (SPK/CSV) samples.
- **Basilisk** has no official “compare to GMAT OEM” example.
- **NASA 42** `EllipSpline.txt` / spline orbit types in Demo. Stock splines are not generated from this GMAT A3 transfer.

## What changed

- Shared burn schedule and reference CSV from [05](../05_leo_geo_ep_design/results/).
- `plot_basilisk_42_compare.py` and `compare_physics_parity.py` in [06](../06_transfer_execution/scripts/basilisk/).
- Spline builder `bsp_to_42_spline.py` (NASA 42 `a3_gmat_spline.txt`).

## Scripts

[05](../05_leo_geo_ep_design/) and [06](../06_transfer_execution/).

## Results in this repo

- `results/a3_sma_compare.png`, `a3_basilisk_nasa42_compare.png`, zoom PNG
- `results/physics_parity_report.md` / `.json` and parity plots
- `compare_manifest.json` from the full-mission rerun
