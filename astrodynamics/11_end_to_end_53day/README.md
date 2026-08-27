# 11 — Long-duration end-to-end transfer campaign (~53 days)

## What this is

Roll-up of the electric-spiral campaign: ~52.5 days of thrusting inside a ~53.2-day horizon. Schedule ingestion matches the exported burn window within 1 s in both executors. This folder stores the **campaign-level metrics and plots**; it does not duplicate the Python/42 sources.

## Official example this started from

No official GMAT, Basilisk, or NASA 42 sample runs a 53-day LEO–GEO EP **design → execution → ADCS/power/fuel** campaign with cross-tool tables. Closest pieces are the short EP/spiral/Demo cases cited in [05](../05_leo_geo_ep_design/) and [06](../06_transfer_execution/).

## What changed

End-to-end bookkeeping: final SMA vs design, fuel vs 3.766 kg design, duty cycle, eclipse fraction, peak pointing, thrust/eclipse pattern (penumbra dips vs binary pulses).

## Scripts

[05](../05_leo_geo_ep_design/) and [06](../06_transfer_execution/).

## Results in this repo

| Campaign metric | Basilisk | NASA 42 |
|-----------------|----------|---------|
| Mission horizon [days] | 53.2 | 53.2 |
| Final SMA [km] | 50227.1 | 41844.5 |
| SMA error vs design [km] | +8063 | −319 |
| Fuel [kg] | 3.969 | 3.767 |
| Mean spiral thrust [mN] | 17.6 | 15.0 |
| Mean spiral power [kW] | 0.819 | 0.713 |
| Thrust duty [%] | 100.0 | 87.1 |
| Max thrust-pointing [deg] | 2.311 | 8.724 |

- `results/basilisk/summary.json`, `orbit_semi_major_axis.png`
- `results/nasa42/validation_summary.md`, `a3_orbit_vs_gmat.png`
