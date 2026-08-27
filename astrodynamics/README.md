# Astrodynamics benchmarks

Report §1. Folders are numbered in the same order as the internship experience report.

| # | Folder | Starts from (typical official sample) | What this repo adds |
|---|--------|----------------------------------------|---------------------|
| 01 | [two-body validation](01_two_body_validation/) | GMAT two-body propagate; Basilisk `scenarioBasicOrbit.py` | Common ICs, Kepler analytical truth, GMAT–Basilisk error table |
| 02 | [perturbation ladder](02_perturbation_ladder/) | GMAT force-model samples; Basilisk basic orbit | Five-case ladder, cross-tool drift metrics |
| 03 | [Madrid contacts / eclipse](03_madrid_contacts_eclipse/) | GMAT contact/eclipse samples; Basilisk ground-location examples | Madrid GS, 24 h pass/eclipse metrics |
| 04 | [EO imaging / RAAN](04_eo_imaging_raan/) | Basilisk `scenarioGroundLocationImaging.py` | RAAN sweep, off-nadir, regional coverage |
| 05 | [LEO–GEO EP design](05_leo_geo_ep_design/) | GMAT electric / finite-burn samples | SMA-targeted ~52.5 day spiral, burn schedule export |
| 06 | [transfer execution](06_transfer_execution/) | Basilisk `scenarioInertialSpiral.py`; NASA 42 `Demo` | Execute GMAT schedule with ADCS, mesh, EGM96 |
| 07 | [power-aware EP](07_power_aware_ep/) | GMAT solar-power objects | Penumbra vs binary eclipse, duty cycle |
| 08 | [attitude / thrust pointing](08_attitude_thrust_pointing/) | Basilisk MRP + RW examples; 42 PrototypeFSW | Thrust-to-velocity pointing on the spiral |
| 09 | [RW / momentum](09_rw_momentum_management/) | Basilisk RW/MTB examples | Four-wheel pyramid + MTB unload on the campaign |
| 10 | [cross-tool agreement](10_cross_tool_agreement/) | GMAT ephemeris export | Basilisk / 42 vs shared reference |
| 11 | [53-day campaign](11_end_to_end_53day/) | (same pipeline as 05–06) | Fuel, SMA, duty, pointing roll-up |
| 12 | [constellation / replay](12_constellation_replay/) | GMAT constellation; Basilisk `scenarioSatelliteConstellation.py` | Five-sc synchronized force-model cases |

Scripts for 07–11 are not duplicated. Use [05](05_leo_geo_ep_design/) and [06](06_transfer_execution/).
