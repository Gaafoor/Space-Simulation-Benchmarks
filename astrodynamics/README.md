# Astrodynamics benchmarks

Report §1. Folders numbered in report order.

| # | Folder | Original files | Changed |
|---|--------|----------------|---------|
| 01 | [two-body validation](01_two_body_validation/) | GMAT two-body samples; Basilisk `scenarioBasicOrbit.py` | Common ICs, Kepler truth, cross-tool CSV |
| 02 | [perturbation ladder](02_perturbation_ladder/) | GMAT force-model samples; Basilisk basic orbit | Five-case ladder, cross-tool drift |
| 03 | [Madrid contacts / eclipse](03_madrid_contacts_eclipse/) | GMAT contact/eclipse; Basilisk ground-location examples | Madrid GS, 24 h metrics |
| 04 | [EO imaging / RAAN](04_eo_imaging_raan/) | Basilisk `scenarioGroundLocationImaging.py` | RAAN sweep, coverage metrics |
| 05 | [LEO–GEO EP design](05_leo_geo_ep_design/) | GMAT electric / CSALT samples | SMA-targeted spiral, burn export |
| 06 | [transfer execution](06_transfer_execution/) | Basilisk `scenarioInertialSpiral.py`; NASA 42 `Demo` | GMAT schedule + ADCS, mesh, EGM96 |
| 07 | [power-aware EP](07_power_aware_ep/) | GMAT solar power; Basilisk power demos | Penumbra vs binary eclipse |
| 08 | [attitude / thrust pointing](08_attitude_thrust_pointing/) | Basilisk RW/MRP; 42 PrototypeFSW | Thrust-axis pointing on spiral |
| 09 | [RW / momentum](09_rw_momentum_management/) | Basilisk RW/MTB; 42 wheel decks | Matched limits on campaign |
| 10 | [cross-tool agreement](10_cross_tool_agreement/) | GMAT ephemeris export; 42 splines | vs shared GMAT reference |
| 11 | [53-day campaign](11_end_to_end_53day/) | 05 + 06 pipeline pieces | Fuel, SMA, duty roll-up |
| 12 | [constellation / replay](12_constellation_replay/) | GMAT/Basilisk constellation examples | Five-sc force-model ladder |

Scripts for 07–11: [05](05_leo_geo_ep_design/) and [06](06_transfer_execution/).
