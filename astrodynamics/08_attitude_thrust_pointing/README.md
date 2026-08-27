# 08 — Attitude-control and thrust-pointing performance

## What this is

Closed-loop ADCS keeps thrust aligned with inertial velocity during the electric spiral. Basilisk peak thrust-axis error **2.311°** (mean ~2.22°). NASA 42 peak **8.724°** (mean ~4.45°) with four RW, three-axis gyros, star tracker, and mesh environmental torques.

A shorter **B2A** 24 h sun-pointing CubeSat case (Basilisk MRP + NASA 42 PrototypeFSW) is included under `scripts/` as the earlier attitude-GNC ladder.

## Official example this started from

- **Basilisk** `scenarioAttitudeFeedbackRW.py`, `scenarioAttitudePointing.py`, `BskSim/scenarios/scenario_AttFeedback.py`. Official examples slew to a reference on a short horizon; they do not track velocity-aligned thrust for 53 days on a GMAT burn schedule.
- **NASA 42** `Demo` + `PROTOTYPE_FSW` pointing commands. Stock pointing is not the A3 EP thrust-axis vs velocity metric.

## What changed

- Thrust-to-velocity (and nadir) error histories for the full campaign.
- B2 decks: `scripts/nasa42_b2a/` (500 km SSO, 4-wheel pyramid, 24 h) plus Basilisk `run_basilisk_b2_attitude_gnc.py`.
- Campaign pointing plots: `a3_basilisk_adcs_pointing.png`, `a3_adcs_pointing.png`.

## Scripts

Campaign execution: [06](../06_transfer_execution/).  
B2 precursor:

```bash
cd scripts
python3 run_basilisk_b2_attitude_gnc.py --case b2a
# NASA 42: NASA42_ROOT must point at your 42 install
python3 run_nasa42_b2_attitude_gnc.py --case b2a
```

## Results in this repo

| Pointing metric | Basilisk | NASA 42 |
|-----------------|----------|---------|
| Max thrust-pointing error [deg] | 2.311 | 8.724 |
| Mean thrust-pointing error [deg] | 2.222 | 4.448 |
| P95 thrust-pointing error [deg] | 2.267 | 5.917 |
| Mean / max nadir error [deg] | — | 2.285 / 6.049 |
| Star-tracker valid fraction [%] | — | 90.36 |
