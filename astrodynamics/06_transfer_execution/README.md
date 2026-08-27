# 06 — Realistic transfer execution (design vs execution)

## What this is

The GMAT electric-spiral **schedule** from [05](../05_leo_geo_ep_design/) is executed in higher-fidelity 6-DOF environments:

- **Basilisk** — free orbit propagation with perturbations, closed-loop thrust pointing, RW/MTB.
- **NASA 42** — spline-guided reference (`a3_gmat_spline.txt`), four-wheel pyramid, facet mesh aero/SRP, EGM96 18×18, gyros + star tracker.

Both match the commanded burn window within 1 s and stay within 0.1% of design propellant in the 42 case; Basilisk uses 3.969 kg (+5.4%). Final SMA differs because Basilisk flies free while 42 tracks the GMAT spline (see [10](../10_cross_tool_agreement/) and [11](../11_end_to_end_53day/)).

**This folder holds the canonical Basilisk pipeline and NASA 42 decks used by report §1.6–1.11.**

## Official example this started from

- **Basilisk** `examples/scenarioInertialSpiral.py` and GNC examples (`scenarioAttitudeFeedbackRW.py`, `scenarioMtbMomentumManagement.py`). Those demos spiral or point in isolation; they do not ingest a GMAT burn schedule, solar-power polynomials, or a 53-day handoff.
- **NASA 42** `Demo/` / `InOut/` spacecraft + `PROTOTYPE_FSW`. Stock Demo is a short multi-body showcase, not a GMAT-ephemeris spline, A3 facet mesh, or 53-day EP campaign.

## What changed

- Python stack: `run_basilisk_gmat_gnc_handoff.py` plus `gnc_stack.py`, `thruster_stack.py`, `force_model.py`, `solar_power_model.py`, `spacecraft_geometry.py`.
- NASA 42 decks: `Inp_Sim.txt` (~53 day FAST run), `Orb_A3_GMAT.txt`, `SC_A3_RW.txt`, `Nodes_A3_EP.txt`, `A3_facet_L1.obj`, spline file.
- Helpers: `bsp_to_42_spline.py`, `generate_42_inp_cmd.py`, `42fsw_a3_pointing_torque.patch`.
- Design-to-execution tables (SMA, fuel, burn timing, peak pointing).

## How to run

Basilisk (from a machine with Basilisk installed):

```bash
cd scripts/basilisk
python3 run_basilisk_gmat_gnc_handoff.py
```

NASA 42: copy `scripts/nasa42/` over a 42 `InOut` (or run with that directory as the input path), set `NASA42_ROOT`, then:

```bash
cd "$NASA42_ROOT"
./42 /path/to/this/scripts/nasa42
```

Use `Inp_Sim_smoke_2h.txt` for a 2 h check. Overlay remaining stock 42 files (graphics maps, etc.) from your 42 install; only custom decks are stored here.

## Results in this repo

- Basilisk: `results/basilisk/summary.json`, orbit/pointing PNGs, `compare_nasa42/`
- NASA 42: `results/nasa42/validation_summary.md` and ADCS/orbit PNGs (full `a3_42_telemetry.csv` is not included; ~26 MB)

| Metric | Basilisk | NASA 42 |
|--------|----------|---------|
| Final SMA [km] | 50227.1 | 41844.5 |
| Fuel [kg] | 3.9692 | 3.7668 |
| Burn start timing error [s] | 0.0 | 0.0 |
| Max thrust-pointing error [deg] | 2.311 | 8.724 |
