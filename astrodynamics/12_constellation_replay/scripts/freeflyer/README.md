# CubeSat Cairo / Madrid / Paris — FreeFlyer Milestone 1

Kinematic attitude mission (US slew-example style): Sun-point between passes, track cities above 10° elevation, capture images in 45–80° when boresight error &lt; 2°.

## Quick start

1. Read **[INSTRUCTIONS.md](INSTRUCTIONS.md)** and create GUI objects.
2. Use **six** FreeForm blocks in order (or open `CubeSat_Cairo_Madrid_Paris.MissionPlan` if already patched).
3. Set orbit: **500 km**, **i = 97.4°**, epoch **02 Jun 2026**, step **15 s**.
4. Run; review plots + **[PLOTTING.md](PLOTTING.md)** for tuning.

## Scripts

| File | Purpose |
|------|---------|
| `01_console_setup.ff` | Console appearance + intro text |
| `02_compute_target_attitude.ff` | Sun / Earth / ground-station aim |
| `03_slew_spacecraft.ff` | Rate-limited slew (2 deg/s) |
| `04_create_objects_init.ff` | Constants, flags, initial Sun snap |
| `05_setup_plots.ff` | PlotWindows + MissionDebugLog DataTable |
| `06_visualize_and_propagate.ff` | Map/3D, loop, plot updates, PNG export |
