# Benchmark A2 Basilisk Run Log

## Status

- Result: pass
- Variant: A2 realistic mission (J2 + Madrid access + eclipse)

## Mission metrics

- observation_window_count: 4
- avg_observation_window_duration_min: 5.458333333333333
- total_contact_time_min: 21.833333333333332
- coverage_revisit_time_min: 94.16666666666667
- mean_revisit_time_min: 292.1666666666667
- eclipse_time_percent: 15.24305555555555

## Plots

- data/benchmark_a/basilisk_a2/plots/basilisk_a2_mission_timeline.png
- data/benchmark_a/basilisk_a2/plots/basilisk_a2_madrid_passes_gantt.png
- data/benchmark_a/basilisk_a2/plots/basilisk_a2_orbit_eci.png

## Summary

- benchmark_id: A2
- simulator: Basilisk
- basilisk_version: 2.10.2
- python_version: 3.12.10
- platform: Windows-11-10.0.26200-SP0
- dynamics_model: Earth J2 spherical harmonics (GGM03S J2-only, degree 2)
- spice_used: yes
- j2_enabled: yes
- drag_enabled: no
- srp_enabled: no
- third_body_enabled: no
- ground_target: Madrid
- minimum_elevation_deg: 10.0
- simulated_duration_s: 86400.0
- wall_clock_runtime_s: 0.39267490000929683
- runtime_ratio: 220029.34233370767
- observation_window_count: 4
- avg_observation_window_duration_min: 5.458333333333333
- total_contact_time_min: 21.833333333333332
- coverage_revisit_time_min: 94.16666666666667
- mean_revisit_time_min: 292.1666666666667
- eclipse_time_percent: 15.24305555555555
- passes_csv: data/benchmark_a/basilisk_a2/basilisk_a2_madrid_passes.csv
- timeline_csv: data/benchmark_a/basilisk_a2/basilisk_a2_timeline.csv
- state_samples_csv: data/benchmark_a/basilisk_a2/basilisk_a2_state_samples.csv
