# Benchmark A3 GMAT Run Log

## Status

- Result: pass for GMAT A3 selected case
- Selected case: RAAN 61 deg
- Date documented: 2026-06-08
- Workflow: GMAT state/contact export plus Python FOV post-processing

## What GMAT Provided

- Orbit propagation using the A2 J2 policy
- Dense spacecraft state export
- Madrid spacecraft-to-ground contact report
- `Imager` and `ConicalFOV` object setup
- OpenFrames/GroundTrack visualization context

## Native Sensor-Contact Probe Result

Native sensor-contact workflow was tested through `ContactLocator` and was not supported by the tested syntax:

- `ContactLocator.Target = A3_Camera` failed because the target must be a `Spacecraft` or `PlanetographicRegion`.
- `ContactLocator.Sensor = A3_Camera` failed because `Sensor` is not a permitted `ContactLocator` field.

Therefore, GMAT A3 uses Python post-processing for the 10 deg camera FOV imaging test.

## Coverage and Heatmap Source

- In this repository's A3 implementation, coverage percentage and heatmap outputs are computed by custom Python functions in `benchmarks/benchmark_a/run_gmat_a3_analysis.py`.
- GMAT provides state propagation and contact exports; it is not producing these A3 coverage/heatmap products natively in the current workflow.

## Selected RAAN Result

RAAN 61 deg was selected from `benchmarks/benchmark_a/results/raan_trade_study.csv`.

| Metric | Value |
|--------|-------|
| Pass count | 2 |
| Imaging opportunities | 1 |
| Minimum off-nadir angle | 2.174576 deg |
| Best pass off-nadir angle | 2.174576 deg |
| Maximum elevation | 87.652561 deg |

## Pass Summary

| Pass | UTC start | UTC stop | Duration [min] | Min off-nadir [deg] | Max elevation [deg] | Imaging candidate |
|------|-----------|----------|----------------|---------------------|---------------------|-------------------|
| 1 | 02 Jun 2026 00:32:51.194 | 02 Jun 2026 00:39:25.500 | 6.57 | 52.49 | 31.17 | No |
| 2 | 02 Jun 2026 11:08:49.185 | 02 Jun 2026 11:16:11.768 | 7.38 | 2.17 | 87.65 | Yes |

## Artifacts

- `data/benchmark_a/gmat_a3/gmat_a3_summary.json`
- `data/benchmark_a/gmat_a3/gmat_a3_evaluation_sheet.csv`
- `data/benchmark_a/A3_COMPARISON_REPORT.md`
- `benchmarks/benchmark_a/results/raan_trade_study.csv`
- `benchmarks/benchmark_a/results/RAAN_061/summary.json`
- `benchmarks/benchmark_a/results/RAAN_061/off_nadir_angle_vs_time.png`
