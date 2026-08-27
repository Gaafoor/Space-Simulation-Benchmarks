# Benchmark A3 - Imaging Opportunity Analysis

Status: Basilisk, GMAT, and Orekit A3 are closed for the selected RAAN 61 case. STK remains deferred.

## Scope

A3 extends Benchmark A with a camera/sensor geometry layer:

- Orbit: 500 km SSO-like LEO inherited from A1/A2
- Target: Madrid
- Duration: 24 h
- Camera model: 10 deg full-cone field of view
- Default pointing assumption: nadir pointing
- Output goal: imaging opportunity analysis, not rendered image generation

## GMAT Native Sensor-Contact Test

GMAT R2026a was tested to determine whether native `ContactLocator` access can be constrained by an attached camera/imager field of view.

The script successfully defines:

- `ConicalFOV A3_Camera_FOV`
- `Imager A3_Camera`
- `CubeSat_A3.AddHardware = {A3_Camera}`

This confirms that the GMAT script interface can represent an imager/FOV hardware object attached to the spacecraft.

Two native sensor-contact probes were then tested.

### Probe A - Imager as ContactLocator Target

Tested concept:

```text
Create ContactLocator A3_Camera_Contacts;
A3_Camera_Contacts.Target = A3_Camera;
A3_Camera_Contacts.Observers = {Madrid_GS_A3};
```

Observed GMAT error:

```text
Target for EventLocator A3_Camera_Contacts must be set to either a Spacecraft or a PlanetographicRegion.
```

Result: not supported through this `ContactLocator` target syntax.

### Probe B - Sensor Property on ContactLocator

Tested concept:

```text
Create ContactLocator A3_Sensor_Assigned_Contacts;
A3_Sensor_Assigned_Contacts.Target = CubeSat_A3;
A3_Sensor_Assigned_Contacts.Observers = {Madrid_GS_A3};
A3_Sensor_Assigned_Contacts.Sensor = A3_Camera;
```

Observed GMAT error:

```text
The field name "Sensor" on object "A3_Sensor_Assigned_Contacts" is not permitted.
```

Result: not supported through this `ContactLocator` property syntax.

## GMAT Interim Finding

Native sensor-contact workflow tested, not supported through `ContactLocator` using the attempted script interfaces.

The current evidence does not prove that GMAT has no sensor capability. It shows a narrower, evidence-based result:

> GMAT R2026a can define and attach an `Imager` with a `ConicalFOV`, but the tested `ContactLocator` workflow cannot directly produce camera-FOV-constrained Madrid imaging windows. For A3, GMAT imaging opportunities therefore require external post-processing of spacecraft state and target geometry unless a separate verified GMAT sensor-access API is found.

## Current GMAT Workflow

The working GMAT A3 script remains useful for:

- exporting dense spacecraft state history at 10 s cadence;
- exporting spacecraft-to-Madrid contact windows;
- documenting imager/FOV object setup;
- supporting external A3 FOV geometry post-processing.

The baseline spacecraft contact report is not the final imaging opportunity result, because it does not enforce the 10 deg camera FOV.

## GMAT/Python A3 Architecture

The current GMAT A3 benchmark is implemented as a hybrid workflow:

```text
GMAT
 |- Orbit propagation
 |- Ground station contacts
 `- State export

Python
 |- Madrid position
 |- Nadir vector
 |- Off-nadir angle
 |- FOV test
 `- Imaging opportunities
```

This architecture is intentional. GMAT is used for the mission-analysis/orbit-propagation part that it supports well, while Python performs the explicit 10 deg camera-FOV geometry test that was not available through the tested GMAT `ContactLocator` sensor workflow.

## RAAN Trade Study Result

A small RAAN trade study was run using `benchmarks/benchmark_a/results/raan_trade_study.csv`. Each case used the same A3 geometry rule:

- camera full FOV: 10 deg;
- half-angle imaging threshold: 5 deg;
- nadir-pointing camera assumption;
- Madrid target;
- GMAT state export followed by Python FOV post-processing.

From the recorded trade table, **RAAN = 61 deg** was selected as the best tested case because it is the only listed RAAN case that produced an imaging opportunity under the required off-nadir/FOV limit.

| RAAN [deg] | Pass count | Min off-nadir [deg] | Imaging opportunities | Best pass off-nadir [deg] | Max elevation [deg] |
|------------|------------|---------------------|-----------------------|---------------------------|---------------------|
| 61 | 2 | 2.175 | 1 | 2.175 | 87.653 |

The selected RAAN 61 case produced two Madrid contact passes. The first pass was visible but not imageable by the narrow nadir camera; the second pass passed almost directly over Madrid:

| Pass | UTC start | UTC stop | Duration [min] | Min off-nadir [deg] | Max elevation [deg] |
|------|-----------|----------|----------------|---------------------|---------------------|
| 1 | 02 Jun 2026 00:32:51.194 | 02 Jun 2026 00:39:25.500 | 6.57 | 52.487 | 31.172 |
| 2 | 02 Jun 2026 11:08:49.185 | 02 Jun 2026 11:16:11.768 | 7.38 | 2.175 | 87.653 |

Interim interpretation:

> For GMAT A3, RAAN 61 deg is the best tested RAAN because it produces one valid nadir-camera imaging opportunity for Madrid with a minimum off-nadir angle of approximately 2.17 deg, inside the 5 deg half-angle requirement. Other tested RAAN values produced Madrid contact opportunities but did not satisfy the fixed nadir 10 deg FOV imaging condition.

## Orekit A3 Implementation (New)

Orekit A3 was implemented with the same frozen A3 geometry assumptions and output products as Basilisk/GMAT post-processing:

- orbit: 500 km circular SSO-like case (RAAN 61 selected case);
- duration: 24 h;
- sample cadence: 10 s;
- camera: 10 deg full cone (5 deg half-angle), nadir-pointing;
- target: Madrid.

### Orekit pipeline architecture

```text
Orekit (Python-Java bridge)
 |- Numerical propagation (J2-style 2x0 harmonics)
 `- ECI state history at 10 s

Python
 |- Off-nadir geometry and FOV test
 |- Imaging windows and revisit
 |- Footprint and regional coverage grid
 `- Comparison-ready summary artifacts
```

### Orekit setup and difficulty notes

Compared with Basilisk and GMAT runs in this repository, Orekit had the highest setup overhead for first execution:

1. Install `orekit_jpype`.
2. Install a JVM (OpenJDK).
3. Download and load `orekit-data.zip`.
4. Validate JVM/data bootstrap before running A3 pipeline.

After setup, runtime execution was stable and reproducible.

### Orekit selected-case result snapshot (RAAN 61)

| Metric | Value |
|--------|-------|
| Imaging opportunities | 1 |
| Minimum off-nadir angle | 2.174550 deg |
| Max target elevation | 87.652590 deg |
| Madrid regional coverage | 67.03% |
| Total pipeline runtime | 9.056 s |
| Propagation runtime | 6.369 s |
| Post-processing runtime | 2.686 s |

Artifacts:

- `benchmarks/benchmark_a/run_orekit_a3_imaging_analysis.py`
- `benchmarks/benchmark_a/results/orekit_a3/summary.json`
- `data/benchmark_a/orekit_a3/orekit_a3_summary.json`
- `data/benchmark_a/orekit_a3/orekit_a3_run_log.md`
- `data/benchmark_a/orekit_a3/orekit_a3_evaluation_sheet.csv`

## Coverage and Heatmap Implementation Source

For this repository's A3 runs, coverage percentage, regional coverage statistics, and heatmap plots are generated by custom Python post-processing functions, not by simulator-native built-in coverage APIs:

- Basilisk: `run_basilisk_a3_imaging_analysis.py` computes coverage grid/heatmap in Python.
- GMAT: `run_gmat_a3_analysis.py` computes coverage grid/heatmap in Python from GMAT-exported state data.
- Orekit: `run_orekit_a3_imaging_analysis.py` computes coverage grid/heatmap in Python from Orekit-propagated state data.

This common post-processing approach is used intentionally so implementation difficulty and output comparability can be evaluated on the same geometry method.

## Remaining Open Work

- Add STK A3 evidence when STK is available.
- If needed, re-measure Basilisk A3 runtime in an environment where `Basilisk` module is available so all three tools include directly measured runtime splits.

## GMAT A3 Closure Artifacts

- `data/benchmark_a/gmat_a3/gmat_a3_summary.json`
- `data/benchmark_a/gmat_a3/gmat_a3_evaluation_sheet.csv`
- `data/benchmark_a/gmat_a3/gmat_a3_run_log.md`
- `data/benchmark_a/a3_imaging_metrics_comparison.csv`
- `benchmarks/benchmark_a/results/raan_trade_study.csv`
- `benchmarks/benchmark_a/results/RAAN_061/summary.json`
- `benchmarks/benchmark_a/results/RAAN_061/off_nadir_angle_vs_time.png`

## Basilisk A3 Closure Artifacts

- `data/benchmark_a/basilisk_a3/basilisk_a3_summary.json`
- `data/benchmark_a/basilisk_a3/basilisk_a3_run_log.md`
- `data/benchmark_a/basilisk_a3/basilisk_a3_evaluation_sheet.csv`
- `benchmarks/benchmark_a/results/basilisk_a3/summary.json`
- `benchmarks/benchmark_a/results/basilisk_a3/coverage_percentage.json`

## Orekit A3 Closure Artifacts

- `data/benchmark_a/orekit_a3/orekit_a3_summary.json`
- `data/benchmark_a/orekit_a3/orekit_a3_run_log.md`
- `data/benchmark_a/orekit_a3/orekit_a3_evaluation_sheet.csv`
- `benchmarks/benchmark_a/results/orekit_a3/summary.json`
