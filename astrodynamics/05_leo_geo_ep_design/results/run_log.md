# GMAT A-3 LEO → GEO transfer — run log

## Script

`catec_ws/benchmarks/benchmark_a/gmat/scripts/Benchmark_A3_LEO_GEO_Transfer.script`

## Approach (current run)

| Step | GMAT object |
|------|-------------|
| Orbit raise | `FiniteBurn` + continuous low-thrust spiral until `SMA = 42164 km` |
| Post-spiral | Propagate to `Z = 0`, then 12 h coast (not part of SMA milestone) |
| Forces | Earth 20×20 JGM2 + drag + SRP + Sun/Moon (A-0 case 5 pattern) |
| Ground | `GroundStation` Madrid + `ContactLocator` |

Legacy `DifferentialCorrector` / impulsive plane-change objects remain in the script but are **not executed** in this run (discrete targeting did not converge; fuel limited).

## Frozen initial state

- Epoch: `2026-06-15T12:00:00.000Z`
- LEO SMA: 6878.137 km, i = 28.5 deg
- GEO target SMA: 42164.0 km (SMA-only scope for this milestone)

## SMA achievement (scoped success criterion)

Scoped goal: converge semi-major axis to GEO altitude only. Eccentricity and inclination are out of scope for this milestone.

| Field | Value |
|-------|-------|
| **Goal** | SMA = 42164 km (GEO altitude) |
| **Achieved at** | `eventCode` 11 — end of spiral burn |
| **Mission epoch** | 15 Jun 2026 12:00:00.000 UTC |
| **Elapsed time** | 4534018.822 s (52.477 days) |
| **UTC epoch** | 06 Aug 2026 23:26:58.822 UTC |
| **SMA at event** | 42164.000 km |
| **SMA-only converged** | Yes |

### Source artifacts

| Role | Path |
|------|------|
| Event record | `catec_ws/data/benchmark_a/gmat/a3_leo_geo_transfer/spiral_events.txt` |
| Ephemeris (valid at this epoch) | `catec_ws/data/benchmark_a/gmat/a3_leo_geo_transfer/a3_reference.bsp` |

### Notes

- **Authoritative milestone:** `spiral_events.txt` row with `eventCode = 11`, immediately after `EndFiniteBurn`.
- **Not the achievement epoch:** `gmat_final.txt` is after post-spiral propagation (`Z = 0`) and 12 h coast; SMA drifts slightly from 42164.000 km.
- **UTC epoch derivation:** mission epoch + `ElapsedSecs` at event 11.

### State at event 11 (traceability only)

| Quantity | Value |
|----------|-------|
| Total mass | 13.234 kg |
| Eccentricity | 0.0765 |
| Inclination | 28.31 deg |

## End-of-script result (`gmat_final.txt`)

| Metric | Value |
|--------|-------|
| Elapsed time | 4581897.270 s |
| Final SMA | 42164.391 km |
| Final e | 0.0766 |
| Final i | 28.31 deg |
| Final mass | 13.234 kg |

## Exports

| File | Role |
|------|------|
| `a3_reference.bsp` | SPK ephemeris (full run) |
| `spiral_events.txt` | Mission milestones |
| `gmat_state.csv` | State samples (t = 0 + post-GEO coast) |
| `gmat_final.txt` | End-of-script summary |
| `madrid_contacts.txt` | Ground contacts (15–20 Jun 2026 window) |
