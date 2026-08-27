# Physics parity comparison (step 1)

## Force models
- **Baseline:** None
- **Parity:** {'gravity': 'JGM2-equivalent GGM2BData 20x20', 'atmosphere': 'NRLMSISE-00 (MsisAtmosphere)', 'drag': True, 'srp': True, 'third_bodies': ['Sun', 'Moon']}

## Key metrics
| Metric | Baseline | Parity | GMAT ref |
|--------|----------|--------|----------|
| Final SMA [km] | 42162.493 | 42018.796 | 42164.39112523785 |
| Day SMA≥42164 reached | 17.24 | 17.25 | 52.48 (spiral end) |
| Fuel used [kg] | 1.236 | 1.236 | 3.765698493845001 |
| Max BSP error [km] | 72164.5 | 74829.1 | 0 (reference) |
| Spiral stop reason | sma_target_coast | sma_target_coast | — |

Plots: `physics_parity_sma_vs_time.png`, `physics_parity_ephemeris_error.png`