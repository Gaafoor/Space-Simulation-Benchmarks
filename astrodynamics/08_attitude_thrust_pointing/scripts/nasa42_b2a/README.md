# NASA 42 — B2A nominal GNC decks

24 h sun-pointing CubeSat decks used as the **short** attitude-GNC precursor (report §1.8 / intern Benchmark B2A). The 53-day transfer ADCS decks are in `astrodynamics/06_transfer_execution/scripts/nasa42/`.

| File | Purpose |
|------|---------|
| `Inp_Sim.txt` | 24 h, 1 s step, J2, DE430 |
| `Orb_B2A_LEO.txt` | 500 km circular SSO |
| `SC_B2A_CubeSat.txt` | 12 kg bus, 4-wheel pyramid |
| `Inp_Cmd.txt` | Sun-pointing (+Z→SUN, +Y→VELOCITY) |

From `astrodynamics/08_attitude_thrust_pointing/scripts/`:

```bash
python3 run_nasa42_b2_attitude_gnc.py --case b2a
```

Set `NASA42_ROOT` to your NASA 42 install. Overlay these files on that install’s input directory. Stock 42 maps and remaining Demo files are not included.
