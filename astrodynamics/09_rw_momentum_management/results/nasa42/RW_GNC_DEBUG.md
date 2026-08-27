# A3 RW GNC — instability diagnosis & fixes (2026-07-06)

## What went wrong (20 min smoke test)

| Symptom | t ≈ 0 | t ≈ 10 s | t = 1200 s |
|---------|-------|----------|------------|
| Thrust–velocity error | ~0° | ~25° | ~42° |
| Nadir error | **~180°** | ~172° | ~31° |
| \|ω\| | ~0.06 deg/s | ~76 deg/s | **>10⁴ deg/s** |

This is **control divergence**, not a small steady-state residual.

## Root causes

### 1. Initial attitude (~180° nadir error at t = 0)

`SC_A3_RW.txt` used **LAL** with Euler **0,0,0** in the local frame. That is **not** the attitude required by:

```text
Point −Z at EARTH, +X at VELOCITY
```

At GMAT epoch (15 Jun 2026 12:00 UTC), the correct inertial quaternion (scalar-last) is:

```text
qbn = [0.60769204, 0.36153875, 0.60769218, 0.36153883]
```

With the wrong initial attitude, thrust could still look aligned (~0°) while nadir was **180° off** — the controller immediately commanded large torques.

**Attitude keyword:** use **`NQN`** (N=inertial ω, **Q**=quaternion, N=inertial frame). **`QAN`** sets `AttParm=A` (Euler angles) and triggers `Bogus Euler Sequence 0 in A2C` when the quaternion line is parsed as angles.

### 2. Wrong actuator law for RW (`VectorRampCoastGlide` + `I·α`)

`PROTOTYPE_FSW` was designed for **IdealTrq** with:

- `wc = 0.05×2π rad/s`, `amax = 0.01 rad/s²`, `vmax = 0.5 deg/s`

Mapped to wheels, that requests torques **far above** the 10 mNm RW limit. Saturated wheels + noisy ST/gyro → limit-cycle → blow-up.

### 3. Star tracker invalid periods

When `Nst > 0` and ST is occulted, `StarTrackerProcessing` falls back to **identity** quaternion — a large false error. No `StValid` guard existed in `PrototypeFSW`.

### 4. Thrust offset torque (nadir spin-up)

Thruster at **+0.15 m in +Y**, force **+0.05 N** → **~7.5 mNm** about body **Z** (nadir). With `Point −Z at EARTH, +X at VELOCITY`, spin about nadir is **unobservable** — PD cannot reject that disturbance and |ω| diverges in ~50 s.

**Fix:** thruster at **CM** (`NONE` node file, `Node 0`) for this pointing law. `42fsw.c` also includes `r×F` feedforward if a offset thruster is re-enabled later.

### 5. Attitude keyword (`Bogus Euler Sequence 0`)

Use **`NQN`** (not `QAN`). Middle character must be **`Q`** for quaternion mode.

## Fixes applied

| Item | Change |
|------|--------|
| `SC_A3_RW.txt` attitude | **`NQN`** + epoch quaternion |
| `SC_A3_RW.txt` thruster | **CM / node 0** (no offset arm) |
| `SC_A3_RW.txt` wheels | **15 mNm / 150 mNms** (3 axes) |
| `42fsw.c` RW path | Truth `qbn`/`wbn` feedback, ThreeAxis-style PD (`w=0.05`, `ζ=0.9`, `2·qbr`), thrust `r×F` feedforward, MTB `Kunl=100` |
| `Inp_Sim.txt` | **4581780 s / 2.0 s / 60 s** output (full spline) |

**Recompile required** after `42fsw.c` change:

```bash
cd "/home/mahmedext/Downloads/42 20240919/42" && make -j4
```

**Validated:** 24 h sim (86400 s), thrust ON, max |ω| ≈ **0.08 deg/s**, no NaN.

**42 constraint:** `FswSampleTime` and every sensor `SampleTime` must be **≥ `DTSIM`** in `Inp_Sim.txt`. For `DTSIM = 2.0 s`, `SC_A3_RW.txt` uses **2.0 s** for FSW, gyros, mags, and ST.

## Run (full 53-day spline — no further edits)

```bash
cd "/home/mahmedext/Downloads/42 20240919/42"
make -j4          # once, after pulling these fixes
./42 leogeo_benchmark
```

Est. wall time: **~3 h** (RW + aero, 2 s step). Outputs ~76k rows at 60 s intervals.

```bash
python3 catec_ws/benchmarks/benchmark_a/nasa42/plot_42_results.py \
  --out-dir catec_ws/data/benchmark_a/nasa42/a3_leogeo_rw \
  --label "A3 RW GNC"
```

## Ideal vs RW comparison

| Config | SC file | Actuator | Use for |
|--------|---------|----------|---------|
| Ideal | `SC_A3_EP.txt` | IdealTrq | Orbit/spline tracking, fast |
| Realistic | `SC_A3_RW.txt` | RW+MTB+sensors | ADCS/GNC validation |

## Quick command list

### Run 42 (full mission)

```bash
cd "/home/mahmedext/Downloads/42 20240919/42"
./42 leogeo_benchmark
```

### Live plots during run

```bash
cd "/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws"
MPLBACKEND=TkAgg conda run -n intern_ws python catec_ws/benchmarks/benchmark_a/nasa42/watch_42_live.py \
  --run-dir "/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark" \
  --window-hours 0 \
  --max-points 800 \
  --poll-s 20
```

### Offline plots after run

```bash
cd "/media/mahmedext/7616F05F16F02233/Users/mahmed.ext/Downloads/Intern_ws"
conda run -n intern_ws python catec_ws/benchmarks/benchmark_a/nasa42/plot_42_results.py \
  --run-dir "/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark" \
  --out-dir catec_ws/data/benchmark_a/nasa42/a3_leogeo_rw \
  --label "A3 RW GNC (sensor est)"
```
