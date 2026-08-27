# FreeFlyer Milestone 1 — GUI Setup Instructions

CubeSat sequential imaging (Cairo, Madrid, Paris) with Sun-pointing between passes.  
Scripts live in `freeforms/` — paste each file into **five separate FreeForm blocks** in order.

---

## 1. Create a new Mission Plan

1. Open FreeFlyer 7.10+.
2. **File → New → Mission Plan**.
3. Save as e.g. `CubeSat_Cairo_Madrid_Paris.MissionPlan`.

---

## 2. Mission sequence (FreeForm blocks only)

Add **five FreeForm** commands to the Mission Sequence in this order:


| Order | Label (suggested)       | Source file                                   |
| ----- | ----------------------- | --------------------------------------------- |
| 1     | `ConsoleSetup`          | `freeforms/01_console_setup.ff`               |
| 2     | `ComputeTargetAttitude` | `freeforms/02_compute_target_attitude.ff`     |
| 3     | `SlewSpacecraft`        | `freeforms/03_slew_spacecraft.ff`             |
| 4     | `CreateObjectsInit`     | `freeforms/04_create_objects_init.ff`         |
| 5     | `VisualizeAndPropagate` | `freeforms/05_visualization_and_propagate.ff` |


**Important:** Blocks 2 and 3 define procedures. Block 4 calls them at init. Block 5 runs the main `While` loop. Do not add extra Step/While commands in the GUI sequence — stepping is inside FreeForm 5.

---

## 3. Celestial objects (defaults)

Use the built-in **Earth** and **Sun** objects. No changes required for milestone 1.

---

## 4. Ground stations

Create three **GroundStation** objects with these approximate coordinates:


| Name     | Latitude | Longitude | Notes                                             |
| -------- | -------- | --------- | ------------------------------------------------- |
| `Cairo`  | 30.044°  | 31.236°   | Egypt                                             |
| `Madrid` | 40.417°  | 356.296°  | Use 356.296° (= −3.704°) if FreeFlyer uses 0–360° |
| `Paris`  | 48.857°  | 2.352°    | France                                            |


**GUI steps (each station):**

1. **Add Object → GroundStation**.
2. Set **Name** exactly: `Cairo`, `Madrid`, or `Paris` (must match scripts).
3. Set lat/lon on the Properties page.
4. Pick a distinct **Color** (used in Console messages).

---

## 5. CubeSat spacecraft (6U-style)

### 5.1 Create spacecraft

1. **Add Object → Spacecraft**.
2. Name: `**CubeSat`** (exact spelling).

### 5.2 Mass and body (representative 6U)


| Property                 | Suggested value                                                                           |
| ------------------------ | ----------------------------------------------------------------------------------------- |
| Mass                     | **12 kg**                                                                                 |
| Attitude reference frame | **LVLH** or **ICRF** at creation (scripts set orientation each step via `SetOrientation`) |


You do **not** need reaction wheels, momentum dumps, or actuator models for milestone 1. Slewing is kinematic via `SlewSpacecraft`.

### 5.3 Propagator


| Property        | Value                                        | Why                                                                     |
| --------------- | -------------------------------------------- | ----------------------------------------------------------------------- |
| Propagator type | **RK89** (or default numerically integrator) | Standard LEO                                                            |
| Step size       | **15 sec**                                   | Catches 45–80° elevation window; avoids skipping passes with 60 s steps |
| Central body    | Earth                                        | —                                                                       |


### 5.4 Initial orbit (SSO-like baseline)

Set **Keplerian elements** at epoch **02 Jun 2026 00:00:00 UTC** (adjust epoch format to your FreeFlyer version):


| Element                        | Value                               |
| ------------------------------ | ----------------------------------- |
| Semi-major axis                | **6878.137 km** (≈ 500 km altitude) |
| Eccentricity                   | **0.001**                           |
| Inclination                    | **97.4°**                           |
| RAAN                           | **Start with 0°** — tune (see §8)   |
| Argument of perigee            | **0°**                              |
| True anomaly (or Mean anomaly) | **Start with 0°** — tune (see §8)   |


Equivalent: altitude **500 km**, sun-synchronous-like inclination **97.4°**.

### 5.5 Attach a sensor (for footprint / swath)

1. Open **CubeSat → Sensors**.
2. **Add Sensor**.
3. Suggested settings:


| Property         | Value                                                                 |
| ---------------- | --------------------------------------------------------------------- |
| Name             | e.g. `Camera` (first sensor = index `**[1]`** in script)              |
| Type             | Conical (or default)                                                  |
| Half-angle / FOV | **15°** (30° full cone) — illustrative                                |
| Boresight        | **+X body axis** (aligns with `BuildVector(4, CubeSat, 1)` in script) |


The script uses `CubeSat.Sensors[1]`. If your first sensor is not index 1, change the index in `05_visualization_and_propagate.ff`.

---

## 6. Coordinate system object

1. **Add Object → CoordinateSystem**.
2. Name: `**targetCS`**.
3. No manual alignment needed — scripts rebuild it every step.

---

## 7. Visualization objects

### 7.1 Map view

1. Ensure a **Map** view exists (default name `**MapView`**).
2. Scripts add a viewpoint named `map` and activate it.

### 7.2 3D view

1. Ensure a **View** window exists (default name `**ThreeDView`**).
2. Scripts add a viewpoint `view` centered on `CubeSat`.

If your window names differ, edit `MapView` / `ThreeDView` in `05_visualization_and_propagate.ff` to match.

### 7.3 Graphics overlay (red sensor swath)

1. **Add Object → GraphicsOverlay** (or add overlay to Map).
2. Name: `**sensorSwath`**.

### 7.4 Window overlay (HUD text)

1. **Add Object → WindowOverlay** attached to Map or View.
2. Name: `**SensorTarget`**.

---

## 8. Tune orbit so all three cities are imaged

The scripts run up to **5 days**. If Console never shows `Imaging Cairo!` / `Madrid` / `Paris`, the ground track is not passing cities in the **45–80°** window.

### What to tune (in order)

1. **RAAN** — shifts which longitudes each orbit crosses. Change in **±5° to ±15°** steps.
2. **True / Mean anomaly at epoch** — shifts where the sat is along the orbit at start.
3. **Simulation length** — already 5 days in script; increase in FreeForm 5 if needed.
4. **Argument of perigee** — secondary fine-tune.

### Quick verification (before full attitude logic)

1. Temporarily comment out the `While` loop in FreeForm 5 (or make a test plan).
2. Propagate `CubeSat` for 5 days with GUI **Step** or a short FreeForm: `While (CubeSat.ElapsedTime < TimeSpan.FromDays(5)); Step CubeSat; End;`
3. Plot `**Cairo.Elevation(CubeSat)`**, `**Madrid.Elevation(CubeSat)**`, `**Paris.Elevation(CubeSat)**` vs time.
4. Confirm each curve exceeds **45°** at least once.

Inclination **97.4°** already covers latitudes 30–49°N; missing cities are almost always **RAAN / anomaly / duration**, not inclination.

---

## 9. Mission behavior reference

### Attitude priority (each 15 s step)

1. **Image** — uncaptured city, elevation **45–80°**, boresight error **< 2°** → capture + Console message.
2. **Track** — uncaptured city, elevation **≥ 10°** → slew toward city (prepare for image window).
3. **Sun** — sunlit, no active pass → battery charging.
4. **Earth** — eclipse.

Tie-break when multiple cities qualify: **Cairo → Madrid → Paris**.

After all three `Imaging ...!` messages: **Sun-only** until sim end.

### Constants (edit in `04_create_objects_init.ff`)


| Constant                 | Default   |
| ------------------------ | --------- |
| `SLEW_RATE`              | 2 deg/s   |
| `ACQUISITION_ELEV`       | 10°       |
| `IMAGE_ELEV_MIN` / `MAX` | 45° / 80° |
| `SETTLE_ANGLE`           | 2°        |


---

## 10. Run checklist

- [ ] Objects exist: `CubeSat`, `Cairo`, `Madrid`, `Paris`, `Earth`, `Sun`, `targetCS`, `sensorSwath`, `SensorTarget`
- [ ] `CubeSat` propagator step = **15 s**
- [ ] Sensor attached; script index `[1]` matches
- [ ] Five FreeForms pasted in order
- [ ] **Run** Mission Plan
- [ ] Console shows slews and `Imaging ...!` for all three cities
- [ ] Red swath on map during ground tracking

---

## 11. Troubleshooting


| Symptom                                | Likely cause                    | Fix                                                  |
| -------------------------------------- | ------------------------------- | ---------------------------------------------------- |
| Compile error on `MapView`             | Window renamed                  | Match names in FreeForm 5                            |
| `Sensors[1]` error                     | No sensor or wrong index        | Add sensor or fix index                              |
| Never captures images                  | Orbit / RAAN                    | §8 elevation plots                                   |
| Captures but boresight err always > 2° | Slew too slow or step too large | Increase `SLEW_RATE` slightly or reduce step to 10 s |
| Skips image window                     | Step too large                  | Use 15 s or 10 s steps                               |
| Stuck on Sun over city                 | Elevation < 10° or > 80°        | Wait for next pass; tune orbit                       |


---

## 12. File map

```
freeflyer-milestone1/
├── INSTRUCTIONS.md          ← this file
└── freeforms/
    ├── 01_console_setup.ff
    ├── 02_compute_target_attitude.ff
    ├── 03_slew_spacecraft.ff
    ├── 04_create_objects_init.ff
    └── 05_visualization_and_propagate.ff
```

---

## 13. Milestone 2 (not in this package)

- Coverage heatmap  
- Real image products  
- Reaction wheel torque / saturation  
- Sun keep-out cones  
- Azimuth constraints

