# 07 — Basilisk OpNav (Hough vs CNN)

## What this is

Two Basilisk OpNav cases on a Mars elliptical orbit (a = 18000 km, e = 0.6): limb Hough circles (`scenario_OpNavAttOD.py`) and CNN limb/measurement (`scenario_CNNAttOD.py`). Both close the loop from camera imagery to attitude/orbit estimates.

Circ position RMS 0.55 vs CNN 5.24; Circ velocity RMS 1.41 vs CNN 13.18; CNN runtime ~2.3× Circ.

## Official upstream (AVSLab Basilisk)

| Official file | Role |
|---------------|------|
| `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavAttOD.py` | Hough-circle limb; combined attitude + orbit determination |
| `examples/OpNavScenarios/scenario_CNNAttOD.py` | CNN-based attitude + orbit determination |

Both scripts are **official demos**. This repo does not fork or store them.

## Files in this repo

| Content | Status |
|---------|--------|
| `results/opnav_summary.json` | **Custom** — paired metrics table |
| `results/bsk_opnav_circ_plots.png`, `bsk_opnav_cnn_plots.png` | **Custom** — side-by-side panels |

## What changed vs official Basilisk

- Paired execution on the **same** Mars orbit ICs and a single metrics table (runtime, pos/vel RMS, image method).
- Side-by-side plot panels for direct Circ vs CNN comparison.

The scenario sources remain in your Basilisk tree.

## How to run

```bash
cd /path/to/basilisk/examples/OpNavScenarios/scenariosOpNav
python3 scenario_OpNavAttOD.py

cd /path/to/basilisk/examples/OpNavScenarios
python3 scenario_CNNAttOD.py
```

## Results in this repo

`results/opnav_summary.json` and the two plot PNGs.
