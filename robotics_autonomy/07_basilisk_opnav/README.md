# 07 — Basilisk OpNav (Hough vs CNN)

## What this is

Two Basilisk OpNav cases on a Mars elliptical orbit (a = 18000 km, e = 0.6): limb Hough circles (`scenario_OpNavAttOD.py`) and CNN limb/measurement (`scenario_CNNAttOD.py`). Both close the loop from camera imagery to attitude/orbit estimates.

Circ position RMS 0.55 vs CNN 5.24; Circ velocity RMS 1.41 vs CNN 13.18; CNN runtime ~2.3× Circ.

## Official example this started from

**Basilisk** `examples/OpNavScenarios/scenariosOpNav/scenario_OpNavAttOD.py` and `scenario_CNNAttOD.py`. These **are** the official demos. This benchmark ran both on the same orbit and compared RMS and runtime (`opnav_summary.json`). The scenario sources stay in your Basilisk tree.

## What changed

- Paired execution and a single metrics table (runtime, pos/vel RMS, image method).
- Side-by-side plot panels `bsk_opnav_circ_plots.png` / `bsk_opnav_cnn_plots.png`.

## How to run

```bash
cd /path/to/basilisk-develop/examples/OpNavScenarios/scenariosOpNav
python3 scenario_OpNavAttOD.py
python3 scenario_CNNAttOD.py
```

## Results in this repo

`results/opnav_summary.json` and the two plot PNGs.
