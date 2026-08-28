# 06 — BSK-RL RSO inspection

## What this is

An inspector satellite trained with BSK-RL against a non-cooperative RSO on Basilisk dynamics. Hill-frame impulsive Δv, eclipse and relative-state observations, 100 surface inspection points, 10 m/s Δv budget, 5700×10 s episode limit. Training: 147 iterations, best mean return −0.446 (iter 136). Multi-seed eval (30 episodes, seeds 42–71): mean inspected fraction 0.264 ± 0.116.

## Official upstream (BSK-RL)

| Official location | Role |
|-------------------|------|
| `bsk_rl/benchmarks/rso_inspection.py` | Benchmark MDP definition and training entry point (AVSLab BSK-RL repo) |
| `examples/rso_inspection.ipynb` | Tutorial notebook (not the benchmark script itself) |

The script in `scripts/rso_inspection.py` is the **official benchmark file** copied from the BSK-RL repo. This repo does not redefine the MDP; it archives a trained run.

## Files in this repo

| File | Status |
|------|--------|
| `scripts/rso_inspection.py` | **Official** (from `bsk_rl/benchmarks/`) |
| `results/eval_summary.json`, `mission_summary.json` | **Custom** — internship training/eval outputs |
| `results/RSOVizard.mp4` | **Custom** — Vizard replay recording |

## What changed vs official BSK-RL

- Training run to 147 iterations and checkpoint selection (official repo ships the environment; not a pre-trained policy).
- Multi-seed evaluation table (`eval_summary.json`).
- Seed 42 mission replay: 33/100 points, 0.636 m/s Δv, 184.8 m final range (`mission_summary.json`).
- Report figures and Vizard demo video.

## How to run

Requires `bsk_rl` installed against Basilisk:

```bash
cd scripts
python3 rso_inspection.py          # train / as configured in the script
# evaluation was run from the BSK-RL eval helpers; see eval_summary.json
```

## Results in this repo

`results/eval_summary.json`, `mission_summary.json`, `bskrl_rso_*.png`, `RSOVizard.mp4`.
