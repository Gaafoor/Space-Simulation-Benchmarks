# 06 — BSK-RL RSO inspection

## What this is

An inspector satellite trained with BSK-RL against a non-cooperative RSO on Basilisk dynamics. Hill-frame impulsive Δv, eclipse and relative-state observations, 100 surface inspection points, 10 m/s Δv budget, 5700×10 s episode limit. Training: 147 iterations, best mean return −0.446 (iter 136). Multi-seed eval (30 episodes, seeds 42–71): mean inspected fraction 0.264 ± 0.116.

## Official example this started from

**BSK-RL** `bsk_rl/benchmarks/rso_inspection.py` (AVS Lab). The official benchmark defines the inspection MDP. This repo keeps the entry-point script used for the internship run plus **trained-policy evaluation artifacts** (eval JSON, seed-42 mission, plots).

## What changed

- Training run to 147 iterations and checkpoint selection.
- Multi-seed evaluation table (`eval_summary.json`).
- Seed 42 mission replay: 33/100 points, 0.636 m/s Δv, 184.8 m final range (`mission_summary.json`).
- Report figures: training curves, Hill-frame trajectory, Vizard still.

## How to run

Requires `bsk_rl` installed against Basilisk:

```bash
cd scripts
python3 rso_inspection.py          # train / as configured in the script
# evaluation was run from the BSK-RL eval helpers; see eval_summary.json
```

## Results in this repo

`results/eval_summary.json`, `mission_summary.json`, `bskrl_rso_*.png`.
