# 08 — Basilisk–ROS 2 leader–follower control (PID vs MPC)

## What this is

Three spacecraft (`leaderSc`, `followerSc_1`, `followerSc_2`) in LEO, 12 thrusters each, Hill-frame wrench interface through `bsk-ros2-bridge` (ZMQ). Identical goal sequences recorded as `pid_run` and `mpc_run`. MPC lowers position RMSE (~0.13 m vs ~0.18 m) with higher force/torque effort; peak error is similar (same initial step).

## Official example this started from

**bsk-ros2-bridge** `examples/scenarioRosLeaderFollowerBasic_wrench.py` and `scenarioRosBasic_wrench.py`. Official examples show one spacecraft or a basic leader–follower wrench PID through the bridge. They do not run a three-satellite formation, identical PID vs Acados MPC goal sequences, or the bag-replay metrics table in the report.

## What changed

- `scenarioRosLeaderFollowerOrbit_wrench.py` — three-sc Hill-frame plant + bridge topics (`hill_trans_state`, `cmd_force` / `cmd_torque`).
- `pid_control_node.py` (Kp = [0.9, 0.9, 0.9], Kd = [9, 9, 9] translation) vs MPC (m = 17.8 kg, I = 0.315 I₃, Δt = 0.2 s, N = 30, soft keep-out).
- `compare_pid_mpc_bags.py` / `generate_pid_mpc_report_artifacts.py` and `summary_metrics.csv`.

ROS bags (`pid_run/`, `mpc_run/`) are **not** in this repo (~leave them in `ros2_ws`).

## How to run

Requires a sourced ROS 2 workspace with `bsk-ros2-bridge`, `bsk-ros2-mpc`, and Basilisk:

```bash
# terminal A: Basilisk plant
python3 scripts/scenarioRosLeaderFollowerOrbit_wrench.py
# terminal B: PID or MPC nodes + goal sequence (see formation_goal_sequence.py)
python3 scripts/compare_pid_mpc_bags.py
```

## Results in this repo

`results/summary_metrics.csv`, `step_metrics.csv`, `pid_mpc_*.png`.
