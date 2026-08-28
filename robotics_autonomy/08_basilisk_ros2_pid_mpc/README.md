# 08 — Basilisk–ROS 2 leader–follower control (PID vs MPC)

## What this is

Three spacecraft (`leaderSc`, `followerSc_1`, `followerSc_2`) in LEO, 12 thrusters each, Hill-frame wrench interface through `bsk-ros2-bridge` (ZMQ). Identical 4-waypoint goal sequences recorded as `pid_run` and `mpc_run`. MPC lowers position RMSE (~0.13 m vs ~0.18 m) with higher force/torque effort; peak error is similar (same initial step).

This benchmark uses **two upstream GitHub repos** plus **custom GNC/metrics code** written for the internship comparison.

## Official upstream — repo 1: bsk-ros2-bridge (DISCOWER)

GitHub: [DISCOWER/bsk-ros2-bridge](https://github.com/DISCOWER/bsk-ros2-bridge)

| Official file | Role |
|---------------|------|
| `examples/scenarioRosLeaderFollowerOrbit_wrench.py` | Three spacecraft in LEO Hill frame; wrench commands via bridge |
| `examples/scenarioRosLeaderFollowerBasic_wrench.py` | Leader + 2 followers, no Earth orbit (reference only here) |
| `examples/scenarioRosBasic_wrench.py` | Single-spacecraft wrench demo |

The plant scenario in `scripts/scenarioRosLeaderFollowerOrbit_wrench.py` is **copied verbatim** from the bridge repo.

Upstream bridge does **not** ship a PID controller, unified goal sequencer, or bag-replay metrics.

## Official upstream — repo 2: bsk-ros2-mpc (DISCOWER)

GitHub: [DISCOWER/bsk-ros2-mpc](https://github.com/DISCOWER/bsk-ros2-mpc)

Reference copy: `~/Downloads/All examples/bsk-ros2-mpc-main/`

Installed in the ROS 2 workspace (`ros2_ws/src/bsk-ros2-mpc/`). **Not vendored** in this benchmarks repo.

| Official component | Role |
|--------------------|------|
| `bsk_ros2_mpc/nodes/bsk_mpc_node.py` | Acados nonlinear MPC node (`bsk-mpc` executable) |
| `controllers/mpc_wrench.py`, `mpc_follower_wrench.py` | Wrench-level and leader-relative follower MPC |
| `launch/mpc.launch.py` | Single agent + optional RViz / `SetPose` service |
| `launch/mpc_leader.launch.py` | Leader MPC + built-in `waypoint-publisher` (5 waypoints) |
| `launch/mpc_followers.launch.py` | Two `follower_wrench` MPCs + `follower-publisher` (relative offsets) |
| `planners/waypoint_publisher.py`, `follower_publisher.py` | Official setpoint generators for formation demo |

### Official MPC workflow (as shipped)

```bash
# after Basilisk plant + bridge are running:
ros2 launch bsk-ros2-mpc mpc_leader.launch.py use_sim_time:=True
ros2 launch bsk-ros2-mpc mpc_followers.launch.py use_sim_time:=True
```

- Leader: absolute Hill-frame waypoints from `waypoint-publisher`.
- Followers: `follower_wrench` MPC tracking **relative** offsets from the leader.
- RViz on by default (`use_rviz:=True`); interactive via `SetPose` service.
- **No PID baseline**, no paired bag metrics.

### Internship workflow (this benchmark — different from official MPC demo)

| Aspect | Official bsk-ros2-mpc | This benchmark |
|--------|----------------------|----------------|
| Launch | `mpc_leader` + `mpc_followers` | **3× `mpc.launch.py`** (`type:=wrench`, all symmetric) |
| Follower mode | `follower_wrench` (relative to leader) | **Independent absolute-position** wrench MPC per spacecraft |
| Setpoints | `waypoint-publisher` + `follower-publisher` | **Custom `formation_goal_sequence.py`** — same 4 waypoints to all 3 SC |
| RViz | Default on | **`use_rviz:=False`** for recorded runs |
| Comparison | MPC only | **PID vs MPC** on identical references |
| Metrics | None | ROS bags → `compare_pid_mpc_bags.py` → CSV/plots |

The MPC **solver code is official**; the launch pattern and setpoint source were changed so MPC and PID see the **same** absolute goal sequence.

## Files in this repo

| File | Source |
|------|--------|
| `scripts/scenarioRosLeaderFollowerOrbit_wrench.py` | **Official** — bsk-ros2-bridge |
| `scripts/scenarioRosLeaderFollowerBasic_wrench.py` | **Official** — bsk-ros2-bridge (unused in reported runs) |
| `scripts/pid_control_node.py` | **Custom** — Hill-frame wrench PID (local bridge fork; mirrored here) |
| `scripts/formation_goal_sequence.py` | **Custom** — 4-waypoint publisher for fair PID/MPC comparison |
| `scripts/compare_pid_mpc_bags.py` | **Custom** — bag replay metrics |
| `scripts/generate_pid_mpc_report_artifacts.py` | **Custom** — report plots/tables |

Also in the local bridge fork (not copied here): `bsk_ros2_bridge/launch/pid_leader_follower.launch.py`.

MPC package files (`bsk-ros2-mpc`) stay in `ros2_ws` — use the official repo unchanged.

## What changed vs upstream (summary)

**vs bsk-ros2-bridge:** added PID node, goal-sequence publisher, launch file, bag metrics.

**vs bsk-ros2-mpc:** kept official Acados MPC nodes; replaced official leader/follower launch + waypoint publishers with 3× symmetric `mpc.launch.py` + `formation_goal_sequence.py` so MPC matches the PID reference trajectory.

ROS bags (`pid_run/`, `mpc_run/`) are **not** in this repo — leave them in `ros2_ws`.

## How to run

Requires Basilisk, sourced `ros2_ws` with `bsk-ros2-bridge`, `bsk-ros2-mpc`, and acados.

```bash
# terminal A — Basilisk plant
python3 scripts/scenarioRosLeaderFollowerOrbit_wrench.py

# terminal B — bridge
ros2 launch bsk-ros2-bridge bridge.launch.py

# terminal C — PID run
ros2 launch bsk-ros2-bridge pid_leader_follower.launch.py use_sim_time:=true

# terminal C — MPC run (three terminals, or sequential)
ros2 launch bsk-ros2-mpc mpc.launch.py namespace:=leaderSc type:=wrench \
  use_hill:=True use_sim_time:=True use_rviz:=False \
  name_others:="followerSc_1 followerSc_2" skip_build:=False
# ... same for followerSc_1 and followerSc_2 with skip_build:=True

# terminal D — same for both runs
ros2 run bsk-ros2-bridge formation-goal-sequence --ros-args \
  -p use_sim_time:=true -p call_set_pose_service:=false

# post-run
python3 scripts/compare_pid_mpc_bags.py
```

To run the **official** MPC formation demo instead, use `mpc_leader.launch.py` + `mpc_followers.launch.py` (no PID, no `formation-goal-sequence`).

## Results in this repo

`results/summary_metrics.csv`, `step_metrics.csv`, `pid_mpc_*.png`, demo MP4s (`pid_benchmark.mp4`, `mpc_benchmark.mp4`, `mpc_rviz_demo.mp4`).
