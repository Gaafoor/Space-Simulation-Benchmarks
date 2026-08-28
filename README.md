# Space Simulation Benchmarks

Cross-tool benchmarks for **spacecraft astrodynamics** and **space robotics / autonomy**.

The cases started from official samples and demos (GMAT, Basilisk, NASA 42, Orekit, FreeFlyer, Space ROS, BSK-RL) and were extended into comparable mission-style applications: perturbation ladders, Madrid ground access, Earth-observation imaging, a ~53-day LEO-to-GEO electric transfer with design-vs-execution checks, contact/docking, rendezvous, RL inspection, optical navigation, and ROS 2 leader–follower control.

This folder is what you push to GitHub: scripts, input decks, slim results, plots, and READMEs. It does **not** contain the simulation tools or the multi-gigabyte raw archives.

## Repository name

**Space-Simulation-Benchmarks** is the folder name used here. It covers both families in one repo.

Other names that also work: `Spacecraft-Simulation-Benchmarks`, `Space-GNC-Robotics-Benchmarks`. `Astrodynamics-Simulation-Benchmarks` undersells the robotics half.

## Layout

```text
astrodynamics/          # report §1 — numbered 01–12
robotics_autonomy/      # report §2 — numbered 01–09
docs/reports/           # internship + technical PDFs
docs/figures/           # figures used in the reports
NOTICE.md               # third-party tools and what is not shipped
```

Each benchmark folder:

- `README.md` — summary, original files, changes, run commands
- `scripts/` — runners and input decks
- `results/` — JSON/CSV summaries and plots

The LEO-to-GEO electric-transfer campaign (report §1.5–1.11) shares one execution pipeline. Canonical scripts live in `astrodynamics/05_leo_geo_ep_design/` (GMAT design) and `astrodynamics/06_transfer_execution/` (Basilisk + NASA 42). Folders 07–11 keep their own README and results and point back to those scripts.

## Astrodynamics

| Folder | Concept | Tools |
|--------|---------|-------|
| [01_two_body_validation](astrodynamics/01_two_body_validation/) | 24 h Kepler check vs analytical reference | GMAT, Basilisk |
| [02_perturbation_ladder](astrodynamics/02_perturbation_ladder/) | Two-body → J2 → drag → SRP → third body | GMAT, Basilisk, Orekit, FreeFlyer |
| [03_madrid_contacts_eclipse](astrodynamics/03_madrid_contacts_eclipse/) | Madrid passes and eclipse windows | GMAT, Basilisk, FreeFlyer |
| [04_eo_imaging_raan](astrodynamics/04_eo_imaging_raan/) | RAAN trade, off-nadir, coverage | GMAT, Basilisk, Orekit |
| [05_leo_geo_ep_design](astrodynamics/05_leo_geo_ep_design/) | Low-thrust spiral design (SMA target) | GMAT |
| [06_transfer_execution](astrodynamics/06_transfer_execution/) | Design vs high-fidelity execution | Basilisk, NASA 42 |
| [07_power_aware_ep](astrodynamics/07_power_aware_ep/) | Dynamic solar power and eclipse gating | GMAT, Basilisk, NASA 42 |
| [08_attitude_thrust_pointing](astrodynamics/08_attitude_thrust_pointing/) | Thrust-axis pointing under ADCS | Basilisk, NASA 42 |
| [09_rw_momentum_management](astrodynamics/09_rw_momentum_management/) | Reaction wheels and magnetorquer unload | Basilisk, NASA 42 |
| [10_cross_tool_agreement](astrodynamics/10_cross_tool_agreement/) | States vs common GMAT reference | GMAT, Basilisk, NASA 42 |
| [11_end_to_end_53day](astrodynamics/11_end_to_end_53day/) | Full ~53-day campaign metrics | GMAT, Basilisk, NASA 42 |
| [12_constellation_replay](astrodynamics/12_constellation_replay/) | Multi-spacecraft divergence / replay | GMAT, Basilisk, FreeFlyer |

## Robotics and autonomy

| Folder | Concept | Tools |
|--------|---------|-------|
| [01_mujoco_hub_rebound](robotics_autonomy/01_mujoco_hub_rebound/) | 1U hub-to-hub contact rebound (custom scenario) | Basilisk + MuJoCo |
| [02_mujoco_arm_docking](robotics_autonomy/02_mujoco_arm_docking/) | Arm docking with capture weld (custom scenario) | Basilisk + MuJoCo |
| [03_mujoco_thruster_arm](robotics_autonomy/03_mujoco_thruster_arm/) | 6-DOF hub + thruster arms + inspection path | Basilisk + MuJoCo |
| [04_nasa42_docking](robotics_autonomy/04_nasa42_docking/) | Face-to-face CubeSat docking | NASA 42 |
| [05_basilisk_rendezvous](robotics_autonomy/05_basilisk_rendezvous/) | Servicer–debris proximity / ADCS modes | Basilisk |
| [06_bsk_rl_rso_inspection](robotics_autonomy/06_bsk_rl_rso_inspection/) | RL inspector vs non-cooperative RSO | BSK-RL |
| [07_basilisk_opnav](robotics_autonomy/07_basilisk_opnav/) | Camera limb OD (Hough vs CNN) | Basilisk OpNav |
| [08_basilisk_ros2_pid_mpc](robotics_autonomy/08_basilisk_ros2_pid_mpc/) | Official plant + custom PID/MPC comparison layer | Basilisk + ROS 2 |
| [09_space_ros_canadarm2](robotics_autonomy/09_space_ros_canadarm2/) | Canadarm2 in Gazebo / MoveIt / Isaac | Space ROS |

## What you need locally

Install the tools yourself. Point the runners at your installs, for example:

```text
GMAT          ~/Downloads/GMAT/R2026a
Basilisk      conda / pip install of basilisk-develop
NASA 42       ~/Downloads/42 20240919/42   (set NASA42_ROOT)
Orekit        Python bindings + orekit-data
FreeFlyer     commercial install (optional)
ROS 2         local ros2_ws with bsk-ros2-bridge (PID/MPC case)
Space ROS     local space_ros_ws (Canadarm2 case)
```

Typical pattern: `cd` into a benchmark `scripts/` folder and run the Python runner, or open the GMAT `.script` / NASA 42 deck in that tool. Exact commands are in each benchmark README.

## Results policy

Checked in: `summary.json`, comparison CSVs, parameter tables, and the plots used in the internship report.

Not checked in: Vizard recordings, full `state_history.csv` arcs, SPICE kernels, ROS bags. Those remain on the intern workstation.

## Reports

- [Internship experience report](docs/reports/Internship_Experience_Report.pdf)
- [Technical benchmarks report](docs/reports/Benchmarks_Technical_Report.pdf)

## License

Original material: [MIT](LICENSE). Upstream samples keep their own headers; see [NOTICE.md](NOTICE.md).
