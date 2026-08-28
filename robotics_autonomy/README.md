# Robotics and autonomy benchmarks

Report §2. Contact, docking, manipulation, rendezvous, RL, OpNav, ROS 2 formation control.

| # | Folder | Original files | Changed |
|---|--------|----------------|---------|
| 01 | [MuJoCo hub rebound](01_mujoco_hub_rebound/) | `scenarioUnbalancedThrusters`, `scenarioReactionWheel`, `scenarioSimpleDocking` | Custom rebound scenario + XML, contact metrics |
| 02 | [MuJoCo arm docking](02_mujoco_arm_docking/) | `scenarioSimpleDocking`, `scenarioArmWithThrusters` | Custom mesh docking, LEO env, capture weld |
| 03 | [MuJoCo thruster-arm](03_mujoco_thruster_arm/) | `scenarioThrArmControl`, `sat_w_thruster_arms.xml` | + `ThrArmPaper.py` inspection path |
| 04 | [NASA 42 docking](04_nasa42_docking/) | NASA 42 `Demo` decks | Tuned contact, pulsed thrusters, plots |
| 05 | [Basilisk rendezvous](05_basilisk_rendezvous/) | `scenarioRendezVous.py` | Evaluation plots only |
| 06 | [BSK-RL RSO inspection](06_bsk_rl_rso_inspection/) | `bsk_rl/benchmarks/rso_inspection.py` | Training, multi-seed eval, Vizard video |
| 07 | [Basilisk OpNav](07_basilisk_opnav/) | `scenario_OpNavAttOD.py`, `scenario_CNNAttOD.py` | Paired metrics, Circ vs CNN |
| 08 | [PID vs MPC](08_basilisk_ros2_pid_mpc/) | [bsk-ros2-bridge](https://github.com/DISCOWER/bsk-ros2-bridge) plant + [bsk-ros2-mpc](https://github.com/DISCOWER/bsk-ros2-mpc) | PID node, goal sequencer, bag metrics |
| 09 | [Space ROS Canadarm2](09_space_ros_canadarm2/) | Space ROS `demos/canadarm2` | Gazebo/MoveIt run, Isaac Sim import |

Basilisk + MuJoCo: contact physics. NASA 42: whole-vehicle proximity / ADCS. Report §2.5 compares both.
