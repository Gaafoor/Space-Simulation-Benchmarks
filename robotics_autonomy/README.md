# Robotics and autonomy benchmarks

Report §2. Contact, docking, manipulation, rendezvous, RL, OpNav, and ROS 2 formation control.

| # | Folder | Starts from (typical official sample) | What this repo adds |
|---|--------|----------------------------------------|---------------------|
| 01 | [MuJoCo hub rebound](01_mujoco_hub_rebound/) | `examples/mujoco/scenarioCubeSatContactRebound.py` | Realistic 1U buses, RW/tank bodies, logged wrenches |
| 02 | [MuJoCo arm docking](02_mujoco_arm_docking/) | `examples/mujoco/scenarioSimpleDocking.py` | Mesh probe/receptacle, LEO environment, capture weld |
| 03 | [MuJoCo thruster-arm](03_mujoco_thruster_arm/) | `examples/mujoco/scenarioThrArmControl.py` | Hover + 8 h inspection path (`ThrArmPaper.py`) |
| 04 | [NASA 42 docking](04_nasa42_docking/) | NASA 42 `Demo` docking | Tuned contact, CubeSat face-to-face, relative-motion plots |
| 05 | [Basilisk rendezvous](05_basilisk_rendezvous/) | `examples/scenarioRendezVous.py` | Run artifacts for the official servicer–debris case |
| 06 | [BSK-RL RSO inspection](06_bsk_rl_rso_inspection/) | `bsk_rl/benchmarks/rso_inspection.py` | Trained policy, multi-seed eval, seed-42 mission |
| 07 | [Basilisk OpNav](07_basilisk_opnav/) | `scenario_OpNavAttOD.py`, `scenario_CNNAttOD.py` | Circ vs CNN metrics on the same Mars orbit |
| 08 | [Basilisk–ROS 2 PID vs MPC](08_basilisk_ros2_pid_mpc/) | `scenarioRosLeaderFollowerBasic_wrench.py` | Three-sc Hill-frame wrench, identical goals, bag metrics |
| 09 | [Space ROS Canadarm2](09_space_ros_canadarm2/) | Space ROS Canadarm2 Gazebo demo | Gazebo + MoveIt run; arm assets imported in Isaac Sim |

Basilisk + MuJoCo is the contact-physics path. NASA 42 is the whole-vehicle proximity / ADCS path. See the internship report §2.5 for that comparison.
