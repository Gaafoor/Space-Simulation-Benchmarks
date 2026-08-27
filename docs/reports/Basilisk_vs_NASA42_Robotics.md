# Basilisk vs. NASA 42 — GNC, Robotics & Autonomy Comparison

**Scope:** Capability comparison of the **Basilisk** astrodynamics framework ([AVSLab/basilisk](https://github.com/AVSLab/basilisk)) and NASA Goddard's **42** spacecraft simulator ([ericstoneking/42](https://github.com/ericstoneking/42)), extended from the GNC/ADCS comparison into **robotics & autonomy**: perception, localization, planning, contact/articulated dynamics, trajectory design/optimization, ROS 2, and HIL.

**Evidence basis:** Repo-grounded (GitHub source inspection, not marketing). Basilisk at commit `639efa6b` (2026-08-18), `src/simulation/` + `src/fswAlgorithms/`; 42 at commit `edc74576` (2026-08-13), `Source/` (23 C files, 26,441 lines). Basilisk FSW algorithms are C/C++ (`.c`/`.cpp` + `.h`); 42 is pure C. BSK-RL ([AVSLab/bsk_rl](https://github.com/AVSLab/bsk_rl)) and `bsk-ros2-bridge`/`bsk-ros2-mpc` ([DISCOWER](https://github.com/DISCOWER/bsk-ros2-bridge)) are third-party Basilisk ecosystems.

---

## Executive Verdict

| Question | Verdict |
|---|---|
| GNC algorithm breadth (estimation, guidance, control) | **Basilisk** — 104 FSW algorithm modules incl. EKF/UKF/SEKF/SuKF attitude determination, optical-nav relative-OD UKF, Lambert guidance, CNN image processing. 42 ships no onboard estimator. |
| Robotics / autonomy stack | **Basilisk** — perception (opnav, CNN image processing), RL planning (BSK-RL), MuJoCo contact, ROS 2 bridge, acados MPC. 42 has no perception, planning, or autonomy modules. |
| Contact / servicing dynamics & mission assets | **42** — native multi-body contact + chaser/target/arm/hand/rover models, PFFIAT heritage. Basilisk closes the gap via MuJoCo (newer). |
| Trajectory optimization (SNOPT/IPOPT) | **Neither** ships a native general-purpose NLP optimizer. Basilisk has Lambert (global) + third-party acados MPC (local); 42 has neither. |
| HIL / flight-software integration | **42** — first-class socket IPC, cFS, NOS3, GMSEC. Basilisk HIL is DIY or via the ROS 2 bridge. |
| Real-time vs fast runs | 42 standalone has `FAST_TIME` (CPU-speed) for accelerated runs; NOS3 is real-time (wall-clock-locked). Basilisk runs faster than real time by default. |
| Algorithm portability to cFS/NOS3 | **Tie** — both ship extractable C/C++ GNC source; porting effort is comparable (42's pure C is only a minor cFS convenience). |

**Bottom line:** **Basilisk is the robotics/autonomy + GNC algorithm platform** (modular Python/C++/C, RL, perception, MuJoCo, ROS 2). **42 is the turn-key dynamics + HIL/flight-software integration platform** (native contact/servicing, NOS3/cFS, built-in 3D viz). They are complementary: design GNC/autonomy algorithms in Basilisk, then validate the cFS flight implementation end-to-end in NOS3 (which uses 42 as its truth engine).

---

## Capability Matrix

| Capability | Basilisk | NASA 42 |
|---|---|---|
| **Language / architecture** | C++ core + SWIG Python + Rust; modular message-bus | Pure C, monolithic single executable, text I/O |
| **Onboard estimation (localization)** | EKF/UKF/SEKF/SuKF (`attDetermination/`: sunlineEKF, InertialUKF, headingSuKF, okeefeEKF, CSSEst); optical relative-OD UKF; small-body nav EKF/UKF | None — truth-based FSW; estimation left to external FSW over IPC |
| **Perception** | `imageProcessing/`: `centerRadiusCNN` (CNN), `houghCircles`, `limbFinding`; `opticalNavigation/relativeODuKF`, `pixelLineBiasUKF` | None |
| **Localization via CNN / sensor fusion** | CNN modules produce image-derived measurements (limb/center) consumed by opnav UKFs; full sensor-fusion EKF/UKF pipelines | None |
| **Planning — RL** | **BSK-RL** ([AVSLab/bsk_rl](https://github.com/AVSLab/bsk_rl)) — Basilisk Gymnasium environments for training autonomy policies | None |
| **Planning — classical (FSM, behavior trees)** | Not a packaged native framework; implementable as external Python/C++/ROS 2 nodes reading the message bus | None (42 FSW modes are scenario-fixed, not a planner) |
| **Control laws** | MRP PD/feedback/steering, full-nonlinear rate servo, MTB/thruster momentum mgmt | Scenario FSW "personalities" (Spinner, ThreeAxis, ISS, CMG, Thruster) |
| **Guidance / trajectory design** | Lambert solver + planner + validator (global targeting); `dvGuidance` | Quaternion/ΔV command interpreter; Hill/CW; no Lambert |
| **Trajectory optimization** | **No native SNOPT/IPOPT**; third-party `bsk-ros2-mpc` uses **acados** for local MPC | No native optimizer; no MPC |
| **Contact / articulated dynamics** | **MuJoCo** integration (`mujocoDynamics/`); hinged/spring-mass effectors | **Native, first-class** — Kane's-method multi-body, joints, inter-spacecraft & surface contact, ARC chaser/target/arm/hand, lander, rover |
| **Formation flying / RPO** | `formationFlying/` (barycenter, hill-frame control); rendezvous examples | Multi-spacecraft + contact; PFFIAT heritage; RACS2/Space-ROS bridges possible |
| **Environment models** | NRLMSISE-00, WMM, faceted SRP, albedo, Earth radiation, SPICE ephemeris | EGM96/GLGM2/GMM2B gravity; NRLMSIS atmosphere; own ephemeris |
| **3D visualization** | Vizard (standalone Unity app) | Built-in OpenGL/GPGPU in the same executable |
| **ROS 2 support** | Third-party **bsk-ros2-bridge** (ZeroMQ↔ROS 2, real-time `simSynch`); `bsk-ros2-mpc` proven on lab free-flyers | None |
| **Monte Carlo** | Native Python dispersion + 153 scenarios | Julia/MATLAB external scripts |
| **HIL / FSW integration** | Possible (message bus, external C++, bridge); no native cFS/NOS3 bridge | **First-class** — socket IPC, GMSEC, NOS3 (`42nos3.c`), cFS FSW (`CfsFSW`), `AcApp.c` |
| **Tests / CI / packaging** | 427 pytest + 9 CI workflows + PyPI/Docker | No formal tests; minimal CI |
| **Time modes** | Faster-than-real-time default (wall-clock lock optional via `simSynch`) | `FAST_TIME` (CPU-speed) / `REAL_TIME` (nanosleep) |

---

## Robotics & Autonomy — Notes

- **Perception → localization:** Basilisk's CNN image-processing modules (`centerRadiusCNN`, `houghCircles`, `limbFinding`) generate image-derived measurements (limb/centerline) that feed the optical-navigation UKFs (`relativeODuKF`, `pixelLineBiasUKF`) and the attitude-determination filters — a perception-to-estimation pipeline. 42 has no perception or estimator modules at all; its FSW is truth-based.
- **Planning:** Basilisk's packaged autonomy path is **reinforcement learning** via BSK-RL. **Classical** task-planning approaches (finite-state machines, behavior trees, state/behaviour-based logic) are **not** shipped natively by either tool — they must be implemented as external FSW or ROS 2 nodes that read/write the Basilisk message bus (or the cFS software bus in the 42 path).
- **Trajectory optimization:** Neither repo ships a native general-purpose nonlinear optimizer (SNOPT/IPOPT). Basilisk provides a Lambert solver for **global** targeting; **local** optimal control (MPC) is available only via the third-party `bsk-ros2-mpc`, which uses **acados** (not SNOPT/IPOPT). For production trajectory optimization you bring your own NLP solver (SNOPT/IPOPT/acados/Pyomo) and couple it to either simulator's dynamics.
- **Contact/servicing:** 42 is stronger natively (Kane's-method multi-body, joints, contact forces, and bundled mission assets). Basilisk's MuJoCo integration is the modern path for articulated/contact robotics but is newer and less turn-key than 42's bundled models.
- **Flight path:** 42's cFS/NOS3 stack is flight-proven; Basilisk algorithm code (C/C++) still requires extraction and re-wrapping as a cFS app before flight — but the extraction effort is comparable to 42's (see the GNC/ADCS comparison, §2.18).
- **Architecture for autonomy experiments:** Basilisk wins (modular Python + C/C++ modules + RL/ROS 2 ecosystem, unit-testable). 42 is scenario/physics/HWIL-oriented, not an autonomy-experimentation platform.

---

## Recommendations by Use Case

| Use case | Recommended |
|---|---|
| GNC algorithm R&D (estimation, control, guidance) | **Basilisk** |
| Perception / optical navigation / CNN image processing | **Basilisk** |
| Autonomy & RL policy training | **Basilisk + BSK-RL** |
| Classical autonomy (FSM / behavior trees) | External FSW/ROS 2 node + **Basilisk** bus (neither ships it) |
| Global trajectory targeting (Lambert / ΔV) | **Basilisk** |
| Local optimal control / MPC | **Basilisk + bsk-ros2-mpc (acados)** |
| General trajectory optimization (SNOPT/IPOPT) | Bring your own optimizer; couple to **either** |
| Contact / servicing / articulated dynamics | **42** (or Basilisk + MuJoCo) |
| Formation flying / RPO | **42** (scenarios) / **Basilisk** (algorithms) |
| ROS 2-based FSW + tooling (RViz, rosbag) | **Basilisk + bsk-ros2-bridge** |
| Hardware-in-the-loop (cFS, NOS3, GMSEC) | **42** |
| Fast / accelerated GNC validation runs (non-real-time) | **42 (`FAST_TIME`) or Basilisk** (NOS3 is real-time by design) |
| 3D visualization & concept demos | **42** (built-in) |
| Industrial SE quality (tests, CI, packaging) | **Basilisk** |

---

## Bottom Line

**Basilisk** is the stronger **robotics & autonomy + GNC algorithm** platform: real estimators, perception/CNN, RL planning (BSK-RL), MuJoCo contact, ROS 2 tooling, and modern software engineering. **42** is the stronger **turn-key dynamics + HIL/flight-software integration** platform: native multi-body/contact/servicing dynamics, built-in 3D visualization, and first-class cFS/NOS3/GMSEC integration used on STF-1, Dellingr, and Lunar IceCube. Neither ships a native SNOPT/IPOPT trajectory optimizer, and neither ships a native classical-planning framework (FSM/behavior trees) — both gaps must be filled externally. For a robotics/autonomy + GNC development workflow: design and train algorithms in Basilisk, validate the portable C/C++ core as a cFS app, then run real-time SIL/HIL/V&V in NOS3 (42 as the truth engine).
