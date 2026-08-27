# Basilisk vs. NASA 42 — GNC / ADCS Capability Comparison

**Scope:** Direct, capability-for-capability comparison of the **Basilisk** astrodynamics simulation framework ([AVSLab/basilisk](https://github.com/AVSLab/basilisk)) and NASA Goddard's **42** spacecraft simulation ([ericstoneking/42](https://github.com/ericstoneking/42)), restricted to **Guidance, Navigation & Control (GNC) / Attitude Determination & Control System (ADCS)** scope.

**Evidence basis (repo-grounded, not marketing):** Both repositories were cloned and inspected in detail. Findings are based on actual source trees, module inventories, header files, example/demo sets, documentation, and test suites — not on README claims alone.

- **Basilisk** inspected at commit `639efa6b` (2026-08-18), `src/simulation/` + `src/fswAlgorithms/` trees, `examples/`, `docs/`, `supportData/`, `.github/workflows/`.
- **42** inspected at commit `edc74576` (2026-08-13), `Source/` (23 C files, ~26.4 k lines), `Include/`, `Model/`, `World/`, `Demo/`, `Docs/`, `MonteCarlo/`.

The methodology: (1) compare the same capability side by side with an efficiency/quality assessment, then (2) list capabilities that exist in only one of the two tools.

**Note on "efficiency":** No quantitative runtime benchmark was executed for this report. Efficiency judgments below are **architectural/workflow-based** (e.g., pure-C single-executable vs. Python-configured C++ with SWIG overhead, native vs. external-tool visualization, native vs. scripted Monte Carlo), not measured wall-clock performance.

---

## Executive Verdict

| Question | Verdict |
|---|---|
| Which has more capabilities? | **Basilisk**, by a wide margin — 108 simulation modules + 104 FSW algorithm modules (module directories, excluding tests/scaffolding) spanning SPICE ephemeris, MuJoCo contact dynamics, ML-based optical navigation, and Lambert orbit guidance, versus 42's monolithic 23-file / ~26.4 k-line C codebase. |
| Which is higher industrial / SOTA quality? | **Basilisk** — modern modular C++/Python/Rust architecture, SWIG Python API, PyPI distribution, 427 Python unit tests, 9 CI workflows, Sphinx docs, active multi-contributor development. 42 is a mature, well-engineered but single-author, monolithic C program with text-file I/O and a built-in OpenGL GUI. |
| More used for servicing satellites? | **Public evidence favors 42 as the simulator infrastructure**: 42's multi-spacecraft + surface-contact dynamics underlie NASA GSFC's **PFFIAT Formation Flying Test Bed** ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20040171251/downloads/20040171251.pdf)) and a 2024 *J. Guidance, Control, and Dynamics* formation-flying fault-tolerance study ([AIAA ARC](https://arc.aiaa.org/doi/10.2514/1.G008087)); Basilisk has a newer servicing story (MuJoCo contact, IAC-2025 servicing testbed ([IAF](https://iafastro.directory/iac/paper/id/99032/abstract-pdf/IAC-25,D1,4,5,x99032.brief.pdf?2025-03-27.11:05:09))) and stronger RPO *algorithms* (relative-OD UKF, Lambert guidance). |
| More used for CubeSats / small sats? | **Both have strong, documented CubeSat use.** **42** is the dynamics engine of NASA's **NOS3** simulator, built for the **STF-1** CubeSat and used on **Dellingr** (6U) and **Lunar IceCube** ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20170011566/downloads/20170011566.pdf), [NASA NOS3](https://www.nasa.gov/nasa-operational-simulation-for-small-satellites/)). **Basilisk** is used at **HuskySat-2** (UW) and **OreSat** (PSAS) for GNC/HIL ([huskysat.org](https://huskysat.org/projects), [PSAS poster](https://spacegrant.oregonstate.edu/posters/hardware-loop-testing-infrastructure-small-satellite-labs)) and was designed for CubeSat-to-interplanetary missions ([ESA ICATT paper](https://indico.esa.int/event/224/contributions/3869/)). |

Bottom line: **Basilisk is the more capable and more modern/SOTA tool for GNC/ADCS algorithm development** — more modules, real onboard estimators, Lambert guidance, ML-based optical navigation, 427 tests, and a flown mission connection (LASP's Emirates Mars Mission "Hope" probe, for which Basilisk was developed as the ADCS simulation tool ([LASP](https://lasp.colorado.edu/missions/emm-hope/), [Schaub lab](https://hanspeterschaub.info/research-mission.html))). **42 is a more self-contained, turn-key simulator** with superior built-in 3D visualization, multi-body contact/servicing dynamics assets, and deep NASA HWIL/small-sat integration (NOS3, cFS, OpenSatKit) used on CubeSats STF-1, Dellingr, and Lunar IceCube ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20170011566/downloads/20170011566.pdf)). They are complementary: Basilisk for algorithm R&D and Monte-Carlo campaigns; 42 for rapid concept visualization, contact/servicing scenarios, and HWIL integration via sockets/NOS3/cFS.

---

## 1. Architectural Snapshot

| Dimension | Basilisk | NASA 42 |
|---|---|---|
| Origin | AVSLab, University of Colorado Boulder / LASP | Eric Stoneking, NASA Goddard Space Flight Center |
| License | **ISC** (permissive) ([GitHub](https://github.com/AVSLab/basilisk)) | U.S. Government work, no copyright under Title 17 (effectively public domain); SourceForge lists NASA Open Source Agreement (NOSA) ([SourceForge](https://sourceforge.net/projects/fortytwospacecraftsimulation/)) |
| Language | C++ core + SWIG Python bindings + Rust core components | C (monolithic) + optional Julia/MATLAB Monte-Carlo scripts |
| Architecture | Modular: each module is a self-contained C++ class instantiated in a Python `SimulationBaseClass`, communicating via a typed message-passing bus | Monolithic: a single executable reading text input files (`Inp_Sim.txt`, etc.); FSW runs in-process or as a separate "Ac" app over sockets |
| Distribution | `pip install bsk[all]` on PyPI, Docker on GHCR, multi-arch (amd64/arm64) | Build from source via `Makefile`; platform auto-detect (Linux/macOS/msys) |
| Tests | 427 Python test scripts under `tests/` + per-module C++ unit tests + pytest suite | No formal unit-test framework; validation by example scenarios + `Demo/` orbits |
| CI | 9 GitHub Actions workflows (pull-request, nightly-wheels, publish-container, canary, pre-commit) | Minimal `.github/` presence |
| Docs | Sphinx site (`docs/source/`), 153 example scenarios, Vizard viz docs | PDF/ODP docs in `Docs/` (Overview, FSW Models, Flight Regimes, Kane_NBody, Joints, HWIL IPC) |
| Ephemeris | NASA SPICE via `pyswice` wrapper (real SPICE kernels) | Own ephemeris model (`42ephem.c`); no SPICE dependency |
| 3D Visualization | Standalone **Vizard** app (separate download), Unity-based, proto-buffer interface | Built-in OpenGL (`42gl.c` 5,618 lines + `42glfw`/`42glut`), GPGPU shaders (`42gpgpu.c`), live spacecraft viewing |

---

## 2. Same-Capability Comparison (Direct, Capability-for-Capability)

### 2.1 Spacecraft Attitude Dynamics (rigid body)

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | `simulation/dynamics/spacecraft/`, `spacecraftSystem/` — Hub-effector / modular-body model with message-bus coupling | `42dynamics.c` (4,297 lines) — multi-body tree-topology model |
| Core method | Generalized hub + effector framework; eigenvalue-verified rigid-body integration | Kane's method for N-body systems (see `Docs/Kane_NBody.pdf`) |
| Efficiency | Python-configured, C++-executed; vectorized; numba acceleration available | Pure C, single executable; very fast, minimal overhead |
| Quality | Tested against analytic solutions in 427 pytest cases | Validated against analytic results; used in NASA studies for years |
| Verdict | **Rough parity in physics correctness.** 42 is leaner/faster to run; Basilisk is more modular and individually unit-testable. |

### 2.2 Multi-body / flexible-body dynamics

| Aspect | Basilisk | 42 |
|---|---|---|
| Flexible bodies | `HingedRigidBodies`, `NHingedRigidBodies`, `dualHingedRigidBodies`, `LinearSpringMassDamper`, `sphericalPendulum`, `spinningBodies`, `linearTranslationalBodies`, `constraintEffector` | Rigid **and** flexible bodies, tree topology, rotational + translational joints (`Docs/Kane_NBody_Flex.pdf`, `Translational Joints.pdf`) |
| Joints | `deviceInterface/`: `jointArrayRefProfiler`, `prescribedRotation1DOF`, `prescribedLinearTranslation`, `stepperMotor`, `encoder`, `motorVoltageInterface`, `hingedBodyLinearProfiler` | `42joints.c` + `Docs/Joints.pdf`, `Translational Joints.pdf` — rotational/translational joints with constraints |
| Verdict | **42 is stronger here.** Its multi-body flexible dynamics (Kane's method, tree topology, joints) is a first-class, long-standing feature and the core of the codebase. Basilisk supports hinged/spring-mass appendages and joint effectors, but its flexible-body story is a set of effector modules rather than a unified multi-body solver — MuJoCo integration (see §3) is being added to close this gap. |

### 2.3 Orbit dynamics (two-body & three-body)

| Aspect | Basilisk | 42 |
|---|---|---|
| Two-body | `gravityEffector`, point-mass + spherical-harmonic gravity; SPICE planet states | `42perturb.c` (1,117 lines) — two-body + perturbations |
| Three-body | Supported via SPICE ephemerides + multi-body gravity; Encke-style propagation not a first-class named module | First-class: `Docs/Encke ThreeBody.pdf`, `Flight Regimes.pdf` — two-body or three-body, anywhere in the solar system (per README) |
| Verdict | **42 is stronger for three-body / solar-system-wide regimes** (built-in, documented, mission orbits like `Orb_67P`, `Orb_Eros`, `Orb_EarthEscape`). Basilisk is fully capable via SPICE but the three-body capability is assembled from modules rather than being a documented first-class mode. |

### 2.4 Reaction-wheel / CMG actuation

| Aspect | Basilisk | 42 |
|---|---|---|
| RW | `dynamics/reactionWheels`, `fswSetupRW.py`, jitter via `dynamics/igbmNoiseStateEffector` | `42actuators.c` — RW with **LuGre friction model**, viscous + Stribeck friction, wheel drag |
| CMG/VSCMG | `dynamics/VSCMGs`, `fswSetupVSCMGs.py`; FSW steering `vscmgGimbalRateServo`, `vscmgVelocitySteering` | `CmgFSW`, `struct AcCmgCtrlType`, `CMG.obj`/`CMGSat` models |
| Verdict | **Rough parity**, different emphasis. 42 models friction physics (LuGre) in more detail; Basilisk offers more steering-law variety and Python RW/CMG configuration helpers, plus jitter noise models. |

### 2.5 Thrusters

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | `dynamics/Thrusters`, `fswSetupThrusters.py`, `thrMomentumManagement` | `42actuators.c` thruster modes (`Docs/Thruster Modes.pdf`), `ThrFSW` |
| Verdict | **Parity.** Both model pulsewidth/impulse-bit thrusters; 42 has documented thruster modes, Basilisk has a Python thruster-config pipeline and momentum-management control. |

### 2.6 Magnetic torquers (MTB)

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | `dynamics/MtbEffector`; FSW: `mtbFeedforward`, `mtbMomentumManagement`, `mtbMomentumManagementSimple` | Magnetometer + magnetic torquer support in FSW modes and `AcApp` |
| Verdict | **Basilisk stronger** — dedicated MTB feedforward + momentum-management algorithms with dedicated test coverage. 42 supports MTB within scenario FSW modes. |

### 2.7 Environment models (atmosphere, gravity, magnetic field, SRP)

| Aspect | Basilisk | 42 |
|---|---|---|
| Atmosphere | `ExponentialAtmosphere`, **`MsisAtmosphere` (NRLMSISE-00)**, `TabularAtmosphere`, stochastic density (`stochasticAtmDensity`, `stochasticDragCoeff`) | NRLMSIS-based atmosphere referenced in `42environs.c` + drag in `42perturb.c` |
| Gravity | Point-mass + spherical harmonics; SPICE planet states; `supportData/LocalGravData` | `Model/EGM96.txt`, `GLGM2.txt` (Moon), `GMM2B.txt` (Mars) — full gravity models shipped |
| Magnetic field | `magneticFieldCenteredDipole`, **`magneticFieldWMM` (World Magnetic Model)** | Magnetometer model uses field |
| SRP / eclipse | `dynamics/RadiationPressure`, `facetSRPDynamicEffector`, `facetedSRPEffector`, `environment/eclipse`, `albedo`, `earthRadiationModel`, `solarFlux` | SRP + eclipse in perturbations |
| Verdict | **Rough parity on atmosphere model class** (both reference NRLMSIS); **Basilisk stronger on breadth** — multiple atmosphere variants, WMM magnetic field, faceted SRP, albedo, Earth radiation, spacecraft charging, and stochastic density. 42 ships real gravity models (EGM96/GLGM2/GMM2B). |

### 2.8 Sensor models (ADCS-relevant)

| Sensor | Basilisk | 42 |
|---|---|---|
| Star tracker | `sensors/starTracker` | `AcStarTrackerType`, `StarTracker` in `42sensors.c`/`42gl.c` |
| Sun sensor | `sensors/coarseSunSensor` | `Sun Sensor` in `42sensors.c`/`42init.c` |
| Magnetometer | `sensors/magnetometer` | `AcMagnetometerType`, `Magnetometer` in `42sensors.c` |
| IMU | `sensors/imuSensor` | `imu` in `42sensors.c`/`42main.c` |
| GPS/GNSS | No dedicated GPS receiver sensor module confirmed in `sensors/` (truth/nav via `navigation/simpleNav` + `transDetermination` modules) | **Explicit GPS model** — `AcGpsType`, `GPS` heavily used (63 refs in `42sensors.c`) |
| Camera | `sensors/camera`, `navigation/pinholeCamera` | `42optics.c` (SatCam), `Docs/SatCam Description.txt` |
| Verdict | **42 stronger on explicit GPS sensor modeling.** Basilisk can provide truth/nav messages but does not ship a dedicated GPS receiver sensor module equivalent to 42's. Coverage is otherwise at rough parity. |

### 2.9 Attitude Determination / Onboard Estimation (the key differentiator)

| Aspect | Basilisk | 42 |
|---|---|---|
| Filters shipped | **`attDetermination/`: `CSSEst`, `InertialUKF`, `headingSuKF`, `okeefeEKF`, `sunlineEKF`, `sunlineSEKF`, `sunlineSuKF`, `sunlineUKF`** — real EKF/UKF/SEKF/SuKF attitude-determination filters; plus `opticalNavigation/relativeODuKF`, `pixelLineBiasUKF`, `faultDetection`; `smallBodyNavigation/smallBodyNavEKF` & `smallBodyNavUKF`; `stateEstimation/thrustCMEstimation` | **None as named modules.** 42's FSW is `InstantFSW` (truth-based) plus scenario control modes (`SpinnerFSW`, `ThreeAxisFSW`, `IssFSW`, `CmgFSW`, `ThrFSW`, `HexapodFSW`, `LunarCommFSW`). Grep for `Kalman|EKF|UKF|Filter|covariance` in 42 source returns only OpenGL texture filters (`42gl.c`, `42gpgpu.c`) and graphics — no onboard estimator. |
| Verdict | **Basilisk is decisively stronger.** It ships production-grade, individually unit-tested attitude-determination filters (EKF/UKF/SEKF/SuKF) and optical/relative/small-body navigation estimators. 42 does not expose Basilisk-like standalone estimator modules — estimation appears not to be a packaged first-class capability in the inspected source; 42's FSW modes use truth/ideal states, with estimation left to the user or to an external FSW app over the socket/IPC interface. This is the single biggest capability gap in the GNC scope. |

### 2.10 Attitude Control laws

| Aspect | Basilisk | 42 |
|---|---|---|
| Laws shipped | `attControl/`: `mrpFeedback`, `mrpPD`, `mrpPDRust`, `mrpSteering`, `prvSteering`, `rateServoFullNonlinear`, `lowPassFilterTorqueCommand`, MTB momentum management, thruster momentum management | Scenario FSW modes: `SpinnerFSW`, `MomBiasFSW`, `ThreeAxisFSW`, `IssFSW`, `CmgFSW`, `ThrFSW`, `HexapodFSW`, `SandboxFSW` |
| Verdict | **Different design philosophies, roughly comparable control authority.** Basilisk ships composable, individually-testable control laws (MRP feedback/PD/steering, full-nonlinear rate servo); 42 ships whole FSW "personalities" tuned per mission class. Basilisk is better for algorithm R&D; 42 is faster for a working end-to-end controller. |

### 2.11 Attitude Guidance / pointing modes

| Aspect | Basilisk | 42 |
|---|---|---|
| Modes | `attGuidance/`: `inertial3D`, `inertial3DSpin`, `sunSafePoint`, `velocityPoint`, `hillPoint`, `celestialTwoBodyPoint`, `locationPointing`, `opNavPoint`, `oneAxisSolarArrayPoint`, `eulerRotation`, `mrpRotation`, `rasterManager`, `waypointReference`, `constrainedAttitudeManeuver`, `simpleDeadband`, `attRefCorrection`, `attTrackingError` | Pointing handled inside FSW modes (e.g., ISS sun-pointing, lunar-comm pointing) |
| Verdict | **Basilisk stronger** — explicit, composable, documented pointing/reference modes including constrained attitude maneuvers and waypoint tracking. 42 bakes pointing into scenario FSW modes. |

### 2.12 Orbit guidance / maneuvering

| Aspect | Basilisk | 42 |
|---|---|---|
| Lambert / impulsive ΔV | `orbitControl/`: **`lambertSolver`, `lambertPlanner`, `lambertSecondDV`, `lambertSurfaceRelativeVelocity`, `lambertValidator`**, `orbElemOffset`, `smallBodyWaypointFeedback`; `dvGuidance`; `transDetermination/dvAccumulation`, `chebyPosEphem`, `oeStateEphem` | `FswCmdInterpreter` accepts quaternion & ΔV commands; Hill/CW references in `42dynamics.c`/`42report.c`; no named Lambert solver |
| Verdict | **Basilisk stronger** — ships an actual Lambert solver and ΔV planning/validation pipeline. 42 does orbit commands and Hill-frame relative motion but has no onboard Lambert/targeting module. |

### 2.13 Formation flying / relative motion

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | `formationFlying/`: `formationBarycenter`, `hillFrameRelativeControl`, `inertialCartFeedback`; Hill-frame guidance (`hillPoint`) | Multiple spacecraft with inter-spacecraft contact; Hill/CW references; rendezvous & precision formation flying per README |
| Verdict | **Parity in scope, different maturity.** 42's multi-spacecraft + contact dynamics is a first-class, long-standing feature (servicing/formation); Basilisk has named formation-flying control algorithms but a newer multi-spacecraft story. |

### 2.14 Monte Carlo

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | `utilities/MonteCarlo` + `run_all_test.py`; Python-driven dispersion of any parameter | `MonteCarlo/RunMC.jl` (Julia) + `RunMC.m` (MATLAB) + `How To Monte Carlo.txt` |
| Verdict | **Basilisk stronger** — native Python Monte-Carlo with 153 example scenarios to disperse; 42 relies on external Julia/MATLAB scripts. |

### 2.15 Hardware-in-the-loop (HWIL) / external FSW

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | Message bus + Python; custom C++ modules can be external; no native cFS/NOS3 socket bridge as a named module | First-class: socket IPC (`42commlink.c`, `42ipc.c`, `Docs/HWIL IPC.pdf`), GMSEC, **NOS3 integration** (`42nos3.c`), **cFS FSW mode** (`CfsFSW`), standalone `AcApp.c` FSW process |
| Verdict | **42 is decisively stronger** for HWIL and integration with NASA flight-software ecosystems (cFS, NOS3, GMSEC). This is 42's core design — the FSW runs as a separate process over sockets. Basilisk is simulator-first; HWIL integration is possible but not its native pattern. |

### 2.16 3D Visualization

| Aspect | Basilisk | 42 |
|---|---|---|
| Implementation | **Vizard** — standalone Unity-based app, separate download, proto-buffer interface (`vizInterface`, `dataFileToViz`), asteroid/IceSat2/Martian-moons assets | **Built-in OpenGL** in the same executable: `42gl.c` (5,618 lines), `42glfw.c`, `42glut.c`, GPGPU shaders (`42gpgpu.c`), live 3D spacecraft viewing, POV menu |
| Verdict | **42 is more convenient and integrated** (in-app, no extra download, GPGPU). Vizard is more modern/featured (Unity) but requires a separate tool and setup. |

### 2.17 Test coverage & software maturity

| Aspect | Basilisk | 42 |
|---|---|---|
| Tests | 427 Python pytest files + per-module C++ tests; `pytest.ini`, `conftest.py` | None formal; validation by examples |
| CI | 9 GitHub workflows incl. nightly wheels, PR gates, container publish | Minimal |
| Modern tooling | SWIG Python, Rust core, pre-commit, clang-format, Conan, Docker | Makefile + text I/O |
| Verdict | **Basilisk is decisively more industrial-grade** in software-engineering terms (tests, CI, packaging, multi-language, docs). 42 is mature physics but dated software practice. |

---

## 3. Capabilities Unique to Basilisk (not in 42)

- **Onboard attitude-determination filters** — EKF/UKF/SEKF/SuKF for sunline, CSS, heading, inertial, plus `okeefeEKF` ([repo](https://github.com/AVSLab/basilisk/tree/master/src/fswAlgorithms/attDetermination)).
- **Optical navigation** — `opticalNavigation/relativeODuKF` (relative orbit determination UKF), `pixelLineBiasUKF`, `faultDetection`.
- **Image processing** — `imageProcessing/`: `centerRadiusCNN` (convolutional neural network), `houghCircles`, `limbFinding`.
- **Small-body navigation** — `smallBodyNavEKF`, `smallBodyNavUKF` for asteroid/comet proximity navigation.
- **Lambert orbit guidance** — `lambertSolver`, `lambertPlanner`, `lambertSecondDV`, `lambertSurfaceRelativeVelocity`, `lambertValidator`.
- **Constrained attitude maneuver** & **waypoint reference** guidance (`attGuidance/constrainedAttitudeManeuver`, `waypointReference`).
- **MuJoCo integration for articulated/contact dynamics** — `mujocoDynamics/` modules (`MJSystemMassMatrix`, `JointPIDController`, `NBodyGravity`, stochastic atmosphere) add a high-fidelity contact/robotic solver. Note: 42 also has native contact/surface forces, so this is unique as MuJoCo integration, not as contact dynamics per se.
- **Rust core components** — `architecture/rust/` (`bsk_messages`, `bsk_utilities`) for modern messaging.
- **NASA SPICE integration** via `pyswice` (real SPICE kernels for ephemerides/time).
- **Modern environment models** — NRLMSISE-00 atmosphere (`MsisAtmosphere`), World Magnetic Model (`magneticFieldWMM`), faceted SRP, albedo, Earth radiation, spacecraft charging.
- **Power & thermal subsystems** — `power/` (solar panel, battery, power sink/monitor, RW power) and `thermal/` (motor/sensor thermal).
- **Onboard data handling** — `storageUnit`, `transmitter`, `spaceToGroundTransmitter`, `instrument`; `communication/` link budget.
- **427-test Python suite + 9 CI workflows + PyPI/Docker distribution.**

## 4. Capabilities Unique to 42 (not in Basilisk)

- **Built-in OpenGL 3D visualization in the same executable** — `42gl.c` (5,618 lines) + GLFW/GLUT + GPGPU shaders; no separate tool needed.
- **First-class multi-body flexible dynamics via Kane's method** — tree topology, rotational + translational joints, rigid + flexible bodies (`Docs/Kane_NBody.pdf`, `Kane_NBody_Flex.pdf`); more unified than Basilisk's effector-module approach.
- **First-class three-body / solar-system-wide orbit regimes** — documented Encke + three-body flight regimes, mission orbits for 67P, Eros, Earth-escape, etc.
- **Native contact-force + packaged servicing/ARC scenario assets** — inter-spacecraft & spacecraft-surface contact forces are first-class, shipped with a complete robotic-arm servicing scenario (`ARC_ChaserBody`, `ARC_Target`, `ARC_ArmSegment`, `ARC_Hand`), lander (`LanderBody`+`LanderLeg`), and legged rover (`RoverBody`+`RoverLowerLeg/UpperLeg`). Basilisk also has contact via MuJoCo, but 42's are native and bundled with mission assets.
- **Mission-grade 3D model library shipped in-repo** — `Model/` contains CubeSats (`Cubesat_1U/3U/6U`), real asteroids with surface crust meshes (`67P`, `Eros`, `Itokawa`, `RQ36`/Bennu, `Wirtanen`, `Haidea`), `CMGSat`, `Telescope`, `Aura`, `HST`, `ISS`.
- **Hardware-in-the-loop as a first-class design** — socket IPC (`42commlink.c`, `42ipc.c`), GMSEC, **NOS3 integration** (`42nos3.c`), **cFS FSW mode** (`CfsFSW`), standalone FSW app (`AcApp.c`).
- **RW LuGre friction model** with Stribeck + viscous terms (detailed wheel friction physics).
- **Julia + MATLAB Monte-Carlo wrappers** (`RunMC.jl`, `RunMC.m`).
- **Public-domain-style U.S. Government licensing** — source files state no copyright is claimed under Title 17 and "All Other Rights Reserved"; SourceForge lists NASA Open Source Agreement (NOSA). Redistribution/commercial-use terms should be checked directly, but the terms are permissive for government/contractor use.
- **Fast concept-study workflow** — text input files, fast setup ("fast setup for concept studies" per README).

---

## 5. Industrial / SOTA Quality Assessment

"SOTA / industrial quality" here means **software engineering maturity and algorithmic state-of-the-art**, not raw feature count.

- **Basilisk** is the more SOTA and industrial-grade by standard SE criteria: modular C++/Python/Rust architecture, SWIG Python API, PyPI distribution, Docker multi-arch, 427-test pytest suite, 9 CI workflows, pre-commit/clang-format, Sphinx documentation, 153 example scenarios, active multi-contributor development under a university lab (CU Boulder LASP). It implements current research-grade algorithms — UKF/EKF/SEKF attitude determination, optical-nav relative OD, CNN image processing, Lambert guidance — that are individually unit-testable. ([Basilisk repo](https://github.com/AVSLab/basilisk))
- **42** is mature and physically rigorous but reflects an older single-author C style: monolithic source (~26 k lines across 24 files), text-file configuration, no formal unit tests, minimal CI. Its "industrial" strength is in **domain completeness for contact/servicing/HWIL** and in **built-in visualization**, not in software-engineering modernity. It is a NASA-GSFC product with public-domain licensing, which eases government/contractor adoption. ([42 repo](https://github.com/ericstoneking/42))

**Verdict:** For algorithmic SOTA + industrial SE quality, **Basilisk**. For turn-key mission visualization + HWIL integration + contact/servicing dynamics out of the box, **42**.

---

## 6. Usage: Servicing Satellites (RPO / OSAM)

**42 — stronger as the simulator infrastructure.** 42 was designed from the outset for multi-spacecraft rendezvous, proximity operations, and precision formation flying ([42 README](https://github.com/ericstoneking/42)). It was used in NASA GSFC's **Precision Formation Flying Integrated Analysis Tool (PFFIAT)** at the Formation Flying Test Bed for closed-loop optical-system control across multiple spacecraft ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20040171251/downloads/20040171251.pdf)). A 2024 *J. Guidance, Control, and Dynamics* paper validated reconfigurable fault-tolerant formation-flying guidance "in a high-fidelity simulation scenario, the 42 spacecraft simulator developed by NASA Goddard" ([AIAA ARC](https://arc.aiaa.org/doi/10.2514/1.G008087)). A STAR-CCM+ ↔ 42 interface was built at GSFC for fuel-slosh / coupled-dynamics analysis ([NASA Tech Briefs](https://www.techbriefs.com/component/content/article/26148-gsc-17081-1)).

**Basilisk — stronger for servicing *algorithms*, newer for contact.** Basilisk is used for visual navigation / proximity-operations research at NASA JPL ("Basilisk spacecraft dynamics simulator and Unreal engine to produce photorealistic camera images" for relative-pose estimation ([JPL poster](https://www.jpl.nasa.gov/site/research/media/posters/2022/R21117p.pdf))). A University of Sydney / IAC-2025 paper built an on-orbit servicing testbed on Basilisk for rendezvous-and-docking imagery validation ([IAF paper](https://iafastro.directory/iac/paper/id/99032/abstract-pdf/IAC-25,D1,4,5,x99032.brief.pdf?2025-03-27.11:05:09)). Basilisk ships `scenarioFormationBasic` and `scenarioRendezVous` examples ([Basilisk docs](https://hanspeterschaub.info/basilisk/examples/scenarioFormationBasic.html)).

**Caveat:** No public evidence was found for either tool in DARPA RSGS, NASA OSAM-1, or Asteroid Redirect (ARCM). Neither has confirmed broad commercial operational servicing pipelines at Lockheed/Boeing/Maxar.

**Assessment:** For servicing **scenario simulation + contact dynamics + HWIL**, **42** (ARC chaser/target/robotic-arm/hand models, surface contact, PFFIAT heritage, STAR-CCM+ coupling). For servicing **algorithm development** (relative navigation, autonomous RPO), **Basilisk** (relative-OD UKF, Lambert guidance, JPL visual-nav research).

---

## 7. Usage: CubeSats / Small Sats

**42 — the NASA small-sat ecosystem.** 42 is the dynamics engine of **NOS3** (NASA Operational Simulator for Small Satellites), explicitly built for the **Simulation-to-Flight 1 (STF-1)** CubeSat — West Virginia's first satellite — and used for FSW development and component testing of STF-1 and Lunar IceCube ([NASA NTRS](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20190001386.pdf), [NASA NOS3](https://www.nasa.gov/nasa-operational-simulation-for-small-satellites/)). **Dellingr**, GSFC's first 6U CubeSat (deployed Nov 2017), used 42 for ACS flight-software verification ("the open source dynamic simulator '42' developed by GSFC Code 591... Most of the ACS app debugging was done on cFS/Linux with 42" ([NASA NTRS](https://ntrs.nasa.gov/api/citations/20170011566/downloads/20170011566.pdf)). **OpenSatKit** bundles cFS + COSMOS + 42 as a complete open-source CubeSat/SmallSat FSW solution ([NASA NTRS](https://ntrs.nasa.gov/citations/20170007710), [OpenSatKit](https://opensatkit.github.io/menu/about.html)). University adopters include **University of Chicago Space Program** (PULSE-A) ([UCSP docs](http://docs.uchicagospaceprogram.org/documentation/avionics/fsw/42-simulator.html)) and **GranSat** (University of Granada) ([GranSat intro](https://granasat.ugr.es/wp-content/uploads/2019/02/42-Intro-to-Simulation.pdf)).

**Basilisk — the university CubeSat ADCS algorithm ecosystem.** Used at **HuskySat-2** (UW, 3U CubeSat) for orbital simulation, GNC validation, and HIL testing of its LOST/FOUND optical navigation ([huskysat.org](https://huskysat.org/projects), [huskysat.org HS-2](https://huskysat.org/hs2-division)); at **Portland State Aerospace Society (OreSat)** in the FlatHILS hardware-in-the-loop infrastructure ([PSAS poster](https://spacegrant.oregonstate.edu/posters/hardware-loop-testing-infrastructure-small-satellite-labs)); and in a peer-reviewed 6U CubeSat differential-drag formation-flying study ([smallsateurope.com](https://smallsateurope.com/paper/mission-level-differential-drag-control-for-formation-flying-of-6u-cubesats-through-automated-management-of-rule-based-and-power-budget-modes/)). Basilisk was explicitly designed to span "earth orbiting cubesats at one end and interplanetary probes... at the other" ([ESA ICATT paper](https://indico.esa.int/event/224/contributions/3869/)); a Basilisk + Raspberry Pi HIL platform was proposed as "ideal for... CubeSats or small satellite form-factors" ([hanspeterschaub.info](https://hanspeterschaub.info/Papers/ColsMargenet2017.pdf)).

**Assessment:** **42 has stronger public evidence for NASA/flight-software small-sat V&V** (NOS3, cFS, STF-1, Dellingr, Lunar IceCube, OpenSatKit). **Basilisk has stronger public evidence for university/CubeSat ADCS algorithm R&D** (Python workflow, modern filters, Monte Carlo, HuskySat-2, OreSat).

---

## 8. Bottom-Line Recommendations by Use Case

| Use case | Recommended tool | Why |
|---|---|---|
| ADCS algorithm R&D (pointing, momentum management) | **Basilisk** | Composable, unit-tested control laws + guidance modes |
| Onboard attitude determination (EKF/UKF) | **Basilisk** | 42 ships no onboard estimators |
| Optical / relative / small-body navigation | **Basilisk** | dedicated opnav, relative-OD UKF, small-body nav, CNN image processing |
| Orbit transfer / Lambert / ΔV guidance | **Basilisk** | ships a Lambert solver + ΔV planner |
| Multi-body flexible / articulated dynamics | **42** (or Basilisk+MuJoCo) | first-class Kane's-method multi-body solver |
| Servicing / RPO scenario simulation + contact | **42** | chaser/target/arm/hand models, surface contact, HWIL |
| Hardware-in-the-loop (cFS, NOS3, GMSEC) | **42** | first-class socket IPC + cFS/NOS3 modes |
| 3D visualization & mission concept demos | **42** | built-in OpenGL/GPGPU, no extra tool |
| Monte Carlo campaigns | **Basilisk** | native Python MC + 153 scenarios |
| CubeSat ADCS development | **Basilisk** | modern Python workflow, open ecosystem, full ADCS suite |
| CubeSat concept visualization | **42** | ships 1U/3U/6U models, fast setup |
| Industrial SE quality / CI / testing | **Basilisk** | 427 tests, 9 CI workflows, PyPI/Docker |

---

## 9. Mission & Research Adoption Evidence

**Basilisk.** Developed at CU Boulder's AVS Lab (Prof. Hanspeter Schaub) to support LASP's work on the **Emirates Mars Mission (EMM) "Hope" probe** — the Mars-orbiting mission for which Basilisk was created as the ADCS simulation framework ([Schaub lab](https://hanspeterschaub.info/research-mission.html), [LASP](https://lasp.colorado.edu/missions/emm-hope/)). Foundational paper: Kenneally, Piggott, Schaub, "Basilisk: A Flexible, Scalable and Modular Astrodynamics Simulation Framework," *J. Aerospace Information Systems* 17(9), 2020 ([AIAA ARC](https://arc.aiaa.org/doi/10.2514/1.I010762), ~30–33 citations per [Semantic Scholar](https://www.semanticscholar.org/paper/Basilisk:-A-Flexible,-Scalable-and-Modular-Kenneally-Piggott/4cba668bc068d3478ca95a9b792fab494f2b7357)). ISC license, ~50 contributors, latest release v2.8.0 (Aug 2025) ([GitHub releases](https://github.com/AVSLab/basilisk/releases)). A companion **BSK-RL** reinforcement-learning package is actively developed ([AVSLab/bsk_rl](https://github.com/AVSLab/bsk_rl)).

**42.** Created by Eric Stoneking in the Attitude Control Systems Engineering Branch at NASA Goddard (Code 591); the broader **NASA Operational Simulator (NOS)** middleware — with 42 as its dynamics component — supported software V&V for **JWST, GPM, Juno, and DSCOVR** ([NASA NTRS](https://ntrs.nasa.gov/citations/20150020491)). The **cFS** flight-software framework 42 ships with was first used on **LRO** (2009) and **GPM** (2014) ([OpenSatKit](https://opensatkit.github.io/menu/about.html)). NASA software catalog ID GSC-16720-1 ([software.nasa.gov](https://software.nasa.gov/software/GSC-16720-1)). NOSA license, ~4 contributors, no formal versioned releases (commit-based). Peer-reviewed uses include a *J. Guidance, Control, and Dynamics* 2024 formation-flying fault-tolerance paper ([AIAA ARC](https://arc.aiaa.org/doi/10.2514/1.G008087)) and the *JoSS* NOS3/STF-1 case study ([JoSS](https://jossonline.com/wp-content/uploads/2019/01/Final-Spolaor-NASA-Operational-Simulator-for-Small-Satellites-NOS3-The-STF-1-CubeSat-Case-Study.pdf)).

**Important nuance:** for flagship missions (JWST, GPM, LRO, DSCOVR), the confirmed simulator is the broader NOS/GDS infrastructure; 42 is the open-source dynamics component within that stack. Commercial operational GNC adoption at Lockheed/Boeing/Maxar/Blue Canyon is **not publicly confirmed** for either tool. No rigorous published head-to-head benchmark between Basilisk and 42 was found.

---

## 10. Gaps & Uncertainties

- **External market share** is not directly measurable from repos; usage claims are limited to public evidence gathered separately (see Usage section).
- **Basilisk's flexible-body story** is a set of effector modules + emerging MuJoCo integration; for a unified multi-body flexible solver, 42 is currently more mature.
- **42's "no onboard estimator"** is by design (truth-based FSW for concept studies); users needing estimation must bring their own FSW over the IPC/socket interface or use the cFS path.
- Both tools evolve; findings reflect repo state at the commit SHAs noted at the top. Future MuJoCo work in Basilisk may close the multi-body/contact gap.
