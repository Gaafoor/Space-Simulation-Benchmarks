#
# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this notice appear in all copies.
#
"""LEO robotic docking simulation for two representative 1U CubeSat buses.

Hub 2 carries an articulated arm that points toward Hub 1. Both vehicles
co-orbit in a 400 km circular Earth orbit and approach under a scripted
thruster maneuver until the arm tip contacts Hub 1. Mesh docking interfaces
and extra contact geoms refine the touch geometry. After tip capture an
equality weld is armed so the pair stays latched. Each arm hinge uses a soft
spring-damper controller.

Orbital environment (MuJoCo-native wiring, no classic ``spacecraft`` object):
* Earth gravity with J2 via spherical harmonics
* Moon and Sun point-mass third bodies through SPICE
* Exponential-atmosphere cannonball drag on both hubs
* Cannonball solar-radiation pressure on both hubs

Per-contact forces are logged from an mjData-synced MuJoCo forward pass
(Basilisk hides ``getMujocoData`` from Python, so the logger mirrors qpos/qvel).
"""

import json
import os
from typing import Any, Sequence

import matplotlib.pyplot as plt
import mujoco as mujoco_native
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import (
    MJCmdForceInertialToForceAtSite,
    MJJointReactionForces,
    NBodyGravity,
    cannonballDrag,
    exponentialAtmosphere,
    mujoco,
    pointMassGravityModel,
    spiceInterface,
    sphericalHarmonicsGravityModel,
    svIntegrators,
)
from Basilisk.utilities import SimulationBaseClass, macros, simSetPlanetEnvironment
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = os.path.join(CURRENT_FOLDER, "sats_cubesat_arm_docking_realistic.xml")
MESH_RECEPTACLE_PATH = os.path.join(CURRENT_FOLDER, "meshes", "docking_receptacle.obj")
MESH_PROBE_PATH = os.path.join(CURRENT_FOLDER, "meshes", "docking_probe.obj")

# Tunable maneuver schedule.  The default values are chosen to brake just
# before the bodies meet, then hold zero thrust long enough to observe whether
# the contact settles or rebounds apart.
# Earth-centered 400 km circular equatorial reference orbit. SPICE uses
# zeroBase="earth", so the absolute LEO ICs below remain valid.
EARTH_MU_M3_S2 = 3.986004415e14
EARTH_EQUATORIAL_RADIUS_M = 6_378_136.3
MOON_MU_M3_S2 = 4.902799e12
SUN_MU_M3_S2 = 1.32712440018e20
SPICE_UTC_INIT = "2022 NOV 14 00:01:10"
EARTH_SPHERICAL_HARMONICS_DEGREE = 2  # J2 (and companion degree-2 terms)
LEO_ALTITUDE_M = 400_000.0
LEO_REFERENCE_RADIUS_M = EARTH_EQUATORIAL_RADIUS_M + LEO_ALTITUDE_M
LEO_CIRCULAR_SPEED_M_S = np.sqrt(EARTH_MU_M3_S2 / LEO_REFERENCE_RADIUS_M)
INITIAL_HUB1_POSITION_M = np.array([LEO_REFERENCE_RADIUS_M - 0.4, 0.0, 0.0])
INITIAL_HUB2_POSITION_M = np.array([LEO_REFERENCE_RADIUS_M + 0.4, 0.0, 0.0])
INITIAL_ORBIT_VELOCITY_M_S = np.array([0.0, LEO_CIRCULAR_SPEED_M_S, 0.0])
# Representative 1U projected area / aero / optical properties.
CUBESAT_PROJECTED_AREA_M2 = 0.01
CUBESAT_DRAG_COEFF = 2.2
CUBESAT_SRP_REFLECTIVITY = 1.3
SOLAR_PRESSURE_AT_1AU_N_PER_M2 = 4.56e-6
AU_M = 149_597_870_700.0
APPROACH_FORCE_N = 0.5
BRAKE_FORCE_N = -1.0
# With two 1.13 kg spacecraft, these phases close the initial ~0.38 m
# arm-tip gap and leave about 0.14 m/s relative closing speed at contact.
APPROACH_DURATION_S = 0.76
BRAKE_DURATION_S = 0.30
ZERO_THRUST_HOLD_DURATION_S = 1.0
POST_HOLD_OBSERVATION_DURATION_S = 3.60
CAPTURE_DISTANCE_M = 0.008
CAPTURE_POLL_STEP_S = 0.01
POST_CAPTURE_HOLD_DURATION_S = 1.50
# Arm tip sphere + probe must actually press the receptacle before the weld
# is armed; otherwise the free-joint wrench is mostly equality force.
CAPTURE_MIN_CONTACT_FORCE_N = 0.05
NUM_ARM_JOINTS = 4
PLOT_DECIMATION_STRIDE = 20
# 1 ms is too coarse for tip contact with the current soft-constraint settings.
SIM_TASK_RATE_S = 0.0005
# Initial arm pose (rad) for arm_1, arm_2, arm_3, arm_tip. Hinges rotate about +Y.
# Z-shape: alternating signs so the links zigzag in the XZ plane at t=0.
# The spring-damper setpoint stays at 0, so the arm unfolds toward straight
# during the approach and joint motion is visible.
INITIAL_ARM_JOINT_ANGLES_RAD = np.deg2rad([-40.0, 40.0, 40.0, -40.0])
# V-shape alternative (uncomment to use):
# INITIAL_ARM_JOINT_ANGLES_RAD = np.deg2rad([35.0, -70.0, 35.0, 0.0])

# Documented fallback DOF layout, derived from the fixed body/joint order in
# sats_cubesat_arm_docking.xml: hub_1 free joint (DOFs 0-5), hub_2 free joint
# (DOFs 6-11), then the 4 arm hinge joints in body-tree order. run() attempts
# to resolve this dynamically from MJJointReactionForces after
# InitializeSimulation() and only falls back to these constants if the
# reaction message isn't populated yet at that point in the lifecycle.
HUB1_FREE_JOINT_DOF_START = 0
ARM_JOINT_DOF_STARTS = [12, 13, 14, 15]
# MuJoCo free joints use seven qpos entries: position (3) + quaternion (4).
# These starts follow the two top-level free bodies in the XML.
HUB1_FREE_JOINT_QPOS_START = 0
HUB2_FREE_JOINT_QPOS_START = 7
# Facing contact sites sit on the chassis faces. Soft contact can close a
# small positive site gap, so treat near-zero gaps as chassis-chassis contact.
HUB_HUB_CONTACT_SITE_TOL_M = 0.01
# Arm tip sphere radius is 0.02 m and the tip site is 0.02 m inward from the
# sphere center, so tip-to-hub1 site distances below ~0.03 m are tip contact.
HUB_ARM_CONTACT_SITE_TOL_M = 0.03


def _copy_double_vector(vector) -> np.ndarray:
    """Copy a Basilisk SWIG ``std::vector<double>`` into NumPy without retaining the wrapper."""
    size = int(vector.size())
    if size == 0:
        return np.empty(0, dtype=float)
    return np.array([float(vector[i]) for i in range(size)], dtype=float)


def _copy_int_vector(vector) -> np.ndarray:
    """Copy a Basilisk SWIG ``std::vector<int>`` into NumPy without retaining the wrapper."""
    size = int(vector.size())
    if size == 0:
        return np.empty(0, dtype=int)
    return np.array([int(vector[i]) for i in range(size)], dtype=int)


def _make_hub1_relative_replay_qpos(qpos: np.ndarray) -> np.ndarray:
    """Recenter recorded free-body positions on Hub 1 for stable visualization.

    The simulation and saved results remain in Earth-centered inertial
    coordinates. Only this replay copy is translated into a local frame.
    """
    replayQpos = np.array(qpos, dtype=float, copy=True)
    if replayQpos.ndim != 2 or replayQpos.shape[1] < HUB2_FREE_JOINT_QPOS_START + 3:
        raise ValueError(f"Unexpected MJScene qpos shape for two free bodies: {replayQpos.shape}")

    hub1Position = replayQpos[
        :, HUB1_FREE_JOINT_QPOS_START:HUB1_FREE_JOINT_QPOS_START + 3
    ].copy()
    replayQpos[
        :, HUB1_FREE_JOINT_QPOS_START:HUB1_FREE_JOINT_QPOS_START + 3
    ] -= hub1Position
    replayQpos[
        :, HUB2_FREE_JOINT_QPOS_START:HUB2_FREE_JOINT_QPOS_START + 3
    ] -= hub1Position
    return replayQpos


def _attribute_hub1_contact_channels(
    hub1ForceMagnitude_N: np.ndarray,
    hub1ToHub2Distance_m: np.ndarray,
    armTipToHub1Distance_m: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Split Hub 1 free-joint contact magnitude into hub-hub and hub-arm channels.

    ``MJJointReactionForces`` reports one free-joint wrench on Hub 1. That wrench
    is the total contact load on Hub 1 from every geom that touches it. The two
    docking partners are attributed with site-gap gates:

    * hub-arm when the arm tip site is within ``HUB_ARM_CONTACT_SITE_TOL_M``
    * hub-hub when the hub contact sites are within ``HUB_HUB_CONTACT_SITE_TOL_M``
      and the tip is *not* in the arm-contact band (arm contact takes priority)
    """
    if not (
        len(hub1ForceMagnitude_N)
        == len(hub1ToHub2Distance_m)
        == len(armTipToHub1Distance_m)
    ):
        raise ValueError("Contact force and distance histories must have equal length.")

    hubArmActive = armTipToHub1Distance_m <= HUB_ARM_CONTACT_SITE_TOL_M
    hubHubActive = (hub1ToHub2Distance_m <= HUB_HUB_CONTACT_SITE_TOL_M) & (~hubArmActive)

    hubToArmWrenchMagnitude_N = np.where(hubArmActive, hub1ForceMagnitude_N, 0.0)
    hubToHubWrenchMagnitude_N = np.where(hubHubActive, hub1ForceMagnitude_N, 0.0)
    return hubToHubWrenchMagnitude_N, hubToArmWrenchMagnitude_N


def _peak_preserving_downsample(
    times_s: np.ndarray,
    *series: np.ndarray,
    stride: int = PLOT_DECIMATION_STRIDE,
) -> tuple[np.ndarray, ...]:
    """Downsample while keeping the maximum absolute sample in each stride bin.

    Contact spikes can last only a few milliseconds. Plain ``array[::stride]``
    often skips them entirely, which makes the wrench plot look empty.
    """
    if stride <= 1 or len(times_s) <= stride:
        return (times_s, *series)

    nBins = int(np.ceil(len(times_s) / stride))
    outTimes = np.empty(nBins, dtype=float)
    outSeries = [np.empty(nBins, dtype=float) for _ in series]
    for binIdx in range(nBins):
        start = binIdx * stride
        stop = min(len(times_s), start + stride)
        # Choose the sample whose absolute value is largest across all series.
        localAbs = np.zeros(stop - start, dtype=float)
        for values in series:
            localAbs = np.maximum(localAbs, np.abs(values[start:stop]))
        peakOffset = int(np.argmax(localAbs))
        sampleIdx = start + peakOffset
        outTimes[binIdx] = times_s[sampleIdx]
        for seriesIdx, values in enumerate(series):
            outSeries[seriesIdx][binIdx] = values[sampleIdx]
    return (outTimes, *outSeries)


def setThrusterForce(msgs: Sequence[messaging.SingleActuatorMsg], thrust: Sequence[float]):
    """Write the requested force command to each actuator message."""
    for msg, value in zip(msgs, thrust):
        msg.write(messaging.SingleActuatorMsgPayload(input=value))


def _find_free_joint_starts(joint_types: Sequence[int], joint_dof_start: Sequence[int]) -> tuple[int, int]:
    """Return the qfrc_constraint start indices for the two free hubs."""
    joint_types_arr = np.asarray(joint_types).reshape(-1)
    joint_dof_start_arr = np.asarray(joint_dof_start).reshape(-1)
    free_joint_starts = [
        int(start)
        for jtype, start in zip(joint_types_arr, joint_dof_start_arr)
        if int(np.asarray(jtype).reshape(())) == 0
    ]
    if len(free_joint_starts) < 2:
        raise RuntimeError("Expected two free joints for the two CubeSat hubs.")
    return int(free_joint_starts[0]), int(free_joint_starts[1])





def _find_hinge_joint_starts(
    joint_types: Sequence[int],
    joint_dof_start: Sequence[int],
    expected_count: int = NUM_ARM_JOINTS,
) -> list[int]:
    """Return the scalar DOF start index for every hinge joint in model order."""
    joint_types_arr = np.asarray(joint_types).reshape(-1)
    joint_dof_start_arr = np.asarray(joint_dof_start).reshape(-1)
    hinge_joint_starts = [
        int(start)
        for jtype, start in zip(joint_types_arr, joint_dof_start_arr)
        if int(np.asarray(jtype).reshape(())) == 3
    ]
    if len(hinge_joint_starts) < expected_count:
        raise RuntimeError(
            f"Expected at least {expected_count} hinge joints, found {len(hinge_joint_starts)}."
        )
    return hinge_joint_starts[-expected_count:]


def _read_reaction_force_layout(jointReactionForces: MJJointReactionForces.MJJointReactionForces) -> tuple[np.ndarray, np.ndarray, int]:
    """Return joint metadata and the hub-1 free-joint DOF start from ``MJJointReactionForces``."""
    layoutPayload = jointReactionForces.reactionForcesOutMsg.read()
    jointTypes = _copy_int_vector(layoutPayload.jointTypes)
    jointDOFStart = _copy_int_vector(layoutPayload.jointDOFStart)
    hub1Start, _ = _find_free_joint_starts(jointTypes, jointDOFStart)
    return jointTypes, jointDOFStart, hub1Start


class CannonballSrp(sysModel.SysModel):
    """Cannonball solar-radiation pressure for one MuJoCo spacecraft body."""

    def __init__(self):
        super().__init__()
        self.solarPressureAt1Au_N_per_m2 = SOLAR_PRESSURE_AT_1AU_N_PER_M2
        self.au_m = AU_M
        self.area_m2 = CUBESAT_PROJECTED_AREA_M2
        self.reflectivityCoeff = CUBESAT_SRP_REFLECTIVITY
        self.scStateInMsg = messaging.SCStatesMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.cmdForceOutMsg = messaging.CmdForceInertialMsg()

    def UpdateState(self, CurrentSimNanos: int):
        scState = self.scStateInMsg()
        sunState = self.sunStateInMsg()
        rSc_N = np.asarray(scState.r_BN_N, dtype=float).reshape(3)
        rSun_N = np.asarray(sunState.PositionVector, dtype=float).reshape(3)
        rSunToSc_N = rSc_N - rSun_N
        distance_m = float(np.linalg.norm(rSunToSc_N))
        if not np.isfinite(distance_m) or distance_m < 1.0:
            force_N = np.zeros(3, dtype=float)
        else:
            scale_N = (
                self.solarPressureAt1Au_N_per_m2
                * self.area_m2
                * self.reflectivityCoeff
                * (self.au_m / distance_m) ** 2
            )
            force_N = scale_N * (rSunToSc_N / distance_m)

        payload = messaging.CmdForceInertialMsgPayload()
        payload.forceRequestInertial = force_N.tolist()
        self.cmdForceOutMsg.write(payload, CurrentSimNanos, self.moduleID)


class MjDataContactLogger(sysModel.SysModel):
    """Log per-contact wrenches from an mjData-synced MuJoCo forward pass.

    Basilisk's SWIG layer ignores ``MJScene.getMujocoData()``, so this logger
    mirrors the live ``qpos``/``qvel`` into a parallel MuJoCo model and calls
    ``mj_forward`` + ``mj_contactForce`` each tick.

    Free-body positions are recentered on Hub 1 before the forward pass. At LEO
    radii (~6.8e6 m) raw inertial coordinates make MuJoCo contact detection
    unreliable for centimetre-scale docking geoms.
    """

    def __init__(self, xmlPath: str):
        super().__init__()
        self.xmlPath = xmlPath
        self.sceneStateInMsg = messaging.MJSceneStateMsgReader()
        self._model: mujoco_native.MjModel | None = None
        self._data: mujoco_native.MjData | None = None
        self._hub1GeomIds: set[int] = set()
        self._hub2ChassisGeomIds: set[int] = set()
        self._armGeomIds: set[int] = set()
        self.times_s: list[float] = []
        self.nContacts: list[int] = []
        self.hubArmForceMagnitude_N: list[float] = []
        self.hubHubForceMagnitude_N: list[float] = []
        self.totalContactForceMagnitude_N: list[float] = []
        self.contactRecords: list[dict[str, Any]] = []
        self.latestHubArmForce_N = 0.0
        self.latestHubHubForce_N = 0.0
        self.latestTotalForce_N = 0.0
        self.latestNContacts = 0

    def Reset(self, CurrentSimNanos: int):
        self._model = mujoco_native.MjModel.from_xml_path(self.xmlPath)
        self._data = mujoco_native.MjData(self._model)
        self._hub1GeomIds.clear()
        self._hub2ChassisGeomIds.clear()
        self._armGeomIds.clear()
        for geomId in range(self._model.ngeom):
            name = mujoco_native.mj_id2name(self._model, mujoco_native.mjtObj.mjOBJ_GEOM, geomId)
            if not name:
                continue
            if name.startswith("hub_1"):
                self._hub1GeomIds.add(geomId)
            elif name == "hub_2_chassis":
                self._hub2ChassisGeomIds.add(geomId)
            elif name.startswith("arm_"):
                self._armGeomIds.add(geomId)
        self.times_s.clear()
        self.nContacts.clear()
        self.hubArmForceMagnitude_N.clear()
        self.hubHubForceMagnitude_N.clear()
        self.totalContactForceMagnitude_N.clear()
        self.contactRecords.clear()
        self.latestHubArmForce_N = 0.0
        self.latestHubHubForce_N = 0.0
        self.latestTotalForce_N = 0.0
        self.latestNContacts = 0

    def UpdateState(self, CurrentSimNanos: int):
        if self._model is None or self._data is None:
            return
        sceneState = self.sceneStateInMsg()
        qpos = np.asarray(sceneState.qpos, dtype=float).reshape(-1)
        qvel = np.asarray(sceneState.qvel, dtype=float).reshape(-1)
        if qpos.size != self._model.nq or qvel.size != self._model.nv:
            return

        qposLocal = np.array(qpos, dtype=float, copy=True)
        hub1Position = qposLocal[HUB1_FREE_JOINT_QPOS_START:HUB1_FREE_JOINT_QPOS_START + 3].copy()
        qposLocal[HUB1_FREE_JOINT_QPOS_START:HUB1_FREE_JOINT_QPOS_START + 3] -= hub1Position
        qposLocal[HUB2_FREE_JOINT_QPOS_START:HUB2_FREE_JOINT_QPOS_START + 3] -= hub1Position

        self._data.qpos[:] = qposLocal
        self._data.qvel[:] = qvel
        mujoco_native.mj_forward(self._model, self._data)

        hubArmForce = 0.0
        hubHubForce = 0.0
        totalForce = 0.0
        force6 = np.zeros(6, dtype=float)
        for contactIdx in range(int(self._data.ncon)):
            contact = self._data.contact[contactIdx]
            mujoco_native.mj_contactForce(self._model, self._data, contactIdx, force6)
            forceMag = float(np.linalg.norm(force6[0:3]))
            totalForce += forceMag
            geomPair = {int(contact.geom1), int(contact.geom2)}
            touchesHub1 = bool(geomPair & self._hub1GeomIds)
            touchesArm = bool(geomPair & self._armGeomIds)
            touchesHub2Chassis = bool(geomPair & self._hub2ChassisGeomIds)
            if touchesHub1 and touchesArm:
                hubArmForce += forceMag
            elif touchesHub1 and touchesHub2Chassis:
                hubHubForce += forceMag

            if forceMag > 1e-9:
                name1 = mujoco_native.mj_id2name(
                    self._model, mujoco_native.mjtObj.mjOBJ_GEOM, int(contact.geom1)
                )
                name2 = mujoco_native.mj_id2name(
                    self._model, mujoco_native.mjtObj.mjOBJ_GEOM, int(contact.geom2)
                )
                self.contactRecords.append(
                    {
                        "time_s": CurrentSimNanos * macros.NANO2SEC,
                        "geom1": name1,
                        "geom2": name2,
                        "force_normal_N": float(force6[0]),
                        "force_magnitude_N": forceMag,
                        "contact_distance_m": float(contact.dist),
                    }
                )

        self.latestHubArmForce_N = hubArmForce
        self.latestHubHubForce_N = hubHubForce
        self.latestTotalForce_N = totalForce
        self.latestNContacts = int(self._data.ncon)
        self.times_s.append(CurrentSimNanos * macros.NANO2SEC)
        self.nContacts.append(self.latestNContacts)
        self.hubArmForceMagnitude_N.append(hubArmForce)
        self.hubHubForceMagnitude_N.append(hubHubForce)
        self.totalContactForceMagnitude_N.append(totalForce)


def getDockSiteDistance_m(scene: mujoco.MJScene) -> float:
    """Return the inertial distance between the capture weld sites."""
    hub1Dock = scene.getBody("hub_1").getSite("hub_1_dock_site").stateOutMsg.read()
    tipDock = scene.getBody("arm_tip").getSite("arm_tip_dock_site").stateOutMsg.read()
    return float(np.linalg.norm(np.asarray(hub1Dock.r_BN_N) - np.asarray(tipDock.r_BN_N)))


class ActiveImpedanceController(sysModel.SysModel):
    """
    Stable discrete spring-damper controller for one arm joint.

    Implemented as a plain ``SysModel`` (not ``StatefulSysModel``) because it
    has no continuous-time Basilisk states to integrate. Reaction forces are
    recorded for post-processing, not read back inside the control loop. Reading
    a recorder's complete growing history every tick made runtime quadratic.
    """

    def __init__(self):
        super().__init__()

        # ==========================================================
        # MOTOR TORQUE LIMIT
        # ==========================================================

        # Arm link inertias in sats_cubesat_arm_docking.xml are O(1e-5)
        # kg*m^2. Keep the motor torque small enough for a bent initial pose
        # to unfold without destabilizing the fixed-step integration.
        self.torqueLimitNm = 0.001

        # ==========================================================
        # PASSIVE / ACTIVE IMPEDANCE
        # ==========================================================

        # Soft spring / damper scaled to the tiny arm inertias. The XML adds
        # 0.002 N*m*s/rad of passive hinge damping.
        self.nominalStiffnessNmPerRad = 0.005
        self.nominalDampingNmSPerRad = 0.001

        # ==========================================================
        # MESSAGES
        # ==========================================================

        self.measuredPosInMsg = messaging.ScalarJointStateMsgReader()

        self.measuredVelInMsg = messaging.ScalarJointStateMsgReader()

        self.outputOutMsg = messaging.SingleActuatorMsg()

    def UpdateState(self, CurrentSimNanos: int):

        # ==========================================================
        # READ JOINT POSITION
        # ==========================================================

        measuredJointState = self.measuredPosInMsg()

        jointAngleRad = float(
            measuredJointState.state
        )

        # ==========================================================
        # READ JOINT VELOCITY
        # ==========================================================

        measuredJointRate = self.measuredVelInMsg()

        jointRateRadPerSec = float(
            measuredJointRate.state
        )

        # Target a straight arm. With a bent initial pose the joints visibly
        # unfold toward this setpoint; XML damping adds passive compliance.
        virtualEquilibriumRad = 0.0

        # ==========================================================
        # SPRING ERROR
        # ==========================================================

        positionErrorRad = (

            virtualEquilibriumRad
            -
            jointAngleRad

        )

        # ==========================================================
        # DAMPING ERROR
        # ==========================================================

        velocityErrorRadPerSec = (

            -
            jointRateRadPerSec

        )

        # ==========================================================
        # ACTIVE IMPEDANCE TORQUE
        # ==========================================================

        torqueNm = (

            self.nominalStiffnessNmPerRad
            *
            positionErrorRad

            +

            self.nominalDampingNmSPerRad
            *
            velocityErrorRadPerSec

        )

        # ==========================================================
        # MOTOR TORQUE SATURATION
        # ==========================================================

        # Saturate at the inertia-compatible motor limit.
        torqueNm = float(

            np.clip(

                torqueNm,

                -self.torqueLimitNm,

                self.torqueLimitNm

            )

        )

        if not np.isfinite(torqueNm):
            torqueNm = 0.0

        # ==========================================================
        # SEND MOTOR TORQUE
        # ==========================================================

        payload = messaging.SingleActuatorMsgPayload(
            input=torqueNm
        )

        self.outputOutMsg.write(

            payload,

            CurrentSimNanos,

            self.moduleID

        )


def run(showPlots: bool = False, visualize: bool = False):
    """Run the active impedance CubeSat approach, impact, and rebound case."""
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(SIM_TASK_RATE_S)))

    scene = mujoco.MJScene.fromFile(
        XML_PATH,
        files=[MESH_RECEPTACLE_PATH, MESH_PROBE_PATH],
    )
    scene.extraEoMCall = True
    scSim.AddModelToTask("test", scene, 200)

    # Fixed-step RK4 avoids the RKF45 contact hang. Contact solref/solimp in the
    # XML were softened so this step size remains stable through impact.
    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    # Keep environment modules / messages alive for the whole run().
    environmentKeeps: list[Any] = []

    # SPICE ephemerides in an Earth-centered frame so the LEO ICs stay valid.
    spice = spiceInterface.SpiceInterface()
    spice.ModelTag = "spiceEarthMoonSun"
    spice.UTCCalInit = SPICE_UTC_INIT
    spice.zeroBase = "earth"
    spice.addPlanetNames(["earth", "moon", "sun"])
    scene.AddModelToDynamicsTask(spice)
    environmentKeeps.append(spice)

    # Earth J2 (degree-2 spherical harmonics) + Moon/Sun point-mass thirds.
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "nBodyGravityEarthMoonSun"
    scene.AddModelToDynamicsTask(gravity)

    earthGravityModel = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
    earthGravityModel.muBody = EARTH_MU_M3_S2
    earthGravityModel.radEquator = EARTH_EQUATORIAL_RADIUS_M
    earthGravityModel.loadFromFile(
        str(get_path(DataFile.LocalGravData.GGM03S)),
        EARTH_SPHERICAL_HARMONICS_DEGREE,
    )
    earthGravitySource = gravity.addGravitySource("earth", earthGravityModel, True)
    earthGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[0])

    moonGravityModel = pointMassGravityModel.PointMassGravityModel()
    moonGravityModel.muBody = MOON_MU_M3_S2
    moonGravitySource = gravity.addGravitySource("moon", moonGravityModel, False)
    moonGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[1])

    sunGravityModel = pointMassGravityModel.PointMassGravityModel()
    sunGravityModel.muBody = SUN_MU_M3_S2
    sunGravitySource = gravity.addGravitySource("sun", sunGravityModel, False)
    sunGravitySource.stateInMsg.subscribeTo(spice.planetStateOutMsgs[2])

    gravityBodyNames = ["hub_1", "hub_2", "arm_1", "arm_2", "arm_3", "arm_tip"]
    for bodyName in gravityBodyNames:
        gravity.addGravityTarget(bodyName, scene.getBody(bodyName))
    environmentKeeps.extend(
        [gravity, earthGravityModel, moonGravityModel, sunGravityModel]
    )

    hub1Body = scene.getBody("hub_1")
    hub2Body = scene.getBody("hub_2")

    # Exponential atmosphere + cannonball drag on both hubs.
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "expAtmoEarth"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.planetPosInMsg.subscribeTo(spice.planetStateOutMsgs[0])
    atmo.addSpacecraftToModel(hub1Body.getCenterOfMass().stateOutMsg)
    atmo.addSpacecraftToModel(hub2Body.getCenterOfMass().stateOutMsg)
    scene.AddModelToDynamicsTask(atmo)
    environmentKeeps.append(atmo)

    dragModels = []
    dragGeometryMsgs = []
    for hubOrdinal, hubBody in enumerate((hub1Body, hub2Body)):
        drag = cannonballDrag.CannonballDrag()
        drag.ModelTag = f"drag_hub_{hubOrdinal + 1}"
        scene.AddModelToDynamicsTask(drag)
        drag.applyTo(hubBody)
        dragGeomMsg = messaging.DragGeometryMsg()
        dragGeomMsg.write(
            messaging.DragGeometryMsgPayload(
                projectedArea=CUBESAT_PROJECTED_AREA_M2,
                dragCoeff=CUBESAT_DRAG_COEFF,
                r_CP_S=[0.0, 0.0, 0.0],
            )
        )
        drag.dragGeometryInMsg.subscribeTo(dragGeomMsg)
        drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[hubOrdinal])
        dragModels.append(drag)
        dragGeometryMsgs.append(dragGeomMsg)
    environmentKeeps.extend(dragModels + dragGeometryMsgs)

    # Cannonball SRP on both hubs, mapped through site-frame force actuators.
    srpModels = []
    srpAdapters = []
    srpActuators = []
    for hubOrdinal, hubBody in enumerate((hub1Body, hub2Body)):
        srp = CannonballSrp()
        srp.ModelTag = f"srp_hub_{hubOrdinal + 1}"
        srp.scStateInMsg.subscribeTo(hubBody.getCenterOfMass().stateOutMsg)
        srp.sunStateInMsg.subscribeTo(spice.planetStateOutMsgs[2])
        scene.AddModelToDynamicsTask(srp)

        srpActuator = scene.addForceActuator(
            f"srp_force_hub_{hubOrdinal + 1}",
            hubBody.getCenterOfMass(),
        )
        inertialToSite = MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite()
        inertialToSite.ModelTag = f"srp_inertial_to_site_hub_{hubOrdinal + 1}"
        inertialToSite.siteFrameStateInMsg.subscribeTo(hubBody.getCenterOfMass().stateOutMsg)
        inertialToSite.cmdForceInertialInMsg.subscribeTo(srp.cmdForceOutMsg)
        srpActuator.forceInMsg.subscribeTo(inertialToSite.forceOutMsg)
        scene.AddModelToDynamicsTask(inertialToSite)

        srpModels.append(srp)
        srpAdapters.append(inertialToSite)
        srpActuators.append(srpActuator)
    environmentKeeps.extend(srpModels + srpAdapters + srpActuators)

    hub1ThrusterMsg = messaging.SingleActuatorMsg()
    hub2ThrusterMsg = messaging.SingleActuatorMsg()
    scene.getSingleActuator("hub_1_thruster").actuatorInMsg.subscribeTo(hub1ThrusterMsg)
    scene.getSingleActuator("hub_2_thruster").actuatorInMsg.subscribeTo(hub2ThrusterMsg)

    jointReactionForces = MJJointReactionForces.MJJointReactionForces()
    jointReactionForces.ModelTag = "MJJointReactionForces"
    jointReactionForces.scene = scene
    scSim.AddModelToTask("test", jointReactionForces, 199)

    armJointSpecs = [
        ("arm_1", "arm_1_joint", "arm_1_motor"),
        ("arm_2", "arm_2_joint", "arm_2_motor"),
        ("arm_3", "arm_3_joint", "arm_3_motor"),
        ("arm_tip", "arm_tip_joint", "arm_tip_motor"),
    ]
    armControllers = []
    armJointStateRecorders = []
    armJointRateRecorders = []
    armJointTorqueRecorders = []

    for armJointOrdinal, (bodyName, jointName, actuatorName) in enumerate(armJointSpecs):
        joint = scene.getBody(bodyName).getScalarJoint(jointName)
        controller = ActiveImpedanceController()
        controller.ModelTag = f"{jointName}_impedance_controller"
        controller.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
        controller.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)
        scene.getSingleActuator(actuatorName).actuatorInMsg.subscribeTo(controller.outputOutMsg)

        # Discrete spring-damper control on the process task.
        scSim.AddModelToTask("test", controller, 190 - armJointOrdinal)

        armControllers.append(controller)
        armJointStateRecorders.append(joint.stateOutMsg.recorder())
        armJointRateRecorders.append(joint.stateDotOutMsg.recorder())
        armJointTorqueRecorders.append(controller.outputOutMsg.recorder())

    hub1ContactRecorder = scene.getBody("hub_1").getSite("hub_1_contact_point").stateOutMsg.recorder()
    armTipRecorder = scene.getBody("arm_tip").getSite("arm_tip_contact_point").stateOutMsg.recorder()
    hub2ContactRecorder = scene.getBody("hub_2").getSite("hub_2_contact_point").stateOutMsg.recorder()
    hub1DockRecorder = scene.getBody("hub_1").getSite("hub_1_dock_site").stateOutMsg.recorder()
    tipDockRecorder = scene.getBody("arm_tip").getSite("arm_tip_dock_site").stateOutMsg.recorder()
    reactionForcesRecorder = jointReactionForces.reactionForcesOutMsg.recorder()
    stateRecorder = scene.stateOutMsg.recorder()

    contactLogger = MjDataContactLogger(XML_PATH)
    contactLogger.ModelTag = "mjDataContactLogger"
    contactLogger.sceneStateInMsg.subscribeTo(scene.stateOutMsg)
    scSim.AddModelToTask("test", contactLogger, 150)

    for recorder in [
        hub1ContactRecorder,
        armTipRecorder,
        hub2ContactRecorder,
        hub1DockRecorder,
        tipDockRecorder,
        reactionForcesRecorder,
        stateRecorder,
        *armJointStateRecorders,
        *armJointRateRecorders,
        *armJointTorqueRecorders,
    ]:
        scSim.AddModelToTask("test", recorder)

    scSim.InitializeSimulation()

    # DOF-start resolution: dynamic lookup was evaluated and is not usable
    # here. Reading jointTypes/jointDOFStart requires either accessing the
    # live MJJointReactionsMsgPayload directly (its std::vector<int>/
    # std::vector<double> fields come back as unwrapped SWIG pointers in
    # this Basilisk build -- see ReactionForceCache above) or waiting for
    # the shared recorder to collect its first sample, which only happens
    # once ExecuteSimulation() has advanced at least one timestep -- i.e.
    # after the controllers below would already need valid DOF starts.
    # The documented, hardcoded ARM_JOINT_DOF_STARTS / HUB1_FREE_JOINT_DOF_START
    # constants (derived from the fixed body/joint order in
    # sats_cubesat_arm_docking.xml and already cross-checked against the
    # results JSON) are therefore used directly; the controllers were
    # already constructed with them above.

    # Keep the geometry face-to-face along the local radial X direction. The
    # arm on the outer chaser points inward toward the target. A common +Y
    # circular velocity gives both vehicles the same initial co-orbiting state.
    scene.getBody("hub_1").setPosition(INITIAL_HUB1_POSITION_M.tolist())
    scene.getBody("hub_2").setPosition(INITIAL_HUB2_POSITION_M.tolist())
    scene.getBody("hub_1").setVelocity(INITIAL_ORBIT_VELOCITY_M_S.tolist())
    scene.getBody("hub_2").setVelocity(INITIAL_ORBIT_VELOCITY_M_S.tolist())

    # Start bent (Z-shape by default) so joint motion is obvious as the
    # impedance controller pulls each hinge toward the straight setpoint.
    if len(INITIAL_ARM_JOINT_ANGLES_RAD) != len(armJointSpecs):
        raise ValueError(
            f"Expected {len(armJointSpecs)} initial arm angles, "
            f"got {len(INITIAL_ARM_JOINT_ANGLES_RAD)}"
        )
    print(
        "[sim] initial arm joints (deg): "
        + ", ".join(
            f"{name}={np.rad2deg(angle):+.1f}"
            for (_, name, _), angle in zip(armJointSpecs, INITIAL_ARM_JOINT_ANGLES_RAD)
        ),
        flush=True,
    )
    for (bodyName, jointName, _), angleRad in zip(armJointSpecs, INITIAL_ARM_JOINT_ANGLES_RAD):
        scene.getBody(bodyName).getScalarJoint(jointName).setPosition(float(angleRad))

    thrustMessages = [hub1ThrusterMsg, hub2ThrusterMsg]
    maneuverPhases = [
        {"name": "approach", "force_N": [APPROACH_FORCE_N, APPROACH_FORCE_N], "duration_s": APPROACH_DURATION_S},
        {"name": "brake", "force_N": [BRAKE_FORCE_N, BRAKE_FORCE_N], "duration_s": BRAKE_DURATION_S},
    ]
    maneuverStopTime_s = 0.0
    for phase in maneuverPhases:
        setThrusterForce(thrustMessages, phase["force_N"])
        maneuverStopTime_s += phase["duration_s"]
        print(f"[sim] phase={phase['name']} -> t={maneuverStopTime_s:.3f}s", flush=True)
        scSim.ConfigureStopTime(macros.sec2nano(maneuverStopTime_s))
        scSim.ExecuteSimulation()

    # Coast while polling the dock sites; arm the weld as soon as capture occurs.
    setThrusterForce(thrustMessages, [0.0, 0.0])
    captureDeadline_s = maneuverStopTime_s + ZERO_THRUST_HOLD_DURATION_S + POST_HOLD_OBSERVATION_DURATION_S
    dockActivated = False
    captureTime_s = None
    currentStop_s = maneuverStopTime_s
    print(
        f"[sim] phase=capture_search (threshold={CAPTURE_DISTANCE_M:.3f} m) "
        f"-> deadline t={captureDeadline_s:.3f}s",
        flush=True,
    )
    while currentStop_s < captureDeadline_s and not dockActivated:
        currentStop_s = min(captureDeadline_s, currentStop_s + CAPTURE_POLL_STEP_S)
        scSim.ConfigureStopTime(macros.sec2nano(currentStop_s))
        scSim.ExecuteSimulation()
        dockGap_m = getDockSiteDistance_m(scene)
        tipPressed = contactLogger.latestHubArmForce_N >= CAPTURE_MIN_CONTACT_FORCE_N
        if dockGap_m <= CAPTURE_DISTANCE_M and tipPressed:
            scene.getEquality("dock_capture").setActive(True)
            dockActivated = True
            captureTime_s = currentStop_s
            print(
                f"[sim] capture weld armed at t={captureTime_s:.3f}s "
                f"(dock gap={dockGap_m:.4f} m, hub-arm contact={contactLogger.latestHubArmForce_N:.3f} N)",
                flush=True,
            )

    # Hold after capture (or after the search window) to observe the latched state.
    setThrusterForce(thrustMessages, [0.0, 0.0])
    finalStop_s = currentStop_s + POST_CAPTURE_HOLD_DURATION_S
    print(
        f"[sim] phase={'post_capture_hold' if dockActivated else 'post_hold_observation'} "
        f"-> t={finalStop_s:.3f}s (weld_active={dockActivated})",
        flush=True,
    )
    scSim.ConfigureStopTime(macros.sec2nano(finalStop_s))
    scSim.ExecuteSimulation()
    print("[sim] complete; writing results / opening plots", flush=True)

    rHub1_N = np.squeeze(hub1ContactRecorder.r_BN_N)
    rArmTip_N = np.squeeze(armTipRecorder.r_BN_N)
    rHub2_N = np.squeeze(hub2ContactRecorder.r_BN_N)
    rHub1Dock_N = np.squeeze(hub1DockRecorder.r_BN_N)
    rTipDock_N = np.squeeze(tipDockRecorder.r_BN_N)
    hub1ToHub2Distance = np.linalg.norm(rHub1_N - rHub2_N, axis=1)
    armTipToHub1Distance = np.linalg.norm(rArmTip_N - rHub1_N, axis=1)
    dockSiteDistance = np.linalg.norm(rHub1Dock_N - rTipDock_N, axis=1)
    perContactTimes = np.asarray(contactLogger.times_s, dtype=float)
    perContactHubArm = np.asarray(contactLogger.hubArmForceMagnitude_N, dtype=float)
    perContactHubHub = np.asarray(contactLogger.hubHubForceMagnitude_N, dtype=float)
    perContactTotal = np.asarray(contactLogger.totalContactForceMagnitude_N, dtype=float)
    perContactCount = np.asarray(contactLogger.nContacts, dtype=int)
    constraintForces = np.asarray(reactionForcesRecorder.constraintForces, dtype=float)
    jointTypes = np.asarray(reactionForcesRecorder.jointTypes)
    jointDOFStart = np.asarray(reactionForcesRecorder.jointDOFStart)
    hub1Start, hub2Start = _find_free_joint_starts(jointTypes, jointDOFStart)
    hingeStarts = _find_hinge_joint_starts(jointTypes, jointDOFStart)
    hub1ContactWrench = constraintForces[:, hub1Start:hub1Start + 6]
    hub2ContactWrench = constraintForces[:, hub2Start:hub2Start + 6]
    hub1ContactForce_N = hub1ContactWrench[:, 0:3]
    hub2ContactForce_N = hub2ContactWrench[:, 0:3]
    hub1ContactForceMagnitude = np.linalg.norm(hub1ContactForce_N, axis=1)
    hub2ContactForceMagnitude = np.linalg.norm(hub2ContactForce_N, axis=1)
    actuatorForces = np.asarray(reactionForcesRecorder.actuatorForces)
    actuatorForceMagnitude = np.linalg.norm(actuatorForces, axis=1)
    contactTimes = armTipRecorder.times() * macros.NANO2SEC
    reactionTimes = reactionForcesRecorder.times() * macros.NANO2SEC

    # Hub1 free-joint wrench is total contact on Hub 1. Attribute it to the
    # physically active docking partner using the recorded site gaps.
    hubToHubWrenchMagnitude = np.zeros_like(hub1ContactForceMagnitude)
    hubToArmWrenchMagnitude = np.zeros_like(hub1ContactForceMagnitude)
    if len(reactionTimes) == len(contactTimes):
        hubToHubWrenchMagnitude, hubToArmWrenchMagnitude = _attribute_hub1_contact_channels(
            hub1ContactForceMagnitude,
            hub1ToHub2Distance,
            armTipToHub1Distance,
        )
    else:
        # Recorders can disagree by one sample at the stop boundary; interpolate.
        hub1ToHub2OnReaction = np.interp(reactionTimes, contactTimes, hub1ToHub2Distance)
        armTipToHub1OnReaction = np.interp(reactionTimes, contactTimes, armTipToHub1Distance)
        hubToHubWrenchMagnitude, hubToArmWrenchMagnitude = _attribute_hub1_contact_channels(
            hub1ContactForceMagnitude,
            hub1ToHub2OnReaction,
            armTipToHub1OnReaction,
        )

    impactCenterIdx = int(np.argmin(armTipToHub1Distance))
    impactCenterTime_s = float(contactTimes[impactCenterIdx])
    impactStart_s = float(max(0.0, impactCenterTime_s - 0.05))
    impactEnd_s = float(impactCenterTime_s + 0.05)
    impactMask = (reactionTimes >= impactStart_s) & (reactionTimes <= impactEnd_s)

    if np.any(impactMask):
        impactHubToHubWrenchMagnitude = hubToHubWrenchMagnitude[impactMask]
        impactHubToArmWrenchMagnitude = hubToArmWrenchMagnitude[impactMask]
        impactHub1ContactForce_N = hub1ContactForce_N[impactMask]
        impactReactionTimes = reactionTimes[impactMask]
    else:
        impactHubToHubWrenchMagnitude = hubToHubWrenchMagnitude
        impactHubToArmWrenchMagnitude = hubToArmWrenchMagnitude
        impactHub1ContactForce_N = hub1ContactForce_N
        impactReactionTimes = reactionTimes

    if len(armTipToHub1Distance) == len(reactionTimes):
        impactArmTipDistance = (
            armTipToHub1Distance[impactMask] if np.any(impactMask) else armTipToHub1Distance
        )
    else:
        impactArmTipDistance = np.interp(impactReactionTimes, contactTimes, armTipToHub1Distance)

    impactNonZeroMask = impactHubToArmWrenchMagnitude > 1e-9
    if np.any(impactNonZeroMask):
        nonZeroTimes = impactReactionTimes[impactNonZeroMask]
        impactDuration_s = float(nonZeroTimes[-1] - nonZeroTimes[0] + SIM_TASK_RATE_S)
        impulseHubToArm_Ns = float(np.trapezoid(impactHubToArmWrenchMagnitude[impactNonZeroMask], nonZeroTimes))
    else:
        impactDuration_s = 0.0
        impulseHubToArm_Ns = 0.0
    impulseHubToHub_Ns = float(
        np.trapezoid(impactHubToHubWrenchMagnitude, impactReactionTimes)
        if len(impactReactionTimes) > 1 else 0.0
    )

    def _fit_speed(time_window_s: np.ndarray, distance_window_m: np.ndarray) -> float:
        if len(time_window_s) < 2:
            return 0.0
        coeffs = np.polyfit(time_window_s, distance_window_m, 1)
        return float(coeffs[0])

    preMask = (contactTimes >= max(0.0, impactStart_s - 0.05)) & (contactTimes < impactStart_s)
    postMask = (contactTimes > impactEnd_s) & (contactTimes <= impactEnd_s + 0.05)
    preImpactRelativeSpeed_mps = _fit_speed(contactTimes[preMask], armTipToHub1Distance[preMask]) if np.any(preMask) else 0.0
    postImpactRelativeSpeed_mps = _fit_speed(contactTimes[postMask], armTipToHub1Distance[postMask]) if np.any(postMask) else 0.0
    restitutionEstimate = float(-postImpactRelativeSpeed_mps / preImpactRelativeSpeed_mps) if preImpactRelativeSpeed_mps not in (0.0, -0.0) else 0.0

    armJointAnglesRad = np.column_stack([np.asarray(rec.state, dtype=float).reshape(-1) for rec in armJointStateRecorders])
    armJointRatesRadPerSec = np.column_stack([np.asarray(rec.state, dtype=float).reshape(-1) for rec in armJointRateRecorders])
    armJointTorqueCommandsNm = np.column_stack([np.asarray(rec.input, dtype=float).reshape(-1) for rec in armJointTorqueRecorders])
    armJointContactLoadN = np.column_stack([
        constraintForces[:, dofIdx] if dofIdx < constraintForces.shape[1] else np.zeros(constraintForces.shape[0])
        for dofIdx in hingeStarts[:4]
    ])

    summary = {
        "scenario": "scenarioArmDockingRealistic",
        "model": {
            "spacecraft": "representative 1U CubeSat with active impedance docking arm",
            "hub_1_mass_kg": 1.13,
            "hub_1_diagonal_inertia_kg_m2": [0.0020, 0.0021, 0.0023],
            "hub_2_bus_mass_kg": 0.90,
            "hub_2_bus_diagonal_inertia_kg_m2": [0.0016, 0.0017, 0.0018],
            "arm_total_mass_kg": 0.23,
            "contact_model": (
                "MuJoCo mesh + multi-geom contact; dock_capture weld armed after tip capture; "
                "per-contact forces from mjData-synced forward pass"
            ),
            "arm_structure": "4-link articulated arm on Hub 2 with stable spring-damper control",
            "contact_solver_parameters": {"solref": [0.01, 0.5], "solimp": [0.95, 0.99, 0.001]},
            "docking_meshes": [MESH_RECEPTACLE_PATH, MESH_PROBE_PATH],
            "note": "Reaction forces are recorded for analysis but are not copied back into the Python control loop."
        },
        "environment": {
            "gravity_model": "Earth spherical harmonics (J2/degree-2) + Moon/Sun point masses via NBodyGravity",
            "spice_utc_init": SPICE_UTC_INIT,
            "spice_zero_base": "earth",
            "earth_mu_m3_s2": EARTH_MU_M3_S2,
            "moon_mu_m3_s2": MOON_MU_M3_S2,
            "sun_mu_m3_s2": SUN_MU_M3_S2,
            "earth_spherical_harmonics_degree": EARTH_SPHERICAL_HARMONICS_DEGREE,
            "drag_model": "exponentialAtmosphere + cannonballDrag on both hubs",
            "srp_model": "cannonball SRP on both hubs",
            "cubesat_projected_area_m2": CUBESAT_PROJECTED_AREA_M2,
            "cubesat_drag_coeff": CUBESAT_DRAG_COEFF,
            "cubesat_srp_reflectivity": CUBESAT_SRP_REFLECTIVITY,
            "leo_altitude_m": LEO_ALTITUDE_M,
            "reference_orbit_radius_m": LEO_REFERENCE_RADIUS_M,
            "initial_circular_speed_m_s": LEO_CIRCULAR_SPEED_M_S,
            "initial_hub_1_position_m": INITIAL_HUB1_POSITION_M.tolist(),
            "initial_hub_2_position_m": INITIAL_HUB2_POSITION_M.tolist(),
            "initial_orbit_velocity_m_s": INITIAL_ORBIT_VELOCITY_M_S.tolist(),
            "gravity_target_bodies": gravityBodyNames
        },
        "control_summary": {
            "mode": "joint_pd",
            "controller_type": "discrete spring-damper",
            "arm_joint_names": [spec[1] for spec in armJointSpecs],
            "arm_joint_dof_starts": [int(idx) for idx in hingeStarts[:4]],
            "torque_limit_Nm": armControllers[0].torqueLimitNm,
            "stiffness_nominal_Nm_per_rad": armControllers[0].nominalStiffnessNmPerRad,
            "damping_nominal_Nm_s_per_rad": armControllers[0].nominalDampingNmSPerRad,
            "note": "The controller holds a zero-angle equilibrium; XML joint damping provides passive contact compliance."
        },
        "approach": {"force_per_spacecraft_N": APPROACH_FORCE_N, "force_duration_s": APPROACH_DURATION_S},
        "maneuver_sequence": maneuverPhases + [
            {
                "name": "capture_search",
                "force_N": [0.0, 0.0],
                "duration_s": currentStop_s - maneuverStopTime_s,
                "capture_distance_m": CAPTURE_DISTANCE_M,
            },
            {
                "name": "post_capture_hold" if dockActivated else "post_hold_observation",
                "force_N": [0.0, 0.0],
                "duration_s": POST_CAPTURE_HOLD_DURATION_S,
            },
        ],
        "capture": {
            "weld_name": "dock_capture",
            "capture_distance_m": CAPTURE_DISTANCE_M,
            "weld_activated": dockActivated,
            "capture_time_s": captureTime_s,
            "final_dock_site_distance_m": float(dockSiteDistance[-1]),
            "minimum_dock_site_distance_m": float(np.min(dockSiteDistance)),
        },
        "results": {
            "minimum_arm_tip_to_hub1_distance_m": float(np.min(armTipToHub1Distance)),
            "final_arm_tip_to_hub1_distance_m": float(armTipToHub1Distance[-1]),
            "minimum_hub1_to_hub2_distance_m": float(np.min(hub1ToHub2Distance)),
            "final_hub1_to_hub2_distance_m": float(hub1ToHub2Distance[-1]),
            "maximum_hub_to_arm_wrench_N": float(np.max(hubToArmWrenchMagnitude)),
            "maximum_hub_to_hub_wrench_N": float(np.max(hubToHubWrenchMagnitude)),
            "maximum_total_hub1_contact_wrench_N": float(np.max(hub1ContactForceMagnitude)),
            "maximum_per_contact_hub_arm_force_N": float(np.max(perContactHubArm) if len(perContactHubArm) else 0.0),
            "maximum_per_contact_hub_hub_force_N": float(np.max(perContactHubHub) if len(perContactHubHub) else 0.0),
            "maximum_per_contact_total_force_N": float(np.max(perContactTotal) if len(perContactTotal) else 0.0),
            "maximum_simultaneous_contacts": int(np.max(perContactCount) if len(perContactCount) else 0),
            "mean_hub_to_arm_wrench_N": float(np.mean(hubToArmWrenchMagnitude)),
            "mean_hub_to_hub_wrench_N": float(np.mean(hubToHubWrenchMagnitude)),
            "peak_hub_to_arm_wrench_time_s": float(reactionTimes[np.argmax(hubToArmWrenchMagnitude)]),
            "impact_window_start_s": impactStart_s,
            "impact_window_end_s": impactEnd_s,
            "recording_interval_s": SIM_TASK_RATE_S,
            "maximum_abs_arm_joint_torque_command_Nm": float(np.max(np.abs(armJointTorqueCommandsNm)))
        },
        "impact_summary": {
            "impact_center_time_s": impactCenterTime_s,
            "impact_window_start_s": impactStart_s,
            "impact_window_end_s": impactEnd_s,
            "impact_duration_s": impactDuration_s,
            "peak_hub_to_arm_wrench_N": float(np.max(impactHubToArmWrenchMagnitude)),
            "peak_hub_to_hub_wrench_N": float(np.max(impactHubToHubWrenchMagnitude)),
            "peak_hub_to_arm_wrench_time_s": float(
                impactReactionTimes[np.argmax(impactHubToArmWrenchMagnitude)]
            ),
            "impulse_hub_to_arm_N_s": impulseHubToArm_Ns,
            "impulse_hub_to_hub_N_s": impulseHubToHub_Ns,
            "pre_impact_relative_speed_mps": preImpactRelativeSpeed_mps,
            "post_impact_relative_speed_mps": postImpactRelativeSpeed_mps,
            "estimated_restitution": restitutionEstimate
        },
        "time_histories": {
            "arm_tip_times_s": contactTimes.tolist(),
            "arm_tip_to_hub1_distance_m": armTipToHub1Distance.tolist(),
            "hub1_to_hub2_distance_m": hub1ToHub2Distance.tolist(),
            "dock_site_distance_m": dockSiteDistance.tolist(),
            "constraint_force_times_s": impactReactionTimes.tolist(),
            "constraint_force_hub_1_N": impactHub1ContactForce_N.tolist(),
            "hub_to_hub_wrench_magnitude_N": impactHubToHubWrenchMagnitude.tolist(),
            "hub_to_arm_wrench_magnitude_N": impactHubToArmWrenchMagnitude.tolist(),
            "per_contact_times_s": perContactTimes.tolist(),
            "per_contact_hub_arm_force_magnitude_N": perContactHubArm.tolist(),
            "per_contact_hub_hub_force_magnitude_N": perContactHubHub.tolist(),
            "per_contact_total_force_magnitude_N": perContactTotal.tolist(),
            "per_contact_count": perContactCount.tolist(),
            "per_contact_events": contactLogger.contactRecords[:5000],
            "actuator_force_magnitude_N": actuatorForceMagnitude.tolist(),
            "arm_joint_angles_rad": armJointAnglesRad.tolist(),
            "arm_joint_rates_rad_s": armJointRatesRadPerSec.tolist(),
            "arm_joint_torque_commands_Nm": armJointTorqueCommandsNm.tolist(),
            "arm_joint_contact_load_N": armJointContactLoadN.tolist()
        }
    }
    jsonPath = os.path.join(CURRENT_FOLDER, "scenarioArmDockingRealistic_results.json")
    with open(jsonPath, "w", encoding="utf-8") as jsonFile:
        json.dump(summary, jsonFile, indent=2)

    if showPlots:
        plt.figure()
        plt.plot(contactTimes[::PLOT_DECIMATION_STRIDE], armTipToHub1Distance[::PLOT_DECIMATION_STRIDE], label="Arm tip to Hub 1")
        plt.plot(contactTimes[::PLOT_DECIMATION_STRIDE], hub1ToHub2Distance[::PLOT_DECIMATION_STRIDE], label="Hub 1 body to Hub 2")
        plt.plot(contactTimes[::PLOT_DECIMATION_STRIDE], dockSiteDistance[::PLOT_DECIMATION_STRIDE], label="Dock site gap")
        if captureTime_s is not None:
            plt.axvline(captureTime_s, color="tab:green", linestyle="--", label="Capture weld")
        plt.xlabel("Time [s]")
        plt.ylabel("Site Separation [m]")
        plt.title("1U CubeSat Arm Docking Geometry")
        plt.legend()

        plt.figure()
        wrenchTimes, wrenchHubArm, wrenchHubHub = _peak_preserving_downsample(
            reactionTimes,
            hubToArmWrenchMagnitude,
            hubToHubWrenchMagnitude,
            stride=PLOT_DECIMATION_STRIDE,
        )
        plt.plot(wrenchTimes, wrenchHubArm, label="Hub 1 free-joint ↔ arm (attributed)")
        plt.plot(wrenchTimes, wrenchHubHub, label="Hub 1 free-joint ↔ Hub 2 (attributed)")
        plt.axvspan(impactStart_s, impactEnd_s, color="tab:orange", alpha=0.12, label="Impact window")
        if captureTime_s is not None:
            plt.axvline(captureTime_s, color="tab:green", linestyle="--", label="Capture weld")
        plt.xlabel("Time [s]")
        plt.ylabel("Contact Constraint Force Magnitude [N]")
        plt.title("Free-Joint Contact Wrench Attribution")
        plt.legend()

        if len(perContactTimes):
            plt.figure()
            pcTimes, pcArm, pcHub, pcTotal = _peak_preserving_downsample(
                perContactTimes,
                perContactHubArm,
                perContactHubHub,
                perContactTotal,
                stride=PLOT_DECIMATION_STRIDE,
            )
            plt.plot(pcTimes, pcArm, label="mjData Hub↔arm contacts")
            plt.plot(pcTimes, pcHub, label="mjData Hub↔hub contacts")
            plt.plot(pcTimes, pcTotal, label="mjData all contacts", linestyle="--", alpha=0.7)
            plt.axvspan(impactStart_s, impactEnd_s, color="tab:orange", alpha=0.12, label="Impact window")
            if captureTime_s is not None:
                plt.axvline(captureTime_s, color="tab:green", linestyle="--", label="Capture weld")
            plt.xlabel("Time [s]")
            plt.ylabel("Per-Contact Force Magnitude [N]")
            plt.title("mjData Per-Contact Forces")
            plt.legend()

        plt.show()

    if visualize:
        import shutil
        import tempfile

        decimatedTimes = stateRecorder.times()[::PLOT_DECIMATION_STRIDE]
        decimatedQpos = np.squeeze(stateRecorder.qpos)[::PLOT_DECIMATION_STRIDE]
        replayQpos = _make_hub1_relative_replay_qpos(decimatedQpos)
        # saveToFile keeps meshdir="meshes"; stage the OBJ assets next to the
        # temporary model.xml so the replay tool can open them.
        with tempfile.TemporaryDirectory() as replayDir:
            modelFile = os.path.join(replayDir, "model.xml")
            scene.saveToFile(modelFile)
            meshDir = os.path.join(replayDir, "meshes")
            os.makedirs(meshDir, exist_ok=True)
            shutil.copy(MESH_RECEPTACLE_PATH, os.path.join(meshDir, "docking_receptacle.obj"))
            shutil.copy(MESH_PROBE_PATH, os.path.join(meshDir, "docking_probe.obj"))
            mujoco.visualize(
                decimatedTimes,
                replayQpos,
                modelFile,
                0.25,
                track="hub_1",
            )


if __name__ == "__main__":
    run(True, True)