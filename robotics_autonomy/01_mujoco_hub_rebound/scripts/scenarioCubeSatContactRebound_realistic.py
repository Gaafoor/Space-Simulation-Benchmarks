#
# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
"""Realistic contact-only MuJoCo simulation of two representative 1U CubeSat buses.

This variant keeps the contact experiment from the simpler rebound example,
but each spacecraft now includes a Basilisk-style reaction wheel and four tank
sub-bodies so the model structure is closer to the reaction-wheel and
unbalanced-thruster examples. The internal actuators are kept at zero command
for now so the contact response stays isolated.
"""

import json
import os
from typing import Sequence

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import MJJointReactionForces, mujoco, svIntegrators
from Basilisk.utilities import SimulationBaseClass, macros

CURRENT_FOLDER = os.path.dirname(__file__)
XML_PATH = os.path.join(CURRENT_FOLDER, "sats_cubesat_contact_realistic.xml")

# Tunable maneuver schedule.
INITIAL_HUB1_POSITION_M = np.array([-0.4, 0.0, 0.0])
INITIAL_HUB2_POSITION_M = np.array([0.4, 0.0, 0.0])
APPROACH_FORCE_N = 1.0
BRAKE_FORCE_N = -2.0
APPROACH_DURATION_S = 0.72
BRAKE_DURATION_S = 0.18
ZERO_THRUST_HOLD_DURATION_S = 0.50
POST_HOLD_OBSERVATION_DURATION_S = 2.60


def setThrusterForce(msgs: Sequence[messaging.SingleActuatorMsg], thrust: Sequence[float]):
    """Write the requested force command to each actuator message."""
    for msg, value in zip(msgs, thrust):
        msg.write(messaging.SingleActuatorMsgPayload(input=value))


def _find_free_joint_starts(joint_types: Sequence[int], joint_dof_start: Sequence[int]) -> tuple[int, int]:
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


def run(showPlots: bool = False, visualize: bool = False):
    """Run the contact-only CubeSat approach, impact, and rebound case."""
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(0.001)))

    scene = mujoco.MJScene.fromFile(XML_PATH)
    scene.extraEoMCall = True
    scSim.AddModelToTask("test", scene)

    integrator = svIntegrators.svIntegratorRKF45(scene)
    scene.setIntegrator(integrator)

    hub1ThrusterMsg = messaging.SingleActuatorMsg()
    hub2ThrusterMsg = messaging.SingleActuatorMsg()
    scene.getSingleActuator("hub_1_thruster").actuatorInMsg.subscribeTo(hub1ThrusterMsg)
    scene.getSingleActuator("hub_2_thruster").actuatorInMsg.subscribeTo(hub2ThrusterMsg)

    # Keep the internal wheels and tanks defined by the more realistic XML at
    # zero command for now. This makes the model richer without changing the
    # contact experiment into an active attitude-control case.
    internalActuatorMessages = {}
    internalActuatorNames = [
        "hub_1_wheel_motor",
        "hub_2_wheel_motor",
        "hub_1_tank_1_thrust",
        "hub_1_tank_2_thrust",
        "hub_1_tank_3_thrust",
        "hub_1_tank_4_thrust",
        "hub_2_tank_1_thrust",
        "hub_2_tank_2_thrust",
        "hub_2_tank_3_thrust",
        "hub_2_tank_4_thrust",
    ]
    for actuatorName in internalActuatorNames:
        actuatorMsg = messaging.SingleActuatorMsg()
        actuatorMsg.write(messaging.SingleActuatorMsgPayload(input=0.0))
        scene.getSingleActuator(actuatorName).actuatorInMsg.subscribeTo(actuatorMsg)
        internalActuatorMessages[actuatorName] = actuatorMsg

    jointReactionForces = MJJointReactionForces.MJJointReactionForces()
    jointReactionForces.ModelTag = "MJJointReactionForces"
    jointReactionForces.scene = scene
    scSim.AddModelToTask("test", jointReactionForces)

    hub1ContactRecorder = scene.getBody("hub_1").getSite("hub_1_contact_point").stateOutMsg.recorder()
    hub2ContactRecorder = scene.getBody("hub_2").getSite("hub_2_contact_point").stateOutMsg.recorder()
    reactionForcesRecorder = jointReactionForces.reactionForcesOutMsg.recorder()
    stateRecorder = scene.stateOutMsg.recorder()
    for recorder in [hub1ContactRecorder, hub2ContactRecorder, reactionForcesRecorder, stateRecorder]:
        scSim.AddModelToTask("test", recorder)

    scSim.InitializeSimulation()

    scene.getBody("hub_1").setPosition(INITIAL_HUB1_POSITION_M.tolist())
    scene.getBody("hub_2").setPosition(INITIAL_HUB2_POSITION_M.tolist())

    thrustMessages = [hub1ThrusterMsg, hub2ThrusterMsg]
    maneuverPhases = [
        {"name": "approach", "force_N": [APPROACH_FORCE_N, APPROACH_FORCE_N], "duration_s": APPROACH_DURATION_S},
        {"name": "brake", "force_N": [BRAKE_FORCE_N, BRAKE_FORCE_N], "duration_s": BRAKE_DURATION_S},
        {"name": "zero_thrust_hold", "force_N": [0.0, 0.0], "duration_s": ZERO_THRUST_HOLD_DURATION_S},
    ]
    maneuverStopTime_s = 0.0
    for phase in maneuverPhases:
        setThrusterForce(thrustMessages, phase["force_N"])
        maneuverStopTime_s += phase["duration_s"]
        scSim.ConfigureStopTime(macros.sec2nano(maneuverStopTime_s))
        scSim.ExecuteSimulation()

    setThrusterForce(thrustMessages, [0.0, 0.0])
    scSim.ConfigureStopTime(macros.sec2nano(maneuverStopTime_s + POST_HOLD_OBSERVATION_DURATION_S))
    scSim.ExecuteSimulation()

    rContact1_N = np.squeeze(hub1ContactRecorder.r_BN_N)
    rContact2_N = np.squeeze(hub2ContactRecorder.r_BN_N)
    distance = np.linalg.norm(rContact1_N - rContact2_N, axis=1)
    constraintForces = np.asarray(reactionForcesRecorder.constraintForces, dtype=float)
    jointTypes = np.asarray(reactionForcesRecorder.jointTypes)
    jointDOFStart = np.asarray(reactionForcesRecorder.jointDOFStart)
    hub1Start, hub2Start = _find_free_joint_starts(jointTypes, jointDOFStart)
    hub1ContactWrench = constraintForces[:, hub1Start:hub1Start + 6]
    hub2ContactWrench = constraintForces[:, hub2Start:hub2Start + 6]
    hub1ContactForce_N = hub1ContactWrench[:, 0:3]
    hub2ContactForce_N = hub2ContactWrench[:, 0:3]
    hub1ContactForceMagnitude = np.linalg.norm(hub1ContactForce_N, axis=1)
    hub2ContactForceMagnitude = np.linalg.norm(hub2ContactForce_N, axis=1)
    actuatorForces = np.asarray(reactionForcesRecorder.actuatorForces)
    actuatorForceMagnitude = np.linalg.norm(actuatorForces, axis=1)
    contactTimes = hub1ContactRecorder.times() * macros.NANO2SEC
    reactionTimes = reactionForcesRecorder.times() * macros.NANO2SEC
    impactCenterIdx = int(np.argmin(distance))
    impactCenterTime_s = float(contactTimes[impactCenterIdx])
    impactStart_s = float(max(0.0, impactCenterTime_s - 0.05))
    impactEnd_s = float(impactCenterTime_s + 0.05)
    impactMask = (reactionTimes >= impactStart_s) & (reactionTimes <= impactEnd_s)

    if np.any(impactMask):
        impactHub1ContactForceMagnitude = hub1ContactForceMagnitude[impactMask]
        impactHub2ContactForceMagnitude = hub2ContactForceMagnitude[impactMask]
        impactHub1ContactForce_N = hub1ContactForce_N[impactMask]
        impactHub2ContactForce_N = hub2ContactForce_N[impactMask]
        impactReactionTimes = reactionTimes[impactMask]
    else:
        impactHub1ContactForceMagnitude = hub1ContactForceMagnitude
        impactHub2ContactForceMagnitude = hub2ContactForceMagnitude
        impactHub1ContactForce_N = hub1ContactForce_N
        impactHub2ContactForce_N = hub2ContactForce_N
        impactReactionTimes = reactionTimes

    impactNonZeroMask = impactHub1ContactForceMagnitude > 1e-9
    if np.any(impactNonZeroMask):
        nonZeroTimes = impactReactionTimes[impactNonZeroMask]
        impactDuration_s = float(nonZeroTimes[-1] - nonZeroTimes[0] + 0.001)
        impulseHub1_Ns = float(np.trapezoid(impactHub1ContactForceMagnitude[impactNonZeroMask], nonZeroTimes))
        impulseHub2_Ns = float(np.trapezoid(impactHub2ContactForceMagnitude[impactNonZeroMask], nonZeroTimes))
    else:
        impactDuration_s = 0.0
        impulseHub1_Ns = 0.0
        impulseHub2_Ns = 0.0

    def _fit_speed(time_window_s: np.ndarray, distance_window_m: np.ndarray) -> float:
        if len(time_window_s) < 2:
            return 0.0
        coeffs = np.polyfit(time_window_s, distance_window_m, 1)
        return float(coeffs[0])

    preMask = (contactTimes >= max(0.0, impactStart_s - 0.05)) & (contactTimes < impactStart_s)
    postMask = (contactTimes > impactEnd_s) & (contactTimes <= impactEnd_s + 0.05)
    preImpactRelativeSpeed_mps = _fit_speed(contactTimes[preMask], distance[preMask]) if np.any(preMask) else 0.0
    postImpactRelativeSpeed_mps = _fit_speed(contactTimes[postMask], distance[postMask]) if np.any(postMask) else 0.0
    restitutionEstimate = float(-postImpactRelativeSpeed_mps / preImpactRelativeSpeed_mps) if preImpactRelativeSpeed_mps not in (0.0, -0.0) else 0.0

    summary = {
        "scenario": "scenarioCubeSatContactReboundRealistic",
        "model": {
            "spacecraft": "representative 1U CubeSat bus with wheel and tank sub-bodies",
            "mass_per_spacecraft_kg": 1.13,
            "inertia_per_spacecraft_kg_m2": [0.0020, 0.0021, 0.0023],
            "contact_model": "MuJoCo chassis geom contact only; no equality/weld",
            "internal_subsystems": ["reaction_wheel", "4_tank_bodies"],
            "contact_solver_parameters": {"solref": [0.01, 0.1], "solimp": [0.99, 0.99, 0.001]},
            "note": "Internal subsystems are present but held at zero command; contact tuning remains illustrative."
        },
        "approach": {"force_per_spacecraft_N": APPROACH_FORCE_N, "force_duration_s": APPROACH_DURATION_S},
        "maneuver_sequence": maneuverPhases + [{"name": "post_hold_observation", "force_N": [0.0, 0.0], "duration_s": POST_HOLD_OBSERVATION_DURATION_S}],
        "results": {
            "minimum_contact_site_distance_m": float(np.min(distance)),
            "final_contact_site_distance_m": float(distance[-1]),
            "maximum_contact_force_hub_1_N": float(np.max(hub1ContactForceMagnitude)),
            "maximum_contact_force_hub_2_N": float(np.max(hub2ContactForceMagnitude)),
            "mean_contact_force_hub_1_N": float(np.mean(hub1ContactForceMagnitude)),
            "mean_contact_force_hub_2_N": float(np.mean(hub2ContactForceMagnitude)),
            "peak_contact_force_time_s": float(reactionTimes[np.argmax(hub1ContactForceMagnitude)]),
            "impact_window_start_s": impactStart_s,
            "impact_window_end_s": impactEnd_s,
            "recording_interval_s": 0.001
        },
        "impact_summary": {
            "impact_center_time_s": impactCenterTime_s,
            "impact_window_start_s": impactStart_s,
            "impact_window_end_s": impactEnd_s,
            "impact_duration_s": impactDuration_s,
            "peak_contact_force_hub_1_N": float(np.max(impactHub1ContactForceMagnitude)),
            "peak_contact_force_hub_2_N": float(np.max(impactHub2ContactForceMagnitude)),
            "peak_contact_force_time_s": float(impactReactionTimes[np.argmax(impactHub1ContactForceMagnitude)]),
            "impulse_hub_1_N_s": impulseHub1_Ns,
            "impulse_hub_2_N_s": impulseHub2_Ns,
            "pre_impact_relative_speed_mps": preImpactRelativeSpeed_mps,
            "post_impact_relative_speed_mps": postImpactRelativeSpeed_mps,
            "estimated_restitution": restitutionEstimate
        },
        "time_histories": {
            "contact_site_times_s": contactTimes.tolist(),
            "contact_site_distance_m": distance.tolist(),
            "constraint_force_times_s": impactReactionTimes.tolist(),
            "constraint_force_hub_1_N": impactHub1ContactForce_N.tolist(),
            "constraint_force_hub_2_N": impactHub2ContactForce_N.tolist(),
            "constraint_force_hub_1_magnitude_N": impactHub1ContactForceMagnitude.tolist(),
            "constraint_force_hub_2_magnitude_N": impactHub2ContactForceMagnitude.tolist(),
            "actuator_force_magnitude_N": actuatorForceMagnitude.tolist()
        }
    }
    jsonPath = os.path.join(CURRENT_FOLDER, "scenarioCubeSatContactReboundRealistic_results.json")
    with open(jsonPath, "w", encoding="utf-8") as jsonFile:
        json.dump(summary, jsonFile, indent=2)

    if showPlots:
        plt.figure()
        plt.plot(contactTimes, distance)
        plt.xlabel("Time [s]")
        plt.ylabel("Contact Site Distance [m]")
        plt.title("1U CubeSat Contact Site Separation")

        plt.figure()
        plt.plot(reactionTimes, hub1ContactForceMagnitude, label="Hub 1 contact force")
        plt.plot(reactionTimes, hub2ContactForceMagnitude, label="Hub 2 contact force")
        plt.axvspan(impactStart_s, impactEnd_s, color="tab:orange", alpha=0.12, label="Impact window")
        plt.xlabel("Time [s]")
        plt.ylabel("Contact Constraint Force Magnitude [N]")
        plt.title("1U CubeSat Contact Force (No Weld)")
        plt.legend()
        plt.show()

    if visualize:
        mujoco.visualize(stateRecorder.times(), np.squeeze(stateRecorder.qpos), scene, 0.25)


if __name__ == "__main__":
    run(True, True)
