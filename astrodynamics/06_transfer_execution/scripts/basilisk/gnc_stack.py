"""Velocity-pointing GNC helpers for GMAT handoff execution."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (
    attTrackingError,
    dipoleMapping,
    mtbFeedforward,
    mtbMomentumManagementSimple,
    mrpFeedback,
    mrpRotation,
    rwMotorTorque,
    tamComm,
    torque2Dipole,
    velocityPoint,
)
from Basilisk.simulation import MtbEffector, magnetometer, magneticFieldWMM, reactionWheelStateEffector, simpleNav
from Basilisk.utilities import RigidBodyKinematics, macros, simIncludeRW, unitTestSupport
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

# velocityPoint aligns +Y||V; z-axis quarter turn maps thrust to body +X (42 / GMAT convention).
THRUST_AXIS_ROTATION_MRP = [0.0, 0.0, float(np.tan(np.pi / 8.0))]

# Match NASA 42 A3 RW/MTB stack (SC_A3_RW.txt).
RW_MAX_TORQUE_NM = 0.015
RW_MAX_MOMENTUM_NMS = 0.150
MTB_MAX_DIPOLE_A_M2 = 5.0
HANDOFF_INERTIA_KG_M2 = (0.15, 0.18, 0.12)
# Orthogonal triad + body diagonal (Basilisk B2 pyramid); over-actuated RW null-space.
RW_PYRAMID_AXES = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 1.0, 1.0),
)

# 42 RW PD (42fsw.c: 0.035 Hz / 0.95 zeta) raised for velocity-frame tracking under GG.
RW_CONTROL_BANDWIDTH_HZ = 0.15
RW_CONTROL_DAMPING = 0.95
CHARACTERISTIC_INERTIA_KG_M2 = sum(HANDOFF_INERTIA_KG_M2) / len(HANDOFF_INERTIA_KG_M2)
# 42 MTB unload Kunl=60; simple-module Kp tuned for visible LEO coast unloading.
MTB_UNLOAD_KP = 0.05


@dataclass(frozen=True)
class NavNoiseConfig:
    enabled: bool = True
    attitude_noise_sigma_deg: float = 0.001
    rate_noise_sigma_deg_s: float = 0.0005
    attitude_error_3sigma_deg: float = 0.01
    rate_error_3sigma_deg_s: float = 0.005


def scaled_mrp_gains(*, characteristic_inertia_kg_m2: float = CHARACTERISTIC_INERTIA_KG_M2) -> tuple[float, float]:
    wn = 2.0 * math.pi * RW_CONTROL_BANDWIDTH_HZ
    k_gain = characteristic_inertia_kg_m2 * wn * wn
    p_gain = 2.0 * RW_CONTROL_DAMPING * wn * characteristic_inertia_kg_m2
    return k_gain, p_gain


def configure_noisy_simple_nav(s_nav: simpleNav.SimpleNav, noise: NavNoiseConfig) -> None:
    """Enable Gauss-Markov attitude and rate errors on SimpleNav."""
    if not noise.enabled:
        return
    state_count = 18
    p_diag = np.zeros(state_count, dtype=float)
    walk_bounds = np.zeros(state_count, dtype=float)
    att_sigma = math.radians(float(noise.attitude_noise_sigma_deg))
    rate_sigma = math.radians(float(noise.rate_noise_sigma_deg_s))
    att_bound = math.radians(float(noise.attitude_error_3sigma_deg))
    rate_bound = math.radians(float(noise.rate_error_3sigma_deg_s))

    for index in range(6, 9):
        p_diag[index] = att_sigma
        walk_bounds[index] = max(att_bound, 3.0 * att_sigma)
    for index in range(9, 12):
        p_diag[index] = rate_sigma
        walk_bounds[index] = max(rate_bound, 3.0 * rate_sigma)

    s_nav.PMatrix = np.diag(p_diag)
    s_nav.walkBounds = walk_bounds


def compute_gnc_initial_sigma(r_m: np.ndarray, v_m_s: np.ndarray, mu_earth: float) -> np.ndarray:
    """Initial body attitude with body +X along velocity (matches RW control target)."""
    return compute_thrust_pointing_sigma(r_m, v_m_s, mu_earth)


def compute_thrust_pointing_sigma(r_m: np.ndarray, v_m_s: np.ndarray, mu_earth: float) -> np.ndarray:
    """Reference attitude for GMAT VNB +X prograde electric thrust (body +X along velocity)."""
    del mu_earth
    v_hat = np.asarray(v_m_s, dtype=float).reshape(3)
    speed = float(np.linalg.norm(v_hat))
    if speed <= 0.0:
        return np.zeros(3, dtype=float)
    v_hat /= speed
    h = np.cross(np.asarray(r_m, dtype=float).reshape(3), np.asarray(v_m_s, dtype=float).reshape(3))
    h_norm = float(np.linalg.norm(h))
    if h_norm <= 1e-9:
        return np.zeros(3, dtype=float)
    z_hat = h / h_norm
    x_hat = v_hat
    y_hat = np.cross(z_hat, x_hat)
    y_norm = float(np.linalg.norm(y_hat))
    if y_norm <= 1e-9:
        return np.zeros(3, dtype=float)
    y_hat /= y_norm
    dcm_bn = np.vstack([x_hat, y_hat, z_hat])
    return np.array(RigidBodyKinematics.C2MRP(dcm_bn), dtype=float).reshape(3)


def pointing_series_from_state_rec(state_rec) -> list[tuple[float, float]]:
    """Build thrust-axis pointing error samples from a spacecraft state recorder."""
    log_times = np.asarray(state_rec.times() * 1e-9, dtype=float)
    if log_times.size == 0:
        return []
    sigmas = np.asarray(state_rec.sigma_BN, dtype=float)
    vels = np.asarray(state_rec.v_BN_N, dtype=float)
    if sigmas.ndim == 1:
        sigmas = sigmas.reshape(-1, 3)
    if vels.ndim == 1:
        vels = vels.reshape(-1, 3)
    return [
        (float(t), thrust_axis_pointing_error_deg(sigmas[idx], vels[idx]))
        for idx, t in enumerate(log_times)
    ]


def thrust_axis_pointing_error_deg(sigma_bn: np.ndarray, v_m_s: np.ndarray) -> float:
    """Angle between body +X and the instantaneous velocity vector."""
    v_hat = np.asarray(v_m_s, dtype=float)
    speed = float(np.linalg.norm(v_hat))
    if speed <= 0.0:
        return float("nan")
    v_hat /= speed
    dcm_bn = RigidBodyKinematics.MRP2C(np.asarray(sigma_bn, dtype=float).reshape(3))
    thrust_hat = dcm_bn.T @ np.array([1.0, 0.0, 0.0], dtype=float)
    thrust_hat /= max(float(np.linalg.norm(thrust_hat)), 1e-12)
    return math.degrees(math.acos(float(np.clip(np.dot(thrust_hat, v_hat), -1.0, 1.0))))


def sync_spacecraft_attitude(sc_object, sigma_bn: np.ndarray, *, omega_bn_b: np.ndarray | None = None) -> None:
    sigma = np.asarray(sigma_bn, dtype=float).reshape(3)
    omega = np.zeros(3) if omega_bn_b is None else np.asarray(omega_bn_b, dtype=float).reshape(3)
    att_ref = sc_object.dynManager.getStateObject(sc_object.hub.nameOfHubSigma)
    rate_ref = sc_object.dynManager.getStateObject(sc_object.hub.nameOfHubOmega)
    att_ref.setState(unitTestSupport.np2EigenVectorXd(sigma))
    rate_ref.setState(unitTestSupport.np2EigenVectorXd(omega))


def attach_velocity_pointing_gnc(
    sc_sim,
    task_name: str,
    sc_object,
    *,
    mu_earth: float,
    sample_step_s: float = 60.0,
    epoch_msg=None,
    enable_mtb: bool = False,
    nav_noise: NavNoiseConfig | None = None,
    nav_mode: str = "simpleNav",
    hardware_nav: dict | None = None,
) -> dict:
    """Attach realistic RW closed-loop GNC for velocity / +X thrust pointing."""
    rw_factory = simIncludeRW.rwFactory()
    for idx, axis in enumerate(RW_PYRAMID_AXES, start=1):
        axis_hat = (np.asarray(axis, dtype=float) / np.linalg.norm(axis)).tolist()
        rw = rw_factory.create("BCT_RWP015", axis_hat, useRWfriction=False, useMaxTorque=True, label=f"RW{idx}")
        rw.u_max = RW_MAX_TORQUE_NM

    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "HandoffRWCluster"
    rw_factory.addToSpacecraft(sc_object.ModelTag, rw_effector, sc_object)
    rw_config_msg = rw_factory.getConfigMessage()

    s_nav = simpleNav.SimpleNav()
    s_nav.ModelTag = "HandoffSimpleNav"
    s_nav.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    nav_noise_cfg = nav_noise or NavNoiseConfig()
    if nav_mode == "simpleNav":
        configure_noisy_simple_nav(s_nav, nav_noise_cfg)

    att_nav_msg = s_nav.attOutMsg
    trans_nav_msg = s_nav.transOutMsg
    if nav_mode == "hardware" and hardware_nav is not None:
        att_nav_msg = hardware_nav["nav_att_msg"]
        trans_nav_msg = hardware_nav["trans_nav"].transOutMsg

    att_guidance = velocityPoint.velocityPoint()
    att_guidance.ModelTag = "HandoffVelocityPoint"
    att_guidance.transNavInMsg.subscribeTo(trans_nav_msg)
    att_guidance.mu = float(mu_earth)

    att_ref_rot = mrpRotation.mrpRotation()
    att_ref_rot.ModelTag = "HandoffThrustAxisRotation"
    att_ref_rot.mrpSet = THRUST_AXIS_ROTATION_MRP
    att_ref_rot.attRefInMsg.subscribeTo(att_guidance.attRefOutMsg)

    att_error = attTrackingError.attTrackingError()
    att_error.ModelTag = "HandoffAttError"
    att_error.sigma_R0R = [0.0, 0.0, 0.0]
    att_error.attRefInMsg.subscribeTo(att_ref_rot.attRefOutMsg)
    att_error.attNavInMsg.subscribeTo(att_nav_msg)

    mrp_k, mrp_p = scaled_mrp_gains()
    mrp_control = mrpFeedback.mrpFeedback()
    mrp_control.ModelTag = "HandoffMrpFeedback"
    mrp_control.guidInMsg.subscribeTo(att_error.attGuidOutMsg)
    mrp_control.K = mrp_k
    mrp_control.P = mrp_p
    mrp_control.Ki = -1.0

    ixx, iyy, izz = HANDOFF_INERTIA_KG_M2
    veh_config = messaging.VehicleConfigMsgPayload()
    veh_config.ISCPntB_B = [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]
    veh_config_msg = messaging.VehicleConfigMsg().write(veh_config)
    mrp_control.vehConfigInMsg.subscribeTo(veh_config_msg)
    mrp_control.rwParamsInMsg.subscribeTo(rw_config_msg)
    mrp_control.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)

    rw_torque = rwMotorTorque.rwMotorTorque()
    rw_torque.ModelTag = "HandoffRwMotorTorque"
    rw_torque.vehControlInMsg.subscribeTo(mrp_control.cmdTorqueOutMsg)
    rw_torque.rwParamsInMsg.subscribeTo(rw_config_msg)
    rw_torque.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rw_effector.rwMotorCmdInMsg.subscribeTo(rw_torque.rwMotorTorqueOutMsg)

    sample_ns = macros.sec2nano(sample_step_s)
    att_err_log = att_error.attGuidOutMsg.recorder(sample_ns)
    att_ref_log = att_ref_rot.attRefOutMsg.recorder(sample_ns)
    rw_speed_log = rw_effector.rwSpeedOutMsg.recorder(sample_ns)
    cmd_torque_log = mrp_control.cmdTorqueOutMsg.recorder(sample_ns)
    rw_out_logs = [msg.recorder(sample_ns) for msg in rw_effector.rwOutMsgs]
    rw_torque_log = rw_torque.rwMotorTorqueOutMsg.recorder(sample_ns)

    mtb_cmd_log = None
    use_mtb = enable_mtb and epoch_msg is not None
    mrp_priority = 8
    rw_torque_priority = 6 if use_mtb else 7
    if use_mtb:
        mag_module = magneticFieldWMM.MagneticFieldWMM()
        mag_module.ModelTag = "HandoffMagField"
        mag_module.configureWMMFile(str(get_path(DataFile.MagneticFieldData.WMM)))
        mag_module.epochInMsg.subscribeTo(epoch_msg)
        mag_module.addSpacecraftToModel(sc_object.scStateOutMsg)

        mtb_eff = MtbEffector.MtbEffector()
        mtb_eff.ModelTag = "HandoffMtbEff"
        sc_object.addDynamicEffector(mtb_eff)

        tam = magnetometer.Magnetometer()
        tam.ModelTag = "HandoffTam"
        tam.scaleFactor = 1.0
        tam.senNoiseStd = [0.0, 0.0, 0.0]

        tam_comm = tamComm.tamComm()
        tam_comm.ModelTag = "HandoffTamComm"
        tam_comm.dcm_BS = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        mtb_config = messaging.MTBArrayConfigMsgPayload()
        mtb_config.numMTB = 3
        mtb_config.GtMatrix_B = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        mtb_config.maxMtbDipoles = [MTB_MAX_DIPOLE_A_M2] * 3
        mtb_params_msg = messaging.MTBArrayConfigMsg().write(mtb_config)

        mtb_unload = mtbMomentumManagementSimple.mtbMomentumManagementSimple()
        mtb_unload.ModelTag = "HandoffMtbUnload"
        mtb_unload.Kp = MTB_UNLOAD_KP

        tau_to_dipole = torque2Dipole.torque2Dipole()
        tau_to_dipole.ModelTag = "HandoffTorque2Dipole"

        dipole_map = dipoleMapping.dipoleMapping()
        dipole_map.ModelTag = "HandoffDipoleMap"
        dipole_map.steeringMatrix = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        tam.stateInMsg.subscribeTo(sc_object.scStateOutMsg)
        tam.magInMsg.subscribeTo(mag_module.envOutMsgs[0])
        tam_comm.tamInMsg.subscribeTo(tam.tamDataOutMsg)
        mtb_unload.rwParamsInMsg.subscribeTo(rw_config_msg)
        mtb_unload.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)
        tau_to_dipole.tauRequestInMsg.subscribeTo(mtb_unload.tauMtbRequestOutMsg)
        tau_to_dipole.tamSensorBodyInMsg.subscribeTo(tam_comm.tamOutMsg)
        dipole_map.dipoleRequestBodyInMsg.subscribeTo(tau_to_dipole.dipoleRequestOutMsg)
        dipole_map.mtbArrayConfigParamsInMsg.subscribeTo(mtb_params_msg)
        mtb_eff.mtbCmdInMsg.subscribeTo(dipole_map.dipoleRequestMtbOutMsg)
        mtb_eff.mtbParamsInMsg.subscribeTo(mtb_params_msg)
        mtb_eff.magInMsg.subscribeTo(mag_module.envOutMsgs[0])

        mtb_ff = mtbFeedforward.mtbFeedforward()
        mtb_ff.ModelTag = "HandoffMtbFeedforward"
        mtb_ff.vehControlInMsg.subscribeTo(mrp_control.cmdTorqueOutMsg)
        mtb_ff.dipoleRequestMtbInMsg.subscribeTo(dipole_map.dipoleRequestMtbOutMsg)
        mtb_ff.tamSensorBodyInMsg.subscribeTo(tam_comm.tamOutMsg)
        mtb_ff.mtbArrayConfigParamsInMsg.subscribeTo(mtb_params_msg)
        rw_torque.vehControlInMsg.subscribeTo(mtb_ff.vehControlOutMsg)

        mtb_cmd_log = dipole_map.dipoleRequestMtbOutMsg.recorder(sample_ns)

        sc_sim.AddModelToTask(task_name, mag_module, 200)
        sc_sim.AddModelToTask(task_name, mtb_eff, 199)
        sc_sim.AddModelToTask(task_name, tam, 108)
        sc_sim.AddModelToTask(task_name, tam_comm, 107)
        sc_sim.AddModelToTask(task_name, mtb_unload, 11)
        sc_sim.AddModelToTask(task_name, tau_to_dipole, 10)
        sc_sim.AddModelToTask(task_name, dipole_map, 9)
        sc_sim.AddModelToTask(task_name, mtb_ff, 7)

    sc_sim.AddModelToTask(task_name, rw_effector, 301)
    if nav_mode == "simpleNav" or hardware_nav is None:
        sc_sim.AddModelToTask(task_name, s_nav, 109)
    sc_sim.AddModelToTask(task_name, att_guidance, 10)
    sc_sim.AddModelToTask(task_name, att_ref_rot, 9)
    sc_sim.AddModelToTask(task_name, att_error, 8)
    sc_sim.AddModelToTask(task_name, mrp_control, mrp_priority)
    sc_sim.AddModelToTask(task_name, rw_torque, rw_torque_priority)
    for model in (att_err_log, att_ref_log, rw_speed_log, cmd_torque_log, rw_torque_log, *rw_out_logs):
        sc_sim.AddModelToTask(task_name, model)
    if mtb_cmd_log is not None:
        sc_sim.AddModelToTask(task_name, mtb_cmd_log)

    out = {
        "att_error_log": att_err_log,
        "att_ref_log": att_ref_log,
        "rw_speed_log": rw_speed_log,
        "cmd_torque_log": cmd_torque_log,
        "rw_out_logs": rw_out_logs,
        "rw_torque_log": rw_torque_log,
        "rw_factory": rw_factory,
        "rw_config_msg": rw_config_msg,
        "veh_config_msg": veh_config_msg,
        "nav_noise_config": nav_noise_cfg,
        "nav_mode": nav_mode,
        "hardware_nav": hardware_nav,
        "mu_earth": float(mu_earth),
        "mrp_k": mrp_k,
        "mrp_p": mrp_p,
        "mtb_enabled": use_mtb,
    }
    if mtb_cmd_log is not None:
        out["mtb_cmd_log"] = mtb_cmd_log
        out["mtb_params_msg"] = mtb_params_msg
    return out
