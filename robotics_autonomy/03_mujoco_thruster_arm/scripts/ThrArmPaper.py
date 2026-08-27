#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Schwend & Schaub (2026) thruster-arm inspection scenario.
#  Deputy dynamics in MuJoCo; chief as a passive Basilisk spacecraft with simpleNav.
#  LOS pointing via locationPointing; time-varying deputy CoM reference for inspection.

import argparse
import inspect
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import mujoco, spacecraft, simpleNav
from Basilisk.utilities import macros as mc, unitTestSupport, RigidBodyKinematics as rbk

try:
  import mujoco as mj
  from mujoco import viewer as mj_viewer
  _MJ_VIEWER_OK = True
except ImportError:
  _MJ_VIEWER_OK = False

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
fileNameForImages = os.path.basename(os.path.splitext(__file__)[0])

sys.path.append(path + "/mujocoModels")
import BSK_MujocoDynamics_Paper as BSK_MujocoDynamics
import BSK_MujocoFSW_Paper
from BSK_mujocoMasters import BSKSim, BSKScenario

XML_PATH = BSK_MujocoDynamics.XML_PATH

# Paper deputy initial relative offset at t=0 (chief at origin)
RHO0_N = np.array([0.5, 19.65, -0.25])

PHASE_C_JOINT_NAMES = [
    "arm_1_joint_1",
    "arm_1_joint_2",
    "arm_1_joint_3",
    "arm_1_joint_4",
    "arm_2_joint_1",
    "arm_2_joint_2",
    "arm_2_joint_3",
    "arm_2_joint_4",
]


class CmdNconLogger(sysModel.SysModel):
  """Phase C: for each new desJointAngles write, evaluate MuJoCo ncon at that pose."""

  def __init__(self, des_angle_reader, xml_path=XML_PATH):
    super().__init__()
    self.desAngleReader = des_angle_reader
    self.lastWriteTime = -1
    self.records = []  # list of dicts
    self.n_cmd = 0
    self.n_collide = 0
    self._mj = None
    self._model = None
    self._data = None
    self._qadr = None
    if not _MJ_VIEWER_OK:
      print("[PhaseC] mujoco python package unavailable — CmdNconLogger disabled", flush=True)
      return
    self._mj = mj
    self._model = mj.MjModel.from_xml_path(xml_path)
    self._data = mj.MjData(self._model)
    self._qadr = []
    for name in PHASE_C_JOINT_NAMES:
      jid = mj.mj_name2id(self._model, mj.mjtObj.mjOBJ_JOINT, name)
      self._qadr.append(int(self._model.jnt_qposadr[jid]))

  def _ncon_at(self, angles):
    self._data.qpos[:] = 0.0
    self._data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i, a in enumerate(angles):
      self._data.qpos[self._qadr[i]] = float(a)
    self._mj.mj_forward(self._model, self._data)
    return int(self._data.ncon)

  def UpdateState(self, CurrentSimNanos):
    if self._model is None:
      return
    t_write = self.desAngleReader.timeWritten()
    if t_write == self.lastWriteTime or t_write < 0:
      return
    self.lastWriteTime = t_write
    raw = list(self.desAngleReader().states)
    if len(raw) < 8:
      return
    angles = np.asarray(raw[:8], dtype=float)
    ncon = self._ncon_at(angles)
    self.n_cmd += 1
    if ncon > 0:
      self.n_collide += 1
    rec = {
        "t": CurrentSimNanos * mc.NANO2SEC,
        "ncon": ncon,
        "theta": angles.copy(),
    }
    self.records.append(rec)
    tag = "CONTACT" if ncon > 0 else "clear"
    print(
        f"[PhaseC] cmd θ t={rec['t']:.2f}s ncon={ncon} [{tag}] θ="
        + np.array2string(angles, precision=3, separator=","),
        flush=True,
    )

  def report(self):
    print("\n========== Phase C in-sim ncon vs commanded θ ==========", flush=True)
    print(f"  geometry={BSK_MujocoFSW_Paper.GEOMETRY_MODE}", flush=True)
    print(f"  allocation commands checked: {self.n_cmd}", flush=True)
    print(f"  commands with ncon>0: {self.n_collide}", flush=True)
    ok = self.n_cmd > 0 and self.n_collide == 0
    if self.n_cmd == 0:
      print("  [FAIL] no allocation commands observed", flush=True)
      ok = False
    else:
      print(
          f"  [{'PASS' if ok else 'FAIL'}] allocator never commanded a colliding pose",
          flush=True,
      )
    print(
        f"========== Phase C in-sim: {'PASS' if ok else 'FAIL'} ==========\n",
        flush=True,
    )
    return ok


# One full inspection arc over an 8 h mission (not the paper's unpublished Fig. 6a path).
# Phase D default uses a thrust-feasible sped-up period (not 600 s — that outruns 2×0.25 N).
# Feasibility: a_cent ≈ A(2π/T)² must stay ≪ F_max/m ≈ 5e-4 m/s² → T ≳ 1 h for A=2 m.
INSPECTION_PERIOD_PAPER_S = 28800.0
INSPECTION_PERIOD_SPEEDUP_S = 3600.0
INSPECTION_PERIOD_SPEEDUP_RUNTIME_S = 900.0  # 1/4 of sped-up arc (short gate)
INSPECTION_PERIOD = INSPECTION_PERIOD_PAPER_S
INSPECTION_AMPLITUDE_YZ = 2.0
USE_FIXED_OFFSET = False
PHASE_D_ACTIVE = False
ALIGN_LOS_AT_INIT = True
# With --fixedOffset, place the hub this far from RHO0 so allocation must reconfigure
# arms / thrusters. Perfect ICs keep tip-only spin and booms look "horizontal".
FIXED_OFFSET_IC_DELTA_N = np.array([0.0, 3.0, 0.0])

# Paper hub-only mass; arms add ~20 kg at zero joint angles
PAPER_HUB_MASS_KG = 980.0
PAPER_SYSTEM_MASS_KG = 1000.0


def mrp_point_body_axis(p_hat_B, direction_N):
  """MRP so body axis p_hat_B points along direction_N (inertial)."""
  d = np.array(direction_N, dtype=float)
  d /= np.linalg.norm(d)
  aux = np.array([0.0, 0.0, 1.0])
  if abs(np.dot(d, aux)) > 0.9:
    aux = np.array([0.0, 1.0, 0.0])
  b1 = d
  b2 = np.cross(aux, b1)
  b2 /= np.linalg.norm(b2)
  b3 = np.cross(b1, b2)
  BN = np.vstack([b1, b2, b3])
  return np.array(rbk.C2MRP(BN)).flatten()


def inspection_offset(t: float):
  """Relative offset rho*(t) from chief to desired deputy CoM."""
  if USE_FIXED_OFFSET:
    return RHO0_N.copy(), np.zeros(3), np.zeros(3)

  omega = 2.0 * np.pi / INSPECTION_PERIOD
  # delta(0)=0 so hub IC at RHO0 matches reference at t=0
  delta = np.array([
      0.0,
      INSPECTION_AMPLITUDE_YZ * np.sin(omega * t),
      INSPECTION_AMPLITUDE_YZ * (1.0 - np.cos(omega * t)),
  ])
  rho = RHO0_N + delta
  v_rho = np.array([
      0.0,
      INSPECTION_AMPLITUDE_YZ * omega * np.cos(omega * t),
      INSPECTION_AMPLITUDE_YZ * omega * np.sin(omega * t),
  ])
  a_rho = np.array([
      0.0,
      -INSPECTION_AMPLITUDE_YZ * omega**2 * np.sin(omega * t),
      INSPECTION_AMPLITUDE_YZ * omega**2 * np.cos(omega * t),
  ])
  return rho, v_rho, a_rho


def _write_trans_ref(trans_ref_msg, r_N, v_N, a_N, sim_nanos=0):
  payload = messaging.TransRefMsgPayload()
  payload.r_RN_N = np.array(r_N, dtype=float).tolist()
  payload.v_RN_N = np.array(v_N, dtype=float).tolist()
  payload.a_RN_N = np.array(a_N, dtype=float).tolist()
  trans_ref_msg.write(payload, sim_nanos)


class DeputyTransRefUpdater(sysModel.SysModel):
  """Write time-varying deputy CoM reference: r*_d = r_chief + rho(t)."""

  def __init__(self, trans_ref_msg, chief_nav_reader):
    super().__init__()
    self.transRefMsg = trans_ref_msg
    self.chiefNavReader = chief_nav_reader

  def UpdateState(self, CurrentSimNanos):
    t = CurrentSimNanos * mc.NANO2SEC
    rho, v_rho, a_rho = inspection_offset(t)
    chief = self.chiefNavReader()
    r_ref = np.array(chief.r_BN_N) + rho
    v_ref = np.array(chief.v_BN_N) + v_rho
    _write_trans_ref(self.transRefMsg, r_ref, v_ref, a_rho, CurrentSimNanos)


class scenarioThrArmPaper(BSKSim, BSKScenario):
  def __init__(
      self,
      outerRate=10.0,
      innerRate=0.01,
      dynRate=0.01,
      tau_pre_limit_nm=None,
      one_cycle=False,
      log_ncon=False,
      moving_path=False,
      path_period_s=None,
  ):
    super(scenarioThrArmPaper, self).__init__(fswRate=innerRate, dynRate=dynRate)
    self.name = "scenarioThrArmPaper"
    self.tau_pre_limit_nm = tau_pre_limit_nm
    self.one_cycle = one_cycle
    self.log_ncon = bool(log_ncon)
    self.moving_path = bool(moving_path)
    self.path_period_s = path_period_s
    self.outerRate = outerRate
    self.innerRate = innerRate

    self.posRec = None
    self.attRec = None
    self.stateRec = None
    self.transRefRec = None
    self.tauPreRecs = []
    self.jointRecs = []
    self.jointNames = []
    self.nconLogger = None

    self.chiefSC = None
    self.chiefNav = None
    self.transRefUpdater = None

    self.set_DynModel(BSK_MujocoDynamics)
    self._set_paper_fsw()
    self._setup_chief()
    self.log_outputs()
    if self.log_ncon:
      self._setup_ncon_logger()

  def _setup_ncon_logger(self):
    FSW = self.get_FswModel()
    self.nconLogger = CmdNconLogger(FSW.desAngleReader, xml_path=XML_PATH)
    # Run after allocation on the outer loop.
    self.AddModelToTask(FSW.outerLoopTaskName, self.nconLogger, 40)
    print(
        f"[PhaseC] logging ncon vs commanded θ (geometry={BSK_MujocoFSW_Paper.GEOMETRY_MODE})",
        flush=True,
    )

  def _set_paper_fsw(self):
    self.fsw_added = True
    self.FSWProcessName = "FSWProcess"
    self.fswProc = self.CreateNewProcess(self.FSWProcessName)

    self.FSWModels = BSK_MujocoFSW_Paper.BSKMujocoFSWModels(
        self,
        self.outerRate,
        self.innerRate,
        tau_pre_limit_nm=self.tau_pre_limit_nm,
        one_cycle=self.one_cycle,
        moving_path=self.moving_path,
        path_period_s=self.path_period_s if self.path_period_s is not None else INSPECTION_PERIOD,
    )

  def _setup_chief(self):
    """Chief spacecraft: Basilisk rigid body only (not in MuJoCo XML)."""
    DynModel = self.get_DynModel()
    self.chiefSC = spacecraft.Spacecraft()
    self.chiefSC.ModelTag = "chief"
    self.chiefSC.hub.mHub = 1000.0
    self.chiefSC.hub.IHubPntBc_B = [
        [900.0, 0.0, 0.0],
        [0.0, 800.0, 0.0],
        [0.0, 0.0, 600.0],
    ]

    self.chiefNav = simpleNav.SimpleNav()
    self.chiefNav.ModelTag = "chiefNav"
    self.chiefNav.scStateInMsg.subscribeTo(self.chiefSC.scStateOutMsg)

    self.AddModelToTask(DynModel.taskName, self.chiefSC)
    self.AddModelToTask(DynModel.taskName, self.chiefNav)

    FSWModel = self.get_FswModel()
    FSWModel.connect_chief_target(self.chiefNav.transOutMsg)

    self.transRefUpdater = DeputyTransRefUpdater(
        FSWModel.transRefMsg,
        messaging.NavTransMsgReader(),
    )
    self.transRefUpdater.chiefNavReader.subscribeTo(self.chiefNav.transOutMsg)
    # Keep ρ*(t) live on the dynamics task so it does not freeze while FSW outer
    # is disabled during jointMotion / thrFiring (Phase D lag source).
    self.AddModelToTask(DynModel.taskName, self.transRefUpdater, 5)

  def configure_initial_conditions(self):
    DynModel = self.get_DynModel()
    FSWModel = self.get_FswModel()
    hubName = FSWModel.hubName
    jointRefs = FSWModel.jointRefs

    self.chiefSC.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    self.chiefSC.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    self.chiefSC.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    self.chiefSC.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    for _, (bodyName, jointName) in enumerate(jointRefs):
      joint = DynModel.scene.getBody(bodyName).getScalarJoint(jointName)
      joint.setPosition(0.0)
      joint.setVelocity(0.0)

    # TransRef follows rho*(t). For --fixedOffset, start a few meters off RHO0 so
    # the controller must thrust and reconfigure (perfect ICs leave booms near 0).
    rho0, v_rho0, a_rho0 = inspection_offset(0.0)
    if USE_FIXED_OFFSET:
      r0_N = rho0 + FIXED_OFFSET_IC_DELTA_N
      v0_N = np.zeros(3)
    else:
      r0_N = rho0
      v0_N = v_rho0
    DynModel.scene.getBody(hubName).setPosition(r0_N.tolist())
    DynModel.scene.getBody(hubName).setVelocity(v0_N.tolist())

    if ALIGN_LOS_AT_INIT:
      los_hat = -r0_N / np.linalg.norm(r0_N)
      sigma0 = mrp_point_body_axis([1.0, 0.0, 0.0], los_hat)
    else:
      sigma0 = np.array([0.16, 0.05, -0.54])
    DynModel.scene.getBody(hubName).setAttitude(sigma0.tolist())
    DynModel.scene.getBody(hubName).setAttitudeRate([0.0, 0.0, 0.0])

    _write_trans_ref(FSWModel.transRefMsg, rho0, v_rho0, a_rho0)
    if USE_FIXED_OFFSET:
      print(
          f"[ThrArmPaper] fixedOffset IC: hub at RHO0+{FIXED_OFFSET_IC_DELTA_N.tolist()}, "
          f"TransRef at RHO0 (initial |pos err|={np.linalg.norm(FIXED_OFFSET_IC_DELTA_N):.2f} m)",
          flush=True,
      )
    elif self.moving_path or PHASE_D_ACTIVE:
      print(
          f"[PhaseD] moving ρ*(t): period={INSPECTION_PERIOD:.1f}s  "
          f"A_yz={INSPECTION_AMPLITUDE_YZ:.2f}m  "
          f"IC at ρ(0) with v=ρ̇(0)  |v|={np.linalg.norm(v_rho0):.4f} m/s",
          flush=True,
      )

  def log_outputs(self):
    DynModel = self.get_DynModel()
    FSWModel = self.get_FswModel()

    self.posRec = DynModel.systemCoM.comStatesOutMsg.recorder()
    self.attRec = DynModel.scene.getBody("hub").getOrigin().stateOutMsg.recorder()
    self.stateRec = DynModel.scene.stateOutMsg.recorder()
    self.transRefRec = FSWModel.transRefMsg.recorder()
    self.tauPreRecs = [gw.recorder() for gw in FSWModel.bodyTorqueGateway]
    self.jointRecs = []
    self.jointNames = []
    for bodyName, jointName in FSWModel.jointRefs:
      rec = DynModel.scene.getBody(bodyName).getScalarJoint(jointName).stateOutMsg.recorder()
      self.jointRecs.append(rec)
      self.jointNames.append(f"{bodyName}/{jointName}")

    self.AddModelToTask(DynModel.taskName, self.posRec)
    self.AddModelToTask(DynModel.taskName, self.attRec)
    self.AddModelToTask(DynModel.taskName, self.stateRec)
    for rec in self.jointRecs:
      self.AddModelToTask(DynModel.taskName, rec)
    self.AddModelToTask(FSWModel.jointMotionTaskName, self.tauPreRecs[0])
    self.AddModelToTask(FSWModel.jointMotionTaskName, self.tauPreRecs[1])
    self.AddModelToTask(FSWModel.jointMotionTaskName, self.tauPreRecs[2])
    # Record TransRef on dynamics task (always running) — matches live ρ*(t).
    self.AddModelToTask(DynModel.taskName, self.transRefRec)

  def pull_outputs(self, showPlots: bool = False):
    times = self.posRec.times() * mc.NANO2SEC
    r_CN_N = self.posRec.r_CN_N
    t_ref = self.transRefRec.times() * mc.NANO2SEC
    r_ref_N = self.transRefRec.r_RN_N
    r_ref_interp = np.zeros_like(r_CN_N)
    for axis in range(3):
      if len(t_ref) > 1:
        r_ref_interp[:, axis] = np.interp(times, t_ref, r_ref_N[:, axis])
      else:
        r_ref_interp[:, axis] = r_ref_N[0, axis]
    pos_err = r_CN_N - r_ref_interp
    sigma_BN = self.attRec.sigma_BN
    tau_pre = np.vstack([rec.input for rec in self.tauPreRecs]).T
    tau_times = self.tauPreRecs[0].times() * mc.NANO2SEC

    # Print simple convergence diagnostics over the final time window.
    # This avoids relying on interactive plots when tuning.
    if len(times) > 0:
      window_sec = 60.0  # [s]
      window_start = times[-1] - window_sec
      win_mask = times >= window_start

      if np.any(win_mask):
        pos_err_win = pos_err[win_mask, :]
        sigma_win = sigma_BN[win_mask, :]

        rms_pos_err = np.sqrt(np.mean(pos_err_win**2, axis=0))
        max_abs_pos_err = np.max(np.abs(pos_err_win), axis=0)
        rms_sigma = np.sqrt(np.mean(sigma_win**2, axis=0))
        max_abs_sigma = np.max(np.abs(sigma_win), axis=0)

        tau_rms = np.array([0.0, 0.0, 0.0])
        tau_max_abs = np.array([0.0, 0.0, 0.0])
        if len(tau_times) > 0:
          tau_win_mask = tau_times >= window_start
          if np.any(tau_win_mask):
            tau_win = tau_pre[tau_win_mask, :]
            tau_rms = np.sqrt(np.mean(tau_win**2, axis=0))
            tau_max_abs = np.max(np.abs(tau_win), axis=0)

        print(
            "[ThrArmPaper] last 60 s metrics:"
            f" pos_rms=[{rms_pos_err[0]:.3f},{rms_pos_err[1]:.3f},{rms_pos_err[2]:.3f}]"
            f" pos_max_abs=[{max_abs_pos_err[0]:.3f},{max_abs_pos_err[1]:.3f},{max_abs_pos_err[2]:.3f}]"
            f" sigma_rms=[{rms_sigma[0]:.4f},{rms_sigma[1]:.4f},{rms_sigma[2]:.4f}]"
            f" sigma_max_abs=[{max_abs_sigma[0]:.4f},{max_abs_sigma[1]:.4f},{max_abs_sigma[2]:.4f}]"
            f" tau_rms=[{tau_rms[0]:.2f},{tau_rms[1]:.2f},{tau_rms[2]:.2f}]"
            f" tau_max_abs=[{tau_max_abs[0]:.2f},{tau_max_abs[1]:.2f},{tau_max_abs[2]:.2f}]"
        )

    joint_hist = []
    if self.jointRecs:
      joint_hist = [np.asarray(rec.state, dtype=float).reshape(-1) for rec in self.jointRecs]
      j_max = [float(np.max(np.abs(j))) if j.size else 0.0 for j in joint_hist]
      j_final = [float(j[-1]) if j.size else 0.0 for j in joint_hist]
      print(
          "[ThrArmPaper] joints max|θ|="
          + ",".join(f"{v:.3f}" for v in j_max)
          + " final="
          + ",".join(f"{v:.3f}" for v in j_final),
          flush=True,
      )

    if getattr(self, "one_cycle", False):
      self.print_phase_a_report()
    elif USE_FIXED_OFFSET:
      self.print_phase_b_report()
    elif self.moving_path or PHASE_D_ACTIVE:
      self.print_phase_d_report()
    if getattr(self, "nconLogger", None) is not None:
      self.nconLogger.report()

    color_x = unitTestSupport.getLineColor(0, 3)
    color_y = unitTestSupport.getLineColor(1, 3)
    color_z = unitTestSupport.getLineColor(2, 3)

    figureList = {}

    fig1 = plt.figure()
    plt.plot(times, pos_err[:, 0], color=color_x, label="x")
    plt.plot(times, pos_err[:, 1], color=color_y, label="y")
    plt.plot(times, pos_err[:, 2], color=color_z, label="z")
    plt.xlabel("Time [s]")
    plt.ylabel("CoM Position Error [m]")
    plt.legend()
    figureList[fileNameForImages + "_position_error"] = fig1

    fig2 = plt.figure()
    plt.plot(times, sigma_BN[:, 0], color=color_x, label=r"$\sigma_1$")
    plt.plot(times, sigma_BN[:, 1], color=color_y, label=r"$\sigma_2$")
    plt.plot(times, sigma_BN[:, 2], color=color_z, label=r"$\sigma_3$")
    plt.xlabel("Time [s]")
    plt.ylabel(r"Deputy Attitude [MRP]")
    plt.legend()
    figureList[fileNameForImages + "_attitude"] = fig2

    fig3 = plt.figure()
    if len(tau_times) > 0:
      plt.plot(tau_times, tau_pre[:, 0], color=color_x, label=r"$\tau_{pre,x}$")
      plt.plot(tau_times, tau_pre[:, 1], color=color_y, label=r"$\tau_{pre,y}$")
      plt.plot(tau_times, tau_pre[:, 2], color=color_z, label=r"$\tau_{pre,z}$")
    plt.xlabel("Time [s]")
    plt.ylabel(r"Prescribed Hub Torque [N m]")
    plt.legend()
    figureList[fileNameForImages + "_tau_pre"] = fig3

    if joint_hist:
      fig4 = plt.figure()
      j_times = self.jointRecs[0].times() * mc.NANO2SEC
      for i, (name, hist) in enumerate(zip(self.jointNames, joint_hist)):
        short = name.split("/")[-1]
        plt.plot(j_times, hist, label=short, color=unitTestSupport.getLineColor(i, 8))
      plt.xlabel("Time [s]")
      plt.ylabel("Joint angle [rad]")
      plt.legend(fontsize=7, ncol=2)
      figureList[fileNameForImages + "_joints"] = fig4

    # Phase D / Fig. 6-style: relative YZ path (chief at origin → ρ ≈ r_CN).
    if self.moving_path or PHASE_D_ACTIVE or (not USE_FIXED_OFFSET):
      rho_star = np.zeros_like(r_CN_N)
      for i, t in enumerate(times):
        rho_star[i], _, _ = inspection_offset(float(t))
      fig5 = plt.figure()
      plt.plot(rho_star[:, 1], rho_star[:, 2], "k--", lw=1.5, label=r"$\rho^*(t)$")
      plt.plot(r_CN_N[:, 1], r_CN_N[:, 2], color=color_y, lw=1.2, label=r"$\rho(t)$")
      plt.plot(rho_star[0, 1], rho_star[0, 2], "ko", ms=5, label="start")
      plt.xlabel(r"$\rho_y$ [m]")
      plt.ylabel(r"$\rho_z$ [m]")
      plt.axis("equal")
      plt.legend()
      plt.title("Inspection path (YZ) — qualitative Fig. 6-style")
      figureList[fileNameForImages + "_path_yz"] = fig5

      fig6 = plt.figure()
      plt.plot(times, rho_star[:, 1], "k--", label=r"$\rho^*_y$")
      plt.plot(times, r_CN_N[:, 1], color=color_y, label=r"$\rho_y$")
      plt.plot(times, rho_star[:, 2], "k:", label=r"$\rho^*_z$")
      plt.plot(times, r_CN_N[:, 2], color=color_z, label=r"$\rho_z$")
      plt.xlabel("Time [s]")
      plt.ylabel(r"Relative offset [m]")
      plt.legend(fontsize=8, ncol=2)
      figureList[fileNameForImages + "_rho_yz_time"] = fig6

    return figureList

  def print_phase_a_report(self):
    """Score the first allocate→settle→fire→coast cycle (Phase A)."""
    FSW = self.get_FswModel()
    log = dict(FSW.cycleLog)
    times = self.posRec.times() * mc.NANO2SEC
    r_CN_N = np.asarray(self.posRec.r_CN_N, dtype=float)
    rho0, _, _ = inspection_offset(0.0)

    def _err_at(t_s):
      if t_s is None or len(times) == 0:
        return None
      i = int(np.argmin(np.abs(times - float(t_s))))
      return r_CN_N[i] - rho0

    e0 = r_CN_N[0] - rho0 if len(r_CN_N) else None
    e_joint = _err_at(log.get("t_joint_start"))
    e_coast = _err_at(log.get("t_coast"))
    e_end = r_CN_N[-1] - rho0 if len(r_CN_N) else None

    tau_pre = np.vstack([rec.input for rec in self.tauPreRecs]).T if self.tauPreRecs else np.zeros((0, 3))
    tau_max = float(np.max(np.abs(tau_pre))) if tau_pre.size else 0.0
    tau_limit = float(log.get("tau_pre_limit_nm") or BSK_MujocoFSW_Paper.DEFAULT_TAU_PRE_LIMIT_NM)

    # Joints inside FSW box at fire? Allow settle-band overshoot.
    box = BSK_MujocoFSW_Paper.ALLOC_JOINT_LIMIT_RAD
    settle_band = BSK_MujocoFSW_Paper.JOINT_SETTLE_ANGLE_TOL_RAD
    angles_fire = log.get("angles_at_fire")
    in_box = None
    j_max_abs = None
    if angles_fire is not None:
      af = np.asarray(angles_fire, dtype=float)
      j_max_abs = float(np.max(np.abs(af)))
      in_box = bool(np.all(np.abs(af) <= box + settle_band + 1e-3))

    # y-error should move toward 0 (measurable) and not reverse through the setpoint.
    dy_ok = False
    if e_joint is not None and e_end is not None:
      dy_joint = float(e_joint[1])
      dy_end = float(e_end[1])
      dy_ok = (abs(dy_end) <= abs(dy_joint) - 0.02) and (
          dy_end * dy_joint >= 0.0 or abs(dy_end) < 0.25
      )

    settle_ok = (log.get("t_fire") is not None) and (not log.get("settle_timeout", True))
    theta_err = log.get("max_abs_theta_err_at_fire")
    theta_ok = theta_err is not None and float(theta_err) < 0.08
    tau_ok = tau_max <= tau_limit + 1e-6
    one_reconfig = int(log.get("n_reconfig") or 0) == 1
    cycle_done = bool(FSW.cycle_complete)

    checks = {
        "cycle_completed": cycle_done,
        "single_reconfig": one_reconfig,
        "settled_no_timeout": settle_ok,
        "theta_err_at_fire_ok": theta_ok,
        "joints_in_alloc_box": bool(in_box) if in_box is not None else False,
        "tau_pre_within_cap": tau_ok,
        "y_error_toward_RHO0": dy_ok,
    }
    passed = all(checks.values())

    print("\n========== Phase A report (one outer cycle) ==========", flush=True)
    print(f"  t_joint={log.get('t_joint_start')}  t_fire={log.get('t_fire')}  t_coast={log.get('t_coast')}", flush=True)
    print(f"  n_reconfig={log.get('n_reconfig')}  settle_timeout={log.get('settle_timeout')}", flush=True)
    print(f"  max|θ_err| at fire={theta_err}", flush=True)
    if log.get("des_angles") is not None:
      print(
          "  desθ="
          + np.array2string(np.asarray(log["des_angles"], float), precision=3, separator=","),
          flush=True,
      )
    if angles_fire is not None:
      print(
          "  θ_fire="
          + np.array2string(np.asarray(angles_fire, float), precision=3, separator=","),
          flush=True,
      )
      print(f"  joints_in_box={in_box}  max|θ|_fire={j_max_abs}", flush=True)
    if log.get("on_times") is not None:
      print(
          "  thr on-times="
          + np.array2string(np.asarray(log["on_times"], float), precision=4, separator=","),
          flush=True,
      )
    if e0 is not None:
      print(f"  pos_err IC   =[{e0[0]:.3f},{e0[1]:.3f},{e0[2]:.3f}]", flush=True)
    if e_joint is not None:
      print(f"  pos_err joint=[{e_joint[0]:.3f},{e_joint[1]:.3f},{e_joint[2]:.3f}]", flush=True)
    if e_coast is not None:
      print(f"  pos_err coast=[{e_coast[0]:.3f},{e_coast[1]:.3f},{e_coast[2]:.3f}]", flush=True)
    if e_end is not None:
      print(f"  pos_err end  =[{e_end[0]:.3f},{e_end[1]:.3f},{e_end[2]:.3f}]", flush=True)
    print(f"  tau_pre max|τ|={tau_max:.2f} N·m  (cap ±{tau_limit:.1f})", flush=True)
    for k, v in checks.items():
      print(f"  [{'PASS' if v else 'FAIL'}] {k}", flush=True)
    print(
        f"========== Phase A: {'PASS' if passed else 'FAIL'} ==========\n",
        flush=True,
    )
    return passed

  def print_phase_b_report(self):
    """Score fixed-offset multi-cycle hover (Phase B)."""
    FSW = self.get_FswModel()
    times = self.posRec.times() * mc.NANO2SEC
    r_CN_N = np.asarray(self.posRec.r_CN_N, dtype=float)
    sigma_BN = np.asarray(self.attRec.sigma_BN, dtype=float)
    rho0, _, _ = inspection_offset(0.0)
    if len(times) == 0:
      print("[PhaseB] no samples", flush=True)
      return False

    window_sec = 60.0
    win = times >= (times[-1] - window_sec)
    pos_err = r_CN_N - rho0
    pos_err_win = pos_err[win]
    sig_win = sigma_BN[win]
    pos_rms = np.sqrt(np.mean(pos_err_win**2, axis=0))
    pos_max = np.max(np.abs(pos_err_win), axis=0)
    sig_rms = np.sqrt(np.mean(sig_win**2, axis=0))
    sig_max = np.max(np.abs(sig_win), axis=0)
    pos_norm_rms = float(np.sqrt(np.mean(np.sum(pos_err_win**2, axis=1))))
    pos_norm_max = float(np.max(np.linalg.norm(pos_err_win, axis=1)))

    joint_static = True
    j_std_max = 0.0
    if self.jointRecs:
      j_times = self.jointRecs[0].times() * mc.NANO2SEC
      j_win = j_times >= (j_times[-1] - window_sec) if len(j_times) else []
      stds = []
      for rec in self.jointRecs:
        hist = np.asarray(rec.state, dtype=float).reshape(-1)
        if len(hist) and np.any(j_win):
          stds.append(float(np.std(hist[j_win])))
      j_std_max = max(stds) if stds else 0.0
      joint_static = j_std_max < 0.25

    # Attitude: σ1/σ2 should be quiet; a steady σ3 bias is OK for LOS MRP framing.
    att_quiet = float(max(sig_rms[0], sig_rms[1])) < 0.05 and float(max(sig_max[0], sig_max[1])) < 0.15

    tau_pre = np.vstack([rec.input for rec in self.tauPreRecs]).T if self.tauPreRecs else np.zeros((0, 3))
    tau_times = self.tauPreRecs[0].times() * mc.NANO2SEC if self.tauPreRecs else np.array([])
    tau_max = 0.0
    if tau_pre.size and len(tau_times):
      tw = tau_times >= (tau_times[-1] - window_sec)
      if np.any(tw):
        tau_max = float(np.max(np.abs(tau_pre[tw])))

    checks = {
        "pos_norm_rms_<0.5m": pos_norm_rms < 0.5,
        "pos_norm_max_<0.8m": pos_norm_max < 0.8,
        "att_sigma12_quiet": att_quiet,
        "arms_nearly_static": joint_static,
        "few_settle_timeouts": int(getattr(FSW, "n_settle_timeout", 0)) <= 3,
    }
    passed = all(checks.values())

    print("\n========== Phase B report (fixed-offset hover) ==========", flush=True)
    print(
        f"  last 60s |ρ−ρ*| rms={pos_norm_rms:.3f}m max={pos_norm_max:.3f}m  "
        f"pos_rms=[{pos_rms[0]:.3f},{pos_rms[1]:.3f},{pos_rms[2]:.3f}]  "
        f"pos_max=[{pos_max[0]:.3f},{pos_max[1]:.3f},{pos_max[2]:.3f}]",
        flush=True,
    )
    print(
        f"  last 60s σ rms={np.array2string(sig_rms, precision=4)}  "
        f"max={np.array2string(sig_max, precision=4)}  "
        f"joint std_max={j_std_max:.3f}  τ_pre max={tau_max:.2f}",
        flush=True,
    )
    print(
        f"  reconfigs={FSW.cycleLog.get('n_reconfig')}  "
        f"directFires={getattr(FSW, 'n_direct_fire', 0)}  "
        f"settle_timeouts={getattr(FSW, 'n_settle_timeout', 0)}",
        flush=True,
    )
    for k, v in checks.items():
      print(f"  [{'PASS' if v else 'FAIL'}] {k}", flush=True)
    print(
        f"========== Phase B: {'PASS' if passed else 'FAIL'} ==========\n",
        flush=True,
    )
    return passed

  def print_phase_d_report(self):
    """Score moving ρ*(t) tracking (Phase D). Path itself is not published — qualitative only."""
    FSW = self.get_FswModel()
    times = self.posRec.times() * mc.NANO2SEC
    r_CN_N = np.asarray(self.posRec.r_CN_N, dtype=float)
    sigma_BN = np.asarray(self.attRec.sigma_BN, dtype=float)
    if len(times) == 0:
      print("[PhaseD] no samples", flush=True)
      return False

    # Tracking error vs the live TransRef (r*_d = r_c + ρ*).
    t_ref = self.transRefRec.times() * mc.NANO2SEC
    r_ref_N = np.asarray(self.transRefRec.r_RN_N, dtype=float)
    r_ref_interp = np.zeros_like(r_CN_N)
    for axis in range(3):
      if len(t_ref) > 1:
        r_ref_interp[:, axis] = np.interp(times, t_ref, r_ref_N[:, axis])
      elif len(t_ref) == 1:
        r_ref_interp[:, axis] = r_ref_N[0, axis]
    pos_err = r_CN_N - r_ref_interp

    window_sec = min(60.0, 0.25 * float(times[-1] - times[0]) if times[-1] > times[0] else 60.0)
    window_sec = max(window_sec, 30.0)
    win = times >= (times[-1] - window_sec)
    pos_err_win = pos_err[win]
    sig_win = sigma_BN[win]
    pos_rms = np.sqrt(np.mean(pos_err_win**2, axis=0))
    pos_max = np.max(np.abs(pos_err_win), axis=0)
    sig_rms = np.sqrt(np.mean(sig_win**2, axis=0))
    sig_max = np.max(np.abs(sig_win), axis=0)
    pos_norm_rms = float(np.sqrt(np.mean(np.sum(pos_err_win**2, axis=1))))
    pos_norm_max = float(np.max(np.linalg.norm(pos_err_win, axis=1)))
    pos_norm_rms_full = float(np.sqrt(np.mean(np.sum(pos_err**2, axis=1))))
    pos_norm_max_full = float(np.max(np.linalg.norm(pos_err, axis=1)))

    # Commanded ρ* coverage in YZ (ellipse of radius A around RHO0+(0,0,A)).
    rho_star = np.vstack([inspection_offset(float(t))[0] for t in times])
    dy = rho_star[:, 1] - RHO0_N[1]
    dz = rho_star[:, 2] - RHO0_N[2]
    span_y = float(np.max(dy) - np.min(dy))
    span_z = float(np.max(dz) - np.min(dz))
    # Actual relative path span (chief at origin → ρ ≈ r_CN).
    act_span_y = float(np.max(r_CN_N[:, 1]) - np.min(r_CN_N[:, 1]))
    act_span_z = float(np.max(r_CN_N[:, 2]) - np.min(r_CN_N[:, 2]))
    frac_period = float(times[-1] - times[0]) / max(INSPECTION_PERIOD, 1e-9)

    paper_speed = abs(INSPECTION_PERIOD - INSPECTION_PERIOD_PAPER_S) < 1.0
    track_rms_gate = 1.0 if paper_speed else 0.50
    track_max_gate = 1.8 if paper_speed else 1.0

    # Soft gates: moving path is harder than hover; Fig. 6 data unavailable.
    att_quiet = float(max(sig_rms[0], sig_rms[1])) < 0.15
    path_moved = span_y > 0.5 * INSPECTION_AMPLITUDE_YZ or span_z > 0.5 * INSPECTION_AMPLITUDE_YZ
    # Both Y and Z must follow ρ* (not just horizontal drift).
    followed_y = act_span_y > 0.45 * max(span_y, 1e-6)
    followed_z = act_span_z > 0.45 * max(span_z, 1e-6)
    followed = followed_y and followed_z
    settle_ok = int(getattr(FSW, "n_settle_timeout", 0)) <= 8
    not_fixed = not USE_FIXED_OFFSET
    arms_locked_ok = not getattr(FSW, "arms_locked", False)

    checks = {
        "not_fixedOffset": not_fixed,
        "rho_star_moves": path_moved,
        "deputy_follows_y": followed_y,
        "deputy_follows_z": followed_z,
        "track_rms_<gate": pos_norm_rms < track_rms_gate,
        "track_max_<gate": pos_norm_max < track_max_gate,
        "full_run_max_<gate": pos_norm_max_full < (1.4 * track_max_gate),
        "att_sigma12_quiet": att_quiet,
        "few_settle_timeouts": settle_ok,
        "arms_not_locked_at_end": arms_locked_ok,
    }
    passed = all(checks.values())

    print("\n========== Phase D report (moving inspection path) ==========", flush=True)
    print(
        f"  ρ*(t): period={INSPECTION_PERIOD:.1f}s  A_yz={INSPECTION_AMPLITUDE_YZ:.2f}m  "
        f"sim≈{frac_period:.2f} periods  "
        f"({'paper-speed' if abs(INSPECTION_PERIOD - INSPECTION_PERIOD_PAPER_S) < 1.0 else 'sped-up'})",
        flush=True,
    )
    print(
        f"  last {window_sec:.0f}s |ρ−ρ*| rms={pos_norm_rms:.3f}m max={pos_norm_max:.3f}m  "
        f"full-run max={pos_norm_max_full:.3f}m  gates rms<{track_rms_gate:.2f} max<{track_max_gate:.2f}  "
        f"pos_rms=[{pos_rms[0]:.3f},{pos_rms[1]:.3f},{pos_rms[2]:.3f}]  "
        f"pos_max=[{pos_max[0]:.3f},{pos_max[1]:.3f},{pos_max[2]:.3f}]",
        flush=True,
    )
    print(
        f"  last {window_sec:.0f}s σ rms={np.array2string(sig_rms, precision=4)}  "
        f"max={np.array2string(sig_max, precision=4)}",
        flush=True,
    )
    print(
        f"  ρ* YZ span Δy={span_y:.2f}m Δz={span_z:.2f}m  "
        f"actual Δy={act_span_y:.2f}m Δz={act_span_z:.2f}m",
        flush=True,
    )
    print(
        f"  reconfigs={FSW.cycleLog.get('n_reconfig')}  "
        f"directFires={getattr(FSW, 'n_direct_fire', 0)}  "
        f"settle_timeouts={getattr(FSW, 'n_settle_timeout', 0)}  "
        f"momDumps={getattr(FSW, 'n_momentum_dump', 0)}  "
        f"arms_locked={getattr(FSW, 'arms_locked', False)}",
        flush=True,
    )
    print(
        "  Note: Schwend & Schaub Fig. 6a path data are unpublished — "
        "compare qualitatively (smooth YZ inspection + LOS), not numerically.",
        flush=True,
    )
    for k, v in checks.items():
      print(f"  [{'PASS' if v else 'FAIL'}] {k}", flush=True)
    print(
        f"========== Phase D: {'PASS' if passed else 'FAIL'} ==========\n",
        flush=True,
    )
    return passed


class LivePlotter:
  """Update matplotlib traces while the simulation is running."""

  def __init__(self):
    plt.ion()
    self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    self.fig.canvas.manager.set_window_title("ThrArmPaper live")
    self.lines = {}
    labels = [
        ("pos", ["x", "y", "z"], "CoM position error [m]"),
        ("att", [r"$\sigma_1$", r"$\sigma_2$", r"$\sigma_3$"], "Deputy attitude [MRP]"),
        ("tau", [r"$\tau_x$", r"$\tau_y$", r"$\tau_z$"], r"$\tau_{pre}$ [N m]"),
    ]
    for ax_idx, (key, names, ylabel) in enumerate(labels):
      ax = self.axes[ax_idx]
      self.lines[key] = []
      for i, name in enumerate(names):
        line, = ax.plot([], [], color=unitTestSupport.getLineColor(i, 3), label=name)
        self.lines[key].append(line)
      ax.set_ylabel(ylabel)
      ax.legend(loc="upper right", fontsize=8)
      ax.grid(True, alpha=0.3)
    self.axes[-1].set_xlabel("Time [s]")
    self.fig.tight_layout()
    self.fig.canvas.draw()
    self.fig.canvas.flush_events()

  def update(self, scenario):
    times = scenario.posRec.times() * mc.NANO2SEC
    if len(times) == 0:
      return

    r_CN_N = scenario.posRec.r_CN_N
    t_ref = scenario.transRefRec.times() * mc.NANO2SEC
    r_ref_N = scenario.transRefRec.r_RN_N
    r_ref_interp = np.zeros_like(r_CN_N)
    for axis in range(3):
      if len(t_ref) > 1:
        r_ref_interp[:, axis] = np.interp(times, t_ref, r_ref_N[:, axis])
      elif len(t_ref) == 1:
        r_ref_interp[:, axis] = r_ref_N[0, axis]
    pos_err = r_CN_N - r_ref_interp
    sigma_BN = scenario.attRec.sigma_BN

    datasets = [pos_err, sigma_BN]
    keys = ["pos", "att"]
    for key, data in zip(keys, datasets):
      for i, line in enumerate(self.lines[key]):
        line.set_data(times, data[:, i])

    tau_times = scenario.tauPreRecs[0].times() * mc.NANO2SEC
    if len(tau_times) > 0:
      for i, line in enumerate(self.lines["tau"]):
        line.set_data(tau_times, scenario.tauPreRecs[i].input)

    for ax in self.axes:
      ax.relim()
      ax.autoscale_view()
    self.fig.canvas.draw_idle()
    self.fig.canvas.flush_events()
    plt.pause(0.001)


class LiveMujocoView:
  """Passive MuJoCo viewer synced to the latest Basilisk MJScene state."""

  def __init__(self, scenario):
    if not _MJ_VIEWER_OK:
      raise RuntimeError("mujoco.viewer not available; install the mujoco Python package.")
    self.model = mj.MjModel.from_xml_path(XML_PATH)
    self.data = mj.MjData(self.model)
    self.reader = messaging.MJSceneStateMsgReader()
    self.reader.subscribeTo(scenario.get_DynModel().scene.stateOutMsg)
    self.viewer = mj_viewer.launch_passive(self.model, self.data)
    self.viewer.cam.trackbodyid = self.model.body("hub").id
    self.viewer.cam.distance = 8.0

  def sync(self):
    if self.viewer is None or not self.viewer.is_running():
      return
    if not self.reader.isWritten():
      return
    qpos = np.asarray(self.reader().qpos, dtype=float).reshape(-1)
    if qpos.size != self.model.nq:
      return
    self.data.qpos[:] = qpos
    mj.mj_forward(self.model, self.data)
    self.viewer.sync()

  def close(self):
    if self.viewer is not None:
      try:
        if self.viewer.is_running():
          self.viewer.close()
      except Exception:
        pass
      self.viewer = None


def runScenario(
    scenario,
    runTime: float = 320.0,
    live_plots: bool = False,
    live_view: bool = False,
    live_interval: float = 30.0,
    show_progress: bool = False,
):
  scenario.InitializeSimulation()
  scenario.configure_initial_conditions()
  # Always show a progress bar for non-trivial runs so a long StepUntilStop
  # is not mistaken for a hang (GDB shows a semaphore wait inside the sim).
  scenario.SetProgressBar(show_progress or runTime >= 30.0)
  total_stop_nano = mc.sec2nano(runTime)

  # Chunked execution gives terminal heartbeats even without --livePlots.
  chunk_sec = live_interval if (live_plots or live_view) else min(30.0, max(5.0, runTime / 10.0))
  chunk_nano = max(mc.sec2nano(chunk_sec), mc.sec2nano(0.01))

  live_plotter = LivePlotter() if live_plots else None
  live_viewer = LiveMujocoView(scenario) if live_view else None
  wall_start = time.time()

  print(
      f"[ThrArmPaper] starting sim: runTime={runTime:.1f}s, chunk={chunk_sec:.1f}s"
      f" (plots/viz appear after the sim finishes)",
      flush=True,
  )

  try:
    next_stop = 0
    while next_stop < total_stop_nano:
      next_stop = min(next_stop + chunk_nano, total_stop_nano)
      scenario.ConfigureStopTime(next_stop)
      scenario.ExecuteSimulation()

      sim_t = scenario.TotalSim.CurrentNanos * mc.NANO2SEC
      if live_plotter is not None:
        live_plotter.update(scenario)
      if live_viewer is not None:
        if live_viewer.viewer is not None and not live_viewer.viewer.is_running():
          print("[ThrArmPaper] MuJoCo viewer closed; continuing simulation without live view.")
          live_viewer.close()
          live_viewer = None
        else:
          live_viewer.sync()

      wall_elapsed = time.time() - wall_start
      print(
          f"[ThrArmPaper] sim {sim_t:,.1f} s / {runTime:,.1f} s"
          f" | wall {wall_elapsed:.1f} s",
          flush=True,
      )
  finally:
    if live_viewer is not None:
      live_viewer.close()
    if live_plotter is not None:
      plt.ioff()
  print("[ThrArmPaper] simulation finished.", flush=True)


def run(
    showPlots: bool = False,
    visualize: bool = False,
    livePlots: bool = False,
    liveView: bool = False,
    liveInterval: float = 30.0,
    progress: bool = False,
    outerTimeStep: float = 10.0,
    innerTimeStep: float = 0.01,
    runTime: float = 320.0,
    tau_pre_limit_nm: float = None,
    vizSpeed: float = 50.0,
    fixed_offset: bool = False,
    align_los: bool = True,
    ic_offset_y: float = None,
    one_cycle: bool = False,
    phase_b: bool = False,
    phase_c: bool = False,
    phase_d: bool = False,
    log_ncon: bool = False,
    inspection_period: float = None,
    paper_speed: bool = False,
):
  global USE_FIXED_OFFSET, ALIGN_LOS_AT_INIT, FIXED_OFFSET_IC_DELTA_N
  global INSPECTION_PERIOD, PHASE_D_ACTIVE

  if phase_c:
    # Static mesh/bounds audit first (no Basilisk sim).
    print("[PhaseC] running static full-hub geometry audit…", flush=True)
    if path not in sys.path:
      sys.path.insert(0, path)
    import phaseC_geometry_audit as pca
    audit_rc = pca.audit(samples=4000, seed=0)
    if audit_rc != 0:
      print("[PhaseC] static audit FAILED — still running short in-sim ncon log.", flush=True)
    # Short commanded-θ / ncon probe under Phase B-like hover.
    phase_b = True
    log_ncon = True
    if runTime == 320.0:
      runTime = 60.0
    print(
        f"[PhaseC] in-sim ncon log: phaseB-style hover, runTime={runTime:.1f}s",
        flush=True,
    )

  if phase_d:
    # Moving ρ*(t): drop fixedOffset; use thrust-feasible sped-up arc unless paper-speed.
    fixed_offset = False
    one_cycle = False
    phase_b = False
    PHASE_D_ACTIVE = True
    if paper_speed:
      INSPECTION_PERIOD = INSPECTION_PERIOD_PAPER_S
      if runTime == 320.0:
        runTime = INSPECTION_PERIOD_PAPER_S
    elif inspection_period is not None:
      INSPECTION_PERIOD = float(inspection_period)
      # If user only set period and left default runTime, cover ~1/4 of that period.
      if runTime == 320.0:
        runTime = min(INSPECTION_PERIOD, max(600.0, 0.25 * INSPECTION_PERIOD))
    else:
      INSPECTION_PERIOD = INSPECTION_PERIOD_SPEEDUP_S
      if runTime == 320.0:
        runTime = INSPECTION_PERIOD_SPEEDUP_RUNTIME_S
    if abs(outerTimeStep - 10.0) < 1e-12:
      outerTimeStep = 5.0
    a_cent = INSPECTION_AMPLITUDE_YZ * (2.0 * np.pi / max(INSPECTION_PERIOD, 1e-9)) ** 2
    print(
        f"[PhaseD] moving inspection path: period={INSPECTION_PERIOD:.1f}s, "
        f"A_yz={INSPECTION_AMPLITUDE_YZ:.2f}m, a_cent≈{a_cent:.2e} m/s², "
        f"outer={outerTimeStep:.1f}s, runTime={runTime:.1f}s  (no --fixedOffset)",
        flush=True,
    )
  else:
    PHASE_D_ACTIVE = False
    if inspection_period is not None:
      INSPECTION_PERIOD = float(inspection_period)
    elif paper_speed:
      INSPECTION_PERIOD = INSPECTION_PERIOD_PAPER_S

  if phase_b:
    fixed_offset = True
    one_cycle = False
    # Prefer a shorter outer period and a 1 m IC unless the user overrode them.
    if abs(outerTimeStep - 10.0) < 1e-12:
      outerTimeStep = 5.0
    if ic_offset_y is None:
      FIXED_OFFSET_IC_DELTA_N = np.array([0.0, 1.0, 0.0])
    print(
        f"[PhaseB] hover mode: outer={outerTimeStep:.1f}s, "
        f"IC Δy={(ic_offset_y if ic_offset_y is not None else 1.0):.1f}m, "
        f"runTime={runTime:.1f}s",
        flush=True,
    )

  USE_FIXED_OFFSET = fixed_offset or one_cycle
  ALIGN_LOS_AT_INIT = align_los
  if ic_offset_y is not None:
    FIXED_OFFSET_IC_DELTA_N = np.array([0.0, float(ic_offset_y), 0.0])

  if one_cycle:
    # Enough time for allocate→settle→fire→coast plus observe Δr.
    runTime = min(runTime, 40.0) if runTime == 320.0 else runTime
    print(
        f"[PhaseA] one-cycle mode: fixedOffset IC, freeze after first coast, runTime={runTime:.1f}s",
        flush=True,
    )

  moving_path = bool(phase_d) or ((not USE_FIXED_OFFSET) and (not one_cycle) and PHASE_D_ACTIVE)

  scenario = scenarioThrArmPaper(
      outerRate=outerTimeStep,
      innerRate=innerTimeStep,
      dynRate=innerTimeStep,
      tau_pre_limit_nm=tau_pre_limit_nm,
      one_cycle=one_cycle,
      log_ncon=log_ncon or phase_c,
      moving_path=moving_path or phase_d,
      path_period_s=INSPECTION_PERIOD if (moving_path or phase_d) else None,
  )

  use_live = livePlots or liveView
  if use_live:
    runScenario(
        scenario,
        runTime,
        live_plots=livePlots,
        live_view=liveView,
        live_interval=liveInterval,
        show_progress=progress or runTime >= 600.0,
    )
  else:
    runScenario(
        scenario,
        runTime,
        show_progress=progress or runTime >= 600.0,
    )

  figureList = scenario.pull_outputs(showPlots)

  if showPlots:
    plt.show()

  if visualize:
    DynModel = scenario.get_DynModel()
    print(f"[ThrArmPaper] Opening post-run MuJoCo replay (speedUp={vizSpeed})...")
    mujoco.visualize(
        scenario.stateRec.times(),
        np.squeeze(scenario.stateRec.qpos),
        DynModel.scene,
        speedUp=vizSpeed,
        track="hub",
    )

  return figureList


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Run ThrArmPaper inspection scenario")
  parser.add_argument(
      "--showPlots",
      action="store_true",
      default=False,
      help="Display matplotlib figures.",
  )
  parser.add_argument(
      "--outerTimeStep",
      type=float,
      default=10.0,
      help="Outer loop guidance update period [s].",
  )
  parser.add_argument(
      "--innerTimeStep",
      type=float,
      default=0.01,
      help="Inner loop joint dynamics update period [s].",
  )
  parser.add_argument(
      "--runTime",
      type=float,
      default=320.0,
      help="Simulation run time [s].",
  )
  parser.add_argument(
      "--visualize",
      action="store_true",
      help="Open MuJoCo replay after simulation.",
  )
  parser.add_argument(
      "--livePlots",
      action="store_true",
      help="Update matplotlib plots during the simulation.",
  )
  parser.add_argument(
      "--liveView",
      action="store_true",
      help="Open a live MuJoCo 3D window during the simulation.",
  )
  parser.add_argument(
      "--liveInterval",
      type=float,
      default=30.0,
      help="Sim seconds between live plot/viewer updates [s].",
  )
  parser.add_argument(
      "--progress",
      action="store_true",
      help="Show terminal progress bar.",
  )
  parser.add_argument(
      "--vizSpeed",
      type=float,
      default=50.0,
      help="Post-run MuJoCo replay speed-up factor.",
  )
  parser.add_argument(
      "--tauPreLimit",
      type=float,
      default=None,
      help="Optional hub torque limit for jointMotionCompensator [N m].",
  )
  parser.add_argument(
      "--fixedOffset",
      action="store_true",
      help="Hold deputy reference fixed at RHO0 (IC starts 3 m off in +y so arms move).",
  )
  parser.add_argument(
      "--icOffsetY",
      type=float,
      default=None,
      help="With --fixedOffset, override initial y-offset from RHO0 [m] (default 3).",
  )
  parser.add_argument(
      "--noAlignLos",
      action="store_true",
      help="Use paper MRP IC instead of aligning +x body axis.",
  )
  parser.add_argument(
      "--oneCycle",
      action="store_true",
      help="Phase A: run one allocate→settle→fire→coast cycle then freeze FSW (implies --fixedOffset).",
  )
  parser.add_argument(
      "--phaseB",
      action="store_true",
      help="Phase B hover: fixedOffset, outer=5s, IC Δy=1m (override with --icOffsetY/--outerTimeStep).",
  )
  parser.add_argument(
      "--phaseC",
      action="store_true",
      help="Phase C: static full-hub geometry audit + short in-sim ncon vs commanded θ log.",
  )
  parser.add_argument(
      "--logNcon",
      action="store_true",
      help="Log MuJoCo ncon for every allocation commanded joint pose (Phase C probe).",
  )
  parser.add_argument(
      "--phaseD",
      action="store_true",
      help=(
          "Phase D: drop fixedOffset; track moving ρ*(t). "
          "Default sped-up period=3600s / runTime=900s (thrust-feasible). "
          "Use --paperSpeed for the 8 h arc."
      ),
  )
  parser.add_argument(
      "--paperSpeed",
      action="store_true",
      help="Use paper inspection period 28800 s (with --phaseD also sets runTime=28800 if default).",
  )
  parser.add_argument(
      "--inspectionPeriod",
      type=float,
      default=None,
      help="Override ρ*(t) period [s] (default paper 28800; Phase D sped-up 600).",
  )

  args = parser.parse_args()
  # Phase D must not imply fixedOffset even if other flags are set.
  fixed = args.fixedOffset or args.oneCycle or args.phaseB or args.phaseC
  if args.phaseD:
    fixed = False
  run(
      showPlots=args.showPlots,
      visualize=args.visualize,
      livePlots=args.livePlots,
      liveView=args.liveView,
      liveInterval=args.liveInterval,
      progress=args.progress,
      outerTimeStep=args.outerTimeStep,
      innerTimeStep=args.innerTimeStep,
      runTime=args.runTime,
      tau_pre_limit_nm=args.tauPreLimit,
      vizSpeed=args.vizSpeed,
      fixed_offset=fixed,
      align_los=not args.noAlignLos,
      ic_offset_y=args.icOffsetY,
      one_cycle=args.oneCycle,
      phase_b=(args.phaseB or args.phaseC) and not args.phaseD,
      phase_c=args.phaseC and not args.phaseD,
      phase_d=args.phaseD,
      log_ncon=args.logNcon or (args.phaseC and not args.phaseD),
      inspection_period=args.inspectionPeriod,
      paper_speed=args.paperSpeed,
  )