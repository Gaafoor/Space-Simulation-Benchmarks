#!/usr/bin/env python3
"""Benchmark B2 Attitude & GNC — Basilisk runner (B2A/B2B/B2C)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import shutil
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import Basilisk
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import locationPointing, mrpFeedback, rwMotorTorque
from Basilisk.simulation import (
    coarseSunSensor,
    eclipse,
    ephemerisConverter,
    reactionWheelStateEffector,
    simpleBattery,
    simpleNav,
    simplePowerSink,
    simpleSolarPanel,
    spacecraft,
)
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import (
    RigidBodyKinematics,
    SimulationBaseClass,
    fswSetupRW,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    simIncludeRW,
    unitTestSupport,
    vizSupport,
)
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

from b2_attitude_config import (
    B2Case,
    B2Config,
    case_uses_drag,
    case_uses_gravity_gradient,
    case_uses_sensor_errors,
    case_uses_srp,
    initial_misalignment_deg,
    initial_rates_deg_s,
    wheel_momentum_limit_nms,
    wheel_torque_limit_nm,
)

MU_EARTH = 3.986004418e14

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
RESULTS_DIR = SCRIPT_DIR / "results/basilisk_b2"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_b/b2_attitude_gnc/basilisk"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B2_METRICS_COMPARISON.csv"
VIZARD_INSTALL_DIR = WORKSPACE_ROOT / "tools/Vizard"
VIZARD_DOWNLOAD_URL = "https://hanspeterschaub.info/bskFiles/Vizard_Linux.zip"
DEFAULT_VIZ_DURATION_S = 2.0 * 3600.0

RW_PYRAMID_AXES = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 1.0, 1.0),
)


@dataclass(frozen=True)
class VizardOptions:
    """Basilisk Unity/Vizard visualization options."""

    enabled: bool = False
    live_stream: bool = False
    save_file: Path | None = None
    launch_after_run: bool = True
    sprite: str = "bskSat"


@dataclass(frozen=True)
class BasiliskB2ForceModelSpec:
    j2_enabled: bool
    drag_enabled: bool
    srp_enabled: bool
    gravity_gradient_enabled: bool
    sun_sensor_ideal: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "j2_enabled": self.j2_enabled,
            "drag_enabled": self.drag_enabled,
            "srp_enabled": self.srp_enabled,
            "gravity_gradient_enabled": self.gravity_gradient_enabled,
            "sun_sensor_ideal": self.sun_sensor_ideal,
        }


def force_model_spec(case: B2Case) -> BasiliskB2ForceModelSpec:
    return BasiliskB2ForceModelSpec(
        j2_enabled=True,
        drag_enabled=case_uses_drag(case),
        srp_enabled=case_uses_srp(case),
        gravity_gradient_enabled=case_uses_gravity_gradient(case),
        sun_sensor_ideal=not case_uses_sensor_errors(case),
    )


def load_spice_kernels() -> None:
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de430)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.naif0012)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de_403_masses)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.pck00010)))


def sun_hat_inertial(config: B2Config) -> np.ndarray:
    load_spice_kernels()
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(config.spice_epoch_utc, et)
    sun_state = pyswice.new_doubleArray(6)
    lt = pyswice.new_doubleArray(1)
    pyswice.spkezr_c("SUN", pyswice.doubleArray_getitem(et, 0), "J2000", "NONE", "EARTH", sun_state, lt)
    sun_pos = np.array([pyswice.doubleArray_getitem(sun_state, i) for i in range(3)], dtype=float)
    return sun_pos / np.linalg.norm(sun_pos)


def initial_state_from_elements(config: B2Config) -> tuple[np.ndarray, np.ndarray]:
    oe = orbitalMotion.ClassicElements()
    oe.a = config.semi_major_axis_m
    oe.e = config.eccentricity
    oe.i = math.radians(config.inclination_deg)
    oe.Omega = math.radians(config.raan_deg)
    oe.omega = math.radians(config.arg_perigee_deg)
    oe.f = math.radians(config.true_anomaly_deg)
    r0, v0 = orbitalMotion.elem2rv(MU_EARTH, oe)
    return np.array(r0, dtype=float), np.array(v0, dtype=float)


def sun_pointing_dcm(sun_hat_n: np.ndarray) -> np.ndarray:
    """Return ``C_NB``: body basis columns expressed in inertial frame (+Z -> Sun)."""
    z_n = np.asarray(sun_hat_n, dtype=float)
    z_n /= np.linalg.norm(z_n)
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(z_n, ref))) > 0.99:
        ref = np.array([0.0, 1.0, 0.0])
    y_n = np.cross(z_n, ref)
    y_n /= np.linalg.norm(y_n)
    x_n = np.cross(y_n, z_n)
    return np.column_stack([x_n, y_n, z_n])


def initial_sigma_bn(config: B2Config, case: B2Case, sun_hat_n: np.ndarray) -> np.ndarray:
    c_nb_align = sun_pointing_dcm(sun_hat_n)
    axis = np.asarray(config.misalignment_axis_body, dtype=float)
    axis /= np.linalg.norm(axis)
    angle_rad = math.radians(initial_misalignment_deg(case, config))
    c_mis = RigidBodyKinematics.PRV2C(axis * angle_rad)
    c_nb_init = c_nb_align @ c_mis
    # Basilisk ``sigma_BN`` parameterizes ``C_BN`` (inertial -> body), i.e. ``C_BN = C_NB.T``.
    return np.array(RigidBodyKinematics.C2MRP(c_nb_init.T), dtype=float).reshape(3)


def inertia_matrix(config: B2Config) -> list[float]:
    ixx, iyy, izz = config.inertia_kg_m2
    return [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]


def create_reaction_wheels(config: B2Config, case: B2Case) -> tuple[simIncludeRW.rwFactory, reactionWheelStateEffector.ReactionWheelStateEffector]:
    rw_factory = simIncludeRW.rwFactory()
    torque_limit = wheel_torque_limit_nm(case, config)

    for index, axis in enumerate(RW_PYRAMID_AXES, start=1):
        normalized_axis = list(np.asarray(axis, dtype=float) / np.linalg.norm(axis))
        wheel = rw_factory.create(
            "BCT_RWP015",
            normalized_axis,
            useRWfriction=False,
            useMaxTorque=True,
            label=f"RW{index}",
        )
        wheel.u_max = torque_limit

    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_effector.ModelTag = "B2RWCluster"
    return rw_factory, rw_effector


def pointing_error_deg(sigma_bn: np.ndarray, sun_hat_n: np.ndarray, panel_axis_b: np.ndarray) -> float:
    """Angle between body panel axis and Sun direction in inertial frame.

    Basilisk ``MRP2C(sigma_BN)`` returns ``C_BN`` (inertial N -> body B), so a body
    vector is mapped to inertial via ``C_BN.T`` (equivalently ``C_NB``).
    """
    dcm_bn = RigidBodyKinematics.MRP2C(np.asarray(sigma_bn, dtype=float).reshape(3))
    panel_hat_b = panel_axis_b / np.linalg.norm(panel_axis_b)
    panel_hat_n = dcm_bn.T @ panel_hat_b
    cos_angle = float(np.clip(np.dot(panel_hat_n, sun_hat_n), -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))


def default_vizard_bin_path(case: B2Case) -> Path:
    return DATA_DIR / case.value / f"b2_{case.value}_UnityViz.bin"


def find_vizard_binary() -> Path | None:
    env_path = os.environ.get("VIZARD_PATH")
    if env_path:
        candidate = Path(env_path).expanduser()
        if candidate.is_file():
            return candidate

    search_roots = [
        VIZARD_INSTALL_DIR,
        Path.home() / "Vizard",
        Path.home() / ".local/share/Vizard",
    ]
    names = ("Vizard.x86_64", "Vizard.x64", "Vizard")
    for root in search_roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.name in names:
                return path
    return None


def install_vizard() -> Path:
    """Download and unpack the Basilisk Vizard Linux binary into tools/Vizard."""
    import urllib.request
    import zipfile

    VIZARD_INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    archive_path = VIZARD_INSTALL_DIR / "Vizard_Linux.zip"
    if not archive_path.exists():
        print(f"Downloading Vizard from {VIZARD_DOWNLOAD_URL} ...")
        request = urllib.request.Request(
            VIZARD_DOWNLOAD_URL,
            headers={"User-Agent": "Mozilla/5.0 (Basilisk B2 benchmark setup)"},
        )
        with urllib.request.urlopen(request) as response, archive_path.open("wb") as handle:
            shutil.copyfileobj(response, handle)

    print(f"Extracting {archive_path} ...")
    with zipfile.ZipFile(archive_path, "r") as archive:
        archive.extractall(VIZARD_INSTALL_DIR)

    binary = find_vizard_binary()
    if binary is None:
        raise RuntimeError(f"Vizard binary not found under {VIZARD_INSTALL_DIR} after extraction")
    binary.chmod(binary.stat().st_mode | 0o111)
    print(f"Vizard installed: {binary}")
    return binary


def ensure_vizard_binary() -> Path:
    binary = find_vizard_binary()
    if binary is not None:
        return binary
    return install_vizard()


def launch_vizard(bin_path: Path, live_stream: bool = False) -> subprocess.Popen[bytes] | None:
    vizard_bin = ensure_vizard_binary()
    env = os.environ.copy()
    env.setdefault("GDK_BACKEND", "x11")
    command = [str(vizard_bin)]
    if live_stream:
        command.append("-liveStream")
    elif bin_path.exists():
        command.extend(["-loadFile", str(bin_path.resolve())])
    else:
        raise FileNotFoundError(f"Vizard recording not found: {bin_path}")

    print(f"Launching Vizard: {' '.join(command)}")
    return subprocess.Popen(command, env=env)


def attach_vizard_interface(
    sc_sim: SimulationBaseClass.SimBaseClass,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    rw_effector: reactionWheelStateEffector.ReactionWheelStateEffector,
    css: coarseSunSensor.CoarseSunSensor,
    vizard: VizardOptions,
) -> str | None:
    if not vizard.enabled:
        return None
    if not vizSupport.vizFound:
        raise RuntimeError("Basilisk was built without vizInterface support; cannot enable Vizard.")

    save_file = str(vizard.save_file) if vizard.save_file is not None else str(SCRIPT_DIR / "run_basilisk_b2_attitude_gnc.py")
    vizSupport.enableUnityVisualization(
        sc_sim,
        task_name,
        sc_object,
        saveFile=save_file if not vizard.live_stream else None,
        rwEffectorList=rw_effector,
        cssList=[[css]],
        spriteList=[vizard.sprite],
        liveStream=vizard.live_stream,
    )

    if vizard.live_stream:
        return None
    if vizard.save_file is not None and vizard.save_file.suffix.lower() == ".bin":
        return str(vizard.save_file)
    file_name = Path(save_file).stem
    file_path = Path(save_file).parent if Path(save_file).parent.as_posix() != "." else Path(".")
    return str(file_path / "_VizFiles" / f"{file_name}_UnityViz.bin")


def compute_metrics(
    config: B2Config,
    times_s: np.ndarray,
    storage_j: np.ndarray,
    panel_power_w: np.ndarray,
    shadow_factor: np.ndarray,
    pointing_errors_deg: np.ndarray,
    control_torque_nm: np.ndarray,
    wheel_momentum_nms: np.ndarray | None = None,
    wheel_momentum_limit_nms: float | None = None,
    wheel_torque_saturated: np.ndarray | None = None,
) -> dict[str, float]:
    capacity_j = config.battery_capacity_j
    soc_percent = 100.0 * storage_j / capacity_j
    sunlit = shadow_factor >= config.eclipse_shadow_threshold

    charging_time_s = float("nan")
    target_mask = soc_percent >= config.target_soc_percent
    if np.any(target_mask):
        charging_time_s = float(times_s[np.argmax(target_mask)])

    sunlit_pointing = pointing_errors_deg[sunlit] if np.any(sunlit) else pointing_errors_deg
    sunlit_panel = panel_power_w[sunlit] if np.any(sunlit) else panel_power_w

    control_rms = float(np.sqrt(np.mean(np.sum(control_torque_nm**2, axis=1))))

    wheel_saturation_fraction = 0.0
    if wheel_momentum_nms is not None and wheel_momentum_nms.size > 0:
        limit = wheel_momentum_limit_nms or config.wheel_momentum_limit_nms
        wheel_saturation_fraction = float(np.mean(np.abs(wheel_momentum_nms) >= 0.99 * limit))
    if wheel_torque_saturated is not None and wheel_torque_saturated.size > 0:
        wheel_saturation_fraction = max(wheel_saturation_fraction, float(np.mean(wheel_torque_saturated)))

    eclipse_fraction = float(np.mean(shadow_factor < config.eclipse_shadow_threshold))
    eclipse_duration_s = float(np.sum(np.diff(times_s, prepend=times_s[0]) * (shadow_factor < config.eclipse_shadow_threshold)))

    return {
        "battery_soc_initial_percent": float(soc_percent[0]),
        "battery_soc_final_percent": float(soc_percent[-1]),
        "battery_soc_delta_percent": float(soc_percent[-1] - soc_percent[0]),
        "charging_time_to_target_soc_s": charging_time_s,
        "generated_power_peak_w": float(np.max(panel_power_w)),
        "generated_power_mean_sunlit_w": float(np.mean(sunlit_panel)),
        "eclipse_duration_s": eclipse_duration_s,
        "eclipse_fraction": eclipse_fraction,
        "pointing_error_mean_sunlit_deg": float(np.mean(sunlit_pointing)),
        "pointing_error_max_sunlit_deg": float(np.max(sunlit_pointing)),
        "control_effort_rms_nm": control_rms,
        "wheel_saturation_fraction": wheel_saturation_fraction,
    }


def write_time_series_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_plots(
    config: B2Config,
    case_dir: Path,
    case: B2Case,
    times_h: np.ndarray,
    soc_percent: np.ndarray,
    panel_power_w: np.ndarray,
    net_power_w: np.ndarray,
    pointing_errors_deg: np.ndarray,
    shadow_factor: np.ndarray,
) -> dict[str, str]:
    artifacts: dict[str, str] = {}

    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axes[0].plot(times_h, soc_percent, color="tab:green")
    axes[0].axhline(config.target_soc_percent, color="gray", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Battery SoC [%]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times_h, panel_power_w, label="Panel power", color="tab:orange")
    axes[1].plot(times_h, net_power_w, label="Net power", color="tab:blue")
    axes[1].set_ylabel("Power [W]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times_h, pointing_errors_deg, color="tab:red")
    axes[2].set_ylabel("Pointing error [deg]")
    axes[2].grid(True, alpha=0.3)

    in_eclipse = shadow_factor < config.eclipse_shadow_threshold
    axes[3].fill_between(times_h, 0.0, 1.0, where=in_eclipse, color="0.7", alpha=0.4, label="Eclipse")
    axes[3].set_ylabel("Eclipse flag")
    axes[3].set_xlabel("Time [h]")
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"Basilisk B2 {case.value} — energy and GNC")
    fig.tight_layout()
    plot_path = case_dir / "b2_timeseries.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    artifacts["timeseries_plot"] = str(plot_path)
    return artifacts


def save_gnc_plots(
    case_dir: Path,
    case: B2Case,
    times_h: np.ndarray,
    sigma_br: np.ndarray,
    omega_br_b: np.ndarray,
    torque_nm: np.ndarray,
    position_km: np.ndarray,
) -> dict[str, str]:
    """Basilisk-style ADCS plots: position, rate error, torque, attitude error."""
    artifacts: dict[str, str] = {}
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    for index in range(3):
        axes[0].plot(
            times_h,
            position_km[:, index],
            color=unitTestSupport.getLineColor(index, 3),
            label=rf"$r_{{BN,{index}}}$",
        )
    axes[0].set_ylabel("Inertial position [km]")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    for index in range(3):
        axes[1].plot(
            times_h,
            omega_br_b[:, index],
            color=unitTestSupport.getLineColor(index, 3),
            label=rf"$\omega_{{BR,{index}}}$",
        )
    axes[1].set_ylabel("Rate tracking error [rad/s]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    for index in range(3):
        axes[2].plot(
            times_h,
            torque_nm[:, index],
            color=unitTestSupport.getLineColor(index, 3),
            label=rf"$L_{{r,{index}}}$",
        )
    axes[2].set_ylabel("Control torque [Nm]")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    for index in range(3):
        axes[3].plot(
            times_h,
            sigma_br[:, index],
            color=unitTestSupport.getLineColor(index, 3),
            label=rf"$\sigma_{{{index}}}$",
        )
    axes[3].set_ylabel(r"Attitude error $\sigma_{B/R}$")
    axes[3].set_xlabel("Time [h]")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"Basilisk B2 {case.value} — ADCS (Basilisk example style)")
    fig.tight_layout()
    plot_path = case_dir / "b2_gnc_plots.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    artifacts["gnc_plots"] = str(plot_path)
    return artifacts


def propagate_case(
    config: B2Config,
    case: B2Case,
    vizard: VizardOptions | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any], float, BasiliskB2ForceModelSpec]:
    if case != B2Case.B2A:
        raise NotImplementedError(f"Basilisk runner currently implements B2A only; requested {case.value}")

    spec = force_model_spec(case)
    sun_hat_n = sun_hat_inertial(config)
    sigma_init = initial_sigma_bn(config, case, sun_hat_n)
    rates_deg_s = initial_rates_deg_s(case, config)
    omega_init = np.deg2rad(np.asarray(rates_deg_s, dtype=float))

    sc_sim = SimulationBaseClass.SimBaseClass()
    step_ns = macros.sec2nano(config.basilisk_integration_step_s)
    sim_process = sc_sim.CreateNewProcess("b2Process")
    sim_task = sc_sim.CreateNewTask("b2Task", step_ns)
    sim_process.addTask(sim_task)
    task_name = "b2Task"

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "BenchmarkB2CubeSat"
    sc_object.hub.mHub = config.mass_kg
    sc_object.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(inertia_matrix(config))
    r0, v0 = initial_state_from_elements(config)
    sc_object.hub.r_CN_NInit = r0
    sc_object.hub.v_CN_NInit = v0
    sc_object.hub.sigma_BNInit = [[float(v)] for v in sigma_init]
    sc_object.hub.omega_BN_BInit = [[float(v)] for v in omega_init]

    grav_factory = simIncludeGravBody.gravBodyFactory()
    grav_bodies = grav_factory.createBodies(["sun", "earth"])
    earth = grav_bodies["earth"]
    earth.isCentralBody = True
    j2_path = get_path(getattr(DataFile.LocalGravData, "GGM03S_J2_only"))
    earth.useSphericalHarmonicsGravityModel(str(j2_path), config.basilisk_j2_max_degree)
    grav_factory.addBodiesTo(sc_object)

    load_spice_kernels()
    spice_object = grav_factory.createSpiceInterface(time=config.spice_epoch_utc, epochInMsg=True)
    spice_object.zeroBase = "earth"
    spice_object.referenceBase = "J2000"
    sun_msg = spice_object.planetStateOutMsgs[0]
    earth_msg = spice_object.planetStateOutMsgs[1]

    rw_factory, rw_effector = create_reaction_wheels(config, case)
    rw_factory.addToSpacecraft(sc_object.ModelTag, rw_effector, sc_object)

    s_nav = simpleNav.SimpleNav()
    s_nav.ModelTag = "B2SimpleNav"
    s_nav.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    s_nav.sunStateInMsg.subscribeTo(sun_msg)

    eclipse_object = eclipse.Eclipse()
    eclipse_object.ModelTag = "B2Eclipse"
    eclipse_object.sunInMsg.subscribeTo(sun_msg)
    eclipse_object.addPlanetToModel(earth_msg)
    eclipse_object.addSpacecraftToModel(sc_object.scStateOutMsg)

    css = coarseSunSensor.CoarseSunSensor()
    css.ModelTag = "B2SunSensor"
    css.stateInMsg.subscribeTo(sc_object.scStateOutMsg)
    css.sunInMsg.subscribeTo(sun_msg)
    css.sunEclipseInMsg.subscribeTo(eclipse_object.eclipseOutMsgs[0])
    css.nHat_B = list(config.sun_pointing_axis_body)
    css.fov = 90.0 * macros.D2R
    css.applySensorErrors = 0 if spec.sun_sensor_ideal else 1

    solar_panel = simpleSolarPanel.SimpleSolarPanel()
    solar_panel.ModelTag = "B2SolarPanel"
    solar_panel.stateInMsg.subscribeTo(sc_object.scStateOutMsg)
    solar_panel.sunInMsg.subscribeTo(sun_msg)
    solar_panel.sunEclipseInMsg.subscribeTo(eclipse_object.eclipseOutMsgs[0])
    solar_panel.setPanelParameters(
        list(config.sun_pointing_axis_body),
        config.solar_panel_area_m2,
        config.solar_efficiency,
    )

    power_sink = simplePowerSink.SimplePowerSink()
    power_sink.ModelTag = "B2BusLoad"
    power_sink.nodePowerOut = -config.bus_load_w

    battery = simpleBattery.SimpleBattery()
    battery.ModelTag = "B2Battery"
    battery.storageCapacity = config.battery_capacity_j
    battery.storedCharge_Init = config.initial_stored_charge_j
    battery.addPowerNodeToModel(solar_panel.nodePowerOutMsg)
    battery.addPowerNodeToModel(power_sink.nodePowerOutMsg)

    sun_ephem_converter = ephemerisConverter.EphemerisConverter()
    sun_ephem_converter.ModelTag = "B2SunEphemConverter"
    sun_ephem_converter.addSpiceInputMsg(sun_msg)

    sun_guidance = locationPointing.locationPointing()
    sun_guidance.ModelTag = "B2SunGuidance"
    sun_guidance.pHat_B = list(config.sun_pointing_axis_body)
    sun_guidance.cHat_B = list(config.sun_secondary_axis_body)
    sun_guidance.useBoresightRateDamping = 1 if config.use_boresight_rate_damping else 0
    sun_guidance.scTransInMsg.subscribeTo(s_nav.transOutMsg)
    sun_guidance.scAttInMsg.subscribeTo(s_nav.attOutMsg)
    sun_guidance.celBodyInMsg.subscribeTo(sun_ephem_converter.ephemOutMsgs[0])

    mrp_control = mrpFeedback.mrpFeedback()
    mrp_control.ModelTag = "B2MrpFeedback"
    # Use locationPointing attGuidOutMsg directly (Basilisk sun-pointing pattern).
    mrp_control.guidInMsg.subscribeTo(sun_guidance.attGuidOutMsg)
    mrp_control.K = config.mrp_k
    mrp_control.P = config.mrp_p
    mrp_control.Ki = config.mrp_ki
    if config.mrp_ki > 0:
        mrp_control.integralLimit = 2.0 / config.mrp_ki * 0.1

    vehicle_config = messaging.VehicleConfigMsgPayload()
    ixx, iyy, izz = config.inertia_kg_m2
    vehicle_config.ISCPntB_B = [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz]
    vehicle_config_msg = messaging.VehicleConfigMsg().write(vehicle_config)

    fswSetupRW.clearSetup()
    for rw in rw_factory.rwList.values():
        fswSetupRW.create(
            unitTestSupport.EigenVector3d2np(rw.gsHat_B),
            rw.Js,
            rw.u_max,
        )
    rw_config_msg = fswSetupRW.writeConfigMessage()

    mrp_control.vehConfigInMsg.subscribeTo(vehicle_config_msg)
    mrp_control.rwParamsInMsg.subscribeTo(rw_config_msg)
    mrp_control.rwSpeedsInMsg.subscribeTo(rw_effector.rwSpeedOutMsg)

    rw_motor_torque = rwMotorTorque.rwMotorTorque()
    rw_motor_torque.ModelTag = "B2RwMotorTorque"
    rw_motor_torque.vehControlInMsg.subscribeTo(mrp_control.cmdTorqueOutMsg)
    rw_motor_torque.rwParamsInMsg.subscribeTo(rw_config_msg)
    rw_motor_torque.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    sample_ns = macros.sec2nano(config.output_sample_s)
    sc_rec = sc_object.scStateOutMsg.recorder(sample_ns)
    att_rec = s_nav.attOutMsg.recorder(sample_ns)
    panel_rec = solar_panel.nodePowerOutMsg.recorder(sample_ns)
    sink_rec = power_sink.nodePowerOutMsg.recorder(sample_ns)
    battery_rec = battery.batPowerOutMsg.recorder(sample_ns)
    eclipse_rec = eclipse_object.eclipseOutMsgs[0].recorder(sample_ns)
    att_err_rec = sun_guidance.attGuidOutMsg.recorder(sample_ns)
    torque_rec = mrp_control.cmdTorqueOutMsg.recorder(sample_ns)
    sun_rec = sun_msg.recorder(sample_ns)
    rw_speed_rec = rw_effector.rwSpeedOutMsg.recorder(sample_ns)
    rw_recs = [rw_out.recorder(sample_ns) for rw_out in rw_effector.rwOutMsgs]

    sc_sim.AddModelToTask(task_name, spice_object)
    sc_sim.AddModelToTask(task_name, sun_ephem_converter)
    sc_sim.AddModelToTask(task_name, eclipse_object)
    sc_sim.AddModelToTask(task_name, rw_effector, 2)
    sc_sim.AddModelToTask(task_name, sc_object, 1)
    sc_sim.AddModelToTask(task_name, s_nav)
    sc_sim.AddModelToTask(task_name, css)
    sc_sim.AddModelToTask(task_name, solar_panel)
    sc_sim.AddModelToTask(task_name, power_sink)
    sc_sim.AddModelToTask(task_name, battery)
    sc_sim.AddModelToTask(task_name, sun_guidance)
    sc_sim.AddModelToTask(task_name, mrp_control)
    sc_sim.AddModelToTask(task_name, rw_motor_torque)

    for recorder in (
        sc_rec,
        att_rec,
        panel_rec,
        sink_rec,
        battery_rec,
        eclipse_rec,
        att_err_rec,
        torque_rec,
        sun_rec,
        rw_speed_rec,
        *rw_recs,
    ):
        sc_sim.AddModelToTask(task_name, recorder)

    rw_effector.rwMotorCmdInMsg.subscribeTo(rw_motor_torque.rwMotorTorqueOutMsg)

    viz_bin_path: str | None = None
    if vizard is not None and vizard.enabled:
        data_case_dir = DATA_DIR / case.value
        data_case_dir.mkdir(parents=True, exist_ok=True)
        viz_options = vizard
        if viz_options.save_file is None and not viz_options.live_stream:
            viz_options = VizardOptions(
                enabled=True,
                live_stream=vizard.live_stream,
                save_file=default_vizard_bin_path(case),
                launch_after_run=vizard.launch_after_run,
                sprite=vizard.sprite,
            )
        viz_bin_path = attach_vizard_interface(sc_sim, task_name, sc_object, rw_effector, css, viz_options)

    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(macros.sec2nano(config.simulated_duration_s))

    start_s = time.perf_counter()
    sc_sim.ExecuteSimulation()
    runtime_s = time.perf_counter() - start_s

    times_s = sc_rec.times() * macros.NANO2SEC
    sigma_bn = np.array(att_rec.sigma_BN)
    storage_j = np.array(battery_rec.storageLevel, dtype=float).reshape(-1)
    panel_power_w = np.array(panel_rec.netPower, dtype=float).reshape(-1)
    sink_power_w = np.array(sink_rec.netPower, dtype=float).reshape(-1)
    net_power_w = np.array(battery_rec.currentNetPower, dtype=float).reshape(-1)
    shadow_factor = np.array(eclipse_rec.shadowFactor, dtype=float).reshape(-1)
    torque_nm = np.array(torque_rec.torqueRequestBody, dtype=float)
    sigma_br = np.array(att_err_rec.sigma_BR, dtype=float)
    omega_br_b = np.array(att_err_rec.omega_BR_B, dtype=float)
    sc_positions = np.array(sc_rec.r_BN_N, dtype=float)
    sun_positions = np.array(sun_rec.PositionVector, dtype=float)

    rw_momenta = []
    wheel_speeds = np.array(rw_speed_rec.wheelSpeeds, dtype=float)
    if wheel_speeds.ndim == 1:
        wheel_speeds = wheel_speeds.reshape(-1, len(rw_recs))
    rw_labels = list(rw_factory.rwList.keys())
    for rw_index, label in enumerate(rw_labels):
        wheel_js = float(rw_factory.rwList[label].Js)
        rw_momenta.append(wheel_js * wheel_speeds[:, rw_index])
    wheel_momentum_nms = np.vstack(rw_momenta).T if rw_momenta else np.empty((len(times_s), 0))

    torque_limit = wheel_torque_limit_nm(case, config)
    rw_torque_saturation = []
    for rw_rec in rw_recs:
        applied_torque = np.array(rw_rec.u_current, dtype=float).reshape(-1)
        rw_torque_saturation.append(np.abs(applied_torque) >= 0.99 * torque_limit)
    torque_saturated = np.any(np.vstack(rw_torque_saturation), axis=0) if rw_torque_saturation else np.zeros(len(times_s), dtype=bool)

    panel_axis_b = np.array(config.sun_pointing_axis_body, dtype=float)
    pointing_errors = np.empty(len(times_s), dtype=float)
    for index in range(len(times_s)):
        sun_vector = sun_positions[index] - sc_positions[index]
        sun_vector /= np.linalg.norm(sun_vector)
        pointing_errors[index] = pointing_error_deg(sigma_bn[index], sun_vector, panel_axis_b)

    metrics = compute_metrics(
        config,
        times_s,
        storage_j,
        panel_power_w,
        shadow_factor,
        pointing_errors,
        torque_nm,
        wheel_momentum_nms,
        wheel_momentum_limit_nms(case, config),
        torque_saturated,
    )

    rows: list[dict[str, Any]] = []
    soc_percent = 100.0 * storage_j / config.battery_capacity_j
    for index, time_s in enumerate(times_s):
        rows.append(
            {
                "time_s": float(time_s),
                "battery_soc_percent": float(soc_percent[index]),
                "generated_power_w": float(panel_power_w[index]),
                "bus_load_w": float(-sink_power_w[index]),
                "net_power_w": float(net_power_w[index]),
                "pointing_error_deg": float(pointing_errors[index]),
                "shadow_factor": float(shadow_factor[index]),
                "control_torque_x_nm": float(torque_nm[index, 0]),
                "control_torque_y_nm": float(torque_nm[index, 1]),
                "control_torque_z_nm": float(torque_nm[index, 2]),
                "sigma_br_0": float(sigma_br[index, 0]),
                "sigma_br_1": float(sigma_br[index, 1]),
                "sigma_br_2": float(sigma_br[index, 2]),
                "omega_br_0_rad_s": float(omega_br_b[index, 0]),
                "omega_br_1_rad_s": float(omega_br_b[index, 1]),
                "omega_br_2_rad_s": float(omega_br_b[index, 2]),
                "r_bn_x_km": float(sc_positions[index, 0] / 1000.0),
                "r_bn_y_km": float(sc_positions[index, 1] / 1000.0),
                "r_bn_z_km": float(sc_positions[index, 2] / 1000.0),
                "max_wheel_momentum_nms": float(
                    np.max(np.abs(wheel_momentum_nms[index])) if wheel_momentum_nms.size else 0.0
                ),
            }
        )

    summary = {
        "benchmark": "B2",
        "simulator": "Basilisk",
        "case_id": case.value,
        "run_id": f"b2_basilisk_{case.value}",
        "status": "pass" if metrics["battery_soc_final_percent"] > metrics["battery_soc_initial_percent"] else "analyzed",
        "force_model": spec.to_dict(),
        "controller": {
            "type": "MRP feedback + locationPointing Sun guidance (attGuid direct)",
            "K": config.mrp_k,
            "P": config.mrp_p,
            "Ki": config.mrp_ki,
            "note": (
                "Basilisk-tuned gains frozen for Basilisk B2 runs; "
                f"scaled from I={config.mrp_gain_reference_inertia_kg_m2} kg·m² reference"
            ),
        },
        "implementation_notes": {
            "integration_step_s": config.basilisk_integration_step_s,
            "output_sample_s": config.output_sample_s,
            "reaction_wheel_model": "BCT_RWP015 with per-wheel u_max override to frozen torque limit",
            "guidance_module": (
                "locationPointing + ephemerisConverter(SPICE Sun); "
                f"cHat_B={config.sun_secondary_axis_body}; "
                f"useBoresightRateDamping={int(config.use_boresight_rate_damping)}"
            ),
            "mrp_gain_scaling": (
                f"K={config.mrp_k:.6f}, P={config.mrp_p:.6f} "
                f"(reference K={config.mrp_gain_reference_k}, P={config.mrp_gain_reference_p} "
                f"at I={config.mrp_gain_reference_inertia_kg_m2} kg·m²)"
            ),
            "sun_sensor": "coarseSunSensor ideal (applySensorErrors=0)",
            "attitude_error_note": (
                "sigma_BR[2] ~ 0 is expected: locationPointing embeds 2-DOF pointing error; "
                "sigma_0/sigma_1 are in-plane tilt, not body X/Y axes"
            ),
        },
        "metrics": metrics,
        "basilisk_version": Basilisk.__version__,
        "platform": platform.platform(),
        "runtime_s": {
            "simulation": runtime_s,
            "runtime_ratio": config.simulated_duration_s / runtime_s if runtime_s > 0 else None,
        },
    }
    if viz_bin_path is not None:
        summary["artifacts"] = {"vizard_bin": viz_bin_path}
    summary["_gnc_plot_series"] = {
        "times_h": (times_s / 3600.0).tolist(),
        "sigma_br": sigma_br.tolist(),
        "omega_br_b": omega_br_b.tolist(),
        "torque_nm": torque_nm.tolist(),
        "position_km": (sc_positions / 1000.0).tolist(),
    }
    return rows, summary, runtime_s, spec


def write_case_outputs(
    config: B2Config,
    case: B2Case,
    rows: list[dict[str, Any]],
    summary: dict[str, Any],
    runtime_s: float,
    spec: BasiliskB2ForceModelSpec,
) -> dict[str, Any]:
    case_dir = RESULTS_DIR / case.value
    data_case_dir = DATA_DIR / case.value
    case_dir.mkdir(parents=True, exist_ok=True)
    data_case_dir.mkdir(parents=True, exist_ok=True)

    history_csv = case_dir / "b2_history.csv"
    write_time_series_csv(history_csv, rows)
    write_time_series_csv(data_case_dir / "b2_history.csv", rows)

    times_h = np.array([row["time_s"] for row in rows]) / 3600.0
    plot_artifacts = save_plots(
        config,
        data_case_dir,
        case,
        times_h,
        np.array([row["battery_soc_percent"] for row in rows]),
        np.array([row["generated_power_w"] for row in rows]),
        np.array([row["net_power_w"] for row in rows]),
        np.array([row["pointing_error_deg"] for row in rows]),
        np.array([row["shadow_factor"] for row in rows]),
    )

    gnc_series = summary.pop("_gnc_plot_series", None)
    if gnc_series is not None:
        plot_artifacts.update(
            save_gnc_plots(
                data_case_dir,
                case,
                np.array(gnc_series["times_h"]),
                np.array(gnc_series["sigma_br"]),
                np.array(gnc_series["omega_br_b"]),
                np.array(gnc_series["torque_nm"]),
                np.array(gnc_series["position_km"]),
            )
        )

    artifacts = {
        "history_csv": str(history_csv),
        **plot_artifacts,
    }
    artifacts.update(summary.get("artifacts", {}))

    summary = {
        **summary,
        "runtime_s": {"simulation": runtime_s, "runtime_ratio": summary["runtime_s"]["runtime_ratio"]},
        "artifacts": artifacts,
    }

    with (case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    with (data_case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary


def merge_metrics_csv(summary: dict[str, Any], spec: BasiliskB2ForceModelSpec, runtime_s: float) -> None:
    metrics = summary["metrics"]
    row = {
        "simulator": "Basilisk",
        "run_id": summary["run_id"],
        "case_id": summary["case_id"],
        "battery_soc_initial_percent": metrics["battery_soc_initial_percent"],
        "battery_soc_final_percent": metrics["battery_soc_final_percent"],
        "battery_soc_delta_percent": metrics["battery_soc_delta_percent"],
        "charging_time_to_target_soc_s": metrics["charging_time_to_target_soc_s"],
        "generated_power_peak_w": metrics["generated_power_peak_w"],
        "generated_power_mean_sunlit_w": metrics["generated_power_mean_sunlit_w"],
        "eclipse_duration_s": metrics["eclipse_duration_s"],
        "eclipse_fraction": metrics["eclipse_fraction"],
        "pointing_error_mean_sunlit_deg": metrics["pointing_error_mean_sunlit_deg"],
        "pointing_error_max_sunlit_deg": metrics["pointing_error_max_sunlit_deg"],
        "control_effort_rms_nm": metrics["control_effort_rms_nm"],
        "wheel_saturation_fraction": metrics["wheel_saturation_fraction"],
        "wall_clock_runtime_s": runtime_s,
        "runtime_ratio": summary["runtime_s"]["runtime_ratio"],
        "drag_enabled": spec.drag_enabled,
        "srp_enabled": spec.srp_enabled,
        "gravity_gradient_enabled": spec.gravity_gradient_enabled,
    }

    fieldnames = list(row.keys())
    existing: list[dict[str, Any]] = []
    if METRICS_CSV.exists():
        with METRICS_CSV.open(newline="", encoding="utf-8") as handle:
            existing = [
                item
                for item in csv.DictReader(handle)
                if not (item.get("simulator") == "Basilisk" and item.get("case_id") == summary["case_id"])
            ]

    METRICS_CSV.parent.mkdir(parents=True, exist_ok=True)
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(existing)
        writer.writerow(row)


def write_run_log(path: Path, config: B2Config, summary: dict[str, Any], runtime_s: float) -> None:
    metrics = summary["metrics"]
    fm = summary["force_model"]
    ctrl = summary["controller"]
    lines = [
        f"# Basilisk B2 run log ({datetime.now(timezone.utc).strftime('%Y-%m-%d')})",
        "",
        "## Configuration",
        "",
        "- Parameter table: `data/benchmark_b/B2_PARAMETER_TABLE.md`",
        f"- Case: `{summary['case_id']}`",
        f"- Duration: {config.simulated_duration_s / 3600.0:.1f} h",
        f"- Basilisk version: {Basilisk.__version__}",
        "",
        "## Force model",
        "",
        f"- J2: {fm['j2_enabled']}",
        f"- Drag: {fm['drag_enabled']}",
        f"- SRP: {fm['srp_enabled']}",
        f"- Gravity-gradient: {fm['gravity_gradient_enabled']}",
        f"- Sun sensor ideal: {fm['sun_sensor_ideal']}",
        "",
        "## Controller (Basilisk-tuned, frozen for Basilisk)",
        "",
        f"- Type: {ctrl['type']}",
        f"- K: {ctrl['K']}",
        f"- P: {ctrl['P']}",
        f"- Ki: {ctrl['Ki']}",
        "",
        "## Basilisk implementation notes",
        "",
        f"- Integration step: {summary.get('implementation_notes', {}).get('integration_step_s', 'n/a')} s",
        f"- Output sample cadence: {summary.get('implementation_notes', {}).get('output_sample_s', 'n/a')} s",
        f"- Reaction wheels: {summary.get('implementation_notes', {}).get('reaction_wheel_model', 'n/a')}",
        f"- Guidance: {summary.get('implementation_notes', {}).get('guidance_module', 'n/a')}",
        f"- Sun sensor: {summary.get('implementation_notes', {}).get('sun_sensor', 'n/a')}",
        "",
        "## Primary metrics",
        "",
        f"- Initial SoC: {metrics['battery_soc_initial_percent']:.2f} %",
        f"- Final SoC: {metrics['battery_soc_final_percent']:.2f} %",
        f"- ΔSoC: {metrics['battery_soc_delta_percent']:.2f} %",
        f"- Time to {config.target_soc_percent:.0f}% SoC: {metrics['charging_time_to_target_soc_s']}",
        f"- Mean sunlit pointing error: {metrics['pointing_error_mean_sunlit_deg']:.3f} deg",
        f"- Eclipse fraction: {metrics['eclipse_fraction']:.3f}",
        f"- Control effort RMS: {metrics['control_effort_rms_nm']:.6f} Nm",
        "",
        "## Runtime",
        "",
        f"- Simulation wall clock: {runtime_s:.3f} s",
        f"- Runtime ratio: {summary['runtime_s']['runtime_ratio']:.1f}",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def parse_case(value: str) -> B2Case:
    normalized = value.lower().strip()
    for case in B2Case:
        if normalized in {case.value, case.name.lower()}:
            return case
    raise argparse.ArgumentTypeError(f"Unknown B2 case: {value}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Benchmark B2 Attitude & GNC in Basilisk")
    parser.add_argument("--case", type=parse_case, default=B2Case.B2A, help="B2 case id (b2a, b2b, b2c)")
    parser.add_argument(
        "--duration-s",
        type=float,
        default=None,
        help="Override simulated duration in seconds (default: frozen 24 h; with --vizard defaults to 2 h)",
    )
    parser.add_argument(
        "--vizard",
        action="store_true",
        help="Record Basilisk Unity/Vizard telemetry (.bin) and launch Vizard",
    )
    parser.add_argument(
        "--vizard-live",
        action="store_true",
        help="Stream simulation live to a running Vizard instance (launch Vizard with -liveStream first)",
    )
    parser.add_argument(
        "--vizard-file",
        type=Path,
        default=None,
        help="Explicit path for the Vizard .bin recording",
    )
    parser.add_argument(
        "--no-vizard-launch",
        action="store_true",
        help="Only write the Vizard .bin file; do not start the Vizard application",
    )
    parser.add_argument(
        "--install-vizard",
        action="store_true",
        help="Download/install the Vizard Linux binary into tools/Vizard and exit",
    )
    args = parser.parse_args()

    if args.install_vizard:
        binary = install_vizard()
        print(f"Vizard ready at: {binary}")
        return

    duration_s = args.duration_s
    if args.vizard or args.vizard_live:
        if duration_s is None:
            duration_s = DEFAULT_VIZ_DURATION_S
            print(f"Vizard mode: using {duration_s / 3600.0:.1f} h duration (pass --duration-s to override)")

    config = B2Config()
    if duration_s is not None:
        config = B2Config(simulated_duration_s=float(duration_s))

    vizard = VizardOptions(
        enabled=args.vizard or args.vizard_live,
        live_stream=args.vizard_live,
        save_file=args.vizard_file,
        launch_after_run=not args.no_vizard_launch,
    )

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    if vizard.enabled and vizard.live_stream and vizard.launch_after_run:
        ensure_vizard_binary()
        launch_vizard(default_vizard_bin_path(args.case), live_stream=True)
        time.sleep(3.0)

    print(f"Running Basilisk B2 {args.case.value}...")
    rows, summary, runtime_s, spec = propagate_case(config, args.case, vizard=vizard if vizard.enabled else None)
    summary = write_case_outputs(config, args.case, rows, summary, runtime_s, spec)
    merge_metrics_csv(summary, spec, runtime_s)
    write_run_log(DATA_DIR / "basilisk_b2_run_log.md", config, summary, runtime_s)

    metrics = summary["metrics"]
    print(f"Basilisk B2 {args.case.value} complete.")
    print(f"  Initial SoC: {metrics['battery_soc_initial_percent']:.2f} %")
    print(f"  Final SoC:   {metrics['battery_soc_final_percent']:.2f} %")
    print(f"  ΔSoC:        {metrics['battery_soc_delta_percent']:.2f} %")
    print(f"  Runtime:     {runtime_s:.3f} s")
    print(f"  Outputs:     {DATA_DIR / args.case.value}")

    if vizard.enabled and not vizard.live_stream:
        viz_bin = summary.get("artifacts", {}).get("vizard_bin")
        if viz_bin:
            print(f"  Vizard file: {viz_bin}")
            if vizard.launch_after_run:
                launch_vizard(Path(viz_bin))


if __name__ == "__main__":
    main()
