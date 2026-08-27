"""Post-process NASA 42 B2 ``*.42`` logs into Basilisk-compatible metrics and plots."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from b2_attitude_config import (
    B2Case,
    B2Config,
    SOLAR_CONSTANT_W_M2,
    wheel_momentum_limit_nms,
    wheel_torque_limit_nm,
)

LINE_COLORS = ("C0", "C1", "C2")
MU_EARTH = 3.986004418e14


def load_42_vector(path: Path, columns: int) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Missing NASA 42 log: {path}")
    data = np.loadtxt(path, dtype=float)
    if data.ndim == 1:
        if columns == 1:
            return data.reshape(-1, 1)
        return data.reshape(1, -1)
    if data.shape[1] != columns:
        raise ValueError(f"Expected {columns} columns in {path.name}, got {data.shape[1]}")
    return data


def q_to_dcm_bn(q: np.ndarray) -> np.ndarray:
    """NASA 42 quaternion [qx, qy, qz, qw] -> ``Q2C`` matrix.

    In NASA 42, `Q2C(qbn, CBN)` produces a DCM that maps inertial vectors
    expressed in `N` into body vectors expressed in `B` (i.e. N -> B).
    """
    qx, qy, qz, qw = q
    two_q00 = 2.0 * qx * qx
    two_q11 = 2.0 * qy * qy
    two_q22 = 2.0 * qz * qz
    two_q01 = 2.0 * qx * qy
    two_q02 = 2.0 * qx * qz
    two_q03 = 2.0 * qx * qw
    two_q12 = 2.0 * qy * qz
    two_q13 = 2.0 * qy * qw
    two_q23 = 2.0 * qz * qw
    return np.array(
        [
            [1.0 - (two_q11 + two_q22), two_q01 + two_q23, two_q02 - two_q13],
            [two_q01 - two_q23, 1.0 - (two_q22 + two_q00), two_q12 + two_q03],
            [two_q02 + two_q13, two_q12 - two_q03, 1.0 - (two_q00 + two_q11)],
        ],
        dtype=float,
    )


def dcm_to_q(dcm: np.ndarray) -> np.ndarray:
  tr = float(np.trace(dcm))
  if tr > 0.0:
    s = math.sqrt(tr + 1.0) * 2.0
    qw = 0.25 * s
    qx = (dcm[2, 1] - dcm[1, 2]) / s
    qy = (dcm[0, 2] - dcm[2, 0]) / s
    qz = (dcm[1, 0] - dcm[0, 1]) / s
  elif dcm[0, 0] > dcm[1, 1] and dcm[0, 0] > dcm[2, 2]:
    s = math.sqrt(1.0 + dcm[0, 0] - dcm[1, 1] - dcm[2, 2]) * 2.0
    qw = (dcm[2, 1] - dcm[1, 2]) / s
    qx = 0.25 * s
    qy = (dcm[0, 1] + dcm[1, 0]) / s
    qz = (dcm[0, 2] + dcm[2, 0]) / s
  elif dcm[1, 1] > dcm[2, 2]:
    s = math.sqrt(1.0 + dcm[1, 1] - dcm[0, 0] - dcm[2, 2]) * 2.0
    qw = (dcm[0, 2] - dcm[2, 0]) / s
    qx = (dcm[0, 1] + dcm[1, 0]) / s
    qy = 0.25 * s
    qz = (dcm[1, 2] + dcm[2, 1]) / s
  else:
    s = math.sqrt(1.0 + dcm[2, 2] - dcm[0, 0] - dcm[1, 1]) * 2.0
    qw = (dcm[1, 0] - dcm[0, 1]) / s
    qx = (dcm[0, 2] + dcm[2, 0]) / s
    qy = (dcm[1, 2] + dcm[2, 1]) / s
    qz = 0.25 * s
  q = np.array([qx, qy, qz, qw], dtype=float)
  return q / np.linalg.norm(q)


def q_multiply(q_left: np.ndarray, q_right: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q_left
    x2, y2, z2, w2 = q_right
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def q_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def q_to_mrp(q: np.ndarray) -> np.ndarray:
    q = q / np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    q0, q1, q2, q3 = q[3], q[0], q[1], q[2]
    den = 1.0 + q0
    if abs(den) < 1.0e-12:
        return np.array([0.0, 0.0, 0.0])
    return np.array([q1 / den, q2 / den, q3 / den], dtype=float)


def triad(n1: np.ndarray, n2: np.ndarray, b1: np.ndarray, b2: np.ndarray) -> np.ndarray:
    """Return C_RN (reference frame R expressed in inertial N), NASA 42 TRIAD convention."""
    n1 = n1 / np.linalg.norm(n1)
    n2 = n2 / np.linalg.norm(n2)
    b1 = b1 / np.linalg.norm(b1)
    b2 = b2 / np.linalg.norm(b2)

    n3 = np.cross(n1, n2)
    n3 /= np.linalg.norm(n3)
    n2 = np.cross(n3, n1)
    n2 /= np.linalg.norm(n2)

    b3 = np.cross(b1, b2)
    b3 /= np.linalg.norm(b3)
    b2 = np.cross(b3, b1)
    b2 /= np.linalg.norm(b2)

    c_nr = np.column_stack([n1, n2, n3]) @ np.column_stack([b1, b2, b3]).T
    return c_nr


def pointing_error_deg_from_attitude(
    qbn: np.ndarray,
    svn: np.ndarray,
    panel_axis_b: np.ndarray,
) -> np.ndarray:
    """Basilisk-compatible pointing: angle between body panel axis and Sun in N.

    Basilisk ``MRP2C`` returns ``C_BN`` and maps the panel axis with ``C_BN.T``.
  NASA 42 ``Q2C(qbn)`` maps body -> inertial, so the same geometry is
  ``Q2C(qbn) @ panel_hat_B``.
    """
    panel_hat_b = panel_axis_b / np.linalg.norm(panel_axis_b)
    errors = np.empty(len(qbn), dtype=float)
    for index, q in enumerate(qbn):
        c_nb = q_to_dcm_bn(q)
        # `c_nb` maps N->B, so invert it to express the body axis in N.
        panel_hat_n = c_nb.T @ panel_hat_b
        sun_hat_n = svn[index] / np.linalg.norm(svn[index])
        cos_angle = float(np.clip(np.dot(panel_hat_n / np.linalg.norm(panel_hat_n), sun_hat_n), -1.0, 1.0))
        errors[index] = math.degrees(math.acos(cos_angle))
    return errors


def pointing_error_deg_from_svb(svb: np.ndarray, panel_axis_b: np.ndarray) -> np.ndarray:
    """NASA 42 internal proxy: angle between body +Z and logged ``svb``.

    ``svb`` is computed in 42 as ``CN @ svn`` where ``CN = Q2C(qbn)``. This is
    what the GUI highlights and what PROTOTYPE_FSW drives toward zero, but it is
    not the same as Basilisk geometric sun-pointing (see comparison notes).
    """
    axis = panel_axis_b / np.linalg.norm(panel_axis_b)
    sv_unit = svb / np.linalg.norm(svb, axis=1, keepdims=True)
    cos_angle = np.clip(sv_unit @ axis, -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))


def sun_incidence_from_attitude(
    qbn: np.ndarray,
    svn: np.ndarray,
    panel_axis_b: np.ndarray,
) -> np.ndarray:
    panel_hat_b = panel_axis_b / np.linalg.norm(panel_axis_b)
    incidence = np.empty(len(qbn), dtype=float)
    for index, q in enumerate(qbn):
        c_nb = q_to_dcm_bn(q)
        # `c_nb` maps N->B, so invert it to express the body axis in N.
        panel_hat_n = c_nb.T @ panel_hat_b
        sun_hat_n = svn[index] / np.linalg.norm(svn[index])
        incidence[index] = float(np.clip(np.dot(panel_hat_n / np.linalg.norm(panel_hat_n), sun_hat_n), 0.0, 1.0))
    return incidence


def cylindrical_shadow_factor(pos_n: np.ndarray, sun_hat_n: np.ndarray, earth_radius_m: float = 6378137.0) -> np.ndarray:
    sun_hat = sun_hat_n / np.linalg.norm(sun_hat_n, axis=1, keepdims=True)
    along = np.sum(pos_n * sun_hat, axis=1)
    perp = pos_n - along[:, None] * sun_hat
    perp_dist = np.linalg.norm(perp, axis=1)
    return np.where((along < 0.0) & (perp_dist < earth_radius_m), 0.0, 1.0)


def load_nasa42_logs(log_dir: Path) -> dict[str, np.ndarray]:
    times_s = load_42_vector(log_dir / "time.42", 1).reshape(-1)
    qbn = load_42_vector(log_dir / "qbn.42", 4)
    wbn = load_42_vector(log_dir / "wbn.42", 3)
    pos_n = load_42_vector(log_dir / "PosN.42", 3)
    vel_n = load_42_vector(log_dir / "VelN.42", 3)
    svn = load_42_vector(log_dir / "svn.42", 3)
    svb = load_42_vector(log_dir / "svb.42", 3)

    logs: dict[str, np.ndarray] = {
        "times_s": times_s,
        "qbn": qbn,
        "wbn": wbn,
        "pos_n": pos_n,
        "vel_n": vel_n,
        "svn": svn,
        "svb": svb,
    }

    u_path = log_dir / "u00.42"
    if u_path.exists():
        logs["u00"] = load_42_vector(u_path, 6)

    illum_path = log_dir / "Illum.42"
    if illum_path.exists():
        illum = load_42_vector(illum_path, 1).reshape(-1)
        logs["illum"] = illum

    hwhl_path = log_dir / "Hwhl.42"
    if hwhl_path.exists():
        logs["hwhl"] = load_42_vector(hwhl_path, 4)

    return logs


def derive_timeseries(config: B2Config, case: B2Case, logs: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    times_s = logs["times_s"]
    qbn = logs["qbn"]
    wbn = logs["wbn"]
    pos_n = logs["pos_n"]
    vel_n = logs["vel_n"]
    svn = logs["svn"]
    svb = logs["svb"]

    sun_hat_n = svn / np.linalg.norm(svn, axis=1, keepdims=True)
    panel_axis = np.array(config.sun_pointing_axis_body, dtype=float)
    pointing_errors_deg = pointing_error_deg_from_attitude(qbn, svn, panel_axis)

    if "illum" in logs and np.any(logs["illum"] > 1.0e-6):
        shadow_factor = np.clip(logs["illum"], 0.0, 1.0)
    else:
        shadow_factor = cylindrical_shadow_factor(pos_n, sun_hat_n)

    cos_incidence = sun_incidence_from_attitude(qbn, svn, panel_axis)
    panel_power_w = (
        SOLAR_CONSTANT_W_M2
        * config.solar_panel_area_m2
        * config.solar_efficiency
        * cos_incidence
        * shadow_factor
    )
    bus_load_w = np.full(len(times_s), config.bus_load_w)
    net_power_w = panel_power_w - bus_load_w

    dt = np.diff(times_s, prepend=times_s[0])
    storage_j = np.empty(len(times_s), dtype=float)
    storage_j[0] = config.initial_stored_charge_j
    for index in range(1, len(times_s)):
        storage_j[index] = np.clip(
            storage_j[index - 1] + net_power_w[index] * dt[index],
            0.0,
            config.battery_capacity_j,
        )
    soc_percent = 100.0 * storage_j / config.battery_capacity_j

    primary_b = np.array(config.sun_pointing_axis_body, dtype=float)
    secondary_b = np.array(config.sun_secondary_axis_body, dtype=float)
    qrn = np.zeros((len(times_s), 4), dtype=float)
    wrn = np.zeros((len(times_s), 3), dtype=float)
    sigma_br = np.zeros((len(times_s), 3), dtype=float)
    omega_br_b = np.zeros((len(times_s), 3), dtype=float)

    for index in range(len(times_s)):
        vel = vel_n[index]
        vel_norm = np.linalg.norm(vel)
        if vel_norm < 1.0e-3:
            vel_hat = np.array([0.0, 1.0, 0.0])
        else:
            vel_hat = vel / vel_norm
        c_rn = triad(sun_hat_n[index], vel_hat, primary_b, secondary_b)
        q_rn = dcm_to_q(c_rn)
        qrn[index] = q_rn
        q_bn = qbn[index]
        q_br = q_multiply(q_bn, q_conjugate(q_rn))
        if q_br[3] < 0.0:
            q_br = -q_br
        sigma_br[index] = q_to_mrp(q_br)

        if index > 0:
            dt_i = max(times_s[index] - times_s[index - 1], 1.0e-6)
            q_prev = qrn[index - 1]
            q_curr = q_rn
            q_dot = (q_curr - q_prev) / dt_i
            q_w = 2.0 * q_multiply(q_dot, q_conjugate(q_curr))
            wrn[index] = q_w[0:3]

    omega_br_b = wbn - wrn

    if "u00" in logs:
        torque_nm = logs["u00"][:, 0:3].copy()
    else:
        torque_nm = np.zeros((len(times_s), 3))

    wheel_momentum = logs.get("hwhl")
    torque_limit = wheel_torque_limit_nm(case, config)
    if wheel_momentum is not None and wheel_momentum.size:
        wheel_saturation = np.any(np.abs(wheel_momentum) >= 0.99 * wheel_momentum_limit_nms(case, config), axis=1)
    else:
        wheel_saturation = np.abs(torque_nm).max(axis=1) >= 0.99 * torque_limit

    return {
        "times_s": times_s,
        "times_h": times_s / 3600.0,
        "soc_percent": soc_percent,
        "panel_power_w": panel_power_w,
        "net_power_w": net_power_w,
        "bus_load_w": bus_load_w,
        "pointing_errors_deg": pointing_errors_deg,
        "shadow_factor": shadow_factor,
        "sigma_br": sigma_br,
        "omega_br_b": omega_br_b,
        "torque_nm": torque_nm,
        "position_km": pos_n / 1000.0,
        "wheel_momentum_nms": wheel_momentum,
        "wheel_saturation": wheel_saturation,
    }


def compute_metrics(config: B2Config, case: B2Case, series: dict[str, np.ndarray]) -> dict[str, float]:
    times_s = series["times_s"]
    soc_percent = series["soc_percent"]
    panel_power_w = series["panel_power_w"]
    shadow_factor = series["shadow_factor"]
    pointing_errors_deg = series["pointing_errors_deg"]
    torque_nm = series["torque_nm"]
    wheel_momentum = series.get("wheel_momentum_nms")
    wheel_saturation = series["wheel_saturation"]

    sunlit = shadow_factor >= config.eclipse_shadow_threshold
    charging_time_s = float("nan")
    target_mask = soc_percent >= config.target_soc_percent
    if np.any(target_mask):
        charging_time_s = float(times_s[np.argmax(target_mask)])

    sunlit_pointing = pointing_errors_deg[sunlit] if np.any(sunlit) else pointing_errors_deg
    sunlit_panel = panel_power_w[sunlit] if np.any(sunlit) else panel_power_w
    control_rms = float(np.sqrt(np.mean(np.sum(torque_nm**2, axis=1))))

    wheel_saturation_fraction = float(np.mean(wheel_saturation)) if wheel_saturation.size else 0.0

    eclipse_fraction = float(np.mean(shadow_factor < config.eclipse_shadow_threshold))
    dt = np.diff(times_s, prepend=times_s[0])
    eclipse_duration_s = float(np.sum(dt * (shadow_factor < config.eclipse_shadow_threshold)))

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


def save_timeseries_csv(path: Path, series: dict[str, np.ndarray]) -> None:
    rows: list[dict[str, Any]] = []
    wheel_momentum = series.get("wheel_momentum_nms")
    for index, time_s in enumerate(series["times_s"]):
        row = {
            "time_s": float(time_s),
            "battery_soc_percent": float(series["soc_percent"][index]),
            "generated_power_w": float(series["panel_power_w"][index]),
            "bus_load_w": float(series["bus_load_w"][index]),
            "net_power_w": float(series["net_power_w"][index]),
            "pointing_error_deg": float(series["pointing_errors_deg"][index]),
            "shadow_factor": float(series["shadow_factor"][index]),
            "control_torque_x_nm": float(series["torque_nm"][index, 0]),
            "control_torque_y_nm": float(series["torque_nm"][index, 1]),
            "control_torque_z_nm": float(series["torque_nm"][index, 2]),
            "sigma_br_0": float(series["sigma_br"][index, 0]),
            "sigma_br_1": float(series["sigma_br"][index, 1]),
            "sigma_br_2": float(series["sigma_br"][index, 2]),
            "omega_br_0_rad_s": float(series["omega_br_b"][index, 0]),
            "omega_br_1_rad_s": float(series["omega_br_b"][index, 1]),
            "omega_br_2_rad_s": float(series["omega_br_b"][index, 2]),
            "r_bn_x_km": float(series["position_km"][index, 0]),
            "r_bn_y_km": float(series["position_km"][index, 1]),
            "r_bn_z_km": float(series["position_km"][index, 2]),
        }
        if wheel_momentum is not None and wheel_momentum.size:
            row["max_wheel_momentum_nms"] = float(np.max(np.abs(wheel_momentum[index])))
        else:
            row["max_wheel_momentum_nms"] = 0.0
        rows.append(row)

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_timeseries_plot(config: B2Config, case: B2Case, case_dir: Path, series: dict[str, np.ndarray]) -> Path:
    times_h = series["times_h"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axes[0].plot(times_h, series["soc_percent"], color="tab:green")
    axes[0].axhline(config.target_soc_percent, color="gray", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("Battery SoC [%]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times_h, series["panel_power_w"], label="Panel power (post-process proxy)", color="tab:orange")
    axes[1].plot(times_h, series["net_power_w"], label="Net power", color="tab:blue")
    axes[1].set_ylabel("Power [W]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times_h, series["pointing_errors_deg"], color="tab:red")
    axes[2].set_ylabel("Pointing error [deg]")
    axes[2].grid(True, alpha=0.3)

    in_eclipse = series["shadow_factor"] < config.eclipse_shadow_threshold
    axes[3].fill_between(times_h, 0.0, 1.0, where=in_eclipse, color="0.7", alpha=0.4, label="Eclipse")
    axes[3].set_ylabel("Eclipse flag")
    axes[3].set_xlabel("Time [h]")
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"NASA 42 B2 {case.value} — energy and GNC (post-processed)")
    fig.tight_layout()
    plot_path = case_dir / "b2_timeseries.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def save_gnc_plots(case: B2Case, case_dir: Path, series: dict[str, np.ndarray]) -> Path:
    times_h = series["times_h"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    for index in range(3):
        axes[0].plot(times_h, series["position_km"][:, index], color=LINE_COLORS[index], label=rf"$r_{{BN,{index}}}$")
    axes[0].set_ylabel("Inertial position [km]")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    for index in range(3):
        axes[1].plot(times_h, series["omega_br_b"][:, index], color=LINE_COLORS[index], label=rf"$\omega_{{BR,{index}}}$")
    axes[1].set_ylabel("Rate tracking error [rad/s]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    for index in range(3):
        axes[2].plot(times_h, series["torque_nm"][:, index], color=LINE_COLORS[index], label=rf"$L_{{r,{index}}}$")
    axes[2].set_ylabel("Control torque [Nm]")
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    for index in range(3):
        axes[3].plot(times_h, series["sigma_br"][:, index], color=LINE_COLORS[index], label=rf"$\sigma_{{{index}}}$")
    axes[3].set_ylabel(r"Attitude error $\sigma_{B/R}$")
    axes[3].set_xlabel("Time [h]")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"NASA 42 B2 {case.value} — ADCS (Basilisk-style layout)")
    fig.tight_layout()
    plot_path = case_dir / "b2_gnc_plots.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def postprocess_case(
    config: B2Config,
    case: B2Case,
    log_dir: Path,
    output_dir: Path,
) -> dict[str, Any]:
    logs = load_nasa42_logs(log_dir)
    series = derive_timeseries(config, case, logs)
    metrics = compute_metrics(config, case, series)

    output_dir.mkdir(parents=True, exist_ok=True)
    history_csv = output_dir / "b2_history.csv"
    save_timeseries_csv(history_csv, series)
    timeseries_plot = save_timeseries_plot(config, case, output_dir, series)
    gnc_plot = save_gnc_plots(case, output_dir, series)

    summary = {
        "benchmark": "B2",
        "simulator": "NASA42",
        "case_id": case.value,
        "run_id": f"b2_nasa42_{case.value}",
        "status": "pass" if metrics["battery_soc_final_percent"] > metrics["battery_soc_initial_percent"] else "analyzed",
        "milestone": 2,
        "metrics": metrics,
        "implementation_notes": {
            "post_processing": "Derived from native NASA 42 *.42 logs",
            "battery_soc": "Analytical proxy from CSS incidence + cylindrical eclipse (42 has no EPS model)",
            "pointing_error": "Geometric sun-pointing angle from qbn+svn (panel +Z vs Sun in inertial frame)",
            "reference_attitude": "TRIAD: +Z->SUN, +Y->VELOCITY (matches Inp_Cmd)",
            "control_torque": "u00.42 body torque channels (IdealTrq from PROTOTYPE_FSW)",
            "wheel_momentum": "Hwhl.42 (zeros when wheels not actuated)",
        },
        "artifacts": {
            "history_csv": str(history_csv),
            "timeseries_plot": str(timeseries_plot),
            "gnc_plots": str(gnc_plot),
        },
    }

    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary
