#!/usr/bin/env python3
"""Six-panel Basilisk vs NASA 42 comparison from recorded run outputs."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.utilities import orbitalMotion

from a3_leo_geo_config import MU_EARTH

THRUSTER_AXIS_BODY = np.array([1.0, 0.0, 0.0])
BSK_COLOR = "#d95f02"
N42_COLOR = "#1b78b4"
GMAT_COLOR = "#9467bd"
GEO_SMA_KM = 42164.0


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def read_scalar_file(path: Path) -> np.ndarray:
    vals: list[float] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            vals.append(float(line.split()[0]))
        except ValueError:
            continue
    if not vals:
        raise RuntimeError(f"No numeric rows parsed from {path}")
    return np.asarray(vals, dtype=float)


def read_vector_file(path: Path, cols: int) -> np.ndarray:
    rows: list[list[float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        try:
            vals = [float(p) for p in parts[:cols]]
        except ValueError:
            continue
        if len(vals) == cols:
            rows.append(vals)
    if not rows:
        raise RuntimeError(f"No numeric rows parsed from {path}")
    return np.asarray(rows, dtype=float)


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    x, y, z, s = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - s * z), 2 * (x * z + s * y)],
            [2 * (x * y + s * z), 1 - 2 * (x * x + z * z), 2 * (y * z - s * x)],
            [2 * (x * z - s * y), 2 * (y * z + s * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def load_csv_dicts(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def parse_thrust_level_cmd_schedule(cmd_path: Path) -> list[tuple[float, float]]:
    """Return sorted (time_s, ThrustLevelCmd) breakpoints from Inp_Cmd.txt."""
    pat = re.compile(
        r"^\s*([0-9.eE+\-]+)\s+SC\[0\]\.AC\.Thr\[0\]\.ThrustLevelCmd\s*=\s*([0-9.eE+\-]+)"
    )
    points: list[tuple[float, float]] = []
    for line in cmd_path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = pat.match(line.strip())
        if m:
            points.append((float(m.group(1)), float(m.group(2))))
    if not points:
        raise RuntimeError(f"No ThrustLevelCmd entries found in {cmd_path}")
    points.sort(key=lambda x: x[0])
    return points


def thrust_level_cmd_at(times: np.ndarray, schedule: list[tuple[float, float]]) -> np.ndarray:
    out = np.zeros_like(times, dtype=float)
    for i, t in enumerate(times):
        level = schedule[0][1]
        for t_break, cmd in schedule:
            if t + 1e-9 >= t_break:
                level = cmd
            else:
                break
        out[i] = level
    return out


def basilisk_thrusting_phase_flag(times: np.ndarray, spiral_start_s: float, spiral_end_s: float) -> np.ndarray:
    return ((times + 1e-9 >= spiral_start_s) & (times <= spiral_end_s + 1e-9)).astype(float)


def interp_to_grid(x_src: np.ndarray, y_src: np.ndarray, x_dst: np.ndarray) -> np.ndarray:
    if x_src.size == 0:
        return np.full_like(x_dst, np.nan, dtype=float)
    order = np.argsort(x_src)
    x = x_src[order]
    y = y_src[order]
    uniq_x, idx = np.unique(x, return_index=True)
    y = y[idx]
    return np.interp(x_dst, uniq_x, y, left=np.nan, right=np.nan)


def load_basilisk(bsk_dir: Path, schedule: dict) -> dict[str, np.ndarray]:
    thrust_rows = load_csv_dicts(bsk_dir / "thrust_history.csv")
    pointing_rows = load_csv_dicts(bsk_dir / "pointing_error_history.csv")

    thrust_t = np.array([float(r["time_s"]) for r in thrust_rows], dtype=float)
    thrust_n = np.array([float(r["thrust_n"]) for r in thrust_rows], dtype=float)
    mass = np.array([float(r["mass_kg"]) for r in thrust_rows], dtype=float)
    power_kw = np.array([float(r["power_kw"]) for r in thrust_rows], dtype=float)
    eclipse_factor = np.array([float(r["eclipse_factor"]) for r in thrust_rows], dtype=float)

    point_t = np.array([float(r["time_s"]) for r in pointing_rows], dtype=float)
    point_deg = np.array([float(r["pointing_error_deg"]) for r in pointing_rows], dtype=float)

    propulsion = schedule.get("propulsion", {})
    initial_mass = float(propulsion.get("initial_mass_kg", 17.0))
    burns = schedule.get("burns", [])
    spiral = next(b for b in burns if b.get("id") == "spiral_to_geo")
    spiral_start = float(spiral["start_s"])
    spiral_end = float(spiral.get("gmat_end_s", spiral.get("end_s", thrust_t[-1])))

    # Full-mission mass: constant before first thrust sample, thrust samples during spiral, hold final after.
    mass_t = thrust_t.copy()
    mass_vals = mass.copy()
    if thrust_t.size and thrust_t[0] > spiral_start + 1e-6:
        mass_t_full = np.concatenate(([spiral_start], mass_t))
        mass_vals_full = np.concatenate(([initial_mass], mass_vals))
    else:
        mass_t_full = mass_t
        mass_vals_full = mass_vals

    return {
        "thrust_t": thrust_t,
        "thrust_n": thrust_n,
        "mass_t": mass_t,
        "mass_kg": mass_vals,
        "mass_t_full": mass_t_full,
        "mass_kg_full": mass_vals_full,
        "power_t": thrust_t,
        "power_kw": power_kw,
        "eclipse_t": thrust_t,
        "eclipse_factor": eclipse_factor,
        "point_t": point_t,
        "point_deg": point_deg,
        "spiral_start_s": spiral_start,
        "spiral_end_s": spiral_end,
        "initial_mass_kg": initial_mass,
    }


def load_nasa42(run_dir: Path, cmd_path: Path) -> dict[str, np.ndarray]:
    t = read_scalar_file(run_dir / "time.42")
    pos = read_vector_file(run_dir / "PosN.42", 3)
    vel = read_vector_file(run_dir / "VelN.42", 3)
    q = read_vector_file(run_dir / "qbn.42", 4)
    thr = read_scalar_file(run_dir / "Thr.42")
    mass = read_scalar_file(run_dir / "Mass.42")
    power = read_scalar_file(run_dir / "Power.42")
    eclipse = read_scalar_file(run_dir / "Eclipse.42")

    n = min(len(t), len(pos), len(vel), len(q), len(thr), len(mass), len(power), len(eclipse))
    t = t[:n]
    pos = pos[:n]
    vel = vel[:n]
    q = q[:n]
    thr = thr[:n]
    mass = mass[:n]
    power = power[:n]
    eclipse = eclipse[:n]

    cmd_schedule = parse_thrust_level_cmd_schedule(cmd_path)
    thrust_level_cmd = thrust_level_cmd_at(t, cmd_schedule)

    point_deg = np.full(n, np.nan, dtype=float)
    for i in range(n):
        cbn = quat_to_dcm(q[i])
        vel_n = vel[i]
        thrust_n = cbn @ THRUSTER_AXIS_BODY
        if np.linalg.norm(vel_n) > 1.0:
            point_deg[i] = angle_deg(thrust_n, vel_n)

    return {
        "t": t,
        "pos_m": pos,
        "vel_m_s": vel,
        "thrust_level_cmd": thrust_level_cmd,
        "thrust_n": thr,
        "mass_kg": mass,
        "power_kw": power,
        "eclipse": eclipse,
        "point_deg": point_deg,
    }


def sma_km_from_rv(r_m: np.ndarray, v_m_s: np.ndarray) -> np.ndarray:
    """Osculating semi-major axis [km] for N inertial state samples."""
    sma = np.full(r_m.shape[0], np.nan, dtype=float)
    for i in range(r_m.shape[0]):
        if not (np.all(np.isfinite(r_m[i])) and np.all(np.isfinite(v_m_s[i]))):
            continue
        sma[i] = float(orbitalMotion.rv2elem(MU_EARTH, r_m[i], v_m_s[i]).a) / 1000.0
    return sma


def load_basilisk_sma(bsk_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = load_csv_dicts(bsk_dir / "orbit_elements_history.csv")
    if not rows:
        raise RuntimeError(f"No orbit elements in {bsk_dir / 'orbit_elements_history.csv'}")
    t = np.array([float(r["time_s"]) for r in rows], dtype=float)
    sma = np.array([float(r["sma_km"]) for r in rows], dtype=float)
    return t, sma


def load_gmat_sma_from_spline(spline_path: Path, schedule: dict) -> tuple[np.ndarray, np.ndarray]:
    """GMAT reference SMA from the exported SPLINE state file (full mission coverage)."""
    from datetime import datetime, timezone

    epoch = datetime.fromisoformat(schedule["mission_epoch_utc_iso"].replace("Z", "+00:00")).astimezone(
        timezone.utc
    )
    pat = re.compile(
        r"(\d{4}):(\d{2}):(\d{2}):(\d{2}):(\d{2}):(\d{2})\.(\d+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)"
    )
    times: list[float] = []
    pos_km: list[list[float]] = []
    vel_km_s: list[list[float]] = []
    for line in spline_path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = pat.match(line.strip())
        if not m:
            continue
        y, mo, d, h, mi, s, frac, x, yk, z, vx, vy, vz = m.groups()
        usec = int(frac[:6].ljust(6, "0"))
        t_utc = datetime(
            int(y), int(mo), int(d), int(h), int(mi), int(s), usec, tzinfo=timezone.utc
        )
        times.append((t_utc - epoch).total_seconds())
        pos_km.append([float(x), float(yk), float(z)])
        vel_km_s.append([float(vx), float(vy), float(vz)])
    if not times:
        raise RuntimeError(f"No spline states parsed from {spline_path}")
    t = np.asarray(times, dtype=float)
    r_m = 1000.0 * np.asarray(pos_km, dtype=float)
    v_m_s = 1000.0 * np.asarray(vel_km_s, dtype=float)
    return t, sma_km_from_rv(r_m, v_m_s)


def load_gmat_sma(schedule: dict, *, spline_path: Path | None = None) -> tuple[np.ndarray, np.ndarray]:
    if spline_path is not None and spline_path.is_file():
        return load_gmat_sma_from_spline(spline_path, schedule)
    ref = schedule.get("reference_ephemeris", {})
    bsp_path = Path(ref.get("path", ""))
    if not bsp_path.is_file():
        raise RuntimeError("GMAT reference requires --gmat-spline or a valid BSP path in the schedule.")
    from ephemeris_handoff import SpiceKernelSession, SpiceReferenceEphemeris, read_spice_state_at

    gmat_ref = schedule.get("gmat_reference", {})
    mission_end = float(gmat_ref.get("mission_end_s", 4_592_697.0))
    min_t = float(ref.get("bsp_coverage_start_s", 0.0))
    times = np.arange(min_t, mission_end + 3600.0, 3600.0, dtype=float)
    sma = np.full(times.shape, np.nan, dtype=float)
    spice_ref = SpiceReferenceEphemeris.from_metadata(ref)
    with SpiceKernelSession(bsp_path):
        for i, t in enumerate(times):
            try:
                r_ref, v_ref = read_spice_state_at(spice_ref, float(t))
                sma[i] = float(orbitalMotion.rv2elem(MU_EARTH, r_ref, v_ref).a) / 1000.0
            except Exception:
                continue
    return times, sma


def make_sma_compare_plot(
    bsk_dir: Path,
    n42_dir: Path,
    schedule: dict,
    out_path: Path,
    *,
    gmat_spline: Path | None = None,
    label_bsk: str = "Basilisk",
    label_n42: str = "NASA 42",
    label_gmat: str = "GMAT reference",
) -> None:
    """Semi-major axis vs mission time for Basilisk, NASA 42, and GMAT BSP."""
    bsk_t, bsk_sma = load_basilisk_sma(bsk_dir)
    n42_t = read_scalar_file(n42_dir / "time.42")
    n42_pos = read_vector_file(n42_dir / "PosN.42", 3)
    n42_vel = read_vector_file(n42_dir / "VelN.42", 3)
    n = min(len(n42_t), len(n42_pos), len(n42_vel))
    n42_sma = sma_km_from_rv(n42_pos[:n], n42_vel[:n])
    gmat_t, gmat_sma = load_gmat_sma(schedule, spline_path=gmat_spline)

    burns = schedule.get("burns", [])
    spiral = next(b for b in burns if b.get("id") == "spiral_to_geo")
    spiral_start = float(spiral["start_s"])
    spiral_end = float(spiral.get("gmat_end_s", spiral.get("end_s", spiral_start)))

    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    ax.plot(bsk_t / 86400.0, bsk_sma, color=BSK_COLOR, lw=1.0, label=label_bsk)
    ax.plot(n42_t[:n] / 86400.0, n42_sma, color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(gmat_t / 86400.0, gmat_sma, color=GMAT_COLOR, lw=1.2, ls="--", label=label_gmat)
    ax.axhline(GEO_SMA_KM, color="green", ls=":", lw=1.0, label="GEO SMA target")
    ax.axvspan(spiral_start / 86400.0, spiral_end / 86400.0, color="#f4b6b6", alpha=0.2, label="Exported spiral window")
    ax.set_xlabel("Mission time [days]")
    ax.set_ylabel("Semi-major axis [km]")
    ax.set_title("A3 electric-spiral semi-major axis: execution vs GMAT reference")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def _style_axis(ax: plt.Axes, title: str, ylabel: str, *, xlim_days: float | None = None) -> None:
    ax.set_title(title, fontsize=10)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Mission elapsed time [days]")
    ax.grid(True, alpha=0.3)
    if xlim_days is not None:
        ax.set_xlim(0.0, xlim_days)


def make_compare_plots(
    bsk: dict[str, np.ndarray],
    n42: dict[str, np.ndarray],
    out_dir: Path,
    *,
    label_bsk: str,
    label_n42: str,
    zoom_span_days: float = 7.0,
    zoom_pad_days: float = 0.1,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    t42 = n42["t"]
    days42 = t42 / 86400.0
    max_days = max(days42[-1], bsk["point_t"][-1] / 86400.0 if bsk["point_t"].size else 0.0)

    bsk_phase = basilisk_thrusting_phase_flag(bsk["thrust_t"], bsk["spiral_start_s"], bsk["spiral_end_s"])
    bsk_thrust_on = (bsk["thrust_n"] > 1e-6).astype(float)

    bsk_phase_h = interp_to_grid(bsk["thrust_t"], bsk_phase, t42)
    bsk_thrust_on_h = interp_to_grid(bsk["thrust_t"], bsk_thrust_on, t42)
    bsk_power_h = interp_to_grid(bsk["power_t"], bsk["power_kw"], t42)
    bsk_thrust_h = interp_to_grid(bsk["thrust_t"], bsk["thrust_n"], t42)
    bsk_eclipse_h = interp_to_grid(bsk["eclipse_t"], bsk["eclipse_factor"], t42)
    bsk_mass_h = interp_to_grid(bsk["mass_t_full"], bsk["mass_kg_full"], t42)
    bsk_point_h = interp_to_grid(bsk["point_t"], bsk["point_deg"], t42)

    bsk_fuel_h = bsk["initial_mass_kg"] - bsk_mass_h
    n42_fuel_h = n42["mass_kg"][0] - n42["mass_kg"]

    fig, axes = plt.subplots(6, 1, figsize=(12, 18), sharex=True, constrained_layout=True)
    fig.suptitle(
        "Basilisk vs NASA 42 — comparable propulsion / eclipse / pointing channels",
        fontsize=13,
        y=1.01,
    )

    ax = axes[0]
    ax.step(days42, n42["thrust_level_cmd"], where="post", color=N42_COLOR, lw=1.4, label=f"{label_n42}: ThrustLevelCmd")
    ax.step(
        days42,
        bsk_phase_h,
        where="post",
        color=BSK_COLOR,
        lw=1.2,
        ls="--",
        label=f"{label_bsk}: thrusting phase flag (schedule)",
    )
    ax.plot(
        days42,
        bsk_thrust_on_h,
        color=BSK_COLOR,
        lw=0.9,
        alpha=0.65,
        label=f"{label_bsk}: thrust_n > 0 (hourly interp)",
    )
    ax.set_ylim(-0.05, 1.15)
    _style_axis(ax, "1) Thrust ON window", "Command / phase [0–1]", xlim_days=max_days)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[1]
    ax.plot(days42, n42["power_kw"], color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(days42, bsk_power_h, color=BSK_COLOR, lw=1.0, alpha=0.9, label=f"{label_bsk} (hourly interp)")
    _style_axis(ax, "2) Available propulsion power", "Power [kW]", xlim_days=max_days)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[2]
    ax.step(days42, 1.0 - n42["eclipse"], where="post", color=N42_COLOR, lw=1.2, label=f"{label_n42}: sunlit (1−Eclipse)")
    ax.plot(days42, bsk_eclipse_h, color=BSK_COLOR, lw=1.0, alpha=0.9, label=f"{label_bsk}: eclipse_factor")
    ax.set_ylim(-0.05, 1.05)
    _style_axis(ax, "3) Eclipse / sunlit state", "Sunlit fraction [0–1]", xlim_days=max_days)
    ax.legend(loc="lower right", fontsize=8)

    ax = axes[3]
    ax.plot(days42, n42["thrust_n"] * 1000.0, color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(days42, bsk_thrust_h * 1000.0, color=BSK_COLOR, lw=1.0, alpha=0.9, label=f"{label_bsk} (hourly interp)")
    _style_axis(ax, "4) Applied thrust", "Thrust [mN]", xlim_days=max_days)
    ax.legend(loc="upper right", fontsize=8)

    ax = axes[4]
    ax.plot(days42, n42_fuel_h, color=N42_COLOR, lw=1.2, label=f"{label_n42}: fuel used")
    ax.plot(days42, bsk_fuel_h, color=BSK_COLOR, lw=1.0, alpha=0.9, label=f"{label_bsk}: fuel used (hourly interp)")
    _style_axis(ax, "5) Fuel depletion", "Fuel used [kg]", xlim_days=max_days)
    ax.legend(loc="upper left", fontsize=8)

    ax = axes[5]
    ax.plot(days42, n42["point_deg"], color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(days42, bsk_point_h, color=BSK_COLOR, lw=1.0, alpha=0.9, label=f"{label_bsk} (hourly interp)")
    _style_axis(ax, "6) Thrust-axis vs velocity pointing error", "Error [deg]", xlim_days=max_days)
    ax.legend(loc="upper right", fontsize=8)

    out_png = out_dir / "basilisk_vs_nasa42_compare.png"
    fig.savefig(out_png, dpi=150, bbox_inches="tight")
    plt.close(fig)

    # Early-transfer zoom: first N days after spiral ignition (not the full spiral).
    spiral_start_d = bsk["spiral_start_s"] / 86400.0
    z0 = max(0.0, spiral_start_d - zoom_pad_days)
    z1 = spiral_start_d + zoom_span_days

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True, constrained_layout=True)
    fig.suptitle(
        f"Basilisk vs NASA 42 — early transfer zoom (days {z0:.2f}–{z1:.2f})",
        fontsize=13,
        y=1.01,
    )

    mask42 = (days42 >= z0) & (days42 <= z1)
    mask_bsk = (bsk["thrust_t"] / 86400.0 >= z0) & (bsk["thrust_t"] / 86400.0 <= z1)
    d_bsk = bsk["thrust_t"][mask_bsk] / 86400.0

    ax = axes[0]
    ax.step(days42[mask42], n42["thrust_level_cmd"][mask42], where="post", color=N42_COLOR, lw=1.4, label=label_n42)
    ax.step(d_bsk, bsk_phase[mask_bsk], where="post", color=BSK_COLOR, lw=1.0, ls="--", label=f"{label_bsk} phase")
    ax.set_ylim(-0.05, 1.15)
    _style_axis(ax, "1) Thrust ON window (early transfer)", "Command / phase [0–1]")
    ax.set_xlim(z0, z1)
    ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(days42[mask42], n42["power_kw"][mask42], color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(d_bsk, bsk["power_kw"][mask_bsk], color=BSK_COLOR, lw=0.8, alpha=0.85, label=label_bsk)
    _style_axis(ax, "2) Available propulsion power (early transfer)", "Power [kW]")
    ax.set_xlim(z0, z1)
    ax.legend(fontsize=8)

    ax = axes[2]
    ax.step(days42[mask42], 1.0 - n42["eclipse"][mask42], where="post", color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(d_bsk, bsk["eclipse_factor"][mask_bsk], color=BSK_COLOR, lw=0.8, alpha=0.85, label=label_bsk)
    ax.set_ylim(-0.05, 1.05)
    _style_axis(ax, "3) Eclipse / sunlit state (early transfer)", "Sunlit fraction [0–1]")
    ax.set_xlim(z0, z1)
    ax.legend(fontsize=8)

    ax = axes[3]
    ax.plot(days42[mask42], n42["thrust_n"][mask42] * 1000.0, color=N42_COLOR, lw=1.2, label=label_n42)
    ax.plot(d_bsk, bsk["thrust_n"][mask_bsk] * 1000.0, color=BSK_COLOR, lw=0.8, alpha=0.85, label=label_bsk)
    _style_axis(ax, "4) Applied thrust (early transfer)", "Thrust [mN]")
    ax.set_xlim(z0, z1)
    ax.legend(fontsize=8)

    zoom_png = out_dir / "basilisk_vs_nasa42_compare_spiral_zoom.png"
    fig.savefig(zoom_png, dpi=150, bbox_inches="tight")
    plt.close(fig)

    meta = {
        "basilisk_label": label_bsk,
        "nasa42_label": label_n42,
        "spiral_start_s": bsk["spiral_start_s"],
        "spiral_end_s": bsk["spiral_end_s"],
        "zoom_window_days": [z0, z1],
        "zoom_span_days": zoom_span_days,
        "basilisk_thrust_samples": int(bsk["thrust_t"].size),
        "nasa42_output_samples": int(t42.size),
        "nasa42_output_interval_s": float(np.median(np.diff(t42))) if t42.size > 1 else None,
        "notes": [
            "Basilisk eclipse_factor and power are from thrust_history (spiral only, ~10 s cadence).",
            "NASA 42 Power.42 is thruster available power [kW]; Eclipse.42 is binary.",
            "Hourly interpolation aligns Basilisk dense logs to NASA 42 output cadence in full-mission plot.",
        ],
        "outputs": {
            "full_mission_png": str(out_png),
            "spiral_zoom_png": str(zoom_png),
        },
    }
    (out_dir / "compare_manifest.json").write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--basilisk-dir",
        type=Path,
        default=root / "data/pipeline_handoff/basilisk/gmat_gnc_handoff_dynamic_solar",
    )
    parser.add_argument(
        "--nasa42-dir",
        type=Path,
        default=Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark"),
    )
    parser.add_argument(
        "--cmd-file",
        type=Path,
        default=Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark/Inp_Cmd.txt"),
    )
    parser.add_argument(
        "--schedule",
        type=Path,
        default=root / "data/pipeline_handoff/gmat/burn_schedule.json",
    )
    parser.add_argument(
        "--gmat-spline",
        type=Path,
        default=Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark/a3_gmat_spline.txt"),
        help="GMAT SPLINE state file for full-mission reference SMA.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Defaults to <basilisk-dir>/compare_nasa42",
    )
    parser.add_argument("--label-bsk", default="Basilisk dynamic_solar")
    parser.add_argument("--label-n42", default="NASA 42 N9 full")
    parser.add_argument(
        "--zoom-span-days",
        type=float,
        default=7.0,
        help="Width of early-transfer zoom window [days] from spiral ignition.",
    )
    parser.add_argument(
        "--zoom-pad-days",
        type=float,
        default=0.1,
        help="Padding before spiral ignition in early-transfer zoom [days].",
    )
    args = parser.parse_args()

    out_dir = args.out_dir or (args.basilisk_dir / "compare_nasa42")
    schedule = json.loads(args.schedule.read_text(encoding="utf-8"))
    bsk = load_basilisk(args.basilisk_dir, schedule)
    n42 = load_nasa42(args.nasa42_dir, args.cmd_file)
    make_compare_plots(
        bsk,
        n42,
        out_dir,
        label_bsk=args.label_bsk,
        label_n42=args.label_n42,
        zoom_span_days=args.zoom_span_days,
        zoom_pad_days=args.zoom_pad_days,
    )

    sma_png = out_dir / "basilisk_nasa42_gmat_sma_compare.png"
    make_sma_compare_plot(
        args.basilisk_dir,
        args.nasa42_dir,
        schedule,
        sma_png,
        gmat_spline=args.gmat_spline,
        label_bsk="Basilisk execution",
        label_n42="NASA 42 execution",
        label_gmat="GMAT reference",
    )
    manifest = json.loads((out_dir / "compare_manifest.json").read_text(encoding="utf-8"))
    manifest["outputs"]["sma_compare_png"] = str(sma_png)
    (out_dir / "compare_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {out_dir / 'basilisk_vs_nasa42_compare.png'}")
    print(f"Wrote {out_dir / 'basilisk_vs_nasa42_compare_spiral_zoom.png'}")
    print(f"Wrote {sma_png}")
    print(f"Wrote {out_dir / 'compare_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
