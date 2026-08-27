#!/usr/bin/env python3
"""Validation dashboard for GMAT -> Basilisk handoff runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from Basilisk.utilities import orbitalMotion

from a3_leo_geo_config import MU_EARTH

DEFAULT_OUT = Path(__file__).resolve().parents[1] / "data/pipeline_handoff/basilisk/gmat_gnc_handoff"
DEFAULT_SCHEDULE = Path(__file__).resolve().parents[1] / "data/pipeline_handoff/gmat/burn_schedule.json"


def _load_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _load_state(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = _load_csv(path)
    if not rows:
        return np.array([]), np.zeros((0, 3)), np.zeros((0, 3))
    t = np.array([float(r["time_s"]) for r in rows], dtype=float)
    r = np.array([[float(r["rx_m"]), float(r["ry_m"]), float(r["rz_m"])] for r in rows], dtype=float)
    v = np.array([[float(r["vx_m_s"]), float(r["vy_m_s"]), float(r["vz_m_s"])] for r in rows], dtype=float)
    return t, r, v


def _elements_from_rv(mu: float, r: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sma = np.full(r.shape[0], np.nan)
    ecc = np.full(r.shape[0], np.nan)
    inc = np.full(r.shape[0], np.nan)
    for i in range(r.shape[0]):
        if not (np.all(np.isfinite(r[i])) and np.all(np.isfinite(v[i]))):
            continue
        oe = orbitalMotion.rv2elem(mu, r[i], v[i])
        sma[i] = float(oe.a) / 1000.0
        ecc[i] = float(oe.e)
        inc[i] = math.degrees(float(oe.i))
    return sma, ecc, inc


def _mask_during_burns(times_s: np.ndarray, burns: list[dict]) -> np.ndarray:
    mask = np.zeros(times_s.shape, dtype=bool)
    for t0, t1 in _burn_windows(burns):
        mask |= (times_s >= t0) & (times_s <= t1)
    return mask


def _prepare_pointing_series(
    times_s: np.ndarray,
    errors_deg: np.ndarray,
    burns: list[dict],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return burn-window samples used for execution validation plots."""
    times = np.asarray(times_s, dtype=float)
    errors = np.asarray(errors_deg, dtype=float)
    valid = np.isfinite(times) & np.isfinite(errors)
    times = times[valid]
    errors = errors[valid]
    if times.size == 0:
        return times, errors, np.zeros(0, dtype=bool)

    during = _mask_during_burns(times, burns)
    if np.any(during):
        return times[during], errors[during], during
    return times, errors, np.ones(times.shape, dtype=bool)


def _pointing_axis_limits(limit_deg: float, errors_deg: np.ndarray) -> tuple[float, float]:
    finite = np.asarray(errors_deg, dtype=float)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return 0.0, float(limit_deg) * 1.15

    peak = float(np.max(finite))
    if peak <= float(limit_deg):
        ymax = float(limit_deg) * 1.15
    else:
        ymax = max(float(limit_deg) * 1.2, peak * 1.08)
    return 0.0, ymax


def _plot_pointing_error_on_ax(
    ax,
    pt: np.ndarray,
    pe: np.ndarray,
    *,
    limit_deg: float,
    zoom_to_limit: bool,
) -> float:
    over = pe > float(limit_deg)
    if np.any(~over):
        ax.plot(pt[~over] / 3600.0, pe[~over], color="#2ca02c", lw=1.1, label="within limit")
    if np.any(over):
        ax.plot(pt[over] / 3600.0, pe[over], color="#d62728", lw=1.1, label="above limit")

    ax.axhspan(0.0, float(limit_deg), color="#2ca02c", alpha=0.08, zorder=0)
    ax.axhline(float(limit_deg), color="red", ls="--", lw=1.2, label=f"limit {limit_deg:.0f}°")
    peak = float(np.max(pe))
    if zoom_to_limit:
        ax.set_ylim(0.0, float(limit_deg) * 1.15)
    else:
        ymin, ymax = _pointing_axis_limits(limit_deg, pe)
        ax.set_ylim(ymin, ymax)
    return peak


def _plot_pointing_error(
    ax,
    times_s: np.ndarray,
    errors_deg: np.ndarray,
    burns: list[dict],
    *,
    limit_deg: float,
    title: str,
    add_zoom_inset: bool = True,
) -> bool:
    pt, pe, _ = _prepare_pointing_series(times_s, errors_deg, burns)
    if pt.size == 0:
        return False

    peak = _plot_pointing_error_on_ax(ax, pt, pe, limit_deg=limit_deg, zoom_to_limit=False)
    _shade_burns(ax, burns, label="burn window")
    ax.text(
        0.02,
        0.97,
        f"max during burns: {peak:.2f}°",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=8,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
    )
    ax.set_ylabel("Pointing error [deg]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    if add_zoom_inset and peak > float(limit_deg) * 1.25:
        from mpl_toolkits.axes_grid1.inset_locator import inset_axes

        axins = inset_axes(ax, width="42%", height="42%", loc="upper right", borderpad=1.0)
        _plot_pointing_error_on_ax(axins, pt, pe, limit_deg=limit_deg, zoom_to_limit=True)
        axins.set_title(f"0–{limit_deg:.0f}° envelope", fontsize=7)
        axins.tick_params(labelsize=7)
        axins.grid(True, alpha=0.25)
    return True

def _burn_windows(burns: list[dict]) -> list[tuple[float, float]]:
    windows = []
    for burn in burns:
        if burn.get("type") not in {"finite_thrust", "electric_spiral"}:
            continue
        t0 = float(burn.get("time_s", burn.get("gmat_start_s", 0.0)))
        t1 = t0 + float(burn.get("duration_s", 0.0))
        if t1 > t0:
            windows.append((t0, t1))
    return windows


def _shade_burns(ax, burns: list[dict], *, label: str = "burn window") -> None:
    labeled = False
    for t0, t1 in _burn_windows(burns):
        ax.axvspan(t0 / 3600.0, t1 / 3600.0, color="#f4b6b6", alpha=0.25, label=None if labeled else label)
        labeled = True


def _try_gmat_sma_from_bsp(times_s: np.ndarray, schedule: dict) -> np.ndarray | None:
    ref = schedule.get("reference_ephemeris", {})
    if not ref.get("path"):
        return None
    try:
        from ephemeris_handoff import SpiceKernelSession, SpiceReferenceEphemeris, read_spice_state_at
    except ImportError:
        return None
    bsp_path = Path(ref["path"])
    if not bsp_path.is_file() or bsp_path.suffix.lower() != ".bsp":
        return None
    spice_ref = SpiceReferenceEphemeris.from_metadata(ref)
    min_t = float(ref.get("bsp_coverage_start_s", 0.0))
    gmat_sma = np.full(times_s.shape, np.nan)
    step = max(1, times_s.size // 400)
    with SpiceKernelSession(bsp_path):
        for i in range(0, times_s.size, step):
            t = float(times_s[i])
            if t < min_t:
                continue
            try:
                r_ref, v_ref = read_spice_state_at(spice_ref, t)
                gmat_sma[i] = float(orbitalMotion.rv2elem(MU_EARTH, r_ref, v_ref).a) / 1000.0
            except Exception:
                continue
    return gmat_sma


def _compute_validation_checks(summary: dict, schedule: dict, *, out_dir: Path | None = None) -> dict:
    target = schedule.get("target", {})
    gmat_ref = summary.get("gmat_reference", schedule.get("gmat_reference", {}))
    propulsion = schedule.get("propulsion", {})
    fuel_used = float(summary.get("fuel_used_kg", 0.0))
    gmat_fuel = float(gmat_ref.get("fuel_mass_kg") or propulsion.get("gmat_fuel_used_kg") or 0.0)
    fuel_frac = abs(fuel_used - gmat_fuel) / gmat_fuel if gmat_fuel > 0 else float("nan")

    pointing_path = summary.get("pointing_error_history")
    if pointing_path and not str(pointing_path).endswith(".csv"):
        pointing_path = None
    if not pointing_path and out_dir is not None:
        pointing_path = str(out_dir / "pointing_error_history.csv")
    max_pointing = float("nan")
    if pointing_path and Path(pointing_path).is_file():
        rows = _load_csv(Path(pointing_path))
        vals = [float(r["pointing_error_deg"]) for r in rows if r.get("pointing_error_deg") and np.isfinite(float(r["pointing_error_deg"]))]
        if vals:
            burns = summary.get("burns_applied", schedule.get("burns", []))
            windows = _burn_windows(burns)
            during = []
            for row in rows:
                t = float(row["time_s"])
                for t0, t1 in windows:
                    if t0 <= t <= t1:
                        during.append(float(row["pointing_error_deg"]))
                        break
            max_pointing = max(during) if during else max(vals)

    checks = {
        "attitude_pointing": {
            "passed": math.isfinite(max_pointing) and max_pointing <= float(target.get("max_pointing_error_deg", 5.0)),
            "value_deg": max_pointing,
            "limit_deg": float(target.get("max_pointing_error_deg", 5.0)),
            "scope": "during_burns",
        },
        "fuel_usage": {
            "passed": math.isfinite(fuel_frac) and fuel_frac <= float(target.get("max_fuel_error_fraction", 0.25)),
            "value_kg": fuel_used,
            "gmat_reference_kg": gmat_fuel,
            "relative_error": fuel_frac,
            "limit_fraction": float(target.get("max_fuel_error_fraction", 0.25)),
        },
        "burn_schedule_timing": {
            "passed": float(summary.get("max_burn_start_error_s", 0.0)) <= float(target.get("max_burn_start_error_s", 120.0)),
            "value_s": float(summary.get("max_burn_start_error_s", 0.0)),
            "limit_s": float(target.get("max_burn_start_error_s", 120.0)),
        },
        "ephemeris_envelope": {
            "passed": (
                math.isfinite(float(summary.get("max_ephemeris_position_error_m", float("nan"))))
                and float(summary.get("max_ephemeris_position_error_m", float("nan")))
                <= float(target.get("max_ephemeris_position_error_m", 500_000.0))
            ),
            "value_m": float(summary.get("max_ephemeris_position_error_m", float("nan"))),
            "limit_m": float(target.get("max_ephemeris_position_error_m", 500_000.0)),
        },
        "orbit_sma": {
            "passed": (
                abs(float(summary.get("final_sma_km", 0.0)) * 1000.0 - float(target.get("semi_major_axis_m", 42164000.0)))
                <= float(target.get("sma_tolerance_m", 1000.0))
            ),
            "value_m": abs(float(summary.get("final_sma_km", 0.0)) * 1000.0 - float(target.get("semi_major_axis_m", 42164000.0))),
            "limit_m": float(target.get("sma_tolerance_m", 1000.0)),
            "informational": True,
        },
        "orbit_ecc": {
            "passed": float(summary.get("final_ecc", 1.0)) <= float(target.get("ecc_max", 0.05)),
            "value": float(summary.get("final_ecc", float("nan"))),
            "limit": float(target.get("ecc_max", 0.05)),
            "informational": True,
        },
        "orbit_inc": {
            "passed": (
                abs(float(summary.get("final_inc_deg", 90.0)) - float(target.get("inclination_deg", 0.0)))
                <= float(target.get("inc_tolerance_deg", 0.1))
            ),
            "value_deg": abs(float(summary.get("final_inc_deg", 90.0)) - float(target.get("inclination_deg", 0.0))),
            "limit_deg": float(target.get("inc_tolerance_deg", 0.1)),
            "informational": True,
        },
    }
    exec_names = schedule.get("execution_validation", {}).get(
        "checks",
        ["attitude_pointing", "fuel_usage", "burn_schedule_timing", "ephemeris_envelope"],
    )
    summary["validation_checks"] = checks
    summary["execution_validation_checks"] = {k: checks[k] for k in exec_names if k in checks}
    summary["informational_checks"] = {
        k: checks[k]
        for k in schedule.get("execution_validation", {}).get("informational_checks", ["orbit_sma", "orbit_ecc", "orbit_inc"])
        if k in checks
    }
    summary["validation_passed"] = all(summary["execution_validation_checks"][k]["passed"] for k in summary["execution_validation_checks"])
    return checks


def _format_check_line(name: str, check: dict) -> str:
    status = "PASS" if check.get("passed") else ("INFO" if check.get("informational") else "FAIL")
    if name == "attitude_pointing":
        return f"[{status}] Max pointing during burns: {check.get('value_deg', float('nan')):.3f} deg (limit {check.get('limit_deg')} deg)"
    if name == "fuel_usage":
        return f"[{status}] Fuel used: {check.get('value_kg', float('nan')):.3f} kg (GMAT ref {check.get('gmat_reference_kg', float('nan')):.3f} kg, err {100*check.get('relative_error', float('nan')):.1f}%)"
    if name == "burn_schedule_timing":
        return f"[{status}] Burn start timing error: {check.get('value_s', float('nan')):.2f} s (limit {check.get('limit_s')} s)"
    if name == "ephemeris_envelope":
        return f"[{status}] Ephemeris position error: {check.get('value_m', float('nan'))/1000:.1f} km (limit {check.get('limit_m', float('nan'))/1000:.1f} km)"
    if name == "orbit_sma":
        return f"[{status}] SMA residual vs GEO target: {check.get('value_m', float('nan'))/1000:.1f} km (informational)"
    if name == "orbit_ecc":
        return f"[{status}] Final eccentricity: {check.get('value', float('nan')):.6f} (informational)"
    if name == "orbit_inc":
        return f"[{status}] Final inclination residual: {check.get('value_deg', float('nan')):.3f} deg (informational)"
    return f"[{status}] {name}"


def generate_validation_plots(
    out_dir: Path,
    *,
    schedule_path: Path | None = None,
    save_path: Path | None = None,
) -> Path:
    out_dir = Path(out_dir)
    schedule_path = schedule_path or DEFAULT_SCHEDULE
    summary_path = out_dir / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    schedule = json.loads(Path(schedule_path).read_text(encoding="utf-8")) if schedule_path.is_file() else {}
    checks = _compute_validation_checks(summary, schedule, out_dir=out_dir)

    t, r, v = _load_state(out_dir / "state_history.csv")
    if t.size == 0:
        raise ValueError(f"No state history in {out_dir / 'state_history.csv'}")
    t_h = t / 3600.0
    sma_km, ecc, inc_deg = _elements_from_rv(MU_EARTH, r, v)
    gmat_sma_km = _try_gmat_sma_from_bsp(t, schedule)

    target = schedule.get("target", {})
    gmat_ref = summary.get("gmat_reference", schedule.get("gmat_reference", {}))
    burns = summary.get("burns_applied", schedule.get("burns", []))

    fig, axes = plt.subplots(3, 2, figsize=(13, 11), sharex=False)
    ax_sma, ax_ecc, ax_inc, ax_point, ax_thrust, ax_summary = axes.ravel()

    ax_sma.plot(t_h, sma_km, color="#1f77b4", lw=1.2, label="Basilisk SMA")
    if gmat_sma_km is not None and np.any(np.isfinite(gmat_sma_km)):
        ax_sma.plot(t_h, gmat_sma_km, color="#9467bd", lw=1.0, alpha=0.8, label="GMAT reference (BSP)")
    ax_sma.axhline(float(target.get("semi_major_axis_m", 42164000.0)) / 1000.0, color="green", ls="--", lw=1.2, label="GEO SMA target")
    if gmat_ref.get("final_sma_km") is not None:
        ax_sma.axhline(float(gmat_ref["final_sma_km"]), color="#ff7f0e", ls=":", lw=1.2, label="GMAT final SMA")
    _shade_burns(ax_sma, burns)
    ax_sma.set_ylabel("SMA [km]")
    ax_sma.set_title("Semi-major axis (informational orbit check)")
    ax_sma.grid(True, alpha=0.3)
    ax_sma.legend(loc="best", fontsize=8)

    ax_ecc.plot(t_h, ecc, color="#ff7f0e", lw=1.0)
    ax_ecc.axhline(float(target.get("ecc_max", 0.05)), color="red", ls="--", lw=1.0, label="ecc limit (info)")
    _shade_burns(ax_ecc, burns, label="_nolegend_")
    ax_ecc.set_ylabel("Eccentricity")
    ax_ecc.set_title("Eccentricity (informational)")
    ax_ecc.grid(True, alpha=0.3)
    ax_ecc.legend(loc="best", fontsize=8)

    ax_inc.plot(t_h, inc_deg, color="#2ca02c", lw=1.0)
    ax_inc.axhline(float(target.get("inclination_deg", 0.0)), color="green", ls="--", lw=1.0, label="GEO inc target")
    if gmat_ref.get("final_inc_deg") is not None:
        ax_inc.axhline(float(gmat_ref["final_inc_deg"]), color="#ff7f0e", ls=":", lw=1.0, label="GMAT final inc")
    _shade_burns(ax_inc, burns, label="_nolegend_")
    ax_inc.set_ylabel("Inclination [deg]")
    ax_inc.set_title("Inclination (informational)")
    ax_inc.grid(True, alpha=0.3)
    ax_inc.legend(loc="best", fontsize=8)

    pointing_rows = _load_csv(out_dir / "pointing_error_history.csv")
    if pointing_rows:
        pt = np.array([float(r["time_s"]) for r in pointing_rows], dtype=float)
        pe = np.array([float(r["pointing_error_deg"]) for r in pointing_rows], dtype=float)
        limit_deg = float(target.get("max_pointing_error_deg", 5.0))
        if not _plot_pointing_error(
            ax_point,
            pt,
            pe,
            burns,
            limit_deg=limit_deg,
            title="Velocity-pointing error during burns (execution check)",
        ):
            ax_point.text(0.5, 0.5, "No valid pointing samples", ha="center", va="center", transform=ax_point.transAxes)
            ax_point.set_title("Velocity-pointing error")
    else:
        ax_point.text(0.5, 0.5, "Pointing not logged in this run", ha="center", va="center", transform=ax_point.transAxes)
        ax_point.set_title("Velocity-pointing error (execution check)")

    thrust_rows = _load_csv(out_dir / "thrust_history.csv")
    if thrust_rows:
        tt = np.array([float(r["time_s"]) for r in thrust_rows], dtype=float)
        thrust = np.array([float(r["thrust_n"]) for r in thrust_rows], dtype=float)
        mass = np.array([float(r["mass_kg"]) for r in thrust_rows], dtype=float)
        ax_thrust.plot(tt / 3600.0, thrust, color="#d62728", lw=1.5, label="Thrust [N]")
        ax_mass = ax_thrust.twinx()
        ax_mass.plot(tt / 3600.0, mass, color="#1f77b4", lw=1.0, alpha=0.7, label="Mass [kg]")
        ax_thrust.set_ylabel("Thrust [N]", color="#d62728")
        ax_mass.set_ylabel("Mass [kg]", color="#1f77b4")
        ax_thrust.set_title("Thrust command execution")
        ax_thrust.grid(True, alpha=0.3)
        lines, labels = ax_thrust.get_legend_handles_labels()
        lines2, labels2 = ax_mass.get_legend_handles_labels()
        ax_thrust.legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    else:
        ax_thrust.text(0.5, 0.5, "No thrust history", ha="center", va="center", transform=ax_thrust.transAxes)

    ax_summary.axis("off")
    verdict = "PASS" if summary.get("validation_passed") else "FAIL"
    lines = [
        f"VALIDATION {verdict}",
        f"GMAT converged (design): {summary.get('converged')}",
        f"mission_mode: {summary.get('mission_mode')}",
        "",
        "Execution checks (Basilisk must pass):",
    ]
    for name, check in summary.get("execution_validation_checks", {}).items():
        lines.append(_format_check_line(name, check))
    lines.extend(["", "Informational orbit checks (GMAT reference context):"])
    for name, check in summary.get("informational_checks", {}).items():
        lines.append(_format_check_line(name, check))
    ax_summary.text(0.02, 0.98, "\n".join(lines), va="top", family="monospace", fontsize=8.5)

    for ax in (ax_sma, ax_ecc, ax_inc, ax_point, ax_thrust):
        ax.set_xlabel("Time [h]")
    fig.suptitle("GMAT design authority -> Basilisk execution validation", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.02, 1, 0.97])
    dashboard_path = save_path or (out_dir / "handoff_validation.png")
    fig.savefig(dashboard_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    # Focused SMA plot (what you asked for as main convergence view)
    fig2, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(t_h, sma_km, color="#1f77b4", lw=1.4, label="Basilisk executed SMA")
    if gmat_sma_km is not None and np.any(np.isfinite(gmat_sma_km)):
        ax.plot(t_h, gmat_sma_km, color="#9467bd", lw=1.1, alpha=0.85, label="GMAT reference SMA (BSP)")
    ax.axhline(float(target.get("semi_major_axis_m", 42164000.0)) / 1000.0, color="green", ls="--", lw=1.4, label="GEO SMA target (42164 km)")
    _shade_burns(ax, burns)
    ax.set_xlabel("Time [h]")
    ax.set_ylabel("Semi-major axis [km]")
    ax.set_title("Basilisk vs GMAT semi-major axis")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    sma_path = out_dir / "orbit_semi_major_axis.png"
    fig2.savefig(sma_path, dpi=150, bbox_inches="tight")
    plt.close(fig2)

    # Real pointing plot only when data exists
    if pointing_rows and len(pointing_rows) > 1:
        fig3, (ax_full, ax_zoom) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True, gridspec_kw={"height_ratios": [2.0, 1.0]})
        pt = np.array([float(r["time_s"]) for r in pointing_rows], dtype=float)
        pe = np.array([float(r["pointing_error_deg"]) for r in pointing_rows], dtype=float)
        limit_deg = float(target.get("max_pointing_error_deg", 5.0))
        pt_burn, pe_burn, _ = _prepare_pointing_series(pt, pe, burns)
        peak = float(np.max(pe_burn)) if pe_burn.size else float("nan")
        _plot_pointing_error_on_ax(ax_full, pt_burn, pe_burn, limit_deg=limit_deg, zoom_to_limit=False)
        _shade_burns(ax_full, burns)
        ax_full.text(
            0.02,
            0.97,
            f"max during burns: {peak:.2f}°",
            transform=ax_full.transAxes,
            va="top",
            ha="left",
            fontsize=8,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
        )
        ax_full.set_ylabel("Pointing error [deg]")
        ax_full.set_title("Velocity-pointing attitude error during burns (full scale)")
        ax_full.grid(True, alpha=0.3)
        ax_full.legend(loc="best", fontsize=8)

        _plot_pointing_error_on_ax(ax_zoom, pt_burn, pe_burn, limit_deg=limit_deg, zoom_to_limit=True)
        _shade_burns(ax_zoom, burns, label="_nolegend_")
        ax_zoom.set_xlabel("Time [h]")
        ax_zoom.set_ylabel("Pointing error [deg]")
        ax_zoom.set_title(f"Execution envelope zoom (0–{limit_deg:.0f}°)")
        ax_zoom.grid(True, alpha=0.3)
        pointing_path = out_dir / "pointing_error_history.png"
        fig3.savefig(pointing_path, dpi=150, bbox_inches="tight")
        plt.close(fig3)
        summary["pointing_error_plot"] = str(pointing_path)

    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return dashboard_path


def main() -> None:
    p = argparse.ArgumentParser(description="Generate handoff validation plots")
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    p.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    p.add_argument("--save", type=Path, default=None)
    args = p.parse_args()
    path = generate_validation_plots(args.out_dir, schedule_path=args.schedule, save_path=args.save)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
