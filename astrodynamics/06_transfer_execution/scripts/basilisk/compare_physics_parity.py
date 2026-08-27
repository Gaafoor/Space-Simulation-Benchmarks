#!/usr/bin/env python3
"""Compare legacy vs JGM2+MSIS physics parity handoff runs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ephemeris_handoff import SpiceReferenceEphemeris, position_error_envelope

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_BASELINE = WORKSPACE / "data/pipeline_handoff/basilisk/gmat_gnc_handoff"
DEFAULT_PARITY = WORKSPACE / "data/pipeline_handoff/basilisk/gmat_gnc_handoff_physics_parity"
DEFAULT_SCHEDULE = WORKSPACE / "data/pipeline_handoff/gmat/burn_schedule.json"


def _load_orbit_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    t = np.array([float(r["time_s"]) for r in rows], dtype=float)
    sma = np.array([float(r["sma_km"]) for r in rows], dtype=float)
    return t, sma


def _sma_milestone_day(t_s: np.ndarray, sma_km: np.ndarray, target_km: float) -> float | None:
    hit = np.where(sma_km >= target_km)[0]
    if hit.size == 0:
        return None
    return float(t_s[hit[0]] / 86400.0)


def _load_state(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    t = np.array([float(r["time_s"]) for r in rows], dtype=float)
    pos = np.array([[float(r["rx_m"]), float(r["ry_m"]), float(r["rz_m"])] for r in rows], dtype=float)
    return t, pos


def compare(
    baseline_dir: Path,
    parity_dir: Path,
    schedule_path: Path,
    *,
    out_dir: Path | None = None,
) -> dict:
    out_dir = out_dir or parity_dir
    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    baseline_summary = json.loads((baseline_dir / "summary.json").read_text(encoding="utf-8"))
    parity_summary = json.loads((parity_dir / "summary.json").read_text(encoding="utf-8"))

    t_b, sma_b = _load_orbit_csv(baseline_dir / "orbit_elements_history.csv")
    t_p, sma_p = _load_orbit_csv(parity_dir / "orbit_elements_history.csv")

    gmat_ref = schedule.get("gmat_reference", {})
    gmat_spiral_end_day = float(gmat_ref.get("spiral_end_s", float("nan"))) / 86400.0

    report = {
        "baseline_dir": str(baseline_dir),
        "parity_dir": str(parity_dir),
        "gmat_spiral_end_day": gmat_spiral_end_day,
        "baseline": {
            "force_model": baseline_summary.get("force_model"),
            "fidelity": baseline_summary.get("fidelity", "baseline"),
            "final_sma_km": baseline_summary.get("final_sma_km"),
            "fuel_used_kg": baseline_summary.get("fuel_used_kg"),
            "max_ephemeris_error_km": float(baseline_summary.get("max_ephemeris_position_error_m", float("nan"))) / 1000.0,
            "sma_42164_day": _sma_milestone_day(t_b, sma_b, 42164.0),
            "stop_reason": (baseline_summary.get("burns_applied") or [{}])[0].get("stop_reason"),
        },
        "parity": {
            "force_model": parity_summary.get("force_model"),
            "fidelity": parity_summary.get("fidelity", "unknown"),
            "final_sma_km": parity_summary.get("final_sma_km"),
            "fuel_used_kg": parity_summary.get("fuel_used_kg"),
            "max_ephemeris_error_km": float(parity_summary.get("max_ephemeris_position_error_m", float("nan"))) / 1000.0,
            "sma_42164_day": _sma_milestone_day(t_p, sma_p, 42164.0),
            "stop_reason": (parity_summary.get("burns_applied") or [{}])[0].get("stop_reason"),
        },
    }

    # SMA vs time
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t_b / 86400.0, sma_b, label="Baseline (GGM03S + exponential)", lw=1.2)
    ax.plot(t_p / 86400.0, sma_p, label="Physics parity (GGM2B + MSIS)", lw=1.2)
    ax.axhline(42164.0, color="gray", ls="--", lw=0.8, label="GEO SMA ~42164 km")
    if np.isfinite(gmat_spiral_end_day):
        ax.axvline(gmat_spiral_end_day, color="green", ls=":", lw=1.0, label=f"GMAT spiral end ~{gmat_spiral_end_day:.1f} d")
    ax.set_xlabel("Mission time [days]")
    ax.set_ylabel("Osculating SMA [km]")
    ax.set_title("LEO→GEO spiral: Basilisk SMA vs GMAT reference timeline")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout()
    sma_plot = out_dir / "physics_parity_sma_vs_time.png"
    fig.savefig(sma_plot, dpi=150)
    plt.close(fig)

    # Position error vs BSP (parity run)
    ref_meta = schedule.get("reference_ephemeris", {})
    t_state, pos = _load_state(parity_dir / "state_history.csv")
    ref = SpiceReferenceEphemeris.from_metadata(ref_meta)
    _, err_m = position_error_envelope(
        t_state,
        pos,
        spice_ref=ref,
        min_time_s=float(ref_meta.get("bsp_coverage_start_s", 0.0)),
    )
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(t_state / 86400.0, err_m / 1000.0, color="tab:orange", lw=0.8)
    ax2.set_xlabel("Mission time [days]")
    ax2.set_ylabel("Position error vs GMAT BSP [km]")
    ax2.set_title("Physics parity run — along-path error")
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    err_plot = out_dir / "physics_parity_ephemeris_error.png"
    fig2.savefig(err_plot, dpi=150)
    plt.close(fig2)

    report["plots"] = {"sma_vs_time": str(sma_plot), "ephemeris_error": str(err_plot)}
    (out_dir / "physics_parity_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    md = [
        "# Physics parity comparison (step 1)",
        "",
        "## Force models",
        f"- **Baseline:** {report['baseline']['force_model']} ({report['baseline'].get('fidelity', 'baseline')})",
        f"- **Parity:** {report['parity']['force_model']} ({report['parity'].get('fidelity', 'unknown')})",
        "",
        "## Key metrics",
        "| Metric | Baseline | Parity | GMAT ref |",
        "|--------|----------|--------|----------|",
        f"| Final SMA [km] | {report['baseline']['final_sma_km']:.3f} | {report['parity']['final_sma_km']:.3f} | {gmat_ref.get('final_sma_km', 'n/a')} |",
        f"| Day SMA≥42164 reached | {report['baseline']['sma_42164_day']:.2f} | {report['parity']['sma_42164_day']:.2f} | {gmat_spiral_end_day:.2f} (spiral end) |",
        f"| Fuel used [kg] | {report['baseline']['fuel_used_kg']:.3f} | {report['parity']['fuel_used_kg']:.3f} | {gmat_ref.get('fuel_mass_kg', 'n/a')} |",
        f"| Max BSP error [km] | {report['baseline']['max_ephemeris_error_km']:.1f} | {report['parity']['max_ephemeris_error_km']:.1f} | 0 (reference) |",
        f"| Spiral stop reason | {report['baseline']['stop_reason']} | {report['parity']['stop_reason']} | — |",
        "",
        f"Plots: `{sma_plot.name}`, `{err_plot.name}`",
    ]
    (out_dir / "physics_parity_report.md").write_text("\n".join(md), encoding="utf-8")
    return report


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--baseline-dir", type=Path, default=DEFAULT_BASELINE)
    p.add_argument("--parity-dir", type=Path, default=DEFAULT_PARITY)
    p.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    args = p.parse_args()
    report = compare(args.baseline_dir, args.parity_dir, args.schedule)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
