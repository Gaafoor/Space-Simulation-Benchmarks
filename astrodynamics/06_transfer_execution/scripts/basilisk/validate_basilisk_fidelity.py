#!/usr/bin/env python3
"""Validate Basilisk handoff fidelity tiers against GMAT/42 gates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_SCHEDULE = WORKSPACE / "data/pipeline_handoff/gmat/burn_schedule.json"


def _load_thrust_history(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    return list(csv.DictReader(path.open(encoding="utf-8")))


def validate_run(out_dir: Path, schedule_path: Path, *, fidelity: str) -> dict:
    summary_path = out_dir / "summary.json"
    progress_path = out_dir / "run_progress.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.is_file() else {}
    progress = json.loads(progress_path.read_text(encoding="utf-8")) if progress_path.is_file() else {}
    run_status = progress.get("status") or summary.get("run_status", "completed")
    partial_run = run_status == "in_progress"
    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    target = schedule.get("target", {})
    gmat_ref = schedule.get("gmat_reference", {})
    checks: dict[str, dict] = {}

    final_sma_km = float(summary.get("final_sma_km", progress.get("final_sma_km", float("nan"))))
    gmat_sma_km = float(gmat_ref.get("final_sma_km", target.get("semi_major_axis_m", 0.0) / 1000.0))
    sma_tol_km = float(target.get("sma_tolerance_m", 1000.0)) / 1000.0
    checks["final_sma"] = {
        "value_km": final_sma_km,
        "reference_km": gmat_sma_km,
        "error_km": final_sma_km - gmat_sma_km,
        "tolerance_km": sma_tol_km,
        "passed": (
            False
            if partial_run
            else abs(final_sma_km - gmat_sma_km) <= sma_tol_km if final_sma_km == final_sma_km else False
        ),
        "partial_only": partial_run,
    }

    fuel_used = float(summary.get("fuel_used_kg", progress.get("fuel_used_kg", float("nan"))))
    gmat_fuel = float(gmat_ref.get("fuel_mass_kg", schedule.get("propulsion", {}).get("gmat_fuel_used_kg", 0.0)))
    fuel_tol = max(0.05, float(target.get("max_fuel_error_fraction", 0.25)) * gmat_fuel)
    checks["fuel_used"] = {
        "value_kg": fuel_used,
        "reference_kg": gmat_fuel,
        "error_kg": fuel_used - gmat_fuel,
        "tolerance_kg": fuel_tol,
        "passed": (
            False
            if partial_run
            else abs(fuel_used - gmat_fuel) <= fuel_tol if fuel_used == fuel_used else False
        ),
        "partial_only": partial_run,
    }

    thrust_rows = _load_thrust_history(out_dir / "thrust_history.csv")
    if thrust_rows and "power_kw" in thrust_rows[0]:
        powers = [float(r["power_kw"]) for r in thrust_rows if float(r.get("thrust_n", 0.0)) > 0.0]
        eclipses = [float(r.get("eclipse_factor", 1.0)) for r in thrust_rows if float(r.get("thrust_n", 0.0)) > 0.0]
        checks["dynamic_power"] = {
            "samples": len(powers),
            "max_power_kw": max(powers) if powers else float("nan"),
            "min_power_kw": min(powers) if powers else float("nan"),
            "eclipse_duty": float(sum(1 for e in eclipses if e < 0.999) / max(len(eclipses), 1)),
            "passed": len(powers) > 0 and (max(powers) - min(powers) > 1e-6 if fidelity != "baseline" else True),
        }
    else:
        checks["dynamic_power"] = {"passed": fidelity == "baseline", "samples": 0}

    pointing_csv = out_dir / "pointing_error_history.csv"
    if pointing_csv.is_file():
        errs = [abs(float(r["pointing_error_deg"])) for r in csv.DictReader(pointing_csv.open(encoding="utf-8"))]
        max_point = max(errs) if errs else float("nan")
        checks["pointing"] = {
            "max_error_deg": max_point,
            "limit_deg": float(target.get("max_pointing_error_deg", 5.0)),
            "passed": max_point <= float(target.get("max_pointing_error_deg", 5.0)) if max_point == max_point else False,
        }

    max_ephem = float(
        summary.get(
            "max_ephemeris_position_error_m",
            progress.get("max_ephemeris_position_error_m", float("nan")),
        )
    )
    ephem_limit_m = float(target.get("max_ephemeris_position_error_m", 500000.0))
    if max_ephem == max_ephem:
        checks["ephemeris_envelope"] = {
            "max_error_m": max_ephem,
            "limit_m": ephem_limit_m,
            "passed": max_ephem <= ephem_limit_m,
        }

    max_timing = float(
        summary.get(
            "max_burn_start_error_s",
            progress.get("max_burn_start_error_s", float("nan")),
        )
    )
    timing_limit_s = float(target.get("max_burn_start_error_s", 120.0))
    if max_timing == max_timing:
        checks["burn_timing"] = {
            "max_error_s": max_timing,
            "limit_s": timing_limit_s,
            "passed": max_timing <= timing_limit_s,
        }

    passed = all(item.get("passed", False) for item in checks.values() if "passed" in item)
    report = {
        "fidelity": fidelity,
        "out_dir": str(out_dir),
        "run_status": run_status,
        "partial_run": partial_run,
        "progress_fraction": progress.get("progress_fraction"),
        "checkpoint_time_s": progress.get("checkpoint_time_s"),
        "validation_passed": passed and not partial_run,
        "partial_validation_passed": passed if partial_run else None,
        "checks": checks,
        "summary": summary,
        "run_progress": progress or None,
    }
    (out_dir / "fidelity_validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", type=Path, required=True)
    p.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    p.add_argument("--fidelity", default="baseline")
    args = p.parse_args()
    report = validate_run(args.out_dir, args.schedule, fidelity=args.fidelity)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
