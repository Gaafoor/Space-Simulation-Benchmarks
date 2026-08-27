#!/usr/bin/env python3
"""Post-run N9 validation for NASA 42 full LEO-GEO mission."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

MU_EARTH = 3.986004418e14

CASE_DIR = Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark")
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SCHEDULE = SCRIPT_DIR.parents[3] / "data/pipeline_handoff/gmat/burn_schedule.json"
DEFAULT_OUT = SCRIPT_DIR.parents[3] / "data/benchmark_a/nasa42/a3_leogeo_full_advanced"


def read_vec3(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 3:
            rows.append([float(x) for x in parts])
    return np.asarray(rows, dtype=float)


def read_scalar(path: Path) -> np.ndarray:
    return np.array([float(x) for x in path.read_text().split()], dtype=float)


def osculating_sma_m(r: np.ndarray, v: np.ndarray) -> float:
    rm = float(np.linalg.norm(r))
    vm = float(np.linalg.norm(v))
    energy = 0.5 * vm * vm - MU_EARTH / rm
    if energy >= 0.0:
        return float("nan")
    return -MU_EARTH / (2.0 * energy)


def first_crossing(times: np.ndarray, signal: np.ndarray, threshold: float, *, rising: bool) -> float | None:
    if rising:
        idx = np.where(signal >= threshold - 1e-12)[0]
    else:
        idx = np.where(signal <= threshold + 1e-12)[0]
    if idx.size == 0:
        return None
    return float(times[int(idx[0])])


def burn_times_from_stdout(log_path: Path) -> tuple[float | None, float | None]:
    """Parse ThrustLevelCmd on/off times from NASA 42 stdout (sub-hour precision)."""
    if not log_path.is_file():
        return None, None
    on_re = re.compile(
        r"^(\d+(?:\.\d+)?)\s+SC\[0\]\.AC\.Thr\[0\]\.ThrustLevelCmd\s*=\s*1\.0",
        re.M,
    )
    off_re = re.compile(
        r"^(\d+(?:\.\d+)?)\s+SC\[0\]\.AC\.Thr\[0\]\.ThrustLevelCmd\s*=\s*0\.0",
        re.M,
    )
    text = log_path.read_text(encoding="utf-8", errors="replace")
    on_hits = [float(m.group(1)) for m in on_re.finditer(text)]
    off_hits = [float(m.group(1)) for m in off_re.finditer(text)]
    burn_on = on_hits[0] if on_hits else None
    burn_off = off_hits[-1] if len(off_hits) > 1 else (off_hits[0] if off_hits else None)
    return burn_on, burn_off


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    schedule = json.loads(DEFAULT_SCHEDULE.read_text(encoding="utf-8"))
    target = schedule.get("target", {})
    propulsion = schedule.get("propulsion", {})
    gmat_ref = schedule.get("gmat_reference", {})
    spiral = next(b for b in schedule["burns"] if b["id"] == "spiral_to_geo")

    run_dir = CASE_DIR
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    required = ["time.42", "PosN.42", "VelN.42", "Thr.42", "Mass.42", "Power.42"]
    missing = [f for f in required if not (run_dir / f).is_file()]
    if missing:
        print(f"ERROR: missing outputs in {run_dir}: {missing}", file=sys.stderr)
        print("Run the full mission first.", file=sys.stderr)
        return 1

    times = read_scalar(run_dir / "time.42")
    pos = read_vec3(run_dir / "PosN.42")
    vel = read_vec3(run_dir / "VelN.42")
    thr = read_scalar(run_dir / "Thr.42")
    mass = read_scalar(run_dir / "Mass.42")
    power = read_scalar(run_dir / "Power.42")

    if times[-1] < float(schedule["dynamics"]["mission_end_s"]) - 3600.0:
        print(
            f"WARNING: simulation ended at t={times[-1]:.1f} s, "
            f"expected ~{schedule['dynamics']['mission_end_s']:.1f} s",
            file=sys.stderr,
        )

    sma = np.array([osculating_sma_m(pos[i], vel[i]) for i in range(len(times))])
    sma_km = sma / 1000.0

    burn_on_target = float(spiral["start_s"])
    burn_off_target = float(spiral["gmat_end_s"])
    stdout_log = out_dir / "run_stdout.log"
    burn_on_actual, burn_off_actual = burn_times_from_stdout(stdout_log)
    burn_timing_source = "stdout"
    if burn_on_actual is None or burn_off_actual is None:
        burn_on_actual = first_crossing(times, thr, 1e-6, rising=True)
        burn_off_actual = first_crossing(times, thr, 1e-6, rising=False)
        burn_timing_source = "Thr.42"

    initial_mass = float(propulsion.get("initial_mass_kg", mass[0]))
    final_mass = float(mass[-1])
    fuel_used = initial_mass - final_mass
    gmat_fuel = float(gmat_ref.get("fuel_mass_kg", propulsion.get("gmat_fuel_used_kg", 0.0)))
    fuel_err_frac = abs(fuel_used - gmat_fuel) / max(gmat_fuel, 1e-9)

    final_sma_km = float(sma_km[-1])
    gmat_final_sma_km = float(gmat_ref.get("final_sma_km", target.get("semi_major_axis_m", 0.0) / 1000.0))
    sma_err_km = final_sma_km - gmat_final_sma_km

    burn_start_err = abs(burn_on_actual - burn_on_target) if burn_on_actual is not None else float("nan")
    burn_stop_err = abs(burn_off_actual - burn_off_target) if burn_off_actual is not None else float("nan")

    spiral_mask = (times >= burn_on_target) & (times <= burn_off_target)
    mean_thrust_n = float(thr[spiral_mask].mean()) if np.any(spiral_mask) else 0.0
    mean_power_kw = float(power[spiral_mask].mean()) if np.any(spiral_mask) else 0.0
    duty_fraction = float((thr[spiral_mask] > 1e-6).mean()) if np.any(spiral_mask) else 0.0

    checks = {
        "burn_schedule_timing": {
            "pass": burn_start_err <= float(target.get("max_burn_start_error_s", 120.0))
            and burn_stop_err <= float(target.get("max_burn_stop_error_s", 120.0)),
            "source": burn_timing_source,
            "burn_start_error_s": burn_start_err,
            "burn_stop_error_s": burn_stop_err,
            "burn_on_actual_s": burn_on_actual,
            "burn_off_actual_s": burn_off_actual,
        },
        "fuel_usage": {
            "pass": fuel_err_frac <= float(target.get("max_fuel_error_fraction", 0.25)),
            "fuel_used_kg": fuel_used,
            "gmat_fuel_kg": gmat_fuel,
            "fuel_error_fraction": fuel_err_frac,
            "final_mass_kg": final_mass,
            "gmat_final_mass_kg": float(gmat_ref.get("fuel_mass_kg", 0.0)),  # placeholder overwritten below
        },
        "orbit_sma": {
            "informational": True,
            "final_sma_km": final_sma_km,
            "gmat_final_sma_km": gmat_final_sma_km,
            "sma_error_km": sma_err_km,
            "sma_tolerance_km": float(target.get("sma_tolerance_m", 1000.0)) / 1000.0,
        },
        "propulsion_dynamics": {
            "informational": True,
            "mean_spiral_thrust_n": mean_thrust_n,
            "mean_spiral_power_kw": mean_power_kw,
            "thrust_duty_fraction": duty_fraction,
            "nominal_thrust_n": float(propulsion.get("nominal_thrust_n", 0.0)),
        },
    }
    checks["fuel_usage"]["gmat_final_mass_kg"] = float(
        propulsion.get("dry_mass_kg", 12.0) + propulsion.get("initial_fuel_mass_kg", 5.0) - gmat_fuel
    )

    summary = {
        "schema": "nasa42_n9_full_validation_v1",
        "sim_duration_s": float(times[-1]),
        "mission_end_target_s": float(schedule["dynamics"]["mission_end_s"]),
        "checks": checks,
        "gmat_reference": gmat_ref,
    }
    out_path = out_dir / "summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {out_path}")
    print(f"  Sim duration:     {times[-1]:.1f} s")
    print(f"  Burn start err:   {burn_start_err:.1f} s  (limit {target.get('max_burn_start_error_s', 120)} s)")
    print(f"  Fuel used:        {fuel_used:.4f} kg  (GMAT {gmat_fuel:.4f} kg, err {100*fuel_err_frac:.1f}%)")
    print(f"  Final SMA:        {final_sma_km:.3f} km  (GMAT {gmat_final_sma_km:.3f} km, Δ {sma_err_km:+.3f} km)")
    print(f"  Mean spiral thrust:{mean_thrust_n*1000:.3f} mN over {100*duty_fraction:.1f}% duty")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
