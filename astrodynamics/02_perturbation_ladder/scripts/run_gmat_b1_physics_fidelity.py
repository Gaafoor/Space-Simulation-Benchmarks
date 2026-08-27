#!/usr/bin/env python3
"""Benchmark B1 Physics Fidelity — GMAT post-processing for Cases 1–5.

Parses GMAT state CSV exports (km, km/s), computes drift metrics, and writes
artifacts under data/benchmark_b/b1_physics_fidelity/gmat/.

Expects five CSV files from benchmark_b1_gmat.script (600 s cadence, 7 days).
Default input directory: benchmarks/benchmark_b/results/gmat_b1/
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import platform
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from b1_metrics_postprocess import (
    compute_endpoint_drift_metrics,
    eci_position_to_subpoint_lat_lon,
    histories_from_rows,
)
from b1_physics_config import (
    B1Case,
    B1Config,
    R_EARTH,
    case_uses_drag,
    case_uses_j2,
    case_uses_srp,
    case_uses_third_body,
    load_b1_config_from_env,
)

KM_TO_M = 1000.0
MU_EARTH = 3.986004418e14

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "results/gmat_b1"
RESULTS_DIR = SCRIPT_DIR / "results/gmat_b1"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_b/b1_physics_fidelity/gmat"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B1_METRICS_COMPARISON.csv"

CASE_CSV_NAMES: dict[B1Case, str] = {
    B1Case.CASE1_TWOBODY: "gmat_b1_case1_twobody_state.csv",
    B1Case.CASE2_J2: "gmat_b1_case2_j2_state.csv",
    B1Case.CASE3_J2_DRAG: "gmat_b1_case3_j2_drag_state.csv",
    B1Case.CASE4_J2_DRAG_SRP: "gmat_b1_case4_j2_drag_srp_state.csv",
    B1Case.CASE5_ALL_SUPPORTED: "gmat_b1_case5_all_supported_state.csv",
}


@dataclass(frozen=True)
class GmatForceModelSpec:
    gravity_potential_file: str
    gravity_harmonics_degree: int
    gravity_harmonics_order: int
    drag_enabled: bool
    atmosphere_model: str | None
    srp_enabled: bool
    srp_model: str | None
    third_body_bodies: tuple[str, ...]
    integrator: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "gravity_potential_file": self.gravity_potential_file,
            "gravity_harmonics_degree": self.gravity_harmonics_degree,
            "gravity_harmonics_order": self.gravity_harmonics_order,
            "drag_enabled": self.drag_enabled,
            "atmosphere_model": self.atmosphere_model,
            "srp_enabled": self.srp_enabled,
            "srp_model": self.srp_model,
            "third_body_bodies": list(self.third_body_bodies),
            "integrator": self.integrator,
        }


def force_model_spec(case: B1Case) -> GmatForceModelSpec:
    integrator = "RungeKutta89, InitialStepSize=10 s, Accuracy=1e-12"
    if case == B1Case.CASE1_TWOBODY:
        return GmatForceModelSpec("JGM2.cof", 0, 0, False, None, False, None, (), integrator)
    if case == B1Case.CASE2_J2:
        return GmatForceModelSpec("JGM2.cof", 2, 0, False, None, False, None, (), integrator)
    if case == B1Case.CASE3_J2_DRAG:
        return GmatForceModelSpec(
            "JGM2.cof",
            2,
            0,
            True,
            "NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
            False,
            None,
            (),
            integrator,
        )
    if case == B1Case.CASE4_J2_DRAG_SRP:
        return GmatForceModelSpec(
            "JGM2.cof",
            2,
            0,
            True,
            "NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
            True,
            "Spherical SRP",
            (),
            integrator,
        )
    return GmatForceModelSpec(
        "JGM2.cof",
        2,
        0,
        True,
        "NRLMSISE-00 with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
        True,
        "Spherical SRP",
        ("Sun", "Luna"),
        integrator,
    )


def describe_gmat_case(case: B1Case) -> dict[str, Any]:
    base_forces = {
        B1Case.CASE1_TWOBODY: [
            "Earth central gravity via JGM-2, degree 0, order 0 (point-mass Earth)",
        ],
        B1Case.CASE2_J2: [
            "Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)",
        ],
        B1Case.CASE3_J2_DRAG: [
            "Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)",
            "Native drag with Cd=2.2, DragArea=0.01 m²",
            "NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
        ],
        B1Case.CASE4_J2_DRAG_SRP: [
            "Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)",
            "Native drag with Cd=2.2, DragArea=0.01 m²",
            "NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
            "Spherical solar radiation pressure with Cr=1.8, SRPArea=1.0 m²",
        ],
        B1Case.CASE5_ALL_SUPPORTED: [
            "Earth gravity via JGM-2, degree 2, order 0 (J2-only policy)",
            "Native drag with Cd=2.2, DragArea=0.01 m²",
            "NRLMSISE-00 atmosphere with CSSISpaceWeatherFile (SpaceWeather-All-v1.2.txt)",
            "Spherical solar radiation pressure with Cr=1.8, SRPArea=1.0 m²",
            "Sun and Luna point-mass third-body gravity",
        ],
    }
    increments = {
        B1Case.CASE1_TWOBODY: "Baseline case. Point-mass Earth gravity only (JGM-2 deg 0).",
        B1Case.CASE2_J2: "Adds J2 via JGM-2 degree 2, order 0.",
        B1Case.CASE3_J2_DRAG: "Adds native atmospheric drag (NRLMSISE-00).",
        B1Case.CASE4_J2_DRAG_SRP: "Adds spherical solar radiation pressure.",
        B1Case.CASE5_ALL_SUPPORTED: (
            "Adds Sun and Luna point-mass third-body gravity. Inherits all force models from Case 4."
        ),
    }
    previous_case = {
        B1Case.CASE1_TWOBODY: None,
        B1Case.CASE2_J2: B1Case.CASE1_TWOBODY,
        B1Case.CASE3_J2_DRAG: B1Case.CASE2_J2,
        B1Case.CASE4_J2_DRAG_SRP: B1Case.CASE3_J2_DRAG,
        B1Case.CASE5_ALL_SUPPORTED: B1Case.CASE4_J2_DRAG_SRP,
    }

    doc: dict[str, Any] = {
        "enabled_gmat_force_models": base_forces[case],
        "increment_vs_previous_case": increments[case],
        "previous_case_id": previous_case[case].value if previous_case[case] else None,
    }

    if case == B1Case.CASE5_ALL_SUPPORTED:
        doc["all_supported_gmat_definition"] = {
            "note": (
                "Cross-tool Case 5 label means 'all perturbations this tool supports natively "
                "for this benchmark'. The GMAT-specific definition is below."
            ),
            "inherits_from_case": B1Case.CASE4_J2_DRAG_SRP.value,
            "added_in_case5_vs_case4": [
                "Sun point-mass gravity",
                "Luna (Moon) point-mass gravity",
            ],
            "full_force_stack": base_forces[case],
            "not_added_in_case5": [
                "Higher-degree/order gravity field (beyond J2-only policy)",
                "Solid Earth tides or ocean tides on gravity field",
                "Additional planets as third bodies",
                "Relativistic corrections",
                "Earth albedo / infrared radiation pressure",
                "SPAD or facet SRP models",
            ],
        }

    return doc


def gmat_b1_physics_scope() -> dict[str, Any]:
    return {
        "covered_in_this_b1_run": [
            "Numerical propagation (Runge-Kutta 89)",
            "JGM-2 central gravity and J2-only harmonics policy",
            "NRLMSISE-00 atmospheric drag with CSSI space-weather file",
            "Spherical solar radiation pressure",
            "Sun and Luna third-body point-mass gravity",
            "7-day LEO state history export and drift metrics",
        ],
        "gmat_native_but_not_in_b1": [
            "Full gravity field to high degree/order (e.g. EGM96, tides)",
            "Alternative atmosphere models (MSISE90, JacchiaRoberts, exponential)",
            "SPAD file-based or facet SRP models",
            "Third-body perturbations from additional solar-system bodies",
            "Relativistic correction force models",
            "Attitude-dependent drag and SRP (requires attitude dynamics, B2 scope)",
            "Maneuver modeling and event detection beyond fixed-force propagation",
        ],
        "conclusion": (
            "GMAT B1 demonstrates a staged perturbation ladder using native GMAT force models "
            "aligned with the frozen cross-tool benchmark. It does not claim to exercise every "
            "physics feature GMAT supports."
        ),
    }


def parse_gmat_state_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    if not path.is_file():
        raise FileNotFoundError(f"GMAT state file not found: {path}")

    times_s: list[float] = []
    positions_km: list[list[float]] = []
    velocities_kms: list[list[float]] = []
    raw_rows = 0

    with path.open(encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or "ElapsedSecs" in stripped:
                continue
            parts = stripped.split()
            if len(parts) < 7:
                continue
            try:
                times_s.append(float(parts[0]))
                positions_km.append([float(parts[1]), float(parts[2]), float(parts[3])])
                velocities_kms.append([float(parts[4]), float(parts[5]), float(parts[6])])
                raw_rows += 1
            except ValueError:
                continue

    if not times_s:
        raise ValueError(f"No data rows parsed from {path}")

    times = np.array(times_s, dtype=float)
    positions_m = np.array(positions_km, dtype=float) * KM_TO_M
    velocities_mps = np.array(velocities_kms, dtype=float) * KM_TO_M

    order = np.argsort(times)
    times = times[order]
    positions_m = positions_m[order]
    velocities_mps = velocities_mps[order]

    unique_times: list[float] = []
    unique_positions: list[np.ndarray] = []
    unique_velocities: list[np.ndarray] = []
    for t_s, r_m, v_mps in zip(times, positions_m, velocities_mps):
        if unique_times and abs(t_s - unique_times[-1]) < 1e-3:
            unique_positions[-1] = r_m
            unique_velocities[-1] = v_mps
        else:
            unique_times.append(float(t_s))
            unique_positions.append(r_m)
            unique_velocities.append(v_mps)

    return (
        np.array(unique_times, dtype=float),
        np.vstack(unique_positions),
        np.vstack(unique_velocities),
        raw_rows,
    )


def resample_to_cadence(
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    config: B1Config,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    target_times = np.arange(
        0.0,
        config.simulated_duration_s + 0.5 * config.output_sample_s,
        config.output_sample_s,
        dtype=float,
    )
    resampled_positions: list[np.ndarray] = []
    resampled_velocities: list[np.ndarray] = []
    for target_t in target_times:
        index = int(np.argmin(np.abs(times_s - target_t)))
        resampled_positions.append(positions_m[index])
        resampled_velocities.append(velocities_mps[index])
    return target_times, np.vstack(resampled_positions), np.vstack(resampled_velocities)


def rv2elem(
    position_m: np.ndarray,
    velocity_mps: np.ndarray,
) -> tuple[float, float, float, float, float, float]:
    """Return a, e, i, raan, arg_perigee, true_anomaly (angles in radians)."""
    r = np.asarray(position_m, dtype=float).reshape(3)
    v = np.asarray(velocity_mps, dtype=float).reshape(3)
    rn = float(np.linalg.norm(r))
    vn = float(np.linalg.norm(v))
    h_vec = np.cross(r, v)
    hn = float(np.linalg.norm(h_vec))
    e_vec = np.cross(v, h_vec) / MU_EARTH - r / rn
    e = float(np.linalg.norm(e_vec))
    energy = vn**2 / 2.0 - MU_EARTH / rn
    a = -MU_EARTH / (2.0 * energy)

    i = math.acos(np.clip(h_vec[2] / hn, -1.0, 1.0))
    n_vec = np.cross(np.array([0.0, 0.0, 1.0]), h_vec)
    nn = float(np.linalg.norm(n_vec))
    if nn > 1e-12:
        raan = math.acos(np.clip(n_vec[0] / nn, -1.0, 1.0))
        if n_vec[1] < 0.0:
            raan = 2.0 * math.pi - raan
    else:
        raan = 0.0

    if nn > 1e-12 and e > 1e-12:
        arg_perigee = math.acos(np.clip(float(np.dot(n_vec, e_vec)) / (nn * e), -1.0, 1.0))
        if e_vec[2] < 0.0:
            arg_perigee = 2.0 * math.pi - arg_perigee
    else:
        arg_perigee = 0.0

    if e > 1e-12:
        true_anomaly = math.acos(np.clip(float(np.dot(e_vec, r)) / (e * rn), -1.0, 1.0))
        if float(np.dot(r, v)) < 0.0:
            true_anomaly = 2.0 * math.pi - true_anomaly
    elif nn > 1e-12:
        true_anomaly = math.acos(np.clip(float(np.dot(n_vec, r)) / (nn * rn), -1.0, 1.0))
        if r[2] < 0.0:
            true_anomaly = 2.0 * math.pi - true_anomaly
    else:
        true_anomaly = 0.0

    return a, e, i, raan, arg_perigee, true_anomaly


def rows_from_state_history(
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for time_s, position, velocity in zip(times_s, positions_m, velocities_mps):
        r_vec = np.asarray(position, dtype=float).reshape(3)
        v_vec = np.asarray(velocity, dtype=float).reshape(3)
        a, e, inc, raan, argp, nu = rv2elem(r_vec, v_vec)
        arg_lat_deg = math.degrees(argp + nu)
        lat_deg, lon_deg = eci_position_to_subpoint_lat_lon(r_vec, float(time_s))
        rows.append(
            {
                "time_s": float(time_s),
                "x_m": float(r_vec[0]),
                "y_m": float(r_vec[1]),
                "z_m": float(r_vec[2]),
                "vx_m_s": float(v_vec[0]),
                "vy_m_s": float(v_vec[1]),
                "vz_m_s": float(v_vec[2]),
                "semi_major_axis_m": float(a),
                "eccentricity": float(e),
                "inclination_deg": math.degrees(inc),
                "raan_deg": math.degrees(raan),
                "arg_perigee_deg": math.degrees(argp),
                "true_anomaly_deg": math.degrees(nu),
                "argument_of_latitude_deg": arg_lat_deg,
                "lat_deg": lat_deg,
                "lon_deg": lon_deg,
            }
        )
    return rows


def write_state_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "semi_major_axis_m",
        "eccentricity",
        "inclination_deg",
        "raan_deg",
        "arg_perigee_deg",
        "true_anomaly_deg",
        "argument_of_latitude_deg",
        "lat_deg",
        "lon_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_ground_track_plot(path: Path, rows: list[dict[str, Any]], title: str) -> None:
    lats = [row["lat_deg"] for row in rows]
    lons = [row["lon_deg"] for row in rows]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(lons, lats, linewidth=0.8)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_altitude_plot(path: Path, rows: list[dict[str, Any]], title: str) -> None:
    times_h = [row["time_s"] / 3600.0 for row in rows]
    altitudes_km = [(row["semi_major_axis_m"] - R_EARTH) / 1000.0 for row in rows]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times_h, altitudes_km, linewidth=1.0)
    ax.set_xlabel("Time [hours]")
    ax.set_ylabel("Altitude [km]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run_id_for_case(case: B1Case) -> str:
    return f"b1_gmat_{case.value.removeprefix('b1_')}"


def write_case_outputs(
    case: B1Case,
    rows: list[dict[str, Any]],
    spec: GmatForceModelSpec,
    source_csv: Path,
    raw_row_count: int,
    drift_metrics: dict[str, float] | None,
    config: B1Config,
) -> dict[str, Any]:
    case_dir = RESULTS_DIR / case.value
    data_case_dir = DATA_DIR / case.value
    case_dir.mkdir(parents=True, exist_ok=True)
    data_case_dir.mkdir(parents=True, exist_ok=True)

    state_csv = case_dir / "state_history.csv"
    write_state_csv(state_csv, rows)
    write_state_csv(data_case_dir / "state_history.csv", rows)

    ground_track_plot = case_dir / "ground_track.png"
    altitude_plot = case_dir / "altitude_vs_time.png"
    save_ground_track_plot(ground_track_plot, rows, f"GMAT B1 {case.value} ground track")
    save_altitude_plot(altitude_plot, rows, f"GMAT B1 {case.value} altitude")

    summary = {
        "benchmark": "B1",
        "simulator": "GMAT",
        "run_id": run_id_for_case(case),
        "case_id": case.value,
        "status": "pass" if len(rows) == config.output_sample_count else "partial",
        "epoch_utc": config.mission_epoch_utc_iso,
        "simulated_duration_s": rows[-1]["time_s"],
        "sample_count": len(rows),
        "expected_sample_count": config.output_sample_count,
        "source_gmat_csv": str(source_csv),
        "raw_gmat_csv_rows_parsed": raw_row_count,
        "force_model": spec.to_dict(),
        "spacecraft": {
            "dry_mass_kg": config.dry_mass_kg,
            "drag_coefficient": config.drag_coefficient,
            "srp_reflectivity": config.srp_reflectivity,
            "drag_area_m2": config.drag_area_m2,
            "srp_area_m2": config.srp_area_m2,
            "attitude_policy": "CoordinateSystemFixed (isotropic drag/SRP coefficients)",
        },
        "drift_metrics_vs_case1": drift_metrics or {},
        "gmat_perturbation_documentation": describe_gmat_case(case),
        "artifacts": {
            "state_history_csv": str(state_csv),
            "ground_track_plot": str(ground_track_plot),
            "altitude_plot": str(altitude_plot),
        },
    }

    with (case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    with (data_case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary


def merge_metrics_csv(gmat_rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "simulator",
        "run_id",
        "case_id",
        "position_drift_vs_case1_m",
        "ground_track_lon_drift_deg",
        "raan_change_deg",
        "altitude_change_m",
        "along_track_phase_drift_deg",
        "final_altitude_m",
        "wall_clock_runtime_s",
        "runtime_ratio",
        "drag_enabled",
        "srp_enabled",
        "third_body_enabled",
    ]

    existing_rows: list[dict[str, Any]] = []
    if METRICS_CSV.exists():
        with METRICS_CSV.open(newline="", encoding="utf-8") as handle:
            existing_rows = [
                row for row in csv.DictReader(handle) if row.get("simulator") != "GMAT"
            ]

    METRICS_CSV.parent.mkdir(parents=True, exist_ok=True)
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(existing_rows)
        writer.writerows(gmat_rows)


def write_case5_definition_markdown(
    path: Path,
    case5_doc: dict[str, Any],
    physics_scope: dict[str, Any],
) -> None:
    lines = [
        "# GMAT B1 Case 5 — all supported (GMAT-specific definition)",
        "",
        "Cross-tool benchmark documents use the label **all supported** generically. "
        "This file defines exactly what that means for **GMAT** in Benchmark B1.",
        "",
        "## Inherits from",
        "",
        f"- `{case5_doc.get('inherits_from_case', 'b1_case4_j2_drag_srp')}` (full Case 4 force stack)",
        "",
        "## Added in Case 5 vs Case 4",
        "",
        *[f"- {item}" for item in case5_doc.get("added_in_case5_vs_case4", [])],
        "",
        "## Full GMAT force stack in Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("full_force_stack", [])],
        "",
        "## Explicitly not added in GMAT Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("not_added_in_case5", [])],
        "",
        "## Does GMAT B1 cover all GMAT physics fidelity features?",
        "",
        physics_scope["conclusion"],
        "",
        "### Covered in this B1 run",
        "",
        *[f"- {item}" for item in physics_scope["covered_in_this_b1_run"]],
        "",
        "### GMAT-native but not in B1",
        "",
        *[f"- {item}" for item in physics_scope["gmat_native_but_not_in_b1"]],
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_run_log(
    path: Path,
    config: B1Config,
    summaries: list[dict[str, Any]],
    input_dir: Path,
    postprocess_runtime_s: float,
    gmat_version: str,
    gmat_wall_clock_no_plots_s: float | None,
    gmat_wall_clock_with_plots_s: float | None,
) -> None:
    run_date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    lines = [
        f"# GMAT B1 run log ({run_date})",
        "",
        "## Configuration",
        "",
        "- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`",
        f"- GMAT script: `benchmarks/benchmark_b/gmat/benchmark_b1_gmat.script`",
        f"- Input CSV directory: `{input_dir}`",
        f"- Duration: {config.simulated_duration_s / 86400.0:.1f} days",
        f"- Output cadence: {config.output_sample_s} s",
        f"- GMAT version: {gmat_version}",
        f"- Python post-processor: `{Path(__file__).relative_to(WORKSPACE_ROOT)}`",
        "",
        "## Cases processed",
        "",
    ]
    for summary in summaries:
        fm = summary["force_model"]
        lines.extend(
            [
                f"### {summary['case_id']}",
                "",
                f"- Run ID: `{summary['run_id']}`",
                f"- Status: {summary['status']} ({summary['sample_count']} samples)",
                f"- Source CSV: `{summary['source_gmat_csv']}`",
                f"- Gravity: {fm['gravity_potential_file']} "
                f"{fm['gravity_harmonics_degree']}x{fm['gravity_harmonics_order']}",
                f"- Drag: {fm['drag_enabled']} ({fm['atmosphere_model'] or 'n/a'})",
                f"- SRP: {fm['srp_enabled']} ({fm['srp_model'] or 'n/a'})",
                f"- Third body: {', '.join(fm['third_body_bodies']) if fm['third_body_bodies'] else 'none'}",
                "",
            ]
        )
        gmat_doc = summary.get("gmat_perturbation_documentation", {})
        if gmat_doc:
            lines.extend(
                [
                    "GMAT force models enabled:",
                    *[f"- {item}" for item in gmat_doc.get("enabled_gmat_force_models", [])],
                    "",
                    f"Increment vs previous case: {gmat_doc.get('increment_vs_previous_case', 'n/a')}",
                    "",
                ]
            )
        if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value:
            case5 = gmat_doc.get("all_supported_gmat_definition", {})
            lines.extend(
                [
                    "**Case 5 — all supported (GMAT-specific definition):**",
                    f"- Inherits from: `{case5.get('inherits_from_case', 'n/a')}`",
                    "- Added in Case 5 vs Case 4:",
                    *[f"  - {item}" for item in case5.get("added_in_case5_vs_case4", [])],
                    "- Not included in GMAT Case 5:",
                    *[f"  - {item}" for item in case5.get("not_added_in_case5", [])],
                    "",
                ]
            )

        drift = summary.get("drift_metrics_vs_case1", {})
        if drift:
            lines.extend(
                [
                    "Drift vs Case 1 (final epoch):",
                    f"- Position drift: {drift['position_drift_vs_case1_m']:.3f} m",
                    f"- Ground-track lon drift: {drift['ground_track_lon_drift_deg']:.6f} deg",
                    f"- RAAN change: {drift['raan_change_deg']:.6f} deg",
                    f"- Altitude change: {drift['altitude_change_m']:.3f} m",
                    "",
                ]
            )

    scope = gmat_b1_physics_scope()
    lines.extend(
        [
            "## GMAT B1 physics scope",
            "",
            scope["conclusion"],
            "",
            "Covered in this B1 run:",
            *[f"- {item}" for item in scope["covered_in_this_b1_run"]],
            "",
            "GMAT-native features not exercised in B1:",
            *[f"- {item}" for item in scope["gmat_native_but_not_in_b1"]],
            "",
            "## Runtime",
            "",
            f"- Python post-processing: {postprocess_runtime_s:.3f} s",
            "- GMAT mission: all five cases run sequentially in one script execution",
        ]
    )
    case_count = len(B1Case)
    if gmat_wall_clock_no_plots_s is not None:
        per_case = gmat_wall_clock_no_plots_s / case_count
        lines.extend(
            [
                f"- GMAT GUI total (no live plots): {gmat_wall_clock_no_plots_s:.3f} s",
                f"- GMAT per-case amortized (no plots): {per_case:.3f} s",
                (
                    f"- Runtime ratio per case (no plots): "
                    f"{config.simulated_duration_s / per_case:.1f}"
                ),
            ]
        )
    if gmat_wall_clock_with_plots_s is not None:
        per_case_plots = gmat_wall_clock_with_plots_s / case_count
        lines.extend(
            [
                (
                    "- GMAT GUI total (OpenFrames orbit view + ground track live plots): "
                    f"{gmat_wall_clock_with_plots_s:.1f} s"
                ),
                f"- GMAT per-case amortized (with plots): {per_case_plots:.3f} s",
                (
                    f"- Runtime ratio per case (with plots): "
                    f"{config.simulated_duration_s / per_case_plots:.1f}"
                ),
            ]
        )
    if gmat_wall_clock_no_plots_s is None and gmat_wall_clock_with_plots_s is None:
        lines.append(
            "- GMAT GUI wall clock: not recorded "
            "(pass --gmat-wall-clock-no-plots-s and/or --gmat-wall-clock-with-plots-s)"
        )
    if gmat_wall_clock_no_plots_s is not None:
        lines.append(
            "- Metrics CSV uses per-case amortized no-plots wall clock for cross-tool comparison"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def process_case(
    case: B1Case,
    input_dir: Path,
    config: B1Config,
) -> tuple[list[dict[str, Any]], GmatForceModelSpec, Path, int]:
    csv_name = CASE_CSV_NAMES[case]
    source_csv = input_dir / csv_name
    times_s, positions_m, velocities_mps, raw_rows = parse_gmat_state_csv(source_csv)
    target_times, positions_m, velocities_mps = resample_to_cadence(
        times_s, positions_m, velocities_mps, config
    )
    rows = rows_from_state_history(target_times, positions_m, velocities_mps)
    return rows, force_model_spec(case), source_csv, raw_rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Post-process GMAT B1 Physics Fidelity CSV exports.")
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help=f"Directory containing gmat_b1_case*_state.csv files (default: {DEFAULT_INPUT_DIR})",
    )
    parser.add_argument(
        "--gmat-version",
        default="R2026a (user GUI run)",
        help="GMAT version string recorded in the run log",
    )
    parser.add_argument(
        "--gmat-wall-clock-no-plots-s",
        type=float,
        default=None,
        help="GMAT Total Run Time for all five cases without live plots (seconds)",
    )
    parser.add_argument(
        "--gmat-wall-clock-with-plots-s",
        type=float,
        default=None,
        help=(
            "GMAT wall clock with OpenFrames orbit view and ground track enabled "
            "(seconds, all five cases)"
        ),
    )
    parser.add_argument(
        "--wall-clock-total-s",
        type=float,
        default=None,
        help="Deprecated alias for --gmat-wall-clock-no-plots-s",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_dir = args.input_dir.resolve()
    gmat_wall_clock_no_plots_s = args.gmat_wall_clock_no_plots_s
    if gmat_wall_clock_no_plots_s is None and args.wall_clock_total_s is not None:
        gmat_wall_clock_no_plots_s = args.wall_clock_total_s
    gmat_wall_clock_with_plots_s = args.gmat_wall_clock_with_plots_s
    config = load_b1_config_from_env()
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    total_start_s = time.perf_counter()
    print(f"Post-processing GMAT B1 Physics Fidelity from {input_dir} ...")

    histories: dict[B1Case, list[dict[str, Any]]] = {}
    prepared: list[dict[str, Any]] = []

    for case in B1Case:
        print(f"  Parsing {CASE_CSV_NAMES[case]} ...")
        rows, spec, source_csv, raw_rows = process_case(case, input_dir, config)
        histories[case] = rows
        prepared.append(
            {
                "case": case,
                "rows": rows,
                "spec": spec,
                "source_csv": source_csv,
                "raw_rows": raw_rows,
            }
        )

    case1_history = histories_from_rows(histories[B1Case.CASE1_TWOBODY])
    final_summaries: list[dict[str, Any]] = []
    metrics_rows: list[dict[str, Any]] = []

    per_case_wall_clock = (
        gmat_wall_clock_no_plots_s / len(B1Case)
        if gmat_wall_clock_no_plots_s is not None
        else None
    )

    for item in prepared:
        case = item["case"]
        rows = item["rows"]
        drift_metrics = None
        if case != B1Case.CASE1_TWOBODY:
            drift_metrics = compute_endpoint_drift_metrics(
                histories_from_rows(rows),
                case1_history,
            )
        summary = write_case_outputs(
            case,
            rows,
            item["spec"],
            item["source_csv"],
            item["raw_rows"],
            drift_metrics,
            config,
        )
        final_summaries.append(summary)

        fm = summary["force_model"]
        runtime_ratio = ""
        if per_case_wall_clock and per_case_wall_clock > 0:
            runtime_ratio = rows[-1]["time_s"] / per_case_wall_clock
        metrics_rows.append(
            {
                "simulator": "GMAT",
                "run_id": summary["run_id"],
                "case_id": summary["case_id"],
                "position_drift_vs_case1_m": (drift_metrics or {}).get("position_drift_vs_case1_m", 0.0),
                "ground_track_lon_drift_deg": (drift_metrics or {}).get("ground_track_lon_drift_deg", 0.0),
                "raan_change_deg": (drift_metrics or {}).get(
                    "raan_change_deg", rows[-1]["raan_deg"] - rows[0]["raan_deg"]
                ),
                "altitude_change_m": (drift_metrics or {}).get(
                    "altitude_change_m",
                    rows[-1]["semi_major_axis_m"] - rows[0]["semi_major_axis_m"],
                ),
                "along_track_phase_drift_deg": (drift_metrics or {}).get("along_track_phase_drift_deg", 0.0),
                "final_altitude_m": (drift_metrics or {}).get(
                    "final_altitude_m", rows[-1]["semi_major_axis_m"] - R_EARTH
                ),
                "wall_clock_runtime_s": per_case_wall_clock if per_case_wall_clock is not None else "",
                "runtime_ratio": runtime_ratio if runtime_ratio != "" else "",
                "drag_enabled": fm["drag_enabled"],
                "srp_enabled": fm["srp_enabled"],
                "third_body_enabled": bool(fm["third_body_bodies"]),
            }
        )

    merge_metrics_csv(metrics_rows)

    physics_scope = gmat_b1_physics_scope()
    case5_summary = next(
        summary for summary in final_summaries if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value
    )
    case5_doc = case5_summary["gmat_perturbation_documentation"]["all_supported_gmat_definition"]

    postprocess_runtime_s = time.perf_counter() - total_start_s
    aggregate_summary = {
        "benchmark": "B1",
        "simulator": "GMAT",
        "status": "pass" if all(s["status"] == "pass" for s in final_summaries) else "partial",
        "cases": [summary["case_id"] for summary in final_summaries],
        "summaries": final_summaries,
        "metrics_csv": str(METRICS_CSV),
        "parameter_table": "data/benchmark_b/B1_PARAMETER_TABLE.md",
        "gmat_version": args.gmat_version,
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "input_dir": str(input_dir),
        "gmat_case5_all_supported_definition": case5_doc,
        "gmat_b1_physics_scope": physics_scope,
        "runtime_s": {
            "post_processing": postprocess_runtime_s,
            "gmat_gui_total_no_plots_s": gmat_wall_clock_no_plots_s,
            "gmat_gui_total_with_plots_s": gmat_wall_clock_with_plots_s,
            "gmat_gui_per_case_no_plots_s": per_case_wall_clock,
            "gmat_gui_per_case_with_plots_s": (
                gmat_wall_clock_with_plots_s / len(B1Case)
                if gmat_wall_clock_with_plots_s is not None
                else None
            ),
        },
    }
    with (DATA_DIR / "gmat_b1_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(aggregate_summary, handle, indent=2)

    write_case5_definition_markdown(DATA_DIR / "gmat_b1_case5_definition.md", case5_doc, physics_scope)
    write_run_log(
        DATA_DIR / "gmat_b1_run_log.md",
        config,
        final_summaries,
        input_dir,
        postprocess_runtime_s,
        args.gmat_version,
        gmat_wall_clock_no_plots_s,
        gmat_wall_clock_with_plots_s,
    )

    print(f"GMAT B1 post-processing complete. Results: {DATA_DIR}")
    print(f"Metrics CSV: {METRICS_CSV}")
    print(f"Post-processing runtime: {postprocess_runtime_s:.3f} s")
    for summary in final_summaries:
        drift = summary.get("drift_metrics_vs_case1", {})
        pos = drift.get("position_drift_vs_case1_m", 0.0)
        print(f"  {summary['case_id']}: {summary['sample_count']} samples, drift vs Case 1 = {pos:.1f} m")


if __name__ == "__main__":
    main()
