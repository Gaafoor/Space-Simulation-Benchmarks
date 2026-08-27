"""Benchmark A1 - GMAT analytical two-body validation.

Parses GMAT report CSV (km, km/s), compares to the same Kepler reference as
Basilisk A1, and writes comparison artifacts under data/benchmark_a/gmat_a1/.

Expects gmat_a1_state.csv with ~145 rows at 0, 600, ..., 86400 s from the
fixed GMAT script (A1_State_Report.Add = {}).
"""

from __future__ import annotations

import csv
import json
import math
import platform
from dataclasses import asdict, dataclass
from datetime import date
from pathlib import Path

import numpy as np

from a1_analytical_reference import A1Config, analytical_two_body_state

KM_TO_M = 1000.0
EXPECTED_SAMPLES = 145
WALL_CLOCK_RUNTIME_S = 0.829  # GMAT GUI run 2026-06-03 (plots on, clean CSV)


def resolve_paths() -> tuple[Path, Path]:
    repo_root = Path(__file__).resolve().parents[2]
    gmat_csv = repo_root / "benchmarks" / "benchmark_a" / "gmat" / "gmat_a1_state.csv"
    output_dir = repo_root / "data" / "benchmark_a" / "gmat_a1"
    output_dir.mkdir(parents=True, exist_ok=True)
    return gmat_csv, output_dir


def parse_gmat_state_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(
            f"GMAT state file not found: {path}\n"
            "Run benchmark_a1_gmat.script in GMAT first (after fixing A1_State_Report.Add)."
        )

    times_s: list[float] = []
    positions_km: list[list[float]] = []
    velocities_kms: list[list[float]] = []

    with path.open(encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not stripped or stripped.startswith("CubeSat_A1"):
                continue
            parts = stripped.split()
            if len(parts) < 7:
                continue
            try:
                t_s = float(parts[0])
                position_km = [float(parts[1]), float(parts[2]), float(parts[3])]
                velocity_kms = [float(parts[4]), float(parts[5]), float(parts[6])]
            except ValueError:
                continue
            times_s.append(t_s)
            positions_km.append(position_km)
            velocities_kms.append(velocity_kms)

    if not times_s:
        raise ValueError(f"No data rows parsed from {path}")

    times = np.array(times_s, dtype=float)
    positions_m = np.array(positions_km, dtype=float) * KM_TO_M
    velocities_mps = np.array(velocities_kms, dtype=float) * KM_TO_M

    # Deduplicate equal epochs (keep last row).
    order = np.argsort(times)
    times = times[order]
    positions_m = positions_m[order]
    velocities_mps = velocities_mps[order]
    unique_times: list[float] = []
    unique_positions: list[np.ndarray] = []
    unique_velocities: list[np.ndarray] = []
    for t_s, r_m, v_mps in zip(times, positions_m, velocities_mps):
        if unique_times and abs(t_s - unique_times[-1]) < 1e-6:
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
    )


def resample_to_cadence(
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    config: A1Config,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    target_times = np.arange(
        0.0, config.simulated_duration_s + 1e-9, config.output_sample_s, dtype=float
    )
    resampled_positions: list[np.ndarray] = []
    resampled_velocities: list[np.ndarray] = []
    for target_t in target_times:
        index = int(np.argmin(np.abs(times_s - target_t)))
        resampled_positions.append(positions_m[index])
        resampled_velocities.append(velocities_mps[index])
    return target_times, np.vstack(resampled_positions), np.vstack(resampled_velocities)


def write_gmat_outputs(
    output_dir: Path,
    config: A1Config,
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    raw_sample_count: int,
) -> dict[str, float | str | int]:
    comparison_path = output_dir / "gmat_a1_state_comparison.csv"
    rows: list[dict[str, float]] = []

    for t_s, r_gmat, v_gmat in zip(times_s, positions_m, velocities_mps):
        r_ref, v_ref = analytical_two_body_state(config, float(t_s))
        rows.append(
            {
                "time_s": float(t_s),
                "gmat_rx_m": float(r_gmat[0]),
                "gmat_ry_m": float(r_gmat[1]),
                "gmat_rz_m": float(r_gmat[2]),
                "gmat_vx_mps": float(v_gmat[0]),
                "gmat_vy_mps": float(v_gmat[1]),
                "gmat_vz_mps": float(v_gmat[2]),
                "reference_rx_m": float(r_ref[0]),
                "reference_ry_m": float(r_ref[1]),
                "reference_rz_m": float(r_ref[2]),
                "reference_vx_mps": float(v_ref[0]),
                "reference_vy_mps": float(v_ref[1]),
                "reference_vz_mps": float(v_ref[2]),
                "position_difference_vs_reference_m": float(np.linalg.norm(r_gmat - r_ref)),
                "velocity_difference_vs_reference_mps": float(np.linalg.norm(v_gmat - v_ref)),
            }
        )

    with comparison_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    max_pos_error = max(row["position_difference_vs_reference_m"] for row in rows)
    final_pos_error = rows[-1]["position_difference_vs_reference_m"]
    max_vel_error = max(row["velocity_difference_vs_reference_mps"] for row in rows)
    final_vel_error = rows[-1]["velocity_difference_vs_reference_mps"]
    runtime_ratio = config.simulated_duration_s / WALL_CLOCK_RUNTIME_S

    summary = {
        "benchmark_id": "A1",
        "simulator": "GMAT",
        "gmat_version": "R2026a (64-bit)",
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "dynamics_model": "two-body Earth central gravity only (Degree/Order 0)",
        "spice_used": "no",
        "j2_enabled": "no",
        "drag_enabled": "no",
        "srp_enabled": "no",
        "third_body_enabled": "no",
        "simulated_duration_s": config.simulated_duration_s,
        "wall_clock_runtime_s": WALL_CLOCK_RUNTIME_S,
        "runtime_ratio": runtime_ratio,
        "raw_csv_rows_parsed": raw_sample_count,
        "samples": len(rows),
        "position_difference_vs_reference_final_m": final_pos_error,
        "position_difference_vs_reference_max_m": max_pos_error,
        "velocity_difference_vs_reference_final_mps": final_vel_error,
        "velocity_difference_vs_reference_max_mps": max_vel_error,
        "state_comparison_csv": str(comparison_path),
    }

    summary_path = output_dir / "gmat_a1_summary.json"
    with summary_path.open("w", encoding="utf-8") as file:
        json.dump({"config": asdict(config), "summary": summary}, file, indent=2)

    status = "pass" if len(rows) == EXPECTED_SAMPLES else "partial"
    log_path = output_dir / "gmat_a1_run_log.md"
    with log_path.open("w", encoding="utf-8") as file:
        file.write("# Benchmark A1 GMAT Run Log\n\n")
        file.write("## Status\n\n")
        file.write(f"- Result: {status}\n")
        file.write(f"- Expected samples: {EXPECTED_SAMPLES} (600 s cadence)\n")
        file.write(f"- Parsed samples used: {len(rows)}\n")
        file.write("- SPICE used: no\n\n")
        file.write("## Summary\n\n")
        for key, value in summary.items():
            file.write(f"- {key}: {value}\n")

    eval_path = output_dir / "gmat_a1_evaluation_sheet.csv"
    write_evaluation_sheet(eval_path, summary, status)
    write_cross_tool_comparison(
        output_dir.parent, config, times_s, positions_m, velocities_mps, summary
    )

    return summary


def write_evaluation_sheet(
    path: Path, summary: dict[str, float | str | int], status: str
) -> None:
    rows = [
        ["date", "simulator", "version", "operator", "run_id", "status", "notes"],
        [
            date.today().isoformat(),
            "GMAT",
            str(summary.get("gmat_version", "")),
            "user",
            "a1_gmat_001",
            status,
            "Variant A1 analytical two-body; wall time from GMAT GUI log",
        ],
        [],
        ["benchmark_specific_metric", "value", "unit", "comment"],
        [
            "position_difference_vs_reference_final",
            summary["position_difference_vs_reference_final_m"],
            "meters",
            "after 24 hours",
        ],
        [
            "position_difference_vs_reference_max",
            summary["position_difference_vs_reference_max_m"],
            "meters",
            "max over sampled trajectory",
        ],
        [
            "velocity_difference_vs_reference_final",
            summary["velocity_difference_vs_reference_final_mps"],
            "m_per_s",
            "after 24 hours",
        ],
        [
            "velocity_difference_vs_reference_max",
            summary["velocity_difference_vs_reference_max_mps"],
            "m_per_s",
            "max over sampled trajectory",
        ],
        ["simulated_duration", 24, "hours", ""],
        ["wall_clock_runtime", summary["wall_clock_runtime_s"], "seconds", "GMAT GUI with plots"],
        ["runtime_ratio", summary["runtime_ratio"], "sim_time_over_wall_time", ""],
        ["samples", summary["samples"], "count", "600 s cadence"],
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows(rows)


def load_basilisk_comparison(
    basilisk_csv: Path,
) -> dict[float, dict[str, float | np.ndarray]]:
    by_time: dict[float, dict[str, float | np.ndarray]] = {}
    with basilisk_csv.open(encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            t_s = float(row["time_s"])
            by_time[round(t_s, 3)] = {
                "r_bsk": np.array(
                    [
                        float(row["basilisk_rx_m"]),
                        float(row["basilisk_ry_m"]),
                        float(row["basilisk_rz_m"]),
                    ],
                    dtype=float,
                ),
                "v_bsk": np.array(
                    [
                        float(row["basilisk_vx_mps"]),
                        float(row["basilisk_vy_mps"]),
                        float(row["basilisk_vz_mps"]),
                    ],
                    dtype=float,
                ),
                "pos_err_bsk_ref_m": float(row["position_difference_vs_reference_m"]),
                "vel_err_bsk_ref_mps": float(row["velocity_difference_vs_reference_mps"]),
            }
    return by_time


def write_cross_tool_comparison(
    benchmark_a_dir: Path,
    config: A1Config,
    gmat_times_s: np.ndarray,
    gmat_positions_m: np.ndarray,
    gmat_velocities_mps: np.ndarray,
    gmat_summary: dict[str, float | str | int],
) -> None:
    basilisk_csv = benchmark_a_dir / "basilisk_a1" / "basilisk_a1_state_comparison.csv"
    basilisk_summary_path = benchmark_a_dir / "basilisk_a1" / "basilisk_a1_summary.json"
    if not basilisk_csv.is_file():
        return

    bsk_by_time = load_basilisk_comparison(basilisk_csv)
    bsk_summary: dict[str, object] = {}
    if basilisk_summary_path.is_file():
        with basilisk_summary_path.open(encoding="utf-8") as file:
            bsk_summary = json.load(file).get("summary", {})

    cross_rows: list[dict[str, float]] = []
    three_way_rows: list[dict[str, float]] = []
    for t_s, r_gmat, v_gmat in zip(gmat_times_s, gmat_positions_m, gmat_velocities_mps):
        key = round(float(t_s), 3)
        if key not in bsk_by_time:
            continue
        bsk = bsk_by_time[key]
        r_bsk = bsk["r_bsk"]
        v_bsk = bsk["v_bsk"]
        assert isinstance(r_bsk, np.ndarray) and isinstance(v_bsk, np.ndarray)

        r_ref, v_ref = analytical_two_body_state(config, float(t_s))
        pos_gmat_ref = float(np.linalg.norm(r_gmat - r_ref))
        vel_gmat_ref = float(np.linalg.norm(v_gmat - v_ref))
        pos_bsk_gmat = float(np.linalg.norm(r_gmat - r_bsk))
        vel_bsk_gmat = float(np.linalg.norm(v_gmat - v_bsk))

        cross_rows.append(
            {
                "time_s": float(t_s),
                "position_difference_gmat_vs_basilisk_m": pos_bsk_gmat,
                "velocity_difference_gmat_vs_basilisk_mps": vel_bsk_gmat,
            }
        )
        three_way_rows.append(
            {
                "time_s": float(t_s),
                "position_difference_basilisk_vs_reference_m": float(bsk["pos_err_bsk_ref_m"]),
                "velocity_difference_basilisk_vs_reference_mps": float(bsk["vel_err_bsk_ref_mps"]),
                "position_difference_gmat_vs_reference_m": pos_gmat_ref,
                "velocity_difference_gmat_vs_reference_mps": vel_gmat_ref,
                "position_difference_gmat_vs_basilisk_m": pos_bsk_gmat,
                "velocity_difference_gmat_vs_basilisk_mps": vel_bsk_gmat,
            }
        )

    if not cross_rows:
        return

    out_path = benchmark_a_dir / "a1_cross_tool_comparison.csv"
    with out_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(cross_rows[0].keys()))
        writer.writeheader()
        writer.writerows(cross_rows)

    three_way_path = benchmark_a_dir / "a1_three_way_comparison.csv"
    with three_way_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(three_way_rows[0].keys()))
        writer.writeheader()
        writer.writerows(three_way_rows)

    def _max(key: str) -> float:
        return max(float(row[key]) for row in three_way_rows)

    report_path = benchmark_a_dir / "A1_COMPARISON_REPORT.md"
    with report_path.open("w", encoding="utf-8") as file:
        file.write("# Benchmark A1 — Basilisk vs GMAT vs Analytical Reference\n\n")
        file.write("Independent Kepler two-body reference (WGS84, 500 km SSO-like LEO, 24 h, 600 s samples).\n\n")
        file.write("## Summary\n\n")
        file.write("| Metric | Basilisk | GMAT |\n")
        file.write("|--------|----------|------|\n")
        file.write(
            f"| Max position error vs reference [m] | "
            f"{bsk_summary.get('position_difference_vs_reference_max_m', 'N/A')} | "
            f"{gmat_summary['position_difference_vs_reference_max_m']:.6f} |\n"
        )
        file.write(
            f"| Final position error vs reference [m] | "
            f"{bsk_summary.get('position_difference_vs_reference_final_m', 'N/A')} | "
            f"{gmat_summary['position_difference_vs_reference_final_m']:.6f} |\n"
        )
        file.write(
            f"| Max velocity error vs reference [m/s] | "
            f"{bsk_summary.get('velocity_difference_vs_reference_max_mps', 'N/A')} | "
            f"{gmat_summary['velocity_difference_vs_reference_max_mps']:.9f} |\n"
        )
        file.write(
            f"| Wall-clock runtime [s] | "
            f"{bsk_summary.get('wall_clock_runtime_s', 'N/A')} | "
            f"{gmat_summary['wall_clock_runtime_s']} |\n"
        )
        file.write(
            f"| Runtime ratio (sim/wall) | "
            f"{bsk_summary.get('runtime_ratio', 'N/A')} | "
            f"{gmat_summary['runtime_ratio']:.1f} |\n"
        )
        file.write("\n## Cross-tool (same epochs)\n\n")
        file.write(f"- Epochs compared: {len(cross_rows)}\n")
        file.write(f"- Max |r_GMAT − r_Basilisk| [m]: {_max('position_difference_gmat_vs_basilisk_m'):.6f}\n")
        file.write(f"- Max |v_GMAT − v_Basilisk| [m/s]: {_max('velocity_difference_gmat_vs_basilisk_mps'):.9f}\n")
        file.write(f"- Max |r_GMAT − r_Basilisk| at t=86400 s [m]: ")
        file.write(f"{three_way_rows[-1]['position_difference_gmat_vs_basilisk_m']:.6f}\n\n")
        file.write("## Artifacts\n\n")
        file.write(f"- Basilisk vs reference: `basilisk_a1/basilisk_a1_state_comparison.csv`\n")
        file.write(f"- GMAT vs reference: `gmat_a1/gmat_a1_state_comparison.csv`\n")
        file.write(f"- Three-way table: `a1_three_way_comparison.csv`\n")
        file.write(f"- GMAT vs Basilisk: `a1_cross_tool_comparison.csv`\n\n")
        file.write("## Interpretation\n\n")
        file.write(
            "Both simulators are compared to the same analytical Kepler propagator, not to each other as truth. "
            "Small GMAT–Basilisk separation indicates consistent two-body implementation; "
            "each tool's gap to the reference reflects integrator and frame handling differences.\n"
        )


def main() -> None:
    config = A1Config()
    gmat_csv, output_dir = resolve_paths()
    raw_times, raw_positions, raw_velocities = parse_gmat_state_csv(gmat_csv)

    if len(raw_times) > EXPECTED_SAMPLES * 2:
        print(
            f"WARNING: {len(raw_times)} epochs parsed — expected ~{EXPECTED_SAMPLES}. "
            "Re-run GMAT with A1_State_Report.Add = {} and a fresh gmat_a1_state.csv."
        )

    times_s, positions_m, velocities_mps = resample_to_cadence(
        raw_times, raw_positions, raw_velocities, config
    )
    summary = write_gmat_outputs(
        output_dir, config, times_s, positions_m, velocities_mps, len(raw_times)
    )

    print("Benchmark A1 GMAT analytical validation complete.")
    print(f"Output directory: {output_dir}")
    print(f"Raw CSV epochs: {summary['raw_csv_rows_parsed']}")
    print(f"Samples (600 s grid): {summary['samples']}")
    print(f"Final position difference vs reference [m]: {summary['position_difference_vs_reference_final_m']:.6f}")
    print(f"Max position difference vs reference [m]: {summary['position_difference_vs_reference_max_m']:.6f}")
    report = resolve_paths()[1].parent / "A1_COMPARISON_REPORT.md"
    if report.is_file():
        print(f"Three-way report: {report}")


if __name__ == "__main__":
    main()
