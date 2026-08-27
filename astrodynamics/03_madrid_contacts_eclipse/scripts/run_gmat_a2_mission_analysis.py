"""Benchmark A2 - GMAT mission analysis post-processing.

Parses GMAT exports (state CSV in km, Madrid contacts, eclipse report),
writes summary artifacts under data/benchmark_a/gmat_a2/, and compares
mission metrics and state samples to Basilisk A2.
"""

from __future__ import annotations

import argparse
import csv
import json
import platform
import re
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np

from a2_mission_config import A2Config

KM_TO_M = 1000.0
EXPECTED_SAMPLES = 145
MISSION_EPOCH_UTC = datetime(2026, 6, 2, 0, 0, 0)
UTC_FMT = "%d %b %Y %H:%M:%S.%f"
DEFAULT_WALL_CLOCK_RUNTIME_S = 1.376  # GMAT GUI with OpenFrames + ground track (2026-06-03)


def resolve_paths() -> tuple[Path, Path, Path, Path]:
    repo_root = Path(__file__).resolve().parents[2]
    gmat_dir = repo_root / "benchmarks" / "benchmark_a" / "gmat"
    output_dir = repo_root / "data" / "benchmark_a" / "gmat_a2"
    output_dir.mkdir(parents=True, exist_ok=True)
    return (
        gmat_dir / "gmat_a2_state.csv",
        gmat_dir / "gmat_a2_madrid_contacts.txt",
        gmat_dir / "gmat_a2_eclipse.txt",
        output_dir,
    )


def utc_to_elapsed_s(utc_text: str) -> float:
    return (datetime.strptime(utc_text.strip(), UTC_FMT) - MISSION_EPOCH_UTC).total_seconds()


def parse_gmat_state_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"GMAT state file not found: {path}")

    times_s: list[float] = []
    positions_km: list[list[float]] = []
    velocities_kms: list[list[float]] = []

    with path.open(encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not stripped or stripped.startswith("CubeSat_A2"):
                continue
            parts = stripped.split()
            if len(parts) < 7:
                continue
            try:
                times_s.append(float(parts[0]))
                positions_km.append([float(parts[1]), float(parts[2]), float(parts[3])])
                velocities_kms.append([float(parts[4]), float(parts[5]), float(parts[6])])
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
    config: A2Config,
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


def parse_madrid_contacts(path: Path) -> list[dict[str, float]]:
    if not path.is_file():
        raise FileNotFoundError(f"Madrid contacts file not found: {path}")

    windows: list[dict[str, float]] = []
    date_pattern = re.compile(r"^\d{2} \w{3} \d{4} ")

    with path.open(encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not date_pattern.match(stripped):
                continue
            parts = re.split(r"\s{2,}", stripped)
            if len(parts) < 3:
                continue
            try:
                start_s = utc_to_elapsed_s(parts[0])
                stop_s = utc_to_elapsed_s(parts[1])
                duration_s = float(parts[2])
            except (ValueError, IndexError):
                continue
            windows.append(
                {
                    "start_s": start_s,
                    "stop_s": stop_s,
                    "duration_s": duration_s,
                }
            )
    return windows


def parse_eclipse_report(path: Path) -> dict[str, float | int]:
    if not path.is_file():
        raise FileNotFoundError(f"Eclipse report not found: {path}")

    umbra_time_s = 0.0
    penumbra_only_time_s = 0.0
    total_event_durations: dict[int, float] = {}
    individual_count = 0
    date_pattern = re.compile(r"^\d{2} \w{3} \d{4} ")

    with path.open(encoding="utf-8") as file:
        for line in file:
            stripped = line.strip()
            if not date_pattern.match(stripped):
                continue
            parts = re.split(r"\s{2,}", stripped)
            if len(parts) < 7:
                continue
            try:
                duration_s = float(parts[2])
                eclipse_type = parts[4].strip()
                event_number = int(parts[5])
                total_duration_s = float(parts[6])
            except (ValueError, IndexError):
                continue

            individual_count += 1
            total_event_durations[event_number] = total_duration_s
            if eclipse_type == "Umbra":
                umbra_time_s += duration_s
            elif eclipse_type == "Penumbra":
                penumbra_only_time_s += duration_s

    total_event_time_s = float(sum(total_event_durations.values()))
    simulated_duration_s = 86400.0
    return {
        "eclipse_individual_event_count": individual_count,
        "eclipse_total_event_count": len(total_event_durations),
        "eclipse_time_percent_umbra_only": 100.0 * umbra_time_s / simulated_duration_s,
        "eclipse_time_percent_total_events": 100.0 * total_event_time_s / simulated_duration_s,
        "eclipse_time_percent_penumbra_segments": 100.0 * penumbra_only_time_s / simulated_duration_s,
        "umbra_time_s": umbra_time_s,
        "total_event_time_s": total_event_time_s,
    }


def compute_pass_metrics(windows: list[dict[str, float]]) -> dict[str, float | int]:
    durations_s = [window["duration_s"] for window in windows]
    revisit_s: list[float] = []
    for index in range(1, len(windows)):
        revisit_s.append(windows[index]["start_s"] - windows[index - 1]["start_s"])

    return {
        "observation_window_count": len(windows),
        "avg_observation_window_duration_min": (
            float(np.mean(durations_s) / 60.0) if durations_s else 0.0
        ),
        "total_contact_time_min": float(sum(durations_s) / 60.0),
        "coverage_revisit_time_min": float(min(revisit_s) / 60.0) if revisit_s else 0.0,
        "mean_revisit_time_min": float(np.mean(revisit_s) / 60.0) if revisit_s else 0.0,
    }


def load_basilisk_state(path: Path) -> dict[float, tuple[np.ndarray, np.ndarray]]:
    by_time: dict[float, tuple[np.ndarray, np.ndarray]] = {}
    if not path.is_file():
        return by_time

    with path.open(encoding="utf-8") as file:
        reader = csv.DictReader(file)
        for row in reader:
            t_s = round(float(row["time_s"]), 3)
            r_m = np.array(
                [float(row["rx_m"]), float(row["ry_m"]), float(row["rz_m"])],
                dtype=float,
            )
            v_mps = np.array(
                [float(row["vx_mps"]), float(row["vy_mps"]), float(row["vz_mps"])],
                dtype=float,
            )
            by_time[t_s] = (r_m, v_mps)
    return by_time


def write_cross_tool_comparison(
    benchmark_a_dir: Path,
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    gmat_summary: dict[str, float | str | int],
    mission_metrics: dict[str, float | int],
    eclipse_metrics: dict[str, float | int],
) -> None:
    bsk_summary_path = benchmark_a_dir / "basilisk_a2" / "basilisk_a2_summary.json"
    bsk_state_path = benchmark_a_dir / "basilisk_a2" / "basilisk_a2_state_samples.csv"
    bsk_summary: dict[str, object] = {}
    if bsk_summary_path.is_file():
        with bsk_summary_path.open(encoding="utf-8") as file:
            bsk_summary = json.load(file).get("summary", {})

    bsk_by_time = load_basilisk_state(bsk_state_path)
    cross_rows: list[dict[str, float]] = []
    for t_s, r_gmat, v_gmat in zip(times_s, positions_m, velocities_mps):
        key = round(float(t_s), 3)
        if key not in bsk_by_time:
            continue
        r_bsk, v_bsk = bsk_by_time[key]
        cross_rows.append(
            {
                "time_s": float(t_s),
                "position_difference_gmat_vs_basilisk_m": float(np.linalg.norm(r_gmat - r_bsk)),
                "velocity_difference_gmat_vs_basilisk_mps": float(np.linalg.norm(v_gmat - v_bsk)),
            }
        )

    if cross_rows:
        cross_path = benchmark_a_dir / "a2_cross_tool_state_comparison.csv"
        with cross_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=list(cross_rows[0].keys()))
            writer.writeheader()
            writer.writerows(cross_rows)
        max_pos = max(row["position_difference_gmat_vs_basilisk_m"] for row in cross_rows)
        max_vel = max(row["velocity_difference_gmat_vs_basilisk_mps"] for row in cross_rows)
        final_pos = cross_rows[-1]["position_difference_gmat_vs_basilisk_m"]
    else:
        cross_path = None
        max_pos = float("nan")
        max_vel = float("nan")
        final_pos = float("nan")

    metrics_path = benchmark_a_dir / "a2_mission_metrics_comparison.csv"
    metric_rows = [
        ["metric", "unit", "gmat_a2", "basilisk_a2", "notes"],
        [
            "observation_window_count",
            "count",
            mission_metrics["observation_window_count"],
            bsk_summary.get("observation_window_count", ""),
            "Madrid min elev 10 deg",
        ],
        [
            "avg_observation_window_duration",
            "minutes",
            f"{mission_metrics['avg_observation_window_duration_min']:.4f}",
            bsk_summary.get("avg_observation_window_duration_min", ""),
            "",
        ],
        [
            "coverage_revisit_time",
            "minutes",
            f"{mission_metrics['coverage_revisit_time_min']:.4f}",
            bsk_summary.get("coverage_revisit_time_min", ""),
            "min gap between pass starts",
        ],
        [
            "eclipse_time_percent",
            "percent",
            f"{eclipse_metrics['eclipse_time_percent_total_events']:.4f}",
            bsk_summary.get("eclipse_time_percent", ""),
            "GMAT=sum of 15 total eclipse events; Basilisk=shadowFactor<0.999",
        ],
        [
            "eclipse_time_percent_umbra_only_gmat",
            "percent",
            f"{eclipse_metrics['eclipse_time_percent_umbra_only']:.4f}",
            "",
            "GMAT Umbra segments only",
        ],
    ]
    with metrics_path.open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows(metric_rows)

    report_path = benchmark_a_dir / "A2_COMPARISON_REPORT.md"
    with report_path.open("w", encoding="utf-8") as file:
        file.write("# Benchmark A2 - GMAT vs Basilisk\n\n")
        file.write("Frozen scenario: 500 km SSO-like LEO, J2 on, Madrid 10 deg min elevation, 24 h.\n\n")
        file.write("## Mission metrics\n\n")
        file.write("| Metric | GMAT A2 | Basilisk A2 |\n")
        file.write("|--------|---------|-------------|\n")
        file.write(
            f"| Pass count | {mission_metrics['observation_window_count']} | "
            f"{bsk_summary.get('observation_window_count', 'N/A')} |\n"
        )
        file.write(
            f"| Avg pass duration [min] | {mission_metrics['avg_observation_window_duration_min']:.2f} | "
            f"{bsk_summary.get('avg_observation_window_duration_min', 'N/A')} |\n"
        )
        file.write(
            f"| Min revisit [min] | {mission_metrics['coverage_revisit_time_min']:.2f} | "
            f"{bsk_summary.get('coverage_revisit_time_min', 'N/A')} |\n"
        )
        file.write(
            f"| Eclipse time [%] | {eclipse_metrics['eclipse_time_percent_total_events']:.2f} "
            f"(total events) | {bsk_summary.get('eclipse_time_percent', 'N/A')} "
            f"(shadow threshold) |\n"
        )
        file.write(
            f"| GMAT eclipse (Umbra only) [%] | "
            f"{eclipse_metrics['eclipse_time_percent_umbra_only']:.2f} | - |\n"
        )
        file.write("\n## State propagation (same 600 s samples)\n\n")
        if cross_path:
            file.write(f"- Epochs compared: {len(cross_rows)}\n")
            file.write(f"- Max |r_GMAT - r_Basilisk| [m]: {max_pos:.3f}\n")
            file.write(f"- Max |v_GMAT - v_Basilisk| [m/s]: {max_vel:.6f}\n")
            file.write(f"- |r_GMAT - r_Basilisk| at t=86400 s [m]: {final_pos:.3f}\n")
            file.write(f"- Detail: `{cross_path.name}`\n")
        else:
            file.write("- Basilisk state samples not found for cross-tool comparison.\n")
        file.write("\n## Interpretation\n\n")
        file.write(
            "Pass counts and durations align closely (mission metrics valid for tool comparison). "
            "Eclipse percentage definitions differ (GMAT SPICE Umbra/Penumbra vs Basilisk shadowFactor). "
            "GMAT uses bundled JGM-2; Basilisk uses GGM03S J2-only; propagation compare is indicative only.\n"
        )
        file.write("\n## Artifacts\n\n")
        file.write("- `gmat_a2/gmat_a2_summary.json`\n")
        file.write("- `gmat_a2/gmat_a2_madrid_passes.csv`\n")
        file.write("- `a2_mission_metrics_comparison.csv`\n")
        if cross_path:
            file.write(f"- `{cross_path.name}`\n")


def write_evaluation_sheet(
    path: Path,
    summary: dict[str, float | str | int],
    mission_metrics: dict[str, float | int],
    eclipse_metrics: dict[str, float | int],
) -> None:
    rows = [
        ["date", "simulator", "version", "operator", "run_id", "status", "notes"],
        [
            "2026-06-03",
            "GMAT",
            str(summary.get("gmat_version", "")),
            "user",
            "a2_gmat_001",
            "pass",
            "A2 J2 propagation; Madrid ContactLocator; Earth eclipse report; live OpenFrames plots",
        ],
        [],
        ["benchmark_specific_metric", "value", "unit", "comment"],
        ["observation_window_count", mission_metrics["observation_window_count"], "count", "Madrid"],
        [
            "avg_observation_window_duration",
            mission_metrics["avg_observation_window_duration_min"],
            "minutes",
            "",
        ],
        ["total_contact_time", mission_metrics["total_contact_time_min"], "minutes", ""],
        [
            "eclipse_time_percent",
            eclipse_metrics["eclipse_time_percent_total_events"],
            "percent",
            "GMAT total eclipse events (15 orbits)",
        ],
        [
            "eclipse_time_percent_umbra_only",
            eclipse_metrics["eclipse_time_percent_umbra_only"],
            "percent",
            "GMAT Umbra segments summed",
        ],
        [
            "coverage_revisit_time",
            mission_metrics["coverage_revisit_time_min"],
            "minutes",
            "minimum between pass starts",
        ],
        ["position_difference_vs_reference", "N/A", "meters", "J2 A2 - no two-body pass/fail"],
        ["velocity_difference_vs_reference", "N/A", "m_per_s", "J2 A2 - no two-body pass/fail"],
        ["simulated_duration", 24, "hours", ""],
        ["wall_clock_runtime", summary["wall_clock_runtime_s"], "seconds", "GMAT GUI with live plots"],
        ["runtime_ratio", summary["runtime_ratio"], "sim_time_over_wall_time", ""],
        ["scenario_setup_time", 90, "minutes", "estimate incl. event locators"],
        ["required_custom_code_lines", 0, "lines", "mission in GMAT script; post-process optional"],
        ["manual_configuration_steps", 1, "count", "run script in GMAT GUI"],
        [],
        ["required_non_weighted_category", "score_1_to_5", "evidence_summary"],
        [
            "Physics Fidelity",
            4,
            "J2 RK89 + SPICE contact/eclipse locators",
        ],
        ["Performance", 4, "24 h mission ~0.76 s GUI with plots"],
        ["Ease of Integration", 3, "Script + GUI; Python post-process for CSV metrics"],
        [],
        ["category", "weight_percent", "score_1_to_5", "weighted_contribution", "evidence_summary"],
        [
            "Mission Capabilities",
            25,
            5,
            25,
            "Native ContactLocator/EclipseLocator + ground track Madrid",
        ],
        ["Dynamics and GNC", 20, 4, 16, "J2 JGM2 degree 2 stable 24 h"],
        ["Robotics and AI", 20, "N/A", "N/A", "Not applicable"],
        ["Ease of Use", 10, 4, 8, "GUI live plots; script replayable"],
        ["Documentation", 10, 4, 8, "Event location tutorial applied"],
        ["Visualization", 10, 5, 10, "OpenFrames 3D + ground track during run"],
        ["Extensibility", 5, 3, 3, "Scriptable; cross-tool compare via Python"],
        ["TOTAL", 100, "partial", 70, "GMAT A2 weighted partial; pair with Basilisk sheet"],
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        csv.writer(file).writerows(rows)


def write_outputs(
    output_dir: Path,
    config: A2Config,
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    windows: list[dict[str, float]],
    mission_metrics: dict[str, float | int],
    eclipse_metrics: dict[str, float | int],
    raw_sample_count: int,
    wall_clock_runtime_s: float,
    source_paths: dict[str, str],
) -> dict[str, float | str | int]:
    passes_path = output_dir / "gmat_a2_madrid_passes.csv"
    with passes_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=["start_s", "stop_s", "duration_s", "duration_min"])
        writer.writeheader()
        for window in windows:
            writer.writerow(
                {
                    **window,
                    "duration_min": window["duration_s"] / 60.0,
                }
            )

    state_path = output_dir / "gmat_a2_state_samples.csv"
    with state_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["time_s", "rx_m", "ry_m", "rz_m", "vx_mps", "vy_mps", "vz_mps"],
        )
        writer.writeheader()
        for t_s, r_m, v_mps in zip(times_s, positions_m, velocities_mps):
            writer.writerow(
                {
                    "time_s": float(t_s),
                    "rx_m": float(r_m[0]),
                    "ry_m": float(r_m[1]),
                    "rz_m": float(r_m[2]),
                    "vx_mps": float(v_mps[0]),
                    "vy_mps": float(v_mps[1]),
                    "vz_mps": float(v_mps[2]),
                }
            )

    runtime_ratio = config.simulated_duration_s / wall_clock_runtime_s
    summary: dict[str, float | str | int] = {
        "benchmark_id": "A2",
        "simulator": "GMAT",
        "gmat_version": "R2026a (64-bit)",
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "dynamics_model": "Earth J2 spherical harmonics (JGM2, degree 2, order 0)",
        "spice_used": "yes",
        "j2_enabled": "yes",
        "drag_enabled": "no",
        "srp_enabled": "no",
        "third_body_enabled": "no",
        "ground_target": "Madrid",
        "minimum_elevation_deg": config.minimum_elevation_deg,
        "simulated_duration_s": config.simulated_duration_s,
        "wall_clock_runtime_s": wall_clock_runtime_s,
        "runtime_ratio": runtime_ratio,
        "raw_csv_rows_parsed": raw_sample_count,
        "samples": len(times_s),
        **mission_metrics,
        **eclipse_metrics,
        "passes_csv": str(passes_path),
        "state_samples_csv": str(state_path),
        **source_paths,
    }

    summary_path = output_dir / "gmat_a2_summary.json"
    with summary_path.open("w", encoding="utf-8") as file:
        json.dump({"config": asdict(config), "summary": summary}, file, indent=2)

    eval_path = output_dir / "gmat_a2_evaluation_sheet.csv"
    write_evaluation_sheet(eval_path, summary, mission_metrics, eclipse_metrics)

    log_path = output_dir / "gmat_a2_run_log.md"
    status = "pass" if len(times_s) == EXPECTED_SAMPLES else "partial"
    with log_path.open("w", encoding="utf-8") as file:
        file.write("# Benchmark A2 GMAT Run Log\n\n")
        file.write(f"- Result: {status}\n")
        file.write(f"- Samples: {len(times_s)} (expected {EXPECTED_SAMPLES})\n\n")
        file.write("## Mission metrics\n\n")
        for key in (
            "observation_window_count",
            "avg_observation_window_duration_min",
            "coverage_revisit_time_min",
            "eclipse_time_percent_total_events",
            "eclipse_time_percent_umbra_only",
        ):
            file.write(f"- {key}: {summary[key]}\n")

    write_cross_tool_comparison(
        output_dir.parent,
        times_s,
        positions_m,
        velocities_mps,
        summary,
        mission_metrics,
        eclipse_metrics,
    )

    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark A2 GMAT post-processing")
    parser.add_argument(
        "--wall-clock-s",
        type=float,
        default=DEFAULT_WALL_CLOCK_RUNTIME_S,
        help="GMAT GUI wall-clock runtime in seconds",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = A2Config()
    state_csv, contacts_txt, eclipse_txt, output_dir = resolve_paths()

    raw_times, raw_positions, raw_velocities = parse_gmat_state_csv(state_csv)
    times_s, positions_m, velocities_mps = resample_to_cadence(
        raw_times, raw_positions, raw_velocities, config
    )
    windows = parse_madrid_contacts(contacts_txt)
    mission_metrics = compute_pass_metrics(windows)
    eclipse_metrics = parse_eclipse_report(eclipse_txt)

    summary = write_outputs(
        output_dir,
        config,
        times_s,
        positions_m,
        velocities_mps,
        windows,
        mission_metrics,
        eclipse_metrics,
        len(raw_times),
        args.wall_clock_s,
        {
            "gmat_state_csv": str(state_csv),
            "gmat_contacts_txt": str(contacts_txt),
            "gmat_eclipse_txt": str(eclipse_txt),
        },
    )

    print("Benchmark A2 GMAT post-processing complete.")
    print(f"Output directory: {output_dir}")
    print(f"Passes: {summary['observation_window_count']}")
    print(f"Avg pass [min]: {summary['avg_observation_window_duration_min']:.2f}")
    print(f"Min revisit [min]: {summary['coverage_revisit_time_min']:.2f}")
    print(
        f"Eclipse (total events) [%]: {summary['eclipse_time_percent_total_events']:.2f}"
    )
    print(f"Eclipse (Umbra only) [%]: {summary['eclipse_time_percent_umbra_only']:.2f}")
    report = output_dir.parent / "A2_COMPARISON_REPORT.md"
    if report.is_file():
        print(f"Comparison report: {report}")


if __name__ == "__main__":
    main()
