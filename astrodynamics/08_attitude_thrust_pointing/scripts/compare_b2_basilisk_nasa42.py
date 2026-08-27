#!/usr/bin/env python3
"""Cross-tool overlay comparison for Benchmark B2 (Basilisk vs NASA 42)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from b2_attitude_config import B2Case, B2Config

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
DATA_ROOT = WORKSPACE_ROOT / "data/benchmark_b/b2_attitude_gnc"
COMPARISON_DIR = WORKSPACE_ROOT / "data/benchmark_b/b2_attitude_gnc/comparison"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B2_METRICS_COMPARISON.csv"

HISTORY_COLUMNS = (
    "battery_soc_percent",
    "generated_power_w",
    "net_power_w",
    "pointing_error_deg",
    "shadow_factor",
    "control_torque_x_nm",
    "control_torque_y_nm",
    "control_torque_z_nm",
    "sigma_br_0",
    "sigma_br_1",
    "sigma_br_2",
    "omega_br_0_rad_s",
    "omega_br_1_rad_s",
    "omega_br_2_rad_s",
    "r_bn_x_km",
    "r_bn_y_km",
    "r_bn_z_km",
    "max_wheel_momentum_nms",
)


def parse_case(value: str) -> B2Case:
    normalized = value.lower().strip()
    for case in B2Case:
        if normalized in {case.value, case.name.lower()}:
            return case
    raise argparse.ArgumentTypeError(f"Unknown B2 case: {value}")


def load_history(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"Missing history CSV: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"Empty history CSV: {path}")

    series: dict[str, np.ndarray] = {"time_s": np.array([float(row["time_s"]) for row in rows])}
    for column in HISTORY_COLUMNS:
        series[column] = np.array([float(row[column]) for row in rows], dtype=float)
    return series


def align_series(basilisk: dict[str, np.ndarray], nasa42: dict[str, np.ndarray]) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    if basilisk["time_s"].shape != nasa42["time_s"].shape:
        raise ValueError(
            f"History lengths differ: Basilisk {basilisk['time_s'].size}, NASA 42 {nasa42['time_s'].size}"
        )
    if not np.allclose(basilisk["time_s"], nasa42["time_s"], atol=1.0e-6, rtol=0.0):
        raise ValueError("History time grids do not align on common samples")
    return basilisk, nasa42


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def compute_diff_metrics(
    config: B2Config,
    basilisk: dict[str, np.ndarray],
    nasa42: dict[str, np.ndarray],
) -> dict[str, Any]:
    sunlit = basilisk["shadow_factor"] >= config.eclipse_shadow_threshold
    metrics: dict[str, Any] = {
        "sample_count": int(basilisk["time_s"].size),
        "pointing_error_deg_t0_basilisk": float(basilisk["pointing_error_deg"][0]),
        "pointing_error_deg_t0_nasa42": float(nasa42["pointing_error_deg"][0]),
        "pointing_error_deg_t0_abs_diff": float(
            abs(basilisk["pointing_error_deg"][0] - nasa42["pointing_error_deg"][0])
        ),
        "pointing_error_deg_rms_diff": rms(basilisk["pointing_error_deg"] - nasa42["pointing_error_deg"]),
        "pointing_error_deg_max_abs_diff": float(
            np.max(np.abs(basilisk["pointing_error_deg"] - nasa42["pointing_error_deg"]))
        ),
        "battery_soc_final_abs_diff_percent": float(
            abs(basilisk["battery_soc_percent"][-1] - nasa42["battery_soc_percent"][-1])
        ),
        "position_km_rms_diff": rms(
            np.linalg.norm(
                np.column_stack(
                    (
                        basilisk["r_bn_x_km"] - nasa42["r_bn_x_km"],
                        basilisk["r_bn_y_km"] - nasa42["r_bn_y_km"],
                        basilisk["r_bn_z_km"] - nasa42["r_bn_z_km"],
                    )
                ),
                axis=1,
            )
        ),
    }
    if np.any(sunlit):
        metrics["pointing_error_deg_rms_diff_sunlit"] = rms(
            (basilisk["pointing_error_deg"] - nasa42["pointing_error_deg"])[sunlit]
        )
        metrics["pointing_error_deg_max_abs_diff_sunlit"] = float(
            np.max(np.abs((basilisk["pointing_error_deg"] - nasa42["pointing_error_deg"])[sunlit]))
        )
    return metrics


def save_overlay_timeseries(
    config: B2Config,
    case: B2Case,
    output_dir: Path,
    basilisk: dict[str, np.ndarray],
    nasa42: dict[str, np.ndarray],
) -> Path:
    times_h = basilisk["time_s"] / 3600.0
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(times_h, basilisk["battery_soc_percent"], color="tab:blue", label="Basilisk")
    axes[0].plot(times_h, nasa42["battery_soc_percent"], color="tab:orange", linestyle="--", label="NASA 42")
    axes[0].axhline(config.target_soc_percent, color="gray", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("Battery SoC [%]")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times_h, basilisk["generated_power_w"], color="tab:blue", label="Basilisk panel")
    axes[1].plot(times_h, nasa42["generated_power_w"], color="tab:orange", linestyle="--", label="NASA 42 panel")
    axes[1].set_ylabel("Generated power [W]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times_h, basilisk["pointing_error_deg"], color="tab:blue", label="Basilisk")
    axes[2].plot(times_h, nasa42["pointing_error_deg"], color="tab:orange", linestyle="--", label="NASA 42")
    axes[2].set_ylabel("Pointing error [deg]")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    in_eclipse = basilisk["shadow_factor"] < config.eclipse_shadow_threshold
    axes[3].fill_between(times_h, 0.0, 1.0, where=in_eclipse, color="0.85", alpha=0.5, label="Basilisk eclipse")
    axes[3].set_ylabel("Eclipse flag")
    axes[3].set_xlabel("Time [h]")
    axes[3].set_ylim(-0.05, 1.05)
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"B2 {case.value} — Basilisk vs NASA 42 (energy & pointing)")
    fig.tight_layout()
    plot_path = output_dir / "b2_timeseries_compare.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def save_overlay_gnc(
    case: B2Case,
    output_dir: Path,
    basilisk: dict[str, np.ndarray],
    nasa42: dict[str, np.ndarray],
) -> Path:
    times_h = basilisk["time_s"] / 3600.0
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    labels = ("x", "y", "z")
    colors = ("C0", "C1", "C2")

    for index in range(3):
        key = ("r_bn_x_km", "r_bn_y_km", "r_bn_z_km")[index]
        axes[0].plot(times_h, basilisk[key], color=colors[index], label=f"Basilisk r_{labels[index]}")
        axes[0].plot(times_h, nasa42[key], color=colors[index], linestyle="--", label=f"NASA 42 r_{labels[index]}")
    axes[0].set_ylabel("Position [km]")
    axes[0].legend(loc="upper right", ncol=2, fontsize=7)
    axes[0].grid(True, alpha=0.3)

    for index in range(3):
        key = ("omega_br_0_rad_s", "omega_br_1_rad_s", "omega_br_2_rad_s")[index]
        axes[1].plot(times_h, basilisk[key], color=colors[index])
        axes[1].plot(times_h, nasa42[key], color=colors[index], linestyle="--")
    axes[1].set_ylabel(r"$\omega_{BR}$ [rad/s]")
    axes[1].grid(True, alpha=0.3)

    for index in range(3):
        key = ("control_torque_x_nm", "control_torque_y_nm", "control_torque_z_nm")[index]
        axes[2].plot(times_h, basilisk[key], color=colors[index])
        axes[2].plot(times_h, nasa42[key], color=colors[index], linestyle="--")
    axes[2].set_ylabel("Control torque [Nm]")
    axes[2].grid(True, alpha=0.3)

    for index in range(3):
        key = ("sigma_br_0", "sigma_br_1", "sigma_br_2")[index]
        axes[3].plot(times_h, basilisk[key], color=colors[index])
        axes[3].plot(times_h, nasa42[key], color=colors[index], linestyle="--")
    axes[3].set_ylabel(r"$\sigma_{B/R}$")
    axes[3].set_xlabel("Time [h]")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"B2 {case.value} — Basilisk vs NASA 42 (ADCS)")
    fig.tight_layout()
    plot_path = output_dir / "b2_gnc_compare.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def save_pointing_diff_plot(case: B2Case, output_dir: Path, basilisk: dict[str, np.ndarray], nasa42: dict[str, np.ndarray]) -> Path:
    times_h = basilisk["time_s"] / 3600.0
    diff = basilisk["pointing_error_deg"] - nasa42["pointing_error_deg"]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times_h, diff, color="tab:purple")
    ax.axhline(0.0, color="gray", linewidth=0.8)
    ax.set_xlabel("Time [h]")
    ax.set_ylabel("Pointing error diff [deg]\n(Basilisk − NASA 42)")
    ax.set_title(f"B2 {case.value} pointing error difference")
    ax.grid(True, alpha=0.3)
    plot_path = output_dir / "b2_pointing_error_diff.png"
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def write_diff_csv(path: Path, basilisk: dict[str, np.ndarray], nasa42: dict[str, np.ndarray]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "time_s",
                "pointing_error_deg_basilisk",
                "pointing_error_deg_nasa42",
                "pointing_error_deg_diff",
                "battery_soc_percent_basilisk",
                "battery_soc_percent_nasa42",
                "battery_soc_percent_diff",
            ]
        )
        for index, time_s in enumerate(basilisk["time_s"]):
            writer.writerow(
                [
                    time_s,
                    basilisk["pointing_error_deg"][index],
                    nasa42["pointing_error_deg"][index],
                    basilisk["pointing_error_deg"][index] - nasa42["pointing_error_deg"][index],
                    basilisk["battery_soc_percent"][index],
                    nasa42["battery_soc_percent"][index],
                    basilisk["battery_soc_percent"][index] - nasa42["battery_soc_percent"][index],
                ]
            )


def compare_case(
    config: B2Config,
    case: B2Case,
    basilisk_dir: Path | None = None,
    nasa42_dir: Path | None = None,
    output_dir: Path | None = None,
) -> dict[str, Any]:
    basilisk_history = (basilisk_dir or DATA_ROOT / "basilisk" / case.value) / "b2_history.csv"
    nasa42_history = (nasa42_dir or DATA_ROOT / "nasa42" / case.value) / "b2_history.csv"
    out_dir = output_dir or COMPARISON_DIR / case.value
    out_dir.mkdir(parents=True, exist_ok=True)

    b_series = load_history(basilisk_history)
    n_series = load_history(nasa42_history)
    b_aligned, n_aligned = align_series(b_series, n_series)
    metrics = compute_diff_metrics(config, b_aligned, n_aligned)

    artifacts = {
        "timeseries_compare_plot": str(save_overlay_timeseries(config, case, out_dir, b_aligned, n_aligned)),
        "gnc_compare_plot": str(save_overlay_gnc(case, out_dir, b_aligned, n_aligned)),
        "pointing_diff_plot": str(save_pointing_diff_plot(case, out_dir, b_aligned, n_aligned)),
    }
    diff_csv = out_dir / "b2_cross_tool_diff.csv"
    write_diff_csv(diff_csv, b_aligned, n_aligned)
    artifacts["diff_csv"] = str(diff_csv)

    summary = {
        "benchmark": "B2",
        "case_id": case.value,
        "basilisk_history": str(basilisk_history),
        "nasa42_history": str(nasa42_history),
        "diff_metrics": metrics,
        "comparison_notes": [
            "Pointing error: angle between body +Z and Sun in inertial frame (same geometric definition for both tools).",
            "Max abs diff often occurs during early transient settling (first ~2 min), not at t=0, because controllers differ (Basilisk MRP+RW vs NASA 42 PROTOTYPE_FSW+IdealTrq).",
            "NASA 42 SC file requires NQN to load quaternion IC; NAN skips the quaternion line.",
        ],
        "artifacts": artifacts,
    }
    summary_path = out_dir / "comparison_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare Basilisk and NASA 42 B2 histories")
    parser.add_argument("--case", type=parse_case, default=B2Case.B2A)
    parser.add_argument("--basilisk-dir", type=Path, default=None)
    parser.add_argument("--nasa42-dir", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    config = B2Config()
    summary = compare_case(config, args.case, args.basilisk_dir, args.nasa42_dir, args.output_dir)
    metrics = summary["diff_metrics"]

    print(f"B2 {args.case.value} Basilisk vs NASA 42 comparison")
    print(f"  Samples aligned: {metrics['sample_count']}")
    print(f"  Pointing error t=0: Basilisk {metrics['pointing_error_deg_t0_basilisk']:.3f} deg, "
          f"NASA 42 {metrics['pointing_error_deg_t0_nasa42']:.3f} deg "
          f"(|diff| {metrics['pointing_error_deg_t0_abs_diff']:.3f} deg)")
    print(f"  Pointing error RMS diff (all): {metrics['pointing_error_deg_rms_diff']:.3f} deg")
    if "pointing_error_deg_rms_diff_sunlit" in metrics:
        print(f"  Pointing error RMS diff (sunlit): {metrics['pointing_error_deg_rms_diff_sunlit']:.3f} deg")
    print(f"  Position RMS diff: {metrics['position_km_rms_diff']:.3f} km")
    print(f"  Artifacts: {summary['artifacts']['timeseries_compare_plot']}")


if __name__ == "__main__":
    main()
