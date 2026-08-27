#!/usr/bin/env python3
"""Plot B1 cross-tool incremental drift for report figures."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B1_METRICS_COMPARISON.csv"
DEFAULT_OUTPUT = WORKSPACE_ROOT / "docs/report/figures/b1_drift_cross_tool_24h.png"

CASE_ORDER = [
    "b1_case2_j2",
    "b1_case3_j2_drag",
    "b1_case4_j2_drag_srp",
    "b1_case5_all_supported",
]
CASE_LABELS = ["J2", "J2+drag", "J2+drag+SRP", "J2+drag+SRP+third-body"]
TOOL_ORDER = ["GMAT", "Basilisk", "Orekit"]
TOOL_COLORS = {"GMAT": "#4C72B0", "Basilisk": "#DD8452", "Orekit": "#55A868"}


def load_drift_km(metrics_csv: Path) -> dict[str, dict[str, float]]:
    drift: dict[str, dict[str, float]] = {tool: {} for tool in TOOL_ORDER}
    with metrics_csv.open(newline="") as handle:
        for row in csv.DictReader(handle):
            tool = row["simulator"]
            case_id = row["case_id"]
            if tool in drift and case_id in CASE_ORDER:
                drift[tool][case_id] = float(row["position_drift_vs_case1_m"]) / 1000.0
    return drift


def build_incremental_drift_km(drift_km: dict[str, dict[str, float]]) -> dict[str, list[float]]:
    """Convert drift-vs-case1 values to case-to-previous-case increments."""
    incremental: dict[str, list[float]] = {}
    for tool in TOOL_ORDER:
        previous = 0.0
        tool_values: list[float] = []
        for case_id in CASE_ORDER:
            current = drift_km[tool][case_id]
            tool_values.append(current - previous)
            previous = current
        incremental[tool] = tool_values
    return incremental


def plot_cross_tool_drift(
    drift_km: dict[str, dict[str, float]],
    output_path: Path,
    *,
    y_padding_frac: float = 0.18,
) -> None:
    n_cases = len(CASE_ORDER)
    n_tools = len(TOOL_ORDER)
    x = np.arange(n_cases)
    bar_width = 0.24

    fig, ax = plt.subplots(figsize=(8.4, 4.6), layout="constrained")

    all_values: list[float] = []
    incremental_km = build_incremental_drift_km(drift_km)
    for tool_idx, tool in enumerate(TOOL_ORDER):
        values = incremental_km[tool]
        all_values.extend(values)
        offsets = x + (tool_idx - (n_tools - 1) / 2) * bar_width
        bars = ax.bar(
            offsets,
            values,
            width=bar_width,
            label=tool,
            color=TOOL_COLORS[tool],
            edgecolor="white",
            linewidth=0.6,
        )
        for bar, value in zip(bars, values):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                value,
                f"{value:.3f}",
                ha="center",
                va="bottom" if value >= 0 else "top",
                fontsize=7.5,
                rotation=90,
                clip_on=False,
            )

    y_min = min(all_values)
    y_max = max(all_values)
    max_abs = max(abs(y_min), abs(y_max), 1e-3)
    # Use symmetric log scale so large Case-2 jump and small deltas are both legible.
    ax.set_yscale("symlog", linthresh=0.25, linscale=1.0, base=10)
    ax.set_ylim(-1.0, max_abs * 1.3)
    ax.axhline(0.0, color="0.25", linewidth=0.8, linestyle="-")

    ax.set_xticks(x)
    ax.set_xticklabels(CASE_LABELS)
    ax.set_ylabel("Delta vs previous case [km]")
    ax.set_title("B1 perturbation ladder")
    ax.grid(axis="y", alpha=0.35, linestyle="--", linewidth=0.7)
    ax.legend(loc="upper right", frameon=True)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot B1 cross-tool drift comparison figure.")
    parser.add_argument("--metrics-csv", type=Path, default=DEFAULT_METRICS_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    drift_km = load_drift_km(args.metrics_csv)
    plot_cross_tool_drift(drift_km, args.output)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
