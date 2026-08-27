#!/usr/bin/env python3
"""Generate report artifacts from PID/MPC bag comparison CSV outputs.

Inputs (from compare_pid_mpc_bags.py):
  - summary_metrics.csv
  - step_metrics.csv
  - timeseries_pid_<ns>.csv
  - timeseries_mpc_<ns>.csv

Outputs:
  - pid_mpc_comparison_table.tex
  - pid_mpc_<namespace>.png (one combined plot per spacecraft)
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import matplotlib.pyplot as plt


NAMESPACES = ("leaderSc", "followerSc_1", "followerSc_2")


def read_csv_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def to_float(s: str, default: float = 0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default


def format_ns(ns: str) -> str:
    if ns == "leaderSc":
        return "leaderSc"
    if ns == "followerSc_1":
        return "followerSc\\_1"
    if ns == "followerSc_2":
        return "followerSc\\_2"
    return ns.replace("_", "\\_")


def write_latex_table(summary_rows: List[Dict[str, str]], out_tex: Path) -> None:
    # index by (run, namespace)
    idx: Dict[tuple, Dict[str, str]] = {}
    for r in summary_rows:
        idx[(r["run"], r["namespace"])] = r

    lines: List[str] = []
    lines.append("\\begin{table}[H]")
    lines.append("\\centering")
    lines.append("\\small")
    lines.append("\\begin{tabular}{l l r r r r r}")
    lines.append("\\toprule")
    lines.append("Spacecraft & Run & RMSE $\\|e\\|$ [m] & Peak $\\|e\\|$ [m] & $\\max\\|F\\|$ [N] & $\\int\\|F\\|dt$ [N\\,s] & $\\int\\|T\\|dt$ [N\\,m\\,s]\\\\")
    lines.append("\\midrule")

    for ns in NAMESPACES:
        for run in ("pid", "mpc"):
            r = idx.get((run, ns))
            if r is None:
                lines.append(f"{format_ns(ns)} & {run.upper()} & -- & -- & -- & -- & --\\\\")
                continue
            rmse_e = to_float(r.get("rmse_err_norm_m", "nan"), float("nan"))
            peak_e = to_float(r.get("peak_err_norm_m", "nan"), float("nan"))
            max_f = to_float(r.get("max_force_norm_N", "nan"), float("nan"))
            int_f = to_float(r.get("integral_force_norm_Ns", "nan"), float("nan"))
            int_t = to_float(r.get("integral_torque_norm_Nms", "nan"), float("nan"))
            lines.append(
                f"{format_ns(ns)} & {run.upper()} & "
                f"{rmse_e:.4f} & {peak_e:.4f} & {max_f:.4f} & {int_f:.4f} & {int_t:.4f}\\\\"
            )
        lines.append("\\addlinespace")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\caption{PID vs MPC closed-loop tracking and control-effort metrics from bag replay.}")
    lines.append("\\label{tab:pid-mpc-compare}")
    lines.append("\\end{table}")
    lines.append("")

    out_tex.write_text("\n".join(lines), encoding="utf-8")


def read_timeseries(path: Path) -> Dict[str, List[float]]:
    rows = read_csv_rows(path)
    out = {
        "t": [],
        "err_norm": [],
        "fx": [],
        "fy": [],
        "fz": [],
        "x": [],
        "x_ref": [],
    }
    # force is not in timeseries csv; keep placeholders for consistent code
    for r in rows:
        out["t"].append(to_float(r["t_rel_s"]))
        out["err_norm"].append(to_float(r["err_norm_m"]))
        out["x"].append(to_float(r["x_m"]))
        out["x_ref"].append(to_float(r["x_ref_m"]))
    return out


def read_force_series(step_rows: List[Dict[str, str]], run: str, namespace: str) -> Dict[str, float]:
    # Step-level rows are not a force time series; return aggregate indicators.
    # Combined figure will show error trajectory + x tracking, while force effort is in legend text.
    matched = [r for r in step_rows if r["run"] == run and r["namespace"] == namespace]
    return {
        "n_steps": len(matched),
        "avg_step_rmse": (
            sum(to_float(r["step_rmse_err_norm_m"]) for r in matched) / len(matched)
            if matched else 0.0
        ),
    }


def make_combined_plot(
    ns: str,
    pid_ts: Dict[str, List[float]],
    mpc_ts: Dict[str, List[float]],
    step_rows: List[Dict[str, str]],
    out_png: Path,
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # Top: error norm
    axes[0].plot(pid_ts["t"], pid_ts["err_norm"], label="PID", linewidth=1.6)
    axes[0].plot(mpc_ts["t"], mpc_ts["err_norm"], label="MPC", linewidth=1.6)
    axes[0].set_ylabel("||e|| [m]")
    axes[0].set_title(f"{ns}: tracking error and x-axis reference tracking")
    axes[0].grid(True, alpha=0.35)
    axes[0].legend(loc="best")

    # Bottom: x and x_ref for both runs
    axes[1].plot(pid_ts["t"], pid_ts["x"], label="PID x", linewidth=1.4)
    axes[1].plot(pid_ts["t"], pid_ts["x_ref"], label="PID x_ref", linestyle="--", linewidth=1.2)
    axes[1].plot(mpc_ts["t"], mpc_ts["x"], label="MPC x", linewidth=1.4)
    axes[1].plot(mpc_ts["t"], mpc_ts["x_ref"], label="MPC x_ref", linestyle="--", linewidth=1.2)
    axes[1].set_ylabel("x [m]")
    axes[1].set_xlabel("t_rel [s]")
    axes[1].grid(True, alpha=0.35)
    axes[1].legend(loc="best", ncol=2)

    # Add small text block with per-step avg RMSE
    pid_s = read_force_series(step_rows, "pid", ns)
    mpc_s = read_force_series(step_rows, "mpc", ns)
    txt = (
        f"avg step RMSE  PID={pid_s['avg_step_rmse']:.4f} m, "
        f"MPC={mpc_s['avg_step_rmse']:.4f} m"
    )
    fig.text(0.5, 0.01, txt, ha="center", fontsize=9)

    fig.tight_layout(rect=[0.02, 0.04, 0.98, 0.98])
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate LaTeX table + comparison plots from bag_compare_out CSVs.")
    ap.add_argument(
        "--in-dir",
        default="/home/mahmedext/ros2_ws/bag_compare_out",
        help="Input directory containing summary_metrics.csv, step_metrics.csv, and timeseries files",
    )
    ap.add_argument(
        "--out-dir",
        default="/home/mahmedext/ros2_ws/bag_compare_out/report_artifacts",
        help="Output directory for .tex and .png artifacts",
    )
    args = ap.parse_args()

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_csv = in_dir / "summary_metrics.csv"
    step_csv = in_dir / "step_metrics.csv"
    if not summary_csv.exists():
        raise FileNotFoundError(f"Missing {summary_csv}")
    if not step_csv.exists():
        raise FileNotFoundError(f"Missing {step_csv}")

    summary_rows = read_csv_rows(summary_csv)
    step_rows = read_csv_rows(step_csv)

    # Table
    write_latex_table(summary_rows, out_dir / "pid_mpc_comparison_table.tex")

    # Plots per spacecraft
    for ns in NAMESPACES:
        pid_path = in_dir / f"timeseries_pid_{ns}.csv"
        mpc_path = in_dir / f"timeseries_mpc_{ns}.csv"
        if not pid_path.exists() or not mpc_path.exists():
            raise FileNotFoundError(f"Missing timeseries for {ns}: {pid_path} or {mpc_path}")
        pid_ts = read_timeseries(pid_path)
        mpc_ts = read_timeseries(mpc_path)
        make_combined_plot(
            ns=ns,
            pid_ts=pid_ts,
            mpc_ts=mpc_ts,
            step_rows=step_rows,
            out_png=out_dir / f"pid_mpc_{ns}.png",
        )

    print(f"Generated artifacts in: {out_dir}")
    print(" - pid_mpc_comparison_table.tex")
    for ns in NAMESPACES:
        print(f" - pid_mpc_{ns}.png")


if __name__ == "__main__":
    main()

