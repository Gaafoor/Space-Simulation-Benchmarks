#!/usr/bin/env python3
"""Basilisk ADCS pointing figure (parallel layout to NASA 42 a3_adcs_pointing)."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _mrp_angle_deg(sigma: np.ndarray) -> float:
    s = float(np.linalg.norm(sigma))
    return math.degrees(2.0 * math.atan(s))


def make_basilisk_adcs_plot(out_dir: Path, *, out_path: Path | None = None) -> Path:
    out_dir = Path(out_dir)
    pointing_rows = _load_csv(out_dir / "pointing_error_history.csv")
    att_rows = _load_csv(out_dir / "att_guidance_debug.csv")
    if not pointing_rows or not att_rows:
        raise FileNotFoundError(f"Missing pointing or attitude diagnostics in {out_dir}")

    pt_t = np.array([float(r["time_s"]) for r in pointing_rows], dtype=float)
    pt_deg = np.array([float(r["pointing_error_deg"]) for r in pointing_rows], dtype=float)
    att_t = np.array([float(r["time_s"]) for r in att_rows], dtype=float)
    sigma_br = np.array(
        [[float(r[f"sigma_br_{i}"]) for i in range(3)] for r in att_rows],
        dtype=float,
    )
    omega_br = np.array(
        [[float(r[f"omega_br_{i}"]) for i in range(3)] for r in att_rows],
        dtype=float,
    )

    sig_angle = np.array([_mrp_angle_deg(s) for s in sigma_br], dtype=float)
    omega_mag_deg_s = np.degrees(np.linalg.norm(omega_br, axis=1))

    t_h_pt = pt_t / 3600.0
    t_h_att = att_t / 3600.0

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle("Basilisk ADCS pointing during transfer execution", fontsize=13)

    ax = axes[0, 0]
    ax.plot(t_h_pt, pt_deg, color="#d95f02", lw=1.0)
    ax.set_ylabel("deg")
    ax.set_title("+X thruster vs inertial velocity")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[0, 1]
    ax.plot(t_h_att, sig_angle, color="#1b9e77", lw=1.0)
    ax.set_ylabel("deg")
    ax.set_title("Reference attitude error $|\\sigma_{B/R}|$")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[1, 0]
    ax.plot(t_h_att, omega_mag_deg_s, color="#7570b3", lw=1.0)
    ax.set_ylabel("deg/s")
    ax.set_title("Angular rate magnitude $|\\omega_{B/R}|$")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[1, 1]
    ax.plot(t_h_att, np.degrees(sigma_br[:, 0]), label=r"$\sigma_1$", lw=0.9)
    ax.plot(t_h_att, np.degrees(sigma_br[:, 1]), label=r"$\sigma_2$", lw=0.9)
    ax.plot(t_h_att, np.degrees(sigma_br[:, 2]), label=r"$\sigma_3$", lw=0.9)
    ax.set_ylabel("deg")
    ax.set_title("Reference attitude error components")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    out_path = out_path or out_dir / "a3_basilisk_adcs_pointing.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main() -> int:
    root = _repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=root / "data/pipeline_handoff/basilisk/gmat_gnc_handoff_mission_realistic_full_rerun",
    )
    parser.add_argument(
        "--save-path",
        type=Path,
        default=root / "docs/report/figures/a3_basilisk_adcs_pointing.png",
    )
    args = parser.parse_args()
    path = make_basilisk_adcs_plot(args.out_dir, out_path=args.save_path)
    print(f"Wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
