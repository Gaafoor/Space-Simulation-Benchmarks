#!/usr/bin/env python3
"""Extract NASA 42 telemetry and build A3 ADCS/GNC validation plots."""

from __future__ import annotations

import argparse
import re
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline

EPOCH_UTC = datetime(2026, 6, 15, 12, 0, 0, tzinfo=timezone.utc)
THRUSTER_AXIS_BODY = np.array([1.0, 0.0, 0.0])
NADIR_AXIS_BODY = np.array([0.0, 0.0, -1.0])


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def read_vector_file(path: Path, cols: int) -> np.ndarray:
    rows: list[list[float]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        try:
            vals = [float(p) for p in parts[:cols]]
        except ValueError:
            continue
        if len(vals) == cols:
            rows.append(vals)
    if not rows:
        raise RuntimeError(f"No numeric rows parsed from {path}")
    return np.asarray(rows, dtype=float)


def read_scalar_file(path: Path) -> np.ndarray:
  return read_vector_file(path, 1).ravel()


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body-to-inertial DCM from scalar-last quaternion [qx, qy, qz, qs]."""
    x, y, z, s = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - s * z), 2 * (x * z + s * y)],
            [2 * (x * y + s * z), 1 - 2 * (x * x + z * z), 2 * (y * z - s * x)],
            [2 * (x * z - s * y), 2 * (y * z + s * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))))


def _prepare_wrapped_angle_for_plot(angle_deg_series: pd.Series) -> np.ndarray:
    """Return principal-angle series and break wrap jumps for plotting."""
    vals = angle_deg_series.to_numpy(dtype=float, copy=True)
    if vals.size <= 1:
        return vals
    # Keep values in [-180, 180] so amplitude remains interpretable.
    wrapped = (vals + 180.0) % 360.0 - 180.0
    # Break line segments at branch-crossings to avoid solid vertical blocks.
    jumps = np.abs(np.diff(wrapped)) > 180.0
    wrapped[1:][jumps] = np.nan
    return wrapped


def parse_utc_lines(path: Path) -> list[datetime]:
    times: list[datetime] = []
    pat = re.compile(
        r"(\d{4}):(\d{2}):(\d{2}):(\d{2}):(\d{2}):(\d{2})\.(\d+)"
    )
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = pat.search(line)
        if not m:
            continue
        y, mo, d, h, mi, s, frac = m.groups()
        usec = int(frac[:6].ljust(6, "0"))
        times.append(
            datetime(
                int(y), int(mo), int(d), int(h), int(mi), int(s), usec,
                tzinfo=timezone.utc,
            )
        )
    return times


def load_gmat_spline(path: Path) -> pd.DataFrame:
    rows = []
    pat = re.compile(
        r"(\d{4}):(\d{2}):(\d{2}):(\d{2}):(\d{2}):(\d{2})\.(\d+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)"
    )
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = pat.match(line.strip())
        if not m:
            continue
        y, mo, d, h, mi, s, frac, x, yk, z, vx, vy, vz = m.groups()
        usec = int(frac[:6].ljust(6, "0"))
        t = datetime(
            int(y), int(mo), int(d), int(h), int(mi), int(s), usec,
            tzinfo=timezone.utc,
        )
        rows.append(
            {
                "utc": t,
                "t_sec": (t - EPOCH_UTC).total_seconds(),
                "x_km": float(x),
                "y_km": float(yk),
                "z_km": float(z),
                "vx_km_s": float(vx),
                "vy_km_s": float(vy),
                "vz_km_s": float(vz),
            }
        )
    return pd.DataFrame(rows)


def _spline_interp_cols(spline: pd.DataFrame, t_query: np.ndarray) -> pd.DataFrame:
    t_knot = spline["t_sec"].to_numpy()
    cols = ["x_km", "y_km", "z_km", "vx_km_s", "vy_km_s", "vz_km_s"]
    out = {}
    for c in cols:
        cs = CubicSpline(t_knot, spline[c].to_numpy())
        out[c] = cs(t_query)
    return pd.DataFrame(out)


def interp_spline_at_utc(spline: pd.DataFrame, utc_times: list[datetime | None]) -> pd.DataFrame:
    """Interpolate GMAT spline at each 42 UTC sample (seconds since mission epoch)."""
    t_query = np.array(
        [
            (u - EPOCH_UTC).total_seconds() if u is not None else np.nan
            for u in utc_times
        ],
        dtype=float,
    )
    valid = np.isfinite(t_query)
    out = {c: np.full(len(t_query), np.nan) for c in
           ["x_km", "y_km", "z_km", "vx_km_s", "vy_km_s", "vz_km_s"]}
    if valid.any():
        ref = _spline_interp_cols(spline, t_query[valid])
        for c in ref.columns:
            out[c][valid] = ref[c].to_numpy()
    return pd.DataFrame(out)


def load_42_telemetry(run_dir: Path) -> pd.DataFrame:
    t = read_scalar_file(run_dir / "time.42")
    pos = read_vector_file(run_dir / "PosN.42", 3)
    vel = read_vector_file(run_dir / "VelN.42", 3)
    q = read_vector_file(run_dir / "qbn.42", 4)
    w = read_vector_file(run_dir / "wbn.42", 3)
    rpy = read_vector_file(run_dir / "RPY.42", 3)
    n = min(len(t), len(pos), len(vel), len(q), len(w), len(rpy))

    thr_path = run_dir / "Thr.42"
    thr = (
        read_scalar_file(thr_path)[:n]
        if thr_path.is_file() and thr_path.stat().st_size > 0
        else np.zeros(n)
    )
    n = min(n, len(thr))

    st_valid_path = run_dir / "StValid.42"
    st_valid = (
        read_scalar_file(st_valid_path)[:n]
        if st_valid_path.is_file() and st_valid_path.stat().st_size > 0
        else None
    )
    if st_valid is not None:
        n = min(n, len(st_valid))

    eclipse_path = run_dir / "Eclipse.42"
    eclipse = (
        read_scalar_file(eclipse_path)[:n]
        if eclipse_path.is_file() and eclipse_path.stat().st_size > 0
        else None
    )
    if eclipse is not None:
        n = min(n, len(eclipse))

    utc_path = run_dir / "UTC.42"
    utc_list = parse_utc_lines(utc_path) if utc_path.is_file() else []
    utc = utc_list[:n] if len(utc_list) >= n else [None] * n

    df = pd.DataFrame(
        {
            "t_sec": t[:n],
            "utc": utc,
            "pos_x_m": pos[:n, 0],
            "pos_y_m": pos[:n, 1],
            "pos_z_m": pos[:n, 2],
            "vel_x_m_s": vel[:n, 0],
            "vel_y_m_s": vel[:n, 1],
            "vel_z_m_s": vel[:n, 2],
            "q_x": q[:n, 0],
            "q_y": q[:n, 1],
            "q_z": q[:n, 2],
            "q_s": q[:n, 3],
            "w_x_rad_s": w[:n, 0],
            "w_y_rad_s": w[:n, 1],
            "w_z_rad_s": w[:n, 2],
            "roll_deg": rpy[:n, 0],
            "pitch_deg": rpy[:n, 1],
            "yaw_deg": rpy[:n, 2],
            "thruster_force_N": thr[:n],
        }
    )
    if st_valid is not None:
        df["st_valid"] = st_valid[:n]
    if eclipse is not None:
        df["eclipse"] = eclipse[:n]
    return df


def add_pointing_metrics(df: pd.DataFrame) -> pd.DataFrame:
    thrust_err = []
    nadir_err = []
    w_mag = []
    radius_km = []

    for _, row in df.iterrows():
        cbn = quat_to_dcm(np.array([row.q_x, row.q_y, row.q_z, row.q_s]))
        vel_n = np.array([row.vel_x_m_s, row.vel_y_m_s, row.vel_z_m_s])
        pos_n = np.array([row.pos_x_m, row.pos_y_m, row.pos_z_m])

        thrust_n = cbn @ THRUSTER_AXIS_BODY
        nadir_n = cbn @ NADIR_AXIS_BODY

        if np.linalg.norm(vel_n) > 1.0:
            thrust_err.append(angle_deg(thrust_n, vel_n))
        else:
            thrust_err.append(np.nan)

        if np.linalg.norm(pos_n) > 1.0:
            nadir_err.append(angle_deg(nadir_n, -pos_n))
        else:
            nadir_err.append(np.nan)

        w = np.array([row.w_x_rad_s, row.w_y_rad_s, row.w_z_rad_s])
        w_mag.append(np.degrees(np.linalg.norm(w)))
        radius_km.append(np.linalg.norm(pos_n) / 1000.0)

    df = df.copy()
    df["thrust_align_err_deg"] = thrust_err
    df["nadir_align_err_deg"] = nadir_err
    df["w_mag_deg_s"] = w_mag
    df["radius_km"] = radius_km
    return df


def add_gmat_comparison(df: pd.DataFrame, spline: pd.DataFrame) -> pd.DataFrame:
    utc_times = df["utc"].tolist()
    if not all(u is None for u in utc_times):
        ref = interp_spline_at_utc(spline, utc_times)
    else:
        tt_offset = 37.0 + 32.184
        t_lookup = df["t_sec"].to_numpy() + tt_offset
        ref = _spline_interp_cols(spline, t_lookup)
    ref_pos_m = ref[["x_km", "y_km", "z_km"]].to_numpy() * 1000.0
    ref_vel = ref[["vx_km_s", "vy_km_s", "vz_km_s"]].to_numpy() * 1000.0
    pos = df[["pos_x_m", "pos_y_m", "pos_z_m"]].to_numpy()
    vel = df[["vel_x_m_s", "vel_y_m_s", "vel_z_m_s"]].to_numpy()

    df = df.copy()
    df["gmat_radius_km"] = np.linalg.norm(ref_pos_m, axis=1) / 1000.0
    df["pos_err_m"] = np.linalg.norm(pos - ref_pos_m, axis=1)
    df["vel_err_m_s"] = np.linalg.norm(vel - ref_vel, axis=1)
    return df


def make_plots(df: pd.DataFrame, out_dir: Path, label: str = "A3 LEO-GEO") -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t_hr = df["t_sec"] / 3600.0

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    fig.suptitle(f"{label} — 42 ADCS pointing validation", fontsize=13)

    ax = axes[0, 0]
    ax.plot(t_hr, df["thrust_align_err_deg"], color="#d95f02", lw=1.2)
    ax.set_ylabel("deg")
    ax.set_title("+X thruster vs inertial velocity")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[0, 1]
    ax.plot(t_hr, df["nadir_align_err_deg"], color="#1b9e77", lw=1.2)
    ax.set_ylabel("deg")
    ax.set_title("−Z body vs nadir (Earth)")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[1, 0]
    ax.plot(t_hr, df["w_mag_deg_s"], color="#7570b3", lw=1.2)
    ax.set_ylabel("deg/s")
    ax.set_title("Angular rate magnitude |ω|")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")

    ax = axes[1, 1]
    roll_plot = _prepare_wrapped_angle_for_plot(df["roll_deg"])
    pitch_plot = _prepare_wrapped_angle_for_plot(df["pitch_deg"])
    yaw_plot = _prepare_wrapped_angle_for_plot(df["yaw_deg"])
    ax.plot(t_hr, roll_plot, label="Roll", lw=1.0)
    ax.plot(t_hr, pitch_plot, label="Pitch", lw=1.0)
    ax.plot(t_hr, yaw_plot, label="Yaw", lw=1.0)
    ax.set_ylabel("deg")
    ax.set_title("Attitude RPY (L-frame, wrapped)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("Elapsed time [hr]")
    fig.savefig(out_dir / "a3_adcs_pointing.png", dpi=150)
    plt.close(fig)

    if df["thruster_force_N"].max() > 1e-9:
        fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
        ax.plot(t_hr, df["thruster_force_N"] * 1000.0, color="#e6ab02", lw=1.2)
        # Fixed y-range improves visibility of mN-level variations over mission time.
        ax.set_ylim(0.0, 20.0)
        ax.set_ylabel("Thrust [mN]")
        ax.set_xlabel("Elapsed time [hr]")
        ax.set_title("Electric thruster — PROPORTIONAL mode (ThrustLevelCmd = 1.0)")
        ax.grid(True, alpha=0.3)
        fig.savefig(out_dir / "a3_thruster_force.png", dpi=150)
        plt.close(fig)

    if "pos_err_m" in df.columns:
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
        fig.suptitle("A3 — 42 reference orbit vs GMAT SPLINE", fontsize=13)

        ax = axes[0]
        ax.plot(t_hr, df["radius_km"], label="42", lw=1.2)
        ax.plot(t_hr, df["gmat_radius_km"], "--", label="GMAT spline", lw=1.2)
        ax.set_ylabel("Radius [km]")
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(t_hr, df["pos_err_m"], color="#e7298a", lw=1.2)
        ax.set_ylabel("Position error [m]")
        ax.set_xlabel("Elapsed time [hr]")
        ax.grid(True, alpha=0.3)
        fig.savefig(out_dir / "a3_orbit_vs_gmat.png", dpi=150)
        plt.close(fig)


def write_summary(df: pd.DataFrame, out_dir: Path, label: str = "A3") -> None:
    rw_mode = (
        "RW" in label.upper()
        or "REAL" in label.upper()
        or "ADVANCED" in label.upper()
        or "N9" in label.upper()
    )
    lines = [
        f"# {label} — 42 validation summary",
        "",
        f"- Samples: **{len(df)}**",
        f"- Sim span: **{df['t_sec'].iloc[0]:.1f} – {df['t_sec'].iloc[-1]:.1f} s** "
        f"({df['t_sec'].iloc[-1]/3600:.3f} hr)",
        "",
        "## ADCS pointing (PROTOTYPE_FSW + Point commands)",
        "",
        "| Metric | Mean | Max | Final |",
        "|--------|------|-----|-------|",
        f"| Thrust–velocity error [deg] | {df['thrust_align_err_deg'].mean():.3f} "
        f"| {df['thrust_align_err_deg'].max():.3f} "
        f"| {df['thrust_align_err_deg'].iloc[-1]:.3f} |",
        f"| Nadir error [deg] | {df['nadir_align_err_deg'].mean():.3f} "
        f"| {df['nadir_align_err_deg'].max():.3f} "
        f"| {df['nadir_align_err_deg'].iloc[-1]:.3f} |",
        f"| |ω| [deg/s] | {df['w_mag_deg_s'].mean():.4f} "
        f"| {df['w_mag_deg_s'].max():.4f} "
        f"| {df['w_mag_deg_s'].iloc[-1]:.4f} |",
        "",
    ]
    if "pos_err_m" in df.columns:
        lines.extend(
            [
                "## Orbit (FIXED on GMAT SPLINE)",
                "",
                f"- Mean position error vs spline: **{df['pos_err_m'].mean():.3f} m**",
                f"- Max position error: **{df['pos_err_m'].max():.3f} m**",
                f"- Mean velocity error: **{df['vel_err_m_s'].mean():.4f} m/s**",
                "",
            ]
        )
    if df["thruster_force_N"].max() > 1e-6:
        lines.append(
            f"- Thruster force: mean **{df['thruster_force_N'].mean()*1000:.2f} mN**, "
            f"max **{df['thruster_force_N'].max()*1000:.2f} mN** "
            f"(GMAT target 50 mN)"
        )
    else:
        lines.append(
            "- Thruster force log is zero — add "
            "`0.0 SC[0].AC.Thr[0].ThrustLevelCmd = 1.0` to `Inp_Cmd.txt`."
        )

    if "st_valid" in df.columns:
        st_frac = float((df["st_valid"] > 0.5).mean())
        lines.append(f"- Star tracker valid fraction: **{100.0*st_frac:.2f}%**")
    if "eclipse" in df.columns:
        ecl_frac = float((df["eclipse"] > 0.5).mean())
        lines.append(f"- Eclipse fraction: **{100.0*ecl_frac:.2f}%**")

    if "st_valid" in df.columns:
        spike = df["thrust_align_err_deg"] > 20.0
        if spike.any():
            st_off_spike = (df.loc[spike, "st_valid"] <= 0.5).mean()
            lines.append(
                f"- Thrust spikes (>20 deg) with ST invalid: **{100.0*float(st_off_spike):.2f}%**"
            )
    if "eclipse" in df.columns:
        spike = df["thrust_align_err_deg"] > 20.0
        if spike.any():
            ecl_spike = (df.loc[spike, "eclipse"] > 0.5).mean()
            lines.append(
                f"- Thrust spikes (>20 deg) during eclipse: **{100.0*float(ecl_spike):.2f}%**"
            )

    lines.extend(
        [
            "",
            "## Environment (GMAT-aligned)",
            "",
            "| Effect | 42 flag | GMAT / notes |",
            "|--------|---------|--------------|",
            "| Drag | ON (shadows) | NRLMSISE00, F10.7=150, Ap=3, Cd=2.2 |",
            "| SRP | ON (shadows) | Flux implicit 1367 W/m²; `A3_sphere` r≈0.178 m → ~0.1 m² |",
            "| Gravity gradient | ON | Disturbance torque on attitude |",
            "| Orbit drag/SRP | OFF (`GravPert`) | **FIXED** SPLINE — env forces do not re-prop orbit |",
            "",
            "## Model fidelity (what is / is not being validated)",
            "",
            "| Aspect | Current setup | Implication |",
            "|--------|---------------|-------------|",
            (
                "| Attitude control | `PROTOTYPE_FSW` + **4 RW pyramid** + MTB unload | "
                "WheelProcessing; IdealTrq cancels thrust offset torque on body |"
                if rw_mode
                else "| Attitude control | `PROTOTYPE_FSW` + **IdealTrq** | Perfect torque; rejects aero/SRP/gg |"
            ),
            "| Orbit | **FIXED** on GMAT SPLINE | Trajectory imposed; thrust does not change orbit |",
            (
                "| Thruster | PROPORTIONAL @ **+Y offset 0.01 m**, node 1 | Native r×F; IdealTrq cancels body torque |"
                if rw_mode
                else "| Thruster | PROPORTIONAL @ CM, 0.05 N | Zero moment arm (no thrust torque) |"
            ),
            (
                "| Sensors | 3-axis gyro + ST + mag (noisy) | Estimated `qbn`/`wbn`, not truth |"
                if rw_mode
                else "| Sensors | 0 gyros / ST | Truth attitude used by FSW (no noise) |"
            ),
            "| Environment | Aero + SRP + gg **ON** | Disturbance torques on attitude |",
            "| `THR_FSW` | Not used | Demo formation controller; ignores `Point` cmds |",
            "",
            (
                "Pointing error reflects RW saturation, thrust misalignment torque, sensor noise, "
                "and environmental disturbances — a more realistic ADCS stress test than IdealTrq."
                if rw_mode
                else "Steady-state pointing error remains small because **IdealTrq** cancels "
                "environmental torques. Transient peaks at t≈0 reflect initial attitude before settle."
            ),
            "",
        ]
    )
    (out_dir / "validation_summary.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    root = _repo_root()
    default_run = Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark")
    default_spline = Path(
        "/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark/a3_gmat_spline.txt"
    )
    default_out = root / "data/benchmark_a/nasa42/a3_leogeo"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=default_run)
    parser.add_argument("--spline", type=Path, default=default_spline)
    parser.add_argument("--out-dir", type=Path, default=default_out)
    parser.add_argument(
        "--label",
        default="A3 LEO-GEO",
        help="Plot title prefix (e.g. 'A3 ideal' vs 'A3 RW')",
    )
    args = parser.parse_args()

    df = load_42_telemetry(args.run_dir)
    df = add_pointing_metrics(df)

    if args.spline.is_file():
        spline = load_gmat_spline(args.spline)
        df = add_gmat_comparison(df, spline)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out_dir / "a3_42_telemetry.csv", index=False)
    make_plots(df, args.out_dir, label=args.label)
    write_summary(df, args.out_dir, label=args.label)

    print(f"Wrote {args.out_dir / 'a3_42_telemetry.csv'}")
    print(f"Wrote {args.out_dir / 'a3_adcs_pointing.png'}")
    if (args.out_dir / "a3_thruster_force.png").is_file():
        print(f"Wrote {args.out_dir / 'a3_thruster_force.png'}")
    if "pos_err_m" in df.columns:
        print(f"Wrote {args.out_dir / 'a3_orbit_vs_gmat.png'}")
    print(f"Wrote {args.out_dir / 'validation_summary.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
