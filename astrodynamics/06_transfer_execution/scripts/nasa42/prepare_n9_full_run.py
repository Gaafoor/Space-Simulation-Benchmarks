#!/usr/bin/env python3
"""Prepare NASA 42 leogeo_benchmark for N9 full-mission execution."""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path


CASE_DIR = Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark")
BIN_42 = Path("/home/mahmedext/Downloads/42 20240919/42/42")
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SCHEDULE = SCRIPT_DIR.parents[3] / "data/pipeline_handoff/gmat/burn_schedule.json"
OUT_BASE = SCRIPT_DIR.parents[3] / "data/benchmark_a/nasa42"

EARTH_GRAV_N = 18
EARTH_GRAV_M = 18
GEOM_FILE = "A3_facet_L1.obj"
RW4_AXIS = (0.5773502691896258, 0.5773502691896258, 0.5773502691896258)
HANDOFF_INERTIA = (0.15, 0.18, 0.12)
ADVANCED_DT_SIM = 1.0
MISSION_LINE_RE = re.compile(r"^(\d+(?:\.\d+)?)\s+(\d+(?:\.\d+)?)\s+!", re.I)


def _is_mission_duration_line(line: str) -> bool:
    m = MISSION_LINE_RE.match(line.strip())
    if not m:
        return False
    # Mission/smoke stop-time line (not IGRF/gravity pairs like 8 8 or 18 18).
    t_stop = float(m.group(1))
    dt = float(m.group(2))
    return t_stop >= 1_000.0 and dt <= 10.0


_SPICE_MONTHS = ("JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC")


def _parse_epoch_iso(epoch_iso: str) -> datetime:
    text = epoch_iso.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    return datetime.fromisoformat(text)


def _dt_to_spice_utc(dt: datetime) -> str:
    if dt.tzinfo is not None:
        dt = dt.astimezone(timezone.utc).replace(tzinfo=None)
    return (
        f"{dt.year} {_SPICE_MONTHS[dt.month - 1]} {dt.day:02d} "
        f"{dt.hour:02d}:{dt.minute:02d}:{dt.second:02d} UTC"
    )


def _mission_end_utc(mission_end_s: float, epoch_iso: str) -> str:
    end = _parse_epoch_iso(epoch_iso) + timedelta(seconds=float(mission_end_s))
    return _dt_to_spice_utc(end)


def _epoch_to_spice_utc(epoch_iso: str) -> str:
    return _dt_to_spice_utc(_parse_epoch_iso(epoch_iso))


def _find_python_with_spiceypy() -> str:
    """Return a Python executable that can import spiceypy (for BSP spline export)."""
    candidates: list[str] = [sys.executable]
    for name in ("python3", "python"):
        found = shutil.which(name)
        if found:
            candidates.append(found)
    for path in (
        Path.home() / "anaconda3/bin/python3",
        Path.home() / "miniconda3/bin/python3",
    ):
        if path.is_file():
            candidates.append(str(path))

    seen: set[str] = set()
    for exe in candidates:
        if not exe or exe in seen:
            continue
        seen.add(exe)
        try:
            subprocess.run(
                [exe, "-c", "import spiceypy"],
                check=True,
                capture_output=True,
                text=True,
            )
            return exe
        except (subprocess.CalledProcessError, OSError):
            continue

    raise RuntimeError(
        "spiceypy is required to regenerate a3_gmat_spline.txt. "
        "Install it in your active Python (pip install spiceypy) or use Anaconda python3."
    )


def _date_to_time(year: int, month: int, day: int, hour: int, minute: int, second: float) -> float:
    """Match NASA 42 Kit/Source/timekit.c DateToTime (seconds since J2000)."""
    if month < 3:
        year -= 1
        month += 12
    a = year // 100
    b = 2 - a + a // 4
    days = (
        int(365.25 * (year - 2000))
        + int(30.6001 * (month + 1))
        + day
        + b
        - 50.5
    )
    return 86400.0 * days + 3600.0 * hour + 60.0 * minute + second


def _spline_knot_sim_s(line: str, epoch_iso: str, leap_sec: float = 37.0) -> float:
  y, mo, d, h, mi, se = line.split()[0].split(":")
  epoch = _parse_epoch_iso(epoch_iso)
  epoch_tt = _date_to_time(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second)
  epoch_tt += leap_sec + 32.184
  knot_tt = _date_to_time(int(y), int(mo), int(d), int(h), int(mi), float(se)) + leap_sec + 32.184
  return knot_tt - epoch_tt


def _validate_spline_coverage(spline_path: Path, mission_end_s: float, epoch_iso: str) -> None:
    lines = spline_path.read_text(encoding="utf-8").splitlines()
    if len(lines) < 75580:
        raise RuntimeError(
            f"{spline_path.name} has only {len(lines)} lines; expected >=75580 for full-mission coast. "
            "Old spiral-only splines (~75571 lines) EOF near thrust-off (~4534019 s)."
        )
    last_sim_s = _spline_knot_sim_s(lines[-1], epoch_iso)
    if last_sim_s < mission_end_s + 120.0:
        raise RuntimeError(
            f"{spline_path.name} last knot sim={last_sim_s:.1f} s < mission_end+120 s "
            f"({mission_end_s + 120.0:.1f} s). Regenerate with prepare_n9_full_run.py."
        )
    print(
        f"Spline OK: {len(lines)} nodes, last knot sim={last_sim_s:.1f} s "
        f"(mission_end={mission_end_s:.1f} s)"
    )


def _backup_once(name: str) -> None:
    src = CASE_DIR / name
    legacy = CASE_DIR / name.replace(".txt", "_legacy_3rw_sphere.txt")
    if src.is_file() and not legacy.is_file():
        shutil.copy2(src, legacy)


def _write_inp_sim_full(mission_end_s: float, *, dt_sim: float = 2.0, dt_out: float = 3600.0, advanced: bool) -> None:
    lines = (CASE_DIR / "Inp_Sim.txt").read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    for line in lines:
        m = MISSION_LINE_RE.match(line.strip())
        if m and _is_mission_duration_line(line):
            out.append(
                f"{mission_end_s:.15g}    {dt_sim:.1f}                    "
                f"!  N9 full mission (~53 days); {dt_sim:.0f} s dynamics step"
            )
        elif "File Output Interval" in line and line.strip().startswith(
            ("10.0", "60.0", "3600.0")
        ):
            out.append(
                f"{dt_out:.1f}                           "
                f"!  File Output Interval [sec] — 1 h cadence (~1,274 samples)"
            )
        elif advanced and "Earth Gravity Model" in line:
            out.append(
                f"{EARTH_GRAV_N}  {EARTH_GRAV_M}                          "
                f"!  Earth Gravity Model N and M (EGM96 max 18x18; closest to GMAT 20x20)"
            )
        else:
            out.append(line)
    text = "\n".join(out) + "\n"
    (CASE_DIR / "Inp_Sim_full.txt").write_text(text, encoding="utf-8")
    (CASE_DIR / "Inp_Sim.txt").write_text(text, encoding="utf-8")


def _configure_sc_actuator_physics(text: str) -> str:
    """Enable native thrust misalignment torque via EP thruster offset node."""
    text = text.replace(
        "A3_facet_L1.obj               ! Geometry Input File Name (L1 box + solar arrays; mesh aero/SRP)\n"
        "NONE                          ! Node File Name (thruster at CM; offset torque unobservable in Point mode)",
        "A3_facet_L1.obj               ! Geometry Input File Name (L1 box + solar arrays; mesh aero/SRP)\n"
        "Nodes_A3_EP.txt               ! Node File Name (EP +Y 0.01 m offset; native r x F torque in dynamics)",
    )
    text = text.replace(
        "0                             ! Node (at CM — offset thrust torque is unobservable in Point mode)",
        "1                             ! Node (EP thruster at +Y offset per Nodes_A3_EP.txt)",
    )
    return text


def _configure_sc_advanced() -> None:
    path = CASE_DIR / "SC_A3_RW.txt"
    _backup_once("SC_A3_RW.txt")
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "A3 LEO-GEO EP — realistic RW/MTB GNC (GMAT benchmark)",
        "A3 LEO-GEO EP — advanced E2E (4RW pyramid, facet mesh, EGM96 18x18)",
    )
    text = text.replace("A3_sphere.obj", GEOM_FILE)
    text = text.replace(
        "A3_sphere.obj                 ! Geometry Input File Name (r~0.178 m, SRP ~0.1 m^2)",
        f"{GEOM_FILE}               ! Geometry Input File Name (L1 box + solar arrays; mesh aero/SRP)",
    )
    text = text.replace(
        "3                             ! Number of wheels (orthogonal RW cluster)",
        "4                             ! Number of wheels (Basilisk pyramid: XYZ + body diagonal)",
    )
    text = text.replace(
        "0.08  0.08  0.05              ! Moments of Inertia (kg-m^2)",
        f"{HANDOFF_INERTIA[0]:.2f}  {HANDOFF_INERTIA[1]:.2f}  {HANDOFF_INERTIA[2]:.2f}              ! Moments of Inertia (kg-m^2) — Basilisk handoff",
    )
    text = re.sub(
        r"^2\.0\s+!  FSW Sample Time, sec.*$",
        f"{ADVANCED_DT_SIM:.1f}                           !  FSW Sample Time, sec (must be >= Inp_Sim DTSIM)",
        text,
        flags=re.M,
    )
    text = re.sub(
        r"^2\.0\s+! Sample Time,sec \(>= DTSIM\)$",
        f"{ADVANCED_DT_SIM:.1f}                           ! Sample Time,sec (>= DTSIM)",
        text,
        flags=re.M,
    )
    if "=============================  Wheel 3  ================================" not in text:
        wheel3 = f"""=============================  Wheel 3  ================================
0.0                           ! Initial Momentum, N-m-sec
{RW4_AXIS[0]:.9f}   {RW4_AXIS[1]:.9f}   {RW4_AXIS[2]:.9f}  ! Wheel Axis (+X,+Y,+Z body diagonal)
0.015  0.150                  ! Max Torque (N-m), Momentum (N-m-sec)
0.003                         ! Wheel Rotor Inertia, kg-m^2
0                             ! Body
0                             ! Node
NONE                          ! Jitter Input File Name
"""
        text = text.replace(
            "**************************** MTB Parameters ****************************",
            wheel3 + "**************************** MTB Parameters ****************************",
        )
    text = _configure_sc_actuator_physics(text)
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--advanced",
        action="store_true",
        help="Enable 4RW pyramid, EGM96 18x18 gravity, and L1 facet mesh aero/SRP (default for final E2E).",
    )
    parser.add_argument(
        "--legacy",
        action="store_true",
        help="Keep legacy 3RW sphere / 8x8 gravity configuration.",
    )
    args = parser.parse_args()
    advanced = args.advanced or not args.legacy

    if not CASE_DIR.is_dir():
        print(f"ERROR: 42 case folder not found: {CASE_DIR}", file=sys.stderr)
        return 1
    if not BIN_42.is_file():
        print(f"ERROR: 42 binary not found: {BIN_42}", file=sys.stderr)
        return 1
    if not DEFAULT_SCHEDULE.is_file():
        print(f"ERROR: burn schedule not found: {DEFAULT_SCHEDULE}", file=sys.stderr)
        return 1

    schedule = json.loads(DEFAULT_SCHEDULE.read_text(encoding="utf-8"))
    mission_end_s = float(schedule["dynamics"]["mission_end_s"])
    spiral = next(b for b in schedule["burns"] if b["id"] == "spiral_to_geo")
    gmat_ref = schedule.get("gmat_reference", {})

    for name in ("Inp_Sim.txt", "Inp_Cmd.txt"):
        src = CASE_DIR / name
        smoke = CASE_DIR / name.replace(".txt", "_smoke_2h.txt")
        if src.is_file() and not smoke.is_file():
            shutil.copy2(src, smoke)

    if advanced:
        mesh_script = SCRIPT_DIR / "generate_a3_facet_mesh_for_42.py"
        subprocess.run([sys.executable, str(mesh_script)], check=True)
        _configure_sc_advanced()

    gen_cmd = SCRIPT_DIR / "generate_42_inp_cmd.py"
    subprocess.run(
        [sys.executable, str(gen_cmd), "--schedule", str(DEFAULT_SCHEDULE)],
        check=True,
    )

    spline_script = SCRIPT_DIR / "bsp_to_42_spline.py"
    epoch_iso = schedule.get("mission_epoch_utc_iso", "2026-06-15T12:00:00.000Z")
    epoch_utc = _epoch_to_spice_utc(epoch_iso)
    utc_stop = _mission_end_utc(mission_end_s, epoch_iso)
    bsp_path = SCRIPT_DIR.parents[2] / "data/benchmark_a/gmat/a3_leo_geo_transfer/a3_reference.bsp"
    if not bsp_path.is_file():
        bsp_path = SCRIPT_DIR.parents[3] / "data/benchmark_a/gmat/a3_leo_geo_transfer/a3_reference.bsp"
    lsk_path = SCRIPT_DIR / "kernels/naif0012.tls"
    if bsp_path.is_file() and lsk_path.is_file():
        spice_python = _find_python_with_spiceypy()
        subprocess.run(
            [
                spice_python,
                str(spline_script),
                "--bsp",
                str(bsp_path),
                "--lsk",
                str(lsk_path),
                "--utc-start",
                epoch_utc,
                "--utc-stop",
                utc_stop,
                "--out",
                str(CASE_DIR / "a3_gmat_spline.txt"),
            ],
            check=True,
        )
        _validate_spline_coverage(CASE_DIR / "a3_gmat_spline.txt", mission_end_s, epoch_iso)
    else:
        print(f"WARNING: skipping spline regen; missing {bsp_path} or {lsk_path}", file=sys.stderr)

    dt_out = 3600.0
    dt_sim = ADVANCED_DT_SIM if advanced else 2.0
    _write_inp_sim_full(mission_end_s, dt_sim=dt_sim, dt_out=dt_out, advanced=advanced)

    out_dir = OUT_BASE / ("a3_leogeo_full_advanced" if advanced else "a3_leogeo_full")
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "profile": "advanced_e2e" if advanced else "legacy_n9",
        "mission_end_s": mission_end_s,
        "spiral_start_s": float(spiral["start_s"]),
        "spiral_end_s": float(spiral["gmat_end_s"]),
        "gmat_final_sma_km": gmat_ref.get("final_sma_km"),
        "gmat_fuel_kg": gmat_ref.get("fuel_mass_kg"),
        "output_interval_s": dt_out,
        "dynamics_step_s": dt_sim,
        "expected_output_lines": int(mission_end_s / dt_out) + 1,
        "advanced_features": {
            "reaction_wheels": 4,
            "rw_layout": "basilisk_pyramid_xyz_plus_diagonal",
            "earth_gravity": f"EGM96 {EARTH_GRAV_N}x{EARTH_GRAV_M}",
            "geometry": GEOM_FILE,
            "aero_shadows": True,
            "srp_shadows": True,
        }
        if advanced
        else {
            "reaction_wheels": 3,
            "earth_gravity": "EGM96 8x8",
            "geometry": "A3_sphere.obj",
        },
    }
    (out_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    est_wall_h = mission_end_s / 7200.0 * (16.0 / 3600.0)
    profile = "ADVANCED E2E" if advanced else "legacy N9"
    print("")
    print("=" * 72)
    print(f"N9 full-mission prep complete — {profile}")
    print("=" * 72)
    print(f"  Case folder:      {CASE_DIR}")
    print(f"  Mission duration: {mission_end_s:.3f} s  ({timedelta(seconds=int(mission_end_s))})")
    print(f"  Spiral burn:      {spiral['start_s']:.3f} s  ->  {spiral['gmat_end_s']:.3f} s")
    print(f"  Output cadence:   {dt_out:.0f} s  (~{manifest['expected_output_lines']:,} samples/file)")
    if advanced:
        print(f"  Gravity:          EGM96 {EARTH_GRAV_N}x{EARTH_GRAV_M}")
        print(f"  Geometry:         {GEOM_FILE} (mesh aero/SRP with shadows)")
        print(f"  RW cluster:       4-wheel Basilisk pyramid")
    print(f"  Post-run output:  {out_dir}")
    print(f"  Est. wall time:   ~{est_wall_h:.1f} h")
    print("")
    print("Run the full mission:")
    print("")
    print(
        "  cd /media/windows/Users/mahmed.ext/Downloads/Intern_ws && "
        "python3 catec_ws/benchmarks/benchmark_a/nasa42/prepare_n9_full_run.py && "
        'cd "/home/mahmedext/Downloads/42 20240919/42" && ./42 leogeo_benchmark'
    )
    print("")
    print("After completion, validate and plot:")
    print("")
    print(
        "  cd /media/windows/Users/mahmed.ext/Downloads/Intern_ws && "
        "python3 catec_ws/benchmarks/benchmark_a/nasa42/validate_42_n9_full.py "
        f"--out-dir {out_dir} && "
        "python3 catec_ws/benchmarks/benchmark_a/nasa42/plot_42_results.py "
        f"--out-dir {out_dir} --label 'A3 N9 advanced E2E'"
    )
    print("")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
