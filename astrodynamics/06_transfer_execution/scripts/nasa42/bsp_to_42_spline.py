#!/usr/bin/env python3
"""Convert GMAT SPK ephemeris (.bsp) to NASA 42 SPLINE reference-orbit file."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import spiceypy as spice

# GMAT Benchmark A3 spacecraft NAIF ID (from Benchmark_A3_LEO_GEO_Transfer.script)
DEFAULT_TARGET_ID = -10008001
DEFAULT_FRAME = "J2000"
DEFAULT_OBSERVER = "EARTH"


def _utc_colon(et: float) -> str:
    """Format epoch for 42 SPLINE: YYYY:MM:DD:HH:MM:SS.ssssss (UTC)."""
    parts = spice.et2utc(et, "C", 6).split()
    # parts: year mon day hr min sec.sss
    year = parts[0]
    mon = _month_num(parts[1])
    day = parts[2]
    time = parts[3]
    if "." not in time:
        time = f"{time}.000000"
    else:
        base, frac = time.split(".", 1)
        time = f"{base}.{frac[:6].ljust(6, '0')}"
    hr, mn, sc = time.split(":")
    return f"{year}:{mon}:{day}:{hr}:{mn}:{sc}"


def _month_num(mon: str) -> str:
    names = ["JAN", "FEB", "MAR", "APR", "MAY", "JUN",
             "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"]
    return f"{names.index(mon.upper()) + 1:02d}"


def load_kernels(bsp_path: Path, lsk_path: Path | None) -> None:
    spice.kclear()
    if lsk_path is not None:
        spice.furnsh(str(lsk_path))
    spice.furnsh(str(bsp_path))


def coverage_et(bsp_path: Path, target_id: int) -> tuple[float, float]:
    """Return (et_start, et_stop) for the SPK segment."""
    et_start = None
    et_stop = None
    step = 60.0
    # GMAT A3 mission window (wide search)
    et_probe = spice.str2et("2026 JUN 15 00:00:00 UTC")
    et_end_probe = spice.str2et("2026 SEP 01 00:00:00 UTC")
    while et_probe <= et_end_probe:
        try:
            spice.spkezr(str(target_id), et_probe, DEFAULT_FRAME, "NONE", DEFAULT_OBSERVER)
            et_start = et_probe
            break
        except spice.utils.exceptions.SpiceSPKINSUFFDATA:
            et_probe += step
    if et_start is None:
        raise RuntimeError(f"No SPK coverage found for target {target_id} in {bsp_path}")

    et_probe = et_end_probe
    while et_probe >= et_start:
        try:
            spice.spkezr(str(target_id), et_probe, DEFAULT_FRAME, "NONE", DEFAULT_OBSERVER)
            et_stop = et_probe
            break
        except spice.utils.exceptions.SpiceSPKINSUFFDATA:
            et_probe -= step
    if et_stop is None:
        raise RuntimeError(f"Could not determine SPK end time for target {target_id}")

    return et_start, et_stop


def first_valid_et(target_id: int, et_min: float) -> float:
    """Return earliest ET >= et_min with SPK state data (kernels must be loaded)."""
    et = et_min
    for _ in range(int(3600)):  # search up to 1 h
        try:
            spice.spkezr(str(target_id), et, DEFAULT_FRAME, "NONE", DEFAULT_OBSERVER)
            return et
        except spice.utils.exceptions.SpiceSPKINSUFFDATA:
            et += 1.0
    raise RuntimeError(f"No SPK coverage found for target {target_id} at or after ET={et_min}")


def convert_bsp_to_spline(
    bsp_path: Path,
    out_path: Path,
    lsk_path: Path | None,
    target_id: int,
    step_sec: float,
    et_start: float | None,
    et_stop: float | None,
) -> int:
    load_kernels(bsp_path, lsk_path)
    if et_start is None or et_stop is None:
        cov_start, cov_stop = coverage_et(bsp_path, target_id)
        et_start = cov_start if et_start is None else et_start
        et_stop = cov_stop if et_stop is None else et_stop

    first_valid = first_valid_et(target_id, et_start)

    if et_stop < first_valid:
        raise ValueError("et_stop must be >= first_valid SPK time")

    et_mission_stop = et_stop
    # NASA 42 advances a 4-node cubic buffer while DynTime > NodeDynTime[2].
    # Pad past mission stop so the last sim seconds do not hit spline EOF.
    et_stop += 3.0 * step_sec
    last_state = spice.spkezr(
        str(target_id), et_mission_stop, DEFAULT_FRAME, "NONE", DEFAULT_OBSERVER
    )[0]

    # 42 init requires DynTime >= knot[1]; prepend two steps before first valid knot.
    sample_start = first_valid - 2.0 * step_sec

    out_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    et = sample_start
    with out_path.open("w", encoding="utf-8") as fout:
        while et <= et_stop + 1e-6:
            query_et = min(max(et, first_valid), et_mission_stop)
            if query_et >= et_mission_stop - 1e-6:
                state = last_state
            else:
                state, _ = spice.spkezr(
                    str(target_id), query_et, DEFAULT_FRAME, "NONE", DEFAULT_OBSERVER
                )
            # SPICE spkezr returns km and km/s for SPK states
            x, y, z, vx, vy, vz = state
            fout.write(
                f"{_utc_colon(et)} {x:.6f} {y:.6f} {z:.6f} "
                f"{vx:.6f} {vy:.6f} {vz:.6f}\n"
            )
            count += 1
            et += step_sec

        if et - step_sec < et_stop - 1e-6:
            x, y, z, vx, vy, vz = last_state
            fout.write(
                f"{_utc_colon(et_stop)} {x:.6f} {y:.6f} {z:.6f} "
                f"{vx:.6f} {vy:.6f} {vz:.6f}\n"
            )
            count += 1

    return count


def main() -> int:
    repo = Path(__file__).resolve().parents[3]
    default_bsp = (
        repo / "data/benchmark_a/gmat/a3_leo_geo_transfer/a3_reference.bsp"
    )
    default_lsk = repo / "benchmarks/benchmark_a/nasa42/kernels/naif0012.tls"
    default_out = Path(
        "/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark/a3_gmat_spline.txt"
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bsp", type=Path, default=default_bsp, help="Input GMAT SPK file")
    parser.add_argument("--lsk", type=Path, default=default_lsk, help="Leap-second kernel")
    parser.add_argument("--out", type=Path, default=default_out, help="Output 42 SPLINE file")
    parser.add_argument("--target-id", type=int, default=DEFAULT_TARGET_ID)
    parser.add_argument("--step", type=float, default=60.0, help="Sample step [s]")
    parser.add_argument("--utc-start", type=str, default=None, help="UTC start (override)")
    parser.add_argument("--utc-stop", type=str, default=None, help="UTC stop (override)")
    args = parser.parse_args()

    if not args.bsp.is_file():
        print(f"ERROR: BSP not found: {args.bsp}", file=sys.stderr)
        return 1
    if not args.lsk.is_file():
        print(f"ERROR: LSK not found: {args.lsk}", file=sys.stderr)
        return 1

    load_kernels(args.bsp, args.lsk)
    et_start = spice.str2et(args.utc_start) if args.utc_start else None
    et_stop = spice.str2et(args.utc_stop) if args.utc_stop else None

    n = convert_bsp_to_spline(
        args.bsp, args.out, args.lsk, args.target_id, args.step, et_start, et_stop
    )
    print(f"Wrote {n} SPLINE nodes to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
