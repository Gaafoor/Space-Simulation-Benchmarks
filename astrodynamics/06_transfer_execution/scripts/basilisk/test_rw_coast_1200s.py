#!/usr/bin/env python3
"""
Regression test for Basilisk GMAT handoff RW-only velocity pointing.

Runs the 1200s coast case with GNC enabled and MTB disabled and checks:
- execution validation "attitude_pointing" passes
- max pointing error stays within a tight tolerance
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Optional

import numpy as np

from run_basilisk_gmat_gnc_handoff import run


def _max_pointing_error_deg(pointing_csv: Path) -> float:
    if not pointing_csv.is_file():
        return float("nan")
    vals: list[float] = []
    with pointing_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            s = (row.get("pointing_error_deg") or "").strip()
            if not s:
                continue
            try:
                v = float(s)
            except ValueError:
                continue
            if math.isfinite(v):
                vals.append(v)
    return max(vals) if vals else float("nan")


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    workspace = script_dir.parent.parent
    default_schedule = workspace / "data/pipeline_handoff/gmat/burn_schedule.json"

    p = argparse.ArgumentParser()
    p.add_argument("--schedule", type=Path, default=default_schedule)
    p.add_argument("--out-dir", type=Path, default=workspace / "data/pipeline_handoff/basilisk/tests/rw_coast_1200s")
    p.add_argument("--max-time-s", type=float, default=1200.0)
    p.add_argument("--max-pointing-error-deg", type=float, default=0.01)
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    summary = run(
        args.schedule,
        args.out_dir,
        max_time_s=float(args.max_time_s),
        enable_gnc=True,
        enable_mtb=False,
        force_mode="gmat_match",
        vizard=None,
    )

    exec_check = summary.get("execution_validation_checks", {}).get("attitude_pointing", {})
    passed = bool(exec_check.get("passed", False))
    value_deg = float(exec_check.get("value_deg", float("nan")))
    max_csv = _max_pointing_error_deg(args.out_dir / "pointing_error_history.csv")

    ok = True
    if not passed:
        ok = False
    if not math.isfinite(value_deg) or value_deg > float(args.max_pointing_error_deg):
        ok = False
    if not math.isfinite(max_csv) or max_csv > float(args.max_pointing_error_deg):
        ok = False

    print(json.dumps({
        "passed": ok,
        "attitude_pointing_check": exec_check,
        "max_csv_pointing_error_deg": max_csv,
        "out_dir": str(args.out_dir),
    }, indent=2))

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

