#!/usr/bin/env python3
"""Generate NASA 42 Inp_Cmd.txt from GMAT burn_schedule.json (N9 timeline)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


DEFAULT_SCHEDULE = _repo_root() / "data/pipeline_handoff/gmat/burn_schedule.json"
DEFAULT_OUT = Path("/home/mahmedext/Downloads/42 20240919/42/leogeo_benchmark/Inp_Cmd.txt")


def thrust_events(schedule: dict) -> list[tuple[float, float]]:
    """Return (time_s, thrust_level_cmd) pairs sorted by time."""
    events: list[tuple[float, float]] = [(0.0, 0.0)]
    for burn in sorted(schedule.get("burns", []), key=lambda b: float(b.get("start_s", 0.0))):
        btype = burn.get("type", "")
        if btype not in {"finite_thrust", "electric_spiral"}:
            continue
        start = float(burn.get("start_s", 0.0))
        end = start + float(burn.get("duration_s", 0.0))
        if burn.get("gmat_end_s") is not None:
            end = float(burn["gmat_end_s"])
        events.append((start, 1.0))
        events.append((end, 0.0))
    events.sort(key=lambda x: x[0])
    merged: list[tuple[float, float]] = []
    for t, level in events:
        if merged and abs(merged[-1][0] - t) < 1e-9:
            merged[-1] = (t, level)
        else:
            merged.append((t, level))
    return merged


def render_inp_cmd(schedule: dict) -> str:
    lines = [
        "<<<<<<<<<<<<<<<<<  42:  Command Script File  >>>>>>>>>>>>>>>>>",
        "% A3 LEO-GEO full mission — auto-generated from burn_schedule.json (N9)",
        "0.0 SC[0] FswTag = PROTOTYPE_FSW",
        "0.0 Point SC[0].B[0] Primary Vector [0.0 0.0 -1.0] at EARTH",
        "0.0 Point SC[0].B[0] Secondary Vector [1.0 0.0 0.0] at VELOCITY",
    ]
    for t_s, level in thrust_events(schedule):
        lines.append(f"{t_s:.15g} SC[0].AC.Thr[0].ThrustLevelCmd = {level:.1f}")
    lines.append("EOF")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    schedule = json.loads(args.schedule.read_text(encoding="utf-8"))
    text = render_inp_cmd(schedule)
    if args.dry_run:
        print(text)
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    print(f"Wrote {args.out}")
    for t_s, level in thrust_events(schedule):
        print(f"  t={t_s:14.6f} s  ThrustLevelCmd={level:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
