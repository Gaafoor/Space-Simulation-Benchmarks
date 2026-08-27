#!/usr/bin/env python3
"""Run NASA 42 full mission, monitor, then validate/plot on success."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

WS = Path("/media/windows/Users/mahmed.ext/Downloads/Intern_ws")
BIN_DIR = Path("/home/mahmedext/Downloads/42 20240919/42")
OUT = WS / "data/benchmark_a/nasa42/a3_leogeo_full_advanced"
LOG = OUT / "run_stdout.log"
MONITOR = WS / "catec_ws/benchmarks/benchmark_a/nasa42/monitor_42_run.py"
VALIDATE = WS / "catec_ws/benchmarks/benchmark_a/nasa42/validate_42_n9_full.py"
PLOT = WS / "catec_ws/benchmarks/benchmark_a/nasa42/plot_42_results.py"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    with LOG.open("w", encoding="utf-8") as logf:
        proc = subprocess.Popen(
            [str(BIN_DIR / "42"), "leogeo_benchmark"],
            cwd=str(BIN_DIR),
            stdout=logf,
            stderr=subprocess.STDOUT,
        )
    print(f"Started 42 pid={proc.pid} log={LOG}")
    while proc.poll() is None:
        subprocess.run([sys.executable, str(MONITOR), "--log", str(LOG)], check=False)
        rc = subprocess.run([sys.executable, str(MONITOR), "--log", str(LOG)]).returncode
        if rc == 2:
            proc.kill()
            print("ABORT: monitor flagged failure", file=sys.stderr)
            return 2
        time.sleep(300)
    if proc.returncode != 0:
        print(f"42 exited with code {proc.returncode}", file=sys.stderr)
        tail = LOG.read_text(encoding="utf-8", errors="replace")[-2000:]
        print(tail, file=sys.stderr)
        return proc.returncode or 1
    if "NaN" in LOG.read_text(encoding="utf-8", errors="replace"):
        print("ABORT: NaN in log", file=sys.stderr)
        return 1
    subprocess.run([sys.executable, str(VALIDATE), "--out-dir", str(OUT)], check=True)
    subprocess.run(
        [sys.executable, str(PLOT), "--out-dir", str(OUT), "--label", "A3 N9 advanced E2E"],
        check=True,
    )
    print("COMPLETE: validate + plot done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
