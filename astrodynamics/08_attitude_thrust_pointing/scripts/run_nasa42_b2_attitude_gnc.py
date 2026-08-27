#!/usr/bin/env python3
"""Benchmark B2 Attitude & GNC — NASA 42 launcher (milestone 1).

Runs the frozen B2A case using native NASA 42 input decks and archives the
simulator's ``*.42`` state logs. Metrics, plots, and battery SoC are deferred
to a later post-processing milestone (NASA 42 has no built-in EPS model).
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

from b2_attitude_config import B2Case, B2Config

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
NASA42_ROOT = Path(os.environ["NASA42_ROOT"]) if os.environ.get("NASA42_ROOT") else None


def nasa42_binary() -> Path:
    if NASA42_ROOT is None:
        raise SystemExit("Set NASA42_ROOT to your NASA 42 install directory.")
    return NASA42_ROOT / "42"
INPUT_DIR = SCRIPT_DIR / "nasa42"
RESULTS_DIR = SCRIPT_DIR / "results/nasa42_b2"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_b/b2_attitude_gnc/nasa42"

# Native truth/state logs written by NASA 42 when OutFlag is set.
STATE_LOGS = (
    "time.42",
    "DynTime.42",
    "UTC.42",
    "PosN.42",
    "VelN.42",
    "qbn.42",
    "wbn.42",
    "svn.42",
    "svb.42",
    "Hvb.42",
    "Hvn.42",
    "Hwhl.42",
    "Illum.42",
    "RPY.42",
    "u00.42",
    "x00.42",
)


def parse_case(value: str) -> B2Case:
    normalized = value.lower().strip()
    for case in B2Case:
        if normalized in {case.value, case.name.lower()}:
            return case
    raise argparse.ArgumentTypeError(f"Unknown B2 case: {value}")


def case_input_dir(case: B2Case) -> Path:
    return INPUT_DIR / case.value


def resolve_binary(explicit: Path | None) -> Path:
    if explicit is not None:
        if not explicit.is_file():
            raise FileNotFoundError(f"NASA 42 binary not found: {explicit}")
        return explicit

    env_path = os.environ.get("NASA42_BIN")
    if env_path:
        candidate = Path(env_path).expanduser()
        if candidate.is_file():
            return candidate

    binary = nasa42_binary()
    if binary.is_file():
        return binary

    raise FileNotFoundError(
        "NASA 42 executable not found. Set NASA42_BIN or install 42 under "
        f"{binary.parent}"
    )


def stage_inout_dir(source_dir: Path, case: B2Case) -> tuple[Path, Path]:
    """Stage inputs under NASA42_ROOT because 42 only accepts relative InOut paths."""
    link_name = f"BenchmarkB2_{case.value}"
    staged_dir = NASA42_ROOT / link_name
    if staged_dir.is_symlink() or staged_dir.exists():
        # If the symlink already points at our source_dir, reuse it.
        # Some environments disallow deleting the symlink because the parent
        # directory isn't writable, but the target is still correct.
        if staged_dir.is_symlink():
            try:
                current_target = Path(os.readlink(staged_dir)).resolve()
                if current_target == source_dir.resolve():
                    return staged_dir, NASA42_ROOT / link_name
            except OSError:
                # Fall through to the normal (remove+create) behavior.
                pass

        try:
            if staged_dir.is_symlink():
                staged_dir.unlink()
            elif staged_dir.is_dir():
                shutil.rmtree(staged_dir)
            else:
                staged_dir.unlink()
        except PermissionError:
            # Reuse existing staged_dir if we can't delete it.
            return staged_dir, NASA42_ROOT / link_name

    staged_dir.symlink_to(source_dir.resolve())
    return staged_dir, NASA42_ROOT / link_name


def set_graphics_enabled(inout_dir: Path, enabled: bool) -> None:
    sim_file = inout_dir / "Inp_Sim.txt"
    lines = sim_file.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if "Graphics Front End" in line:
            flag = "TRUE" if enabled else "FALSE"
            lines[index] = f"{flag:<32}!  Graphics Front End?"
            break
    sim_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def set_time_mode(inout_dir: Path, real_time: bool) -> None:
    sim_file = inout_dir / "Inp_Sim.txt"
    lines = sim_file.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if line.strip().startswith("FAST") or line.strip().startswith("REAL"):
            mode = "REAL" if real_time else "FAST"
            lines[index] = f"{mode:<32}!  Time Mode (FAST, REAL, or EXTERNAL)"
            break
    sim_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def set_sim_duration(inout_dir: Path, duration_s: float) -> None:
    sim_file = inout_dir / "Inp_Sim.txt"
    lines = sim_file.read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        if "Sim Duration" in line:
            step = line.split("!")[0].split()[1]
            lines[index] = f"{duration_s:.1f}      {step}                !  Sim Duration, Step Size [sec]"
            break
    sim_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ensure_graphics_inputs(inout_dir: Path) -> None:
    required = ("Inp_Graphics.txt", "Inp_FOV.txt")
    missing = [name for name in required if not (inout_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"Graphics enabled but missing required NASA 42 files in {inout_dir}: "
            + ", ".join(missing)
        )


def run_simulation(
    binary: Path,
    inout_dir: Path,
    staged_name: str,
    *,
    duration_s: float | None,
    graphics: bool,
    real_time: bool,
) -> tuple[float, subprocess.CompletedProcess[str | None]]:
    if duration_s is not None:
        set_sim_duration(inout_dir, duration_s)

    set_graphics_enabled(inout_dir, graphics)
    set_time_mode(inout_dir, real_time)
    if graphics:
        ensure_graphics_inputs(inout_dir)

    start = time.perf_counter()
    if graphics:
        # GUI mode: run in foreground without capturing stdio (OpenGL needs a display).
        completed = subprocess.run(
            [str(binary), staged_name],
            cwd=str(NASA42_ROOT),
            check=False,
        )
    else:
        completed = subprocess.run(
            [str(binary), staged_name],
            cwd=str(NASA42_ROOT),
            capture_output=True,
            text=True,
            check=False,
        )
    runtime_s = time.perf_counter() - start
    return runtime_s, completed


def archive_outputs(case: B2Case, inout_dir: Path, runtime_s: float, config: B2Config) -> dict:
    case_results = RESULTS_DIR / case.value
    case_data = DATA_DIR / case.value
    case_results.mkdir(parents=True, exist_ok=True)
    case_data.mkdir(parents=True, exist_ok=True)

    copied: dict[str, str] = {}
    for name in STATE_LOGS:
        source = inout_dir / name
        if not source.exists():
            continue
        for target_root in (case_results, case_data):
            target = target_root / name
            shutil.copy2(source, target)
            if target_root is case_results:
                copied[name] = str(target)

    summary = {
        "benchmark": "B2",
        "simulator": "NASA42",
        "case_id": case.value,
        "run_id": f"b2_nasa42_{case.value}",
        "status": "executed",
        "milestone": 1,
        "force_model": {
            "j2_enabled": True,
            "drag_enabled": False,
            "srp_enabled": False,
            "gravity_gradient_enabled": False,
            "sun_sensor_ideal": True,
        },
        "controller": {
            "type": "PROTOTYPE_FSW (ThreeAxisAttitudeCommand + IdealTrq)",
            "note": (
                "Sun-pointing via Inp_Cmd: Primary +Z to SUN, Secondary +Y to VELOCITY. "
                "Stock PROTOTYPE_FSW uses ideal torque actuators, not the configured wheel "
                "allocation (custom B2_FSW patch deferred)."
            ),
        },
        "implementation_notes": {
            "integration_step_s": 1.0,
            "output_sample_s": config.output_sample_s,
            "fsw_sample_s": 1.0,
            "orbit_mode": "CENTRAL KEP AE + COWELL",
            "earth_gravity_nm": "2,0 (J2 only)",
            "ephemeris": "DE430",
            "epoch_utc": config.mission_epoch_utc_iso,
            "reaction_wheels": "4-wheel pyramid configured; not driven by PROTOTYPE_FSW",
            "coarse_sun_sensor": "1x CSS on +Z, zero noise (B2A ideal)",
            "power_eps": "Not modeled in NASA 42 (deferred to post-processing milestone)",
            "plots": "Not generated in milestone 1; use archived *.42 logs",
        },
        "runtime_s": {
            "simulation": runtime_s,
            "runtime_ratio": config.simulated_duration_s / runtime_s if runtime_s > 0 else None,
        },
        "artifacts": copied,
        "platform": platform.platform(),
    }

    for target_root in (case_results, case_data):
        with (target_root / "summary.json").open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)

    return summary


def write_run_log(
    path: Path,
    config: B2Config,
    summary: dict[str, object],
    returncode: int,
    stderr: str | None,
) -> None:
    notes = summary["implementation_notes"]  # type: ignore[index]
    lines = [
        f"# NASA 42 B2 run log ({datetime.now(timezone.utc).strftime('%Y-%m-%d')})",
        "",
        "## Milestone 1 scope",
        "",
        "- Native NASA 42 input decks for frozen B2A parameters",
        "- 24 h FAST simulation with 10 s ``*.42`` state logging",
        "- No Python metrics/plot post-processing in this milestone",
        "",
        "## Configuration",
        "",
        f"- Case: `{summary['case_id']}`",
        f"- Duration: {config.simulated_duration_s / 3600.0:.1f} h",
        f"- Epoch: {config.mission_epoch_utc_iso}",
        f"- Input deck: `benchmarks/benchmark_b/nasa42/b2a/`",
        "",
        "## Force model (B2A)",
        "",
        "- J2: on (Earth gravity N=2, M=0)",
        "- Drag / SRP / gravity-gradient: off",
        "- CSS: ideal (+Z boresight)",
        "",
        "## GNC",
        "",
        "- FSW: `PROTOTYPE_FSW`",
        "- Command: Point body +Z to `SUN`, body +Y to `VELOCITY`",
        "- Actuation: ideal torque (`IdealTrq`); wheels present but not commanded",
        "",
        "## Known NASA 42 gaps vs Basilisk B2A",
        "",
        "- No battery / solar-panel / bus-load EPS model",
        "- No MRP feedback wheel mapper (would need custom `B2_FSW` in `42fsw.c`)",
        "- Metrics (`battery_soc_percent`, `pointing_error_deg`, plots) require a later post-processor on ``*.42`` logs",
        "",
        "## Execution",
        "",
        f"- Process exit code: {returncode}",
        f"- Wall clock: {summary['runtime_s']['simulation']:.3f} s",  # type: ignore[index]
        f"- Runtime ratio: {summary['runtime_s']['runtime_ratio']}",  # type: ignore[index]
        "",
        "## Archived state logs",
        "",
    ]
    artifacts = summary.get("artifacts", {})
    if artifacts:
        for name, location in sorted(artifacts.items()):
            lines.append(f"- `{name}` → `{location}`")
    else:
        lines.append("- (none captured)")

    if stderr and stderr.strip():
        lines.extend(["", "## Simulator stderr (tail)", "", "```", stderr.strip()[-4000:], "```"])

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Benchmark B2 B2A in NASA 42 (milestone 1)")
    parser.add_argument("--case", type=parse_case, default=B2Case.B2A, help="B2 case id (milestone 1: b2a only)")
    parser.add_argument("--binary", type=Path, default=None, help="Path to NASA 42 executable")
    parser.add_argument("--duration-s", type=float, default=None, help="Override simulated duration [s]")
    parser.add_argument("--graphics", action="store_true", help="Enable NASA 42 OpenGL GUI (requires display)")
    parser.add_argument(
        "--real-time",
        action="store_true",
        help="Use REAL time mode with graphics (default: FAST)",
    )
    parser.add_argument(
        "--viz-duration-s",
        type=float,
        default=7200.0,
        help="Default duration when --graphics is set without --duration-s (default: 2 h)",
    )
    args = parser.parse_args()

    if args.case != B2Case.B2A:
        raise SystemExit("Milestone 1 implements B2A only. B2B/B2C decks are not created yet.")

    config = B2Config()
    duration_s = args.duration_s
    if duration_s is None and args.graphics:
        duration_s = args.viz_duration_s
        print(f"Graphics mode: using {duration_s / 3600.0:.1f} h duration (pass --duration-s to override)")
    if duration_s is not None:
        config = B2Config(simulated_duration_s=float(duration_s))

    inout_dir = case_input_dir(args.case)
    if not inout_dir.is_dir():
        raise FileNotFoundError(f"NASA 42 input deck missing: {inout_dir}")

    binary = resolve_binary(args.binary)
    staged_dir, _ = stage_inout_dir(inout_dir, args.case)
    staged_name = staged_dir.name
    print(f"Running NASA 42 B2 {args.case.value} using {binary} (InOut={staged_name}) ...")
    runtime_s, completed = run_simulation(
        binary,
        inout_dir,
        staged_name,
        duration_s=config.simulated_duration_s,
        graphics=args.graphics,
        real_time=args.real_time,
    )

    if completed.returncode != 0:
        if completed.stdout:
            print(completed.stdout)
        if completed.stderr:
            print(completed.stderr)
        raise SystemExit(f"NASA 42 exited with status {completed.returncode}")

    summary = archive_outputs(args.case, inout_dir, runtime_s, config)
    write_run_log(
        DATA_DIR / "nasa42_b2_run_log.md",
        config,
        summary,
        completed.returncode,
        completed.stderr if isinstance(completed.stderr, str) else None,
    )

    print(f"NASA 42 B2 {args.case.value} complete.")
    print(f"  Runtime: {runtime_s:.3f} s")
    print(f"  Logs:    {DATA_DIR / args.case.value}")
    print(f"  Note:    milestone 1 archives native *.42 state files only (no metrics plots).")


if __name__ == "__main__":
    main()
