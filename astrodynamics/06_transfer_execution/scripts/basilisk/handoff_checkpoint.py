"""Incremental checkpoint output for long Basilisk handoff runs."""

from __future__ import annotations

import csv
import json
import math
import time
from pathlib import Path

import numpy as np
from Basilisk.utilities import orbitalMotion

from ephemeris_handoff import SpiceReferenceEphemeris, burn_schedule_timing_error, position_error_envelope


def _last_vec3(arr: np.ndarray) -> np.ndarray:
    if arr.size == 0:
        return np.full(3, np.nan)
    return np.asarray(arr, dtype=float).reshape(-1, 3)[-1]


def resolve_checkpoint_interval_s(
    *,
    vizard_enabled: bool,
    checkpoint_interval_s: float | None,
) -> float | None:
    if checkpoint_interval_s is not None:
        return float(checkpoint_interval_s) if checkpoint_interval_s > 0 else None
    if vizard_enabled:
        return 3600.0
    return None


class HandoffCheckpointWriter:
    """Flush partial telemetry and validation metrics during propagation."""

    def __init__(
        self,
        out_dir: Path,
        schedule: dict,
        *,
        mu_earth: float,
        interval_s: float,
        initial_mass_kg: float,
    ) -> None:
        self.out_dir = Path(out_dir)
        self.schedule = schedule
        self.mu_earth = float(mu_earth)
        self.interval_s = float(interval_s)
        self.initial_mass_kg = float(initial_mass_kg)
        self._last_checkpoint_t = -float("inf")
        self.out_dir.mkdir(parents=True, exist_ok=True)

    def maybe_write(
        self,
        cur_t: float,
        *,
        state_rec,
        thrust_samples: list[tuple[float, float, float, float, float]],
        pointing_samples: list[tuple[float, float]],
        applied_burns: list[dict],
        current_mass_kg: float,
        vizard_bin: str | None = None,
    ) -> bool:
        if cur_t - self._last_checkpoint_t + 1e-9 < self.interval_s:
            return False
        self._last_checkpoint_t = float(cur_t)
        self.write(
            cur_t,
            state_rec=state_rec,
            thrust_samples=thrust_samples,
            pointing_samples=pointing_samples,
            applied_burns=applied_burns,
            current_mass_kg=current_mass_kg,
            vizard_bin=vizard_bin,
        )
        return True

    def write(
        self,
        cur_t: float,
        *,
        state_rec,
        thrust_samples: list[tuple[float, float, float, float, float]],
        pointing_samples: list[tuple[float, float]],
        applied_burns: list[dict],
        current_mass_kg: float,
        vizard_bin: str | None = None,
        status: str = "in_progress",
        include_summary: bool = True,
    ) -> dict:
        times = np.asarray(state_rec.times() * 1e-9, dtype=float)
        pos = np.asarray(state_rec.r_BN_N, dtype=float)
        vel = np.asarray(state_rec.v_BN_N, dtype=float)

        self._write_state_history(times, pos, vel)
        self._write_orbit_elements(times, pos, vel)
        self._write_thrust_history(thrust_samples, applied_burns)
        self._write_pointing_history(pointing_samples)

        r_fin = _last_vec3(pos)
        v_fin = _last_vec3(vel)
        if pos.size and np.all(np.isfinite(r_fin)) and np.all(np.isfinite(v_fin)):
            oe = orbitalMotion.rv2elem(self.mu_earth, r_fin, v_fin)
            final_sma_km = float(oe.a / 1000.0)
            final_ecc = float(oe.e)
            final_inc_deg = float(np.degrees(oe.i))
        else:
            final_sma_km = final_ecc = final_inc_deg = float("nan")

        target = self.schedule.get("target", {})
        max_ephem = float("nan")
        ref_meta = self.schedule.get("reference_ephemeris", {})
        if ref_meta.get("path") and times.size:
            ref = SpiceReferenceEphemeris.from_metadata(ref_meta)
            max_ephem, _ = position_error_envelope(
                times,
                pos,
                spice_ref=ref,
                min_time_s=float(ref_meta.get("bsp_coverage_start_s", 0.0)),
            )

        max_timing, _ = burn_schedule_timing_error(
            applied_burns,
            tolerance_s=float(target.get("max_burn_start_error_s", 120.0)),
        )
        fuel_used_kg = self.initial_mass_kg - float(current_mass_kg)
        mission_end_s = float(self.schedule.get("dynamics", {}).get("mission_end_s", cur_t))
        progress_fraction = min(1.0, float(cur_t) / mission_end_s) if mission_end_s > 0 else float("nan")

        max_pointing = float("nan")
        if pointing_samples:
            max_pointing = max(abs(err) for _, err in pointing_samples if math.isfinite(err))

        progress = {
            "status": status,
            "checkpoint_time_s": float(cur_t),
            "checkpoint_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "progress_fraction": progress_fraction,
            "mission_end_s": mission_end_s,
            "final_sma_km": final_sma_km,
            "final_ecc": final_ecc,
            "final_inc_deg": final_inc_deg,
            "fuel_used_kg": fuel_used_kg,
            "max_ephemeris_position_error_m": max_ephem,
            "max_burn_start_error_s": max_timing,
            "max_pointing_error_deg": max_pointing,
            "state_samples": int(times.size),
            "thrust_samples": len(thrust_samples),
            "burns_applied": applied_burns,
            "vizard_bin": vizard_bin,
        }
        progress_path = self.out_dir / "run_progress.json"
        progress_path.write_text(json.dumps(progress, indent=2), encoding="utf-8")

        if include_summary:
            partial_summary = {
                "task_id": "gmat_basilisk_gnc_handoff",
                "run_status": status,
                "checkpoint_time_s": float(cur_t),
                "progress_fraction": progress_fraction,
                "final_sma_km": final_sma_km,
                "final_ecc": final_ecc,
                "final_inc_deg": final_inc_deg,
                "fuel_used_kg": fuel_used_kg,
                "max_ephemeris_position_error_m": max_ephem,
                "max_burn_start_error_s": max_timing,
                "burns_applied": applied_burns,
                "gmat_reference": self.schedule.get("gmat_reference", {}),
                "pointing_error_history": str(self.out_dir / "pointing_error_history.csv"),
            }
            (self.out_dir / "summary.json").write_text(json.dumps(partial_summary, indent=2), encoding="utf-8")
        return progress

    def _write_state_history(self, times: np.ndarray, pos: np.ndarray, vel: np.ndarray) -> None:
        with (self.out_dir / "state_history.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "rx_m", "ry_m", "rz_m", "vx_m_s", "vy_m_s", "vz_m_s"])
            for t, r, v in zip(times, pos, vel):
                w.writerow([f"{t:.3f}", *[f"{x:.6f}" for x in r], *[f"{x:.6f}" for x in v]])

    def _write_orbit_elements(self, times: np.ndarray, pos: np.ndarray, vel: np.ndarray) -> None:
        with (self.out_dir / "orbit_elements_history.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "sma_km", "ecc", "inc_deg"])
            for t, r, v in zip(times, pos, vel):
                if np.all(np.isfinite(r)) and np.all(np.isfinite(v)):
                    oe = orbitalMotion.rv2elem(self.mu_earth, r, v)
                    w.writerow(
                        [
                            f"{t:.3f}",
                            f"{float(oe.a)/1000.0:.6f}",
                            f"{float(oe.e):.8f}",
                            f"{math.degrees(float(oe.i)):.6f}",
                        ]
                    )
                else:
                    w.writerow([f"{t:.3f}", "nan", "nan", "nan"])

    def _write_thrust_history(
        self,
        thrust_samples: list[tuple[float, float, float, float, float]],
        applied_burns: list[dict],
    ) -> None:
        with (self.out_dir / "thrust_history.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "thrust_n", "mass_kg", "power_kw", "eclipse_factor"])
            if thrust_samples:
                for t_s, t_n, m_kg, p_kw, ecl in thrust_samples:
                    w.writerow([f"{t_s:.3f}", f"{t_n:.6f}", f"{m_kg:.6f}", f"{p_kw:.6f}", f"{ecl:.6f}"])
            else:
                mass = float(self.schedule["initial_state"]["mass_kg"])
                for b in applied_burns:
                    w.writerow([f"{b['time_s']:.3f}", f"{b.get('thrust_n', 0.0):.6f}", f"{mass:.6f}", "0.0", "1.0"])
                    mass = float(b.get("mass_after_kg", mass))

    def _write_pointing_history(self, pointing_samples: list[tuple[float, float]]) -> None:
        with (self.out_dir / "pointing_error_history.csv").open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "pointing_error_deg"])
            for t, err in pointing_samples:
                w.writerow([f"{t:.3f}", f"{err:.6f}" if math.isfinite(err) else "nan"])
