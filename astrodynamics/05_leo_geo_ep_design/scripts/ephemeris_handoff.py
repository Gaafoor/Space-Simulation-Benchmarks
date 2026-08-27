"""Reference ephemeris utilities."""

from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

_FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


def parse_epoch_utc(epoch_utc: str) -> datetime:
    token = epoch_utc.strip()
    if token.endswith("Z"):
        token = token[:-1] + "+00:00"
    return datetime.fromisoformat(token).astimezone(timezone.utc)


def utc_datetime_to_spice_string(dt: datetime) -> str:
    dt = dt.astimezone(timezone.utc)
    return f"{dt.year} {dt.strftime('%b').upper()} {dt.day} {dt.hour:02d}:{dt.minute:02d}:{dt.second:02d}.{dt.microsecond//1000:03d} (UTC)"


@dataclass(frozen=True)
class SpiceReferenceEphemeris:
    bsp_path: Path
    naif_id: str
    mission_epoch: datetime
    spice_ref: str = "J2000"
    spice_observer: str = "EARTH"

    @classmethod
    def from_metadata(cls, meta: dict) -> "SpiceReferenceEphemeris":
        return cls(Path(meta["path"]), str(meta["naif_id"]), parse_epoch_utc(meta["mission_epoch_utc_iso"]), str(meta.get("spice_ref", "J2000")), str(meta.get("spice_observer", "EARTH")))


class SpiceKernelSession:
    def __init__(self, bsp_path: Path):
        self.bsp_path = Path(bsp_path)
        self._lsk_path: Path | None = None

    def __enter__(self):
        from Basilisk.topLevelModules import pyswice
        from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

        self._lsk_path = Path(get_path(DataFile.EphemerisData.naif0012))
        pyswice.furnsh_c(str(self._lsk_path))
        pyswice.furnsh_c(str(self.bsp_path.resolve()))
        return self

    def __exit__(self, exc_type, exc, tb):
        from Basilisk.topLevelModules import pyswice

        pyswice.unload_c(str(self.bsp_path.resolve()))
        if self._lsk_path is not None:
            pyswice.unload_c(str(self._lsk_path))


def read_spice_state_at(ref: SpiceReferenceEphemeris, time_s: float) -> tuple[np.ndarray, np.ndarray]:
    from Basilisk.utilities.pyswice_spk_utilities import spkRead

    t = ref.mission_epoch + timedelta(seconds=float(time_s))
    state_km = spkRead(ref.naif_id, utc_datetime_to_spice_string(t), ref.spice_ref, ref.spice_observer)
    state_m = 1000.0 * np.asarray(state_km, dtype=float)
    return state_m[:3], state_m[3:]


def parse_gmat_state_rows(path: Path) -> list[dict[str, float]]:
    rows = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not raw.strip() or "ElapsedSecs" in raw:
            continue
        nums = [float(tok) for tok in _FLOAT_RE.findall(raw)]
        if len(nums) < 7:
            continue
        t, x, y, z, vx, vy, vz = nums[:7]
        rows.append({"time_s": t, "r_m": [x * 1000.0, y * 1000.0, z * 1000.0], "v_m_s": [vx * 1000.0, vy * 1000.0, vz * 1000.0]})
    if len(rows) < 1:
        raise ValueError(f"No rows in {path}")
    return rows


def resolve_initial_state(gmat_dir: Path, spice_ref: SpiceReferenceEphemeris | None) -> tuple[np.ndarray, np.ndarray, float | None]:
    state_path = gmat_dir / "gmat_state.csv"
    if state_path.is_file():
        rows = parse_gmat_state_rows(state_path)
        return np.asarray(rows[0]["r_m"], dtype=float), np.asarray(rows[0]["v_m_s"], dtype=float), rows[0]["time_s"]
    if spice_ref is None:
        raise FileNotFoundError(f"Missing initial state in {gmat_dir}")
    with SpiceKernelSession(spice_ref.bsp_path):
        r0, v0 = read_spice_state_at(spice_ref, 0.0)
    return r0, v0, 0.0


def load_reference_ephemeris(path: Path, *, mission_epoch: datetime | None = None) -> list[dict[str, float]]:
    if not path.is_file():
        return []
    if path.suffix.lower() == ".csv":
        rows = []
        with path.open(encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                rows.append({"time_s": float(row["time_s"]), "r_m": [float(row["rx_m"]), float(row["ry_m"]), float(row["rz_m"])], "v_m_s": [float(row["vx_m_s"]), float(row["vy_m_s"]), float(row["vz_m_s"])]})
        return rows
    return parse_gmat_state_rows(path)


def position_error_envelope(
    times_s: np.ndarray,
    positions_m: np.ndarray,
    reference_rows: list[dict[str, float]] | None = None,
    *,
    spice_ref: SpiceReferenceEphemeris | None = None,
    max_time_match_s: float = 30.0,
    min_time_s: float = 0.0,
) -> tuple[float, np.ndarray]:
    if times_s.size == 0:
        return float("nan"), np.array([], dtype=float)
    if spice_ref is not None:
        errs = np.full(times_s.size, np.nan)
        with SpiceKernelSession(spice_ref.bsp_path):
            for i, t in enumerate(times_s):
                if float(t) < float(min_time_s):
                    continue
                try:
                    r_ref, _ = read_spice_state_at(spice_ref, float(t))
                except Exception:
                    continue
                if np.all(np.isfinite(positions_m[i])):
                    errs[i] = float(np.linalg.norm(positions_m[i] - r_ref))
        finite = np.isfinite(errs)
        return (float(np.nanmax(errs[finite])) if np.any(finite) else float("nan")), errs
    if not reference_rows:
        return float("nan"), np.array([])
    ref_t = np.asarray([row["time_s"] for row in reference_rows], dtype=float)
    errs = np.full(ref_t.size, np.nan)
    for i, t in enumerate(ref_t):
        j = int(np.argmin(np.abs(times_s - t)))
        if abs(float(times_s[j]) - t) > max_time_match_s:
            continue
        errs[i] = float(np.linalg.norm(np.asarray(reference_rows[i]["r_m"], dtype=float) - positions_m[j]))
    finite = np.isfinite(errs)
    return (float(np.nanmax(errs[finite])) if np.any(finite) else float("nan")), errs


def burn_schedule_timing_error(applied: list[dict], *, tolerance_s: float) -> tuple[float, bool]:
    errs = []
    for burn in applied:
        if burn.get("gmat_start_s") is None:
            continue
        errs.append(abs(float(burn["time_s"]) - float(burn["gmat_start_s"])))
    if not errs:
        return 0.0, True
    m = max(errs)
    return m, m <= tolerance_s
