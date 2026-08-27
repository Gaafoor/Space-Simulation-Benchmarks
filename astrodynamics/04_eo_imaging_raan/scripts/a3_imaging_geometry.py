"""Shared geometry helpers for Benchmark A3 imaging opportunity analysis."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
MISSION_EPOCH_UTC = datetime(2026, 6, 2, 0, 0, 0, tzinfo=timezone.utc)


@dataclass(frozen=True)
class A3ImagingConfig:
    altitude_m: float = 500_000.0
    simulated_duration_s: float = 24.0 * 3600.0
    camera_fov_deg: float = 10.0
    madrid_lat_deg: float = 40.4168
    madrid_lon_deg: float = -3.7038
    madrid_alt_m: float = 667.0
    minimum_elevation_deg: float = 0.0

    @property
    def camera_half_angle_deg(self) -> float:
        return self.camera_fov_deg / 2.0

    @property
    def footprint_diameter_km(self) -> float:
        diameter_m = 2.0 * self.altitude_m * math.tan(math.radians(self.camera_half_angle_deg))
        return diameter_m / 1000.0


def geodetic_to_ecef_m(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    e2 = WGS84_F * (2.0 - WGS84_F)
    n = WGS84_A_M / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - e2) + alt_m) * math.sin(lat)
    return np.array([x, y, z], dtype=float)


def julian_date(dt: datetime) -> float:
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    dt = dt.astimezone(timezone.utc)
    year = dt.year
    month = dt.month
    day = dt.day
    hour = dt.hour + dt.minute / 60.0 + (dt.second + dt.microsecond / 1e6) / 3600.0
    if month <= 2:
        year -= 1
        month += 12
    a = math.floor(year / 100)
    b = 2 - a + math.floor(a / 4)
    jd0 = (
        math.floor(365.25 * (year + 4716))
        + math.floor(30.6001 * (month + 1))
        + day
        + b
        - 1524.5
    )
    return jd0 + hour / 24.0


def gmst_rad(dt: datetime) -> float:
    jd = julian_date(dt)
    t_ut1 = (jd - 2451545.0) / 36525.0
    gmst_deg = (
        280.46061837
        + 360.98564736629 * (jd - 2451545.0)
        + 0.000387933 * t_ut1**2
        - (t_ut1**3) / 38710000.0
    )
    return math.radians(gmst_deg % 360.0)


def ecef_to_eci_m(ecef_m: np.ndarray, elapsed_s: float) -> np.ndarray:
    dt = MISSION_EPOCH_UTC + timedelta(seconds=float(elapsed_s))
    theta = gmst_rad(dt)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    rotation = np.array(
        [
            [cos_t, -sin_t, 0.0],
            [sin_t, cos_t, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rotation @ ecef_m


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    a_norm = np.linalg.norm(a)
    b_norm = np.linalg.norm(b)
    if a_norm == 0.0 or b_norm == 0.0:
        return float("nan")
    cosine = float(np.dot(a, b) / (a_norm * b_norm))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def elevation_deg(target_eci_m: np.ndarray, spacecraft_eci_m: np.ndarray) -> float:
    los = spacecraft_eci_m - target_eci_m
    los_norm = np.linalg.norm(los)
    zenith = target_eci_m / np.linalg.norm(target_eci_m)
    if los_norm == 0.0:
        return float("nan")
    return math.degrees(math.asin(float(np.dot(los / los_norm, zenith))))


def compute_imaging_timeline(
    times_s: np.ndarray,
    positions_m: np.ndarray,
    config: A3ImagingConfig,
) -> list[dict[str, float | int]]:
    target_ecef = geodetic_to_ecef_m(
        config.madrid_lat_deg,
        config.madrid_lon_deg,
        config.madrid_alt_m,
    )
    rows: list[dict[str, float | int]] = []
    for time_s, spacecraft_eci in zip(times_s, positions_m):
        target_eci = ecef_to_eci_m(target_ecef, float(time_s))
        boresight = -spacecraft_eci
        target_vector = target_eci - spacecraft_eci
        boresight_angle = angle_deg(boresight, target_vector)
        target_elevation = elevation_deg(target_eci, spacecraft_eci)
        slant_range_km = float(np.linalg.norm(target_vector) / 1000.0)
        in_fov = (
            boresight_angle <= config.camera_half_angle_deg
            and target_elevation >= config.minimum_elevation_deg
        )
        rows.append(
            {
                "time_s": float(time_s),
                "boresight_angle_deg": boresight_angle,
                "target_elevation_deg": target_elevation,
                "slant_range_km": slant_range_km,
                "target_in_camera_fov": int(in_fov),
            }
        )
    return rows


def extract_windows(rows: list[dict[str, float | int]]) -> list[dict[str, float]]:
    if not rows:
        return []
    windows: list[dict[str, float]] = []
    previous = int(rows[0]["target_in_camera_fov"])
    start_time = float(rows[0]["time_s"]) if previous else None
    for row in rows[1:]:
        current = int(row["target_in_camera_fov"])
        time_s = float(row["time_s"])
        if current != previous:
            if current == 1:
                start_time = time_s
            elif start_time is not None:
                windows.append(
                    {
                        "start_s": start_time,
                        "stop_s": time_s,
                        "duration_s": time_s - start_time,
                    }
                )
                start_time = None
            previous = current
    if previous == 1 and start_time is not None:
        stop_time = float(rows[-1]["time_s"])
        windows.append(
            {
                "start_s": start_time,
                "stop_s": stop_time,
                "duration_s": stop_time - start_time,
            }
        )
    return windows


def compute_metrics(
    rows: list[dict[str, float | int]],
    windows: list[dict[str, float]],
    config: A3ImagingConfig,
) -> dict[str, float | int | str]:
    durations_s = [window["duration_s"] for window in windows]
    revisit_s = [
        windows[index]["start_s"] - windows[index - 1]["start_s"]
        for index in range(1, len(windows))
    ]
    min_angle = min((float(row["boresight_angle_deg"]) for row in rows), default=float("nan"))
    max_elevation = max((float(row["target_elevation_deg"]) for row in rows), default=float("nan"))
    return {
        "imaging_opportunity_count": len(windows),
        "avg_imaging_opportunity_duration_min": float(np.mean(durations_s) / 60.0) if durations_s else 0.0,
        "total_imaging_time_min": float(sum(durations_s) / 60.0),
        "min_revisit_time_min": float(min(revisit_s) / 60.0) if revisit_s else 0.0,
        "camera_fov_deg": config.camera_fov_deg,
        "camera_half_angle_deg": config.camera_half_angle_deg,
        "ground_footprint_diameter_km": config.footprint_diameter_km,
        "minimum_boresight_angle_deg": min_angle,
        "maximum_target_elevation_deg": max_elevation,
        "rendered_image_output_supported": "No - A3 geometry only; C1 deferred",
    }


def write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
