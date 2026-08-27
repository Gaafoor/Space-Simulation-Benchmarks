"""Shared drift metrics for Benchmark B1 state histories."""

from __future__ import annotations

import math
from datetime import datetime, timedelta, timezone
from typing import Any

import numpy as np

from b1_physics_config import R_EARTH

MISSION_EPOCH_UTC = datetime(2026, 6, 2, 0, 0, 0, tzinfo=timezone.utc)


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


def eci_position_to_subpoint_lat_lon(position_eci_m: np.ndarray, time_s: float) -> tuple[float, float]:
    dt = MISSION_EPOCH_UTC + timedelta(seconds=float(time_s))
    theta = gmst_rad(dt)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    rotation = np.array(
        [
            [cos_t, sin_t, 0.0],
            [-sin_t, cos_t, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    position_ecef_m = rotation @ position_eci_m
    radius = np.linalg.norm(position_ecef_m)
    lat = math.degrees(math.asin(position_ecef_m[2] / radius))
    lon = math.degrees(math.atan2(position_ecef_m[1], position_ecef_m[0]))
    return lat, lon


def normalize_lon_delta_deg(lon_deg: float) -> float:
    while lon_deg > 180.0:
        lon_deg -= 360.0
    while lon_deg < -180.0:
        lon_deg += 360.0
    return lon_deg


def altitude_from_semi_major_axis_m(semi_major_axis_m: float) -> float:
    return semi_major_axis_m - R_EARTH


def compute_endpoint_drift_metrics(
    case_history: dict[str, Any],
    case1_history: dict[str, Any],
) -> dict[str, float]:
    """Compute B1 drift metrics at the final epoch vs same-tool Case 1."""
    pos_case = np.asarray(case_history["positions_m"][-1], dtype=float)
    pos_case1 = np.asarray(case1_history["positions_m"][-1], dtype=float)
    t_end = float(case_history["times_s"][-1])

    _, lon_case = eci_position_to_subpoint_lat_lon(pos_case, t_end)
    _, lon_case1 = eci_position_to_subpoint_lat_lon(pos_case1, t_end)

    raan0 = float(case_history["raan_deg"][0])
    raan_end = float(case_history["raan_deg"][-1])
    alt0 = altitude_from_semi_major_axis_m(float(case_history["semi_major_axis_m"][0]))
    alt_end = altitude_from_semi_major_axis_m(float(case_history["semi_major_axis_m"][-1]))

    arg_lat0 = float(case_history["argument_of_latitude_deg"][0])
    arg_lat_end = float(case_history["argument_of_latitude_deg"][-1])
    arg_lat_case1_end = float(case1_history["argument_of_latitude_deg"][-1])

    return {
        "position_drift_vs_case1_m": float(np.linalg.norm(pos_case - pos_case1)),
        "ground_track_lon_drift_deg": normalize_lon_delta_deg(lon_case - lon_case1),
        "raan_change_deg": raan_end - raan0,
        "altitude_change_m": alt_end - alt0,
        "along_track_phase_drift_deg": normalize_lon_delta_deg(arg_lat_end - arg_lat_case1_end),
        "final_altitude_m": alt_end,
        "final_semi_major_axis_m": float(case_history["semi_major_axis_m"][-1]),
    }


def histories_from_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "times_s": [float(row["time_s"]) for row in rows],
        "positions_m": [
            np.array([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])], dtype=float)
            for row in rows
        ],
        "raan_deg": [float(row["raan_deg"]) for row in rows],
        "semi_major_axis_m": [float(row["semi_major_axis_m"]) for row in rows],
        "argument_of_latitude_deg": [float(row["argument_of_latitude_deg"]) for row in rows],
    }
