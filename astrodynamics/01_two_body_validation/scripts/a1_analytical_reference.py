"""Shared analytical two-body reference for Benchmark A1 (no simulator deps)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

MU_EARTH = 3.986004418e14  # m^3/s^2, WGS84
R_EARTH = 6378137.0  # m, WGS84 equatorial radius


@dataclass(frozen=True)
class A1Config:
    altitude_m: float = 500_000.0
    inclination_deg: float = 97.4
    eccentricity: float = 0.0
    raan_deg: float = 0.0
    arg_perigee_deg: float = 0.0
    true_anomaly_deg: float = 0.0
    simulated_duration_s: float = 24.0 * 3600.0
    dynamics_step_s: float = 10.0
    output_sample_s: float = 600.0

    @property
    def semi_major_axis_m(self) -> float:
        return R_EARTH + self.altitude_m


def perifocal_to_inertial_matrix(config: A1Config) -> np.ndarray:
    raan = math.radians(config.raan_deg)
    inc = math.radians(config.inclination_deg)
    argp = math.radians(config.arg_perigee_deg)

    cos_raan, sin_raan = math.cos(raan), math.sin(raan)
    cos_inc, sin_inc = math.cos(inc), math.sin(inc)
    cos_argp, sin_argp = math.cos(argp), math.sin(argp)

    r3_raan = np.array(
        [[cos_raan, -sin_raan, 0.0], [sin_raan, cos_raan, 0.0], [0.0, 0.0, 1.0]]
    )
    r1_inc = np.array(
        [[1.0, 0.0, 0.0], [0.0, cos_inc, -sin_inc], [0.0, sin_inc, cos_inc]]
    )
    r3_argp = np.array(
        [[cos_argp, -sin_argp, 0.0], [sin_argp, cos_argp, 0.0], [0.0, 0.0, 1.0]]
    )
    return r3_raan @ r1_inc @ r3_argp


def solve_kepler(mean_anomaly: float, eccentricity: float) -> float:
    eccentric_anomaly = mean_anomaly
    for _ in range(50):
        update = (eccentric_anomaly - eccentricity * math.sin(eccentric_anomaly) - mean_anomaly) / (
            1.0 - eccentricity * math.cos(eccentric_anomaly)
        )
        eccentric_anomaly -= update
        if abs(update) < 1e-13:
            break
    return eccentric_anomaly


def analytical_two_body_state(config: A1Config, t_s: float) -> tuple[np.ndarray, np.ndarray]:
    a = config.semi_major_axis_m
    e = config.eccentricity
    n = math.sqrt(MU_EARTH / a**3)

    if e == 0.0:
        true_anomaly = math.radians(config.true_anomaly_deg) + n * t_s
        radius = a
        p = a
    else:
        f0 = math.radians(config.true_anomaly_deg)
        e0 = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(f0 / 2.0), math.sqrt(1.0 + e) * math.cos(f0 / 2.0))
        m0 = e0 - e * math.sin(e0)
        mean_anomaly = m0 + n * t_s
        eccentric_anomaly = solve_kepler(mean_anomaly, e)
        true_anomaly = 2.0 * math.atan2(
            math.sqrt(1.0 + e) * math.sin(eccentric_anomaly / 2.0),
            math.sqrt(1.0 - e) * math.cos(eccentric_anomaly / 2.0),
        )
        p = a * (1.0 - e**2)
        radius = p / (1.0 + e * math.cos(true_anomaly))

    r_pqw = np.array([radius * math.cos(true_anomaly), radius * math.sin(true_anomaly), 0.0])
    v_pqw = math.sqrt(MU_EARTH / p) * np.array(
        [-math.sin(true_anomaly), e + math.cos(true_anomaly), 0.0]
    )
    q_pqw_to_eci = perifocal_to_inertial_matrix(config)
    return q_pqw_to_eci @ r_pqw, q_pqw_to_eci @ v_pqw
