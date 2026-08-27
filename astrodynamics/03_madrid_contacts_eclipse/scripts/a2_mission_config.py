"""Shared frozen mission parameters for Benchmark A2."""

from __future__ import annotations

from dataclasses import dataclass

from a1_analytical_reference import R_EARTH


@dataclass(frozen=True)
class A2Config:
    altitude_m: float = 500_000.0
    inclination_deg: float = 97.4
    eccentricity: float = 0.0
    raan_deg: float = 0.0
    arg_perigee_deg: float = 0.0
    true_anomaly_deg: float = 0.0
    simulated_duration_s: float = 24.0 * 3600.0
    dynamics_step_s: float = 10.0
    output_sample_s: float = 600.0
    spice_epoch_utc: str = "2026 JUN 02 00:00:00.000 (UTC)"
    madrid_lat_deg: float = 40.4168
    madrid_lon_deg: float = -3.7038
    madrid_alt_m: float = 667.0
    minimum_elevation_deg: float = 10.0
    j2_gravity_file: str = "GGM03S-J2-only.txt"
    j2_max_degree: int = 2
    eclipse_shadow_threshold: float = 0.999

    @property
    def semi_major_axis_m(self) -> float:
        return R_EARTH + self.altitude_m
