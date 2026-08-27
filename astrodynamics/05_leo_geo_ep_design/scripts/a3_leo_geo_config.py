"""Local fallback config for A3 LEO->GEO handoff."""

from __future__ import annotations

from dataclasses import dataclass

R_EARTH = 6378137.0
MU_EARTH = 3.986004418e14


@dataclass(frozen=True)
class A3LeoGeoConfig:
    dry_mass_kg: float = 12.0
    mission_epoch_utc_iso: str = "2026-06-15T12:00:00.000Z"
    spice_epoch_utc: str = "2026 JUN 15 12:00:00.000 (UTC)"
    geo_semi_major_axis_m: float = 42_164_000.0
    target_ecc_max: float = 0.001
    target_inc_deg: float = 0.0
    sma_tolerance_m: float = 1000.0
    inc_tolerance_deg: float = 0.1
    dynamics_step_s: float = 10.0
    output_sample_s: float = 60.0
    j2_max_degree: int = 2
    gravity_max_degree: int = 20
    drag_coefficient: float = 2.2
    drag_area_m2: float = 0.08
    srp_reflectivity: float = 1.8
    srp_area_m2: float = 0.10
    geometry_level: str = "L1"
    # GMAT A3 drag weather (NRLMSISE-00 inputs in Benchmark_A3_LEO_GEO_Transfer.script)
    msis_f107: float = 150.0
    msis_f107a: float = 150.0
    msis_ap: float = 3.0
