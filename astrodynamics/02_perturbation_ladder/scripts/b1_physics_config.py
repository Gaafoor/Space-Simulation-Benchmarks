"""Shared frozen parameters for Benchmark B1 Physics Fidelity."""

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import Enum

R_EARTH = 6378137.0  # m, WGS84 equatorial radius (matches A1 reference)


class B1Case(Enum):
    CASE1_TWOBODY = "b1_case1_twobody"
    CASE2_J2 = "b1_case2_j2"
    CASE3_J2_DRAG = "b1_case3_j2_drag"
    CASE4_J2_DRAG_SRP = "b1_case4_j2_drag_srp"
    CASE5_ALL_SUPPORTED = "b1_case5_all_supported"


@dataclass(frozen=True)
class B1Config:
    """Frozen B1 orbit, spacecraft, and timing parameters."""

    altitude_m: float = 500_000.0
    inclination_deg: float = 97.4
    eccentricity: float = 0.0
    raan_deg: float = 0.0
    arg_perigee_deg: float = 0.0
    true_anomaly_deg: float = 0.0
    simulated_duration_s: float = 360.0 * 24.0 * 3600.0
    dynamics_step_s: float = 10.0
    output_sample_s: float = 600.0
    spice_epoch_utc: str = "2026 JUN 02 00:00:00.000 (UTC)"
    mission_epoch_utc_iso: str = "2026-06-02T00:00:00.000Z"

    dry_mass_kg: float = 12.0
    drag_coefficient: float = 2.2
    srp_reflectivity: float = 1.8
    drag_area_m2: float = 0.01
    srp_area_m2: float = 1.0

    basilisk_j2_gravity_file: str = "GGM03S-J2-only.txt"
    basilisk_j2_max_degree: int = 2

    @property
    def semi_major_axis_m(self) -> float:
        return R_EARTH + self.altitude_m

    @property
    def output_sample_count(self) -> int:
        return int(self.simulated_duration_s / self.output_sample_s) + 1


def load_b1_config_from_env() -> B1Config:
    """Load B1 config with optional environment overrides."""
    duration_s = os.environ.get("B1_SIM_DURATION_S")
    output_sample_s = os.environ.get("B1_OUTPUT_SAMPLE_S")
    dynamics_step_s = os.environ.get("B1_DYNAMICS_STEP_S")

    kwargs = {}
    if duration_s is not None:
        kwargs["simulated_duration_s"] = float(duration_s)
    if output_sample_s is not None:
        kwargs["output_sample_s"] = float(output_sample_s)
    if dynamics_step_s is not None:
        kwargs["dynamics_step_s"] = float(dynamics_step_s)

    return B1Config(**kwargs)


def case_uses_j2(case: B1Case) -> bool:
    return case != B1Case.CASE1_TWOBODY


def case_uses_drag(case: B1Case) -> bool:
    return case in {
        B1Case.CASE3_J2_DRAG,
        B1Case.CASE4_J2_DRAG_SRP,
        B1Case.CASE5_ALL_SUPPORTED,
    }


def case_uses_srp(case: B1Case) -> bool:
    return case in {B1Case.CASE4_J2_DRAG_SRP, B1Case.CASE5_ALL_SUPPORTED}


def case_uses_third_body(case: B1Case) -> bool:
    return case == B1Case.CASE5_ALL_SUPPORTED
