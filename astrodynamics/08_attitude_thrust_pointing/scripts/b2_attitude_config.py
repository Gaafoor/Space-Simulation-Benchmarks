"""Shared frozen parameters for Benchmark B2 Attitude & GNC."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from b1_physics_config import R_EARTH

SOLAR_CONSTANT_W_M2 = 1361.0


class B2Case(Enum):
    B2A = "b2a"
    B2A2 = "b2a2"
    B2B = "b2b"
    B2C = "b2c"


@dataclass(frozen=True)
class B2Config:
    """Frozen B2 orbit, spacecraft, power, and timing parameters."""

    altitude_m: float = 500_000.0
    inclination_deg: float = 97.4
    eccentricity: float = 0.0
    raan_deg: float = 0.0
    arg_perigee_deg: float = 0.0
    true_anomaly_deg: float = 0.0
    simulated_duration_s: float = 24.0 * 3600.0
    dynamics_step_s: float = 10.0
    output_sample_s: float = 10.0
    basilisk_integration_step_s: float = 1.0
    spice_epoch_utc: str = "2026 JUN 02 00:00:00.000 (UTC)"
    mission_epoch_utc_iso: str = "2026-06-02T00:00:00.000Z"

    mass_kg: float = 12.0
    inertia_kg_m2: tuple[float, float, float] = (0.15, 0.18, 0.12)
    drag_coefficient: float = 2.2
    srp_reflectivity: float = 1.8
    drag_area_m2: float = 0.01
    srp_area_m2: float = 1.0

    battery_capacity_wh: float = 40.0
    initial_soc_percent: float = 40.0
    target_soc_percent: float = 80.0
    solar_panel_area_m2: float = 0.05
    solar_efficiency: float = 0.30
    bus_load_w: float = 5.0
    sun_pointing_axis_body: tuple[float, float, float] = (0.0, 0.0, 1.0)
    # Secondary axis for locationPointing roll constraint (cHat_B).
    sun_secondary_axis_body: tuple[float, float, float] = (0.0, 1.0, 0.0)
    use_boresight_rate_damping: bool = True

    wheel_count: int = 4
    wheel_torque_limit_nm: float = 0.005
    wheel_momentum_limit_nms: float = 0.02
    wheel_stress_torque_limit_nm: float = 0.003
    wheel_stress_momentum_limit_nms: float = 0.012

    misalignment_deg_b2a_b2b: float = 30.0
    misalignment_deg_b2c: float = 120.0
    initial_rates_deg_s_b2a_b2b: tuple[float, float, float] = (0.5, 0.3, -0.4)
    initial_rates_deg_s_b2c: tuple[float, float, float] = (2.0, 1.5, -1.0)
    misalignment_axis_body: tuple[float, float, float] = (0.0, 1.0, 0.0)

    sun_sensor_noise_sigma_deg: float = 0.15
    sun_sensor_bias_deg: float = 0.5

    # B2A2 navigation noise (SimpleNav Gauss-Markov on attitude / rate states).
    nav_attitude_noise_sigma_deg: float = 0.1
    nav_attitude_error_3sigma_deg: float = 0.3
    nav_rate_noise_sigma_deg_s: float = 0.02
    nav_rate_error_3sigma_deg_s: float = 0.06

    mc_runs_preferred: int = 100
    mc_runs_minimum: int = 50
    mc_seed: int = 42

    basilisk_j2_gravity_file: str = "GGM03S-J2-only.txt"
    basilisk_j2_max_degree: int = 2
    eclipse_shadow_threshold: float = 0.999

    # Basilisk-only MRP feedback gains (tuned on B2A, frozen per tool policy).
    # Reference: scenarioAttitudeFeedbackRW uses K=3.5, P=30 at I≈900 kg·m².
    mrp_gain_reference_inertia_kg_m2: float = 900.0
    mrp_gain_reference_k: float = 3.5
    mrp_gain_reference_p: float = 30.0
    mrp_ki: float = -1.0

    @property
    def mrp_characteristic_inertia_kg_m2(self) -> float:
        return self.inertia_kg_m2[1]

    @property
    def mrp_k(self) -> float:
        scale = self.mrp_characteristic_inertia_kg_m2 / self.mrp_gain_reference_inertia_kg_m2
        return self.mrp_gain_reference_k * scale

    @property
    def mrp_p(self) -> float:
        scale = self.mrp_characteristic_inertia_kg_m2 / self.mrp_gain_reference_inertia_kg_m2
        return self.mrp_gain_reference_p * scale

    @property
    def semi_major_axis_m(self) -> float:
        return R_EARTH + self.altitude_m

    @property
    def battery_capacity_j(self) -> float:
        return self.battery_capacity_wh * 3600.0

    @property
    def initial_stored_charge_j(self) -> float:
        return self.battery_capacity_j * self.initial_soc_percent / 100.0

    @property
    def output_sample_count(self) -> int:
        return int(self.simulated_duration_s / self.output_sample_s) + 1


def case_uses_drag(case: B2Case) -> bool:
    return case in {B2Case.B2A2, B2Case.B2B, B2Case.B2C}


def case_uses_srp(case: B2Case) -> bool:
    return case in {B2Case.B2A2, B2Case.B2B, B2Case.B2C}


def case_uses_gravity_gradient(case: B2Case) -> bool:
    return case in {B2Case.B2A2, B2Case.B2B, B2Case.B2C}


def case_uses_sensor_errors(case: B2Case) -> bool:
    return case in {B2Case.B2A2, B2Case.B2B, B2Case.B2C}


def case_uses_nav_errors(case: B2Case) -> bool:
    return case == B2Case.B2A2


def case_uses_ephemeris_sun_guidance(case: B2Case) -> bool:
    """B2A uses SPICE ephemeris sun vector (not sensor-limited)."""
    return case == B2Case.B2A


def case_uses_css_sun_guidance(case: B2Case) -> bool:
    """B2A2 uses CSS constellation + WLS estimator (eclipse-blind)."""
    return case == B2Case.B2A2


def case_uses_stress_wheel_limits(case: B2Case) -> bool:
    return case == B2Case.B2C


def case_uses_stuck_wheel(case: B2Case) -> bool:
    return case == B2Case.B2C


def initial_misalignment_deg(case: B2Case, config: B2Config) -> float:
    if case == B2Case.B2C:
        return config.misalignment_deg_b2c
    return config.misalignment_deg_b2a_b2b


def initial_rates_deg_s(case: B2Case, config: B2Config) -> tuple[float, float, float]:
    if case == B2Case.B2C:
        return config.initial_rates_deg_s_b2c
    return config.initial_rates_deg_s_b2a_b2b


def wheel_torque_limit_nm(case: B2Case, config: B2Config) -> float:
    if case_uses_stress_wheel_limits(case):
        return config.wheel_stress_torque_limit_nm
    return config.wheel_torque_limit_nm


def wheel_momentum_limit_nms(case: B2Case, config: B2Config) -> float:
    if case_uses_stress_wheel_limits(case):
        return config.wheel_stress_momentum_limit_nms
    return config.wheel_momentum_limit_nms
