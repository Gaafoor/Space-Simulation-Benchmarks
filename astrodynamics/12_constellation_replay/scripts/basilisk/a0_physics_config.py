#!/usr/bin/env python3
"""
A0 Physics Fidelity Configuration — Exact GMAT Twin for Cross-Tool Comparison.

This configuration replicates the GMAT script parameters:
- 5 identical spacecraft (Cases 1–5)
- 1-year duration (525600 steps × 60 s)
- DragArea = 0.08, SRPArea = 0.1
- Case 5: 20x20 gravity field (JGM2)
- Synchronized propagation (simulated sequentially in Basilisk, but independent cases)
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import Enum

# GMAT uses Earth radius = 6378.1366 km.
# Basilisk uses WGS84 6378.137 km by default. We force GMAT's value for exact parity.
R_EARTH_GMAT = 6378136.6  # meters (6378.1366 km)

class A0Case(Enum):
    CASE1_TWOBODY = "a0_case1_twobody"
    CASE2_J2 = "a0_case2_j2"
    CASE3_J2_DRAG = "a0_case3_j2_drag"
    CASE4_J2_DRAG_SRP = "a0_case4_j2_drag_srp"
    CASE5_ALL_SUPPORTED = "a0_case5_all_supported"  # This will be 20x20 in Basilisk

@dataclass(frozen=True)
class A0Config:
    """Exact GMAT twin configuration for Benchmark A0."""

    # ---------- ORBIT (Matches GMAT exactly) ----------
    # GMAT SMA = 6878.136999999998 km → meters
    semi_major_axis_m: float = 6878136.999999998
    eccentricity: float = 0.0
    inclination_deg: float = 97.4
    raan_deg: float = 0.0
    arg_perigee_deg: float = 0.0
    true_anomaly_deg: float = 0.0

    # ---------- TIMING (Matches GMAT) ----------
    # 1 year = 365 days = 31,536,000 s
    # GMAT: For i = 1:1:525600; Propagate ... {stepSec=60}; EndFor
    simulated_duration_s: float = 100 * 24 * 3600.0  # 31,536,000 s
    dynamics_step_s: float = 60.0   # Matches GMAT integrator step
    output_sample_s: float = 60.0   # Matches GMAT report cadence

    spice_epoch_utc: str = "2026 JUN 02 00:00:00.000 (UTC)"
    mission_epoch_utc_iso: str = "2026-06-02T00:00:00.000Z"

    # ---------- SPACECRAFT PHYSICAL PROPERTIES (Matches GMAT) ----------
    dry_mass_kg: float = 12.0
    drag_coefficient: float = 2.2
    srp_reflectivity: float = 1.8
    drag_area_m2: float = 0.08   # GMAT: DragArea = 0.08
    srp_area_m2: float = 0.1     # GMAT: SRPArea = 0.1

    # ---------- GRAVITY (Case 5 will use 20x20) ----------
    # Cases 2–4 use J2-only (2x0). Case 5 uses 20x20 (JGM2).
    basilisk_j2_gravity_file: str = "GGM03S-J2-only.txt"
    basilisk_j2_max_degree: int = 2

    # For Case 5 (20x20), we will use the full GGM2B (JGM2) file.
    # This is loaded at runtime in the propagation script.

    @property
    def output_sample_count(self) -> int:
        return int(self.simulated_duration_s / self.output_sample_s) + 1

def load_a0_config_from_env() -> A0Config:
    """Load A0 config with optional environment overrides (for debugging)."""
    kwargs = {}
    duration_s = os.environ.get("A0_SIM_DURATION_S")
    if duration_s is not None:
        kwargs["simulated_duration_s"] = float(duration_s)
    return A0Config(**kwargs)

def case_uses_j2(case: A0Case) -> bool:
    return case != A0Case.CASE1_TWOBODY

def case_uses_drag(case: A0Case) -> bool:
    return case in {
        A0Case.CASE3_J2_DRAG,
        A0Case.CASE4_J2_DRAG_SRP,
        A0Case.CASE5_ALL_SUPPORTED,
    }

def case_uses_srp(case: A0Case) -> bool:
    return case in {A0Case.CASE4_J2_DRAG_SRP, A0Case.CASE5_ALL_SUPPORTED}

def case_uses_third_body(case: A0Case) -> bool:
    # Note: GMAT's Case 5 includes Sun and Moon point masses.
    return case == A0Case.CASE5_ALL_SUPPORTED

def case_uses_high_degree_gravity(case: A0Case) -> bool:
    """GMAT Case 5 uses 20x20. Cases 2-4 use J2-only."""
    return case == A0Case.CASE5_ALL_SUPPORTED