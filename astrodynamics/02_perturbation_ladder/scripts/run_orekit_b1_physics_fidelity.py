#!/usr/bin/env python3
"""Benchmark B1 Physics Fidelity — Orekit propagation for Cases 1–5."""

from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import orekit_jpype

orekit_jpype.initVM()

from orekit_jpype.pyhelpers import setup_orekit_curdir
from org.hipparchus.ode.nonstiff import DormandPrince853Integrator
from org.hipparchus.util import FastMath
from org.orekit.bodies import CelestialBodyFactory, OneAxisEllipsoid
from org.orekit.forces.drag import DragForce, IsotropicDrag
from org.orekit.forces.gravity import HolmesFeatherstoneAttractionModel, ThirdBodyAttraction
from org.orekit.forces.gravity.potential import GravityFieldFactory
from org.orekit.forces.radiation import IsotropicRadiationSingleCoefficient, SolarRadiationPressure
from org.orekit.frames import FramesFactory
from org.orekit.models.earth.atmosphere import NRLMSISE00
from org.orekit.models.earth.atmosphere.data import CssiSpaceWeatherData
from org.orekit.orbits import KeplerianOrbit, OrbitType, PositionAngleType
from org.orekit.propagation import SpacecraftState
from org.orekit.propagation.numerical import NumericalPropagator
from org.orekit.time import AbsoluteDate, TimeScalesFactory
from org.orekit.utils import Constants, IERSConventions

from b1_metrics_postprocess import (
    compute_endpoint_drift_metrics,
    eci_position_to_subpoint_lat_lon,
    histories_from_rows,
)
from b1_physics_config import B1Case, B1Config, case_uses_drag, case_uses_j2, case_uses_srp, case_uses_third_body, load_b1_config_from_env

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
OREKIT_DATA_ZIP = WORKSPACE_ROOT / "orekit-data.zip"
RESULTS_DIR = SCRIPT_DIR / "results/orekit_b1"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_b/b1_physics_fidelity/orekit"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B1_METRICS_COMPARISON.csv"


@dataclass(frozen=True)
class OrekitForceModelSpec:
    gravity_harmonics_degree: int
    gravity_harmonics_order: int
    drag_enabled: bool
    atmosphere_model: str | None
    srp_enabled: bool
    third_body_bodies: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "gravity_harmonics_degree": self.gravity_harmonics_degree,
            "gravity_harmonics_order": self.gravity_harmonics_order,
            "drag_enabled": self.drag_enabled,
            "atmosphere_model": self.atmosphere_model,
            "srp_enabled": self.srp_enabled,
            "third_body_bodies": list(self.third_body_bodies),
        }


def describe_orekit_case(case: B1Case) -> dict[str, Any]:
    """Orekit-specific perturbation documentation for logs and summaries."""
    base_forces = {
        B1Case.CASE1_TWOBODY: [
            "HolmesFeatherstoneAttractionModel with normalized gravity harmonics 0x0 (central Earth)",
        ],
        B1Case.CASE2_J2: [
            "HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)",
        ],
        B1Case.CASE3_J2_DRAG: [
            "HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)",
            "DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)",
            "NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)",
        ],
        B1Case.CASE4_J2_DRAG_SRP: [
            "HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)",
            "DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)",
            "NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)",
            "SolarRadiationPressure with IsotropicRadiationSingleCoefficient(Cr=1.8, area=1.0 m²)",
        ],
        B1Case.CASE5_ALL_SUPPORTED: [
            "HolmesFeatherstoneAttractionModel with normalized gravity harmonics 2x0 (J2-only policy)",
            "DragForce with IsotropicDrag(Cd=2.2, area=0.01 m²)",
            "NRLMSISE00 atmosphere with CssiSpaceWeatherData(SpaceWeather-All-v1.2.txt)",
            "SolarRadiationPressure with IsotropicRadiationSingleCoefficient(Cr=1.8, area=1.0 m²)",
            "ThirdBodyAttraction(Sun)",
            "ThirdBodyAttraction(Moon)",
        ],
    }
    increments = {
        B1Case.CASE1_TWOBODY: "Baseline case. No perturbations beyond central Earth gravity.",
        B1Case.CASE2_J2: "Adds J2 via gravity harmonics 2x0 (replaces 0x0 central-only gravity).",
        B1Case.CASE3_J2_DRAG: "Adds atmospheric drag (NRLMSISE00 + isotropic drag model).",
        B1Case.CASE4_J2_DRAG_SRP: "Adds solar radiation pressure (isotropic single-coefficient model).",
        B1Case.CASE5_ALL_SUPPORTED: (
            "Adds Sun and Moon third-body gravity via ThirdBodyAttraction. "
            "Inherits all force models from Case 4."
        ),
    }
    previous_case = {
        B1Case.CASE1_TWOBODY: None,
        B1Case.CASE2_J2: B1Case.CASE1_TWOBODY,
        B1Case.CASE3_J2_DRAG: B1Case.CASE2_J2,
        B1Case.CASE4_J2_DRAG_SRP: B1Case.CASE3_J2_DRAG,
        B1Case.CASE5_ALL_SUPPORTED: B1Case.CASE4_J2_DRAG_SRP,
    }

    doc: dict[str, Any] = {
        "enabled_orekit_force_models": base_forces[case],
        "increment_vs_previous_case": increments[case],
        "previous_case_id": previous_case[case].value if previous_case[case] else None,
    }

    if case == B1Case.CASE5_ALL_SUPPORTED:
        doc["all_supported_orekit_definition"] = {
            "note": (
                "Cross-tool Case 5 label means 'all perturbations this tool supports natively "
                "for this benchmark'. The Orekit-specific definition is below."
            ),
            "inherits_from_case": B1Case.CASE4_J2_DRAG_SRP.value,
            "added_in_case5_vs_case4": [
                "ThirdBodyAttraction(Sun)",
                "ThirdBodyAttraction(Moon)",
            ],
            "full_force_stack": base_forces[case],
            "not_added_in_case5": [
                "Higher-degree/order gravity field (beyond 2x0 J2-only policy)",
                "Solid Earth tides or ocean tides on gravity field",
                "Additional planets as third bodies",
                "Relativistic corrections",
                "Earth albedo / infrared radiation pressure",
                "Empirical accelerations or custom force models",
            ],
        }

    return doc


def orekit_b1_physics_scope() -> dict[str, Any]:
    """What this Orekit B1 run exercises vs broader Orekit physics capability."""
    return {
        "covered_in_this_b1_run": [
            "Numerical propagation (DormandPrince853Integrator)",
            "Central gravity and J2-only gravity harmonics policy",
            "NRLMSISE00 atmospheric drag with space-weather data",
            "Isotropic solar radiation pressure",
            "Sun and Moon third-body point-mass gravity",
            "7-day LEO state history export and drift metrics",
        ],
        "orekit_native_but_not_in_b1": [
            "Full gravity field to high degree/order (e.g. EGM2008, tides)",
            "Alternative atmosphere models (Harris-Priester, exponential, DTM2000, etc.)",
            "Box-wing or multi-panel radiation pressure models",
            "Third-body perturbations from additional solar-system bodies",
            "Relativistic correction force models",
            "Earth albedo and infrared radiation pressure",
            "Empirical acceleration / custom force models",
            "Semi-analytical propagators (e.g. DSST) and TLE/SGP4 workflows",
            "Attitude-dependent drag and SRP (requires attitude dynamics, B2 scope)",
            "Maneuver modeling and event detection beyond fixed-force propagation",
        ],
        "conclusion": (
            "Orekit B1 demonstrates a staged perturbation ladder using native Orekit force models "
            "aligned with the frozen cross-tool benchmark. It does not claim to exercise every "
            "physics feature Orekit supports."
        ),
    }


def init_orekit() -> None:
    setup_orekit_curdir(str(OREKIT_DATA_ZIP))


def force_model_spec(case: B1Case) -> OrekitForceModelSpec:
    if case == B1Case.CASE1_TWOBODY:
        return OrekitForceModelSpec(0, 0, False, None, False, ())
    if case == B1Case.CASE2_J2:
        return OrekitForceModelSpec(2, 0, False, None, False, ())
    if case == B1Case.CASE3_J2_DRAG:
        return OrekitForceModelSpec(2, 0, True, "NRLMSISE00 + CssiSpaceWeatherData", False, ())
    if case == B1Case.CASE4_J2_DRAG_SRP:
        return OrekitForceModelSpec(2, 0, True, "NRLMSISE00 + CssiSpaceWeatherData", True, ())
    return OrekitForceModelSpec(
        2,
        0,
        True,
        "NRLMSISE00 + CssiSpaceWeatherData",
        True,
        ("Sun", "Moon"),
    )


def create_initial_orbit(config: B1Config):
    utc = TimeScalesFactory.getUTC()
    date = AbsoluteDate(2026, 6, 2, 0, 0, 0.0, utc)
    frame = FramesFactory.getEME2000()
    return KeplerianOrbit(
        config.semi_major_axis_m,
        config.eccentricity,
        FastMath.toRadians(config.inclination_deg),
        FastMath.toRadians(config.arg_perigee_deg),
        FastMath.toRadians(config.raan_deg),
        FastMath.toRadians(config.true_anomaly_deg),
        PositionAngleType.TRUE,
        frame,
        date,
        Constants.EIGEN5C_EARTH_MU,
    )


def build_propagator(config: B1Config, case: B1Case) -> tuple[NumericalPropagator, OrekitForceModelSpec]:
    spec = force_model_spec(case)
    earth_frame = FramesFactory.getITRF(IERSConventions.IERS_2010, True)
    earth = OneAxisEllipsoid(
        Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
        Constants.WGS84_EARTH_FLATTENING,
        earth_frame,
    )
    sun = CelestialBodyFactory.getSun()
    moon = CelestialBodyFactory.getMoon()

    integrator = DormandPrince853Integrator(0.1, 300.0, 1.0, 1.0)
    propagator = NumericalPropagator(integrator)
    propagator.setOrbitType(OrbitType.CARTESIAN)
    propagator.setInitialState(SpacecraftState(create_initial_orbit(config), config.dry_mass_kg))

    provider = GravityFieldFactory.getNormalizedProvider(
        spec.gravity_harmonics_degree,
        spec.gravity_harmonics_order,
    )
    propagator.addForceModel(HolmesFeatherstoneAttractionModel(earth_frame, provider))

    if case_uses_drag(case):
        space_weather = CssiSpaceWeatherData("SpaceWeather-All-v1.2.txt")
        atmosphere = NRLMSISE00(space_weather, sun, earth)
        drag = IsotropicDrag(config.drag_area_m2, config.drag_coefficient)
        propagator.addForceModel(DragForce(atmosphere, drag))

    if case_uses_srp(case):
        radiation = IsotropicRadiationSingleCoefficient(config.srp_area_m2, config.srp_reflectivity)
        propagator.addForceModel(SolarRadiationPressure(sun, earth, radiation))

    if case_uses_third_body(case):
        propagator.addForceModel(ThirdBodyAttraction(sun))
        propagator.addForceModel(ThirdBodyAttraction(moon))

    return propagator, spec


def keplerian_row(state: SpacecraftState, time_s: float, frame) -> dict[str, float]:
    kep = KeplerianOrbit(state.getOrbit())
    arg_lat_deg = math.degrees(kep.getPerigeeArgument() + kep.getTrueAnomaly())
    return {
        "time_s": float(time_s),
        "semi_major_axis_m": float(kep.getA()),
        "eccentricity": float(kep.getE()),
        "inclination_deg": math.degrees(float(kep.getI())),
        "raan_deg": math.degrees(float(kep.getRightAscensionOfAscendingNode())),
        "arg_perigee_deg": math.degrees(float(kep.getPerigeeArgument())),
        "true_anomaly_deg": math.degrees(float(kep.getTrueAnomaly())),
        "argument_of_latitude_deg": arg_lat_deg,
    }


def propagate_case(config: B1Config, case: B1Case) -> tuple[list[dict[str, Any]], OrekitForceModelSpec, float]:
    frame = FramesFactory.getEME2000()
    propagator, spec = build_propagator(config, case)
    orbit = create_initial_orbit(config)
    date0 = orbit.getDate()
    final_date = date0.shiftedBy(float(config.simulated_duration_s))

    start_s = time.perf_counter()
    generator = propagator.getEphemerisGenerator()
    propagator.propagate(final_date)
    ephemeris = generator.getGeneratedEphemeris()
    propagation_runtime_s = time.perf_counter() - start_s

    sample_times = np.arange(
        0.0,
        config.simulated_duration_s + 0.5 * config.output_sample_s,
        config.output_sample_s,
        dtype=float,
    )
    rows: list[dict[str, Any]] = []
    for time_s in sample_times:
        state = ephemeris.propagate(date0.shiftedBy(float(time_s)))
        pv = state.getPVCoordinates(frame)
        position = pv.getPosition()
        velocity = pv.getVelocity()
        elements = keplerian_row(state, float(time_s), frame)
        lat_deg, lon_deg = eci_position_to_subpoint_lat_lon(
            np.array([float(position.getX()), float(position.getY()), float(position.getZ())]),
            float(time_s),
        )
        rows.append(
            {
                **elements,
                "x_m": float(position.getX()),
                "y_m": float(position.getY()),
                "z_m": float(position.getZ()),
                "vx_m_s": float(velocity.getX()),
                "vy_m_s": float(velocity.getY()),
                "vz_m_s": float(velocity.getZ()),
                "lat_deg": lat_deg,
                "lon_deg": lon_deg,
            }
        )

    return rows, spec, propagation_runtime_s


def write_state_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "time_s",
        "x_m",
        "y_m",
        "z_m",
        "vx_m_s",
        "vy_m_s",
        "vz_m_s",
        "semi_major_axis_m",
        "eccentricity",
        "inclination_deg",
        "raan_deg",
        "arg_perigee_deg",
        "true_anomaly_deg",
        "argument_of_latitude_deg",
        "lat_deg",
        "lon_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_ground_track_plot(path: Path, rows: list[dict[str, Any]], title: str) -> None:
    lats = [row["lat_deg"] for row in rows]
    lons = [row["lon_deg"] for row in rows]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(lons, lats, linewidth=0.8)
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_altitude_plot(path: Path, rows: list[dict[str, Any]], title: str) -> None:
    times_h = [row["time_s"] / 3600.0 for row in rows]
    altitudes_km = [(row["semi_major_axis_m"] - (6378137.0)) / 1000.0 for row in rows]
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times_h, altitudes_km, linewidth=1.0)
    ax.set_xlabel("Time [hours]")
    ax.set_ylabel("Altitude [km]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run_id_for_case(case: B1Case) -> str:
    return f"b1_orekit_{case.value.removeprefix('b1_')}"


def write_case_outputs(
    config: B1Config,
    case: B1Case,
    rows: list[dict[str, Any]],
    spec: OrekitForceModelSpec,
    propagation_runtime_s: float,
    drift_metrics: dict[str, float] | None,
) -> dict[str, Any]:
    case_dir = RESULTS_DIR / case.value
    data_case_dir = DATA_DIR / case.value
    case_dir.mkdir(parents=True, exist_ok=True)
    data_case_dir.mkdir(parents=True, exist_ok=True)

    state_csv = case_dir / "state_history.csv"
    write_state_csv(state_csv, rows)
    write_state_csv(data_case_dir / "state_history.csv", rows)

    ground_track_plot = case_dir / "ground_track.png"
    altitude_plot = case_dir / "altitude_vs_time.png"
    save_ground_track_plot(ground_track_plot, rows, f"Orekit B1 {case.value} ground track")
    save_altitude_plot(altitude_plot, rows, f"Orekit B1 {case.value} altitude")

    summary = {
        "benchmark": "B1",
        "simulator": "Orekit",
        "run_id": run_id_for_case(case),
        "case_id": case.value,
        "status": "pass",
        "epoch_utc": "2026-06-02T00:00:00.000Z",
        "simulated_duration_s": rows[-1]["time_s"],
        "sample_count": len(rows),
        "force_model": spec.to_dict(),
        "spacecraft": {
            "dry_mass_kg": config.dry_mass_kg,
            "drag_coefficient": config.drag_coefficient,
            "srp_reflectivity": config.srp_reflectivity,
            "drag_area_m2": config.drag_area_m2,
            "srp_area_m2": config.srp_area_m2,
            "attitude_policy": "fixed_inertial (isotropic drag/SRP coefficients)",
        },
        "integrator": "DormandPrince853Integrator(0.1, 300.0, 1.0, 1.0)",
        "runtime_s": {
            "propagation": propagation_runtime_s,
            "runtime_ratio": rows[-1]["time_s"] / propagation_runtime_s if propagation_runtime_s > 0 else None,
        },
        "drift_metrics_vs_case1": drift_metrics or {},
        "orekit_perturbation_documentation": describe_orekit_case(case),
        "artifacts": {
            "state_history_csv": str(state_csv),
            "ground_track_plot": str(ground_track_plot),
            "altitude_plot": str(altitude_plot),
        },
    }

    summary_path = case_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    with (data_case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary


def write_metrics_csv(rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "simulator",
        "run_id",
        "case_id",
        "position_drift_vs_case1_m",
        "ground_track_lon_drift_deg",
        "raan_change_deg",
        "altitude_change_m",
        "along_track_phase_drift_deg",
        "final_altitude_m",
        "wall_clock_runtime_s",
        "runtime_ratio",
        "drag_enabled",
        "srp_enabled",
        "third_body_enabled",
    ]
    METRICS_CSV.parent.mkdir(parents=True, exist_ok=True)
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_case5_definition_markdown(
    path: Path,
    case5_doc: dict[str, Any],
    physics_scope: dict[str, Any],
) -> None:
    lines = [
        "# Orekit B1 Case 5 — all supported (Orekit-specific definition)",
        "",
        "Cross-tool benchmark documents use the label **all supported** generically. "
        "This file defines exactly what that means for **Orekit** in Benchmark B1.",
        "",
        "## Inherits from",
        "",
        f"- `{case5_doc.get('inherits_from_case', 'b1_case4_j2_drag_srp')}` (full Case 4 force stack)",
        "",
        "## Added in Case 5 vs Case 4",
        "",
        *[f"- {item}" for item in case5_doc.get("added_in_case5_vs_case4", [])],
        "",
        "## Full Orekit force stack in Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("full_force_stack", [])],
        "",
        "## Explicitly not added in Orekit Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("not_added_in_case5", [])],
        "",
        "## Does Orekit B1 cover all Orekit physics fidelity features?",
        "",
        physics_scope["conclusion"],
        "",
        "### Covered in this B1 run",
        "",
        *[f"- {item}" for item in physics_scope["covered_in_this_b1_run"]],
        "",
        "### Orekit-native but not in B1",
        "",
        *[f"- {item}" for item in physics_scope["orekit_native_but_not_in_b1"]],
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_run_log(
    path: Path,
    config: B1Config,
    summaries: list[dict[str, Any]],
    total_runtime_s: float,
    init_runtime_s: float,
) -> None:
    run_date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    lines = [
        f"# Orekit B1 run log ({run_date})",
        "",
        "## Configuration",
        "",
        f"- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`",
        f"- Duration: {config.simulated_duration_s / 86400.0:.1f} days",
        f"- Output cadence: {config.output_sample_s} s",
        f"- Orekit data: `{OREKIT_DATA_ZIP}`",
        "",
        "## Cases executed",
        "",
    ]
    for summary in summaries:
        fm = summary["force_model"]
        lines.extend(
            [
                f"### {summary['case_id']}",
                "",
                f"- Run ID: `{summary['run_id']}`",
                f"- Gravity harmonics: {fm['gravity_harmonics_degree']}x{fm['gravity_harmonics_order']}",
                f"- Drag: {fm['drag_enabled']} ({fm['atmosphere_model'] or 'n/a'})",
                f"- SRP: {fm['srp_enabled']}",
                f"- Third body: {', '.join(fm['third_body_bodies']) if fm['third_body_bodies'] else 'none'}",
                f"- Propagation runtime: {summary['runtime_s']['propagation']:.3f} s",
                f"- Runtime ratio: {summary['runtime_s']['runtime_ratio']:.1f}",
                "",
            ]
        )
        orekit_doc = summary.get("orekit_perturbation_documentation", {})
        if orekit_doc:
            lines.extend(
                [
                    "Orekit force models enabled:",
                    *[f"- {item}" for item in orekit_doc.get("enabled_orekit_force_models", [])],
                    "",
                    f"Increment vs previous case: {orekit_doc.get('increment_vs_previous_case', 'n/a')}",
                    "",
                ]
            )
        if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value:
            case5 = orekit_doc.get("all_supported_orekit_definition", {})
            lines.extend(
                [
                    "**Case 5 — all supported (Orekit-specific definition):**",
                    f"- Inherits from: `{case5.get('inherits_from_case', 'n/a')}`",
                    "- Added in Case 5 vs Case 4:",
                    *[f"  - {item}" for item in case5.get("added_in_case5_vs_case4", [])],
                    "- Not included in Orekit Case 5:",
                    *[f"  - {item}" for item in case5.get("not_added_in_case5", [])],
                    "",
                ]
            )

        drift = summary.get("drift_metrics_vs_case1", {})
        if drift:
            lines.extend(
                [
                    "Drift vs Case 1 (final epoch):",
                    f"- Position drift: {drift['position_drift_vs_case1_m']:.3f} m",
                    f"- Ground-track lon drift: {drift['ground_track_lon_drift_deg']:.6f} deg",
                    f"- RAAN change: {drift['raan_change_deg']:.6f} deg",
                    f"- Altitude change: {drift['altitude_change_m']:.3f} m",
                    "",
                ]
            )

    scope = orekit_b1_physics_scope()
    lines.extend(
        [
            "## Orekit B1 physics scope",
            "",
            scope["conclusion"],
            "",
            "Covered in this B1 run:",
            *[f"- {item}" for item in scope["covered_in_this_b1_run"]],
            "",
            "Orekit-native features not exercised in B1:",
            *[f"- {item}" for item in scope["orekit_native_but_not_in_b1"]],
            "",
        ]
    )

    lines.extend(
        [
            "## Runtime",
            "",
            f"- Orekit init: {init_runtime_s:.3f} s",
            f"- Total pipeline: {total_runtime_s:.3f} s",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    if not OREKIT_DATA_ZIP.exists():
        raise FileNotFoundError(
            f"Missing {OREKIT_DATA_ZIP}. Download with orekit_jpype.pyhelpers.download_orekit_data_curdir()."
        )

    config = load_b1_config_from_env()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    total_start_s = time.perf_counter()
    init_start_s = time.perf_counter()
    init_orekit()
    init_runtime_s = time.perf_counter() - init_start_s

    print("Running Orekit B1 Physics Fidelity (Cases 1–5)...")

    histories: dict[B1Case, list[dict[str, Any]]] = {}
    summaries: list[dict[str, Any]] = []
    metrics_rows: list[dict[str, Any]] = []

    for case in B1Case:
        print(f"  Propagating {case.value}...")
        rows, spec, propagation_runtime_s = propagate_case(config, case)
        histories[case] = rows
        summaries.append(
            {
                "case": case,
                "rows": rows,
                "spec": spec,
                "propagation_runtime_s": propagation_runtime_s,
            }
        )

    case1_history = histories_from_rows(histories[B1Case.CASE1_TWOBODY])
    final_summaries: list[dict[str, Any]] = []

    for item in summaries:
        case = item["case"]
        rows = item["rows"]
        drift_metrics = None
        if case != B1Case.CASE1_TWOBODY:
            drift_metrics = compute_endpoint_drift_metrics(
                histories_from_rows(rows),
                case1_history,
            )
        summary = write_case_outputs(
            config,
            case,
            rows,
            item["spec"],
            item["propagation_runtime_s"],
            drift_metrics,
        )
        final_summaries.append(summary)

        fm = summary["force_model"]
        metrics_rows.append(
            {
                "simulator": "Orekit",
                "run_id": summary["run_id"],
                "case_id": summary["case_id"],
                "position_drift_vs_case1_m": (drift_metrics or {}).get("position_drift_vs_case1_m", 0.0),
                "ground_track_lon_drift_deg": (drift_metrics or {}).get("ground_track_lon_drift_deg", 0.0),
                "raan_change_deg": (drift_metrics or {}).get("raan_change_deg", rows[-1]["raan_deg"] - rows[0]["raan_deg"]),
                "altitude_change_m": (drift_metrics or {}).get("altitude_change_m", rows[-1]["semi_major_axis_m"] - rows[0]["semi_major_axis_m"]),
                "along_track_phase_drift_deg": (drift_metrics or {}).get("along_track_phase_drift_deg", 0.0),
                "final_altitude_m": (drift_metrics or {}).get("final_altitude_m", rows[-1]["semi_major_axis_m"] - 6378137.0),
                "wall_clock_runtime_s": summary["runtime_s"]["propagation"],
                "runtime_ratio": summary["runtime_s"]["runtime_ratio"],
                "drag_enabled": fm["drag_enabled"],
                "srp_enabled": fm["srp_enabled"],
                "third_body_enabled": bool(fm["third_body_bodies"]),
            }
        )

    write_metrics_csv(metrics_rows)

    physics_scope = orekit_b1_physics_scope()
    case5_summary = next(
        summary for summary in final_summaries if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value
    )
    case5_doc = case5_summary["orekit_perturbation_documentation"]["all_supported_orekit_definition"]

    aggregate_summary = {
        "benchmark": "B1",
        "simulator": "Orekit",
        "status": "pass",
        "cases": [summary["case_id"] for summary in final_summaries],
        "summaries": final_summaries,
        "metrics_csv": str(METRICS_CSV),
        "parameter_table": "data/benchmark_b/B1_PARAMETER_TABLE.md",
        "orekit_case5_all_supported_definition": case5_doc,
        "orekit_b1_physics_scope": physics_scope,
        "runtime_s": {
            "orekit_init": init_runtime_s,
            "total_pipeline": time.perf_counter() - total_start_s,
        },
    }
    with (DATA_DIR / "orekit_b1_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(aggregate_summary, handle, indent=2)

    write_case5_definition_markdown(DATA_DIR / "orekit_b1_case5_definition.md", case5_doc, physics_scope)

    write_run_log(
        DATA_DIR / "orekit_b1_run_log.md",
        config,
        final_summaries,
        aggregate_summary["runtime_s"]["total_pipeline"],
        init_runtime_s,
    )

    print(f"Orekit B1 complete. Results: {DATA_DIR}")
    print(f"Metrics CSV: {METRICS_CSV}")
    print(f"Total runtime: {aggregate_summary['runtime_s']['total_pipeline']:.3f} s")


if __name__ == "__main__":
    main()
