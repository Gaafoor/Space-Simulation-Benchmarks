#!/usr/bin/env python3
"""Benchmark B1 Physics Fidelity — Basilisk propagation for Cases 1–5."""

from __future__ import annotations

import csv
import json
import math
import platform
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import Basilisk
from Basilisk.simulation import dragDynamicEffector, exponentialAtmosphere, radiationPressure, spacecraft
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, simSetPlanetEnvironment
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

from b1_metrics_postprocess import (
    compute_endpoint_drift_metrics,
    eci_position_to_subpoint_lat_lon,
    histories_from_rows,
)
from b1_physics_config import (
    B1Case,
    B1Config,
    R_EARTH,
    case_uses_drag,
    case_uses_j2,
    case_uses_srp,
    case_uses_third_body,
    load_b1_config_from_env,
)

MU_EARTH = 3.986004418e14

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
RESULTS_DIR = SCRIPT_DIR / "results/basilisk_b11"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_b/b1_physics_fidelity/basilisk11"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_b/B1_METRICS_COMPARISON11.csv"


@dataclass(frozen=True)
class BasiliskForceModelSpec:
    gravity_model: str
    j2_enabled: bool
    drag_enabled: bool
    atmosphere_model: str | None
    srp_enabled: bool
    srp_model: str | None
    third_body_bodies: tuple[str, ...]
    spice_enabled: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "gravity_model": self.gravity_model,
            "j2_enabled": self.j2_enabled,
            "drag_enabled": self.drag_enabled,
            "atmosphere_model": self.atmosphere_model,
            "srp_enabled": self.srp_enabled,
            "srp_model": self.srp_model,
            "third_body_bodies": list(self.third_body_bodies),
            "spice_enabled": self.spice_enabled,
        }


def force_model_spec(case: B1Case, config: B1Config) -> BasiliskForceModelSpec:
    if case == B1Case.CASE1_TWOBODY:
        return BasiliskForceModelSpec(
            "Point-mass Earth (mu only, no harmonics)",
            False,
            False,
            None,
            False,
            None,
            (),
            False,
        )
    if case == B1Case.CASE2_J2:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            False,
            None,
            False,
            None,
            (),
            False,
        )
    if case == B1Case.CASE3_J2_DRAG:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            True,
            "ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset)",
            False,
            None,
            (),
            False,
        )
    if case == B1Case.CASE4_J2_DRAG_SRP:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            True,
            "ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset)",
            True,
            "RadiationPressure cannonball model",
            (),
            True,
        )
    return BasiliskForceModelSpec(
        f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
        True,
        True,
        "ExponentialAtmosphere (simSetPlanetEnvironment.exponentialAtmosphere earth preset)",
        True,
        "RadiationPressure cannonball model",
        ("Sun", "Moon"),
        True,
    )


def describe_basilisk_case(case: B1Case, config: B1Config) -> dict[str, Any]:
    base_forces = {
        B1Case.CASE1_TWOBODY: [
            "simIncludeGravBody.gravBodyFactory.createEarth() with point-mass gravity (mu only)",
        ],
        B1Case.CASE2_J2: [
            f"Earth gravity via GGM03S-J2-only.txt (max degree {config.basilisk_j2_max_degree})",
        ],
        B1Case.CASE3_J2_DRAG: [
            f"Earth gravity via GGM03S-J2-only.txt (max degree {config.basilisk_j2_max_degree})",
            "ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')",
            f"DragDynamicEffector (Cd={config.drag_coefficient}, projected area={config.drag_area_m2} m²)",
        ],
        B1Case.CASE4_J2_DRAG_SRP: [
            f"Earth gravity via GGM03S-J2-only.txt (max degree {config.basilisk_j2_max_degree})",
            "ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')",
            f"DragDynamicEffector (Cd={config.drag_coefficient}, projected area={config.drag_area_m2} m²)",
            f"RadiationPressure cannonball (Cr={config.srp_reflectivity}, area={config.srp_area_m2} m²)",
            "SPICE sun ephemeris for SRP",
        ],
        B1Case.CASE5_ALL_SUPPORTED: [
            f"Earth gravity via GGM03S-J2-only.txt (max degree {config.basilisk_j2_max_degree})",
            "ExponentialAtmosphere + simSetPlanetEnvironment.exponentialAtmosphere('earth')",
            f"DragDynamicEffector (Cd={config.drag_coefficient}, projected area={config.drag_area_m2} m²)",
            f"RadiationPressure cannonball (Cr={config.srp_reflectivity}, area={config.srp_area_m2} m²)",
            "SPICE ephemerides (de430, naif0012, de_403_masses, pck00010)",
            "Sun and Moon point-mass gravity via gravBodyFactory",
        ],
    }
    increments = {
        B1Case.CASE1_TWOBODY: "Baseline case. Point-mass Earth gravity only.",
        B1Case.CASE2_J2: "Adds J2 via GGM03S-J2-only spherical harmonics.",
        B1Case.CASE3_J2_DRAG: "Adds ExponentialAtmosphere + DragDynamicEffector.",
        B1Case.CASE4_J2_DRAG_SRP: "Adds RadiationPressure (cannonball) with SPICE Sun ephemeris.",
        B1Case.CASE5_ALL_SUPPORTED: (
            "Adds Sun and Moon point-mass gravity bodies via gravBodyFactory. "
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
        "enabled_basilisk_force_models": base_forces[case],
        "increment_vs_previous_case": increments[case],
        "previous_case_id": previous_case[case].value if previous_case[case] else None,
    }

    if case == B1Case.CASE5_ALL_SUPPORTED:
        doc["all_supported_basilisk_definition"] = {
            "note": (
                "Cross-tool Case 5 label means 'all perturbations this tool supports natively "
                "for this benchmark'. The Basilisk-specific definition is below."
            ),
            "inherits_from_case": B1Case.CASE4_J2_DRAG_SRP.value,
            "added_in_case5_vs_case4": [
                "Moon point-mass gravity via gravBodyFactory",
                "Sun point-mass gravity via gravBodyFactory (in addition to SRP ephemeris use)",
            ],
            "full_force_stack": base_forces[case],
            "not_added_in_case5": [
                "Higher-degree/order gravity field (beyond frozen J2-only policy)",
                "MsisAtmosphere or other atmosphere models",
                "FacetSRPDynamicEffector or faceted drag models",
                "Additional planetary third bodies beyond Sun and Moon",
                "Attitude-dependent drag/SRP (B2 scope)",
            ],
        }

    return doc


def basilisk_b1_physics_scope() -> dict[str, Any]:
    return {
        "covered_in_this_b1_run": [
            "Spacecraft hub propagation with 10 s dynamics step",
            "Point-mass Earth gravity (Case 1)",
            "J2-only GGM03S gravity harmonics (Cases 2–5)",
            "ExponentialAtmosphere drag with DragDynamicEffector",
            "RadiationPressure cannonball SRP with SPICE Sun ephemeris",
            "Sun and Moon point-mass third-body gravity (Case 5)",
            "7-day LEO state history export and drift metrics",
        ],
        "basilisk_native_but_not_in_b1": [
            "Full gravity field to high degree/order",
            "MsisAtmosphere and other atmosphere models",
            "Facet drag / facet SRP dynamic effectors",
            "Additional third bodies or custom gravity bodies",
            "Integrating custom Python force models via DynamicEffector",
            "Attitude-dependent drag and SRP (B2 scope)",
            "Maneuver and event-driven propagation workflows",
        ],
        "conclusion": (
            "Basilisk B1 demonstrates the frozen cross-tool perturbation ladder with native Basilisk "
            "modules. It does not claim to exercise every physics feature Basilisk supports."
        ),
    }


def load_spice_kernels() -> None:
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de430)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.naif0012)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de_403_masses)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.pck00010)))


def initial_state_from_elements(config: B1Config) -> tuple[np.ndarray, np.ndarray]:
    oe = orbitalMotion.ClassicElements()
    oe.a = config.semi_major_axis_m
    oe.e = config.eccentricity
    oe.i = math.radians(config.inclination_deg)
    oe.Omega = math.radians(config.raan_deg)
    oe.omega = math.radians(config.arg_perigee_deg)
    oe.f = math.radians(config.true_anomaly_deg)
    r0, v0 = orbitalMotion.elem2rv(MU_EARTH, oe)
    return np.array(r0, dtype=float), np.array(v0, dtype=float)


def configure_gravity(config: B1Config, case: B1Case, sc_object, grav_factory):
    if case_uses_third_body(case):
        grav_bodies = grav_factory.createBodies(["sun", "earth", "moon"])
        earth = grav_bodies["earth"]
    else:
        earth = grav_factory.createEarth()
        grav_bodies = {"earth": earth}

    earth.isCentralBody = True
    if case_uses_j2(case):
        j2_path = get_path(getattr(DataFile.LocalGravData, "GGM03S_J2_only"))
        earth.useSphericalHarmonicsGravityModel(str(j2_path), config.basilisk_j2_max_degree)
    else:
        earth.mu = MU_EARTH
        earth.radEquator = R_EARTH

    grav_factory.addBodiesTo(sc_object)
    return earth, grav_bodies


def propagate_case(config: B1Config, case: B1Case) -> tuple[list[dict[str, Any]], BasiliskForceModelSpec, float]:
    spec = force_model_spec(case, config)
    sc_sim = SimulationBaseClass.SimBaseClass()
    dyn_process = sc_sim.CreateNewProcess("b1DynProcess")
    env_process = sc_sim.CreateNewProcess("b1EnvProcess")
    dyn_task = sc_sim.CreateNewTask("b1DynTask", macros.sec2nano(config.dynamics_step_s))
    env_task = sc_sim.CreateNewTask("b1EnvTask", macros.sec2nano(config.dynamics_step_s))
    dyn_process.addTask(dyn_task)
    env_process.addTask(env_task)

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "BenchmarkB1CubeSat"
    sc_object.hub.mHub = config.dry_mass_kg
    sc_object.hub.IHubPntBc_B = [
        [0.02, 0.0, 0.0],
        [0.0, 0.02, 0.0],
        [0.0, 0.0, 0.02],
    ]
    r0, v0 = initial_state_from_elements(config)
    sc_object.hub.r_CN_NInit = r0
    sc_object.hub.v_CN_NInit = v0
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth, grav_bodies = configure_gravity(config, case, sc_object, grav_factory)

    spice_object = None
    if case_uses_srp(case) or case_uses_third_body(case):
        load_spice_kernels()
        spice_object = grav_factory.createSpiceInterface(time=config.spice_epoch_utc, epochInMsg=True)
        spice_object.zeroBase = "earth"
        spice_object.referenceBase = "J2000"
        sc_sim.AddModelToTask("b1EnvTask", spice_object)

    atmosphere = None
    drag_effector = None
    if case_uses_drag(case):
        atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
        atmosphere.ModelTag = "B1Atmosphere"
        simSetPlanetEnvironment.exponentialAtmosphere(atmosphere, "earth")
        atmosphere.addSpacecraftToModel(sc_object.scStateOutMsg)

        drag_effector = dragDynamicEffector.DragDynamicEffector()
        drag_effector.ModelTag = "B1Drag"
        drag_effector.coreParams.projectedArea = config.drag_area_m2
        drag_effector.coreParams.dragCoeff = config.drag_coefficient
        drag_effector.atmoDensInMsg.subscribeTo(atmosphere.envOutMsgs[0])
        sc_object.addDynamicEffector(drag_effector)

        sc_sim.AddModelToTask("b1EnvTask", atmosphere)

    srp_effector = None
    if case_uses_srp(case):
        srp_effector = radiationPressure.RadiationPressure()
        srp_effector.ModelTag = "B1SRP"
        srp_effector.area = config.srp_area_m2
        srp_effector.coefficientReflection = config.srp_reflectivity
        srp_effector.setUseCannonballModel()
        srp_effector.sunEphmInMsg.subscribeTo(spice_object.planetStateOutMsgs[0])
        sc_object.addDynamicEffector(srp_effector)

    sample_ns = macros.sec2nano(config.output_sample_s)
    state_recorder = sc_object.scStateOutMsg.recorder(sample_ns)

    sc_sim.AddModelToTask("b1DynTask", sc_object)
    sc_sim.AddModelToTask("b1DynTask", state_recorder)

    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(macros.sec2nano(config.simulated_duration_s))

    start_s = time.perf_counter()
    sc_sim.ExecuteSimulation()
    propagation_runtime_s = time.perf_counter() - start_s

    times_s = state_recorder.times() * macros.NANO2SEC
    positions_m = np.array(state_recorder.r_BN_N)
    velocities_mps = np.array(state_recorder.v_BN_N)

    rows: list[dict[str, Any]] = []
    for time_s, position, velocity in zip(times_s, positions_m, velocities_mps):
        r_vec = np.asarray(position, dtype=float).reshape(3)
        v_vec = np.asarray(velocity, dtype=float).reshape(3)
        oe = orbitalMotion.rv2elem(MU_EARTH, r_vec, v_vec)
        arg_lat_deg = math.degrees(oe.omega + oe.f)
        lat_deg, lon_deg = eci_position_to_subpoint_lat_lon(r_vec, float(time_s))
        rows.append(
            {
                "time_s": float(time_s),
                "x_m": float(r_vec[0]),
                "y_m": float(r_vec[1]),
                "z_m": float(r_vec[2]),
                "vx_m_s": float(v_vec[0]),
                "vy_m_s": float(v_vec[1]),
                "vz_m_s": float(v_vec[2]),
                "semi_major_axis_m": float(oe.a),
                "eccentricity": float(oe.e),
                "inclination_deg": math.degrees(float(oe.i)),
                "raan_deg": math.degrees(float(oe.Omega)),
                "arg_perigee_deg": math.degrees(float(oe.omega)),
                "true_anomaly_deg": math.degrees(float(oe.f)),
                "argument_of_latitude_deg": arg_lat_deg,
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
    altitudes_km = [(row["semi_major_axis_m"] - R_EARTH) / 1000.0 for row in rows]
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
    return f"b1_basilisk_{case.value.removeprefix('b1_')}"


def write_case_outputs(
    config: B1Config,
    case: B1Case,
    rows: list[dict[str, Any]],
    spec: BasiliskForceModelSpec,
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
    save_ground_track_plot(ground_track_plot, rows, f"Basilisk B1 {case.value} ground track")
    save_altitude_plot(altitude_plot, rows, f"Basilisk B1 {case.value} altitude")

    summary = {
        "benchmark": "B1",
        "simulator": "Basilisk",
        "run_id": run_id_for_case(case),
        "case_id": case.value,
        "status": "pass",
        "epoch_utc": config.mission_epoch_utc_iso,
        "simulated_duration_s": rows[-1]["time_s"],
        "sample_count": len(rows),
        "force_model": spec.to_dict(),
        "spacecraft": {
            "dry_mass_kg": config.dry_mass_kg,
            "drag_coefficient": config.drag_coefficient,
            "srp_reflectivity": config.srp_reflectivity,
            "drag_area_m2": config.drag_area_m2,
            "srp_area_m2": config.srp_area_m2,
            "attitude_policy": "fixed inertial (default hub attitude; isotropic drag/SRP coefficients)",
        },
        "integrator": f"Basilisk default integrator, dynamics step {config.dynamics_step_s} s",
        "basilisk_version": Basilisk.__version__,
        "platform": platform.platform(),
        "runtime_s": {
            "propagation": propagation_runtime_s,
            "runtime_ratio": rows[-1]["time_s"] / propagation_runtime_s if propagation_runtime_s > 0 else None,
        },
        "drift_metrics_vs_case1": drift_metrics or {},
        "basilisk_perturbation_documentation": describe_basilisk_case(case, config),
        "artifacts": {
            "state_history_csv": str(state_csv),
            "ground_track_plot": str(ground_track_plot),
            "altitude_plot": str(altitude_plot),
        },
    }

    with (case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
    with (data_case_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)

    return summary


def merge_metrics_csv(basilisk_rows: list[dict[str, Any]]) -> None:
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

    existing_rows: list[dict[str, Any]] = []
    if METRICS_CSV.exists():
        with METRICS_CSV.open(newline="", encoding="utf-8") as handle:
            existing_rows = [
                row for row in csv.DictReader(handle) if row.get("simulator") != "Basilisk"
            ]

    METRICS_CSV.parent.mkdir(parents=True, exist_ok=True)
    with METRICS_CSV.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(existing_rows)
        writer.writerows(basilisk_rows)


def write_case5_definition_markdown(
    path: Path,
    case5_doc: dict[str, Any],
    physics_scope: dict[str, Any],
) -> None:
    lines = [
        "# Basilisk B1 Case 5 — all supported (Basilisk-specific definition)",
        "",
        "Cross-tool benchmark documents use the label **all supported** generically. "
        "This file defines exactly what that means for **Basilisk** in Benchmark B1.",
        "",
        "## Inherits from",
        "",
        f"- `{case5_doc.get('inherits_from_case', 'b1_case4_j2_drag_srp')}` (full Case 4 force stack)",
        "",
        "## Added in Case 5 vs Case 4",
        "",
        *[f"- {item}" for item in case5_doc.get("added_in_case5_vs_case4", [])],
        "",
        "## Full Basilisk force stack in Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("full_force_stack", [])],
        "",
        "## Explicitly not added in Basilisk Case 5",
        "",
        *[f"- {item}" for item in case5_doc.get("not_added_in_case5", [])],
        "",
        "## Does Basilisk B1 cover all Basilisk physics fidelity features?",
        "",
        physics_scope["conclusion"],
        "",
        "### Covered in this B1 run",
        "",
        *[f"- {item}" for item in physics_scope["covered_in_this_b1_run"]],
        "",
        "### Basilisk-native but not in B1",
        "",
        *[f"- {item}" for item in physics_scope["basilisk_native_but_not_in_b1"]],
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def write_run_log(
    path: Path,
    config: B1Config,
    summaries: list[dict[str, Any]],
    total_runtime_s: float,
) -> None:
    run_date = datetime.now(timezone.utc).strftime("%Y-%m-%d")
    lines = [
        f"# Basilisk B1 run log ({run_date})",
        "",
        "## Configuration",
        "",
        "- Parameter table: `data/benchmark_b/B1_PARAMETER_TABLE.md`",
        f"- Duration: {config.simulated_duration_s / 86400.0:.1f} days",
        f"- Dynamics step: {config.dynamics_step_s} s",
        f"- Output cadence: {config.output_sample_s} s",
        f"- Basilisk version: {Basilisk.__version__}",
        "",
        "## Cases executed",
        "",
    ]

    for summary in summaries:
        fm = summary["force_model"]
        basilisk_doc = summary.get("basilisk_perturbation_documentation", {})
        lines.extend(
            [
                f"### {summary['case_id']}",
                "",
                f"- Run ID: `{summary['run_id']}`",
                f"- Gravity: {fm['gravity_model']}",
                f"- Drag: {fm['drag_enabled']} ({fm['atmosphere_model'] or 'n/a'})",
                f"- SRP: {fm['srp_enabled']} ({fm['srp_model'] or 'n/a'})",
                f"- Third body: {', '.join(fm['third_body_bodies']) if fm['third_body_bodies'] else 'none'}",
                f"- SPICE: {fm['spice_enabled']}",
                f"- Propagation runtime: {summary['runtime_s']['propagation']:.3f} s",
                f"- Runtime ratio: {summary['runtime_s']['runtime_ratio']:.1f}",
                "",
            ]
        )
        if basilisk_doc:
            lines.extend(
                [
                    "Basilisk force models enabled:",
                    *[f"- {item}" for item in basilisk_doc.get("enabled_basilisk_force_models", [])],
                    "",
                    f"Increment vs previous case: {basilisk_doc.get('increment_vs_previous_case', 'n/a')}",
                    "",
                ]
            )
        if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value:
            case5 = basilisk_doc.get("all_supported_basilisk_definition", {})
            lines.extend(
                [
                    "**Case 5 — all supported (Basilisk-specific definition):**",
                    f"- Inherits from: `{case5.get('inherits_from_case', 'n/a')}`",
                    "- Added in Case 5 vs Case 4:",
                    *[f"  - {item}" for item in case5.get("added_in_case5_vs_case4", [])],
                    "- Not included in Basilisk Case 5:",
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

    scope = basilisk_b1_physics_scope()
    lines.extend(
        [
            "## Basilisk B1 physics scope",
            "",
            scope["conclusion"],
            "",
            "Covered in this B1 run:",
            *[f"- {item}" for item in scope["covered_in_this_b1_run"]],
            "",
            "Basilisk-native features not exercised in B1:",
            *[f"- {item}" for item in scope["basilisk_native_but_not_in_b1"]],
            "",
            "## Runtime",
            "",
            f"- Total pipeline: {total_runtime_s:.3f} s",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    config = load_b1_config_from_env()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    total_start_s = time.perf_counter()
    print("Running Basilisk B1 Physics Fidelity (Cases 1–5)...")

    histories: dict[B1Case, list[dict[str, Any]]] = {}
    pending: list[dict[str, Any]] = []

    for case in B1Case:
        print(f"  Propagating {case.value}...")
        rows, spec, propagation_runtime_s = propagate_case(config, case)
        histories[case] = rows
        pending.append(
            {
                "case": case,
                "rows": rows,
                "spec": spec,
                "propagation_runtime_s": propagation_runtime_s,
            }
        )

    case1_history = histories_from_rows(histories[B1Case.CASE1_TWOBODY])
    final_summaries: list[dict[str, Any]] = []
    metrics_rows: list[dict[str, Any]] = []

    for item in pending:
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
                "simulator": "Basilisk",
                "run_id": summary["run_id"],
                "case_id": summary["case_id"],
                "position_drift_vs_case1_m": (drift_metrics or {}).get("position_drift_vs_case1_m", 0.0),
                "ground_track_lon_drift_deg": (drift_metrics or {}).get("ground_track_lon_drift_deg", 0.0),
                "raan_change_deg": (drift_metrics or {}).get(
                    "raan_change_deg", rows[-1]["raan_deg"] - rows[0]["raan_deg"]
                ),
                "altitude_change_m": (drift_metrics or {}).get(
                    "altitude_change_m",
                    rows[-1]["semi_major_axis_m"] - rows[0]["semi_major_axis_m"],
                ),
                "along_track_phase_drift_deg": (drift_metrics or {}).get("along_track_phase_drift_deg", 0.0),
                "final_altitude_m": (drift_metrics or {}).get(
                    "final_altitude_m", rows[-1]["semi_major_axis_m"] - R_EARTH
                ),
                "wall_clock_runtime_s": summary["runtime_s"]["propagation"],
                "runtime_ratio": summary["runtime_s"]["runtime_ratio"],
                "drag_enabled": fm["drag_enabled"],
                "srp_enabled": fm["srp_enabled"],
                "third_body_enabled": bool(fm["third_body_bodies"]),
            }
        )

    merge_metrics_csv(metrics_rows)

    physics_scope = basilisk_b1_physics_scope()
    case5_summary = next(
        summary for summary in final_summaries if summary["case_id"] == B1Case.CASE5_ALL_SUPPORTED.value
    )
    case5_doc = case5_summary["basilisk_perturbation_documentation"]["all_supported_basilisk_definition"]

    aggregate_summary = {
        "benchmark": "B1",
        "simulator": "Basilisk",
        "status": "pass",
        "cases": [summary["case_id"] for summary in final_summaries],
        "summaries": final_summaries,
        "metrics_csv": str(METRICS_CSV),
        "parameter_table": "data/benchmark_b/B1_PARAMETER_TABLE.md",
        "basilisk_case5_all_supported_definition": case5_doc,
        "basilisk_b1_physics_scope": physics_scope,
        "runtime_s": {"total_pipeline": time.perf_counter() - total_start_s},
    }
    with (DATA_DIR / "basilisk_b1_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(aggregate_summary, handle, indent=2)

    write_case5_definition_markdown(
        DATA_DIR / "basilisk_b1_case5_definition.md",
        case5_doc,
        physics_scope,
    )
    write_run_log(
        DATA_DIR / "basilisk_b1_run_log.md",
        config,
        final_summaries,
        aggregate_summary["runtime_s"]["total_pipeline"],
    )

    print(f"Basilisk B1 complete. Results: {DATA_DIR}")
    print(f"Metrics CSV: {METRICS_CSV}")
    print(f"Total runtime: {aggregate_summary['runtime_s']['total_pipeline']:.3f} s")


if __name__ == "__main__":
    main()
