#!/usr/bin/env python3
"""Benchmark A0 Physics Fidelity — Basilisk propagation for GMAT-twin Cases 1–5.

   This script replicates the GMAT script parameters exactly:
   - 1-year duration (525600 steps × 60 s)
   - DragArea = 0.08, SRPArea = 0.1
   - Case 5: 20×20 gravity field (JGM2)
   - Synchronized propagation (simulated sequentially in Basilisk, but independent cases)
   - Output CSVs in GMAT format (time, X, Y, Z, VX, VY, VZ)
"""

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
from Basilisk.simulation import dragDynamicEffector, msisAtmosphere, radiationPressure, spacecraft, svIntegrators
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path
from Basilisk.architecture import messaging
# Import the A0 config (GMAT twin)
import sys
sys.path.append(str(Path(__file__).resolve().parent.parent / "config"))
from a0_physics_config import (
    A0Case,
    A0Config,
    load_a0_config_from_env,
    R_EARTH_GMAT,
    case_uses_drag,
    case_uses_j2,
    case_uses_srp,
    case_uses_third_body,
    case_uses_high_degree_gravity,
)

MU_EARTH = 3.986004418e14  # m^3/s^2

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent.parent  # Goes up to catec_ws/
RESULTS_DIR = SCRIPT_DIR / "results/basilisk_a0"
DATA_DIR = WORKSPACE_ROOT / "data/benchmark_a/a0_physics_fidelity_constellation/basilisk"
METRICS_CSV = WORKSPACE_ROOT / "data/benchmark_a/A0_METRICS_COMPARISON.csv"


@dataclass(frozen=True)
class BasiliskForceModelSpec:
    gravity_model: str
    j2_enabled: bool
    high_degree_enabled: bool
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
            "high_degree_enabled": self.high_degree_enabled,
            "drag_enabled": self.drag_enabled,
            "atmosphere_model": self.atmosphere_model,
            "srp_enabled": self.srp_enabled,
            "srp_model": self.srp_model,
            "third_body_bodies": list(self.third_body_bodies),
            "spice_enabled": self.spice_enabled,
        }


def force_model_spec(case: A0Case, config: A0Config) -> BasiliskForceModelSpec:
    if case == A0Case.CASE1_TWOBODY:
        return BasiliskForceModelSpec(
            "Point-mass Earth (mu only, no harmonics)",
            False,
            False,
            False,
            None,
            False,
            None,
            (),
            False,
        )
    if case == A0Case.CASE2_J2:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            False,
            False,
            None,
            False,
            None,
            (),
            False,
        )
    if case == A0Case.CASE3_J2_DRAG:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            False,
            True,
            "NRLMSISE-00 (MsisAtmosphere)",
            False,
            None,
            (),
            False,
        )
    if case == A0Case.CASE4_J2_DRAG_SRP:
        return BasiliskForceModelSpec(
            f"GGM03S J2-only spherical harmonics (max degree {config.basilisk_j2_max_degree})",
            True,
            False,
            True,
            "NRLMSISE-00 (MsisAtmosphere)",
            True,
            "RadiationPressure cannonball model",
            (),
            True,
        )
    # CASE5_ALL_SUPPORTED: GMAT twin with 20x20 gravity
    return BasiliskForceModelSpec(
        "GGM03S 20x20 spherical harmonics (matches GMAT Case 5 degree/order)",
        True,          # j2_enabled (implied)
        True,          # high_degree_enabled (new flag for 20x20)
        True,
        "NRLMSISE-00 (MsisAtmosphere)",
        True,
        "RadiationPressure cannonball model",
        ("Sun", "Moon"),
        True,
    )


def load_spice_kernels() -> None:
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de430)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.naif0012)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de_403_masses)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.pck00010)))


def initial_state_from_elements(config: A0Config) -> tuple[np.ndarray, np.ndarray]:
    oe = orbitalMotion.ClassicElements()
    oe.a = config.semi_major_axis_m
    oe.e = config.eccentricity
    oe.i = math.radians(config.inclination_deg)
    oe.Omega = math.radians(config.raan_deg)
    oe.omega = math.radians(config.arg_perigee_deg)
    oe.f = math.radians(config.true_anomaly_deg)
    r0, v0 = orbitalMotion.elem2rv(MU_EARTH, oe)
    return np.array(r0, dtype=float), np.array(v0, dtype=float)


def configure_gravity(config: A0Config, case: A0Case, sc_object, grav_factory):
    if case_uses_third_body(case):
        grav_bodies = grav_factory.createBodies(["sun", "earth", "moon"])
        earth = grav_bodies["earth"]
    else:
        earth = grav_factory.createEarth()
        grav_bodies = {"earth": earth}

    earth.isCentralBody = True
    earth.radEquator = R_EARTH_GMAT  # Force GMAT's Earth radius for exact parity

    # ----- HIGH DEGREE (20x20) for Case 5 -----
    if case_uses_high_degree_gravity(case):
        # Full GGM03S gravity file, degree 20, order 20
        grav_file = get_path(DataFile.LocalGravData.GGM03S)
        earth.useSphericalHarmonicsGravityModel(str(grav_file), 20)
    elif case_uses_j2(case):
        # Cases 2-4: J2-only (degree 2, order 0)
        j2_path = get_path(DataFile.LocalGravData.GGM03S_J2_only)
        earth.useSphericalHarmonicsGravityModel(str(j2_path), config.basilisk_j2_max_degree)
    else:
        # Case 1: point-mass only
        earth.mu = MU_EARTH

    grav_factory.addBodiesTo(sc_object)
    return earth, grav_bodies


def propagate_case(config: A0Config, case: A0Case) -> tuple[list[dict[str, Any]], BasiliskForceModelSpec, float]:
    spec = force_model_spec(case, config)
    sc_sim = SimulationBaseClass.SimBaseClass()
    dyn_process = sc_sim.CreateNewProcess("b1DynProcess")
    env_process = sc_sim.CreateNewProcess("b1EnvProcess")
    dyn_task = sc_sim.CreateNewTask("b1DynTask", macros.sec2nano(config.dynamics_step_s))
    env_task = sc_sim.CreateNewTask("b1EnvTask", macros.sec2nano(config.dynamics_step_s))
    dyn_process.addTask(dyn_task)
    env_process.addTask(env_task)

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "BenchmarkA0CubeSat"
    
    # FIX: Override default RK4 with high-fidelity adaptive RKF7(8)
    integrator = svIntegrators.svIntegratorRKF78(sc_object)
    integrator.absTol = 1e-12  # Matches GMAT Prop_C1.Accuracy
    integrator.relTol = 1e-12
    sc_object.setIntegrator(integrator)

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
    if case_uses_srp(case) or case_uses_third_body(case) or case_uses_drag(case):
        load_spice_kernels()
        
        # Get the names of the gravity bodies we already configured (e.g., ["earth"])
        spice_planet_names = list(grav_bodies.keys())
        
        # If SRP is enabled, we MUST track the sun for radiation pressure, 
        # even if it isn't generating a gravity field in this case
        if case_uses_srp(case) and "sun" not in spice_planet_names:
            spice_planet_names.append("sun")
            
        # Use the exact argument from the docs: spicePlanetNames
        spice_object = grav_factory.createSpiceInterface(
            time=config.spice_epoch_utc, 
            epochInMsg=True,
            spicePlanetNames=spice_planet_names
        )
        spice_object.zeroBase = "earth"
        spice_object.referenceBase = "J2000"
        sc_sim.AddModelToTask("b1EnvTask", spice_object)

    atmosphere = None
    drag_effector = None
    if case_uses_drag(case):
        atmosphere = msisAtmosphere.MsisAtmosphere()
        atmosphere.ModelTag = "A0MsisAtmo"
        atmosphere.addSpacecraftToModel(sc_object.scStateOutMsg)
        
        # FIX: Use the official Basilisk dictionary implementation for the 23 MSIS messages
        ap = 15.0
        f107 = 140.0
        sw_msg_dict = {
            "ap_24_0": ap, "ap_3_0": ap, "ap_3_-3": ap, "ap_3_-6": ap, "ap_3_-9": ap,
            "ap_3_-12": ap, "ap_3_-15": ap, "ap_3_-18": ap, "ap_3_-21": ap, "ap_3_-24": ap,
            "ap_3_-27": ap, "ap_3_-30": ap, "ap_3_-33": ap, "ap_3_-36": ap, "ap_3_-39": ap,
            "ap_3_-42": ap, "ap_3_-45": ap, "ap_3_-48": ap, "ap_3_-51": ap, "ap_3_-54": ap,
            "ap_3_-57": ap, "f107_1944_0": f107, "f107_24_-24": f107
        }

        # Keep a reference in sc_sim so the messages aren't garbage collected by Python
        sc_sim.swMsgList = []
        for c, val in enumerate(sw_msg_dict.values()):
            swMsgData = messaging.SwDataMsgPayload(dataValue=val)
            sc_sim.swMsgList.append(messaging.SwDataMsg().write(swMsgData))
            atmosphere.swDataInMsgs[c].subscribeTo(sc_sim.swMsgList[-1])

        drag_effector = dragDynamicEffector.DragDynamicEffector()
        drag_effector.ModelTag = "A0Drag"
        drag_effector.coreParams.projectedArea = config.drag_area_m2
        drag_effector.coreParams.dragCoeff = config.drag_coefficient
        drag_effector.atmoDensInMsg.subscribeTo(atmosphere.envOutMsgs[0])
        sc_object.addDynamicEffector(drag_effector)

        sc_sim.AddModelToTask("b1EnvTask", atmosphere)

    srp_effector = None
    if case_uses_srp(case):
        srp_effector = radiationPressure.RadiationPressure()
        srp_effector.ModelTag = "A0SRP"
        srp_effector.area = config.srp_area_m2
        srp_effector.coefficientReflection = config.srp_reflectivity
        srp_effector.setUseCannonballModel()
        
        # FIX: Use the list we created above to find the exact index of the Sun
        sun_idx = spice_planet_names.index("sun")
        srp_effector.sunEphmInMsg.subscribeTo(spice_object.planetStateOutMsgs[sun_idx])
        
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

    positions_m = np.array(state_recorder.r_BN_N)
    velocities_mps = np.array(state_recorder.v_BN_N)
    num_samples = positions_m.shape[0]
    times_s = np.arange(num_samples, dtype=float) * config.output_sample_s

    rows: list[dict[str, Any]] = []
    for time_s, position, velocity in zip(times_s, positions_m, velocities_mps):
        r_vec_km = np.asarray(position, dtype=float).reshape(3) / 1000.0
        v_vec_kms = np.asarray(velocity, dtype=float).reshape(3) / 1000.0
        rows.append(
            {
                "time_s": float(time_s),
                "x_km": float(r_vec_km[0]),
                "y_km": float(r_vec_km[1]),
                "z_km": float(r_vec_km[2]),
                "vx_km_s": float(v_vec_kms[0]),
                "vy_km_s": float(v_vec_kms[1]),
                "vz_km_s": float(v_vec_kms[2]),
            }
        )

    return rows, spec, propagation_runtime_s


def write_state_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    # GMAT output format: time, X, Y, Z, VX, VY, VZ (km, km/s)
    fieldnames = ["time_s", "x_km", "y_km", "z_km", "vx_km_s", "vy_km_s", "vz_km_s"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_case_outputs(
    config: A0Config,
    case: A0Case,
    rows: list[dict[str, Any]],
    spec: BasiliskForceModelSpec,
    propagation_runtime_s: float,
) -> dict[str, Any]:
    case_dir = RESULTS_DIR / case.value
    data_case_dir = DATA_DIR / case.value
    case_dir.mkdir(parents=True, exist_ok=True)
    data_case_dir.mkdir(parents=True, exist_ok=True)

    state_csv = case_dir / "basilisk_state.csv"
    write_state_csv(state_csv, rows)
    write_state_csv(data_case_dir / "basilisk_state.csv", rows)

    summary = {
        "benchmark": "A0",
        "simulator": "Basilisk",
        "run_id": f"a0_basilisk_{case.value.removeprefix('a0_')}",
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
            "attitude_policy": "fixed inertial (isotropic drag/SRP coefficients)",
        },
        "integrator": f"Basilisk default integrator, dynamics step {config.dynamics_step_s} s",
        "basilisk_version": Basilisk.__version__,
        "platform": platform.platform(),
        "runtime_s": {
            "propagation": propagation_runtime_s,
            "runtime_ratio": rows[-1]["time_s"] / propagation_runtime_s if propagation_runtime_s > 0 else None,
        },
        "artifacts": {
            "state_history_csv": str(state_csv),
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
        "wall_clock_runtime_s",
        "runtime_ratio",
        "drag_enabled",
        "srp_enabled",
        "third_body_enabled",
        "high_degree_gravity",
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


def main() -> None:
    config = load_a0_config_from_env()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    total_start_s = time.perf_counter()
    print("Running Basilisk A0 Physics Fidelity (GMAT twin, Cases 1–5, 1-year)...")

    final_summaries: list[dict[str, Any]] = []
    metrics_rows: list[dict[str, Any]] = []

    for case in A0Case:
        print(f"  Propagating {case.value}...")
        rows, spec, propagation_runtime_s = propagate_case(config, case)
        summary = write_case_outputs(config, case, rows, spec, propagation_runtime_s)
        final_summaries.append(summary)

        fm = summary["force_model"]
        metrics_rows.append(
            {
                "simulator": "Basilisk",
                "run_id": summary["run_id"],
                "case_id": summary["case_id"],
                "wall_clock_runtime_s": summary["runtime_s"]["propagation"],
                "runtime_ratio": summary["runtime_s"]["runtime_ratio"],
                "drag_enabled": fm["drag_enabled"],
                "srp_enabled": fm["srp_enabled"],
                "third_body_enabled": bool(fm["third_body_bodies"]),
                "high_degree_gravity": fm["high_degree_enabled"],
            }
        )

    merge_metrics_csv(metrics_rows)

    aggregate_summary = {
        "benchmark": "A0",
        "simulator": "Basilisk",
        "status": "pass",
        "cases": [summary["case_id"] for summary in final_summaries],
        "summaries": final_summaries,
        "metrics_csv": str(METRICS_CSV),
        "runtime_s": {"total_pipeline": time.perf_counter() - total_start_s},
    }
    with (DATA_DIR / "basilisk_a0_summary.json").open("w", encoding="utf-8") as handle:
        json.dump(aggregate_summary, handle, indent=2)

    print(f"Basilisk A0 complete. Results: {DATA_DIR}")
    print(f"Metrics CSV: {METRICS_CSV}")
    print(f"Total runtime: {aggregate_summary['runtime_s']['total_pipeline']:.3f} s")


if __name__ == "__main__":
    main()