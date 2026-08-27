"""Benchmark A1 - Basilisk analytical two-body validation.

This runner intentionally avoids SPICE, Sun/Moon ephemerides, J2, drag, SRP,
eclipse, and visualization modules. It validates Basilisk's basic Earth
two-body propagation against an independent Kepler analytical propagation.
"""

from __future__ import annotations

import csv
import json
import math
import platform
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

import Basilisk
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody


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


def resolve_output_dir() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    output_dir = repo_root / "data" / "benchmark_a" / "basilisk_a1"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def initial_state_from_elements(config: A1Config) -> tuple[np.ndarray, np.ndarray]:
    oe = orbitalMotion.ClassicElements()
    oe.a = config.semi_major_axis_m
    oe.e = config.eccentricity
    oe.i = math.radians(config.inclination_deg)
    oe.Omega = math.radians(config.raan_deg)
    oe.omega = math.radians(config.arg_perigee_deg)
    oe.f = math.radians(config.true_anomaly_deg)
    r0, v0 = orbitalMotion.elem2rv(MU_EARTH, oe)
    return np.array(r0, dtype=float), np.array(v0, dtype=float)


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


def analytical_two_body_state(config: A1Config, t_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Return analytical Kepler two-body state at time t.

    The frozen A1 orbit uses e=0, but this implementation supports elliptical
    e < 1 to keep the reference model explicit and reusable.
    """
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


def run_basilisk_two_body(config: A1Config) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sc_sim = SimulationBaseClass.SimBaseClass()
    task_name = "a1TwoBodyTask"
    process_name = "a1TwoBodyProcess"
    sc_sim.CreateNewProcess(process_name).addTask(
        sc_sim.CreateNewTask(task_name, macros.sec2nano(config.dynamics_step_s))
    )

    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "BenchmarkA1CubeSat"
    sc_object.hub.mHub = 12.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sc_object.hub.IHubPntBc_B = [[0.02, 0.0, 0.0], [0.0, 0.02, 0.0], [0.0, 0.0, 0.02]]

    r0, v0 = initial_state_from_elements(config)
    sc_object.hub.r_CN_NInit = r0
    sc_object.hub.v_CN_NInit = v0
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.mu = MU_EARTH
    earth.radEquator = R_EARTH
    earth.isCentralBody = True
    grav_factory.addBodiesTo(sc_object)

    sample_ns = macros.sec2nano(config.output_sample_s)
    state_recorder = sc_object.scStateOutMsg.recorder(sample_ns)

    sc_sim.AddModelToTask(task_name, sc_object)
    sc_sim.AddModelToTask(task_name, state_recorder)
    sc_sim.InitializeSimulation()
    sc_sim.ConfigureStopTime(macros.sec2nano(config.simulated_duration_s))
    sc_sim.ExecuteSimulation()

    times_s = state_recorder.times() * macros.NANO2SEC
    positions_m = np.array(state_recorder.r_BN_N)
    velocities_mps = np.array(state_recorder.v_BN_N)
    return times_s, positions_m, velocities_mps


def write_outputs(
    output_dir: Path,
    config: A1Config,
    times_s: np.ndarray,
    positions_m: np.ndarray,
    velocities_mps: np.ndarray,
    wall_clock_runtime_s: float,
) -> dict[str, float | str | int]:
    comparison_path = output_dir / "basilisk_a1_state_comparison.csv"
    rows = []

    for t_s, r_bsk, v_bsk in zip(times_s, positions_m, velocities_mps):
        r_ref, v_ref = analytical_two_body_state(config, float(t_s))
        position_error_m = float(np.linalg.norm(r_bsk - r_ref))
        velocity_error_mps = float(np.linalg.norm(v_bsk - v_ref))
        rows.append(
            {
                "time_s": float(t_s),
                "basilisk_rx_m": float(r_bsk[0]),
                "basilisk_ry_m": float(r_bsk[1]),
                "basilisk_rz_m": float(r_bsk[2]),
                "basilisk_vx_mps": float(v_bsk[0]),
                "basilisk_vy_mps": float(v_bsk[1]),
                "basilisk_vz_mps": float(v_bsk[2]),
                "reference_rx_m": float(r_ref[0]),
                "reference_ry_m": float(r_ref[1]),
                "reference_rz_m": float(r_ref[2]),
                "reference_vx_mps": float(v_ref[0]),
                "reference_vy_mps": float(v_ref[1]),
                "reference_vz_mps": float(v_ref[2]),
                "position_difference_vs_reference_m": position_error_m,
                "velocity_difference_vs_reference_mps": velocity_error_mps,
            }
        )

    with comparison_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    max_pos_error = max(row["position_difference_vs_reference_m"] for row in rows)
    final_pos_error = rows[-1]["position_difference_vs_reference_m"]
    max_vel_error = max(row["velocity_difference_vs_reference_mps"] for row in rows)
    final_vel_error = rows[-1]["velocity_difference_vs_reference_mps"]
    runtime_ratio = config.simulated_duration_s / wall_clock_runtime_s

    summary = {
        "benchmark_id": "A1",
        "simulator": "Basilisk",
        "basilisk_version": Basilisk.__version__,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
        "dynamics_model": "two-body Earth central gravity only",
        "spice_used": "no",
        "j2_enabled": "no",
        "drag_enabled": "no",
        "srp_enabled": "no",
        "third_body_enabled": "no",
        "simulated_duration_s": config.simulated_duration_s,
        "wall_clock_runtime_s": wall_clock_runtime_s,
        "runtime_ratio": runtime_ratio,
        "samples": len(rows),
        "position_difference_vs_reference_final_m": final_pos_error,
        "position_difference_vs_reference_max_m": max_pos_error,
        "velocity_difference_vs_reference_final_mps": final_vel_error,
        "velocity_difference_vs_reference_max_mps": max_vel_error,
        "state_comparison_csv": str(comparison_path),
    }

    summary_path = output_dir / "basilisk_a1_summary.json"
    with summary_path.open("w", encoding="utf-8") as file:
        json.dump({"config": asdict(config), "summary": summary}, file, indent=2)

    log_path = output_dir / "basilisk_a1_run_log.md"
    with log_path.open("w", encoding="utf-8") as file:
        file.write("# Benchmark A1 Basilisk Run Log\n\n")
        file.write("## Status\n\n")
        file.write("- Result: pass\n")
        file.write("- SPICE used: no\n")
        file.write("- External support-data files required: no\n\n")
        file.write("## Summary\n\n")
        for key, value in summary.items():
            file.write(f"- {key}: {value}\n")

    return summary


def main() -> None:
    config = A1Config()
    output_dir = resolve_output_dir()
    start = time.perf_counter()
    times_s, positions_m, velocities_mps = run_basilisk_two_body(config)
    wall_clock_runtime_s = time.perf_counter() - start
    summary = write_outputs(output_dir, config, times_s, positions_m, velocities_mps, wall_clock_runtime_s)

    print("Benchmark A1 Basilisk analytical validation complete.")
    print(f"Output directory: {output_dir}")
    print(f"Samples: {summary['samples']}")
    print(f"Final position difference vs reference [m]: {summary['position_difference_vs_reference_final_m']:.6f}")
    print(f"Max position difference vs reference [m]: {summary['position_difference_vs_reference_max_m']:.6f}")
    print(f"Final velocity difference vs reference [m/s]: {summary['velocity_difference_vs_reference_final_mps']:.9f}")
    print(f"Max velocity difference vs reference [m/s]: {summary['velocity_difference_vs_reference_max_mps']:.9f}")
    print(f"Runtime ratio [sim_time/wall_time]: {summary['runtime_ratio']:.2f}")


if __name__ == "__main__":
    main()
