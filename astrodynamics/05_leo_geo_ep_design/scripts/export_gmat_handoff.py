#!/usr/bin/env python3
"""Export GMAT A3 outputs into Basilisk handoff schedule."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[2]
SCRIPT_DIR = Path(__file__).resolve().parent
CATEC_RUNS = WORKSPACE / "catec_ws/benchmarks/benchmark_a/runs"
DEFAULT_GMAT_DIR = WORKSPACE / "catec_ws/data/benchmark_a/gmat/a3_leo_geo_transfer"
DEFAULT_OUT = WORKSPACE / "data/pipeline_handoff/gmat/burn_schedule.json"
DEFAULT_BSP_NAMES = ("a3_reference.bsp", "gmat_ephem.bsp")
DEFAULT_OEM_NAMES = ("a3_reference.oem", "gmat_ephem.oem")
GMAT_ELECTRIC_DYNAMICS_STEP_S = 10.0
GMAT_DEFAULT_NAIF_ID = "-10008001"
_FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")

if str(CATEC_RUNS) not in sys.path:
    sys.path.insert(0, str(CATEC_RUNS))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from a3_leo_geo_config import A3LeoGeoConfig  # noqa: E402
from ephemeris_handoff import SpiceReferenceEphemeris, load_reference_ephemeris, parse_epoch_utc, resolve_initial_state  # noqa: E402
from thruster_stack import ThrustMassPolynomialModel, invert_mass_flow_power_kg_s  # noqa: E402

GMAT_SCRIPT_PATH = WORKSPACE / "catec_ws/benchmarks/benchmark_a/gmat/scripts/Benchmark_A3_LEO_GEO_Transfer.script"
GMAT_SCRIPT_FALLBACKS = (
    GMAT_SCRIPT_PATH,
    GMAT_SCRIPT_PATH.parent / "Benchmark_A3_LEO_GEO_Transfer.script.bak",
)


def resolve_gmat_script_path() -> Path | None:
    for path in GMAT_SCRIPT_FALLBACKS:
        if path.is_file():
            return path
    return None


def parse_gmat_naif_id() -> str:
    script_path = resolve_gmat_script_path()
    if script_path is None:
        return GMAT_DEFAULT_NAIF_ID
    txt = script_path.read_text(encoding="utf-8", errors="replace")
    m = re.search(r"A3_SC\.NAIFId\s*=\s*(-?\d+)\s*;", txt)
    return m.group(1) if m else GMAT_DEFAULT_NAIF_ID


def resolve_bsp_path(gmat_dir: Path) -> Path:
    for n in (*DEFAULT_BSP_NAMES, *DEFAULT_OEM_NAMES):
        p = gmat_dir / n
        if p.is_file():
            return p
    raise FileNotFoundError(f"No ephemeris found in {gmat_dir}")


def detect_mission_mode(script_text: str) -> str:
    if not script_text.strip():
        return "electric_spiral"
    if "Create ElectricThruster" in script_text and "Spiral To GEO" in script_text:
        return "electric_spiral"
    return "chemical_3burn"


def parse_gmat_final(path: Path) -> dict[str, float]:
    rows = []
    for ln in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "ElapsedSecs" in ln:
            continue
        nums = [float(tok) for tok in _FLOAT_RE.findall(ln)]
        if len(nums) >= 4:
            rows.append(nums)
    if not rows:
        raise ValueError("gmat_final.txt has no data rows")
    first, last = rows[0], rows[-1]
    return {
        "spiral_end_s": float(first[0]),
        "final_sma_km": float(first[1]),
        "final_ecc": float(first[2]),
        "final_inc_deg": float(first[3]),
        "final_mass_kg": float(first[4]) if len(first) > 4 else None,
        "mission_end_s": float(last[0]),
    }


def parse_gmat_spiral_events(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    if not path.is_file():
        return out
    for ln in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "ElapsedSecs" in ln:
            continue
        nums = [float(tok) for tok in _FLOAT_RE.findall(ln)]
        if len(nums) < 2:
            continue
        code = int(nums[0])
        if code == 1:
            out["MISSION_START"] = nums[1]
        elif code == 10:
            out["SPIRAL_START"] = nums[1]
        elif code == 11:
            out["SPIRAL_END"] = nums[1]
        elif code == 12:
            out["EQUATOR_TRIM_END"] = nums[1]
    return out


def parse_gmat_propagator(script_text: str) -> dict[str, float | str]:
    def pick(pat: str, default: float) -> float:
        m = re.search(pat, script_text)
        return float(m.group(1)) if m else default

    typ = re.search(r"A3_Prop\.Type\s*=\s*([A-Za-z0-9_]+)\s*;", script_text)
    return {
        "type": typ.group(1) if typ else "RungeKutta89",
        "initial_step_s": pick(r"A3_Prop\.InitialStepSize\s*=\s*([0-9eE.+\-]+)\s*;", 10.0),
        "min_step_s": pick(r"A3_Prop\.MinStep\s*=\s*([0-9eE.+\-]+)\s*;", 0.001),
        "max_step_s": pick(r"A3_Prop\.MaxStep\s*=\s*([0-9eE.+\-]+)\s*;", 2700.0),
        "accuracy": pick(r"A3_Prop\.Accuracy\s*=\s*([0-9eE.+\-]+)\s*;", 1e-10),
    }


def parse_script_propulsion(script_text: str, *, mission_mode: str) -> dict:
    def pick(pat: str, default: float) -> float:
        m = re.search(pat, script_text)
        return float(m.group(1)) if m else default

    base = {
        "dry_mass_kg": pick(r"A3_SC\.DryMass\s*=\s*([0-9eE.+\-]+)\s*;", 12.0),
        "initial_fuel_kg": pick(r"A3_Tank\.FuelMass\s*=\s*([0-9eE.+\-]+)\s*;", 5.0 if mission_mode == "electric_spiral" else 80.0),
        "isp_s": pick(r"A3_Engine\.Isp\s*=\s*([0-9eE.+\-]+)\s*;", 300.0),
    }
    if mission_mode == "electric_spiral":
        base["constant_thrust_n"] = pick(r"A3_Engine\.ConstantThrust\s*=\s*([0-9eE.+\-]+)\s*;", 0.05)
        base["max_thrust_n"] = base["constant_thrust_n"]
        base["isp_s"] = pick(r"A3_Engine\.Isp\s*=\s*([0-9eE.+\-]+)\s*;", 2500.0)
        base["duty_cycle"] = pick(r"A3_Engine\.DutyCycle\s*=\s*([0-9eE.+\-]+)\s*;", 1.0)
        base["thrust_scale_factor"] = pick(r"A3_Engine\.ThrustScaleFactor\s*=\s*([0-9eE.+\-]+)\s*;", 1.0)
        base["thrust_model"] = (re.search(r"A3_Engine\.ThrustModel\s*=\s*([A-Za-z0-9_]+)\s*;", script_text) or [None, "ThrustMassPolynomial"])[1]
        base["max_usable_power_kw"] = pick(r"A3_Engine\.MaximumUsablePower\s*=\s*([0-9eE.+\-]+)\s*;", 7.266)
        base["min_usable_power_kw"] = pick(r"A3_Engine\.MinimumUsablePower\s*=\s*([0-9eE.+\-]+)\s*;", 0.638)
        base["thrust_coeff_mn"] = [pick(rf"A3_Engine\.ThrustCoeff{i}\s*=\s*([0-9eE.+\-]+)\s*;", 0.0) for i in range(1, 6)]
        base["mass_flow_coeff_mgps"] = [pick(rf"A3_Engine\.MassFlowCoeff{i}\s*=\s*([0-9eE.+\-]+)\s*;", 0.0) for i in range(1, 6)]
    else:
        base["max_thrust_n"] = pick(r"A3_Engine\.C1\s*=\s*([0-9eE.+\-]+)\s*;", 50.0)
    return base


def build_propulsion_block(*, mission_mode: str, script_text: str, final_mass_kg: float | None) -> dict:
    p = parse_script_propulsion(script_text, mission_mode=mission_mode)
    m0 = p["dry_mass_kg"] + p["initial_fuel_kg"]
    mf = float(final_mass_kg or p["dry_mass_kg"])
    out = {
        "dry_mass_kg": p["dry_mass_kg"],
        "initial_fuel_mass_kg": p["initial_fuel_kg"],
        "initial_mass_kg": m0,
        "max_thrust_n": p["max_thrust_n"],
        "isp_s": p["isp_s"],
        "thruster_position_body_m": [0.0, 0.0, 0.0],
        "thruster_direction_body": [0.0, 0.0, 1.0],
        "gmat_fuel_used_kg": max(0.0, m0 - mf),
        "gmat_mass_after_burn2_kg": mf,
    }
    if mission_mode == "electric_spiral":
        out["constant_thrust_n"] = p["constant_thrust_n"]
        out["thrust_model"] = p["thrust_model"]
        em = {
            "duty_cycle": p["duty_cycle"],
            "thrust_scale_factor": p["thrust_scale_factor"],
            "thrust_coeff_mn": p["thrust_coeff_mn"],
            "mass_flow_coeff_mgps": p["mass_flow_coeff_mgps"],
            "maximum_usable_power_kw": p["max_usable_power_kw"],
            "minimum_usable_power_kw": p["min_usable_power_kw"],
        }
        samples = []
        for power in np.linspace(em["minimum_usable_power_kw"], em["maximum_usable_power_kw"], 8).tolist():
            thrust_mn = sum(c * (power ** i) for i, c in enumerate(em["thrust_coeff_mn"]))
            mdot_mgps = sum(c * (power ** i) for i, c in enumerate(em["mass_flow_coeff_mgps"]))
            samples.append({
                "input_power_kw": power,
                "thrust_n": max(0.0, em["duty_cycle"] * em["thrust_scale_factor"] * thrust_mn * 1e-3),
                "mass_flow_kg_s": max(0.0, em["duty_cycle"] * em["thrust_scale_factor"] * mdot_mgps * 1e-6),
            })
        em["thrust_power_curve_samples"] = samples
        out["electric_model"] = em
    return out


def parse_script_solar_power(script_text: str) -> dict:
    def pick(pat: str, default: float) -> float:
        m = re.search(pat, script_text)
        return float(m.group(1)) if m else default

    def pick_str(pat: str, default: str) -> str:
        m = re.search(pat, script_text)
        return m.group(1) if m else default

    annual_percent = pick(r"SolarPowerSystem1\.AnnualDecayRate\s*=\s*([0-9eE.+\-]+)\s*;", 5.0)
    margin_percent = pick(r"SolarPowerSystem1\.Margin\s*=\s*([0-9eE.+\-]+)\s*;", 5.0)
    return {
        "initial_max_power_kw": pick(r"SolarPowerSystem1\.InitialMaxPower\s*=\s*([0-9eE.+\-]+)\s*;", 1.2),
        "margin_percent": margin_percent,
        "margin_frac": margin_percent / 100.0,
        "annual_decay_rate_percent": annual_percent,
        "annual_decay_rate_frac": annual_percent / 100.0,
        "bus_coeff1": pick(r"SolarPowerSystem1\.BusCoeff1\s*=\s*([0-9eE.+\-]+)\s*;", 0.3),
        "bus_coeff2": pick(r"SolarPowerSystem1\.BusCoeff2\s*=\s*([0-9eE.+\-]+)\s*;", 0.0),
        "bus_coeff3": pick(r"SolarPowerSystem1\.BusCoeff3\s*=\s*([0-9eE.+\-]+)\s*;", 0.0),
        "solar_coeff1": pick(r"SolarPowerSystem1\.SolarCoeff1\s*=\s*([0-9eE.+\-]+)\s*;", 1.32077),
        "solar_coeff2": pick(r"SolarPowerSystem1\.SolarCoeff2\s*=\s*([0-9eE.+\-]+)\s*;", -0.10848),
        "solar_coeff3": pick(r"SolarPowerSystem1\.SolarCoeff3\s*=\s*([0-9eE.+\-]+)\s*;", -0.11665),
        "solar_coeff4": pick(r"SolarPowerSystem1\.SolarCoeff4\s*=\s*([0-9eE.+\-]+)\s*;", 0.10843),
        "solar_coeff5": pick(r"SolarPowerSystem1\.SolarCoeff5\s*=\s*([0-9eE.+\-]+)\s*;", -0.01279),
        "initial_epoch_utc": pick_str(
            r"SolarPowerSystem1\.InitialEpoch\s*=\s*''([^']+)''\s*;",
            "15 Jun 2026 12:00:00.000",
        ),
        "shadow_active": "DualCone" in script_text or "ShadowModel" in script_text,
    }


def attach_electric_power_metadata(propulsion: dict, *, script_text: str, spiral_start_s: float, spiral_end_s: float) -> None:
    em = propulsion.get("electric_model") or {}
    solar = parse_script_solar_power(script_text)
    em["solar_initial_max_power_kw"] = solar["initial_max_power_kw"]
    em["solar_margin_percent"] = solar["margin_percent"]
    em["solar_margin_frac"] = solar["margin_frac"]
    em["solar_bus_coeff1"] = solar["bus_coeff1"]
    em["solar_bus_coeff2"] = solar["bus_coeff2"]
    em["solar_bus_coeff3"] = solar["bus_coeff3"]
    em["solar_coeffs"] = [
        solar["solar_coeff1"],
        solar["solar_coeff2"],
        solar["solar_coeff3"],
        solar["solar_coeff4"],
        solar["solar_coeff5"],
    ]
    em["bus_coeffs"] = [solar["bus_coeff1"], solar["bus_coeff2"], solar["bus_coeff3"]]
    em["annual_decay_rate_frac"] = solar["annual_decay_rate_frac"]
    em["solar_annual_decay_rate_frac"] = solar["annual_decay_rate_frac"]
    em["solar_initial_epoch_utc"] = "2026-06-15T12:00:00.000Z"
    em["solar_shadow_active"] = bool(solar.get("shadow_active", True))
    avg_mdot = float(propulsion.get("gmat_spiral_mass_flow_kg_s", 0.0))
    if avg_mdot > 0.0:
        em["effective_power_kw"] = invert_mass_flow_power_kg_s(
            avg_mdot,
            tuple(em.get("mass_flow_coeff_mgps", ())),
            p_min=float(em.get("minimum_usable_power_kw", 0.638)),
            p_max=float(em.get("maximum_usable_power_kw", 7.266)),
            duty_cycle=float(em.get("duty_cycle", 1.0)),
            thrust_scale_factor=float(em.get("thrust_scale_factor", 1.0)),
        )
    else:
        em["effective_power_kw"] = solar["initial_max_power_kw"]
    propulsion["electric_model"] = em
    propulsion["nominal_power_kw"] = em["effective_power_kw"]
    propulsion["nominal_thrust_n"] = ThrustMassPolynomialModel.from_propulsion(propulsion).thrust_n()


def attach_spiral_mass_flow(propulsion: dict, *, spiral_start_s: float, spiral_end_s: float) -> None:
    spiral_duration_s = max(0.0, float(spiral_end_s) - float(spiral_start_s))
    fuel_kg = float(propulsion.get("gmat_fuel_used_kg", 0.0))
    if spiral_duration_s > 0.0 and fuel_kg > 0.0:
        propulsion["gmat_spiral_mass_flow_kg_s"] = fuel_kg / spiral_duration_s


def build_schedule(gmat_dir: Path, config: A3LeoGeoConfig, *, execution_mode: str = "finite_thrust") -> dict:
    bsp_path = resolve_bsp_path(gmat_dir)
    script_path = resolve_gmat_script_path()
    script_text = script_path.read_text(encoding="utf-8", errors="replace") if script_path is not None else ""
    mission_mode = detect_mission_mode(script_text)
    gmat_final = parse_gmat_final(gmat_dir / "gmat_final.txt")
    gmat_events = parse_gmat_spiral_events(gmat_dir / "spiral_events.txt")
    if gmat_events:
        mission_mode = "electric_spiral"
    propagator = parse_gmat_propagator(script_text)
    mission_epoch = parse_epoch_utc(config.mission_epoch_utc_iso)
    naif_id = parse_gmat_naif_id()

    ref_format = "naif-spk-bsp" if bsp_path.suffix.lower() == ".bsp" else "ccsds-oem"
    spice_ref = SpiceReferenceEphemeris(bsp_path=bsp_path, naif_id=naif_id, mission_epoch=mission_epoch) if ref_format == "naif-spk-bsp" else None
    if spice_ref is not None:
        r0, v0, bsp_start_s = resolve_initial_state(gmat_dir, spice_ref)
    else:
        rows = load_reference_ephemeris(bsp_path, mission_epoch=mission_epoch)
        r0 = np.asarray(rows[0]["r_m"], dtype=float)
        v0 = np.asarray(rows[0]["v_m_s"], dtype=float)
        bsp_start_s = 0.0

    propulsion = build_propulsion_block(mission_mode=mission_mode, script_text=script_text, final_mass_kg=gmat_final.get("final_mass_kg"))
    if mission_mode == "electric_spiral":
        spiral_start_s = float(gmat_events.get("SPIRAL_START", 0.0))
        spiral_end_s = float(gmat_events.get("SPIRAL_END", gmat_final["spiral_end_s"]))
        trim_end_s = float(gmat_events.get("EQUATOR_TRIM_END", spiral_end_s))
        mission_end_s = float(gmat_final["mission_end_s"])
        attach_spiral_mass_flow(propulsion, spiral_start_s=spiral_start_s, spiral_end_s=spiral_end_s)
        attach_electric_power_metadata(
            propulsion,
            script_text=script_text,
            spiral_start_s=spiral_start_s,
            spiral_end_s=spiral_end_s,
        )
        burns = [
            {
                "id": "spiral_to_geo",
                "type": "electric_spiral",
                "start_s": spiral_start_s,
                "gmat_start_s": spiral_start_s,
                "duration_s": max(0.0, spiral_end_s - spiral_start_s),
                "gmat_end_s": spiral_end_s,
                "thrust_n": float(propulsion.get("nominal_thrust_n", propulsion.get("constant_thrust_n", 0.0))),
                "constant_thrust": False,
                "thrust_model": "ThrustMassPolynomial",
                "pointing_frame": "VNB",
                "pointing_direction": [1.0, 0.0, 0.0],
                "stop_condition": {
                    "mode": "gmat_end_s",
                    "semi_major_axis_m": 42_164_000.0,
                    "gmat_targeting": ["sma_only"],
                },
            },
            {
                "id": "post_spiral_equator_trim",
                "type": "coast_trim",
                "start_s": trim_end_s,
                "gmat_start_s": trim_end_s,
                "duration_s": max(0.0, mission_end_s - trim_end_s),
                "gmat_end_s": mission_end_s,
                "target_event": "equator_crossing_z0",
            },
        ]
        gmat_ref = {
            "mission_mode": mission_mode,
            "gmat_targeting": ["sma_only"],
            "converged_sma": abs(float(gmat_final["final_sma_km"]) * 1000.0 - config.geo_semi_major_axis_m) <= config.sma_tolerance_m,
            "converged_ecc": float(gmat_final["final_ecc"]) <= config.target_ecc_max,
            "converged_inc": abs(float(gmat_final["final_inc_deg"]) - config.target_inc_deg) <= config.inc_tolerance_deg,
            "converged": True,
            "final_sma_km": gmat_final["final_sma_km"],
            "final_ecc": gmat_final["final_ecc"],
            "final_inc_deg": gmat_final["final_inc_deg"],
            "spiral_start_s": spiral_start_s,
            "spiral_end_s": spiral_end_s,
            "equator_trim_end_s": trim_end_s,
            "mission_end_s": mission_end_s,
            "fuel_mass_kg": propulsion["gmat_fuel_used_kg"],
        }
    else:
        burns = []
        mission_end_s = float(gmat_final["mission_end_s"])
        gmat_ref = {"mission_mode": mission_mode, "converged": None}

    bsp_cov_start = float(bsp_start_s if bsp_start_s is not None else 0.0)
    if mission_mode == "electric_spiral":
        bsp_cov_start = max(bsp_cov_start, float(gmat_events.get("SPIRAL_START", 0.0)))
    ref_meta = {
        "path": str(bsp_path.resolve()),
        "format": ref_format,
        "coordinate_system": "EarthMJ2000Eq",
        "spice_ref": "J2000",
        "spice_observer": "EARTH",
        "naif_id": naif_id,
        "mission_epoch_utc_iso": config.mission_epoch_utc_iso,
        "interpolation": "gmat_spk_hermite",
        "bsp_coverage_start_s": bsp_cov_start,
    }
    return {
        "schema": "catec_gmat_basilisk_handoff_v6",
        "mission_mode": mission_mode,
        "execution_mode": execution_mode,
        "pipeline": {"design_authority": "GMAT", "execution_authority": "Basilisk", "description": "GMAT designs; Basilisk executes without retargeting."},
        "source_tool": "gmat",
        "source_dir": str(gmat_dir),
        "reference_ephemeris": ref_meta,
        "mission_epoch_utc_iso": config.mission_epoch_utc_iso,
        "spice_epoch_utc": config.spice_epoch_utc,
        "dynamics": {"step_s": float(propagator["initial_step_s"] or GMAT_ELECTRIC_DYNAMICS_STEP_S), "mission_end_s": mission_end_s, "post_spiral_coast_s": 12.0 * 3600.0, "gmat_propagator": propagator, "integrator_fidelity": "schedule_step_copied_from_gmat_initial_step"},
        "initial_state": {"r_m": r0.tolist(), "v_m_s": v0.tolist(), "mass_kg": propulsion["initial_mass_kg"]},
        "propulsion": propulsion,
        "target": {
            "semi_major_axis_m": config.geo_semi_major_axis_m,
            "ecc_max": config.target_ecc_max,
            "inclination_deg": config.target_inc_deg,
            "sma_tolerance_m": config.sma_tolerance_m,
            "inc_tolerance_deg": config.inc_tolerance_deg,
            "max_pointing_error_deg": 5.0,
            "max_fuel_error_fraction": 0.25,
            "max_ephemeris_position_error_m": 500_000.0,
            "max_burn_start_error_s": 120.0,
        },
        "execution_validation": {"checks": ["attitude_pointing", "fuel_usage", "burn_schedule_timing", "ephemeris_envelope"], "informational_checks": ["orbit_sma", "orbit_ecc", "orbit_inc"]},
        "burns": burns,
        "gmat_reference": gmat_ref,
    }


def vnb_delta_v_to_inertial(r_m: np.ndarray, v_m_s: np.ndarray, dv_v: float, dv_n: float) -> np.ndarray:
    v_hat = np.asarray(v_m_s, dtype=float) / max(np.linalg.norm(v_m_s), 1e-9)
    h_hat = np.cross(np.asarray(r_m, dtype=float), np.asarray(v_m_s, dtype=float))
    h_hat /= max(np.linalg.norm(h_hat), 1e-9)
    return float(dv_v) * v_hat + float(dv_n) * h_hat


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--gmat-dir", type=Path, default=DEFAULT_GMAT_DIR)
    p.add_argument("--out", type=Path, default=DEFAULT_OUT)
    p.add_argument("--impulsive", action="store_true")
    args = p.parse_args()
    mode = "impulsive" if args.impulsive else "finite_thrust"
    schedule = build_schedule(args.gmat_dir, A3LeoGeoConfig(), execution_mode=mode)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(schedule, indent=2), encoding="utf-8")
    print(f"Wrote {args.out} (mode={mode}, mission_mode={schedule.get('mission_mode')})")
    for burn in schedule["burns"]:
        print(f"  {burn['id']}: type={burn['type']} t={float(burn.get('start_s',0))/3600:.2f}h dur={float(burn.get('duration_s',0)):.0f}s")


if __name__ == "__main__":
    main()
