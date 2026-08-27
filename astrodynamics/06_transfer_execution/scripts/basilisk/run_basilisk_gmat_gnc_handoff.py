#!/usr/bin/env python3
"""Run Basilisk execution for GMAT handoff schedule."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import GravityGradientEffector, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, unitTestSupport

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parents[1]
CATEC_RUNS = WORKSPACE / "catec_ws/benchmarks/benchmark_a/runs"
DEFAULT_SCHEDULE = WORKSPACE / "data/pipeline_handoff/gmat/burn_schedule.json"
DEFAULT_OUT = WORKSPACE / "data/pipeline_handoff/basilisk/gmat_gnc_handoff"
if str(CATEC_RUNS) not in sys.path:
    sys.path.insert(0, str(CATEC_RUNS))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from a3_leo_geo_config import A3LeoGeoConfig  # noqa: E402
from attach_solar_environment import attach_solar_environment, latest_eclipse_factor  # noqa: E402
from ephemeris_handoff import SpiceReferenceEphemeris, burn_schedule_timing_error, parse_epoch_utc, position_error_envelope  # noqa: E402
from export_gmat_handoff import vnb_delta_v_to_inertial  # noqa: E402
from fidelity_config import resolve_fidelity  # noqa: E402
from force_model import attach_a3_force_model  # noqa: E402
from gnc_stack import (  # noqa: E402
    attach_velocity_pointing_gnc,
    compute_gnc_initial_sigma,
    pointing_series_from_state_rec,
    thrust_axis_pointing_error_deg,
)
from handoff_checkpoint import HandoffCheckpointWriter, resolve_checkpoint_interval_s  # noqa: E402
from orbit_handoff import resolve_burn_start_s  # noqa: E402
from nav_filter_stack import attach_hardware_nav  # noqa: E402
from solar_power_model import GmatSolarPowerConfig, GmatSolarPowerModel  # noqa: E402
from thruster_stack import (  # noqa: E402
    PropulsionConfig,
    ThrustMassPolynomialModel,
    attach_thruster_system,
    clear_thrust_command,
    deplete_mass,
    fuel_burned_kg,
    set_thrust_command,
    spiral_mass_depletion_kg,
)
from vizard_support import (  # noqa: E402
    VizardOptions,
    attach_handoff_vizard,
    create_gmat_reference_spacecraft,
    default_vizard_bin_path,
    launch_vizard,
    sync_gmat_reference_spacecraft,
)


def run(
    schedule_path: Path,
    out_dir: Path,
    *,
    max_time_s: float | None = None,
    enable_gnc: bool = True,
    enable_mtb: bool = False,
    force_mode: str = "gmat_match",
    fidelity: str = "baseline",
    vizard: VizardOptions | None = None,
    checkpoint_interval_s: float | None = None,
) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
    init = schedule["initial_state"]
    burns = sorted(schedule["burns"], key=lambda b: float(b.get("start_s", 0.0)))
    propulsion = schedule.get("propulsion", {})
    target = schedule.get("target", {})
    dynamics = schedule.get("dynamics", {})
    fidelity_flags = resolve_fidelity(fidelity)
    step_s = float(dynamics.get("step_s", 10.0))
    dyn_step_s = min(step_s, 0.5) if enable_gnc else step_s
    output_sample_s = max(float(dynamics.get("output_sample_s", 60.0)), dyn_step_s)

    sim = SimulationBaseClass.SimBaseClass()
    dyn_proc = sim.CreateNewProcess("dynProc")
    dyn_task = sim.CreateNewTask("dynTask", macros.sec2nano(dyn_step_s))
    dyn_proc.addTask(dyn_task)
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "HandoffSC"
    sc.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d([0.15, 0, 0, 0, 0.18, 0, 0, 0, 0.12])
    sc.hub.r_CN_NInit = init["r_m"]
    sc.hub.v_CN_NInit = init["v_m_s"]
    # Spacecraft integrator registered after effectors below (BskSim ordering).

    prop_cfg = PropulsionConfig(
        dry_mass_kg=float(propulsion.get("dry_mass_kg", 12.0)),
        initial_fuel_mass_kg=float(propulsion.get("initial_fuel_mass_kg", 5.0)),
        max_thrust_n=float(propulsion.get("max_thrust_n", 0.05)),
        isp_s=float(propulsion.get("isp_s", 2500.0)),
    )
    sc.hub.mHub = prop_cfg.initial_mass_kg
    forces = attach_a3_force_model(
        sim,
        "dynTask",
        sc,
        A3LeoGeoConfig(),
        mode=force_mode,
        facet_physics=fidelity_flags.facet_physics,
    )
    solar_env = None
    solar_model = None
    sun_rec = None
    if forces.get("spice_object") is not None:
        sun_rec = forces["spice_object"].planetStateOutMsgs[0].recorder(macros.sec2nano(output_sample_s))
        sim.AddModelToTask("dynTask", sun_rec)
    if fidelity_flags.dynamic_solar and forces.get("spice_object") is not None:
        solar_env = attach_solar_environment(
            sim,
            "dynTask",
            sc,
            forces["spice_object"],
            sample_step_s=output_sample_s,
        )
        if forces.get("srp_effector") is not None:
            forces["srp_effector"].sunEclipseInMsg.subscribeTo(solar_env["eclipse_msg"])
        facet_bundle = forces.get("facet_bundle")
        if facet_bundle is not None:
            facet_bundle["srp_effector"].sunEclipseInMsg.subscribeTo(solar_env["eclipse_msg"])
        em = propulsion.get("electric_model") or {}
        solar_model = GmatSolarPowerModel(
            GmatSolarPowerConfig.from_electric_model(em),
            mission_epoch_utc=schedule.get("mission_epoch_utc_iso"),
        )
    sigma_init = compute_gnc_initial_sigma(
        np.array(init["r_m"], dtype=float),
        np.array(init["v_m_s"], dtype=float),
        forces["earth"].mu,
    )
    sc.hub.sigma_BNInit = [[float(v)] for v in sigma_init]
    sim.AddModelToTask("dynTask", sc, 201)
    thr = attach_thruster_system(sim, "dynTask", sc, prop_cfg, sample_step_s=60.0)
    gg_effector = GravityGradientEffector.GravityGradientEffector()
    gg_effector.ModelTag = "HandoffGravityGradient"
    gg_effector.addPlanetName(forces["earth"].planetName)
    sc.addDynamicEffector(gg_effector)
    sim.AddModelToTask("dynTask", gg_effector, 300)
    gnc = None
    hardware_nav = None
    if enable_gnc and fidelity_flags.full_sensors:
        hardware_nav = attach_hardware_nav(
            sim,
            "dynTask",
            sc,
            sample_step_s=output_sample_s,
        )
    if enable_gnc:
        gnc = attach_velocity_pointing_gnc(
            sim,
            "dynTask",
            sc,
            mu_earth=forces["earth"].mu,
            sample_step_s=output_sample_s,
            epoch_msg=forces["grav_factory"].epochMsg,
            enable_mtb=enable_mtb and force_mode in {"gmat_match", "legacy_ggm03s"},
            nav_mode="hardware" if fidelity_flags.full_sensors else "simpleNav",
            hardware_nav=hardware_nav,
        )

    state_rec = sc.scStateOutMsg.recorder(macros.sec2nano(60.0))
    sim.AddModelToTask("dynTask", state_rec)

    ref_sc = None
    if vizard is not None and vizard.enabled and vizard.show_gmat_reference:
        ref_sc = create_gmat_reference_spacecraft(schedule)
        if ref_sc is not None:
            sim.AddModelToTask("dynTask", ref_sc)

    viz_bin_path = None
    if vizard is not None and vizard.enabled:
        save_file = (vizard.save_file or default_vizard_bin_path(out_dir)).resolve()
        save_file.parent.mkdir(parents=True, exist_ok=True)
        viz_opts = VizardOptions(
            enabled=True,
            live_stream=vizard.live_stream,
            save_file=save_file,
            launch_after_run=vizard.launch_after_run,
            sprite=vizard.sprite,
            show_gmat_reference=vizard.show_gmat_reference,
            reference_sprite=vizard.reference_sprite,
        )
        viz_bin_path = attach_handoff_vizard(
            sim,
            "dynTask",
            sc,
            forces["earth"],
            vizard=viz_opts,
            reference_sc=ref_sc,
        )

    sim.InitializeSimulation()
    if ref_sc is not None:
        sync_gmat_reference_spacecraft(ref_sc, schedule, 0.0)
    sim.SetProgressBar(True)
    pos_ref = sc.dynManager.getStateObject(sc.hub.nameOfHubPosition)
    vel_ref = sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity)
    cur_t = 0.0
    applied = []
    pointing_samples: list[tuple[float, float]] = []
    thrust_samples: list[tuple[float, float, float, float, float]] = []
    sample_step_s = output_sample_s
    next_point_sample = 0.0
    electric_model = ThrustMassPolynomialModel.from_propulsion(propulsion)
    use_polynomial_thrust = electric_model is not None and schedule.get("mission_mode") == "electric_spiral"
    manual_thrust_n: float | None = None
    ckpt_interval = resolve_checkpoint_interval_s(
        vizard_enabled=vizard is not None and vizard.enabled,
        checkpoint_interval_s=checkpoint_interval_s,
    )
    checkpoint_writer = (
        HandoffCheckpointWriter(
            out_dir,
            schedule,
            mu_earth=forces["earth"].mu,
            interval_s=ckpt_interval,
            initial_mass_kg=float(init["mass_kg"]),
        )
        if ckpt_interval is not None
        else None
    )

    def _maybe_checkpoint() -> None:
        if checkpoint_writer is None:
            return
        checkpoint_writer.maybe_write(
            cur_t,
            state_rec=state_rec,
            thrust_samples=thrust_samples,
            pointing_samples=pointing_samples,
            applied_burns=applied,
            current_mass_kg=float(sc.hub.mHub),
            vizard_bin=viz_bin_path,
        )

    def _horizon_s(target_t: float) -> float:
        if max_time_s is not None:
            return min(float(target_t), float(max_time_s))
        return float(target_t)

    def _spiral_stop_mode(burn: dict) -> str:
        return str((burn.get("stop_condition") or {}).get("mode", "osculating_sma"))

    def _latest_sun_pos() -> np.ndarray | None:
        if sun_rec is None:
            return None
        try:
            pos = np.asarray(sun_rec.PositionVector, dtype=float)
        except AttributeError:
            return None
        if pos.size == 0:
            return None
        if pos.ndim == 2:
            return pos[-1].reshape(3)
        return pos.reshape(3)

    def _current_power_kw(r_now: np.ndarray) -> tuple[float, float]:
        if solar_model is None or electric_model is None:
            fallback = float(electric_model.input_power_kw) if electric_model is not None else 0.0
            return fallback, 1.0
        eclipse_factor = latest_eclipse_factor(solar_env["eclipse_rec"] if solar_env else None)
        breakdown = solar_model.propulsion_power_kw(
            cur_t,
            r_now,
            eclipse_factor,
            sun_pos_m=_latest_sun_pos(),
        )
        return breakdown.propulsion_power_kw, breakdown.eclipse_factor

    def _thrust_and_mdot(r_now: np.ndarray | None = None) -> tuple[float, float, float, float]:
        power_kw = electric_model.input_power_kw if electric_model is not None else 0.0
        eclipse_factor = 1.0
        if use_polynomial_thrust and electric_model is not None:
            if solar_model is not None and r_now is not None:
                power_kw, eclipse_factor = _current_power_kw(r_now)
            return (
                electric_model.thrust_n_at_power(power_kw),
                electric_model.mass_flow_kg_s_at_power(power_kw),
                power_kw,
                eclipse_factor,
            )
        thrust_n = float(manual_thrust_n if manual_thrust_n is not None else propulsion.get("max_thrust_n", 0.05))
        mdot = fuel_burned_kg(thrust_n, 1.0, prop_cfg.isp_s)
        return thrust_n, mdot, power_kw, eclipse_factor

    def _propagate_to(target_t: float, *, thrusting: bool = False, log_pointing: bool = False) -> None:
        nonlocal cur_t, next_point_sample
        target_t = _horizon_s(target_t)
        if cur_t >= target_t - 1e-9:
            return
        clear_thrust_command(thr)
        while cur_t < float(target_t) - 1e-9:
            if ref_sc is not None:
                sync_gmat_reference_spacecraft(ref_sc, schedule, cur_t)
            r_now = unitTestSupport.EigenVector3d2np(pos_ref.getState())
            v_now = unitTestSupport.EigenVector3d2np(vel_ref.getState())
            if gnc is not None:
                sigma_bn = unitTestSupport.EigenVector3d2np(
                    sc.dynManager.getStateObject(sc.hub.nameOfHubSigma).getState()
                )
                pointing_err = thrust_axis_pointing_error_deg(sigma_bn, v_now)
                if log_pointing and cur_t + 1e-9 >= next_point_sample:
                    pointing_samples.append((cur_t, pointing_err))
                    next_point_sample += sample_step_s
            thrust_n, mdot_kg_s, power_kw, eclipse_factor = _thrust_and_mdot(r_now) if thrusting else (0.0, 0.0, 0.0, 1.0)
            if thrust_n > 0.0:
                if gnc is not None:
                    set_thrust_command(thr, thrust_n, r_now, v_now, body_frame=True)
                else:
                    set_thrust_command(thr, thrust_n, r_now, v_now, body_frame=False)
                thrust_samples.append((cur_t, thrust_n, float(sc.hub.mHub), power_kw, eclipse_factor))
            nxt = min(float(target_t), cur_t + step_s)
            sim.ConfigureStopTime(macros.sec2nano(nxt))
            sim.ExecuteSimulation()
            if thrust_n > 0.0:
                if use_polynomial_thrust and electric_model is not None:
                    fuel_kg = mdot_kg_s * (nxt - cur_t)
                elif float(propulsion.get("gmat_spiral_mass_flow_kg_s", 0.0)) > 0.0:
                    fuel_kg = float(propulsion["gmat_spiral_mass_flow_kg_s"]) * (nxt - cur_t)
                else:
                    fuel_kg = spiral_mass_depletion_kg(
                        nxt - cur_t,
                        schedule=schedule,
                        thrust_n=thrust_n,
                        isp_s=prop_cfg.isp_s,
                        electric_model=electric_model,
                    )
                deplete_mass(sc, fuel_kg, dry_mass_kg=prop_cfg.dry_mass_kg)
            cur_t = nxt
            if ref_sc is not None:
                sync_gmat_reference_spacecraft(ref_sc, schedule, cur_t)
            _maybe_checkpoint()
        clear_thrust_command(thr)

    for burn in burns:
        start = float(burn.get("start_s", 0.0))
        if max_time_s is not None and start > float(max_time_s) + 1e-9:
            break
        if cur_t >= _horizon_s(cur_t + 1e-9):
            break
        if cur_t < start:
            _propagate_to(start, log_pointing=gnc is not None)
        btype = burn.get("type")
        dur = float(burn.get("duration_s", 0.0))
        if btype in {"finite_thrust", "electric_spiral"}:
            thrust_n_nominal, _, nominal_power_kw, _ = _thrust_and_mdot(
                unitTestSupport.EigenVector3d2np(pos_ref.getState())
            )
            thrust_n = float(burn.get("thrust_n", thrust_n_nominal or prop_cfg.max_thrust_n))
            if btype == "finite_thrust":
                thrust_n = min(thrust_n, prop_cfg.max_thrust_n) or prop_cfg.max_thrust_n
                manual_thrust_n = thrust_n
            end = start + dur
            if burn.get("gmat_end_s") is not None:
                end = min(end, float(burn["gmat_end_s"]))
            end = _horizon_s(end)
            stop_mode = _spiral_stop_mode(burn) if btype == "electric_spiral" else "gmat_end_s"
            sma_target = float((burn.get("stop_condition") or {}).get("semi_major_axis_m", 0.0))
            spiral_thrusting = True
            spiral_stop_reason = "gmat_end"
            while cur_t < end - 1e-9:
                if spiral_thrusting and float(sc.hub.mHub) <= prop_cfg.dry_mass_kg + 1e-6:
                    spiral_thrusting = False
                    spiral_stop_reason = "fuel_depleted"
                _propagate_to(min(end, cur_t + step_s), thrusting=spiral_thrusting, log_pointing=spiral_thrusting)
                if (
                    spiral_thrusting
                    and btype == "electric_spiral"
                    and stop_mode == "osculating_sma"
                    and sma_target > 0.0
                ):
                    r_chk = unitTestSupport.EigenVector3d2np(pos_ref.getState())
                    v_chk = unitTestSupport.EigenVector3d2np(vel_ref.getState())
                    if np.all(np.isfinite(r_chk)) and np.all(np.isfinite(v_chk)):
                        if float(orbitalMotion.rv2elem(forces["earth"].mu, r_chk, v_chk).a) >= sma_target:
                            spiral_thrusting = False
                            spiral_stop_reason = "sma_target_coast"
            clear_thrust_command(thr)
            applied.append(
                {
                    "id": burn["id"],
                    "time_s": start,
                    "gmat_start_s": burn.get("gmat_start_s"),
                    "duration_s": max(0.0, cur_t - start),
                    "type": btype,
                    "thrust_n": thrust_n,
                    "thrust_model": burn.get("thrust_model") or propulsion.get("thrust_model"),
                    "input_power_kw": nominal_power_kw if solar_model is not None else (
                        electric_model.input_power_kw if electric_model is not None else None
                    ),
                    "nominal_thrust_n": thrust_n_nominal if use_polynomial_thrust else thrust_n,
                    "mass_after_kg": float(sc.hub.mHub),
                    "stop_reason": spiral_stop_reason if btype == "electric_spiral" else None,
                    "stop_mode": stop_mode if btype == "electric_spiral" else None,
                }
            )
            manual_thrust_n = None
        elif btype == "coast_trim":
            end = start + dur
            if burn.get("gmat_end_s") is not None:
                end = min(end, float(burn["gmat_end_s"]))
            end = _horizon_s(end)
            if cur_t < end - 1e-9:
                _propagate_to(end, log_pointing=gnc is not None)
            applied.append({"id": burn["id"], "time_s": start, "gmat_start_s": burn.get("gmat_start_s"), "duration_s": max(0.0, cur_t - start), "type": btype, "thrust_n": 0.0, "mass_after_kg": float(sc.hub.mHub)})
        elif btype in {"impulsive_vnb", "impulsive"} and burn.get("gmat_vnb_components_m_s"):
            r_pre = unitTestSupport.EigenVector3d2np(pos_ref.getState())
            v_pre = unitTestSupport.EigenVector3d2np(vel_ref.getState())
            dv = vnb_delta_v_to_inertial(r_pre, v_pre, *burn["gmat_vnb_components_m_s"])
            v_after = v_pre + dv
            vel_ref.setState(unitTestSupport.np2EigenVectorXd(v_after))
            sim.ConfigureStopTime(macros.sec2nano(cur_t + step_s))
            sim.ExecuteSimulation()
            cur_t += step_s
            applied.append({"id": burn["id"], "time_s": start, "gmat_start_s": burn.get("gmat_start_s"), "duration_s": 0.0, "type": btype, "thrust_n": 0.0, "mass_after_kg": float(sc.hub.mHub)})

    final_time = _horizon_s(float(dynamics.get("mission_end_s", cur_t)))
    if final_time > cur_t:
        # Short coast-only horizons: one continuous execute avoids RW allocator
        # issues seen when re-entering the FSW stack every dynamics macro-step.
        short_coast_only = (
            max_time_s is not None
            and not applied
            and cur_t < 1e-9
            and all(float(b.get("start_s", 0.0)) > float(max_time_s) + 1e-9 for b in burns)
        )
        if short_coast_only:
            clear_thrust_command(thr)
            sim.ConfigureStopTime(macros.sec2nano(final_time))
            sim.ExecuteSimulation()
            cur_t = final_time
            _maybe_checkpoint()
        else:
            _propagate_to(final_time, log_pointing=gnc is not None)

    times = np.asarray(state_rec.times() * 1e-9, dtype=float)
    pos = np.asarray(state_rec.r_BN_N, dtype=float)
    vel = np.asarray(state_rec.v_BN_N, dtype=float)
    r_fin = unitTestSupport.EigenVector3d2np(pos_ref.getState())
    v_fin = unitTestSupport.EigenVector3d2np(vel_ref.getState())
    if np.all(np.isfinite(r_fin)) and np.all(np.isfinite(v_fin)):
        oe = orbitalMotion.rv2elem(forces["earth"].mu, r_fin, v_fin)
        final_sma_km, final_ecc, final_inc_deg = float(oe.a / 1000.0), float(oe.e), float(np.degrees(oe.i))
    else:
        final_sma_km, final_ecc, final_inc_deg = float("nan"), float("nan"), float("nan")

    max_ephem = float("nan")
    ref_meta = schedule.get("reference_ephemeris", {})
    if ref_meta.get("path"):
        ref = SpiceReferenceEphemeris.from_metadata(ref_meta)
        max_ephem, _ = position_error_envelope(times, pos, spice_ref=ref, min_time_s=float(ref_meta.get("bsp_coverage_start_s", 0.0)))

    max_timing, _ = burn_schedule_timing_error(applied, tolerance_s=float(target.get("max_burn_start_error_s", 120.0)))
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "state_history.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "rx_m", "ry_m", "rz_m", "vx_m_s", "vy_m_s", "vz_m_s"])
        for t, r, v in zip(times, pos, vel):
            w.writerow([f"{t:.3f}", *[f"{x:.6f}" for x in r], *[f"{x:.6f}" for x in v]])
    with (out_dir / "thrust_history.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "thrust_n", "mass_kg", "power_kw", "eclipse_factor"])
        if thrust_samples:
            for t_s, t_n, m_kg, p_kw, ecl in thrust_samples:
                w.writerow([f"{t_s:.3f}", f"{t_n:.6f}", f"{m_kg:.6f}", f"{p_kw:.6f}", f"{ecl:.6f}"])
        else:
            mass = float(schedule["initial_state"]["mass_kg"])
            for b in applied:
                w.writerow([f"{b['time_s']:.3f}", f"{b.get('thrust_n',0.0):.6f}", f"{mass:.6f}", "0.0", "1.0"])
                mass = float(b.get("mass_after_kg", mass))

    with (out_dir / "orbit_elements_history.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "sma_km", "ecc", "inc_deg"])
        for t, r, v in zip(times, pos, vel):
            if np.all(np.isfinite(r)) and np.all(np.isfinite(v)):
                oe = orbitalMotion.rv2elem(forces["earth"].mu, r, v)
                w.writerow([f"{t:.3f}", f"{float(oe.a)/1000.0:.6f}", f"{float(oe.e):.8f}", f"{math.degrees(float(oe.i)):.6f}"])
            else:
                w.writerow([f"{t:.3f}", "nan", "nan", "nan"])

    pointing_csv = out_dir / "pointing_error_history.csv"
    rw_diag_csv = out_dir / "rw_diagnostics.csv"
    att_guid_csv = out_dir / "att_guidance_debug.csv"
    mtb_diag_csv = out_dir / "mtb_diagnostics.csv"
    if gnc is not None:
        pointing_series = pointing_series_from_state_rec(state_rec)
        with pointing_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "pointing_error_deg"])
            for t, err in pointing_series:
                w.writerow([f"{t:.3f}", f"{err:.6f}" if math.isfinite(err) else "nan"])
        wheel_labels = list(gnc["rw_factory"].rwList.keys())
        rw_js = np.array([float(gnc["rw_factory"].rwList[label].Js) for label in wheel_labels], dtype=float)
        rw_umax = np.array([float(gnc["rw_factory"].rwList[label].u_max) for label in wheel_labels], dtype=float)
        rw_times = np.asarray(gnc["rw_speed_log"].times() * 1e-9, dtype=float)
        rw_speed_msg = np.asarray(gnc["rw_speed_log"].wheelSpeeds, dtype=float)
        rw_motor = np.asarray(gnc["rw_torque_log"].motorTorque, dtype=float)
        motor_times = np.asarray(gnc["rw_torque_log"].times() * 1e-9, dtype=float)
        cmd_body = np.asarray(gnc["cmd_torque_log"].torqueRequestBody, dtype=float)
        cmd_times = np.asarray(gnc["cmd_torque_log"].times() * 1e-9, dtype=float)
        n_wheels = len(wheel_labels)
        if rw_speed_msg.ndim == 2 and rw_speed_msg.shape[0] == len(rw_times):
            rw_speeds = rw_speed_msg[:, :n_wheels]
        elif rw_speed_msg.ndim == 2 and rw_speed_msg.shape[1] == len(rw_times):
            rw_speeds = rw_speed_msg[:n_wheels, :].T
        else:
            rw_speeds = np.empty((len(rw_times), n_wheels))
        rw_omegas = [np.asarray(log.Omega, dtype=float).reshape(-1) for log in gnc["rw_out_logs"]]
        rw_torques = [np.asarray(log.u_current, dtype=float).reshape(-1) for log in gnc["rw_out_logs"]]
        if rw_omegas and all(len(arr) == len(rw_times) for arr in rw_omegas):
            rw_torque_applied = np.vstack([arr for arr in rw_torques]).T[:, :n_wheels]
        else:
            rw_torque_applied = np.zeros_like(rw_speeds)
        if rw_motor.ndim == 2 and rw_motor.shape[0] == len(motor_times):
            rw_torque_cmd = rw_motor[:, :n_wheels]
        elif rw_motor.ndim == 2 and rw_motor.shape[1] == len(motor_times):
            rw_torque_cmd = rw_motor[:n_wheels, :].T
        else:
            rw_torque_cmd = rw_torque_applied
        rw_momentum = rw_speeds * rw_js.reshape(1, -1)
        with rw_diag_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "time_s",
                    "wheel_index",
                    "speed_rad_s",
                    "momentum_nms",
                    "torque_cmd_nm",
                    "torque_applied_nm",
                    "body_torque_cmd_nm",
                    "torque_sat",
                ]
            )
            for i, t in enumerate(rw_times):
                motor_idx = int(np.argmin(np.abs(motor_times - t))) if len(motor_times) else 0
                cmd_idx = int(np.argmin(np.abs(cmd_times - t))) if len(cmd_times) else 0
                body_cmd = cmd_body[cmd_idx] if cmd_body.ndim == 2 and cmd_idx < len(cmd_body) else np.zeros(3)
                for j in range(n_wheels):
                    torque_val = float(rw_torque_cmd[motor_idx, j]) if motor_idx < len(rw_torque_cmd) else 0.0
                    applied_val = float(rw_torque_applied[i, j]) if i < len(rw_torque_applied) else 0.0
                    speed_val = float(rw_speeds[i, j]) if i < len(rw_speeds) else float("nan")
                    w.writerow(
                        [
                            f"{t:.3f}",
                            j,
                            f"{speed_val:.6f}" if math.isfinite(speed_val) else "nan",
                            f"{float(rw_momentum[i, j]):.6f}" if math.isfinite(speed_val) else "nan",
                            f"{torque_val:.6f}" if math.isfinite(torque_val) else "nan",
                            f"{applied_val:.6f}" if math.isfinite(applied_val) else "nan",
                            f"{float(np.linalg.norm(body_cmd)):.6f}" if j == 0 and np.all(np.isfinite(body_cmd)) else "",
                            int(abs(torque_val) >= 0.99 * rw_umax[j]) if math.isfinite(torque_val) else 0,
                        ]
                    )
        att_times = np.asarray(gnc["att_error_log"].times() * 1e-9, dtype=float)
        att_sigma_br = np.asarray(gnc["att_error_log"].sigma_BR, dtype=float)
        att_w_br = np.asarray(gnc["att_error_log"].omega_BR_B, dtype=float)
        with att_guid_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "sigma_br_0", "sigma_br_1", "sigma_br_2", "omega_br_0", "omega_br_1", "omega_br_2"])
            for i in range(len(att_times)):
                w.writerow(
                    [
                        f"{att_times[i]:.3f}",
                        f"{att_sigma_br[i,0]:.9e}",
                        f"{att_sigma_br[i,1]:.9e}",
                        f"{att_sigma_br[i,2]:.9e}",
                        f"{att_w_br[i,0]:.9e}",
                        f"{att_w_br[i,1]:.9e}",
                        f"{att_w_br[i,2]:.9e}",
                    ]
                )
        if "mtb_cmd_log" in gnc:
            mtb_times = np.asarray(gnc["mtb_cmd_log"].times() * 1e-9, dtype=float)
            mtb_cmds = np.asarray(gnc["mtb_cmd_log"].mtbDipoleCmds, dtype=float)
            with mtb_diag_csv.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "dipole_x_a_m2", "dipole_y_a_m2", "dipole_z_a_m2"])
                for i in range(len(mtb_times)):
                    w.writerow(
                        [
                            f"{mtb_times[i]:.3f}",
                            f"{mtb_cmds[i,0]:.9e}",
                            f"{mtb_cmds[i,1]:.9e}",
                            f"{mtb_cmds[i,2]:.9e}",
                        ]
                    )

    summary = {
        "task_id": "gmat_basilisk_gnc_handoff",
        "execution_mode": schedule.get("execution_mode"),
        "mission_mode": schedule.get("mission_mode"),
        "force_mode": force_mode,
        "fidelity": fidelity,
        "fidelity_flags": {
            "dynamic_solar": fidelity_flags.dynamic_solar,
            "facet_physics": fidelity_flags.facet_physics,
            "full_sensors": fidelity_flags.full_sensors,
        },
        "force_model": {
            "gravity": forces["spec"].gravity,
            "atmosphere": forces["spec"].atmosphere,
            "drag": forces["spec"].drag,
            "srp": forces["spec"].srp,
            "geometry": forces["spec"].geometry,
            "third_bodies": list(forces["spec"].third_bodies),
        },
        "validation_passed": False,
        "converged": bool(schedule.get("gmat_reference", {}).get("converged_sma", False)),
        "final_sma_km": final_sma_km,
        "final_ecc": final_ecc,
        "final_inc_deg": final_inc_deg,
        "max_ephemeris_position_error_m": max_ephem,
        "max_burn_start_error_s": max_timing,
        "fuel_used_kg": float(schedule["initial_state"]["mass_kg"] - float(sc.hub.mHub)),
        "gmat_reference": schedule.get("gmat_reference", {}),
        "gnc_mode": (
            "rw_mtb_velocity_pointing_hardware_nav"
            if gnc is not None and gnc.get("mtb_enabled") and fidelity_flags.full_sensors
            else (
                "rw_mtb_velocity_pointing"
                if gnc is not None and gnc.get("mtb_enabled")
                else (
                    "rw_feedback_velocity_pointing_hardware_nav"
                    if gnc is not None and fidelity_flags.full_sensors
                    else ("rw_feedback_velocity_pointing" if gnc is not None else ("kinematic" if not enable_gnc else None))
                )
            )
        ),
        "gnc_gains": {"K": gnc.get("mrp_k"), "P": gnc.get("mrp_p")} if gnc is not None else None,
        "nav_noise": (
            {
                "enabled": bool(gnc["nav_noise_config"].enabled),
                "attitude_noise_sigma_deg": float(gnc["nav_noise_config"].attitude_noise_sigma_deg),
                "rate_noise_sigma_deg_s": float(gnc["nav_noise_config"].rate_noise_sigma_deg_s),
                "attitude_error_3sigma_deg": float(gnc["nav_noise_config"].attitude_error_3sigma_deg),
                "rate_error_3sigma_deg_s": float(gnc["nav_noise_config"].rate_error_3sigma_deg_s),
            }
            if gnc is not None and gnc.get("nav_noise_config") is not None
            else None
        ),
        "propulsion_model": {
            "thrust_model": propulsion.get("thrust_model"),
            "nominal_power_kw": (electric_model.input_power_kw if electric_model is not None else None),
            "dynamic_solar": fidelity_flags.dynamic_solar,
            "nominal_thrust_n": (electric_model.thrust_n() if electric_model is not None else propulsion.get("max_thrust_n")),
            "nominal_mdot_kg_s": (electric_model.mass_flow_kg_s() if electric_model is not None else propulsion.get("gmat_spiral_mass_flow_kg_s")),
        },
        "burns_applied": applied,
        "pointing_error_history": str(pointing_csv) if gnc is not None else None,
        "rw_diagnostics": str(rw_diag_csv) if (gnc is not None and rw_diag_csv.exists()) else None,
        "att_guidance_debug": str(att_guid_csv) if (gnc is not None and att_guid_csv.exists()) else None,
        "mtb_diagnostics": str(mtb_diag_csv) if (gnc is not None and mtb_diag_csv.exists()) else None,
    }
    if viz_bin_path:
        summary["vizard_bin"] = viz_bin_path
        summary["vizard_reference_overlay"] = bool(ref_sc is not None)
    from plot_handoff_validation import _compute_validation_checks

    _compute_validation_checks(summary, schedule, out_dir=out_dir)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if checkpoint_writer is not None:
        checkpoint_writer.write(
            cur_t,
            state_rec=state_rec,
            thrust_samples=thrust_samples,
            pointing_samples=pointing_samples,
            applied_burns=applied,
            current_mass_kg=float(sc.hub.mHub),
            vizard_bin=viz_bin_path,
            status="completed",
            include_summary=False,
        )
    return summary


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--schedule", type=Path, default=DEFAULT_SCHEDULE)
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    p.add_argument("--force-mode", choices=("gmat_match", "legacy_ggm03s"), default="gmat_match")
    p.add_argument(
        "--fidelity",
        choices=("baseline", "dynamic_solar", "facet_physics", "full_sensors", "mission_realistic"),
        default="baseline",
        help="Physics/nav fidelity tier (baseline keeps legacy cannonball + SimpleNav)",
    )
    p.add_argument("--kinematic", action="store_true")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--max-time-s", type=float, default=None)
    p.add_argument("--vizard", action="store_true", help="Record Basilisk Unity/Vizard telemetry (.bin)")
    p.add_argument(
        "--checkpoint-interval-s",
        type=float,
        default=None,
        help="Flush partial CSV/progress checkpoints during the run (default 3600 s when --vizard is set)",
    )
    p.add_argument("--vizard-live", action="store_true", help="Stream simulation live to Vizard")
    p.add_argument("--vizard-file", type=Path, default=None, help="Output path for UnityViz .bin")
    p.add_argument("--no-vizard-launch", action="store_true", help="Record .bin only; do not open Vizard")
    p.add_argument("--no-gmat-reference-viz", action="store_true", help="Hide GMAT BSP reference spacecraft overlay")
    p.add_argument("--mtb", action="store_true", help="Enable MTB momentum-dump FSW (experimental)")
    p.add_argument("--no-mtb", action="store_true", help="Disable MTB momentum dump (default)")
    p.add_argument("--install-vizard", action="store_true", help="Download Vizard into tools/Vizard and exit")
    args = p.parse_args()
    if args.install_vizard:
        from vizard_support import install_vizard

        print(f"Vizard ready at: {install_vizard()}")
        return

    vizard = VizardOptions(
        enabled=args.vizard or args.vizard_live,
        live_stream=args.vizard_live,
        save_file=args.vizard_file or (default_vizard_bin_path(args.out_dir) if (args.vizard or args.vizard_live) else None),
        launch_after_run=not args.no_vizard_launch,
        show_gmat_reference=not args.no_gmat_reference_viz,
    )
    t0 = time.perf_counter()
    summary = run(
        args.schedule,
        args.out_dir,
        max_time_s=args.max_time_s,
        enable_gnc=not args.kinematic,
        enable_mtb=args.mtb and not args.no_mtb,
        force_mode=args.force_mode,
        fidelity=args.fidelity,
        vizard=vizard if vizard.enabled else None,
        checkpoint_interval_s=args.checkpoint_interval_s,
    )
    runtime = time.perf_counter() - t0
    if args.plot:
        from plot_handoff_validation import generate_validation_plots

        generate_validation_plots(args.out_dir, schedule_path=args.schedule)
    print(f"validation_passed={summary['validation_passed']} converged={summary['converged']} runtime_s={runtime:.2f}")
    print(f"final SMA={summary['final_sma_km']:.3f} km e={summary['final_ecc']:.6f} i={summary['final_inc_deg']:.4f} deg")
    print(f"fuel_used={summary['fuel_used_kg']:.3f} kg")
    print(f"ephemeris_max_err={summary.get('max_ephemeris_position_error_m', float('nan'))/1000:.1f} km burn_start_err={summary.get('max_burn_start_error_s',0):.1f} s")
    if summary.get("vizard_bin"):
        print(f"Vizard file: {summary['vizard_bin']}")
    print(f"outputs: {args.out_dir}")
    if vizard.enabled and vizard.launch_after_run and not vizard.live_stream and summary.get("vizard_bin"):
        launch_vizard(Path(summary["vizard_bin"]))


if __name__ == "__main__":
    main()
