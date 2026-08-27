"""Finite-thrust helpers for GMAT->Basilisk handoff."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import extForceTorque, spacecraft


@dataclass(frozen=True)
class PropulsionConfig:
    dry_mass_kg: float
    initial_fuel_mass_kg: float
    max_thrust_n: float
    isp_s: float

    @property
    def initial_mass_kg(self) -> float:
        return self.dry_mass_kg + self.initial_fuel_mass_kg


@dataclass(frozen=True)
class ThrustMassPolynomialModel:
    """GMAT ElectricThruster ThrustMassPolynomial (power in kW -> mN / mg/s)."""

    thrust_coeff_mn: tuple[float, ...]
    mass_flow_coeff_mgps: tuple[float, ...]
    duty_cycle: float = 1.0
    thrust_scale_factor: float = 1.0
    minimum_usable_power_kw: float = 0.638
    maximum_usable_power_kw: float = 7.266
    input_power_kw: float = 1.0

    @classmethod
    def from_propulsion(cls, propulsion: dict) -> ThrustMassPolynomialModel | None:
        if propulsion.get("thrust_model") != "ThrustMassPolynomial":
            return None
        em = propulsion.get("electric_model") or {}
        power_kw = float(
            em.get("effective_power_kw")
            or em.get("solar_initial_max_power_kw")
            or propulsion.get("nominal_power_kw", 1.0)
        )
        return cls(
            thrust_coeff_mn=tuple(float(x) for x in em.get("thrust_coeff_mn", (0.0,))),
            mass_flow_coeff_mgps=tuple(float(x) for x in em.get("mass_flow_coeff_mgps", (0.0,))),
            duty_cycle=float(em.get("duty_cycle", 1.0)),
            thrust_scale_factor=float(em.get("thrust_scale_factor", 1.0)),
            minimum_usable_power_kw=float(em.get("minimum_usable_power_kw", 0.638)),
            maximum_usable_power_kw=float(em.get("maximum_usable_power_kw", 7.266)),
            input_power_kw=power_kw,
        )

    def _clip_power_kw(self, power_kw: float) -> float:
        if power_kw < self.minimum_usable_power_kw:
            return 0.0
        return min(float(power_kw), self.maximum_usable_power_kw)

    @staticmethod
    def _poly(coeffs: tuple[float, ...], x: float) -> float:
        return float(sum(c * (x**i) for i, c in enumerate(coeffs)))

    def thrust_n_at_power(self, power_kw: float) -> float:
        p = self._clip_power_kw(power_kw)
        if p <= 0.0:
            return 0.0
        thrust_mn = self._poly(self.thrust_coeff_mn, p)
        thrust_n = self.duty_cycle * self.thrust_scale_factor * max(0.0, thrust_mn) * 1e-3
        return float(thrust_n)

    def mass_flow_kg_s_at_power(self, power_kw: float) -> float:
        p = self._clip_power_kw(power_kw)
        if p <= 0.0:
            return 0.0
        mdot_mgps = self._poly(self.mass_flow_coeff_mgps, p)
        mdot_kg_s = self.duty_cycle * self.thrust_scale_factor * max(0.0, mdot_mgps) * 1e-6
        return float(mdot_kg_s)

    def thrust_n(self) -> float:
        return self.thrust_n_at_power(self.input_power_kw)

    def mass_flow_kg_s(self) -> float:
        return self.mass_flow_kg_s_at_power(self.input_power_kw)


def invert_mass_flow_power_kg_s(
    target_mdot_kg_s: float,
    coeffs_mgps: tuple[float, ...],
    *,
    p_min: float,
    p_max: float,
    duty_cycle: float = 1.0,
    thrust_scale_factor: float = 1.0,
) -> float:
    """Solve GMAT mass-flow polynomial for input power [kW]."""
    target_mgps = max(0.0, float(target_mdot_kg_s) * 1e6)
    scale = max(1e-12, duty_cycle * thrust_scale_factor)

    def mdot_mgps(p: float) -> float:
        return scale * max(0.0, sum(c * (p**i) for i, c in enumerate(coeffs_mgps)))

    if mdot_mgps(p_min) >= target_mgps:
        return float(p_min)
    if mdot_mgps(p_max) <= target_mgps:
        return float(p_max)
    lo, hi = float(p_min), float(p_max)
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if mdot_mgps(mid) < target_mgps:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def prograde_force_vector(thrust_n: float, r_m: np.ndarray, v_m_s: np.ndarray) -> list[float]:
    v_hat = np.asarray(v_m_s, dtype=float) / max(np.linalg.norm(v_m_s), 1e-9)
    return (float(thrust_n) * v_hat).tolist()


def attach_thruster_system(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    config: PropulsionConfig,
    *,
    sample_step_s: float = 60.0,
) -> dict:
    sc_object.hub.mHub = float(config.initial_mass_kg)
    thrust_ft = extForceTorque.ExtForceTorque()
    thrust_ft.ModelTag = "HandoffThrustForce"
    sc_object.addDynamicEffector(thrust_ft)
    sc_sim.AddModelToTask(task_name, thrust_ft, 199)

    force_inertial_msg = messaging.CmdForceInertialMsg().write(
        messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.0, 0.0, 0.0])
    )
    force_body_msg = messaging.CmdForceBodyMsg().write(
        messaging.CmdForceBodyMsgPayload(forceRequestBody=[0.0, 0.0, 0.0])
    )
    thrust_ft.cmdForceInertialInMsg.subscribeTo(force_inertial_msg)
    thrust_ft.cmdForceBodyInMsg.subscribeTo(force_body_msg)
    force_rec = thrust_ft.cmdForceInertialInMsg.recorder(int(sample_step_s * 1e9))
    sc_sim.AddModelToTask(task_name, force_rec)
    return {
        "thrust_effector": thrust_ft,
        "force_inertial_msg": force_inertial_msg,
        "force_body_msg": force_body_msg,
        "force_msg": force_inertial_msg,
        "force_rec": force_rec,
        "max_thrust_n": float(config.max_thrust_n),
        "isp_s": float(config.isp_s),
    }


def set_thrust_command(thr: dict, thrust_n: float, r_m: np.ndarray, v_m_s: np.ndarray, *, body_frame: bool) -> None:
    if body_frame:
        thr["force_inertial_msg"].write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.0, 0.0, 0.0]))
        thr["force_body_msg"].write(
            messaging.CmdForceBodyMsgPayload(forceRequestBody=[float(thrust_n), 0.0, 0.0])
        )
    else:
        thr["force_body_msg"].write(messaging.CmdForceBodyMsgPayload(forceRequestBody=[0.0, 0.0, 0.0]))
        thr["force_inertial_msg"].write(
            messaging.CmdForceInertialMsgPayload(forceRequestInertial=prograde_force_vector(thrust_n, r_m, v_m_s))
        )


def clear_thrust_command(thr: dict) -> None:
    thr["force_inertial_msg"].write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.0, 0.0, 0.0]))
    thr["force_body_msg"].write(messaging.CmdForceBodyMsgPayload(forceRequestBody=[0.0, 0.0, 0.0]))


def fuel_burned_kg(thrust_n: float, duration_s: float, isp_s: float) -> float:
    return max(0.0, float(thrust_n)) * float(duration_s) / (float(isp_s) * 9.80665)


def spiral_mass_depletion_kg(
    duration_s: float,
    *,
    schedule: dict | None = None,
    thrust_n: float = 0.0,
    isp_s: float = 2500.0,
    electric_model: ThrustMassPolynomialModel | None = None,
) -> float:
    """Return propellant used over duration_s for the active thrust model."""
    if duration_s <= 0.0:
        return 0.0
    if electric_model is not None:
        return electric_model.mass_flow_kg_s() * float(duration_s)
    gmat_ref = (schedule or {}).get("gmat_reference", {})
    if (schedule or {}).get("mission_mode") == "electric_spiral":
        spiral_start_s = float(gmat_ref.get("spiral_start_s", 0.0))
        spiral_end_s = float(gmat_ref.get("spiral_end_s", spiral_start_s))
        fuel_kg = float(gmat_ref.get("fuel_mass_kg", 0.0))
        if spiral_end_s > spiral_start_s and fuel_kg > 0.0:
            return fuel_kg * float(duration_s) / (spiral_end_s - spiral_start_s)
    return fuel_burned_kg(thrust_n, duration_s, isp_s)


def deplete_mass(sc_object: spacecraft.Spacecraft, fuel_kg: float, *, dry_mass_kg: float | None = None) -> float:
    m = float(sc_object.hub.mHub)
    min_m = float(dry_mass_kg or 0.0)
    used = min(max(0.0, float(fuel_kg)), max(0.0, m - min_m))
    sc_object.hub.mHub = m - used
    return used


def average_thrust_n(fuel_kg: float, duration_s: float, isp_s: float) -> float:
    if duration_s <= 0 or fuel_kg <= 0:
        return 0.0
    return float(fuel_kg) * float(isp_s) * 9.80665 / float(duration_s)
