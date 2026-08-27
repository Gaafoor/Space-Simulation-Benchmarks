"""Orbit utility helpers."""

from __future__ import annotations

import math

import numpy as np
from Basilisk.utilities import orbitalMotion


def rv_to_elements(mu: float, r_m: np.ndarray, v_m_s: np.ndarray) -> orbitalMotion.ClassicElements:
    return orbitalMotion.rv2elem(mu, np.asarray(r_m, dtype=float), np.asarray(v_m_s, dtype=float))


def leo_period_s(mu: float, r_m: np.ndarray, v_m_s: np.ndarray) -> float:
    oe = rv_to_elements(mu, r_m, v_m_s)
    return 2.0 * math.pi * math.sqrt(oe.a**3 / mu)


def perigee_time_after_s(mu: float, r_m: np.ndarray, v_m_s: np.ndarray, t_after_s: float) -> float:
    p = leo_period_s(mu, r_m, v_m_s)
    if p <= 0:
        return t_after_s
    return max(1, math.ceil(float(t_after_s) / p)) * p


def transfer_half_period_s(mu: float, semi_major_axis_m: float) -> float:
    return math.pi * math.sqrt(float(semi_major_axis_m) ** 3 / mu)


def resolve_burn_start_s(burn: dict, mu: float, r_m: np.ndarray, v_m_s: np.ndarray, t_now_s: float) -> float:
    if burn.get("trigger") == "perigee":
        return perigee_time_after_s(mu, r_m, v_m_s, t_now_s)
    return float(burn.get("start_s", t_now_s))
