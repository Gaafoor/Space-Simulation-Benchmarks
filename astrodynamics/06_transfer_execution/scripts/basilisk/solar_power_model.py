"""GMAT SolarPowerSystem model for Basilisk handoff."""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone

import numpy as np

AU_M = 149597870700.0


def _julian_day(dt: datetime) -> float:
    dt = dt.astimezone(timezone.utc)
    year = dt.year
    month = dt.month
    day = dt.day + (dt.hour + (dt.minute + dt.second / 60.0) / 60.0) / 24.0
    if month <= 2:
        year -= 1
        month += 12
    a = int(year / 100)
    b = 2 - a + int(a / 4)
    return int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + b - 1524.5


@dataclass(frozen=True)
class GmatSolarPowerConfig:
    initial_max_power_kw: float
    margin_frac: float
    annual_decay_rate_frac: float
    solar_coeffs: tuple[float, float, float, float, float]
    bus_coeffs: tuple[float, float, float]
    shadow_active: bool = True
    initial_epoch_utc: str = "2026-06-15T12:00:00.000Z"

    @classmethod
    def from_electric_model(cls, em: dict) -> GmatSolarPowerConfig:
        solar = tuple(float(x) for x in em.get("solar_coeffs", (1.0, 0.0, 0.0, 0.0, 0.0)))
        if len(solar) < 5:
            solar = (solar + (0.0,) * 5)[:5]
        bus = tuple(float(x) for x in em.get("bus_coeffs", (0.3, 0.0, 0.0)))
        if len(bus) < 3:
            bus = (bus + (0.0,) * 3)[:3]
        annual = float(em.get("annual_decay_rate_frac", em.get("solar_annual_decay_rate_frac", 0.05)))
        return cls(
            initial_max_power_kw=float(em.get("solar_initial_max_power_kw", 1.2)),
            margin_frac=float(em.get("solar_margin_frac", em.get("solar_margin_percent", 5.0) / 100.0)),
            annual_decay_rate_frac=annual,
            solar_coeffs=solar,  # type: ignore[arg-type]
            bus_coeffs=bus,  # type: ignore[arg-type]
            shadow_active=bool(em.get("solar_shadow_active", True)),
            initial_epoch_utc=str(em.get("solar_initial_epoch_utc", "2026-06-15T12:00:00.000Z")),
        )


@dataclass(frozen=True)
class SolarPowerBreakdown:
    time_s: float
    distance_au: float
    eclipse_factor: float
    base_power_kw: float
    generated_power_kw: float
    bus_power_kw: float
    propulsion_power_kw: float


class GmatSolarPowerModel:
    """Mirror NASA 42 / GMAT SolarPowerSystem used in A3 N8."""

    def __init__(self, config: GmatSolarPowerConfig, *, mission_epoch_utc: str | None = None):
        self.config = config
        epoch_text = mission_epoch_utc or config.initial_epoch_utc
        if epoch_text.endswith("Z"):
            epoch_text = epoch_text[:-1] + "+00:00"
        self.mission_epoch = datetime.fromisoformat(epoch_text).astimezone(timezone.utc)
        if config.initial_epoch_utc.endswith("Z"):
            init_text = config.initial_epoch_utc[:-1] + "+00:00"
        else:
            init_text = config.initial_epoch_utc
        self.initial_epoch = datetime.fromisoformat(init_text).astimezone(timezone.utc)
        self.initial_jd = _julian_day(self.initial_epoch)

    @staticmethod
    def distance_au(r_m: np.ndarray, sun_pos_m: np.ndarray | None = None) -> float:
        """Heliocentric Sun distance using Earth-to-Sun vector and geocentric SC position."""
        r = np.asarray(r_m, dtype=float).reshape(3)
        if sun_pos_m is None:
            return 1.0
        sun = np.asarray(sun_pos_m, dtype=float).reshape(3)
        return float(np.linalg.norm(sun - r) / AU_M)

    def base_power_kw(self, time_s: float) -> float:
        dt_years = (_julian_day(self.mission_epoch + timedelta(seconds=float(time_s))) - self.initial_jd) / 365.25
        decay = max(0.0, 1.0 - self.config.annual_decay_rate_frac)
        return self.config.initial_max_power_kw * (decay**dt_years)

    def generated_power_kw(
        self,
        time_s: float,
        r_m: np.ndarray,
        eclipse_factor: float,
        *,
        sun_pos_m: np.ndarray | None = None,
    ) -> float:
        cfg = self.config
        percent_sun = float(np.clip(eclipse_factor, 0.0, 1.0))
        if cfg.shadow_active:
            percent_sun = 0.0 if percent_sun <= 1e-12 else percent_sun
        r_au = self.distance_au(r_m, sun_pos_m)
        r2 = r_au * r_au
        num = cfg.solar_coeffs[0] + cfg.solar_coeffs[1] / r_au + cfg.solar_coeffs[2] / r2
        denom = 1.0 + cfg.solar_coeffs[3] * r_au + cfg.solar_coeffs[4] * r2
        denom = max(denom, 1.0e-12)
        return percent_sun * self.base_power_kw(time_s) * (num / denom) / r2

    @staticmethod
    def bus_power_kw(cfg: GmatSolarPowerConfig, r_au: float) -> float:
        return cfg.bus_coeffs[0] + cfg.bus_coeffs[1] / r_au + cfg.bus_coeffs[2] / (r_au * r_au)

    def propulsion_power_kw(
        self,
        time_s: float,
        r_m: np.ndarray,
        eclipse_factor: float,
        *,
        sun_pos_m: np.ndarray | None = None,
    ) -> SolarPowerBreakdown:
        r_au = self.distance_au(r_m, sun_pos_m)
        generated = self.generated_power_kw(time_s, r_m, eclipse_factor, sun_pos_m=sun_pos_m)
        bus = self.bus_power_kw(self.config, r_au)
        available = max(0.0, (1.0 - self.config.margin_frac) * (generated - bus))
        return SolarPowerBreakdown(
            time_s=float(time_s),
            distance_au=r_au,
            eclipse_factor=float(np.clip(eclipse_factor, 0.0, 1.0)),
            base_power_kw=self.base_power_kw(time_s),
            generated_power_kw=generated,
            bus_power_kw=bus,
            propulsion_power_kw=available,
        )
