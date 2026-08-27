"""Fidelity tier resolution for Basilisk handoff runs."""

from __future__ import annotations

from dataclasses import dataclass

FIDELITY_TIERS = (
    "baseline",
    "dynamic_solar",
    "facet_physics",
    "full_sensors",
    "mission_realistic",
)


@dataclass(frozen=True)
class FidelityFlags:
    dynamic_solar: bool = False
    facet_physics: bool = False
    full_sensors: bool = False

    @property
    def label(self) -> str:
        if self.full_sensors and self.facet_physics and self.dynamic_solar:
            return "mission_realistic"
        if self.full_sensors:
            return "full_sensors"
        if self.facet_physics:
            return "facet_physics"
        if self.dynamic_solar:
            return "dynamic_solar"
        return "baseline"


def resolve_fidelity(fidelity: str) -> FidelityFlags:
    tier = (fidelity or "baseline").strip().lower()
    if tier not in FIDELITY_TIERS:
        raise ValueError(f"Unknown fidelity tier {fidelity!r}; expected one of {FIDELITY_TIERS}")
    if tier == "baseline":
        return FidelityFlags(False, False, False)
    if tier == "dynamic_solar":
        return FidelityFlags(True, False, False)
    if tier == "facet_physics":
        return FidelityFlags(True, True, False)
    if tier == "full_sensors":
        return FidelityFlags(True, True, True)
    return FidelityFlags(True, True, True)
