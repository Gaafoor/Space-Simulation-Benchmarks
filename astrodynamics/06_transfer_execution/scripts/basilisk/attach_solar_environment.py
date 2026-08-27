"""Wire Basilisk eclipse environment for dynamic solar/thrust."""

from __future__ import annotations

from Basilisk.simulation import eclipse, spacecraft


def attach_solar_environment(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    spice_object,
    *,
    earth_spice_index: int = 1,
    sun_spice_index: int = 0,
    sample_step_s: float = 60.0,
) -> dict:
    """Attach conical Earth eclipse model and recorder."""
    eclipse_module = eclipse.Eclipse()
    eclipse_module.ModelTag = "HandoffEclipse"
    eclipse_module.sunInMsg.subscribeTo(spice_object.planetStateOutMsgs[sun_spice_index])
    eclipse_module.addPlanetToModel(spice_object.planetStateOutMsgs[earth_spice_index])
    eclipse_module.addSpacecraftToModel(sc_object.scStateOutMsg)
    sc_sim.AddModelToTask(task_name, eclipse_module)

    sample_ns = int(max(sample_step_s, 1.0) * 1e9)
    eclipse_rec = eclipse_module.eclipseOutMsgs[0].recorder(sample_ns)
    sc_sim.AddModelToTask(task_name, eclipse_rec)
    return {
        "eclipse_module": eclipse_module,
        "eclipse_msg": eclipse_module.eclipseOutMsgs[0],
        "eclipse_rec": eclipse_rec,
    }


def latest_eclipse_factor(eclipse_rec, default: float = 1.0) -> float:
    """Return most recent eclipse illumination factor from a recorder."""
    if eclipse_rec is None:
        return float(default)
    try:
        factors = eclipse_rec.illuminationFactor
    except AttributeError:
        return float(default)
    if factors is None or len(factors) == 0:
        return float(default)
    arr = factors[-1]
    if hasattr(arr, "__len__"):
        return float(arr[0])
    return float(arr)
