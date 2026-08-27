"""Force-model setup for Basilisk handoff runs."""

from __future__ import annotations

from dataclasses import dataclass

from Basilisk.architecture import messaging
from Basilisk.simulation import (
    dragDynamicEffector,
    exponentialAtmosphere,
    msisAtmosphere,
    radiationPressure,
    spacecraft,
    zeroWindModel,
)
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import simIncludeGravBody, simSetPlanetEnvironment
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

from a3_leo_geo_config import A3LeoGeoConfig
from spacecraft_geometry import attach_facet_forces, load_geometry

# Basilisk ships GGM2B (JGM-2 family) in JPL harmonic format. GMAT uses JGM2.cof
# (different file layout); degree/order 20 is the matched truncation.
JGM2_EQUIV_MODEL = "GGM2BData"


@dataclass(frozen=True)
class A3ForceModelSpec:
    gravity: str
    drag: bool
    srp: bool
    third_bodies: tuple[str, ...]
    spice: bool
    atmosphere: str = "none"
    geometry: str = "cannonball"


_SPICE_KERNELS_LOADED = False


def load_spice_kernels() -> None:
    global _SPICE_KERNELS_LOADED
    if _SPICE_KERNELS_LOADED:
        return
    for kernel in (
        DataFile.EphemerisData.de430,
        DataFile.EphemerisData.naif0012,
        DataFile.EphemerisData.de_403_masses,
        DataFile.EphemerisData.pck00010,
    ):
        pyswice.furnsh_c(str(get_path(kernel)))
    _SPICE_KERNELS_LOADED = True


def force_model_spec(config: A3LeoGeoConfig, *, mode: str, facet_physics: bool = False) -> A3ForceModelSpec:
    geometry = "facet" if facet_physics else "cannonball"
    if mode == "j2_only":
        return A3ForceModelSpec("J2 only", False, False, (), False, geometry=geometry)
    if mode == "legacy_ggm03s":
        return A3ForceModelSpec(
            f"GGM03S {config.gravity_max_degree}",
            True,
            True,
            ("Sun", "Moon"),
            True,
            atmosphere="exponential",
            geometry=geometry,
        )
    return A3ForceModelSpec(
        f"JGM2-equivalent {JGM2_EQUIV_MODEL} {config.gravity_max_degree}x{config.gravity_max_degree}",
        True,
        True,
        ("Sun", "Moon"),
        True,
        atmosphere="NRLMSISE-00 (MsisAtmosphere)",
        geometry=geometry,
    )


def _msis_solar_weather_messages(f107: float, ap: float) -> list:
    sw_msg = {
        "ap_24_0": ap,
        "ap_3_0": ap,
        "ap_3_-3": ap,
        "ap_3_-6": ap,
        "ap_3_-9": ap,
        "ap_3_-12": ap,
        "ap_3_-15": ap,
        "ap_3_-18": ap,
        "ap_3_-21": ap,
        "ap_3_-24": ap,
        "ap_3_-27": ap,
        "ap_3_-30": ap,
        "ap_3_-33": ap,
        "ap_3_-36": ap,
        "ap_3_-39": ap,
        "ap_3_-42": ap,
        "ap_3_-45": ap,
        "ap_3_-48": ap,
        "ap_3_-51": ap,
        "ap_3_-54": ap,
        "ap_3_-57": ap,
        "f107_1944_0": f107,
        "f107_24_-24": f107,
    }
    out = []
    for val in sw_msg.values():
        out.append(messaging.SwDataMsg().write(messaging.SwDataMsgPayload(dataValue=float(val))))
    return out


def _attach_msis_atmosphere(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    config: A3LeoGeoConfig,
    grav_factory: simIncludeGravBody.gravBodyFactory,
    spice_object,
    earth_spice_index: int,
) -> msisAtmosphere.MsisAtmosphere:
    atmosphere = msisAtmosphere.MsisAtmosphere()
    atmosphere.ModelTag = "HandoffMsisAtmosphere"
    for idx, sw_msg in enumerate(_msis_solar_weather_messages(config.msis_f107, config.msis_ap)):
        atmosphere.swDataInMsgs[idx].subscribeTo(sw_msg)
    atmosphere.epochInMsg.subscribeTo(grav_factory.epochMsg)
    atmosphere.addSpacecraftToModel(sc_object.scStateOutMsg)
    sc_sim.AddModelToTask(task_name, atmosphere)

    wind_model = zeroWindModel.ZeroWindModel()
    wind_model.ModelTag = "HandoffZeroWind"
    wind_model.planetPosInMsg.subscribeTo(spice_object.planetStateOutMsgs[earth_spice_index])
    wind_model.addSpacecraftToModel(sc_object.scStateOutMsg)
    sc_sim.AddModelToTask(task_name, wind_model)
    return atmosphere, wind_model


def attach_a3_force_model(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    config: A3LeoGeoConfig,
    *,
    mode: str = "gmat_match",
    facet_physics: bool = False,
    eclipse_msg=None,
) -> dict:
    grav_factory = simIncludeGravBody.gravBodyFactory()
    spice_object = None
    drag_effector = None
    srp_effector = None
    facet_bundle = None
    atmosphere = None
    wind_model = None
    earth_spice_index = 1

    if mode in {"gmat_match", "legacy_ggm03s"}:
        load_spice_kernels()
        grav_bodies = grav_factory.createBodies(["sun", "earth", "moon"])
        earth = grav_bodies["earth"]
        earth.isCentralBody = True
        if mode == "legacy_ggm03s":
            earth.useSphericalHarmonicsGravityModel(
                str(get_path(getattr(DataFile.LocalGravData, "GGM03S"))),
                config.gravity_max_degree,
            )
        else:
            earth.useSphericalHarmonicsGravityModel(
                str(get_path(getattr(DataFile.LocalGravData, JGM2_EQUIV_MODEL))),
                config.gravity_max_degree,
            )
        spice_object = grav_factory.createSpiceInterface(time=config.spice_epoch_utc, epochInMsg=True)
        spice_object.zeroBase = "earth"
        spice_object.referenceBase = "J2000"
        sc_sim.AddModelToTask(task_name, spice_object)

        if mode == "legacy_ggm03s":
            atmosphere = exponentialAtmosphere.ExponentialAtmosphere()
            atmosphere.ModelTag = "HandoffAtmosphere"
            simSetPlanetEnvironment.exponentialAtmosphere(atmosphere, "earth")
            atmosphere.addSpacecraftToModel(sc_object.scStateOutMsg)
            sc_sim.AddModelToTask(task_name, atmosphere)
        else:
            atmosphere, wind_model = _attach_msis_atmosphere(
                sc_sim,
                task_name,
                sc_object,
                config,
                grav_factory,
                spice_object,
                earth_spice_index,
            )

        if facet_physics:
            geom = load_geometry(config, level=config.geometry_level)
            facet_bundle = attach_facet_forces(
                sc_object,
                geom,
                atmosphere_msg=atmosphere.envOutMsgs[0],
                sun_msg=spice_object.planetStateOutMsgs[0],
                eclipse_msg=eclipse_msg,
            )
            sc_sim.AddModelToTask(task_name, facet_bundle["drag_effector"])
            sc_sim.AddModelToTask(task_name, facet_bundle["srp_effector"])
        else:
            drag_effector = dragDynamicEffector.DragDynamicEffector()
            drag_effector.ModelTag = "HandoffDrag"
            drag_effector.coreParams.projectedArea = config.drag_area_m2
            drag_effector.coreParams.dragCoeff = config.drag_coefficient
            drag_effector.atmoDensInMsg.subscribeTo(atmosphere.envOutMsgs[0])
            if wind_model is not None:
                drag_effector.windVelInMsg.subscribeTo(wind_model.envOutMsgs[0])
            sc_object.addDynamicEffector(drag_effector)

            srp_effector = radiationPressure.RadiationPressure()
            srp_effector.ModelTag = "HandoffSRP"
            srp_effector.area = config.srp_area_m2
            srp_effector.coefficientReflection = config.srp_reflectivity
            srp_effector.setUseCannonballModel()
            srp_effector.sunEphmInMsg.subscribeTo(spice_object.planetStateOutMsgs[0])
            if eclipse_msg is not None:
                srp_effector.sunEclipseInMsg.subscribeTo(eclipse_msg)
            sc_object.addDynamicEffector(srp_effector)
    else:
        earth = grav_factory.createEarth()
        earth.isCentralBody = True
        earth.useSphericalHarmonicsGravityModel(
            str(get_path(getattr(DataFile.LocalGravData, "GGM03S_J2_only"))),
            config.j2_max_degree,
        )

    grav_factory.addBodiesTo(sc_object)
    return {
        "earth": earth,
        "grav_factory": grav_factory,
        "spice_object": spice_object,
        "drag_effector": drag_effector,
        "srp_effector": srp_effector,
        "atmosphere": atmosphere,
        "wind_model": wind_model,
        "facet_bundle": facet_bundle,
        "spec": force_model_spec(config, mode=mode, facet_physics=facet_physics),
    }
