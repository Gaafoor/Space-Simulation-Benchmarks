"""A3 spacecraft facet geometry for Basilisk facet drag/SRP."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from Basilisk.simulation import facetDragDynamicEffector, facetSRPDynamicEffector, spacecraft
from Basilisk.utilities import RigidBodyKinematics as rbk

from a3_leo_geo_config import A3LeoGeoConfig


@dataclass(frozen=True)
class FacetSpec:
    area_m2: float
    normal_body: tuple[float, float, float]
    location_body_m: tuple[float, float, float]
    drag_coeff: float
    srp_specular: float
    srp_diffuse: float


@dataclass(frozen=True)
class A3GeometrySpec:
    level: str
    facets: tuple[FacetSpec, ...]


def _unit(vec: tuple[float, float, float]) -> np.ndarray:
    arr = np.asarray(vec, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return np.array([1.0, 0.0, 0.0])
    return arr / norm


def _dcm_f0b_from_body_normal(normal_body: np.ndarray) -> np.ndarray:
    """Map facet +Y axis to the requested body-frame normal."""
    target = _unit(tuple(normal_body.tolist()))
    ref = np.array([0.0, 1.0, 0.0])
    cross = np.cross(ref, target)
    dot = float(np.clip(np.dot(ref, target), -1.0, 1.0))
    if float(np.linalg.norm(cross)) <= 1e-12:
        if dot > 0.0:
            return np.eye(3)
        return rbk.PRV2C(np.array([math.pi, 0.0, 0.0]))
    axis = cross / np.linalg.norm(cross)
    angle = math.acos(dot)
    return rbk.PRV2C(axis * angle)


def geometry_level_l1(config: A3LeoGeoConfig) -> A3GeometrySpec:
    """Box hub (6 facets) + 2 solar-array facets (L1 sanity model)."""
    hx, hy, hz = 0.50, 0.40, 0.30
    panel_area = 0.35
    facets = (
        FacetSpec(hx * hy, (0.0, 0.0, 1.0), (0.0, 0.0, 0.5 * hz), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(hx * hy, (0.0, 0.0, -1.0), (0.0, 0.0, -0.5 * hz), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(hx * hz, (0.0, 1.0, 0.0), (0.0, 0.5 * hy, 0.0), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(hx * hz, (0.0, -1.0, 0.0), (0.0, -0.5 * hy, 0.0), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(hy * hz, (1.0, 0.0, 0.0), (0.5 * hx, 0.0, 0.0), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(hy * hz, (-1.0, 0.0, 0.0), (-0.5 * hx, 0.0, 0.0), config.drag_coefficient, 0.85, 0.15),
        FacetSpec(panel_area, (0.0, 1.0, 0.0), (0.0, 0.55, 0.0), config.drag_coefficient * 0.8, 0.80, 0.20),
        FacetSpec(panel_area, (0.0, -1.0, 0.0), (0.0, -0.55, 0.0), config.drag_coefficient * 0.8, 0.80, 0.20),
    )
    return A3GeometrySpec(level="L1", facets=facets)


def geometry_level_l2(config: A3LeoGeoConfig) -> A3GeometrySpec:
    """Expanded 20-facet approximation with appendages."""
    base = list(geometry_level_l1(config).facets)
    extras = (
        FacetSpec(0.03, (1.0, 0.0, 0.0), (0.30, 0.10, 0.0), config.drag_coefficient, 0.70, 0.30),
        FacetSpec(0.03, (-1.0, 0.0, 0.0), (-0.30, -0.10, 0.0), config.drag_coefficient, 0.70, 0.30),
        FacetSpec(0.02, (0.0, 0.0, 1.0), (0.10, 0.0, 0.20), config.drag_coefficient, 0.70, 0.30),
        FacetSpec(0.02, (0.0, 0.0, -1.0), (-0.10, 0.0, -0.20), config.drag_coefficient, 0.70, 0.30),
        FacetSpec(0.04, (0.0, 1.0, 0.0), (0.15, 0.35, 0.05), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.04, (0.0, -1.0, 0.0), (-0.15, -0.35, 0.05), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.025, (1.0, 1.0, 0.0), (0.20, 0.20, 0.0), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.025, (-1.0, 1.0, 0.0), (-0.20, 0.20, 0.0), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.025, (1.0, -1.0, 0.0), (0.20, -0.20, 0.0), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.025, (-1.0, -1.0, 0.0), (-0.20, -0.20, 0.0), config.drag_coefficient, 0.75, 0.25),
        FacetSpec(0.015, (0.0, 1.0, 1.0), (0.0, 0.45, 0.10), config.drag_coefficient, 0.80, 0.20),
        FacetSpec(0.015, (0.0, -1.0, 1.0), (0.0, -0.45, 0.10), config.drag_coefficient, 0.80, 0.20),
    )
    base.extend(extras)
    return A3GeometrySpec(level="L2", facets=tuple(base))


def load_geometry(config: A3LeoGeoConfig, *, level: str = "L1") -> A3GeometrySpec:
    if level.upper() == "L2":
        return geometry_level_l2(config)
    return geometry_level_l1(config)


def attach_facet_forces(
    sc_object: spacecraft.Spacecraft,
    spec: A3GeometrySpec,
    *,
    atmosphere_msg,
    sun_msg,
    eclipse_msg=None,
) -> dict:
    drag = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag.ModelTag = "HandoffFacetDrag"
    drag.atmoDensInMsg.subscribeTo(atmosphere_msg)
    srp = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    srp.ModelTag = "HandoffFacetSRP"
    srp.setNumFacets(len(spec.facets))
    srp.setNumArticulatedFacets(0)
    srp.sunInMsg.subscribeTo(sun_msg)
    if eclipse_msg is not None:
        srp.sunEclipseInMsg.subscribeTo(eclipse_msg)

    for facet in spec.facets:
        normal = _unit(facet.normal_body)
        location = np.asarray(facet.location_body_m, dtype=float)
        drag.addFacet(float(facet.area_m2), float(facet.drag_coeff), normal, location)
        dcm_f0b = _dcm_f0b_from_body_normal(normal)
        n_hat_f = np.array([0.0, 1.0, 0.0])
        rot_hat_f = np.zeros(3)
        srp.addFacet(
            float(facet.area_m2),
            dcm_f0b,
            n_hat_f,
            rot_hat_f,
            location,
            float(facet.srp_diffuse),
            float(facet.srp_specular),
        )

    sc_object.addDynamicEffector(drag)
    sc_object.addDynamicEffector(srp)
    return {"drag_effector": drag, "srp_effector": srp, "geometry_spec": spec}
