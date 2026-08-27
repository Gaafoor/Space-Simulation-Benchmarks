"""Hardware sensor models for Basilisk handoff GNC."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from Basilisk.simulation import coarseSunSensor, imuSensor, starTracker, spacecraft

from gnc_stack import NavNoiseConfig


@dataclass(frozen=True)
class SensorStackConfig:
    star_tracker_noise_sigma_deg: float = 0.001
    star_tracker_walk_bound_deg: float = 0.01
    imu_rate_noise_sigma_deg_s: float = 0.0005
    imu_rate_walk_bound_deg_s: float = 0.005
    css_noise_sigma: float = 0.01
    enable_css: bool = False


def attach_sensor_stack(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    *,
    nav_noise: NavNoiseConfig | None = None,
    sensor_cfg: SensorStackConfig | None = None,
    sample_step_s: float = 60.0,
) -> dict:
    noise = nav_noise or NavNoiseConfig()
    cfg = sensor_cfg or SensorStackConfig(
        star_tracker_noise_sigma_deg=float(noise.attitude_noise_sigma_deg),
        star_tracker_walk_bound_deg=float(noise.attitude_error_3sigma_deg),
        imu_rate_noise_sigma_deg_s=float(noise.rate_noise_sigma_deg_s),
        imu_rate_walk_bound_deg_s=float(noise.rate_error_3sigma_deg_s),
    )

    star = starTracker.StarTracker()
    star.ModelTag = "HandoffStarTracker"
    star.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    att_sigma = math.radians(cfg.star_tracker_noise_sigma_deg)
    att_bound = math.radians(cfg.star_tracker_walk_bound_deg)
    star.PMatrix = np.diag([att_sigma, att_sigma, att_sigma])
    star.setWalkBounds([att_bound, att_bound, att_bound])
    star.setAMatrix(np.eye(3))

    imu = imuSensor.ImuSensor()
    imu.ModelTag = "HandoffImu"
    imu.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
    rate_sigma = math.radians(cfg.imu_rate_noise_sigma_deg_s)
    rate_bound = math.radians(cfg.imu_rate_walk_bound_deg_s)
    imu.PMatrixGyro = np.diag([rate_sigma, rate_sigma, rate_sigma])
    imu.setWalkBoundsGyro(np.array([rate_bound, rate_bound, rate_bound], dtype=float))
    imu.setAMatrixGyro(np.eye(3))
    imu.applySensorErrors = True

    sample_ns = int(max(sample_step_s, 0.5) * 1e9)
    star_log = star.sensorOutMsg.recorder(sample_ns)
    imu_log = imu.sensorOutMsg.recorder(sample_ns)

    css = None
    css_log = None
    if cfg.enable_css:
        css = coarseSunSensor.CoarseSunSensor()
        css.ModelTag = "HandoffCss"
        css.scStateInMsg.subscribeTo(sc_object.scStateOutMsg)
        css.nHat_B = [0.0, 0.0, 1.0]
        css.PMatrix = np.diag([cfg.css_noise_sigma, cfg.css_noise_sigma, cfg.css_noise_sigma])
        css_log = css.sensorOutMsg.recorder(sample_ns)
        sc_sim.AddModelToTask(task_name, css, 107)

    sc_sim.AddModelToTask(task_name, star, 108)
    sc_sim.AddModelToTask(task_name, imu, 107)
    sc_sim.AddModelToTask(task_name, star_log)
    sc_sim.AddModelToTask(task_name, imu_log)

    out = {
        "star_tracker": star,
        "imu": imu,
        "star_log": star_log,
        "imu_log": imu_log,
        "sensor_config": cfg,
    }
    if css is not None:
        out["css"] = css
        out["css_log"] = css_log
    return out
