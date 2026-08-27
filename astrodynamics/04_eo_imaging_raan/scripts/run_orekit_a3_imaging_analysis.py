#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from datetime import datetime, timedelta
import csv
import json
import math
import time

import matplotlib
import numpy as np
import orekit_jpype

matplotlib.use("Agg")
import matplotlib.pyplot as plt

orekit_jpype.initVM()

from orekit_jpype.pyhelpers import setup_orekit_curdir
from org.hipparchus.ode.nonstiff import DormandPrince853Integrator
from org.hipparchus.util import FastMath
from org.orekit.forces.gravity import HolmesFeatherstoneAttractionModel
from org.orekit.forces.gravity.potential import GravityFieldFactory
from org.orekit.frames import FramesFactory
from org.orekit.orbits import KeplerianOrbit, OrbitType, PositionAngleType
from org.orekit.propagation import SpacecraftState
from org.orekit.propagation.numerical import NumericalPropagator
from org.orekit.time import AbsoluteDate, TimeScalesFactory
from org.orekit.utils import Constants, IERSConventions

from a3_imaging_geometry import (
    A3ImagingConfig,
    MISSION_EPOCH_UTC,
    compute_imaging_timeline,
    compute_metrics,
    extract_windows,
    gmst_rad,
)


SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
OUTPUT_DIR = SCRIPT_DIR / "results/orekit_a3"
OREKIT_DATA_ZIP = WORKSPACE_ROOT / "orekit-data.zip"

RAAN_DEG = 61.0
ALTITUDE_M = 500000.0
SIM_DURATION_S = 24.0 * 3600.0
STEP_S = 10.0
EARTH_RADIUS_KM = 6378.137
MADRID_LAT_MIN_DEG = 40.2
MADRID_LAT_MAX_DEG = 40.7
MADRID_LON_MIN_DEG = -4.2
MADRID_LON_MAX_DEG = -3.2
MADRID_GRID_LAT_DIVISIONS = 100
MADRID_GRID_LON_DIVISIONS = 100


def init_orekit() -> None:
    setup_orekit_curdir(str(OREKIT_DATA_ZIP))


def create_initial_orbit():
    utc = TimeScalesFactory.getUTC()
    date = AbsoluteDate(2026, 6, 2, 0, 0, 0.0, utc)
    frame = FramesFactory.getEME2000()
    sma_m = Constants.WGS84_EARTH_EQUATORIAL_RADIUS + ALTITUDE_M

    return KeplerianOrbit(
        sma_m,
        0.0,
        FastMath.toRadians(97.4),
        0.0,
        FastMath.toRadians(RAAN_DEG),
        0.0,
        PositionAngleType.TRUE,
        frame,
        date,
        Constants.EIGEN5C_EARTH_MU,
    )


def run_orekit():
    orbit = create_initial_orbit()
    frame = FramesFactory.getEME2000()
    earth_frame = FramesFactory.getITRF(IERSConventions.IERS_2010, True)
    provider = GravityFieldFactory.getNormalizedProvider(2, 0)

    integrator = DormandPrince853Integrator(0.1, 300.0, 1.0, 1.0)
    propagator = NumericalPropagator(integrator)
    propagator.setOrbitType(OrbitType.CARTESIAN)
    propagator.setInitialState(SpacecraftState(orbit))
    propagator.addForceModel(HolmesFeatherstoneAttractionModel(earth_frame, provider))

    date0 = orbit.getDate()
    sample_count = int(SIM_DURATION_S / STEP_S) + 1
    times = np.zeros(sample_count, dtype=float)
    positions = np.zeros((sample_count, 3), dtype=float)

    for idx in range(sample_count):
        elapsed_s = idx * STEP_S
        state = propagator.propagate(date0.shiftedBy(float(elapsed_s)))
        pv = state.getPVCoordinates(frame).getPosition()
        times[idx] = elapsed_s
        positions[idx, 0] = float(pv.getX())
        positions[idx, 1] = float(pv.getY())
        positions[idx, 2] = float(pv.getZ())

    return times, positions


def eci_to_ecef(position_eci_m, time_s):
    theta = gmst_rad(MISSION_EPOCH_UTC + timedelta(seconds=float(time_s)))
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    rotation = np.array(
        [
            [cos_t, sin_t, 0.0],
            [-sin_t, cos_t, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rotation @ position_eci_m


def eci_to_subpoint_lat_lon(position_eci_m, time_s):
    position_ecef_m = eci_to_ecef(position_eci_m, time_s)
    radius = np.linalg.norm(position_ecef_m)
    lat = math.degrees(math.asin(position_ecef_m[2] / radius))
    lon = math.degrees(math.atan2(position_ecef_m[1], position_ecef_m[0]))
    return lat, lon


def compute_ground_track(times, positions):
    rows = []
    for time_s, position in zip(times, positions):
        lat, lon = eci_to_subpoint_lat_lon(position, time_s)
        rows.append(
            {
                "time_s": float(time_s),
                "lat_deg": lat,
                "lon_deg": lon,
            }
        )
    return rows


def split_ground_track_for_dateline(ground_track):
    segments = []
    current = []
    previous_lon = None

    for row in ground_track:
        lon = row["lon_deg"]
        if previous_lon is not None and abs(lon - previous_lon) > 180.0:
            if current:
                segments.append(current)
            current = []
        current.append(row)
        previous_lon = lon

    if current:
        segments.append(current)

    return segments


def destination_point(lat_deg, lon_deg, bearing_rad, distance_km):
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    angular_distance = distance_km / EARTH_RADIUS_KM
    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular_distance)
        + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1),
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2),
    )

    lon2_deg = (math.degrees(lon2) + 180.0) % 360.0 - 180.0
    return math.degrees(lat2), lon2_deg


def ground_distance_km(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    hav = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_KM * math.asin(min(1.0, math.sqrt(hav)))


def save_timeline_csv(timeline):
    path = OUTPUT_DIR / "timeline.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(timeline[0].keys()))
        writer.writeheader()
        writer.writerows(timeline)
    return path


def save_windows_csv(windows):
    path = OUTPUT_DIR / "imaging_windows.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "start_s",
                "stop_s",
                "duration_s",
            ],
        )
        writer.writeheader()
        writer.writerows(windows)
    return path


def save_revisit_csv(windows):
    path = OUTPUT_DIR / "revisit_windows.csv"
    revisit_rows = []
    for idx in range(1, len(windows)):
        revisit_s = windows[idx]["start_s"] - windows[idx - 1]["start_s"]
        revisit_rows.append(
            {
                "window_index_prev": idx,
                "window_index_next": idx + 1,
                "revisit_s": revisit_s,
                "revisit_min": revisit_s / 60.0,
            }
        )
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "window_index_prev",
                "window_index_next",
                "revisit_s",
                "revisit_min",
            ],
        )
        writer.writeheader()
        writer.writerows(revisit_rows)
    return path, revisit_rows


def save_off_nadir_plot(timeline):
    path = OUTPUT_DIR / "off_nadir_angle_vs_time.png"
    times_h = [r["time_s"] / 3600.0 for r in timeline]
    angles = [r["boresight_angle_deg"] for r in timeline]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times_h, angles, linewidth=0.8)
    ax.axhline(5.0, linestyle="--", linewidth=1.0, label="5 deg threshold")
    ax.set_title(f"Orekit A3 Off-Nadir Angle vs Time (RAAN={RAAN_DEG:g} deg)")
    ax.set_xlabel("Elapsed Time [hours]")
    ax.set_ylabel("Off-Nadir Angle [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_elevation_plot(timeline):
    path = OUTPUT_DIR / "elevation_vs_time.png"
    times_h = [r["time_s"] / 3600.0 for r in timeline]
    elevations = [r["target_elevation_deg"] for r in timeline]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times_h, elevations, linewidth=0.8)
    ax.axhline(0.0, linestyle="--", linewidth=1.0, label="Horizon")
    ax.set_title(f"Orekit A3 Target Elevation vs Time (RAAN={RAAN_DEG:g} deg)")
    ax.set_xlabel("Elapsed Time [hours]")
    ax.set_ylabel("Madrid Elevation [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_ground_track_map(times, positions, timeline):
    path = OUTPUT_DIR / "ground_track_madrid_imaging_map.png"
    config = A3ImagingConfig()
    ground_track = compute_ground_track(times, positions)
    imaging_track = [
        ground_track[index]
        for index, row in enumerate(timeline)
        if row["target_in_camera_fov"]
    ]

    fig, ax = plt.subplots(figsize=(13, 6))
    segments = split_ground_track_for_dateline(ground_track)
    for index, segment in enumerate(segments):
        ax.plot(
            [row["lon_deg"] for row in segment],
            [row["lat_deg"] for row in segment],
            color="tab:blue",
            linewidth=0.8,
            alpha=0.8,
            label="Satellite ground track" if index == 0 else None,
        )

    if imaging_track:
        ax.scatter(
            [row["lon_deg"] for row in imaging_track],
            [row["lat_deg"] for row in imaging_track],
            color="tab:orange",
            s=20,
            label="Imaging samples",
            zorder=4,
        )

    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=90,
        linewidths=2,
        label="Madrid",
        zorder=5,
    )
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 30))
    ax.set_yticks(np.arange(-90, 91, 15))
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(f"Orekit A3 Ground Track and Madrid Imaging Opportunity (RAAN={RAAN_DEG:g} deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_camera_footprint_plot(times, positions, timeline):
    path = OUTPUT_DIR / "camera_footprint_best_instant.png"
    config = A3ImagingConfig()
    best_index, best = min(enumerate(timeline), key=lambda item: item[1]["boresight_angle_deg"])
    subpoint_lat, subpoint_lon = eci_to_subpoint_lat_lon(positions[best_index], times[best_index])
    footprint_radius_km = config.footprint_diameter_km / 2.0
    madrid_distance_km = ground_distance_km(
        subpoint_lat,
        subpoint_lon,
        config.madrid_lat_deg,
        config.madrid_lon_deg,
    )
    madrid_inside = madrid_distance_km <= footprint_radius_km

    bearings = np.linspace(0.0, 2.0 * math.pi, 181)
    circle = [
        destination_point(subpoint_lat, subpoint_lon, bearing, footprint_radius_km)
        for bearing in bearings
    ]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(
        [point[1] for point in circle],
        [point[0] for point in circle],
        color="tab:orange",
        linewidth=2,
        label=f"Camera footprint radius ({footprint_radius_km:.1f} km)",
    )
    ax.scatter([subpoint_lon], [subpoint_lat], color="tab:blue", s=70, label="Satellite subpoint", zorder=5)
    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=100,
        linewidths=2,
        label="Madrid",
        zorder=6,
    )

    margin_deg = max(0.5, footprint_radius_km / 111.0 * 2.5)
    center_lat = (subpoint_lat + config.madrid_lat_deg) / 2.0
    center_lon = (subpoint_lon + config.madrid_lon_deg) / 2.0
    ax.set_xlim(center_lon - margin_deg, center_lon + margin_deg)
    ax.set_ylim(center_lat - margin_deg, center_lat + margin_deg)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "Orekit A3 Camera Footprint at Best Imaging Instant\n"
        f"t={best['time_s']:.1f} s, off-nadir={best['boresight_angle_deg']:.3f} deg, "
        f"Madrid inside footprint={madrid_inside}"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.text(
        0.02,
        0.02,
        f"Madrid distance from subpoint: {madrid_distance_km:.1f} km\n"
        f"Footprint radius: {footprint_radius_km:.1f} km",
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "0.7"},
    )
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def create_madrid_coverage_grid():
    lat_edges = np.linspace(MADRID_LAT_MIN_DEG, MADRID_LAT_MAX_DEG, MADRID_GRID_LAT_DIVISIONS + 1)
    lon_edges = np.linspace(MADRID_LON_MIN_DEG, MADRID_LON_MAX_DEG, MADRID_GRID_LON_DIVISIONS + 1)
    lat_centers = (lat_edges[:-1] + lat_edges[1:]) / 2.0
    lon_centers = (lon_edges[:-1] + lon_edges[1:]) / 2.0
    return lat_edges, lon_edges, lat_centers, lon_centers


def accumulate_madrid_coverage(times, positions):
    config = A3ImagingConfig()
    _, _, lat_centers, lon_centers = create_madrid_coverage_grid()
    coverage_count = np.zeros((MADRID_GRID_LAT_DIVISIONS, MADRID_GRID_LON_DIVISIONS), dtype=int)
    footprint_radius_km = config.footprint_diameter_km / 2.0
    lat_grid_deg, lon_grid_deg = np.meshgrid(lat_centers, lon_centers, indexing="ij")
    lat_grid_rad = np.deg2rad(lat_grid_deg)
    lon_grid_rad = np.deg2rad(lon_grid_deg)

    subpoints = [
        eci_to_subpoint_lat_lon(position, time_s)
        for time_s, position in zip(times, positions)
    ]

    for subpoint_lat, subpoint_lon in subpoints:
        subpoint_lat_rad = math.radians(subpoint_lat)
        subpoint_lon_rad = math.radians(subpoint_lon)
        dlat = lat_grid_rad - subpoint_lat_rad
        dlon = lon_grid_rad - subpoint_lon_rad
        hav = (
            np.sin(dlat / 2.0) ** 2
            + math.cos(subpoint_lat_rad) * np.cos(lat_grid_rad) * np.sin(dlon / 2.0) ** 2
        )
        distances_km = 2.0 * EARTH_RADIUS_KM * np.arcsin(np.minimum(1.0, np.sqrt(hav)))
        coverage_count += (distances_km <= footprint_radius_km).astype(int)

    return {
        "lat_centers": lat_centers,
        "lon_centers": lon_centers,
        "coverage_count": coverage_count,
        "footprint_radius_km": footprint_radius_km,
    }


def save_coverage_heatmap(coverage):
    path = OUTPUT_DIR / "coverage_heatmap.png"
    config = A3ImagingConfig()
    coverage_count = coverage["coverage_count"]

    fig, ax = plt.subplots(figsize=(10, 7))
    image = ax.imshow(
        coverage_count,
        origin="lower",
        extent=[
            MADRID_LON_MIN_DEG,
            MADRID_LON_MAX_DEG,
            MADRID_LAT_MIN_DEG,
            MADRID_LAT_MAX_DEG,
        ],
        aspect="auto",
        cmap="viridis",
    )
    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=90,
        linewidths=2,
        label="Madrid center",
        zorder=5,
    )
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label("Coverage count [samples]")
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "Orekit A3 Madrid Regional Coverage Heatmap\n"
        "100 x 100 grid, coverage accumulated from nadir camera footprint"
    )
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_coverage_statistics_csv(coverage):
    path = OUTPUT_DIR / "madrid_coverage_statistics.csv"
    coverage_count = coverage["coverage_count"]
    lat_centers = coverage["lat_centers"]
    lon_centers = coverage["lon_centers"]

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "lat_index",
                "lon_index",
                "lat_center_deg",
                "lon_center_deg",
                "coverage_count",
                "covered",
            ],
        )
        writer.writeheader()
        for lat_index, lat in enumerate(lat_centers):
            for lon_index, lon in enumerate(lon_centers):
                count = int(coverage_count[lat_index, lon_index])
                writer.writerow(
                    {
                        "lat_index": lat_index,
                        "lon_index": lon_index,
                        "lat_center_deg": lat,
                        "lon_center_deg": lon,
                        "coverage_count": count,
                        "covered": int(count > 0),
                    }
                )
    return path


def save_coverage_percentage_json(coverage, heatmap_path, statistics_path):
    path = OUTPUT_DIR / "coverage_percentage.json"
    coverage_count = coverage["coverage_count"]
    total_cells = int(coverage_count.size)
    covered_cells = int(np.count_nonzero(coverage_count > 0))
    max_count = int(coverage_count.max())
    mean_count = float(coverage_count.mean())
    coverage_percentage = 100.0 * covered_cells / total_cells if total_cells else 0.0

    summary = {
        "region": {
            "lat_min_deg": MADRID_LAT_MIN_DEG,
            "lat_max_deg": MADRID_LAT_MAX_DEG,
            "lon_min_deg": MADRID_LON_MIN_DEG,
            "lon_max_deg": MADRID_LON_MAX_DEG,
        },
        "grid": {
            "lat_divisions": MADRID_GRID_LAT_DIVISIONS,
            "lon_divisions": MADRID_GRID_LON_DIVISIONS,
            "total_cells": total_cells,
        },
        "coverage": {
            "covered_cells": covered_cells,
            "coverage_percentage": coverage_percentage,
            "max_coverage_count": max_count,
            "mean_coverage_count": mean_count,
            "footprint_radius_km": coverage["footprint_radius_km"],
        },
        "artifacts": {
            "coverage_heatmap": str(heatmap_path),
            "coverage_statistics_csv": str(statistics_path),
        },
    }

    with path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    return path, summary


def save_summary(
    timeline,
    windows,
    revisit_rows,
    plot_path,
    elevation_plot_path,
    ground_track_map_path,
    footprint_plot_path,
    coverage_products,
    runtime_seconds,
):
    best = min(timeline, key=lambda r: r["boresight_angle_deg"])
    aggregate = compute_metrics(timeline, windows, A3ImagingConfig())

    summary = {
        "RAAN_deg": RAAN_DEG,
        "minimum_off_nadir_angle_deg": best["boresight_angle_deg"],
        "minimum_off_nadir_time_s": best["time_s"],
        "imaging_opportunity_count": len(windows),
        "imaging_sample_count": sum(1 for row in timeline if row["target_in_camera_fov"]),
        "aggregate_metrics": aggregate,
        "revisit_windows": revisit_rows,
        "off_nadir_plot": str(plot_path),
        "elevation_plot": str(elevation_plot_path),
        "ground_track_map": str(ground_track_map_path),
        "camera_footprint_plot": str(footprint_plot_path),
        "coverage_heatmap": str(coverage_products["coverage_heatmap"]),
        "coverage_statistics_csv": str(coverage_products["coverage_statistics_csv"]),
        "coverage_percentage_json": str(coverage_products["coverage_percentage_json"]),
        "madrid_regional_coverage": coverage_products["coverage_summary"],
        "runtime_seconds": runtime_seconds,
        "implementation_notes": {
            "workflow_type": "Orekit numerical propagation + Python geometry post-processing",
            "orekit_data_setup": "Requires orekit-data.zip and JVM (OpenJDK) before running",
            "gravity_model": "Orekit normalized harmonics 2x0 (J2-only style policy)",
            "native_sensor_contact": "Not used; A3 FOV geometry computed explicitly in Python",
        },
    }

    path = OUTPUT_DIR / "summary.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    return path


def main():
    if not OREKIT_DATA_ZIP.exists():
        raise FileNotFoundError(
            f"Missing {OREKIT_DATA_ZIP}. Download with orekit_jpype.pyhelpers.download_orekit_data_curdir()."
        )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    total_start_s = time.perf_counter()
    init_start_s = time.perf_counter()
    init_orekit()
    init_elapsed_s = time.perf_counter() - init_start_s

    print("Running Orekit A3...")

    propagate_start_s = time.perf_counter()
    times, positions = run_orekit()
    propagate_elapsed_s = time.perf_counter() - propagate_start_s

    post_start_s = time.perf_counter()
    timeline = compute_imaging_timeline(times, positions, A3ImagingConfig())
    windows = extract_windows(timeline)

    save_timeline_csv(timeline)
    save_windows_csv(windows)
    revisit_csv_path, revisit_rows = save_revisit_csv(windows)
    plot_path = save_off_nadir_plot(timeline)
    elevation_plot_path = save_elevation_plot(timeline)
    ground_track_map_path = save_ground_track_map(times, positions, timeline)
    footprint_plot_path = save_camera_footprint_plot(times, positions, timeline)

    coverage = accumulate_madrid_coverage(times, positions)
    coverage_heatmap_path = save_coverage_heatmap(coverage)
    coverage_statistics_path = save_coverage_statistics_csv(coverage)
    coverage_percentage_path, coverage_summary = save_coverage_percentage_json(
        coverage,
        coverage_heatmap_path,
        coverage_statistics_path,
    )
    post_elapsed_s = time.perf_counter() - post_start_s

    runtime_seconds = {
        "orekit_init_and_data_setup": init_elapsed_s,
        "orbit_propagation": propagate_elapsed_s,
        "python_post_processing_and_outputs": post_elapsed_s,
        "total_pipeline": time.perf_counter() - total_start_s,
    }

    save_summary(
        timeline,
        windows,
        revisit_rows,
        plot_path,
        elevation_plot_path,
        ground_track_map_path,
        footprint_plot_path,
        {
            "coverage_heatmap": coverage_heatmap_path,
            "coverage_statistics_csv": coverage_statistics_path,
            "coverage_percentage_json": coverage_percentage_path,
            "coverage_summary": coverage_summary,
            "revisit_csv": revisit_csv_path,
        },
        runtime_seconds,
    )

    best = min(timeline, key=lambda row: row["boresight_angle_deg"])
    print(f"Minimum off-nadir angle: {best['boresight_angle_deg']:.3f} deg")
    print(f"Imaging opportunities: {len(windows)}")
    print(f"Results saved to: {OUTPUT_DIR}")
    print(f"Total Orekit A3 runtime: {runtime_seconds['total_pipeline']:.3f} s")


if __name__ == "__main__":
    main()
