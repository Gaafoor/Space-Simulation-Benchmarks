#!/usr/bin/env python3

from datetime import datetime, timedelta
from pathlib import Path
import csv
import json
import math
import re
import time

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from a3_imaging_geometry import (
    A3ImagingConfig,
    MISSION_EPOCH_UTC,
    compute_imaging_timeline,
    compute_metrics,
    extract_windows,
    gmst_rad,
)


STATE_FILE = Path("gmat/gmat_a3_state.csv")
CONTACT_FILE = Path("gmat/gmat_a3_madrid_contacts.txt")
SCRIPT_FILE = Path("gmat/benchmark_a3_gmat.script")
RESULTS_DIR = Path("results")
MASTER_CSV = RESULTS_DIR / "raan_trade_study.csv"

EPOCH = datetime(2026, 6, 2, 0, 0, 0)
FMT = "%d %b %Y %H:%M:%S.%f"
EARTH_RADIUS_KM = 6378.137
MADRID_LAT_MIN_DEG = 40.2
MADRID_LAT_MAX_DEG = 40.7
MADRID_LON_MIN_DEG = -4.2
MADRID_LON_MAX_DEG = -3.2
MADRID_GRID_LAT_DIVISIONS = 100
MADRID_GRID_LON_DIVISIONS = 100


def load_state_history():
    times = []
    positions = []

    with STATE_FILE.open(encoding="utf-8") as f:
        for line in f:
            s = line.strip()

            if not s:
                continue

            if s.startswith("CubeSat_A3"):
                continue

            parts = s.split()

            times.append(float(parts[0]))

            positions.append([
                float(parts[1]) * 1000.0,
                float(parts[2]) * 1000.0,
                float(parts[3]) * 1000.0
            ])

    times = np.array(times)
    positions = np.array(positions)

    order = np.argsort(times)

    return times[order], positions[order]


def load_contact_windows():
    windows = []

    pattern = re.compile(r"^\d{2} \w{3} \d{4} ")

    with CONTACT_FILE.open(encoding="utf-8") as f:

        for line in f:

            s = line.strip()

            if not pattern.match(s):
                continue

            parts = re.split(r"\s{2,}", s)

            start = (
                datetime.strptime(parts[0], FMT) - EPOCH
            ).total_seconds()

            stop = (
                datetime.strptime(parts[1], FMT) - EPOCH
            ).total_seconds()

            duration = float(parts[2])

            windows.append(
                (
                    start,
                    stop,
                    duration,
                    parts[0],
                    parts[1]
                )
            )

    return windows


def load_raan_deg():
    pattern = re.compile(r"CubeSat_A3\.RAAN\s*=\s*([-+0-9.eE]+)")

    with SCRIPT_FILE.open(encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                return float(match.group(1))

    raise ValueError(f"Could not find CubeSat_A3.RAAN in {SCRIPT_FILE}")


def format_raan_dir_name(raan_deg):
    rounded = int(round(raan_deg))
    return f"RAAN_{rounded:03d}"


def summarize_passes(windows, timeline):
    pass_metrics = []

    for i, (
        start,
        stop,
        duration,
        utc_start,
        utc_stop
    ) in enumerate(windows, start=1):

        subset = [
            r for r in timeline
            if start <= r["time_s"] <= stop
        ]

        if not subset:
            continue

        min_angle = min(
            r["boresight_angle_deg"]
            for r in subset
        )

        max_elevation = max(
            r["target_elevation_deg"]
            for r in subset
        )

        pass_metrics.append(
            {
                "pass_index": i,
                "start_s": start,
                "stop_s": stop,
                "duration_s": duration,
                "duration_min": duration / 60.0,
                "utc_start": utc_start,
                "utc_stop": utc_stop,
                "min_off_nadir_deg": min_angle,
                "max_elevation_deg": max_elevation,
            }
        )

    return pass_metrics


def write_master_csv_row(row):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    file_exists = MASTER_CSV.is_file()

    fieldnames = [
        "RAAN_deg",
        "pass_count",
        "min_off_nadir_deg",
        "imaging_opportunities",
        "best_pass_off_nadir_deg",
        "max_elevation_deg",
    ]

    with MASTER_CSV.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


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


def save_off_nadir_plot(results_dir, timeline, raan_deg):
    times_h = [
        r["time_s"] / 3600.0
        for r in timeline
    ]
    off_nadir = [
        r["boresight_angle_deg"]
        for r in timeline
    ]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times_h, off_nadir, linewidth=0.8)
    ax.axhline(
        A3ImagingConfig().camera_half_angle_deg,
        linestyle="--",
        linewidth=1.0,
        label="5 deg half-angle threshold",
    )
    ax.set_title(f"GMAT A3 Off-Nadir Angle vs Time - RAAN {raan_deg:g} deg")
    ax.set_xlabel("Elapsed time [hours]")
    ax.set_ylabel("Off-nadir angle to Madrid [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()

    plot_path = results_dir / "off_nadir_angle_vs_time.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)

    return plot_path


def save_elevation_plot(results_dir, timeline, raan_deg):
    times_h = [r["time_s"] / 3600.0 for r in timeline]
    elevations = [r["target_elevation_deg"] for r in timeline]

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times_h, elevations, linewidth=0.8)
    ax.axhline(0.0, linestyle="--", linewidth=1.0, label="Horizon")
    ax.set_title(f"GMAT A3 Target Elevation vs Time - RAAN {raan_deg:g} deg")
    ax.set_xlabel("Elapsed time [hours]")
    ax.set_ylabel("Madrid elevation [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()

    plot_path = results_dir / "elevation_vs_time.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return plot_path


def save_ground_track_map(results_dir, times, positions, timeline, raan_deg):
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
    ax.set_title(f"GMAT A3 Ground Track and Madrid Imaging Opportunity (RAAN={raan_deg:g} deg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    fig.tight_layout()

    path = results_dir / "ground_track_madrid_imaging_map.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_camera_footprint_plot(results_dir, times, positions, timeline, raan_deg):
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
        "GMAT A3 Camera Footprint at Best Imaging Instant\n"
        f"RAAN={raan_deg:g} deg, t={best['time_s']:.1f} s, "
        f"off-nadir={best['boresight_angle_deg']:.3f} deg, "
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

    path = results_dir / "camera_footprint_best_instant.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_coverage_heatmap(results_dir, coverage):
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
    ax.set_title("GMAT A3 Madrid Regional Coverage Heatmap\n100 x 100 grid from post-processed camera footprint")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()

    path = results_dir / "coverage_heatmap.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def save_coverage_statistics_csv(results_dir, coverage):
    path = results_dir / "madrid_coverage_statistics.csv"
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


def save_coverage_percentage_json(results_dir, coverage, heatmap_path, statistics_path):
    path = results_dir / "coverage_percentage.json"
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


def save_results(
    raan_deg,
    times,
    positions,
    timeline,
    windows,
    pass_metrics,
    load_elapsed_s,
    compute_elapsed_s,
):
    save_start_s = time.perf_counter()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    results_dir = RESULTS_DIR / format_raan_dir_name(raan_deg)
    results_dir.mkdir(parents=True, exist_ok=True)

    best = min(
        timeline,
        key=lambda r: r["boresight_angle_deg"]
    )

    imaging_windows = extract_windows(timeline)

    best_pass_off_nadir = (
        min(p["min_off_nadir_deg"] for p in pass_metrics)
        if pass_metrics
        else None
    )
    max_elevation = (
        max(p["max_elevation_deg"] for p in pass_metrics)
        if pass_metrics
        else None
    )

    plot_path = save_off_nadir_plot(results_dir, timeline, raan_deg)
    elevation_plot_path = save_elevation_plot(results_dir, timeline, raan_deg)
    ground_track_map_path = save_ground_track_map(results_dir, times, positions, timeline, raan_deg)
    footprint_plot_path = save_camera_footprint_plot(results_dir, times, positions, timeline, raan_deg)
    coverage = accumulate_madrid_coverage(times, positions)
    coverage_heatmap_path = save_coverage_heatmap(results_dir, coverage)
    coverage_stats_path = save_coverage_statistics_csv(results_dir, coverage)
    coverage_percentage_path, coverage_summary = save_coverage_percentage_json(
        results_dir, coverage, coverage_heatmap_path, coverage_stats_path
    )

    timeline_path = results_dir / "imaging_timeline.csv"
    with timeline_path.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "time_s",
            "boresight_angle_deg",
            "target_elevation_deg",
            "slant_range_km",
            "target_in_camera_fov",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(timeline)

    windows_path = results_dir / "imaging_windows.csv"
    with windows_path.open("w", newline="", encoding="utf-8") as f:
        fieldnames = [
            "start_s",
            "stop_s",
            "duration_s",
            "duration_min",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for window in imaging_windows:
            writer.writerow(
                {
                    "start_s": window["start_s"],
                    "stop_s": window["stop_s"],
                    "duration_s": window["duration_s"],
                    "duration_min": window["duration_s"] / 60.0,
                }
            )

    revisit_windows = []
    for idx in range(1, len(imaging_windows)):
        revisit_s = imaging_windows[idx]["start_s"] - imaging_windows[idx - 1]["start_s"]
        revisit_windows.append(
            {
                "window_index_prev": idx,
                "window_index_next": idx + 1,
                "revisit_s": revisit_s,
                "revisit_min": revisit_s / 60.0,
            }
        )

    revisit_path = results_dir / "revisit_windows.csv"
    with revisit_path.open("w", newline="", encoding="utf-8") as f:
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
        writer.writerows(revisit_windows)

    aggregate_metrics = compute_metrics(timeline, imaging_windows, A3ImagingConfig())

    save_elapsed_s = time.perf_counter() - save_start_s
    summary = {
        "RAAN": raan_deg,
        "epoch": EPOCH.isoformat(),
        "pass_count": len(windows),
        "minimum_off_nadir_angle_deg": best["boresight_angle_deg"],
        "minimum_off_nadir_time_s": best["time_s"],
        "imaging_opportunity_count": len(imaging_windows),
        "imaging_sample_count": len([
            r for r in timeline
            if r["boresight_angle_deg"] <= A3ImagingConfig().camera_half_angle_deg
        ]),
        "all_pass_metrics": pass_metrics,
        "aggregate_metrics": aggregate_metrics,
        "revisit_windows": revisit_windows,
        "imaging_windows": [
            {
                "start_s": window["start_s"],
                "stop_s": window["stop_s"],
                "duration_s": window["duration_s"],
                "duration_min": window["duration_s"] / 60.0,
            }
            for window in imaging_windows
        ],
        "artifacts": {
            "off_nadir_plot": str(plot_path),
            "elevation_plot": str(elevation_plot_path),
            "ground_track_map": str(ground_track_map_path),
            "camera_footprint_plot": str(footprint_plot_path),
            "coverage_heatmap": str(coverage_heatmap_path),
            "coverage_statistics_csv": str(coverage_stats_path),
            "coverage_percentage_json": str(coverage_percentage_path),
            "revisit_windows_csv": str(revisit_path),
            "imaging_timeline_csv": str(timeline_path),
            "imaging_windows_csv": str(windows_path),
            "source_state_file": str(STATE_FILE),
            "source_contact_file": str(CONTACT_FILE),
            "source_script_file": str(SCRIPT_FILE),
        },
        "madrid_regional_coverage": coverage_summary,
        "runtime_seconds": {
            "load_inputs": load_elapsed_s,
            "compute_geometry": compute_elapsed_s,
            "save_outputs": save_elapsed_s,
            "total_post_processing": load_elapsed_s + compute_elapsed_s + save_elapsed_s,
        },
        "implementation_notes": {
            "workflow_type": "GMAT propagation/export + Python geometry post-processing",
            "coverage_method": "Computed in Python from exported ECI state history and camera footprint model",
            "native_gmat_sensor_contact": "Not supported via tested ContactLocator syntax",
        },
    }

    summary_path = results_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    write_master_csv_row(
        {
            "RAAN_deg": raan_deg,
            "pass_count": len(windows),
            "min_off_nadir_deg": best["boresight_angle_deg"],
            "imaging_opportunities": len(imaging_windows),
            "best_pass_off_nadir_deg": best_pass_off_nadir,
            "max_elevation_deg": max_elevation,
        }
    )


def main():
    print("\n========== A3 IMAGING ANALYSIS ==========\n")
    total_start = time.perf_counter()
    load_start = time.perf_counter()
    times, positions = load_state_history()
    windows = load_contact_windows()
    raan_deg = load_raan_deg()
    load_elapsed = time.perf_counter() - load_start

    process_start = time.perf_counter()
    timeline = compute_imaging_timeline(
        times,
        positions,
        A3ImagingConfig()
    )

    best = min(
        timeline,
        key=lambda r: r["boresight_angle_deg"]
    )

    print(
        f"Minimum off-nadir angle: "
        f"{best['boresight_angle_deg']:.2f} deg"
    )

    print(
        f"Time: {best['time_s']:.1f} s"
    )

    close = [r for r in timeline if r["boresight_angle_deg"] <= A3ImagingConfig().camera_half_angle_deg]

    print(
        f"\nImaging opportunities "
        f"(<= 5 deg off-nadir): {len(close)}"
    )

    pass_metrics = summarize_passes(windows, timeline)
    process_elapsed = time.perf_counter() - process_start

    print(
        f"\nContact windows found: {len(windows)}\n"
    )

    for i, (
        start,
        stop,
        duration,
        utc_start,
        utc_stop
    ) in enumerate(windows, start=1):

        subset = [
            r for r in timeline
            if start <= r["time_s"] <= stop
        ]

        if not subset:
            continue

        min_angle = min(
            r["boresight_angle_deg"]
            for r in subset
        )

        max_elevation = max(
            r["target_elevation_deg"]
            for r in subset
        )

        print(f"Pass {i}")
        print(f"  Start: {utc_start}")
        print(f"  Stop : {utc_stop}")
        print(f"  Duration: {duration/60:.2f} min")
        print(
            f"  Min Off-Nadir: "
            f"{min_angle:.2f} deg"
        )
        print(
            f"  Max Elevation: "
            f"{max_elevation:.2f} deg"
        )
        print()

    save_results(
        raan_deg,
        times,
        positions,
        timeline,
        windows,
        pass_metrics,
        load_elapsed,
        process_elapsed,
    )

    print("Coverage/footprint/heatmap/revisit outputs generated.")
    print(f"Post-processing runtime: {time.perf_counter() - total_start:.3f} s")
    print("======================================")


if __name__ == "__main__":
    main()