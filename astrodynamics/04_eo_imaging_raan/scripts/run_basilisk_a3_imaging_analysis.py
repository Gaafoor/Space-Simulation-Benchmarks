#!/usr/bin/env python3

import argparse
from dataclasses import dataclass
from pathlib import Path
import csv
import json
import math
from datetime import timedelta
import os
import subprocess
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from Basilisk.simulation import spacecraft
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    vizSupport,
)
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

from a3_imaging_geometry import (
    A3ImagingConfig,
    MISSION_EPOCH_UTC,
    compute_imaging_timeline,
    extract_windows,
    gmst_rad,
)

# Retain vizInterface objects for the full simulation lifetime.
_VIZARD_RETAIN: list[object] = []

OUTPUT_DIR = Path("results/basilisk_a3")
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent
BACKGROUND_MAP_GLOB = "ModifiedBlueMarble*.png"
BACKGROUND_MAP_ENV_VAR = "A3_BACKGROUND_IMAGE"
BACKGROUND_MAP_DEFAULT_PATH = WORKSPACE_ROOT / "data/benchmark_a/ModifiedBlueMarble.jpg"

RAAN_DEG = 61.0
ALTITUDE_M = 500000.0
SIM_DURATION_S = 24.0 * 3600.0
STEP_S = 10.0
SPICE_EPOCH_UTC = "2026 JUN 02 00:00:00.000 (UTC)"
EARTH_RADIUS_KM = 6378.137
MADRID_LAT_MIN_DEG = 40.2
MADRID_LAT_MAX_DEG = 40.7
MADRID_LON_MIN_DEG = -4.2
MADRID_LON_MAX_DEG = -3.2
MADRID_GRID_LAT_DIVISIONS = 100
MADRID_GRID_LON_DIVISIONS = 100


@dataclass(frozen=True)
class VizardOptions:
    enabled: bool = False
    save_file: Path | None = None
    launch_after_run: bool = True
    live_stream: bool = False
    sprite: str = "bskSat"


def load_world_background_image():
    candidates = []

    candidates.append(BACKGROUND_MAP_DEFAULT_PATH)

    env_value = __import__("os").environ.get(BACKGROUND_MAP_ENV_VAR)
    if env_value:
        candidates.append(Path(env_value))

    candidates.extend(
        sorted(
            (WORKSPACE_ROOT / "assets").glob(BACKGROUND_MAP_GLOB)
        )
    )

    # Fallback for Cursor-provided assets not copied into workspace.
    candidates.extend(
        sorted(
            Path.home().glob(
                ".cursor/projects/*/assets/ModifiedBlueMarble*.png"
            )
        )
    )

    seen = set()
    for path in candidates:
        if path in seen:
            continue
        seen.add(path)
        try:
            image = plt.imread(path)
            print(f"Using world background image: {path}")
            return image
        except Exception:
            continue

    print("World background image not found; using plain background.")
    return None


def load_spice_kernels() -> None:
    """Load SPICE kernels so Vizard can rotate Earth with simulation time."""
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de430)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.naif0012)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.de_403_masses)))
    pyswice.furnsh_c(str(get_path(DataFile.EphemerisData.pck00010)))


def create_orbit():
    oe = orbitalMotion.ClassicElements()

    oe.a = 6378137.0 + ALTITUDE_M
    oe.e = 0.0
    oe.i = np.deg2rad(97.4)
    oe.Omega = np.deg2rad(RAAN_DEG)
    oe.omega = 0.0
    oe.f = 0.0

    return orbitalMotion.elem2rv(
        orbitalMotion.MU_EARTH * 1e9,
        oe
    )


def find_vizard_binary() -> Path | None:
    env_path = os.environ.get("VIZARD_PATH")
    if env_path:
        candidate = Path(env_path).expanduser()
        if candidate.is_file():
            return candidate

    search_roots = [
        WORKSPACE_ROOT / "tools/Vizard",
        Path.home() / "Vizard",
        Path.home() / ".local/share/Vizard",
    ]
    names = ("Vizard.x86_64", "Vizard.x64", "Vizard")
    for root in search_roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.name in names:
                return path
    return None


def launch_vizard(bin_path: Path | None = None, live_stream: bool = False) -> None:
    vizard_bin = find_vizard_binary()
    if vizard_bin is None:
        print("Vizard binary not found. Set VIZARD_PATH or install Vizard under tools/Vizard.")
        return
    command = [str(vizard_bin)]
    if live_stream:
        command.append("-liveStream")
    elif bin_path is not None and bin_path.exists():
        command.extend(["-loadFile", str(bin_path.resolve())])
    env = os.environ.copy()
    env.setdefault("GDK_BACKEND", "x11")
    subprocess.Popen(command, env=env)


def _configure_a3_vizard_overlays(
    viz,
    sc: spacecraft.Spacecraft,
    earth,
    config: A3ImagingConfig,
) -> None:
    """Earth-targeted nadir camera cone, HUD overlays, and Madrid locator."""
    viz.settings.viewCameraFrustumHUD = 1
    viz.settings.viewCameraBoresightHUD = 1
    viz.settings.showSpacecraftGroundTrack = 1

    vizSupport.createStandardCamera(
        viz,
        spacecraftName=sc.ModelTag,
        setMode=0,
        setView=0,
        bodyTarget=earth.displayName,
        fieldOfView=config.camera_fov_deg * macros.D2R,
        displayName="A3 Nadir Camera (10 deg)",
    )
    vizSupport.addLocation(
        viz,
        stationName="Madrid",
        parentBodyName=earth.displayName,
        lla_GP=[
            math.radians(config.madrid_lat_deg),
            math.radians(config.madrid_lon_deg),
            config.madrid_alt_m,
        ],
        color="red",
        label="Madrid",
        markerScale=1.5,
        fieldOfView=np.radians(1.0),
    )
    vizSupport.createTargetLine(
        viz,
        toBodyName="Madrid",
        lineColor="yellow",
    )


def attach_vizard_interface(
    sim: SimulationBaseClass.SimBaseClass,
    task_name: str,
    sc: spacecraft.Spacecraft,
    earth,
    vizard: VizardOptions,
) -> str | None:
    if not vizard.enabled:
        return None
    if not vizSupport.vizFound:
        raise RuntimeError("Basilisk was built without vizInterface support; cannot enable Vizard.")

    config = A3ImagingConfig()
    save_file = vizard.save_file or (OUTPUT_DIR / "basilisk_a3_UnityViz.bin")
    viz = vizSupport.enableUnityVisualization(
        sim,
        task_name,
        sc,
        saveFile=str(save_file) if not vizard.live_stream else None,
        liveStream=vizard.live_stream,
        spriteList=[vizard.sprite],
        groundTrackColorList=[vizSupport.toRGBA255("cyan")],
    )
    _VIZARD_RETAIN.append(viz)
    _configure_a3_vizard_overlays(viz, sc, earth, config)

    if vizard.live_stream:
        return None
    return str(save_file) if save_file.suffix.lower() == ".bin" else None


def run_basilisk(sim_duration_s: float, step_s: float, vizard: VizardOptions | None = None):
    sim = SimulationBaseClass.SimBaseClass()

    process = sim.CreateNewProcess("process")
    task = sim.CreateNewTask(
        "task",
        macros.sec2nano(step_s)
    )

    process.addTask(task)

    sc = spacecraft.Spacecraft()
    sc.ModelTag = "CubeSat_A3"

    rN, vN = create_orbit()

    sc.hub.r_CN_NInit = rN
    sc.hub.v_CN_NInit = vN

    grav_factory = simIncludeGravBody.gravBodyFactory()
    use_spice = vizard is not None and vizard.enabled
    body_names = ["sun", "earth"] if use_spice else ["earth"]
    grav_bodies = grav_factory.createBodies(body_names)
    earth = grav_bodies["earth"]
    earth.isCentralBody = True
    j2_path = get_path(getattr(DataFile.LocalGravData, "GGM03S_J2_only"))
    earth.useSphericalHarmonicsGravityModel(str(j2_path), 2)
    grav_factory.addBodiesTo(sc)

    if use_spice:
        load_spice_kernels()
        grav_factory.createSpiceInterface(time=SPICE_EPOCH_UTC, epochInMsg=True)
        grav_factory.spiceObject.zeroBase = "earth"
        grav_factory.spiceObject.referenceBase = "J2000"

    task_name = "task"
    if use_spice:
        sim.AddModelToTask(task_name, grav_factory.spiceObject)
    sim.AddModelToTask(task_name, sc)

    recorder = sc.scStateOutMsg.recorder()

    sim.AddModelToTask(task_name, recorder)

    vizard_bin_path = None
    if vizard is not None and vizard.enabled:
        vizard_bin_path = attach_vizard_interface(sim, task_name, sc, earth, vizard)

    sim.InitializeSimulation()

    sim.ConfigureStopTime(
        macros.sec2nano(sim_duration_s)
    )

    sim.ExecuteSimulation()

    pos = np.array(recorder.r_BN_N)
    times = np.array(recorder.times()) * 1e-9

    return times, pos, vizard_bin_path


def eci_to_ecef(position_eci_m, time_s):
    theta = gmst_rad(
        MISSION_EPOCH_UTC
        + timedelta(seconds=float(time_s))
    )
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
    position_ecef_m = eci_to_ecef(
        position_eci_m,
        time_s
    )
    radius = np.linalg.norm(
        position_ecef_m
    )
    lat = math.degrees(
        math.asin(position_ecef_m[2] / radius)
    )
    lon = math.degrees(
        math.atan2(
            position_ecef_m[1],
            position_ecef_m[0]
        )
    )
    return lat, lon


def compute_ground_track(times, positions):
    rows = []

    for time_s, position in zip(
        times,
        positions
    ):
        lat, lon = eci_to_subpoint_lat_lon(
            position,
            time_s
        )
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
        if (
            previous_lon is not None
            and abs(lon - previous_lon) > 180.0
        ):
            if current:
                segments.append(current)
            current = []

        current.append(row)
        previous_lon = lon

    if current:
        segments.append(current)

    return segments


def save_timeline_csv(timeline):
    path = OUTPUT_DIR / "timeline.csv"

    with path.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=list(timeline[0].keys())
        )

        writer.writeheader()
        writer.writerows(timeline)

    return path


def save_windows_csv(windows):
    path = OUTPUT_DIR / "imaging_windows.csv"

    with path.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=[
                "start_s",
                "stop_s",
                "duration_s"
            ]
        )

        writer.writeheader()
        writer.writerows(windows)

    return path


def save_plot(timeline):
    path = OUTPUT_DIR / "off_nadir_angle_vs_time.png"

    times_h = [
        r["time_s"] / 3600.0
        for r in timeline
    ]

    angles = [
        r["boresight_angle_deg"]
        for r in timeline
    ]

    fig, ax = plt.subplots(
        figsize=(12, 5)
    )

    ax.plot(
        times_h,
        angles,
        linewidth=0.8
    )

    ax.axhline(
        5.0,
        linestyle="--",
        linewidth=1.0,
        label="5 deg threshold"
    )

    ax.set_title(
        f"Basilisk A3 Off-Nadir Angle vs Time (RAAN={RAAN_DEG:g} deg)"
    )

    ax.set_xlabel(
        "Elapsed Time [hours]"
    )

    ax.set_ylabel(
        "Off-Nadir Angle [deg]"
    )

    ax.grid(True)

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        path,
        dpi=150
    )

    plt.close(fig)

    return path
def save_elevation_plot(timeline):
    path = OUTPUT_DIR / "elevation_vs_time.png"

    times_h = [
        r["time_s"] / 3600.0
        for r in timeline
    ]

    elevations = [
        r["target_elevation_deg"]
        for r in timeline
    ]

    fig, ax = plt.subplots(
        figsize=(12, 5)
    )

    ax.plot(
        times_h,
        elevations,
        linewidth=0.8
    )

    ax.axhline(
        0.0,
        linestyle="--",
        linewidth=1.0,
        label="Horizon"
    )

    ax.set_title(
        f"Basilisk A3 Target Elevation vs Time (RAAN={RAAN_DEG:g} deg)"
    )

    ax.set_xlabel(
        "Elapsed Time [hours]"
    )

    ax.set_ylabel(
        "Madrid Elevation [deg]"
    )

    ax.grid(True)

    ax.legend()

    fig.tight_layout()

    fig.savefig(
        path,
        dpi=150
    )

    plt.close(fig)

    return path


def save_ground_track_map(times, positions, timeline):
    path = OUTPUT_DIR / "ground_track_madrid_imaging_map.png"
    config = A3ImagingConfig()
    ground_track = compute_ground_track(
        times,
        positions
    )
    imaging_track = [
        ground_track[index]
        for index, row in enumerate(timeline)
        if row["target_in_camera_fov"]
    ]

    fig, ax = plt.subplots(
        figsize=(13, 6)
    )

    segments = split_ground_track_for_dateline(
        ground_track
    )

    for index, segment in enumerate(segments):
        ax.plot(
            [row["lon_deg"] for row in segment],
            [row["lat_deg"] for row in segment],
            color="tab:blue",
            linewidth=0.8,
            alpha=0.8,
            label="Satellite ground track"
            if index == 0
            else None
        )

    if imaging_track:
        ax.scatter(
            [row["lon_deg"] for row in imaging_track],
            [row["lat_deg"] for row in imaging_track],
            color="tab:orange",
            s=20,
            label="Imaging samples",
            zorder=4
        )

    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=90,
        linewidths=2,
        label="Madrid",
        zorder=5
    )

    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 30))
    ax.set_yticks(np.arange(-90, 91, 15))
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        f"Basilisk A3 Ground Track and Madrid Imaging Opportunity (RAAN={RAAN_DEG:g} deg)"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(
        path,
        dpi=150
    )
    plt.close(fig)

    return path


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
        math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2)
    )

    lon2_deg = (
        (math.degrees(lon2) + 180.0)
        % 360.0
        - 180.0
    )

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
    return 2.0 * EARTH_RADIUS_KM * math.asin(
        min(1.0, math.sqrt(hav))
    )


def save_camera_footprint_plot(times, positions, timeline):
    path = OUTPUT_DIR / "camera_footprint_best_instant.png"
    config = A3ImagingConfig()
    best_index, best = min(
        enumerate(timeline),
        key=lambda item: item[1]["boresight_angle_deg"]
    )
    subpoint_lat, subpoint_lon = eci_to_subpoint_lat_lon(
        positions[best_index],
        times[best_index]
    )
    footprint_radius_km = (
        config.footprint_diameter_km / 2.0
    )
    madrid_distance_km = ground_distance_km(
        subpoint_lat,
        subpoint_lon,
        config.madrid_lat_deg,
        config.madrid_lon_deg
    )
    madrid_inside = (
        madrid_distance_km <= footprint_radius_km
    )

    bearings = np.linspace(
        0.0,
        2.0 * math.pi,
        181
    )
    circle = [
        destination_point(
            subpoint_lat,
            subpoint_lon,
            bearing,
            footprint_radius_km
        )
        for bearing in bearings
    ]

    fig, ax = plt.subplots(
        figsize=(8, 8)
    )

    ax.plot(
        [point[1] for point in circle],
        [point[0] for point in circle],
        color="tab:orange",
        linewidth=2,
        label=f"Camera footprint radius ({footprint_radius_km:.1f} km)"
    )
    ax.scatter(
        [subpoint_lon],
        [subpoint_lat],
        color="tab:blue",
        s=70,
        label="Satellite subpoint",
        zorder=5
    )
    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=100,
        linewidths=2,
        label="Madrid",
        zorder=6
    )

    margin_deg = max(
        0.5,
        footprint_radius_km / 111.0 * 2.5
    )
    center_lat = (
        subpoint_lat + config.madrid_lat_deg
    ) / 2.0
    center_lon = (
        subpoint_lon + config.madrid_lon_deg
    ) / 2.0
    ax.set_xlim(
        center_lon - margin_deg,
        center_lon + margin_deg
    )
    ax.set_ylim(
        center_lat - margin_deg,
        center_lat + margin_deg
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "Basilisk A3 Camera Footprint at Best Imaging Instant\n"
        f"t={best['time_s']:.1f} s, off-nadir={best['boresight_angle_deg']:.3f} deg, "
        f"Madrid inside footprint={madrid_inside}"
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    annotation = (
        f"Madrid distance from subpoint: {madrid_distance_km:.1f} km\n"
        f"Footprint radius: {footprint_radius_km:.1f} km"
    )
    ax.text(
        0.02,
        0.02,
        annotation,
        transform=ax.transAxes,
        va="bottom",
        ha="left",
        bbox={
            "facecolor": "white",
            "alpha": 0.8,
            "edgecolor": "0.7",
        }
    )

    fig.tight_layout()
    fig.savefig(
        path,
        dpi=150
    )
    plt.close(fig)

    return path


def save_global_footprint_coverage_map(times, positions):
    path = OUTPUT_DIR / "global_footprint_coverage_map.png"
    config = A3ImagingConfig()
    footprint_radius_km = config.footprint_diameter_km / 2.0

    ground_track = compute_ground_track(times, positions)
    segments = split_ground_track_for_dateline(ground_track)

    # Limit plotted footprints for readability and runtime while keeping
    # full-orbit spatial coverage visible.
    max_footprints_to_plot = 1500
    stride = max(1, len(ground_track) // max_footprints_to_plot)

    fig, ax = plt.subplots(figsize=(14, 7))

    background = load_world_background_image()
    if background is not None:
        ax.imshow(
            background,
            extent=[-180, 180, -90, 90],
            origin="upper",
            aspect="auto",
            zorder=0,
        )

    for index, segment in enumerate(segments):
        ax.plot(
            [row["lon_deg"] for row in segment],
            [row["lat_deg"] for row in segment],
            color="tab:blue",
            linewidth=1.0,
            alpha=0.9,
            label="Satellite ground track"
            if index == 0
            else None
        )

    bearings = np.linspace(
        0.0,
        2.0 * math.pi,
        73
    )
    for row in ground_track[::stride]:
        center_lat = row["lat_deg"]
        center_lon = row["lon_deg"]
        circle = [
            destination_point(
                center_lat,
                center_lon,
                bearing,
                footprint_radius_km
            )
            for bearing in bearings
        ]
        ax.fill(
            [point[1] for point in circle],
            [point[0] for point in circle],
            color="tab:orange",
            alpha=0.02,
            linewidth=0.0,
            zorder=2,
        )

    ax.scatter(
        [config.madrid_lon_deg],
        [config.madrid_lat_deg],
        color="tab:red",
        marker="x",
        s=100,
        linewidths=2,
        label="Madrid",
        zorder=5
    )

    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(np.arange(-180, 181, 30))
    ax.set_yticks(np.arange(-90, 91, 15))
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "Basilisk A3 Global Ground Track with Camera Footprint Transparency\n"
        f"Nadir camera, footprint radius {footprint_radius_km:.1f} km, "
        f"plotted every {stride * STEP_S:.0f} s"
    )
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(
        path,
        dpi=150
    )
    plt.close(fig)

    return path


def create_madrid_coverage_grid():
    lat_edges = np.linspace(
        MADRID_LAT_MIN_DEG,
        MADRID_LAT_MAX_DEG,
        MADRID_GRID_LAT_DIVISIONS + 1
    )
    lon_edges = np.linspace(
        MADRID_LON_MIN_DEG,
        MADRID_LON_MAX_DEG,
        MADRID_GRID_LON_DIVISIONS + 1
    )
    lat_centers = (
        lat_edges[:-1]
        + lat_edges[1:]
    ) / 2.0
    lon_centers = (
        lon_edges[:-1]
        + lon_edges[1:]
    ) / 2.0

    return lat_edges, lon_edges, lat_centers, lon_centers


def accumulate_madrid_coverage(times, positions):
    config = A3ImagingConfig()
    (
        lat_edges,
        lon_edges,
        lat_centers,
        lon_centers
    ) = create_madrid_coverage_grid()
    coverage_count = np.zeros(
        (
            MADRID_GRID_LAT_DIVISIONS,
            MADRID_GRID_LON_DIVISIONS
        ),
        dtype=int
    )
    footprint_radius_km = (
        config.footprint_diameter_km / 2.0
    )
    lat_grid_deg, lon_grid_deg = np.meshgrid(
        lat_centers,
        lon_centers,
        indexing="ij"
    )
    lat_grid_rad = np.deg2rad(
        lat_grid_deg
    )
    lon_grid_rad = np.deg2rad(
        lon_grid_deg
    )

    subpoints = [
        eci_to_subpoint_lat_lon(
            position,
            time_s
        )
        for time_s, position in zip(
            times,
            positions
        )
    ]

    for subpoint_lat, subpoint_lon in subpoints:
        subpoint_lat_rad = math.radians(
            subpoint_lat
        )
        subpoint_lon_rad = math.radians(
            subpoint_lon
        )
        dlat = (
            lat_grid_rad
            - subpoint_lat_rad
        )
        dlon = (
            lon_grid_rad
            - subpoint_lon_rad
        )
        hav = (
            np.sin(dlat / 2.0) ** 2
            + math.cos(subpoint_lat_rad)
            * np.cos(lat_grid_rad)
            * np.sin(dlon / 2.0) ** 2
        )
        distances_km = (
            2.0
            * EARTH_RADIUS_KM
            * np.arcsin(
                np.minimum(
                    1.0,
                    np.sqrt(hav)
                )
            )
        )
        coverage_count += (
            distances_km <= footprint_radius_km
        ).astype(int)

    return {
        "lat_edges": lat_edges,
        "lon_edges": lon_edges,
        "lat_centers": lat_centers,
        "lon_centers": lon_centers,
        "coverage_count": coverage_count,
        "footprint_radius_km": footprint_radius_km,
    }


def save_coverage_heatmap(coverage):
    path = OUTPUT_DIR / "coverage_heatmap.png"
    config = A3ImagingConfig()
    coverage_count = coverage["coverage_count"]

    fig, ax = plt.subplots(
        figsize=(10, 7)
    )

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
        zorder=5
    )

    colorbar = fig.colorbar(
        image,
        ax=ax
    )
    colorbar.set_label(
        "Coverage count [samples]"
    )
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    ax.set_title(
        "Basilisk A3 Madrid Regional Coverage Heatmap\n"
        "100 x 100 grid, coverage accumulated from nadir camera footprint"
    )
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(
        path,
        dpi=150
    )
    plt.close(fig)

    return path


def save_coverage_statistics_csv(coverage):
    path = OUTPUT_DIR / "madrid_coverage_statistics.csv"
    coverage_count = coverage["coverage_count"]
    lat_centers = coverage["lat_centers"]
    lon_centers = coverage["lon_centers"]

    with path.open(
        "w",
        newline="",
        encoding="utf-8"
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "lat_index",
                "lon_index",
                "lat_center_deg",
                "lon_center_deg",
                "coverage_count",
                "covered",
            ]
        )
        writer.writeheader()
        for lat_index, lat in enumerate(lat_centers):
            for lon_index, lon in enumerate(lon_centers):
                count = int(
                    coverage_count[
                        lat_index,
                        lon_index
                    ]
                )
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
    total_cells = int(
        coverage_count.size
    )
    covered_cells = int(
        np.count_nonzero(
            coverage_count > 0
        )
    )
    max_count = int(
        coverage_count.max()
    )
    mean_count = float(
        coverage_count.mean()
    )
    coverage_percentage = (
        100.0 * covered_cells / total_cells
        if total_cells
        else 0.0
    )

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

    with path.open(
        "w",
        encoding="utf-8"
    ) as f:
        json.dump(
            summary,
            f,
            indent=2
        )

    return path, summary


def save_madrid_coverage_products(times, positions):
    coverage = accumulate_madrid_coverage(
        times,
        positions
    )
    heatmap_path = save_coverage_heatmap(
        coverage
    )
    statistics_path = save_coverage_statistics_csv(
        coverage
    )
    percentage_path, coverage_summary = save_coverage_percentage_json(
        coverage,
        heatmap_path,
        statistics_path
    )

    return {
        "coverage_heatmap": heatmap_path,
        "coverage_statistics_csv": statistics_path,
        "coverage_percentage_json": percentage_path,
        "coverage_summary": coverage_summary,
    }


def save_summary(
    timeline,
    windows,
    plot_path,
    elevation_plot_path,
    ground_track_map_path,
    footprint_plot_path,
    global_footprint_map_path,
    coverage_products,
    vizard_bin_path=None
):
    best = min(
        timeline,
        key=lambda r:
        r["boresight_angle_deg"]
    )

    summary = {
        "RAAN_deg": RAAN_DEG,
        "minimum_off_nadir_angle_deg":
            best["boresight_angle_deg"],
        "minimum_off_nadir_time_s":
            best["time_s"],
        "imaging_opportunity_count":
            len(windows),
        "imaging_sample_count":
            sum(
                1
                for r in timeline
                if r["target_in_camera_fov"]
            ),
        "off_nadir_plot":
            str(plot_path),

        "elevation_plot":
            str(elevation_plot_path),

        "ground_track_map":
            str(ground_track_map_path),

        "camera_footprint_plot":
            str(footprint_plot_path),

        "global_footprint_coverage_map":
            str(global_footprint_map_path),

        "coverage_heatmap":
            str(coverage_products["coverage_heatmap"]),

        "coverage_statistics_csv":
            str(coverage_products["coverage_statistics_csv"]),

        "coverage_percentage_json":
            str(coverage_products["coverage_percentage_json"]),

        "madrid_regional_coverage":
            coverage_products["coverage_summary"],
    }
    if vizard_bin_path:
        summary["vizard_bin"] = str(vizard_bin_path)

    path = OUTPUT_DIR / "summary.json"

    with path.open(
        "w",
        encoding="utf-8"
    ) as f:
        json.dump(
            summary,
            f,
            indent=2
        )

    return path


def parse_args():
    parser = argparse.ArgumentParser(description="Run Basilisk A3 imaging analysis")
    parser.add_argument(
        "--duration-s",
        type=float,
        default=SIM_DURATION_S,
        help="Simulation duration in seconds (default: 24 h)",
    )
    parser.add_argument(
        "--step-s",
        type=float,
        default=STEP_S,
        help="Simulation integration step in seconds (default: 10 s)",
    )
    parser.add_argument(
        "--vizard",
        action="store_true",
        help="Record Vizard .bin and launch Vizard after run",
    )
    parser.add_argument(
        "--vizard-live",
        action="store_true",
        help="Live-stream to a running Vizard instance",
    )
    parser.add_argument(
        "--vizard-file",
        type=Path,
        default=None,
        help="Output path for UnityViz .bin recording",
    )
    parser.add_argument(
        "--no-vizard-launch",
        action="store_true",
        help="Do not auto-launch Vizard after run",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print(
        "Running Basilisk A3..."
    )

    vizard = VizardOptions(
        enabled=args.vizard or args.vizard_live,
        save_file=args.vizard_file,
        launch_after_run=not args.no_vizard_launch,
        live_stream=args.vizard_live,
    )

    times, positions, vizard_bin_path = run_basilisk(
        sim_duration_s=args.duration_s,
        step_s=args.step_s,
        vizard=vizard if vizard.enabled else None,
    )
    
    timeline = compute_imaging_timeline(
        times,
        positions,
        A3ImagingConfig()
    )

    windows = extract_windows(
        timeline
    )

    save_timeline_csv(
        timeline
    )

    save_windows_csv(
        windows
    )

    plot_path = save_plot(
        timeline
    )
    
    elevation_plot_path = save_elevation_plot(
        timeline
    )

    ground_track_map_path = save_ground_track_map(
        times,
        positions,
        timeline
    )

    footprint_plot_path = save_camera_footprint_plot(
        times,
        positions,
        timeline
    )

    global_footprint_map_path = save_global_footprint_coverage_map(
        times,
        positions
    )

    coverage_products = save_madrid_coverage_products(
        times,
        positions
    )

    save_summary(
        timeline,
        windows,
        plot_path,
        elevation_plot_path,
        ground_track_map_path,
        footprint_plot_path,
        global_footprint_map_path,
        coverage_products,
        vizard_bin_path=vizard_bin_path,
    )

    best = min(
        timeline,
        key=lambda r:
        r["boresight_angle_deg"]
    )

    print()
    print(
        f"Minimum off-nadir angle: "
        f"{best['boresight_angle_deg']:.3f} deg"
    )

    print(
        f"Imaging opportunities: "
        f"{len(windows)}"
    )

    print(
        f"Results saved to: "
        f"{OUTPUT_DIR}"
    )
    if vizard.enabled and vizard_bin_path:
        print(f"Vizard file: {vizard_bin_path}")
        if vizard.launch_after_run and not vizard.live_stream:
            launch_vizard(Path(vizard_bin_path))



if __name__ == "__main__":
    main()