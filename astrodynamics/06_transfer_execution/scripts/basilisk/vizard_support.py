"""Basilisk Unity/Vizard helpers for GMAT handoff runs."""

from __future__ import annotations

import os
import shutil
import subprocess
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros, unitTestSupport, vizSupport

from ephemeris_handoff import SpiceReferenceEphemeris, read_spice_state_at

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parents[1]
VIZARD_INSTALL_DIR = WORKSPACE / "tools/Vizard"
VIZARD_DOWNLOAD_URL = "https://hanspeterschaub.info/bskFiles/Vizard_Linux.zip"

_VIZARD_RETAIN: list[object] = []
_GMAT_BSP_PATH: Path | None = None


@dataclass(frozen=True)
class VizardOptions:
    enabled: bool = False
    live_stream: bool = False
    save_file: Path | None = None
    launch_after_run: bool = True
    sprite: str = "bskSat"
    show_gmat_reference: bool = True
    reference_sprite: str = "COMM_Sat"


def default_vizard_bin_path(out_dir: Path) -> Path:
    return out_dir / "gmat_handoff_UnityViz.bin"


def find_vizard_binary() -> Path | None:
    env_path = os.environ.get("VIZARD_PATH")
    if env_path:
        candidate = Path(env_path).expanduser()
        if candidate.is_file():
            return candidate

    search_roots = [
        VIZARD_INSTALL_DIR,
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


def install_vizard() -> Path:
    VIZARD_INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    archive_path = VIZARD_INSTALL_DIR / "Vizard_Linux.zip"
    if not archive_path.exists():
        print(f"Downloading Vizard from {VIZARD_DOWNLOAD_URL} ...")
        request = urllib.request.Request(
            VIZARD_DOWNLOAD_URL,
            headers={"User-Agent": "Mozilla/5.0 (Basilisk GMAT handoff setup)"},
        )
        with urllib.request.urlopen(request) as response, archive_path.open("wb") as handle:
            shutil.copyfileobj(response, handle)

    print(f"Extracting {archive_path} ...")
    with zipfile.ZipFile(archive_path, "r") as archive:
        archive.extractall(VIZARD_INSTALL_DIR)

    binary = find_vizard_binary()
    if binary is None:
        raise RuntimeError(f"Vizard binary not found under {VIZARD_INSTALL_DIR} after extraction")
    binary.chmod(binary.stat().st_mode | 0o111)
    print(f"Vizard installed: {binary}")
    return binary


def ensure_vizard_binary() -> Path:
    binary = find_vizard_binary()
    if binary is not None:
        return binary
    return install_vizard()


def launch_vizard(bin_path: Path | None = None, *, live_stream: bool = False) -> subprocess.Popen[bytes] | None:
    vizard_bin = ensure_vizard_binary()
    env = os.environ.copy()
    env.setdefault("GDK_BACKEND", "x11")
    command = [str(vizard_bin)]
    if live_stream:
        command.append("-liveStream")
    elif bin_path is not None and bin_path.exists():
        command.extend(["-loadFile", str(bin_path.resolve())])
    else:
        raise FileNotFoundError(f"Vizard recording not found: {bin_path}")
    print(f"Launching Vizard: {' '.join(command)}")
    return subprocess.Popen(command, env=env)


def load_gmat_reference_kernel(schedule: dict) -> Path | None:
    global _GMAT_BSP_PATH
    ref_meta = schedule.get("reference_ephemeris", {})
    bsp_path = Path(ref_meta.get("path", ""))
    if not bsp_path.is_file():
        return None
    if _GMAT_BSP_PATH == bsp_path:
        return bsp_path
    from Basilisk.topLevelModules import pyswice
    from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path

    for kernel in (
        DataFile.EphemerisData.naif0012,
        DataFile.EphemerisData.de430,
    ):
        pyswice.furnsh_c(str(get_path(kernel)))
    pyswice.furnsh_c(str(bsp_path.resolve()))
    _GMAT_BSP_PATH = bsp_path
    return bsp_path


def create_gmat_reference_spacecraft(schedule: dict) -> spacecraft.Spacecraft | None:
    ref_meta = schedule.get("reference_ephemeris", {})
    if not ref_meta.get("path"):
        return None
    init = schedule["initial_state"]
    ref_sc = spacecraft.Spacecraft()
    ref_sc.ModelTag = "GMAT_Reference"
    ref_sc.hub.mHub = 1.0
    ref_sc.hub.r_CN_NInit = init["r_m"]
    ref_sc.hub.v_CN_NInit = init["v_m_s"]
    ref_sc.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    ref_sc.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    return ref_sc


def sync_gmat_reference_spacecraft(ref_sc: spacecraft.Spacecraft, schedule: dict, time_s: float) -> bool:
    ref_meta = schedule.get("reference_ephemeris", {})
    min_t = float(ref_meta.get("bsp_coverage_start_s", 0.0))
    if float(time_s) < min_t:
        return False
    if load_gmat_reference_kernel(schedule) is None:
        return False
    spice_ref = SpiceReferenceEphemeris.from_metadata(ref_meta)
    try:
        r_ref, v_ref = read_spice_state_at(spice_ref, float(time_s))
    except Exception:
        return False
    pos_ref = ref_sc.dynManager.getStateObject(ref_sc.hub.nameOfHubPosition)
    vel_ref = ref_sc.dynManager.getStateObject(ref_sc.hub.nameOfHubVelocity)
    pos_ref.setState(unitTestSupport.np2EigenVectorXd(np.asarray(r_ref, dtype=float)))
    vel_ref.setState(unitTestSupport.np2EigenVectorXd(np.asarray(v_ref, dtype=float)))
    return True


def attach_handoff_vizard(
    sc_sim,
    task_name: str,
    sc_object: spacecraft.Spacecraft,
    earth,
    *,
    vizard: VizardOptions,
    reference_sc: spacecraft.Spacecraft | None = None,
) -> str | None:
    if not vizard.enabled:
        return None
    if not vizSupport.vizFound:
        raise RuntimeError("Basilisk was built without vizInterface support; cannot enable Vizard.")

    sc_list = [sc_object]
    sprite_list = [vizard.sprite]
    ground_colors = [vizSupport.toRGBA255("cyan")]
    osc_colors = [vizSupport.toRGBA255("dodgerblue")]
    true_colors = [vizSupport.toRGBA255("deepskyblue")]
    if reference_sc is not None:
        sc_list.append(reference_sc)
        sprite_list.append(vizard.reference_sprite)
        ground_colors.append(vizSupport.toRGBA255("orange"))
        osc_colors.append(vizSupport.toRGBA255("darkorange"))
        true_colors.append(vizSupport.toRGBA255("gold"))

    save_file = vizard.save_file
    if save_file is not None:
        save_file = save_file.resolve()
        save_file.parent.mkdir(parents=True, exist_ok=True)
        if save_file.suffix.lower() != ".bin":
            (save_file.parent / "_VizFiles").mkdir(parents=True, exist_ok=True)
    save_arg = str(save_file) if save_file is not None else None
    viz = vizSupport.enableUnityVisualization(
        sc_sim,
        task_name,
        sc_list,
        saveFile=save_arg,
        liveStream=vizard.live_stream,
        spriteList=sprite_list,
        groundTrackColorList=ground_colors,
        oscOrbitColorList=osc_colors,
        trueOrbitColorList=true_colors,
    )
    _VIZARD_RETAIN.append(viz)
    viz.settings.showSpacecraftGroundTrack = 1
    viz.settings.showSpacecraftLabels = 1
    viz.settings.orbitLinesOn = 2
    viz.settings.spacecraftOrbitLineWidth = 2.0
    vizSupport.createStandardCamera(
        viz,
        spacecraftName=sc_object.ModelTag,
        setMode=0,
        setView=0,
        bodyTarget=earth.displayName,
        fieldOfView=45.0 * macros.D2R,
        displayName="Handoff chase camera",
    )
    if reference_sc is not None:
        vizSupport.createTargetLine(
            viz,
            fromBodyName=sc_object.ModelTag,
            toBodyName=reference_sc.ModelTag,
            lineColor="yellow",
        )

    if save_file is not None and save_file.suffix.lower() == ".bin":
        return str(save_file)
    if save_arg is None:
        return None
    file_name = Path(save_arg).stem
    file_path = Path(save_arg).parent if Path(save_arg).parent.as_posix() != "." else Path(".")
    return str(file_path / "_VizFiles" / f"{file_name}_UnityViz.bin")
