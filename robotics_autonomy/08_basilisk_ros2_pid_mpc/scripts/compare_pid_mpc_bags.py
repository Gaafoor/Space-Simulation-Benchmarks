#!/usr/bin/env python3
"""Compare PID and MPC ROS2 MCAP bags for leader/follower formation runs.

Outputs:
  - summary_metrics.csv: per-run, per-namespace scalar metrics
  - step_metrics.csv: per-step (reference change) error metrics
  - timeseries_<run>_<namespace>.csv: ready-to-plot aligned time series

Requirements:
  pip install mcap mcap-ros2-support
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

from mcap_ros2.reader import read_ros2_messages


NAMESPACES = ("leaderSc", "followerSc_1", "followerSc_2")


@dataclass
class PoseSample:
    t: float
    p: Tuple[float, float, float]
    q: Tuple[float, float, float, float]  # w, x, y, z


@dataclass
class HillSample:
    t: float
    r: Tuple[float, float, float]
    v: Tuple[float, float, float]


@dataclass
class VecSample:
    t: float
    x: Tuple[float, float, float]


def l2(vec: Sequence[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in vec))


def find_bag_file(bag_dir: Path) -> Path:
    mcaps = sorted(bag_dir.glob("*.mcap"))
    if not mcaps:
        raise FileNotFoundError(f"No .mcap file found in {bag_dir}")
    return mcaps[0]


def parse_run(
    bag_file: Path,
    run_kind: str,
) -> Dict[str, Dict[str, List]]:
    """Parse topics needed for one run.

    run_kind: 'pid' or 'mpc'
    """
    if run_kind not in ("pid", "mpc"):
        raise ValueError("run_kind must be 'pid' or 'mpc'")

    data: Dict[str, Dict[str, List]] = {}
    for ns in NAMESPACES:
        data[ns] = {"ref": [], "hill": [], "force": [], "torque": []}

    ref_suffix = "pid/setpoint_pose" if run_kind == "pid" else "bsk_mpc/vehicle_pose_ref"

    for m in read_ros2_messages(str(bag_file)):
        topic = m.channel.topic
        msg = m.ros_msg
        t = m.log_time_ns * 1e-9
        for ns in NAMESPACES:
            prefix = f"/{ns}/"
            if not topic.startswith(prefix):
                continue
            if topic == f"{prefix}{ref_suffix}":
                p = msg.pose.position
                q = msg.pose.orientation
                data[ns]["ref"].append(
                    PoseSample(t=t, p=(float(p.x), float(p.y), float(p.z)), q=(float(q.w), float(q.x), float(q.y), float(q.z)))
                )
            elif topic == f"{prefix}bsk/out/hill_trans_state":
                data[ns]["hill"].append(
                    HillSample(
                        t=t,
                        r=(float(msg.r_dc_h[0]), float(msg.r_dc_h[1]), float(msg.r_dc_h[2])),
                        v=(float(msg.v_dc_h[0]), float(msg.v_dc_h[1]), float(msg.v_dc_h[2])),
                    )
                )
            elif topic == f"{prefix}bsk/in/cmd_force":
                data[ns]["force"].append(
                    VecSample(
                        t=t,
                        x=(
                            float(msg.forcerequestbody[0]),
                            float(msg.forcerequestbody[1]),
                            float(msg.forcerequestbody[2]),
                        ),
                    )
                )
            elif topic == f"{prefix}bsk/in/cmd_torque":
                data[ns]["torque"].append(
                    VecSample(
                        t=t,
                        x=(
                            float(msg.torquerequestbody[0]),
                            float(msg.torquerequestbody[1]),
                            float(msg.torquerequestbody[2]),
                        ),
                    )
                )
    return data


def detect_ref_changes(ref: List[PoseSample], pos_eps: float = 1e-4, quat_eps: float = 1e-4) -> List[float]:
    """Return timestamps where reference changes (including first sample time)."""
    if not ref:
        return []
    changes = [ref[0].t]
    prev = ref[0]
    for s in ref[1:]:
        dp = l2((s.p[0] - prev.p[0], s.p[1] - prev.p[1], s.p[2] - prev.p[2]))
        dq = abs(s.q[0] - prev.q[0]) + abs(s.q[1] - prev.q[1]) + abs(s.q[2] - prev.q[2]) + abs(s.q[3] - prev.q[3])
        if dp > pos_eps or dq > quat_eps:
            changes.append(s.t)
            prev = s
    return changes


def hold_value_pose(samples: List[PoseSample], t_query: float, idx_hint: int) -> Tuple[PoseSample, int]:
    """Sample zero-order-hold pose at t_query, advancing from idx_hint."""
    i = idx_hint
    n = len(samples)
    while i + 1 < n and samples[i + 1].t <= t_query:
        i += 1
    return samples[i], i


def settling_time(
    times: List[float],
    err_norm: List[float],
    t_start: float,
    thresh: float = 0.05,
    hold_s: float = 5.0,
) -> float:
    """First time after t_start where error remains <= thresh for hold_s."""
    if not times:
        return math.nan
    n = len(times)
    for i in range(n):
        if times[i] < t_start:
            continue
        if err_norm[i] > thresh:
            continue
        t_end = times[i] + hold_s
        ok = True
        j = i
        while j < n and times[j] <= t_end:
            if err_norm[j] > thresh:
                ok = False
                break
            j += 1
        if ok and (j == n or times[min(j, n - 1)] >= t_end):
            return times[i] - t_start
    return math.nan


def integral_l2(vec_samples: List[VecSample]) -> float:
    if len(vec_samples) < 2:
        return 0.0
    acc = 0.0
    for a, b in zip(vec_samples[:-1], vec_samples[1:]):
        dt = b.t - a.t
        if dt <= 0.0:
            continue
        acc += 0.5 * (l2(a.x) + l2(b.x)) * dt
    return acc


def rmse(vals: List[float]) -> float:
    if not vals:
        return math.nan
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def write_csv(path: Path, header: Sequence[str], rows: Sequence[Sequence]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def build_metrics_for_run(
    run_name: str,
    data: Dict[str, Dict[str, List]],
    out_dir: Path,
    settle_thresh: float,
    settle_hold: float,
) -> Tuple[List[List], List[List]]:
    summary_rows: List[List] = []
    step_rows: List[List] = []

    for ns in NAMESPACES:
        ref = data[ns]["ref"]
        hill = data[ns]["hill"]
        force = data[ns]["force"]
        torque = data[ns]["torque"]
        if not ref or not hill:
            summary_rows.append([run_name, ns, "missing_ref_or_hill"])
            continue

        ref_changes = detect_ref_changes(ref)
        t_align = ref_changes[0] if ref_changes else hill[0].t

        # Build aligned time series sampled at Hill timestamps.
        ts_rel: List[float] = []
        rxs: List[float] = []
        rys: List[float] = []
        rzs: List[float] = []
        refxs: List[float] = []
        refys: List[float] = []
        refzs: List[float] = []
        exs: List[float] = []
        eys: List[float] = []
        ezs: List[float] = []
        ens: List[float] = []

        j = 0
        for h in hill:
            if h.t < ref[0].t:
                continue
            rs, j = hold_value_pose(ref, h.t, j)
            ex = rs.p[0] - h.r[0]
            ey = rs.p[1] - h.r[1]
            ez = rs.p[2] - h.r[2]
            en = l2((ex, ey, ez))
            ts_rel.append(h.t - t_align)
            rxs.append(h.r[0])
            rys.append(h.r[1])
            rzs.append(h.r[2])
            refxs.append(rs.p[0])
            refys.append(rs.p[1])
            refzs.append(rs.p[2])
            exs.append(ex)
            eys.append(ey)
            ezs.append(ez)
            ens.append(en)

        if not ts_rel:
            summary_rows.append([run_name, ns, "no_aligned_samples"])
            continue

        # Per-step metrics
        change_rel = [t - t_align for t in ref_changes]
        for k, t0 in enumerate(change_rel):
            t1 = change_rel[k + 1] if k + 1 < len(change_rel) else ts_rel[-1]
            idx = [i for i, t in enumerate(ts_rel) if t >= t0 and t <= t1]
            if not idx:
                continue
            step_err = [ens[i] for i in idx]
            st = settling_time([ts_rel[i] for i in idx], step_err, t_start=t0, thresh=settle_thresh, hold_s=settle_hold)
            step_rows.append(
                [
                    run_name,
                    ns,
                    k,
                    t0,
                    t1,
                    max(step_err),
                    rmse(step_err),
                    st if not math.isnan(st) else "",
                ]
            )

        # Whole-run summary
        # Skip pre-align negatives to avoid startup mismatch.
        idx_eval = [i for i, t in enumerate(ts_rel) if t >= 0.0]
        eval_t = [ts_rel[i] for i in idx_eval]
        eval_en = [ens[i] for i in idx_eval]

        st_global = settling_time(eval_t, eval_en, t_start=0.0, thresh=settle_thresh, hold_s=settle_hold)
        fnorms = [l2(f.x) for f in force]
        tnorms = [l2(t.x) for t in torque] if torque else []
        summary_rows.append(
            [
                run_name,
                ns,
                len(eval_t),
                eval_t[0] if eval_t else 0.0,
                eval_t[-1] if eval_t else 0.0,
                max(eval_en) if eval_en else math.nan,
                rmse(eval_en),
                st_global if not math.isnan(st_global) else "",
                max(fnorms) if fnorms else 0.0,
                (sum(fnorms) / len(fnorms)) if fnorms else 0.0,
                integral_l2(force),
                (max(tnorms) if tnorms else 0.0),
                integral_l2(torque) if torque else 0.0,
                len(ref_changes),
            ]
        )

        # Ready-to-plot time series CSV
        ts_rows = [
            [
                run_name,
                ns,
                ts_rel[i],
                rxs[i],
                rys[i],
                rzs[i],
                refxs[i],
                refys[i],
                refzs[i],
                exs[i],
                eys[i],
                ezs[i],
                ens[i],
            ]
            for i in range(len(ts_rel))
        ]
        write_csv(
            out_dir / f"timeseries_{run_name}_{ns}.csv",
            [
                "run",
                "namespace",
                "t_rel_s",
                "x_m",
                "y_m",
                "z_m",
                "x_ref_m",
                "y_ref_m",
                "z_ref_m",
                "err_x_m",
                "err_y_m",
                "err_z_m",
                "err_norm_m",
            ],
            ts_rows,
        )

    return summary_rows, step_rows


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare PID vs MPC MCAP bags and export metrics/plot CSVs.")
    ap.add_argument("--pid-dir", default="/home/mahmedext/ros2_ws/pid_run", help="Directory containing PID .mcap bag")
    ap.add_argument("--mpc-dir", default="/home/mahmedext/ros2_ws/mpc_run", help="Directory containing MPC .mcap bag")
    ap.add_argument("--out-dir", default="/home/mahmedext/ros2_ws/bag_compare_out", help="Output directory for CSV files")
    ap.add_argument("--settle-thresh", type=float, default=0.05, help="Settling error threshold in meters")
    ap.add_argument("--settle-hold", type=float, default=5.0, help="Seconds error must remain under threshold")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    ensure_dir(out_dir)

    pid_bag = find_bag_file(Path(args.pid_dir))
    mpc_bag = find_bag_file(Path(args.mpc_dir))

    print(f"Reading PID bag: {pid_bag}")
    pid_data = parse_run(pid_bag, "pid")
    print(f"Reading MPC bag: {mpc_bag}")
    mpc_data = parse_run(mpc_bag, "mpc")

    summary_rows: List[List] = []
    step_rows: List[List] = []

    s, st = build_metrics_for_run("pid", pid_data, out_dir, args.settle_thresh, args.settle_hold)
    summary_rows.extend(s)
    step_rows.extend(st)
    s, st = build_metrics_for_run("mpc", mpc_data, out_dir, args.settle_thresh, args.settle_hold)
    summary_rows.extend(s)
    step_rows.extend(st)

    write_csv(
        out_dir / "summary_metrics.csv",
        [
            "run",
            "namespace",
            "n_samples_eval",
            "t_eval_start_s",
            "t_eval_end_s",
            "peak_err_norm_m",
            "rmse_err_norm_m",
            "settling_time_s",
            "max_force_norm_N",
            "mean_force_norm_N",
            "integral_force_norm_Ns",
            "max_torque_norm_Nm",
            "integral_torque_norm_Nms",
            "n_ref_changes",
        ],
        summary_rows,
    )
    write_csv(
        out_dir / "step_metrics.csv",
        [
            "run",
            "namespace",
            "step_idx",
            "step_start_t_rel_s",
            "step_end_t_rel_s",
            "step_peak_err_norm_m",
            "step_rmse_err_norm_m",
            "step_settling_time_s",
        ],
        step_rows,
    )

    print(f"\nWrote outputs to: {out_dir}")
    print(" - summary_metrics.csv")
    print(" - step_metrics.csv")
    for ns in NAMESPACES:
        print(f" - timeseries_pid_{ns}.csv")
        print(f" - timeseries_mpc_{ns}.csv")


if __name__ == "__main__":
    main()

