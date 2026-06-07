#!/usr/bin/env python3
import csv
import math
import os
import sys
from collections import Counter


def safe_float(value):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return None


def parse_twist_csv(path):
    samples = 0
    nonzero = 0
    max_linear = 0.0
    max_angular = 0.0
    final_zero = True

    if not os.path.isfile(path):
        return samples, nonzero, max_linear, max_angular, final_zero

    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            numeric = []
            for item in row:
                value = safe_float(item)
                if value is not None:
                    numeric.append(value)
            if len(numeric) < 6:
                continue

            comps = numeric[-6:]
            linear = max(abs(value) for value in comps[:3])
            angular = max(abs(value) for value in comps[3:])
            if linear > 1e-6 or angular > 1e-6:
                nonzero += 1
            max_linear = max(max_linear, linear)
            max_angular = max(max_angular, angular)
            final_zero = linear <= 1e-6 and angular <= 1e-6
            samples += 1

    return samples, nonzero, max_linear, max_angular, final_zero


def count_boolean_true(path):
    count = 0
    if not os.path.isfile(path):
        return count

    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            for item in row:
                lowered = str(item).strip().lower()
                if lowered in {"true", "1", "1.0"}:
                    count += 1
                    break
    return count


def parse_pose_csv(path):
    samples = 0
    first_xyz = None
    last_xyz = None

    if not os.path.isfile(path):
        return samples, 0.0

    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 6:
                continue
            x = safe_float(row[3])
            y = safe_float(row[4])
            z = safe_float(row[5])
            if x is None or y is None or z is None:
                continue
            xyz = (x, y, z)
            if first_xyz is None:
                first_xyz = xyz
            last_xyz = xyz
            samples += 1

    if first_xyz is None or last_xyz is None:
        return samples, 0.0

    displacement = math.sqrt(
        (last_xyz[0] - first_xyz[0]) ** 2
        + (last_xyz[1] - first_xyz[1]) ** 2
        + (last_xyz[2] - first_xyz[2]) ** 2
    )
    return samples, displacement


def count_mode_timeline_samples(run_dir):
    total = 0
    for name in (
        "mode_timeline_tracer1.csv",
        "mode_timeline_tracer2.csv",
        "mode_timeline_tracer3.csv",
    ):
        path = os.path.join(run_dir, name)
        if not os.path.isfile(path):
            continue
        with open(path, newline="") as handle:
            lines = sum(1 for line in handle if line.strip())
        if lines > 0:
            lines -= 1
        total += lines
    return total


ROBOTS = ["tracer1", "tracer2", "tracer3"]
ROBOT_MOCAP = {"tracer1": "rigid17_pose", "tracer2": "rigid14_pose", "tracer3": "rigid15_pose"}
ALL_TOPICS = [
    "/tracer1/cmd_goal", "/tracer1/cmd_vel_desired", "/tracer1/cmd_vel_stamped",
    "/tracer1/cmd_vel", "/tracer1/cmd_stop",
    "/tracer2/cmd_goal", "/tracer2/cmd_vel_desired", "/tracer2/cmd_vel_stamped",
    "/tracer2/cmd_vel", "/tracer2/cmd_stop",
    "/tracer3/cmd_goal", "/tracer3/cmd_vel_desired", "/tracer3/cmd_vel_stamped",
    "/tracer3/cmd_vel", "/tracer3/cmd_stop",
    "/wing_alignment/emergency_stop",
    "/Rigid17/pose", "/Rigid14/pose", "/Rigid15/pose", "/Rigid8/pose",
]

ALT_TOPICS = [
    "mode_timeline_tracer1", "mode_timeline_tracer2", "mode_timeline_tracer3",
    "rx_tracer1", "rx_tracer2", "rx_tracer3",
    "ts_tracer1", "ts_tracer2", "ts_tracer3",
]


def safe_int_row(run_dir, csv_name):
    path = os.path.join(run_dir, csv_name)
    if not os.path.isfile(path):
        return 0
    try:
        with open(path) as fh:
            total = sum(1 for _ in fh)
        return max(total - 1, 0)
    except OSError:
        return 0


def percentile(values, pct):
    if not values:
        return None
    s = sorted(values)
    idx = int(math.ceil(pct / 100.0 * len(s))) - 1
    return s[max(0, min(idx, len(s) - 1))]


def collect_mode_kpi(run_dir):
    mode_kpi = {}
    for robot in ROBOTS:
        path = os.path.join(run_dir, f"mode_timeline_{robot}.csv")
        row = {
            "robot_id": robot,
            "mode_timeline_samples": 0,
            "transmission_mode_dist": "none",
            "execution_mode_dist": "none",
            "phase_dist": "none",
            "AoI_ms_mean": "N/A",
            "AoI_ms_p95": "N/A",
            "effective_freshness_mean": "N/A",
            "effective_freshness_p05": "N/A",
            "safety_override_count": 0,
            "stop_reason_dist": "none",
            "watchdog_state_dist": "none",
            "cmd_v_in_mean": "N/A",
            "cmd_v_out_mean": "N/A",
            "cmd_vel_in_out_ratio": "N/A",
        }
        if not os.path.isfile(path):
            mode_kpi[robot] = row
            continue

        tx_modes = Counter()
        ex_modes = Counter()
        phases = Counter()
        stop_reasons = Counter()
        wd_states = Counter()
        aoi_vals = []
        freshness_vals = []
        safety_count = 0
        v_in_vals = []
        v_out_vals = []
        samples = 0

        with open(path, newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header is None:
                mode_kpi[robot] = row
                continue
            col = {name.strip(): i for i, name in enumerate(header)}
            for record in reader:
                samples += 1
                if len(record) < 19:
                    continue
                tx = record[col.get("transmission_mode", 4)].strip()
                ex = record[col.get("execution_mode", 5)].strip()
                ph = record[col.get("phase", 8)].strip()
                sr = record[col.get("stop_reason", 10)].strip()
                wd = record[col.get("watchdog_state", 11)].strip()
                aoi = safe_float(record[col.get("AoI_ms", 6)])
                eff = safe_float(record[col.get("effective_freshness", 7)])
                vi = safe_float(record[col.get("cmd_v_in", 12)])
                vo = safe_float(record[col.get("cmd_v_out", 14)])

                if tx:
                    tx_modes[tx] += 1
                if ex:
                    ex_modes[ex] += 1
                if ph:
                    phases[ph] += 1
                if sr:
                    stop_reasons[sr] += 1
                if wd:
                    wd_states[wd] += 1
                if aoi is not None:
                    aoi_vals.append(aoi)
                if eff is not None:
                    freshness_vals.append(eff)
                if sr and sr != "NONE":
                    safety_count += 1
                if vi is not None:
                    v_in_vals.append(abs(vi))
                if vo is not None:
                    v_out_vals.append(abs(vo))

        def fmt_dist(counter):
            if not counter:
                return "none"
            return ";".join(f"{k}={v}" for k, v in counter.most_common(5))

        row["mode_timeline_samples"] = samples
        row["transmission_mode_dist"] = fmt_dist(tx_modes)
        row["execution_mode_dist"] = fmt_dist(ex_modes)
        row["phase_dist"] = fmt_dist(phases)
        row["stop_reason_dist"] = fmt_dist(stop_reasons)
        row["watchdog_state_dist"] = fmt_dist(wd_states)
        row["safety_override_count"] = safety_count

        if aoi_vals:
            row["AoI_ms_mean"] = f"{sum(aoi_vals) / len(aoi_vals):.2f}"
            row["AoI_ms_p95"] = f"{percentile(aoi_vals, 95):.2f}"
        if freshness_vals:
            row["effective_freshness_mean"] = f"{sum(freshness_vals) / len(freshness_vals):.2f}"
            row["effective_freshness_p05"] = f"{percentile(freshness_vals, 5):.2f}"
        if v_in_vals:
            row["cmd_v_in_mean"] = f"{sum(v_in_vals) / len(v_in_vals):.6f}"
        if v_out_vals:
            row["cmd_v_out_mean"] = f"{sum(v_out_vals) / len(v_out_vals):.6f}"
        if v_in_vals and v_out_vals:
            mean_in = sum(v_in_vals) / len(v_in_vals)
            mean_out = sum(v_out_vals) / len(v_out_vals)
            if mean_in > 1e-9:
                row["cmd_vel_in_out_ratio"] = f"{mean_out / mean_in:.6f}"
            else:
                row["cmd_vel_in_out_ratio"] = "inf"

        mode_kpi[robot] = row
    return mode_kpi


def write_mode_kpi_summary(run_dir, mode_kpi):
    path = os.path.join(run_dir, "mode_kpi_summary.csv")
    columns = [
        "robot_id", "mode_timeline_samples", "transmission_mode_dist",
        "execution_mode_dist", "phase_dist", "AoI_ms_mean", "AoI_ms_p95",
        "effective_freshness_mean", "effective_freshness_p05",
        "safety_override_count", "stop_reason_dist", "watchdog_state_dist",
        "cmd_v_in_mean", "cmd_v_out_mean", "cmd_vel_in_out_ratio",
    ]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for robot in ROBOTS:
            writer.writerow(mode_kpi[robot])


PAYLOAD_BYTES_EST = 128
PHASE_WEIGHTS = {
    "standby": 0.2,
    "approach": 1.0,
    "transport": 1.2,
    "align": 1.5,
    "slide_align": 1.5,
    "docking": 2.0,
    "final_align": 2.0,
}


def collect_communication_kpi(run_dir):
    comm_kpi = {}
    for robot in ROBOTS:
        path = os.path.join(run_dir, f"mode_timeline_{robot}.csv")
        row = {
            "robot_id": robot,
            "tx_count": 0,
            "tx_rate_hz": "N/A",
            "payload_bytes_est": PAYLOAD_BYTES_EST,
            "traffic_bytes_est": "N/A",
            "traffic_rate_Bps_est": "N/A",
            "source_to_rx_delay_ms_mean": "N/A",
            "source_to_rx_delay_ms_p95": "N/A",
            "rx_to_watchdog_delay_ms_mean": "N/A",
            "rx_to_watchdog_delay_ms_p95": "N/A",
            "source_to_watchdog_delay_ms_mean": "N/A",
            "source_to_watchdog_delay_ms_p95": "N/A",
            "interarrival_ms_mean": "N/A",
            "interarrival_ms_p95": "N/A",
            "jitter_ms_std": "N/A",
            "stale_ratio_50ms": "N/A",
            "stale_ratio_100ms": "N/A",
            "stale_ratio_200ms": "N/A",
            "desired_to_stamped_ratio": "N/A",
            "cmd_delta_norm_mean": "N/A",
            "cmd_delta_norm_p95": "N/A",
            "VoI_proxy_mean": "N/A",
            "VoI_proxy_p95": "N/A",
            "VoI_proxy_sum": "N/A",
            "high_VoI_event_count": 0,
            "phase_nonstandby_ratio": "N/A",
            "phase_unique_count": 0,
            "phase_validity_note": "N/A",
            "active_start_time": "N/A",
            "active_start_reason": "N/A",
            "warmup_excluded_sec": "N/A",
            "cmd_goal_first_time": "N/A",
            "cmd_vel_desired_first_time": "N/A",
            "cmd_vel_stamped_first_time": "N/A",
            "cmd_vel_output_first_time": "N/A",
            "active_alignment_note": "N/A",
        }
        if not os.path.isfile(path):
            comm_kpi[robot] = row
            continue

        source_to_rx = []
        rx_to_wd = []
        source_to_wd = []
        interarrival = []
        cmd_v_in_seq = []
        cmd_w_in_seq = []
        freshness_seq = []
        phase_seq = []
        timestamps = []
        aoi_ms_vals = []
        first_ts = None
        first_tw = None
        first_output_tw = None

        with open(path, newline="") as handle:
            reader = csv.reader(handle)
            header = next(reader, None)
            if header is None:
                continue
            col = {name.strip(): i for i, name in enumerate(header)}
            for record in reader:
                if len(record) < 19:
                    continue
                ts = safe_float(record[col.get("t_source", 16)])
                tr = safe_float(record[col.get("t_rx", 17)])
                tw = safe_float(record[col.get("t_watchdog", 18)])
                aoi = safe_float(record[col.get("AoI_ms", 6)])
                vi = safe_float(record[col.get("cmd_v_in", 12)])
                wi = safe_float(record[col.get("cmd_w_in", 13)])
                ef = safe_float(record[col.get("effective_freshness", 7)])
                ph = record[col.get("phase", 8)].strip()

                if first_ts is None and ts is not None and ts > 0.0:
                    first_ts = ts
                if first_tw is None and tw is not None and tw > 0.0:
                    first_tw = tw
                if first_output_tw is None and vi is not None and abs(vi) > 1e-9 and tw is not None and tw > 0.0:
                    first_output_tw = tw

                if ts is not None and ts > 0.0 and tr is not None and tr > 0.0:
                    source_to_rx.append((tr - ts) * 1000.0)
                if tr is not None and tr > 0.0 and tw is not None and tw > 0.0:
                    rx_to_wd.append((tw - tr) * 1000.0)
                if ts is not None and ts > 0.0 and tw is not None and tw > 0.0:
                    source_to_wd.append((tw - ts) * 1000.0)
                if tw is not None and tw > 0.0:
                    timestamps.append(tw)
                if aoi is not None:
                    aoi_ms_vals.append(aoi)
                if vi is not None:
                    cmd_v_in_seq.append(vi)
                if wi is not None:
                    cmd_w_in_seq.append(wi)
                if ef is not None:
                    freshness_seq.append(ef)
                if ph:
                    phase_seq.append(ph)

        row["tx_count"] = len(timestamps)

        if len(timestamps) >= 2:
            t_span = timestamps[-1] - timestamps[0]
            if t_span > 0.01:
                row["tx_rate_hz"] = f"{(len(timestamps) / t_span):.2f}"
                row["traffic_bytes_est"] = len(timestamps) * PAYLOAD_BYTES_EST
                row["traffic_rate_Bps_est"] = f"{(len(timestamps) * PAYLOAD_BYTES_EST / t_span):.2f}"
            diffs = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
            ia = [d * 1000.0 for d in diffs]
            if ia:
                row["interarrival_ms_mean"] = f"{sum(ia) / len(ia):.2f}"
                row["interarrival_ms_p95"] = f"{percentile(ia, 95):.2f}"
                mean_ia = sum(ia) / len(ia)
                row["jitter_ms_std"] = f"{math.sqrt(sum((x - mean_ia) ** 2 for x in ia) / len(ia)):.2f}"

        if source_to_rx:
            row["source_to_rx_delay_ms_mean"] = f"{sum(source_to_rx) / len(source_to_rx):.2f}"
            row["source_to_rx_delay_ms_p95"] = f"{percentile(source_to_rx, 95):.2f}"
        if rx_to_wd:
            row["rx_to_watchdog_delay_ms_mean"] = f"{sum(rx_to_wd) / len(rx_to_wd):.2f}"
            row["rx_to_watchdog_delay_ms_p95"] = f"{percentile(rx_to_wd, 95):.2f}"
        if source_to_wd:
            row["source_to_watchdog_delay_ms_mean"] = f"{sum(source_to_wd) / len(source_to_wd):.2f}"
            row["source_to_watchdog_delay_ms_p95"] = f"{percentile(source_to_wd, 95):.2f}"

        if aoi_ms_vals:
            stale_50 = sum(1 for v in aoi_ms_vals if v > 50) / len(aoi_ms_vals)
            stale_100 = sum(1 for v in aoi_ms_vals if v > 100) / len(aoi_ms_vals)
            stale_200 = sum(1 for v in aoi_ms_vals if v > 200) / len(aoi_ms_vals)
            row["stale_ratio_50ms"] = f"{stale_50:.4f}"
            row["stale_ratio_100ms"] = f"{stale_100:.4f}"
            row["stale_ratio_200ms"] = f"{stale_200:.4f}"

        cmd_deltas = []
        for i in range(1, min(len(cmd_v_in_seq), len(cmd_w_in_seq))):
            dv = abs(cmd_v_in_seq[i] - cmd_v_in_seq[i - 1])
            dw = abs(cmd_w_in_seq[i] - cmd_w_in_seq[i - 1])
            delta_norm = dv + 0.5 * dw
            cmd_deltas.append(delta_norm)

        if cmd_deltas:
            row["cmd_delta_norm_mean"] = f"{sum(cmd_deltas) / len(cmd_deltas):.6f}"
            row["cmd_delta_norm_p95"] = f"{percentile(cmd_deltas, 95):.6f}"

        voi_vals = []
        high_voi_count = 0
        min_len = min(len(cmd_deltas), len(freshness_seq), len(phase_seq))
        for i in range(min_len):
            delta = cmd_deltas[i]
            ef = freshness_seq[i]
            ph = phase_seq[i] if i < len(phase_seq) else "standby"
            pw = PHASE_WEIGHTS.get(ph, 1.0)
            voi = pw * delta * ef
            voi_vals.append(voi)
            if voi > 0.01:
                high_voi_count += 1

        if voi_vals:
            row["VoI_proxy_mean"] = f"{sum(voi_vals) / len(voi_vals):.6f}"
            row["VoI_proxy_p95"] = f"{percentile(voi_vals, 95):.6f}"
            row["VoI_proxy_sum"] = f"{sum(voi_vals):.6f}"
            row["high_VoI_event_count"] = high_voi_count

        if first_ts is not None:
            row["cmd_vel_stamped_first_time"] = f"{first_ts:.6f}"
        if first_tw is not None:
            row["cmd_vel_desired_first_time"] = f"{first_tw:.6f}"
        if first_output_tw is not None:
            row["cmd_vel_output_first_time"] = f"{first_output_tw:.6f}"
        if timestamps:
            row["active_start_time"] = f"{timestamps[0]:.6f}"
            row["warmup_excluded_sec"] = f"{timestamps[0] - first_tw:.6f}" if first_tw else "N/A"

        goal_first = None
        goal_path = os.path.join(run_dir, f"cmd_goal_{robot}.csv")
        if os.path.isfile(goal_path):
            with open(goal_path, newline="") as fh:
                for rec in csv.reader(fh):
                    val = safe_float(rec[0]) if len(rec) > 0 else None
                    if val is not None and val > 0.0:
                        goal_first = val
                        break
        if goal_first is not None:
            row["cmd_goal_first_time"] = f"{goal_first:.6f}"
            row["active_start_reason"] = "cmd_goal"
        elif first_ts is not None:
            row["active_start_reason"] = "cmd_vel_stamped"
        elif first_tw is not None:
            row["active_start_reason"] = "mode_timeline_first_tw"
        else:
            row["active_start_reason"] = "none"
            row["active_alignment_note"] = "no_activity_detected"

        if goal_first is None:
            row["active_alignment_note"] = "missing_cmd_goal_in_observer_window"

        if phase_seq:
            unique = len(set(p for p in phase_seq if p))
            nonstandby = sum(1 for p in phase_seq if p and p != "standby")
            row["phase_unique_count"] = unique
            row["phase_nonstandby_ratio"] = f"{nonstandby / len(phase_seq):.4f}"
            if nonstandby == 0:
                row["phase_validity_note"] = "standby_only_phase_label"
            else:
                row["phase_validity_note"] = "phase_varies"
        else:
            row["phase_validity_note"] = "no_phase_data"

        ds = safe_int_row(run_dir, f"cmd_vel_desired_{robot}.csv")
        ss = safe_int_row(run_dir, f"cmd_vel_stamped_{robot}.csv")
        if ds > 0:
            row["desired_to_stamped_ratio"] = f"{ss / ds:.4f}"
        elif ds == 0 and ss > 0:
            row["desired_to_stamped_ratio"] = "N/A"
            row["phase_validity_note"] = f"{row['phase_validity_note']};stamped_without_desired"
        # else: ds == 0 and ss == 0 → stays "N/A"

        comm_kpi[robot] = row
    return comm_kpi


def write_communication_kpi_summary(run_dir, comm_kpi):
    path = os.path.join(run_dir, "communication_kpi_summary.csv")
    columns = [
        "robot_id", "tx_count", "tx_rate_hz", "payload_bytes_est",
        "traffic_bytes_est", "traffic_rate_Bps_est",
        "source_to_rx_delay_ms_mean", "source_to_rx_delay_ms_p95",
        "rx_to_watchdog_delay_ms_mean", "rx_to_watchdog_delay_ms_p95",
        "source_to_watchdog_delay_ms_mean", "source_to_watchdog_delay_ms_p95",
        "interarrival_ms_mean", "interarrival_ms_p95",
        "jitter_ms_std", "stale_ratio_50ms", "stale_ratio_100ms",
        "stale_ratio_200ms", "desired_to_stamped_ratio",
        "cmd_delta_norm_mean", "cmd_delta_norm_p95",
        "VoI_proxy_mean", "VoI_proxy_p95", "VoI_proxy_sum",
        "high_VoI_event_count",
        "phase_nonstandby_ratio", "phase_unique_count", "phase_validity_note",
        "active_start_time", "active_start_reason", "warmup_excluded_sec",
        "cmd_goal_first_time", "cmd_vel_desired_first_time",
        "cmd_vel_stamped_first_time", "cmd_vel_output_first_time",
        "active_alignment_note",
    ]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for robot in ROBOTS:
            writer.writerow(comm_kpi[robot])


def collect_per_robot(run_dir):
    data = {}
    for robot in ROBOTS:
        short = robot
        goal_samples, _, _, _, _ = parse_twist_csv(os.path.join(run_dir, f"cmd_goal_{short}.csv"))
        desired_samples, _, _, _, _ = parse_twist_csv(os.path.join(run_dir, f"cmd_vel_desired_{short}.csv"))
        stamped_samples, _, _, _, _ = parse_twist_csv(os.path.join(run_dir, f"cmd_vel_stamped_{short}.csv"))
        output_samples, nonzero, max_linear, max_angular, final_zero = parse_twist_csv(
            os.path.join(run_dir, f"cmd_vel_{short}.csv")
        )
        stop_true = count_boolean_true(os.path.join(run_dir, f"cmd_stop_{short}.csv"))
        mocap_key = ROBOT_MOCAP[robot]
        _, displacement = parse_pose_csv(os.path.join(run_dir, f"{mocap_key}.csv"))

        data[robot] = {
            "robot_id": robot,
            "cmd_goal_samples": int(goal_samples),
            "cmd_vel_desired_samples": int(desired_samples),
            "cmd_vel_stamped_samples": int(stamped_samples),
            "cmd_vel_output_samples": int(output_samples),
            "cmd_vel_output_nonzero_samples": int(nonzero),
            "max_abs_linear": max_linear,
            "max_abs_angular": max_angular,
            "cmd_stop_true_count": int(stop_true),
            "estimated_displacement": displacement,
            "final_cmd_vel_zero": final_zero,
        }
    return data


def collect_topic_flow(run_dir):
    stem_map = {
        "/tracer1/cmd_goal": "cmd_goal_tracer1",
        "/tracer1/cmd_vel_desired": "cmd_vel_desired_tracer1",
        "/tracer1/cmd_vel_stamped": "cmd_vel_stamped_tracer1",
        "/tracer1/cmd_vel": "cmd_vel_tracer1",
        "/tracer1/cmd_stop": "cmd_stop_tracer1",
        "/tracer2/cmd_goal": "cmd_goal_tracer2",
        "/tracer2/cmd_vel_desired": "cmd_vel_desired_tracer2",
        "/tracer2/cmd_vel_stamped": "cmd_vel_stamped_tracer2",
        "/tracer2/cmd_vel": "cmd_vel_tracer2",
        "/tracer2/cmd_stop": "cmd_stop_tracer2",
        "/tracer3/cmd_goal": "cmd_goal_tracer3",
        "/tracer3/cmd_vel_desired": "cmd_vel_desired_tracer3",
        "/tracer3/cmd_vel_stamped": "cmd_vel_stamped_tracer3",
        "/tracer3/cmd_vel": "cmd_vel_tracer3",
        "/tracer3/cmd_stop": "cmd_stop_tracer3",
        "/wing_alignment/emergency_stop": "emergency_stop",
        "/Rigid17/pose": "rigid17_pose",
        "/Rigid14/pose": "rigid14_pose",
        "/Rigid15/pose": "rigid15_pose",
        "/Rigid8/pose": "rigid8_pose",
    }

    flow = {}
    for topic, stem in stem_map.items():
        path = os.path.join(run_dir, f"{stem}.csv")
        if os.path.isfile(path):
            with open(path, newline="") as handle:
                samples = sum(1 for _ in handle)
            flow[topic] = samples
        else:
            flow[topic] = 0

    for alt in ALT_TOPICS:
        samples = safe_int_row(run_dir, f"{alt}.csv")
        flow[alt] = samples

    return flow


def write_run_summary(run_dir, per_robot, aggregate):
    path = os.path.join(run_dir, "run_summary.csv")
    columns = [
        "robot_id", "cmd_goal_samples", "cmd_vel_desired_samples",
        "cmd_vel_stamped_samples", "cmd_vel_output_samples",
        "cmd_vel_output_nonzero_samples", "max_abs_linear",
        "max_abs_angular", "cmd_stop_true_count",
        "estimated_displacement", "final_cmd_vel_zero",
    ]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for robot in ROBOTS:
            writer.writerow(per_robot[robot])
        agg_row = {
            "robot_id": "aggregate",
            "cmd_goal_samples": aggregate["any_robot_cmd_goal_samples"],
            "cmd_vel_desired_samples": aggregate["any_robot_desired_samples"],
            "cmd_vel_stamped_samples": aggregate["any_robot_stamped_samples"],
            "cmd_vel_output_samples": aggregate["any_robot_cmd_vel_output_samples"],
            "cmd_vel_output_nonzero_samples": aggregate["any_robot_cmd_vel_nonzero_samples"],
            "max_abs_linear": aggregate["max_abs_linear_any"],
            "max_abs_angular": aggregate["max_abs_angular_any"],
            "cmd_stop_true_count": "",
            "estimated_displacement": "",
            "final_cmd_vel_zero": "",
        }
        writer.writerow(agg_row)


def write_topic_flow_summary(run_dir, flow):
    path = os.path.join(run_dir, "topic_flow_summary.csv")
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["topic", "samples"])
        for topic in ALL_TOPICS:
            writer.writerow([topic, flow.get(topic, 0)])
        for alt in ALT_TOPICS:
            writer.writerow([alt, flow.get(alt, 0)])


def write_reduced_policy_summary(run_dir):
    path = os.path.join(run_dir, "reduced_decisions.csv")
    if not os.path.isfile(path):
        return
    from collections import Counter
    import csv
    robot_stats = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rid = row.get("robot_id", "").strip()
            if not rid:
                continue
            if rid not in robot_stats:
                robot_stats[rid] = {
                    "rp": row.get("reduced_policy", ""),
                    "eo": row.get("enable_reduced_output", "0"),
                    "reasons": Counter(),
                    "fc_last": 0, "tx_last": 0, "sup_last": 0,
                }
            s = robot_stats[rid]
            s["reasons"][row.get("reduced_suppress_reason", "?")] += 1
            fi = int(row.get("full_input_count_so_far", 0) or 0)
            tx = int(row.get("reduced_tx_count_so_far", 0) or 0)
            su = int(row.get("suppressed_count_so_far", 0) or 0)
            s["fc_last"] = max(s["fc_last"], fi)
            s["tx_last"] = max(s["tx_last"], tx)
            s["sup_last"] = max(s["sup_last"], su)
    out_path = os.path.join(run_dir, "reduced_policy_summary.csv")
    columns = [
        "robot_id", "reduced_policy", "enable_reduced_output",
        "full_input_count", "reduced_tx_count", "suppressed_count",
        "tx_reduction_vs_input", "send_reason_dist",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for rid, s in sorted(robot_stats.items()):
            fi = s["fc_last"]
            tx = s["tx_last"]
            su = s["sup_last"]
            reduction = ((fi - tx) / fi) if fi > 0 else 0.0
            w.writerow({
                "robot_id": rid,
                "reduced_policy": s["rp"],
                "enable_reduced_output": s["eo"],
                "full_input_count": fi,
                "reduced_tx_count": tx,
                "suppressed_count": su,
                "tx_reduction_vs_input": f"{reduction:.4f}",
                "send_reason_dist": ";".join(f"{k}={v}" for k, v in s["reasons"].most_common(10)),
            })


def write_shadow_policy_summary(run_dir):
    path = os.path.join(run_dir, "shadow_decisions.csv")
    if not os.path.isfile(path):
        return
    from collections import Counter
    robot_stats = {}
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            return
        col = {name.strip(): i for i, name in enumerate(header)}
        for row in reader:
            if len(row) < 13:
                continue
            rid = row[col.get("robot_id", 1)].strip()
            ws = row[col.get("would_send", 9)].strip()
            sr = row[col.get("send_reason", 10)].strip()
            sp = row[col.get("shadow_policy", 8)].strip()
            dt = safe_float(row[col.get("shadow_delta_threshold", 11)])
            mh = safe_float(row[col.get("shadow_max_hold_ms", 12)])
            pb = safe_float(row[col.get("shadow_payload_bytes", 15)])
            sx = safe_float(row[col.get("shadow_tx_count_so_far", 13)])
            fx = safe_float(row[col.get("full_update_count_so_far", 14)])
            if rid not in robot_stats:
                robot_stats[rid] = {
                    "sp": sp, "dt": dt, "mh": mh, "pb": pb,
                    "full_count": 0, "shadow_tx": 0,
                    "reasons": Counter(), "max_fc": 0, "max_stx": 0,
                }
            s = robot_stats[rid]
            s["full_count"] += 1
            if ws == "1":
                s["shadow_tx"] += 1
            s["reasons"][sr] += 1
            if fx is not None:
                s["max_fc"] = max(s["max_fc"], int(fx))
            if sx is not None:
                s["max_stx"] = max(s["max_stx"], int(sx))
    out_path = os.path.join(run_dir, "shadow_policy_summary.csv")
    columns = [
        "robot_id", "shadow_policy", "shadow_delta_threshold",
        "shadow_max_hold_ms", "full_update_count", "shadow_tx_count",
        "shadow_tx_reduction_ratio", "shadow_tx_rate_hz",
        "max_hold_send_count", "delta_send_count", "first_send_count",
        "no_send_count", "shadow_payload_bytes",
        "shadow_traffic_bytes_est", "shadow_traffic_rate_Bps_est",
    ]
    with open(out_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for rid, s in sorted(robot_stats.items()):
            fc = s["full_count"]
            stx = s["shadow_tx"]
            fc_final = int(s["max_fc"]) if s["max_fc"] > fc else fc
            ratio = (fc_final - stx) / fc_final if fc_final > 0 else 0.0
            pb = int(s["pb"]) if (s["pb"] is not None and s["pb"] > 0) else 128
            writer.writerow({
                "robot_id": rid,
                "shadow_policy": s["sp"],
                "shadow_delta_threshold": f"{s['dt']:.4f}" if s["dt"] is not None else "N/A",
                "shadow_max_hold_ms": f"{s['mh']:.1f}" if s["mh"] is not None else "N/A",
                "full_update_count": fc,
                "shadow_tx_count": stx,
                "shadow_tx_reduction_ratio": f"{ratio:.4f}",
                "shadow_tx_rate_hz": "N/A",
                "max_hold_send_count": s["reasons"].get("max_hold", 0),
                "delta_send_count": s["reasons"].get("delta", 0),
                "first_send_count": s["reasons"].get("first", 0),
                "no_send_count": s["reasons"].get("no_send", 0),
                "shadow_payload_bytes": pb,
                "shadow_traffic_bytes_est": stx * pb,
                "shadow_traffic_rate_Bps_est": "N/A",
            })


def main():
    if len(sys.argv) != 2:
        print(
            "Usage: summarize_existing_launch_observer_kpi.py <run_dir>",
            file=sys.stderr,
        )
        sys.exit(2)

    run_dir = sys.argv[1]

    per_robot = collect_per_robot(run_dir)

    any_goal = max(r["cmd_goal_samples"] for r in per_robot.values())
    any_desired = max(r["cmd_vel_desired_samples"] for r in per_robot.values())
    any_stamped = max(r["cmd_vel_stamped_samples"] for r in per_robot.values())
    any_output = max(r["cmd_vel_output_samples"] for r in per_robot.values())
    any_nonzero = max(r["cmd_vel_output_nonzero_samples"] for r in per_robot.values())
    max_linear = max(r["max_abs_linear"] for r in per_robot.values())
    max_angular = max(r["max_abs_angular"] for r in per_robot.values())
    active_robots = [r["robot_id"] for r in per_robot.values() if r["cmd_vel_output_samples"] > 0]

    emergency_stop_true = count_boolean_true(os.path.join(run_dir, "emergency_stop.csv"))
    rigid17_samples, _ = parse_pose_csv(os.path.join(run_dir, "rigid17_pose.csv"))
    rigid14_samples, _ = parse_pose_csv(os.path.join(run_dir, "rigid14_pose.csv"))
    rigid15_samples, _ = parse_pose_csv(os.path.join(run_dir, "rigid15_pose.csv"))
    rigid8_samples, _ = parse_pose_csv(os.path.join(run_dir, "rigid8_pose.csv"))
    mocap_samples = rigid17_samples + rigid14_samples + rigid15_samples + rigid8_samples
    mode_timeline_total = count_mode_timeline_samples(run_dir)
    active_str = ",".join(active_robots) if active_robots else "none"

    aggregate = {
        "any_robot_cmd_goal_samples": int(any_goal),
        "any_robot_desired_samples": int(any_desired),
        "any_robot_stamped_samples": int(any_stamped),
        "any_robot_cmd_vel_nonzero_samples": int(any_nonzero),
        "any_robot_cmd_vel_output_samples": int(any_output),
        "active_robot_ids": active_str,
        "max_abs_linear_any": max_linear,
        "max_abs_angular_any": max_angular,
    }

    flow = collect_topic_flow(run_dir)
    mode_kpi = collect_mode_kpi(run_dir)
    comm_kpi = collect_communication_kpi(run_dir)

    write_run_summary(run_dir, per_robot, aggregate)
    write_topic_flow_summary(run_dir, flow)
    write_mode_kpi_summary(run_dir, mode_kpi)
    write_communication_kpi_summary(run_dir, comm_kpi)
    write_shadow_policy_summary(run_dir)
    write_reduced_policy_summary(run_dir)

    for robot in ROBOTS:
        r = per_robot[robot]
        prefix = robot.upper()
        print(f"PK_{prefix}_CMD_GOAL_SAMPLES={r['cmd_goal_samples']}")
        print(f"PK_{prefix}_CMD_VEL_DESIRED_SAMPLES={r['cmd_vel_desired_samples']}")
        print(f"PK_{prefix}_CMD_VEL_STAMPED_SAMPLES={r['cmd_vel_stamped_samples']}")
        print(f"PK_{prefix}_CMD_VEL_OUTPUT_SAMPLES={r['cmd_vel_output_samples']}")
        print(f"PK_{prefix}_CMD_VEL_OUTPUT_NONZERO_SAMPLES={r['cmd_vel_output_nonzero_samples']}")
        print(f"PK_{prefix}_MAX_ABS_LINEAR={r['max_abs_linear']:.6f}")
        print(f"PK_{prefix}_MAX_ABS_ANGULAR={r['max_abs_angular']:.6f}")
        print(f"PK_{prefix}_CMD_STOP_TRUE_COUNT={r['cmd_stop_true_count']}")
        print(f"PK_{prefix}_ESTIMATED_DISPLACEMENT={r['estimated_displacement']:.6f}")
        print(f"PK_{prefix}_FINAL_CMD_VEL_ZERO={'true' if r['final_cmd_vel_zero'] else 'false'}")

    print(f"PK_EMERGENCY_STOP_TRUE_COUNT={int(emergency_stop_true)}")
    print(f"PK_MOCAP_SAMPLES={int(mocap_samples)}")
    print(f"PK_MODE_TIMELINE_SAMPLES={int(mode_timeline_total)}")
    print(f"PK_ACTIVE_ROBOT_IDS={active_str}")
    print(f"PK_ANY_ROBOT_CMD_GOAL_SAMPLES={aggregate['any_robot_cmd_goal_samples']}")
    print(f"PK_ANY_ROBOT_DESIRED_SAMPLES={aggregate['any_robot_desired_samples']}")
    print(f"PK_ANY_ROBOT_STAMPED_SAMPLES={aggregate['any_robot_stamped_samples']}")
    print(f"PK_ANY_ROBOT_CMD_VEL_NONZERO_SAMPLES={aggregate['any_robot_cmd_vel_nonzero_samples']}")
    print(f"PK_MAX_ABS_LINEAR_ANY={aggregate['max_abs_linear_any']:.6f}")
    print(f"PK_MAX_ABS_ANGULAR_ANY={aggregate['max_abs_angular_any']:.6f}")


if __name__ == "__main__":
    main()
