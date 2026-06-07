#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Derive bounded proxy artifacts for hardware_preliminary runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import create_bench_run_manifest as manifest_script  # noqa: E402


COMMAND_RESIDENCE_FIELDS = [
    "run_id",
    "robot_id",
    "command_id",
    "seq",
    "join_key_type",
    "join_tolerance_ms",
    "join_quality",
    "t_tx_scheduler",
    "t_rx_watchdog",
    "t_first_apply_watchdog",
    "t_last_apply_watchdog",
    "net_receive_proxy_ms",
    "age_at_first_use_ms",
    "age_at_last_use_ms",
    "residence_apply_window_ms",
    "classification",
    "interpretation_note",
]

PHASE_ATTRIBUTED_FIELDS = [
    "run_id",
    "robot_id",
    "task_phase",
    "command_id",
    "command_type",
    "seq",
    "t_tx",
    "scheduler_decision",
    "precision_mode",
    "attributed_phase",
    "attributed_local_state",
    "phase_join_quality",
    "phase_join_tolerance_ms",
    "classification",
]

CONTROL_LOOP_TIMING_FIELDS = [
    "run_id",
    "stream_name",
    "robot_id",
    "slide_id",
    "topic",
    "sample_count",
    "mean_interarrival_ms",
    "median_interarrival_ms",
    "p95_interarrival_ms",
    "max_interarrival_ms",
    "gap_count_over_threshold",
    "classification",
    "interpretation_note",
]

EXECUTOR_BACKLOG_FIELDS = [
    "run_id",
    "stream_name",
    "proxy_source",
    "mean_callback_duration_ms",
    "p95_callback_duration_ms",
    "max_callback_duration_ms",
    "watchdog_queue_delay_proxy_ms_mean",
    "watchdog_queue_delay_proxy_ms_p95",
    "watchdog_queue_delay_proxy_ms_max",
    "delta_exec_proxy_ms_mean",
    "delta_exec_proxy_ms_p95",
    "delta_exec_proxy_ms_max",
    "burst_proxy_mean_interarrival_ms",
    "burst_proxy_min_interarrival_ms",
    "burst_proxy_short_gap_count",
    "classification",
    "interpretation_note",
]

AUTHORITY_PROXY_FIELDS = [
    "run_id",
    "timestamp",
    "robot_id",
    "slide_id",
    "chassis_topic",
    "slide_topic",
    "base_authority_weight_internal_proxy",
    "slide_authority_weight_internal_proxy",
    "authority_proxy_chassis_ratio",
    "authority_proxy_slide_ratio",
    "classification",
    "interpretation_note",
    "unavailable_reason",
]

TERMINAL_RESIDUAL_FIELDS = [
    "run_id",
    "source_used",
    "terminal_residual_proxy_mean",
    "terminal_residual_proxy_median",
    "terminal_residual_proxy_max",
    "support_residual_proxy",
    "classification",
    "interpretation_note",
]

DEFAULT_JOIN_TOLERANCE_MS = 100.0
DEFAULT_PHASE_TOLERANCE_MS = 500.0
CONTROL_LOOP_GAP_THRESHOLD_MS = 100.0
BURST_SHORT_GAP_MS = 40.0


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _read_csv(path: Path) -> tuple[list[str], list[dict]]:
    if not path.exists() or path.stat().st_size == 0:
        return [], []
    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        header = list(reader.fieldnames or [])
        rows = list(reader) if header else []
    return header, rows


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _fmt(value) -> str:
    if value in ("", None):
        return ""
    return f"{float(value):.6f}"


def _to_float(value):
    if value in ("", None):
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _truthy(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y", "on", "passed"}


def _first_existing(paths: list[str]) -> Path | None:
    for raw in paths:
        path = Path(raw).expanduser()
        if path.exists():
            return path
    return None


def _artifact_map(manifest: dict, key: str) -> dict[str, Path]:
    out: dict[str, Path] = {}
    for entry in manifest.get(key, []):
        if isinstance(entry, dict):
            name = str(entry.get("name", "")).strip()
            path = str(entry.get("path", "")).strip()
            if name and path:
                out[name] = Path(path).expanduser()
    return out


def _existing_csvs(directory: Path, pattern: str) -> list[Path]:
    if not directory.exists():
        return []
    return sorted(path for path in directory.glob(pattern) if path.is_file())


def _quantile(values: list[float], q: float):
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    return statistics.quantiles(values, n=100, method="inclusive")[int(max(0.0, min(99.0, q * 100.0))) - 1]


def _safe_mean(values: list[float]):
    return statistics.mean(values) if values else None


def _safe_median(values: list[float]):
    return statistics.median(values) if values else None


def _capture_dir_from_manifest(manifest: dict) -> Path:
    capture_map = _artifact_map(manifest, "capture_artifacts")
    if capture_map:
        return next(iter(capture_map.values())).parent
    return Path(".")


def _derived_dir_from_manifest(manifest: dict) -> Path:
    derived_map = _artifact_map(manifest, "derived_artifacts")
    if derived_map:
        return next(iter(derived_map.values())).parent
    return Path(".")


def _log_dir_from_manifest(manifest: dict, role: str) -> Path:
    csv_paths = manifest.get("csv_paths", {})
    existing = _first_existing(csv_paths.get(role, []))
    if existing is not None:
        return existing.parent
    if role in {"scheduler", "watchdog"}:
        root = manifest.get("log_roots", {}).get("cmd_safety_log_root", "")
        run_id = str(manifest.get("run_id", "")).strip()
        if root and run_id:
            return Path(root).expanduser() / run_id
    if role == "mission":
        root = manifest.get("log_roots", {}).get("mission_log_root", "")
        run_id = str(manifest.get("run_id", "")).strip()
        if root and run_id:
            return Path(root).expanduser() / run_id
    return Path(".")


def _load_inputs(manifest: dict, capture_dir: Path, scheduler_log_dir: Path, watchdog_log_dir: Path, mission_log_dir: Path) -> dict:
    capture_map = _artifact_map(manifest, "capture_artifacts")
    derived_map = _artifact_map(manifest, "derived_artifacts")
    scheduler_files = [path for path in _existing_csvs(scheduler_log_dir, "*.csv") if path.name in {"events.csv", "scheduler_audit.csv"}]
    rx_files = _existing_csvs(watchdog_log_dir, "rx_*.csv")
    ts_files = _existing_csvs(watchdog_log_dir, "ts_*.csv")
    mission_files = [path for path in _existing_csvs(mission_log_dir, "mission_runtime_events.csv")]
    return {
        "capture_map": capture_map,
        "derived_map": derived_map,
        "scheduler_files": scheduler_files,
        "rx_files": rx_files,
        "ts_files": ts_files,
        "mission_files": mission_files,
        "capture_dir": capture_dir,
        "scheduler_log_dir": scheduler_log_dir,
        "watchdog_log_dir": watchdog_log_dir,
        "mission_log_dir": mission_log_dir,
    }


def _artifact_rows(files: list[Path]) -> tuple[list[str], list[dict]]:
    header: list[str] = []
    rows: list[dict] = []
    for path in files:
        this_header, this_rows = _read_csv(path)
        if this_header and not header:
            header = list(this_header)
        rows.extend(this_rows)
    return header, rows


def _nearest_row(rows: list[dict], time_field: str, target: float, tolerance_ms: float) -> dict | None:
    best = None
    best_delta = None
    for row in rows:
        value = _to_float(row.get(time_field, ""))
        if value is None:
            continue
        delta_ms = abs((value - target) * 1e3)
        if delta_ms > tolerance_ms:
            continue
        if best_delta is None or delta_ms < best_delta:
            best = row
            best_delta = delta_ms
    return best


def _group_by(rows: list[dict], *fields: str) -> dict[tuple, list[dict]]:
    grouped: dict[tuple, list[dict]] = {}
    for row in rows:
        key = tuple(str(row.get(field, "")).strip() for field in fields)
        grouped.setdefault(key, []).append(row)
    return grouped


def _sorted_numeric(values: list[float]) -> list[float]:
    return sorted(value for value in values if value is not None)


def derive_command_residence(manifest: dict, inputs: dict, report: dict) -> list[dict]:
    _, scheduler_rows = _artifact_rows(inputs["scheduler_files"])
    _, rx_rows = _artifact_rows(inputs["rx_files"])
    _, ts_rows = _artifact_rows(inputs["ts_files"])
    scheduler_rows.sort(key=lambda row: (_to_float(row.get("t_tx", "")) or 0.0, str(row.get("robot_id", ""))))
    rx_by_cmd = _group_by(rx_rows, "run_id", "robot_id", "command_id")
    ts_by_cmd = _group_by(ts_rows, "run_id", "robot_id", "command_id")
    rx_by_seq = _group_by(rx_rows, "run_id", "robot_id", "seq")
    ts_by_seq = _group_by(ts_rows, "run_id", "robot_id", "seq")
    rx_by_robot = _group_by(rx_rows, "run_id", "robot_id")
    ts_by_robot = _group_by(ts_rows, "run_id", "robot_id")

    out_rows: list[dict] = []
    success_count = 0
    unmatched_count = 0
    for row in scheduler_rows:
        run_id = str(row.get("run_id", "")).strip()
        robot_id = str(row.get("robot_id", "")).strip()
        command_id = str(row.get("command_id", "")).strip()
        seq = str(row.get("seq", "")).strip()
        t_tx = _to_float(row.get("t_tx", ""))
        rx_match_rows: list[dict] = []
        ts_match_rows: list[dict] = []
        join_key_type = "unmatched"
        join_quality = "unmatched"

        key_cmd = (run_id, robot_id, command_id)
        key_seq = (run_id, robot_id, seq)
        if command_id and key_cmd in rx_by_cmd:
            rx_match_rows = list(rx_by_cmd.get(key_cmd, []))
            ts_match_rows = list(ts_by_cmd.get(key_cmd, []))
            join_key_type = "run_id+robot_id+command_id"
            join_quality = "exact_command_id" if ts_match_rows or rx_match_rows else "unmatched"
        elif seq and key_seq in rx_by_seq and key_seq in ts_by_seq:
            rx_match_rows = list(rx_by_seq.get(key_seq, []))
            ts_match_rows = list(ts_by_seq.get(key_seq, []))
            join_key_type = "run_id+robot_id+seq"
            join_quality = "exact_seq" if ts_match_rows or rx_match_rows else "unmatched"
        elif t_tx is not None:
            nearest_rx = _nearest_row(rx_by_robot.get((run_id, robot_id), []), "t_rx", t_tx, DEFAULT_JOIN_TOLERANCE_MS)
            nearest_ts = _nearest_row(ts_by_robot.get((run_id, robot_id), []), "t_watchdog", t_tx, DEFAULT_JOIN_TOLERANCE_MS)
            if nearest_rx is not None:
                rx_match_rows = [nearest_rx]
            if nearest_ts is not None:
                ts_match_rows = [nearest_ts]
            if rx_match_rows or ts_match_rows:
                matched_command_id = str((nearest_rx or nearest_ts).get("command_id", "")).strip()
                if matched_command_id:
                    ts_match_rows = ts_by_cmd.get((run_id, robot_id, matched_command_id), ts_match_rows)
                    rx_match_rows = rx_by_cmd.get((run_id, robot_id, matched_command_id), rx_match_rows)
                join_key_type = "run_id+robot_id+nearest_timestamp"
                join_quality = "nearest_timestamp"

        rx_times = _sorted_numeric([_to_float(match.get("t_rx", "")) for match in rx_match_rows])
        ts_times = _sorted_numeric([
            _to_float(match.get("t", "")) if _to_float(match.get("t", "")) is not None else _to_float(match.get("t_watchdog", ""))
            for match in ts_match_rows
        ])
        t_rx = rx_times[0] if rx_times else None
        t_first = ts_times[0] if ts_times else None
        t_last = ts_times[-1] if ts_times else None
        if rx_match_rows or ts_match_rows:
            success_count += 1
        else:
            unmatched_count += 1

        out_rows.append(
            {
                "run_id": run_id,
                "robot_id": robot_id,
                "command_id": command_id or str((rx_match_rows[:1] or ts_match_rows[:1] or [{}])[0].get("command_id", "")).strip(),
                "seq": seq,
                "join_key_type": join_key_type,
                "join_tolerance_ms": _fmt(DEFAULT_JOIN_TOLERANCE_MS),
                "join_quality": join_quality,
                "t_tx_scheduler": _fmt(t_tx),
                "t_rx_watchdog": _fmt(t_rx),
                "t_first_apply_watchdog": _fmt(t_first),
                "t_last_apply_watchdog": _fmt(t_last),
                "net_receive_proxy_ms": _fmt((t_rx - t_tx) * 1e3 if t_rx is not None and t_tx is not None else None),
                "age_at_first_use_ms": _fmt((t_first - t_tx) * 1e3 if t_first is not None and t_tx is not None else None),
                "age_at_last_use_ms": _fmt((t_last - t_tx) * 1e3 if t_last is not None and t_tx is not None else None),
                "residence_apply_window_ms": _fmt((t_last - t_first) * 1e3 if t_last is not None and t_first is not None else None),
                "classification": "proxy",
                "interpretation_note": "scheduler-to-watchdog timing proxy only; not true one-way delay or actuator residence truth",
            }
        )
    report["join_success_counts"]["command_residence"] = success_count
    report["join_failure_counts"]["command_residence"] = unmatched_count
    if not scheduler_rows:
        report["unavailable_fields"].append("command_residence:no_scheduler_events")
    return out_rows


def derive_phase_attributed_scheduler(manifest: dict, inputs: dict, report: dict) -> list[dict]:
    _, scheduler_rows = _artifact_rows(inputs["scheduler_files"])
    _, mission_rows = _artifact_rows(inputs["mission_files"])
    scheduler_rows.sort(key=lambda row: _to_float(row.get("t_tx", "")) or 0.0)
    mission_rows.sort(key=lambda row: _to_float(row.get("timestamp", "")) or 0.0)
    if not mission_rows:
        report["unavailable_fields"].append("phase_attribution:mission_runtime_events_missing")
    out_rows: list[dict] = []
    matched = 0
    for row in scheduler_rows:
        t_tx = _to_float(row.get("t_tx", ""))
        mission_match = _nearest_row(mission_rows, "timestamp", t_tx, DEFAULT_PHASE_TOLERANCE_MS) if t_tx is not None else None
        if mission_match is not None:
            matched += 1
        out_rows.append(
            {
                "run_id": row.get("run_id", ""),
                "robot_id": row.get("robot_id", ""),
                "task_phase": row.get("task_phase", ""),
                "command_id": row.get("command_id", ""),
                "command_type": row.get("command_type", ""),
                "seq": row.get("seq", ""),
                "t_tx": row.get("t_tx", ""),
                "scheduler_decision": row.get("scheduler_decision", row.get("reason", "")),
                "precision_mode": row.get("precision_mode", ""),
                "attributed_phase": mission_match.get("task_phase", "") if mission_match else "",
                "attributed_local_state": mission_match.get("local_state", mission_match.get("mission_state", "")) if mission_match else "",
                "phase_join_quality": "timestamp_nearest" if mission_match else ("unavailable_mission" if not mission_rows else "unmatched"),
                "phase_join_tolerance_ms": _fmt(DEFAULT_PHASE_TOLERANCE_MS),
                "classification": "estimated",
            }
        )
    report["join_success_counts"]["phase_attribution"] = matched
    report["join_failure_counts"]["phase_attribution"] = max(0, len(scheduler_rows) - matched)
    return out_rows


def _interarrival_stats(times: list[float]) -> dict:
    inter = [(times[idx] - times[idx - 1]) * 1e3 for idx in range(1, len(times))]
    inter = [value for value in inter if value >= 0.0]
    return {
        "sample_count": len(times),
        "mean_interarrival_ms": _fmt(_safe_mean(inter)),
        "median_interarrival_ms": _fmt(_safe_median(inter)),
        "p95_interarrival_ms": _fmt(_quantile(inter, 0.95)),
        "max_interarrival_ms": _fmt(max(inter) if inter else None),
        "gap_count_over_threshold": str(sum(1 for value in inter if value > CONTROL_LOOP_GAP_THRESHOLD_MS)),
        "burst_proxy_mean_interarrival_ms": _safe_mean(inter),
        "burst_proxy_min_interarrival_ms": min(inter) if inter else None,
        "burst_proxy_short_gap_count": sum(1 for value in inter if value < BURST_SHORT_GAP_MS),
    }


def derive_control_loop_timing(manifest: dict, capture_map: dict[str, Path], report: dict) -> tuple[list[dict], dict]:
    out_rows: list[dict] = []
    burst_summary: dict[tuple[str, str, str, str], dict] = {}
    for artifact_name in ("chassis_command_samples.csv", "slide_command_samples.csv"):
        _, rows = _read_csv(capture_map.get(artifact_name, Path(artifact_name)))
        grouped: dict[tuple[str, str, str, str], list[dict]] = {}
        for row in rows:
            key = (
                str(row.get("stream_name", "")).strip(),
                str(row.get("robot_id", "")).strip(),
                str(row.get("slide_id", "")).strip(),
                str(row.get("topic", "")).strip(),
            )
            grouped.setdefault(key, []).append(row)
        for key, group in grouped.items():
            times = _sorted_numeric([_to_float(item.get("t_receive_ros", "")) for item in group])
            stats = _interarrival_stats(times)
            burst_summary[key] = stats
            out_rows.append(
                {
                    "run_id": manifest.get("run_id", ""),
                    "stream_name": key[0],
                    "robot_id": key[1],
                    "slide_id": key[2],
                    "topic": key[3],
                    "sample_count": str(stats["sample_count"]),
                    "mean_interarrival_ms": stats["mean_interarrival_ms"],
                    "median_interarrival_ms": stats["median_interarrival_ms"],
                    "p95_interarrival_ms": stats["p95_interarrival_ms"],
                    "max_interarrival_ms": stats["max_interarrival_ms"],
                    "gap_count_over_threshold": stats["gap_count_over_threshold"],
                    "classification": "proxy",
                    "interpretation_note": "inter-arrival timing proxy, not controller internal loop truth",
                }
            )
    if not out_rows:
        report["unavailable_fields"].append("control_loop_timing:no_command_capture_rows")
    return out_rows, burst_summary


def _column_stats(rows: list[dict], field: str) -> tuple[str, str, str]:
    values = _sorted_numeric([_to_float(row.get(field, "")) for row in rows])
    return _fmt(_safe_mean(values)), _fmt(_quantile(values, 0.95)), _fmt(max(values) if values else None)


def derive_executor_backlog(manifest: dict, capture_map: dict[str, Path], ts_rows: list[dict], burst_summary: dict, report: dict) -> list[dict]:
    _, callback_rows = _read_csv(capture_map.get("recorder_callback_timing.csv", Path("recorder_callback_timing.csv")))
    grouped = _group_by(callback_rows, "stream_name", "topic")
    out_rows: list[dict] = []
    queue_mean, queue_p95, queue_max = _column_stats(ts_rows, "queue_delay_proxy_ms")
    exec_mean, exec_p95, exec_max = _column_stats(ts_rows, "delta_exec_proxy_ms")

    for key, rows in grouped.items():
        durations = _sorted_numeric([_to_float(row.get("duration_ms", "")) for row in rows])
        burst = burst_summary.get((key[0], "", "", key[1]), {})
        if not burst:
            for candidate_key, candidate_value in burst_summary.items():
                if candidate_key[0] == key[0] or candidate_key[3] == key[1]:
                    burst = candidate_value
                    break
        out_rows.append(
            {
                "run_id": manifest.get("run_id", ""),
                "stream_name": key[0],
                "proxy_source": "recorder_callback_timing+watchdog+command_interarrival",
                "mean_callback_duration_ms": _fmt(_safe_mean(durations)),
                "p95_callback_duration_ms": _fmt(_quantile(durations, 0.95)),
                "max_callback_duration_ms": _fmt(max(durations) if durations else None),
                "watchdog_queue_delay_proxy_ms_mean": queue_mean,
                "watchdog_queue_delay_proxy_ms_p95": queue_p95,
                "watchdog_queue_delay_proxy_ms_max": queue_max,
                "delta_exec_proxy_ms_mean": exec_mean,
                "delta_exec_proxy_ms_p95": exec_p95,
                "delta_exec_proxy_ms_max": exec_max,
                "burst_proxy_mean_interarrival_ms": _fmt(burst.get("burst_proxy_mean_interarrival_ms")),
                "burst_proxy_min_interarrival_ms": _fmt(burst.get("burst_proxy_min_interarrival_ms")),
                "burst_proxy_short_gap_count": str(burst.get("burst_proxy_short_gap_count", 0)),
                "classification": "proxy",
                "interpretation_note": "not internal executor queue-depth truth",
            }
        )
    if not callback_rows:
        report["unavailable_fields"].append("executor_backlog:no_recorder_callback_timing")
    report["unavailable_internal_executor_queue_depth"] = True
    return out_rows


def _load_authority_limits(manifest: dict) -> tuple[dict, str]:
    limits = manifest.get("authority_proxy_limits", {})
    if not isinstance(limits, dict) or not limits:
        return {}, "authority limits unavailable from manifest/profile/config"
    required = ("v_max", "w_max", "vx_lim", "vy_lim", "vz_lim")
    if any(_to_float(limits.get(name)) is None for name in required):
        return {}, "authority limits incomplete in manifest/profile/config"
    return {name: float(limits[name]) for name in required}, ""


def _nearest_by_time(rows: list[dict], time_field: str, target: float, tolerance_ms: float) -> dict | None:
    return _nearest_row(rows, time_field, target, tolerance_ms)


def derive_authority_proxy(manifest: dict, capture_map: dict[str, Path], mission_rows: list[dict], report: dict) -> list[dict]:
    _, chassis_rows = _read_csv(capture_map.get("chassis_command_samples.csv", Path("chassis_command_samples.csv")))
    _, slide_rows = _read_csv(capture_map.get("slide_command_samples.csv", Path("slide_command_samples.csv")))
    limits, unavailable_reason = _load_authority_limits(manifest)
    out_rows: list[dict] = []
    slide_sorted = sorted(slide_rows, key=lambda row: _to_float(row.get("t_receive_ros", "")) or 0.0)
    mission_sorted = sorted(mission_rows, key=lambda row: _to_float(row.get("timestamp", "")) or 0.0)

    for chassis in chassis_rows:
        timestamp = _to_float(chassis.get("t_receive_ros", "")) or _to_float(chassis.get("t_receive_wall", ""))
        nearest_slide = _nearest_by_time(slide_sorted, "t_receive_ros", timestamp, DEFAULT_JOIN_TOLERANCE_MS) if timestamp is not None else None
        nearest_mission = _nearest_by_time(mission_sorted, "timestamp", timestamp, DEFAULT_PHASE_TOLERANCE_MS) if timestamp is not None else None
        base_internal = nearest_mission.get("base_authority_weight", "") if nearest_mission else ""
        slide_internal = nearest_mission.get("slide_authority_weight", "") if nearest_mission else ""

        reason = unavailable_reason
        chassis_ratio = ""
        slide_ratio = ""
        if nearest_slide is None:
            reason = reason or "nearest slide command unavailable"
        elif limits:
            linear_x = _to_float(chassis.get("linear_x", "")) or 0.0
            angular_z = _to_float(chassis.get("angular_z", "")) or 0.0
            vx = _to_float(nearest_slide.get("vx", ""))
            vy = _to_float(nearest_slide.get("vy", ""))
            vz = _to_float(nearest_slide.get("vz", ""))
            if vx is None and vy is None and vz is None:
                reason = "slide command is not velocity-like"
            else:
                base_norm = math.sqrt((linear_x / limits["v_max"]) ** 2 + (angular_z / limits["w_max"]) ** 2)
                slide_norm = max(
                    abs(vx or 0.0) / limits["vx_lim"],
                    abs(vy or 0.0) / limits["vy_lim"],
                    abs(vz or 0.0) / limits["vz_lim"],
                )
                denom = base_norm + slide_norm
                if denom > 0.0:
                    chassis_ratio = _fmt(base_norm / denom)
                    slide_ratio = _fmt(slide_norm / denom)
                    reason = ""
                else:
                    reason = "zero command magnitudes"

        out_rows.append(
            {
                "run_id": manifest.get("run_id", ""),
                "timestamp": _fmt(timestamp),
                "robot_id": chassis.get("robot_id", ""),
                "slide_id": nearest_slide.get("slide_id", "") if nearest_slide else "",
                "chassis_topic": chassis.get("topic", ""),
                "slide_topic": nearest_slide.get("topic", "") if nearest_slide else "",
                "base_authority_weight_internal_proxy": base_internal,
                "slide_authority_weight_internal_proxy": slide_internal,
                "authority_proxy_chassis_ratio": chassis_ratio,
                "authority_proxy_slide_ratio": slide_ratio,
                "classification": "proxy",
                "interpretation_note": "command-magnitude authority proxy only; not physical authority or true allocation alpha",
                "unavailable_reason": reason,
            }
        )
    if not out_rows:
        report["unavailable_fields"].append("authority_proxy:no_chassis_command_rows")
    elif unavailable_reason:
        report["unavailable_fields"].append("authority_proxy:limits_unavailable")
    return out_rows


def _residual_values_from_mission(mission_rows: list[dict]) -> tuple[list[float], list[float]]:
    dock = _sorted_numeric([_to_float(row.get("docking_residual_proxy", "")) for row in mission_rows])
    support = _sorted_numeric([_to_float(row.get("support_residual_proxy", "")) for row in mission_rows])
    return dock, support


def _residual_values_from_delta(delta_rows: list[dict]) -> list[float]:
    tail = delta_rows[-10:]
    values = []
    for row in tail:
        x = _to_float(row.get("x", ""))
        y = _to_float(row.get("y", ""))
        z = _to_float(row.get("z", ""))
        if x is None or y is None or z is None:
            continue
        values.append(math.sqrt(x * x + y * y + z * z))
    return _sorted_numeric(values)


def derive_terminal_residual(manifest: dict, capture_map: dict[str, Path], mission_rows: list[dict], report: dict) -> list[dict]:
    _, delta_rows = _read_csv(capture_map.get("delta_samples.csv", Path("delta_samples.csv")))
    mission_dock, mission_support = _residual_values_from_mission(mission_rows)
    values = mission_dock
    source_used = "mission_runtime_events.csv"
    support_value = _fmt(_safe_mean(mission_support))
    if not values:
        values = _residual_values_from_delta(delta_rows)
        source_used = "delta_samples.csv:last_window"
        support_value = ""
    if not values:
        report["unavailable_fields"].append("terminal_residual:no_mission_residual_or_delta_rows")
        return []
    return [
        {
            "run_id": manifest.get("run_id", ""),
            "source_used": source_used,
            "terminal_residual_proxy_mean": _fmt(_safe_mean(values)),
            "terminal_residual_proxy_median": _fmt(_safe_median(values)),
            "terminal_residual_proxy_max": _fmt(max(values) if values else None),
            "support_residual_proxy": support_value,
            "classification": "proxy",
            "interpretation_note": "not physical docking error",
        }
    ]


def _clock_sync_policy_result(capture_map: dict[str, Path]) -> dict:
    header, rows = _read_csv(capture_map.get("clock_sync_status.csv", Path("clock_sync_status.csv")))
    sync_verified_any = any(_truthy(row.get("sync_verified")) for row in rows) if header else False
    one_way_allowed_any = any(_truthy(row.get("one_way_delay_allowed")) for row in rows) if header else False
    return {
        "clock_sync_artifact_present": bool(header),
        "sync_verified_any": sync_verified_any,
        "one_way_delay_allowed_any": one_way_allowed_any,
        "one_way_delay_reported": False,
    }


def _input_artifact_summary(inputs: dict, capture_map: dict[str, Path]) -> dict:
    summary = {}
    for name, path in capture_map.items():
        _, rows = _read_csv(path)
        summary[name] = {"path": str(path), "exists": path.exists(), "row_count": len(rows)}
    for label, files in (
        ("scheduler_files", inputs["scheduler_files"]),
        ("rx_files", inputs["rx_files"]),
        ("ts_files", inputs["ts_files"]),
        ("mission_files", inputs["mission_files"]),
    ):
        summary[label] = [{"path": str(path), "exists": path.exists(), "row_count": len(_read_csv(path)[1])} for path in files]
    return summary


def _output_artifact_summary(output_paths: dict[str, Path]) -> dict:
    summary = {}
    for name, path in output_paths.items():
        _, rows = _read_csv(path)
        summary[name] = {"path": str(path), "exists": path.exists(), "row_count": len(rows)}
    return summary


def derive_artifacts(
    manifest_path: str,
    capture_dir: str = "",
    scheduler_log_dir: str = "",
    watchdog_log_dir: str = "",
    mission_log_dir: str = "",
    out_dir: str = "",
    run_id: str = "",
) -> dict:
    manifest_file = Path(manifest_path).expanduser()
    if not manifest_file.exists():
        raise FileNotFoundError(f"manifest not found: {manifest_file}")
    manifest = _read_json(manifest_file)
    if manifest.get("evidence_class") != "hardware_preliminary":
        raise ValueError("derive_hardware_preliminary_artifacts.py requires evidence_class=hardware_preliminary")
    manifest_run_id = str(manifest.get("run_id", "")).strip()
    requested_run_id = str(run_id).strip()
    if requested_run_id and requested_run_id != manifest_run_id:
        raise ValueError("provided --run-id does not match manifest run_id")

    capture_dir_path = Path(capture_dir).expanduser() if capture_dir else _capture_dir_from_manifest(manifest)
    scheduler_dir_path = Path(scheduler_log_dir).expanduser() if scheduler_log_dir else _log_dir_from_manifest(manifest, "scheduler")
    watchdog_dir_path = Path(watchdog_log_dir).expanduser() if watchdog_log_dir else _log_dir_from_manifest(manifest, "watchdog")
    mission_dir_path = Path(mission_log_dir).expanduser() if mission_log_dir else _log_dir_from_manifest(manifest, "mission")
    out_dir_path = Path(out_dir).expanduser() if out_dir else _derived_dir_from_manifest(manifest)
    out_dir_path.mkdir(parents=True, exist_ok=True)

    inputs = _load_inputs(manifest, capture_dir_path, scheduler_dir_path, watchdog_dir_path, mission_dir_path)
    capture_map = _artifact_map(manifest, "capture_artifacts")
    if not capture_map:
        capture_map = {
            name: capture_dir_path / name for name in (
                "delta_samples.csv",
                "chassis_command_samples.csv",
                "slide_command_samples.csv",
                "recorder_callback_timing.csv",
                "clock_sync_status.csv",
            )
        }

    report = {
        "run_id": manifest_run_id,
        "manifest_path": str(manifest_file),
        "manifest_sha256": manifest.get("manifest_sha256", manifest_script._canonical_manifest_sha256(manifest)),
        "evidence_class": "hardware_preliminary",
        "input_artifacts": _input_artifact_summary(inputs, capture_map),
        "output_artifacts": {},
        "row_counts": {},
        "join_success_counts": {},
        "join_failure_counts": {},
        "unavailable_fields": [],
        "clock_sync_policy_result": _clock_sync_policy_result(capture_map),
        "one_way_delay_reported": False,
        "warnings": [],
        "failures": [],
        "derived_artifact_boundary": {
            "all_residence_fields_are_proxy": True,
            "callback_timing_is_passive_recorder_proxy": True,
            "executor_backlog_is_proxy": True,
            "authority_ratios_are_proxy": True,
            "terminal_residual_is_proxy": True,
            "bandwidth_throughput_are_context_only": True,
        },
    }

    _, mission_rows = _artifact_rows(inputs["mission_files"])
    _, ts_rows = _artifact_rows(inputs["ts_files"])
    command_residence_rows = derive_command_residence(manifest, inputs, report)
    phase_rows = derive_phase_attributed_scheduler(manifest, inputs, report)
    control_loop_rows, burst_summary = derive_control_loop_timing(manifest, capture_map, report)
    executor_rows = derive_executor_backlog(manifest, capture_map, ts_rows, burst_summary, report)
    authority_rows = derive_authority_proxy(manifest, capture_map, mission_rows, report)
    terminal_rows = derive_terminal_residual(manifest, capture_map, mission_rows, report)

    output_paths = {
        "command_residence_events.csv": out_dir_path / "command_residence_events.csv",
        "control_loop_timing.csv": out_dir_path / "control_loop_timing.csv",
        "executor_backlog_proxy.csv": out_dir_path / "executor_backlog_proxy.csv",
        "authority_proxy_timeseries.csv": out_dir_path / "authority_proxy_timeseries.csv",
        "terminal_residual_proxy.csv": out_dir_path / "terminal_residual_proxy.csv",
        "phase_attributed_scheduler_events.csv": out_dir_path / "phase_attributed_scheduler_events.csv",
        "derivation_report.json": out_dir_path / "derivation_report.json",
    }
    _write_csv(output_paths["command_residence_events.csv"], COMMAND_RESIDENCE_FIELDS, command_residence_rows)
    _write_csv(output_paths["control_loop_timing.csv"], CONTROL_LOOP_TIMING_FIELDS, control_loop_rows)
    _write_csv(output_paths["executor_backlog_proxy.csv"], EXECUTOR_BACKLOG_FIELDS, executor_rows)
    _write_csv(output_paths["authority_proxy_timeseries.csv"], AUTHORITY_PROXY_FIELDS, authority_rows)
    _write_csv(output_paths["terminal_residual_proxy.csv"], TERMINAL_RESIDUAL_FIELDS, terminal_rows)
    _write_csv(output_paths["phase_attributed_scheduler_events.csv"], PHASE_ATTRIBUTED_FIELDS, phase_rows)

    report["row_counts"] = {
        "command_residence_events.csv": len(command_residence_rows),
        "control_loop_timing.csv": len(control_loop_rows),
        "executor_backlog_proxy.csv": len(executor_rows),
        "authority_proxy_timeseries.csv": len(authority_rows),
        "terminal_residual_proxy.csv": len(terminal_rows),
        "phase_attributed_scheduler_events.csv": len(phase_rows),
    }
    report_path = output_paths["derivation_report.json"]
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report["output_artifacts"] = _output_artifact_summary(output_paths)
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Derive bounded hardware_preliminary proxy artifacts.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--capture-dir", default="")
    parser.add_argument("--scheduler-log-dir", default="")
    parser.add_argument("--watchdog-log-dir", default="")
    parser.add_argument("--mission-log-dir", default="")
    parser.add_argument("--out-dir", default="")
    parser.add_argument("--run-id", default="")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    report = derive_artifacts(
        manifest_path=args.manifest,
        capture_dir=args.capture_dir,
        scheduler_log_dir=args.scheduler_log_dir,
        watchdog_log_dir=args.watchdog_log_dir,
        mission_log_dir=args.mission_log_dir,
        out_dir=args.out_dir,
        run_id=args.run_id,
    )
    print(json.dumps({
        "run_id": report["run_id"],
        "failure_count": len(report["failures"]),
        "warning_count": len(report["warnings"]),
        "one_way_delay_reported": report["one_way_delay_reported"],
        "row_counts": report["row_counts"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
