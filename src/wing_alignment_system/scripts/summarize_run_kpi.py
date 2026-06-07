#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
from pathlib import Path


FULL_UPDATE_BASELINE_BYTES = 256.0
EXECUTION_MODES = ("normal", "degraded", "hold", "safe_stop")
PHASE_ORDER = ("approach", "slide_align", "level_recenter", "transport", "abort", "standby")


def _safe_float(value, default: float = 0.0) -> float:
    text = str(value or "").strip()
    if not text or text.lower() == "n/a":
        return default
    try:
        return float(text)
    except ValueError:
        return default


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _find_csvs(log_root: Path, pattern: str) -> list[Path]:
    return sorted(path for path in log_root.rglob(pattern) if path.is_file())


def _normalize_phase(value: object) -> str:
    phase = str(value or "").strip().lower()
    return phase or "standby"


def _phase_sort_key(phase: str) -> tuple[int, str]:
    return (PHASE_ORDER.index(phase), phase) if phase in PHASE_ORDER else (len(PHASE_ORDER), phase)


def _seq_gap_count(rows: list[dict[str, str]], *, robot_key: str, seq_key: str) -> int:
    total_gap = 0
    by_robot: dict[str, list[int]] = {}
    for row in rows:
        robot = str(row.get(robot_key, "") or "fleet")
        seq = int(_safe_float(row.get(seq_key), 0.0))
        if seq <= 0:
            continue
        by_robot.setdefault(robot, []).append(seq)
    for seqs in by_robot.values():
        seqs = sorted(set(seqs))
        for prev, cur in zip(seqs, seqs[1:]):
            if cur > prev + 1:
                total_gap += cur - prev - 1
    return total_gap


def _dedupe_wrapper_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    deduped: list[dict[str, str]] = []
    seen: set[tuple[str, int]] = set()
    for row in rows:
        robot = str(row.get("robot_id", "") or "fleet")
        seq = int(_safe_float(row.get("seq_id"), 0.0))
        if seq <= 0:
            continue
        key = (robot, seq)
        if key in seen:
            continue
        seen.add(key)
        deduped.append(row)
    return deduped


def _aoi_from_wrapper_row(row: dict[str, str]) -> float:
    direct = _safe_float(row.get("aoi_ms"), -1.0)
    if direct >= 0.0:
        return direct
    return _safe_float(row.get("receiver_side_aoi_proxy_ms"), -1.0)


def _phase_metrics(wrapper_rows: list[dict[str, str]], mission_rows: list[dict[str, str]]) -> dict[str, object]:
    phase_rows: dict[str, list[dict[str, str]]] = {}
    for row in wrapper_rows:
        phase_rows.setdefault(_normalize_phase(row.get("phase")), []).append(row)
    for row in mission_rows:
        phase = _normalize_phase(row.get("task_phase"))
        phase_rows.setdefault(phase, [])

    summary: dict[str, object] = {}
    for phase in sorted(phase_rows.keys(), key=_phase_sort_key):
        rows = phase_rows.get(phase, [])
        tx_count = len(rows)
        payload_bytes = sum(int(_safe_float(row.get("payload_bytes"), 0.0)) for row in rows)
        aoi_values = [_aoi_from_wrapper_row(row) for row in rows]
        aoi_values = [value for value in aoi_values if value >= 0.0]
        summary[f"phase_{phase}_tx_count"] = tx_count
        summary[f"phase_{phase}_payload_bytes"] = payload_bytes
        summary[f"phase_{phase}_avg_AoI_ms"] = round((sum(aoi_values) / len(aoi_values)) if aoi_values else 0.0, 6)
        summary[f"phase_{phase}_max_AoI_ms"] = round(max(aoi_values) if aoi_values else 0.0, 6)
        for execution_mode in EXECUTION_MODES:
            count = sum(1 for row in rows if str(row.get("execution_mode", "") or "normal") == execution_mode)
            summary[f"phase_{phase}_execution_mode_ratio_{execution_mode}"] = round((count / tx_count) if tx_count else 0.0, 6)
        baseline_total = tx_count * FULL_UPDATE_BASELINE_BYTES
        summary[f"phase_{phase}_communication_saving_ratio"] = round(
            max(0.0, 1.0 - (payload_bytes / baseline_total)) if baseline_total > 0.0 else 0.0,
            6,
        )
    return summary


def _count_mode_state_entries(rows: list[dict[str, str]], *, states: set[str], reasons: set[str]) -> int:
    count = 0
    active_by_robot: dict[str, bool] = {}
    ordered = sorted(rows, key=lambda row: (_safe_float(row.get("timestamp"), 0.0), str(row.get("robot_id", ""))))
    for row in ordered:
        robot = str(row.get("robot_id", "") or "fleet")
        active = (
            str(row.get("watchdog_state", "") or "") in states
            or str(row.get("stop_reason", "") or "") in reasons
        )
        if active and not active_by_robot.get(robot, False):
            count += 1
        active_by_robot[robot] = active
    return count


def _last_valid_residual(mission_rows: list[dict[str, str]], key: str) -> float:
    for row in reversed(mission_rows):
        value = _safe_float(row.get(key), -1.0)
        if value >= 0.0:
            return value
    return 0.0


def summarize_run(log_root: Path) -> dict[str, object]:
    mission_rows: list[dict[str, str]] = []
    for path in _find_csvs(log_root, "mission_runtime_events.csv"):
        mission_rows.extend(_read_csv_rows(path))

    wrapper_rows: list[dict[str, str]] = []
    for path in _find_csvs(log_root, "*cmd_channel_meta.csv"):
        wrapper_rows.extend(_read_csv_rows(path))
    wrapper_rows = _dedupe_wrapper_rows(wrapper_rows)

    mode_rows: list[dict[str, str]] = []
    for path in _find_csvs(log_root, "mode_timeline_*.csv"):
        mode_rows.extend(_read_csv_rows(path))

    mission_timestamps = [_safe_float(row.get("timestamp"), -1.0) for row in mission_rows]
    mission_timestamps = [value for value in mission_timestamps if value >= 0.0]
    if mission_timestamps:
        makespan_s = max(mission_timestamps) - min(mission_timestamps)
    else:
        mode_timestamps = [_safe_float(row.get("timestamp"), -1.0) for row in mode_rows]
        mode_timestamps = [value for value in mode_timestamps if value >= 0.0]
        makespan_s = (max(mode_timestamps) - min(mode_timestamps)) if mode_timestamps else 0.0

    mission_states = [str(row.get("mission_state", "") or "").strip().upper() for row in mission_rows]
    mission_success = int("DONE" in mission_states)
    if mission_success:
        outcome = "success"
    elif "ABORT" in mission_states:
        outcome = "abort"
    else:
        outcome = "unknown"

    tx_count = len(wrapper_rows)
    payload_bytes_total = sum(int(_safe_float(row.get("payload_bytes"), 0.0)) for row in wrapper_rows)
    retry_count = sum(int(_safe_float(row.get("retry_count"), 0.0)) for row in wrapper_rows)
    packet_loss_proxy = _seq_gap_count(wrapper_rows, robot_key="robot_id", seq_key="seq_id")

    aoi_values = [_safe_float(row.get("aoi_ms"), -1.0) for row in wrapper_rows]
    aoi_values = [value for value in aoi_values if value >= 0.0]
    if not aoi_values:
        aoi_values = [_safe_float(row.get("AoI_ms"), -1.0) for row in mode_rows]
        aoi_values = [value for value in aoi_values if value >= 0.0]
    avg_aoi_ms = sum(aoi_values) / len(aoi_values) if aoi_values else 0.0
    max_aoi_ms = max(aoi_values) if aoi_values else 0.0

    mode_count = max(len(mode_rows), 1)
    degraded_count = sum(1 for row in mode_rows if str(row.get("execution_mode", "")) == "degraded")
    hold_count = sum(1 for row in mode_rows if str(row.get("execution_mode", "")) == "hold")
    safe_stop_count = len(
        {
            (str(row.get("robot_id", "") or "fleet"), int(_safe_float(row.get("seq"), 0.0)))
            for row in mode_rows
            if (
                str(row.get("execution_mode", "")) == "safe_stop"
                or str(row.get("watchdog_state", "")) == "MODE_SAFE_STOP"
                or str(row.get("stop_reason", "")) == "execution_mode_safe_stop"
            )
            and int(_safe_float(row.get("seq"), 0.0)) > 0
        }
    )
    degraded_time_ratio = degraded_count / mode_count if mode_rows else 0.0
    hold_time_ratio = hold_count / mode_count if mode_rows else 0.0

    communication_saving_ratio = 0.0
    if tx_count > 0:
        baseline_total = tx_count * FULL_UPDATE_BASELINE_BYTES
        communication_saving_ratio = max(0.0, 1.0 - (payload_bytes_total / baseline_total))

    safety_override_count = len(
        {
            (str(row.get("robot_id", "") or "fleet"), int(_safe_float(row.get("seq"), 0.0)))
            for row in mode_rows
            if str(row.get("execution_mode", "") or "normal") in {"degraded", "hold", "safe_stop"}
            and int(_safe_float(row.get("seq"), 0.0)) > 0
        }
    )
    if safety_override_count == 0:
        safety_override_count = len(
            {
                (str(row.get("robot_id", "") or "fleet"), int(_safe_float(row.get("seq_id"), 0.0)))
                for row in wrapper_rows
                if str(row.get("execution_mode", "") or "normal") in {"degraded", "hold", "safe_stop"}
                and int(_safe_float(row.get("seq_id"), 0.0)) > 0
            }
        )
    cmd_stop_count = _count_mode_state_entries(
        mode_rows,
        states={"CMD_STOP"},
        reasons={"cmd_stop_latched"},
    )
    emergency_stop_count = _count_mode_state_entries(
        mode_rows,
        states={"EMERGENCY_STOP"},
        reasons={"emergency_latched"},
    )
    age_stop_count = _count_mode_state_entries(
        mode_rows,
        states={"AGE_STOP"},
        reasons={"age_stop_exceeded"},
    )

    control_error_values = []
    for row in mission_rows:
        candidates = [
            _safe_float(row.get("docking_residual_proxy"), -1.0),
            _safe_float(row.get("slide_residual_proxy"), -1.0),
            _safe_float(row.get("support_residual_proxy"), -1.0),
        ]
        valid = [value for value in candidates if value >= 0.0]
        if valid:
            control_error_values.append(max(valid))
    control_error_proxy_avg = sum(control_error_values) / len(control_error_values) if control_error_values else 0.0
    control_error_proxy_max = max(control_error_values) if control_error_values else 0.0
    final_docking_residual_proxy = _last_valid_residual(mission_rows, "docking_residual_proxy")
    final_slide_residual_proxy = _last_valid_residual(mission_rows, "slide_residual_proxy")
    final_support_residual_proxy = _last_valid_residual(mission_rows, "support_residual_proxy")
    final_control_residual_proxy = max(
        final_docking_residual_proxy,
        final_slide_residual_proxy,
        final_support_residual_proxy,
    )

    cmd_vel_in_values = [_safe_float(row.get("cmd_v_in"), 0.0) for row in mode_rows]
    cmd_vel_out_values = [_safe_float(row.get("cmd_v_out"), 0.0) for row in mode_rows]
    output_scale_values = [_safe_float(row.get("output_scale"), 1.0) for row in mode_rows]
    cmd_vel_in_mean = sum(cmd_vel_in_values) / len(cmd_vel_in_values) if cmd_vel_in_values else 0.0
    cmd_vel_out_mean = sum(cmd_vel_out_values) / len(cmd_vel_out_values) if cmd_vel_out_values else 0.0
    output_scale_mean = sum(output_scale_values) / len(output_scale_values) if output_scale_values else 1.0
    stop_reason_dist: dict[str, int] = {}
    for row in mode_rows:
        sr = str(row.get("stop_reason", "") or "")
        if sr:
            stop_reason_dist[sr] = stop_reason_dist.get(sr, 0) + 1

    summary = {
        "mission_success": mission_success,
        "outcome": outcome,
        "makespan_s": round(makespan_s, 6),
        "tx_count": tx_count,
        "payload_bytes_total": payload_bytes_total,
        "retry_count": retry_count,
        "packet_loss_proxy": packet_loss_proxy,
        "avg_AoI_ms": round(avg_aoi_ms, 6),
        "max_AoI_ms": round(max_aoi_ms, 6),
        "degraded_time_ratio": round(degraded_time_ratio, 6),
        "hold_time_ratio": round(hold_time_ratio, 6),
        "safe_stop_count": safe_stop_count,
        "safety_override_count": safety_override_count,
        "cmd_stop_count": cmd_stop_count,
        "emergency_stop_count": emergency_stop_count,
        "communication_saving_ratio": round(communication_saving_ratio, 6),
        "control_error_proxy_avg": round(control_error_proxy_avg, 6),
        "control_error_proxy_max": round(control_error_proxy_max, 6),
        "final_docking_residual_proxy": round(final_docking_residual_proxy, 6),
        "final_slide_residual_proxy": round(final_slide_residual_proxy, 6),
        "final_support_residual_proxy": round(final_support_residual_proxy, 6),
        "final_control_residual_proxy": round(final_control_residual_proxy, 6),
        "cmd_vel_in_mean": round(cmd_vel_in_mean, 6),
        "cmd_vel_out_mean": round(cmd_vel_out_mean, 6),
        "output_scale_mean": round(output_scale_mean, 6),
        "degraded_count": degraded_count,
        "hold_count": hold_count,
        "age_stop_count": age_stop_count,
        "stop_reason_distribution": str(stop_reason_dist),
    }
    summary.update(_phase_metrics(wrapper_rows, mission_rows))
    return summary


def write_summary_csv(output_path: Path, summary: dict[str, object]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(summary.keys())
    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(summary)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize FR-TAC-P1/P2 run KPIs from existing log artifacts.")
    parser.add_argument("log_root", help="Root log directory to scan recursively")
    parser.add_argument("--output", default="", help="Optional output CSV path; defaults to <log_root>/run_summary.csv")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    log_root = Path(args.log_root).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve() if str(args.output).strip() else log_root / "run_summary.csv"
    summary = summarize_run(log_root)
    write_summary_csv(output_path, summary)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
