#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from staleness_risk import (  # noqa: E402
    STALENESS_RISK_FIELDS,
    STALENESS_RISK_INPUT_FIELDS,
    compute_staleness_risk,
    format_staleness_risk_row,
)


def _warn(warnings: list[str], message: str) -> None:
    warnings.append(message)


def _read_csv(path: Path, label: str, warnings: list[str]) -> list[dict]:
    if not path.exists():
        _warn(warnings, f"{label} CSV not found: {path}")
        return []
    try:
        with path.open("r", encoding="utf-8", newline="") as fp:
            reader = csv.DictReader(fp)
            if not reader.fieldnames:
                _warn(warnings, f"{label} CSV has no header: {path}")
                return []
            return list(reader)
    except Exception as exc:
        _warn(warnings, f"failed to read {label} CSV {path}: {exc}")
        return []


def _find_csvs(run_dir: Path, explicit: str, patterns: list[str], label: str, warnings: list[str]) -> list[Path]:
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_absolute():
            candidate = run_dir / path
            if candidate.exists():
                path = candidate
        return [path]
    if not run_dir.exists():
        _warn(warnings, f"run-dir does not exist: {run_dir}")
        return []
    found: list[Path] = []
    for pattern in patterns:
        found.extend(sorted(run_dir.rglob(pattern)))
    if not found:
        _warn(warnings, f"no {label} CSV found under {run_dir}")
    return found


def _as_float(row: dict, key: str):
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(row: dict, key: str) -> int:
    value = row.get(key, "")
    if value in ("", None):
        return 0
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return 0


def _percentile(values: list[float], q: float):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(ordered) - 1)
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _stats(values: list[float]) -> dict:
    clean = [float(v) for v in values if v is not None]
    if not clean:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p95": None,
            "p99": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(clean),
        "mean": mean(clean),
        "p50": _percentile(clean, 0.50),
        "p95": _percentile(clean, 0.95),
        "p99": _percentile(clean, 0.99),
        "min": min(clean),
        "max": max(clean),
    }


def _robot_id(row: dict) -> str:
    return (row.get("robot_id") or row.get("robot") or "").strip()


def _command_id(row: dict) -> str:
    value = row.get("command_id")
    if value in ("", None):
        value = row.get("seq")
    return str(value).strip() if value not in ("", None) else ""


def _build_scheduler_index(rows: list[dict]) -> dict[tuple[str, str], dict]:
    index: dict[tuple[str, str], dict] = {}
    for row in rows:
        key = (_robot_id(row), _command_id(row))
        if key[0] and key[1]:
            index[key] = row
    return index


def _merge_watchdog_with_scheduler(watchdog_rows: list[dict], scheduler_rows: list[dict]) -> list[dict]:
    scheduler_index = _build_scheduler_index(scheduler_rows)
    merged: list[dict] = []
    for row in watchdog_rows:
        out = dict(row)
        sched = scheduler_index.get((_robot_id(out), _command_id(out)))
        if sched:
            for key in (
                "task_phase",
                "phase",
                "precision_mode",
                "scheduler_decision",
                "communication_profile_mode",
                "link_profile",
                "profile_source",
                "profile",
                "baseline_mode",
                "scheduler_mode",
                "watchdog_mode",
                "baseline_execution_environment",
                "allow_unsafe_baseline",
                "voi",
                "voi_score",
                "VoI",
                "risk_score",
                "risk_proxy",
                "phase_risk",
                "execution_residual",
                "communication_uncertainty",
                "action_correction_uncertainty",
            ):
                if not out.get(key) and sched.get(key):
                    out[key] = sched.get(key)
        merged.append(out)
    return merged


def _age_ms_values(rows: list[dict], warnings: list[str]) -> list[float]:
    values = []
    saw_age_ms = False
    saw_fallback = False
    for row in rows:
        value = _as_float(row, "age_ms")
        if value is not None:
            saw_age_ms = True
            values.append(value)
            continue
        for key in ("age", "age_est"):
            raw = _as_float(row, key)
            if raw is not None:
                saw_fallback = True
                values.append(raw * 1000.0)
                break
    if rows and not saw_age_ms and saw_fallback:
        _warn(warnings, "age_ms missing; used age/age_est seconds as fallback")
    elif rows and not saw_age_ms:
        _warn(warnings, "age_ms missing and no age/age_est fallback found")
    return values


def _metric_values(rows: list[dict], key: str, warnings: list[str]) -> list[float]:
    values = []
    saw_field = False
    for row in rows:
        value = _as_float(row, key)
        if value is not None:
            saw_field = True
            values.append(value)
    if rows and not saw_field:
        _warn(warnings, f"{key} missing or empty in selected rows")
    return values


def _distribution(rows: list[dict], keys: tuple[str, ...]) -> dict:
    counter = Counter()
    for row in rows:
        value = ""
        for key in keys:
            value = str(row.get(key, "")).strip()
            if value:
                break
        counter[value or "unknown"] += 1
    return dict(sorted(counter.items()))


def _group_value(row: dict, group_key: str) -> str:
    if group_key == "profile":
        keys = ("profile", "link_profile", "communication_profile_mode")
    elif group_key == "baseline_mode":
        keys = ("baseline_mode", "scheduler_mode", "watchdog_mode")
    else:
        keys = (group_key,)
    for key in keys:
        value = str(row.get(key, "") or "").strip()
        if value:
            return value
    return "unknown"


def _has_any_field(rows: list[dict], keys: tuple[str, ...]) -> bool:
    for row in rows:
        for key in keys:
            if row.get(key) not in ("", None):
                return True
    return False


def _warn_staleness_source_gaps(rows: list[dict], warnings: list[str]) -> None:
    if not rows:
        return
    checks = [
        (("delta_net_proxy_ms", "t_rx"), "Delta_net_proxy missing; left empty unless t_rx/t_source are available"),
        (("delta_exec_proxy_ms", "t_watchdog", "queue_delay_proxy_ms"), "Delta_exec_proxy missing; derived only when watchdog/queue timestamps are available"),
        (("delta_eff_proxy_ms", "age_ms", "age", "age_est", "t_watchdog"), "Delta_eff_proxy missing; derived from available proxy timestamps or age fallback"),
        (("normalized_delta_eff",), "normalized_delta_eff missing; computed offline from Delta_eff_proxy with default normalizer"),
        (("VoI", "voi_score", "voi"), "VoI missing; defaulted to 0.0"),
        (("phase_risk", "risk_score", "risk_proxy"), "phase_risk missing; used task_phase/phase prior if available"),
        (("execution_residual",), "execution_residual missing; computed offline from Delta_exec_proxy with default normalizer"),
        (("communication_uncertainty",), "communication_uncertainty missing; used profile/injection proxy fields if available"),
        (("action_correction_uncertainty", "target_uncertainty", "vision_uncertainty", "qr_uncertainty", "correction_uncertainty"), "action_correction_uncertainty missing; defaulted to 0.0"),
        (("S_eff",), "S_eff missing; computed offline from proxy terms"),
        (("F_eff",), "F_eff missing; computed offline as 1 - clip(S_eff, 0, 1)"),
        (("baseline_mode", "scheduler_mode", "watchdog_mode"), "baseline_mode missing; grouped stats use unknown unless scheduler/watchdog mode fields exist"),
    ]
    for keys, message in checks:
        if not _has_any_field(rows, keys):
            _warn(warnings, message)


def _compute_staleness_risk_rows(rows: list[dict], warnings: list[str]) -> list[dict]:
    _warn_staleness_source_gaps(rows, warnings)
    out_rows: list[dict] = []
    for row in rows:
        computed = compute_staleness_risk(row)
        out = dict(row)
        out.update(computed)
        out_rows.append(out)
    return out_rows


def _write_staleness_risk_rows(path: Path, rows: list[dict]) -> None:
    preferred = [
        "run_id",
        "robot_id",
        "command_id",
        "task_phase",
        "phase",
        "profile",
        "link_profile",
        "communication_profile_mode",
        "baseline_mode",
        "scheduler_mode",
        "watchdog_mode",
        "baseline_execution_environment",
        "allow_unsafe_baseline",
        "watchdog_action",
        "stale_reason",
        *STALENESS_RISK_INPUT_FIELDS,
    ]
    fieldnames: list[str] = []
    for key in preferred:
        if key not in fieldnames:
            fieldnames.append(key)
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            formatted = format_staleness_risk_row(
                {key: row.get(key) for key in STALENESS_RISK_INPUT_FIELDS if key in row}
            )
            writer.writerow({**row, **formatted})


def _staleness_risk_stats(rows: list[dict]) -> dict:
    return {
        metric: _stats(_metric_values(rows, metric, []))
        for metric in STALENESS_RISK_FIELDS
    }


def _group_metric_stats(rows: list[dict], group_key: str, warnings: list[str]) -> dict:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        group = _group_value(row, group_key)
        grouped[group].append(row)
    if rows and all(group == "unknown" for group in grouped):
        _warn(warnings, f"{group_key} missing; grouped stats use unknown")
    out = {}
    for group, group_rows in sorted(grouped.items()):
        stats = {
            "age_ms": _stats(_age_ms_values(group_rows, [])),
            "delta_net_proxy_ms": _stats(_metric_values(group_rows, "delta_net_proxy_ms", [])),
            "delta_exec_proxy_ms": _stats(_metric_values(group_rows, "delta_exec_proxy_ms", [])),
            "delta_eff_proxy_ms": _stats(_metric_values(group_rows, "delta_eff_proxy_ms", [])),
        }
        stats.update(_staleness_risk_stats(group_rows))
        out[group] = stats
    return out


def _event_counts(watchdog_rows: list[dict], mission_rows: list[dict]) -> dict:
    stale = 0
    decay = 0
    stop = 0
    for row in watchdog_rows:
        action = str(row.get("watchdog_action") or row.get("state") or "").strip()
        reason = str(row.get("stale_reason") or "").strip()
        if reason or action in {"DECAY", "AGE_STOP", "CMD_STOP", "EMERGENCY_STOP"}:
            stale += 1
        if action == "DECAY":
            decay += 1
        if action in {"AGE_STOP", "CMD_STOP", "EMERGENCY_STOP", "STOP"}:
            stop += 1

    freeze = sum(_as_int(row, "base_freeze_event") for row in mission_rows)
    freeze += sum(1 for row in mission_rows if str(row.get("event_type", "")).strip() == "FREEZE_ON")
    safe_abort = sum(_as_int(row, "safe_abort_event") for row in mission_rows)
    safe_abort += sum(1 for row in mission_rows if str(row.get("outcome", "")).strip() == "safe_abort")
    return {
        "stale_event_count": stale,
        "decay_event_count": decay,
        "stop_event_count": stop,
        "freeze_event_count": freeze,
        "safe_abort_event_count": safe_abort,
    }


def _markdown_table(mapping: dict, columns: list[str]) -> list[str]:
    lines = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join(["---"] * len(columns)) + " |",
    ]
    for key, value in mapping.items():
        lines.append("| " + " | ".join([str(key), str(value)]) + " |")
    return lines


def _fmt_stat(value) -> str:
    return "" if value is None else f"{float(value):.3f}"


def _group_stats_table(group_stats: dict) -> list[str]:
    lines = [
        "| group | age_p95_ms | net_p95_ms | exec_p95_ms | eff_p95_ms | eff_p99_ms | S_eff_p95 | F_eff_p50 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for group, stats in sorted(group_stats.items()):
        age = stats["age_ms"]
        net = stats.get("delta_net_proxy_ms", {})
        exec_proxy = stats.get("delta_exec_proxy_ms", {})
        eff = stats["delta_eff_proxy_ms"]
        s_eff = stats.get("S_eff", {})
        f_eff = stats.get("F_eff", {})
        lines.append(
            f"| {group} | {_fmt_stat(age.get('p95'))} | {_fmt_stat(net.get('p95'))} | "
            f"{_fmt_stat(exec_proxy.get('p95'))} | {_fmt_stat(eff.get('p95'))} | {_fmt_stat(eff.get('p99'))} | "
            f"{_fmt_stat(s_eff.get('p95'))} | {_fmt_stat(f_eff.get('p50'))} |"
        )
    if len(lines) == 2:
        lines.append("| none |  |  |  |  |  |  |  |")
    return lines


def render_markdown(summary: dict) -> str:
    lines = [
        "# Effective freshness analysis report",
        "",
        "This report analyzes controller-side proxy freshness fields. It does not claim DDS receive latency, actuator latency, physical residual ground truth, or 5G validation.",
        "",
        "## Inputs",
        "",
        f"- Scheduler CSVs: {summary['inputs']['scheduler_csvs']}",
        f"- Watchdog CSVs: {summary['inputs']['watchdog_csvs']}",
        f"- Mission CSVs: {summary['inputs']['mission_csvs']}",
        "",
        "## Counts",
        "",
    ]
    for key, value in summary["counts"].items():
        lines.append(f"- {key}: {value}")
    lines.extend(["", "## Freshness statistics", ""])
    for metric in ("age_ms", "delta_net_proxy_ms", "delta_exec_proxy_ms", "delta_eff_proxy_ms"):
        stat = summary["freshness_stats"][metric]
        lines.append(
            f"- {metric}: count={stat['count']} mean={_fmt_stat(stat['mean'])} "
            f"p50={_fmt_stat(stat['p50'])} p95={_fmt_stat(stat['p95'])} "
            f"p99={_fmt_stat(stat['p99'])}"
        )
    lines.extend(["", "## Staleness-risk statistics", ""])
    for metric in STALENESS_RISK_FIELDS:
        stat = summary["staleness_risk_stats"][metric]
        lines.append(
            f"- {metric}: count={stat['count']} mean={_fmt_stat(stat['mean'])} "
            f"p50={_fmt_stat(stat['p50'])} p95={_fmt_stat(stat['p95'])} "
            f"p99={_fmt_stat(stat['p99'])}"
        )
    lines.extend(["", "## Watchdog action distribution", ""])
    lines.extend(_markdown_table(summary["watchdog_action_distribution"], ["action", "count"]))
    lines.extend(["", "## Scheduler decision distribution", ""])
    lines.extend(_markdown_table(summary["scheduler_decision_distribution"], ["decision", "count"]))
    lines.extend(["", "## By task phase", ""])
    lines.extend(_group_stats_table(summary["by_task_phase"]))
    lines.extend(["", "## By profile", ""])
    lines.extend(_group_stats_table(summary["by_profile"]))
    lines.extend(["", "## By baseline mode", ""])
    lines.extend(_group_stats_table(summary["by_baseline_mode"]))
    lines.extend(["", "## Warnings", ""])
    if summary["warnings"]:
        for warning in summary["warnings"]:
            lines.append(f"- {warning}")
    else:
        lines.append("- none")
    lines.append("")
    return "\n".join(lines)


def analyze(run_dir: str, scheduler_csv: str = "", watchdog_csv: str = "", mission_csv: str = "", out_dir: str = "") -> dict:
    warnings: list[str] = []
    run_path = Path(run_dir).expanduser()
    scheduler_paths = _find_csvs(run_path, scheduler_csv, ["scheduler_audit.csv", "events.csv", "scheduler_events.csv"], "scheduler", warnings)
    watchdog_paths = _find_csvs(run_path, watchdog_csv, ["ts_*.csv", "watchdog_events.csv"], "watchdog", warnings)
    mission_paths = _find_csvs(run_path, mission_csv, ["mission_runtime_events.csv"], "mission", warnings)

    scheduler_rows: list[dict] = []
    for path in scheduler_paths:
        scheduler_rows.extend(_read_csv(path, "scheduler", warnings))
    watchdog_rows: list[dict] = []
    for path in watchdog_paths:
        watchdog_rows.extend(_read_csv(path, "watchdog", warnings))
    mission_rows: list[dict] = []
    for path in mission_paths:
        mission_rows.extend(_read_csv(path, "mission", warnings))

    merged_watchdog_rows = _merge_watchdog_with_scheduler(watchdog_rows, scheduler_rows)
    metric_rows = merged_watchdog_rows if merged_watchdog_rows else scheduler_rows
    staleness_risk_rows = _compute_staleness_risk_rows(metric_rows, warnings)

    age_values = _age_ms_values(metric_rows, warnings)
    delta_net_values = _metric_values(merged_watchdog_rows, "delta_net_proxy_ms", warnings)
    delta_exec_values = _metric_values(merged_watchdog_rows, "delta_exec_proxy_ms", warnings)
    delta_eff_values = _metric_values(merged_watchdog_rows, "delta_eff_proxy_ms", warnings)

    summary = {
        "inputs": {
            "run_dir": str(run_path),
            "scheduler_csvs": [str(path) for path in scheduler_paths],
            "watchdog_csvs": [str(path) for path in watchdog_paths],
            "mission_csvs": [str(path) for path in mission_paths],
        },
        "counts": {
            "total_scheduler_events": len(scheduler_rows),
            "total_watchdog_events": len(watchdog_rows),
            "total_mission_events": len(mission_rows),
            **_event_counts(merged_watchdog_rows, mission_rows),
        },
        "freshness_stats": {
            "age_ms": _stats(age_values),
            "delta_net_proxy_ms": _stats(delta_net_values),
            "delta_exec_proxy_ms": _stats(delta_exec_values),
            "delta_eff_proxy_ms": _stats(delta_eff_values),
        },
        "staleness_risk_stats": _staleness_risk_stats(staleness_risk_rows),
        "by_task_phase": _group_metric_stats(staleness_risk_rows, "task_phase", warnings),
        "by_link_profile": _group_metric_stats(staleness_risk_rows, "link_profile", warnings),
        "by_profile": _group_metric_stats(staleness_risk_rows, "profile", warnings),
        "by_baseline_mode": _group_metric_stats(staleness_risk_rows, "baseline_mode", warnings),
        "watchdog_action_distribution": _distribution(merged_watchdog_rows, ("watchdog_action", "state")),
        "scheduler_decision_distribution": _distribution(scheduler_rows, ("scheduler_decision", "reason", "decision_class")),
        "warnings": warnings,
    }

    output_dir = Path(out_dir).expanduser() if out_dir else run_path
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "effective_freshness_summary.json"
    report_path = output_dir / "effective_freshness_report.md"
    staleness_risk_path = output_dir / "effective_staleness_risk_rows.csv"
    _write_staleness_risk_rows(staleness_risk_path, staleness_risk_rows)
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    report_path.write_text(render_markdown(summary), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze controller-side effective freshness proxy CSV logs.")
    parser.add_argument("--run-dir", required=True, help="Directory containing scheduler/watchdog/mission CSV logs")
    parser.add_argument("--scheduler-csv", default="", help="Optional explicit scheduler CSV")
    parser.add_argument("--watchdog-csv", default="", help="Optional explicit watchdog ts CSV")
    parser.add_argument("--mission-csv", default="", help="Optional explicit mission runtime CSV")
    parser.add_argument("--out-dir", default="", help="Optional output directory")
    args = parser.parse_args()
    summary = analyze(
        run_dir=args.run_dir,
        scheduler_csv=args.scheduler_csv,
        watchdog_csv=args.watchdog_csv,
        mission_csv=args.mission_csv,
        out_dir=args.out_dir,
    )
    print(json.dumps(summary["counts"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
