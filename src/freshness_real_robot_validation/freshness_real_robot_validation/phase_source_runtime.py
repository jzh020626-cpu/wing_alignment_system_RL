from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


DEFAULT_SOURCE_PRIORITY = ("native_topic", "mission_runtime_tail", "geometry_heuristic")
EXPLICIT_SOURCE_MODES = {"native_topic", "mission_runtime_tail", "mission_runtime_replay", "geometry_heuristic"}


def resolve_phase_source_order(
    *,
    config_priority: Iterable[str],
    phase_source_mode: str,
    fallback_policy: str,
    mission_runtime_events_path: str,
) -> list[str]:
    mode = str(phase_source_mode or "auto").strip() or "auto"
    explicit_runtime_path = bool(str(mission_runtime_events_path or "").strip())

    if mode in EXPLICIT_SOURCE_MODES:
        return [mode]

    order = [str(item).strip() for item in config_priority if str(item).strip()] or list(DEFAULT_SOURCE_PRIORITY)
    if explicit_runtime_path and str(fallback_policy or "").strip() == "geometry_only_if_explicit":
        order = [item for item in order if item != "geometry_heuristic"]
    return order


def resolve_runtime_events_path(
    *,
    mission_runtime_events_path: str,
    mission_runtime_csv: str,
    mission_log_root: str,
    run_id: str,
) -> Path | None:
    explicit_path = str(mission_runtime_events_path or "").strip()
    if explicit_path:
        return Path(explicit_path).expanduser()

    legacy_path = str(mission_runtime_csv or "").strip()
    if legacy_path:
        return Path(legacy_path).expanduser()

    root = str(mission_log_root or "").strip()
    if not root:
        return None

    base = Path(root).expanduser()
    run = str(run_id or "").strip()
    if run:
        return base / run / "mission_runtime_events.csv"
    return base / "mission_runtime_events.csv"


def build_phase_source_status(
    *,
    selected_phase_source: str,
    runtime_path: Path | None,
    file_exists: bool,
    last_modified_ns: int | None,
    tail_update_status: str,
    phase_valid_count: int,
    task_progress_valid_count: int,
    publish_count: int,
) -> dict:
    publish_total = max(int(publish_count), 0)
    phase_valid_rate = 0.0 if publish_total <= 0 else float(phase_valid_count) / float(publish_total)
    task_progress_valid_rate = 0.0 if publish_total <= 0 else float(task_progress_valid_count) / float(publish_total)

    last_modified_time = ""
    if last_modified_ns is not None:
        last_modified_time = datetime.fromtimestamp(last_modified_ns / 1_000_000_000, tz=timezone.utc).isoformat()

    return {
        "selected_phase_source": str(selected_phase_source or "unavailable"),
        "selected_file_path": str(runtime_path) if runtime_path is not None else "",
        "file_exists": bool(file_exists),
        "last_modified_time": last_modified_time,
        "tail_update_status": str(tail_update_status or "unknown"),
        "phase_valid_rate": round(phase_valid_rate, 6),
        "task_progress_valid_rate": round(task_progress_valid_rate, 6),
    }


def format_phase_source_status(status: dict) -> str:
    return (
        "[PHASE_SOURCE] "
        f"selected_phase_source={status.get('selected_phase_source', 'unavailable')} "
        f"file_path={status.get('selected_file_path', '')} "
        f"file_exists={'yes' if status.get('file_exists', False) else 'no'} "
        f"last_modified_time={status.get('last_modified_time', '')} "
        f"tail_update_status={status.get('tail_update_status', 'unknown')} "
        f"phase_valid_rate={float(status.get('phase_valid_rate', 0.0)):.3f} "
        f"task_progress_valid_rate={float(status.get('task_progress_valid_rate', 0.0)):.3f}"
    )
