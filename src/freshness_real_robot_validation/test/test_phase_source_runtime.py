from __future__ import annotations

from pathlib import Path

from freshness_real_robot_validation.phase_source_runtime import (
    build_phase_source_status,
    format_phase_source_status,
    resolve_phase_source_order,
    resolve_runtime_events_path,
)


def test_explicit_mission_runtime_mode_disables_geometry_fallback():
    order = resolve_phase_source_order(
        config_priority=["native_topic", "mission_runtime_tail", "geometry_heuristic"],
        phase_source_mode="mission_runtime_tail",
        fallback_policy="geometry_only_if_explicit",
        mission_runtime_events_path="/tmp/live/mission_runtime_events.csv",
    )

    assert order == ["mission_runtime_tail"]


def test_explicit_mission_runtime_replay_mode_is_supported():
    order = resolve_phase_source_order(
        config_priority=["native_topic", "mission_runtime_tail", "geometry_heuristic"],
        phase_source_mode="mission_runtime_replay",
        fallback_policy="geometry_only_if_explicit",
        mission_runtime_events_path="/tmp/live/mission_runtime_events.csv",
    )

    assert order == ["mission_runtime_replay"]


def test_explicit_runtime_path_uses_run_root_and_run_id_when_leaf_not_provided():
    runtime_path = resolve_runtime_events_path(
        mission_runtime_events_path="",
        mission_runtime_csv="",
        mission_log_root="~/.ros/mission_bench_logs",
        run_id="rlive3b_safe_idle",
    )

    assert runtime_path == Path("~/.ros/mission_bench_logs").expanduser() / "rlive3b_safe_idle" / "mission_runtime_events.csv"


def test_phase_source_status_log_reports_selected_source_and_valid_rates():
    status = build_phase_source_status(
        selected_phase_source="mission_runtime_tail",
        runtime_path=Path("/tmp/live/mission_runtime_events.csv"),
        file_exists=True,
        last_modified_ns=1710000000000000000,
        tail_update_status="fresh",
        phase_valid_count=9,
        task_progress_valid_count=8,
        publish_count=10,
    )

    assert status["phase_valid_rate"] == 0.9
    assert status["task_progress_valid_rate"] == 0.8
    line = format_phase_source_status(status)
    assert "selected_phase_source=mission_runtime_tail" in line
    assert "file_exists=yes" in line
    assert "tail_update_status=fresh" in line
