from __future__ import annotations

from freshness_real_robot_validation.phase_contract import (
    MISSION_STATE_PHASE_MAP,
    MISSION_STATE_PROGRESS_MAP,
    PHASE_ORDER,
    build_phase_status_payload,
    compute_global_task_progress,
)


def test_mission_state_mapping_is_locked_to_real_system_plan():
    assert PHASE_ORDER == ("approach", "slide_align", "level_recenter", "transport")
    assert MISSION_STATE_PHASE_MAP["WAIT_WING"] == "approach"
    assert MISSION_STATE_PHASE_MAP["RUN_ALIGNMENT"] == "approach"
    assert MISSION_STATE_PHASE_MAP["SYNC_SLIDE_ALIGN"] == "slide_align"
    assert MISSION_STATE_PHASE_MAP["SYNC_LEVEL_Z"] == "level_recenter"
    assert MISSION_STATE_PHASE_MAP["TRANSPORT_PRECHECK"] == "transport"
    assert MISSION_STATE_PHASE_MAP["DONE"] == "transport"
    assert MISSION_STATE_PHASE_MAP["ABORT"] == "abort"
    assert MISSION_STATE_PHASE_MAP["STANDBY"] == "standby"


def test_progress_proxy_mapping_is_locked_to_runtime_states():
    assert MISSION_STATE_PROGRESS_MAP["WAIT_WING"] == 0.15
    assert MISSION_STATE_PROGRESS_MAP["SYNC_APPROACH_X"] == 0.40
    assert MISSION_STATE_PROGRESS_MAP["SYNC_APPROACH_Y"] == 0.70
    assert MISSION_STATE_PROGRESS_MAP["WAIT_ENTRY_RELEASE"] == 0.85
    assert MISSION_STATE_PROGRESS_MAP["RUN_ALIGNMENT"] == 0.95
    assert MISSION_STATE_PROGRESS_MAP["PHASE1_DONE_HOLD"] == 1.00
    assert MISSION_STATE_PROGRESS_MAP["SYNC_SLIDE_ALIGN"] == 0.60
    assert MISSION_STATE_PROGRESS_MAP["ALL_READY_HOLD"] == 1.00
    assert MISSION_STATE_PROGRESS_MAP["SYNC_LEVEL_Z"] == 0.40
    assert MISSION_STATE_PROGRESS_MAP["SYNC_RECENTER"] == 0.75
    assert MISSION_STATE_PROGRESS_MAP["LOAD_STABLE_HOLD"] == 1.00
    assert MISSION_STATE_PROGRESS_MAP["TRANSPORT_PRECHECK"] == 0.20
    assert MISSION_STATE_PROGRESS_MAP["SYNC_TRANSPORT"] == 0.70
    assert MISSION_STATE_PROGRESS_MAP["TRANSPORT_SETTLE"] == 0.90
    assert MISSION_STATE_PROGRESS_MAP["DONE"] == 1.00


def test_global_progress_normalization_uses_phase_index_over_four_phases():
    progress = compute_global_task_progress("SYNC_RECENTER")
    assert progress == (2.0 + 0.75) / 4.0


def test_abort_freezes_last_valid_progress_and_marks_payload_aborted():
    payload = build_phase_status_payload(
        mission_state="ABORT",
        source_mode="mission_runtime_tail",
        run_id="run-1",
        last_valid_task_progress=0.8125,
    )

    assert payload["mission_state"] == "ABORT"
    assert payload["task_phase"] == "abort"
    assert payload["task_progress"] == 0.8125
    assert payload["aborted"] is True


def test_standby_resets_global_progress_to_zero():
    payload = build_phase_status_payload(
        mission_state="STANDBY",
        source_mode="geometry_heuristic",
        run_id="run-2",
        last_valid_task_progress=0.5,
    )

    assert payload["task_phase"] == "standby"
    assert payload["phase_progress_proxy"] == 0.0
    assert payload["task_progress"] == 0.0
