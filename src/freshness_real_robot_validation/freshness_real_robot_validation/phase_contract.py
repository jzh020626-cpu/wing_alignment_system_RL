from __future__ import annotations

from dataclasses import dataclass


PHASE_ORDER = ("approach", "slide_align", "level_recenter", "transport")

MISSION_STATE_PHASE_MAP = {
    "WAIT_WING": "approach",
    "RUN_ALIGNMENT": "approach",
    "PHASE1_DONE_HOLD": "approach",
    "SYNC_SLIDE_ALIGN": "slide_align",
    "ALL_READY_HOLD": "slide_align",
    "SYNC_LEVEL_Z": "level_recenter",
    "SYNC_RECENTER": "level_recenter",
    "LOAD_STABLE_HOLD": "level_recenter",
    "TRANSPORT_PRECHECK": "transport",
    "SYNC_TRANSPORT": "transport",
    "TRANSPORT_SETTLE": "transport",
    "DONE": "transport",
    "ABORT": "abort",
    "STANDBY": "standby",
}

MISSION_STATE_PROGRESS_MAP = {
    "WAIT_WING": 0.15,
    "SYNC_APPROACH_X": 0.40,
    "SYNC_APPROACH_Y": 0.70,
    "WAIT_ENTRY_RELEASE": 0.85,
    "RUN_ALIGNMENT": 0.95,
    "PHASE1_DONE_HOLD": 1.00,
    "SYNC_SLIDE_ALIGN": 0.60,
    "ALL_READY_HOLD": 1.00,
    "SYNC_LEVEL_Z": 0.40,
    "SYNC_RECENTER": 0.75,
    "LOAD_STABLE_HOLD": 1.00,
    "TRANSPORT_PRECHECK": 0.20,
    "SYNC_TRANSPORT": 0.70,
    "TRANSPORT_SETTLE": 0.90,
    "DONE": 1.00,
}


@dataclass(frozen=True)
class PhaseStatus:
    mission_state: str
    task_phase: str
    phase_progress_proxy: float
    task_progress: float
    aborted: bool


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def map_mission_state_to_phase(mission_state: str) -> str:
    state = str(mission_state or "").strip().upper()
    if not state:
        return "standby"
    return MISSION_STATE_PHASE_MAP.get(state, state.lower())


def map_mission_state_to_phase_progress(mission_state: str) -> float:
    state = str(mission_state or "").strip().upper()
    return clamp01(MISSION_STATE_PROGRESS_MAP.get(state, 0.0))


def compute_global_task_progress(mission_state: str, *, phase_progress_proxy: float | None = None) -> float:
    phase = map_mission_state_to_phase(mission_state)
    if phase == "standby":
        return 0.0
    if phase not in PHASE_ORDER:
        return 0.0
    phase_index = float(PHASE_ORDER.index(phase))
    progress = map_mission_state_to_phase_progress(mission_state) if phase_progress_proxy is None else clamp01(phase_progress_proxy)
    return (phase_index + progress) / float(len(PHASE_ORDER))


def build_phase_status_payload(
    *,
    mission_state: str,
    source_mode: str,
    run_id: str,
    last_valid_task_progress: float | None = None,
    confidence: float = 1.0,
    phase_progress_proxy: float | None = None,
    native_phase: str | None = None,
) -> dict:
    state = str(mission_state or "").strip().upper()
    task_phase = str(native_phase or map_mission_state_to_phase(state))
    progress_proxy = clamp01(map_mission_state_to_phase_progress(state) if phase_progress_proxy is None else phase_progress_proxy)
    aborted = task_phase == "abort"

    if task_phase == "standby":
        task_progress = 0.0
        progress_proxy = 0.0
    elif aborted:
        task_progress = clamp01(last_valid_task_progress or 0.0)
    elif task_phase in PHASE_ORDER:
        task_progress = compute_global_task_progress(state, phase_progress_proxy=progress_proxy)
    else:
        task_progress = clamp01(last_valid_task_progress or 0.0)

    status = PhaseStatus(
        mission_state=state or "STANDBY",
        task_phase=task_phase,
        phase_progress_proxy=progress_proxy,
        task_progress=task_progress,
        aborted=aborted,
    )
    return {
        "mission_state": status.mission_state,
        "task_phase": status.task_phase,
        "phase_progress_proxy": status.phase_progress_proxy,
        "task_progress": status.task_progress,
        "source_mode": str(source_mode),
        "confidence": clamp01(confidence),
        "run_id": str(run_id),
        "aborted": status.aborted,
    }
