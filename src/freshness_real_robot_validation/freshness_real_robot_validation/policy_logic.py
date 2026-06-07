from __future__ import annotations


PAYLOAD_BYTES = {
    "skip_update": 0,
    "compact_update": 128,
    "full_update": 256,
    "urgent_refresh": 320,
}

DGWS_NAME = "DGWS"
FR_TPO_NAME = "FR-TPO"
FR_TPO_KEY = "FR_TPO"
PERIODIC_FULL_NAME = "periodic_full"
EVENT_TRIGGERED_NAME = "event_triggered"
FRESHNESS_AWARE_NAME = "freshness_aware_tx"
CRITICAL_PHASES = {"narrow_passage", "final_alignment", "slide_align", "level_recenter"}
LOW_RISK_PHASES = {"approach", "transport", "cooperative_transport", "release_exit"}
EXECUTION_MODES = ("normal", "degraded", "hold", "safe_stop")


def _canonical_payload_bytes(mode: str) -> int:
    return int(PAYLOAD_BYTES[str(mode)])


def _dgws_mode(*, task_phase: str, effective_freshness: float, aoi_ms: float, stale_indicator: float) -> str:
    if stale_indicator >= 1.0 or effective_freshness < 0.40 or aoi_ms >= 350.0:
        return "urgent_refresh"
    if task_phase in CRITICAL_PHASES:
        return "full_update"
    return "full_update"


def _fr_tpo_mode(
    *,
    task_phase: str,
    task_progress: float,
    effective_freshness: float,
    aoi_ms: float,
    stale_indicator: float,
) -> tuple[str, bool]:
    low_fresh = effective_freshness < 0.45
    high_aoi = aoi_ms >= 300.0
    stale = stale_indicator >= 1.0
    critical = task_phase in CRITICAL_PHASES
    if stale or (critical and (low_fresh or high_aoi)):
        return ("urgent_refresh" if stale or aoi_ms >= 400.0 else "full_update"), True
    if task_phase in LOW_RISK_PHASES and effective_freshness >= 0.85 and aoi_ms <= 150.0 and task_progress < 0.85:
        return "compact_update", False
    return "full_update", False


def _decision(method_id: str, transmission_mode: str, execution_mode: str = "normal") -> dict:
    return {
        "method_id": str(method_id),
        "transmission_mode": str(transmission_mode),
        "execution_mode": str(execution_mode),
        "payload_bytes": _canonical_payload_bytes(transmission_mode),
    }


def _canonical_method_id(method_id: str) -> str:
    raw = str(method_id or "").strip()
    lowered = raw.lower()
    if lowered in {"b0", PERIODIC_FULL_NAME}:
        return PERIODIC_FULL_NAME
    if lowered in {"b1", EVENT_TRIGGERED_NAME}:
        return EVENT_TRIGGERED_NAME
    if lowered in {"b2", "b3", FRESHNESS_AWARE_NAME, "fr-tpo", "fr_tpo"}:
        return FRESHNESS_AWARE_NAME
    if raw == DGWS_NAME:
        return DGWS_NAME
    return raw or FR_TPO_NAME


def _event_triggered_mode(*, task_phase: str, effective_freshness: float, aoi_ms: float, stale_indicator: float) -> str:
    if task_phase in CRITICAL_PHASES:
        return "full_update"
    if stale_indicator >= 1.0 or effective_freshness < 0.55 or aoi_ms >= 220.0:
        return "full_update"
    return "skip_update"


def _fr_tpo_execution_mode(
    *,
    task_phase: str,
    effective_freshness: float,
    aoi_ms: float,
    stale_indicator: float,
    enable_execution_mode: bool,
) -> str:
    if not enable_execution_mode:
        return "normal"
    if stale_indicator >= 1.0 or aoi_ms >= 500.0:
        return "safe_stop"
    if task_phase in CRITICAL_PHASES and (effective_freshness < 0.40 or aoi_ms >= 350.0):
        return "hold"
    if effective_freshness < 0.65 or aoi_ms >= 220.0:
        return "degraded"
    return "normal"


def decide_shadow_pair(
    *,
    task_phase: str,
    task_progress: float,
    effective_freshness: float,
    aoi_ms: float,
    stale_indicator: float,
    enable_execution_mode: bool = False,
) -> dict:
    dgws_mode = _dgws_mode(
        task_phase=str(task_phase),
        effective_freshness=float(effective_freshness),
        aoi_ms=float(aoi_ms),
        stale_indicator=float(stale_indicator),
    )
    fr_mode, blocked = _fr_tpo_mode(
        task_phase=str(task_phase),
        task_progress=float(task_progress),
        effective_freshness=float(effective_freshness),
        aoi_ms=float(aoi_ms),
        stale_indicator=float(stale_indicator),
    )
    exec_mode = _fr_tpo_execution_mode(
        task_phase=str(task_phase),
        effective_freshness=float(effective_freshness),
        aoi_ms=float(aoi_ms),
        stale_indicator=float(stale_indicator),
        enable_execution_mode=bool(enable_execution_mode),
    )
    return {
        DGWS_NAME: _decision(DGWS_NAME, dgws_mode, "normal"),
        FR_TPO_KEY: _decision(FR_TPO_NAME, fr_mode, exec_mode),
        "blocked_unsafe_downgrade": bool(blocked),
    }


def decide_tx_mode(
    *,
    method_id: str,
    task_phase: str,
    task_progress: float,
    effective_freshness: float,
    aoi_ms: float,
    stale_indicator: float,
    enable_execution_mode: bool = False,
) -> dict:
    canonical_method_id = _canonical_method_id(method_id)
    if canonical_method_id == DGWS_NAME:
        mode = _dgws_mode(
            task_phase=str(task_phase),
            effective_freshness=float(effective_freshness),
            aoi_ms=float(aoi_ms),
            stale_indicator=float(stale_indicator),
        )
        return _decision(method_id, mode, "normal")
    if canonical_method_id == PERIODIC_FULL_NAME:
        return _decision(method_id, "full_update", "normal")
    if canonical_method_id == EVENT_TRIGGERED_NAME:
        mode = _event_triggered_mode(
            task_phase=str(task_phase),
            effective_freshness=float(effective_freshness),
            aoi_ms=float(aoi_ms),
            stale_indicator=float(stale_indicator),
        )
        return _decision(method_id, mode, "normal")
    mode, blocked = _fr_tpo_mode(
        task_phase=str(task_phase),
        task_progress=float(task_progress),
        effective_freshness=float(effective_freshness),
        aoi_ms=float(aoi_ms),
        stale_indicator=float(stale_indicator),
    )
    execution_mode = _fr_tpo_execution_mode(
        task_phase=str(task_phase),
        effective_freshness=float(effective_freshness),
        aoi_ms=float(aoi_ms),
        stale_indicator=float(stale_indicator),
        enable_execution_mode=bool(enable_execution_mode),
    )
    decision = _decision(method_id, mode, execution_mode)
    decision["blocked_unsafe_downgrade"] = bool(blocked)
    return decision
