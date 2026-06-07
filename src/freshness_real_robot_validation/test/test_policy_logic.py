from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from freshness_real_robot_validation.policy_logic import (
    decide_shadow_pair,
    decide_tx_mode,
)


def test_shadow_pair_prefers_compact_for_low_risk_transport():
    pair = decide_shadow_pair(
        task_phase="transport",
        task_progress=0.35,
        effective_freshness=0.92,
        aoi_ms=85.0,
        stale_indicator=0.0,
    )

    assert pair["DGWS"]["transmission_mode"] == "full_update"
    assert pair["FR_TPO"]["transmission_mode"] == "compact_update"
    assert pair["FR_TPO"]["payload_bytes"] < pair["DGWS"]["payload_bytes"]


def test_shadow_pair_blocks_unsafe_downgrade_in_narrow_passage():
    pair = decide_shadow_pair(
        task_phase="narrow_passage",
        task_progress=0.72,
        effective_freshness=0.31,
        aoi_ms=420.0,
        stale_indicator=1.0,
    )

    assert pair["FR_TPO"]["transmission_mode"] in {"full_update", "urgent_refresh"}
    assert pair["blocked_unsafe_downgrade"] is True


def test_tx_policy_keeps_execution_mode_as_normal_execute():
    decision = decide_tx_mode(
        method_id="FR-TPO",
        task_phase="transport",
        task_progress=0.2,
        effective_freshness=0.95,
        aoi_ms=70.0,
        stale_indicator=0.0,
    )

    assert decision["method_id"] == "FR-TPO"
    assert decision["execution_mode"] == "normal"
    assert decision["transmission_mode"] == "compact_update"


def test_tx_policy_can_enable_execution_modes_without_changing_default_callers():
    decision = decide_tx_mode(
        method_id="FR-TPO",
        task_phase="transport",
        task_progress=0.6,
        effective_freshness=0.60,
        aoi_ms=260.0,
        stale_indicator=0.0,
        enable_execution_mode=True,
    )

    assert decision["execution_mode"] == "degraded"


def test_tx_policy_uses_hold_for_critical_low_freshness_when_enabled():
    decision = decide_tx_mode(
        method_id="FR-TPO",
        task_phase="slide_align",
        task_progress=0.6,
        effective_freshness=0.35,
        aoi_ms=360.0,
        stale_indicator=0.0,
        enable_execution_mode=True,
    )

    assert decision["execution_mode"] == "hold"


def test_tx_policy_uses_safe_stop_for_stale_state_when_enabled():
    decision = decide_tx_mode(
        method_id="FR-TPO",
        task_phase="transport",
        task_progress=0.9,
        effective_freshness=0.20,
        aoi_ms=520.0,
        stale_indicator=1.0,
        enable_execution_mode=True,
    )

    assert decision["execution_mode"] == "safe_stop"


def test_periodic_full_baseline_stays_full_update_and_normal():
    decision = decide_tx_mode(
        method_id="periodic_full",
        task_phase="transport",
        task_progress=0.5,
        effective_freshness=0.95,
        aoi_ms=80.0,
        stale_indicator=0.0,
    )

    assert decision["transmission_mode"] == "full_update"
    assert decision["execution_mode"] == "normal"


def test_event_triggered_baseline_skips_low_risk_fresh_updates():
    decision = decide_tx_mode(
        method_id="event_triggered",
        task_phase="transport",
        task_progress=0.5,
        effective_freshness=0.92,
        aoi_ms=90.0,
        stale_indicator=0.0,
    )

    assert decision["transmission_mode"] == "skip_update"
    assert decision["execution_mode"] == "normal"
