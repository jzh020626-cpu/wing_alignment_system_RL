from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from freshness_real_robot_validation.communication_execution_proxy import build_comm_proxy_snapshot


def test_comm_proxy_escalates_aoi_in_critical_phase():
    scenario_cfg = {
        "delay_ms_mean": 55.0,
        "jitter_ms": 10.0,
        "loss_rate": 0.05,
        "burst_loss_rate": 0.10,
        "duplicate_on_critical": True,
    }

    approach = build_comm_proxy_snapshot(
        task_phase="approach",
        scenario_id="real-boundary-onset",
        scenario_cfg=scenario_cfg,
    )
    slide_align = build_comm_proxy_snapshot(
        task_phase="slide_align",
        scenario_id="real-boundary-onset",
        scenario_cfg=scenario_cfg,
    )

    assert slide_align["AoI_ms"] > approach["AoI_ms"]
    assert slide_align["Effective_Freshness"] < approach["Effective_Freshness"]
    assert slide_align["duplicate_proxy"] == 1.0


def test_comm_proxy_marks_stale_for_severe_boundary_case():
    scenario_cfg = {
        "delay_ms_mean": 80.0,
        "jitter_ms": 15.0,
        "loss_rate": 0.12,
        "burst_loss_rate": 0.18,
        "duplicate_on_critical": True,
    }

    snapshot = build_comm_proxy_snapshot(
        task_phase="slide_align",
        scenario_id="stress",
        scenario_cfg=scenario_cfg,
    )

    assert snapshot["AoI_ms"] >= 480.0
    assert snapshot["stale_indicator"] == 1.0
