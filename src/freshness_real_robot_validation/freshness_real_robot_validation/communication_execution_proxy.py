from __future__ import annotations

import math


CRITICAL_PHASES = {"slide_align", "level_recenter", "final_alignment", "narrow_passage"}

PHASE_AOI_BONUS_MS = {
    "approach": 0.0,
    "transport": 35.0,
    "slide_align": 185.0,
    "level_recenter": 150.0,
    "final_alignment": 210.0,
    "narrow_passage": 190.0,
}

PHASE_JITTER_SCALE = {
    "approach": 1.00,
    "transport": 1.10,
    "slide_align": 1.35,
    "level_recenter": 1.25,
    "final_alignment": 1.40,
    "narrow_passage": 1.30,
}


def build_comm_proxy_snapshot(*, task_phase: str, scenario_id: str, scenario_cfg: dict[str, object]) -> dict[str, float | str]:
    phase = str(task_phase or "standby").strip() or "standby"
    critical = phase in CRITICAL_PHASES
    delay_ms_mean = max(0.0, float(scenario_cfg.get("delay_ms_mean", 0.0) or 0.0))
    jitter_ms = max(0.0, float(scenario_cfg.get("jitter_ms", 0.0) or 0.0))
    loss_rate = max(0.0, min(1.0, float(scenario_cfg.get("loss_rate", 0.0) or 0.0)))
    burst_loss_rate = max(0.0, min(1.0, float(scenario_cfg.get("burst_loss_rate", 0.0) or 0.0)))
    duplicate_on_critical = bool(scenario_cfg.get("duplicate_on_critical", False))

    phase_bonus_ms = float(PHASE_AOI_BONUS_MS.get(phase, 20.0))
    delay_proxy_ms = delay_ms_mean + (0.20 * phase_bonus_ms)
    jitter_proxy_ms = jitter_ms * float(PHASE_JITTER_SCALE.get(phase, 1.05))
    duplicate_proxy = 1.0 if duplicate_on_critical and critical else 0.0
    retry_proxy = min(3.0, round((loss_rate * 8.0) + (burst_loss_rate * 10.0) + duplicate_proxy, 3))
    loss_proxy = min(1.0, loss_rate + (0.5 * burst_loss_rate) + (0.02 if duplicate_proxy > 0.0 else 0.0))

    aoi_ms = (
        70.0
        + (1.40 * delay_ms_mean)
        + (3.00 * jitter_ms)
        + (400.0 * loss_rate)
        + (250.0 * burst_loss_rate)
        + phase_bonus_ms
        + (25.0 if duplicate_proxy > 0.0 else 0.0)
    )
    aoi_ms = max(20.0, aoi_ms)

    effective_freshness = math.exp(-aoi_ms / 450.0) * (1.0 - (0.60 * loss_proxy))
    effective_freshness = max(0.0, min(1.0, effective_freshness))
    stale_indicator = 1.0 if (aoi_ms >= 480.0 or effective_freshness < 0.30) else 0.0

    return {
        "scenario_id": str(scenario_id),
        "AoI_ms": round(aoi_ms, 3),
        "Effective_Freshness": round(effective_freshness, 6),
        "stale_indicator": round(stale_indicator, 3),
        "delay_ms_proxy": round(delay_proxy_ms, 3),
        "jitter_ms_proxy": round(jitter_proxy_ms, 3),
        "loss_rate_proxy": round(loss_proxy, 6),
        "duplicate_proxy": round(duplicate_proxy, 3),
        "retry_proxy": round(retry_proxy, 3),
    }
