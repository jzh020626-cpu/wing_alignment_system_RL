#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Controller-side effective staleness-risk proxy utilities.

The quantities computed here are proxy-backed analysis fields. They are not
actuator freshness, physical docking residuals, safety certificates, or optimal
control objectives.
"""

from __future__ import annotations


STALENESS_RISK_FIELDS = [
    "normalized_delta_eff",
    "VoI",
    "phase_risk",
    "execution_residual",
    "communication_uncertainty",
    "action_correction_uncertainty",
    "S_eff",
    "F_eff",
]

STALENESS_RISK_INPUT_FIELDS = [
    "delta_net_proxy_ms",
    "delta_exec_proxy_ms",
    "delta_eff_proxy_ms",
    *STALENESS_RISK_FIELDS,
]

DEFAULT_WEIGHTS = {
    "w_delta": 0.35,
    "w_V": 0.15,
    "w_R": 0.20,
    "w_E": 0.15,
    "w_C": 0.10,
    "w_A": 0.05,
}

DEFAULT_NORMALIZERS = {
    "delta_eff_norm_ms": 250.0,
    "delta_exec_norm_ms": 150.0,
    "communication_delay_norm_ms": 150.0,
}

PHASE_RISK_PRIORS = {
    "transport": 0.20,
    "coarse_transport": 0.20,
    "staging": 0.25,
    "approach": 0.50,
    "docking": 0.85,
    "final_alignment": 0.85,
    "final_alignment_docking": 0.85,
}

PROFILE_UNCERTAINTY_PRIORS = {
    "wifi_nominal": 0.05,
    "nominal": 0.05,
    "wifi_degraded": 0.45,
    "network_delay_only": 0.60,
    "delay_low": 0.20,
    "delay_mid": 0.40,
    "delay_high": 0.70,
    "jitter_high": 0.60,
    "loss_low": 0.50,
    "burst_loss": 0.70,
    "executor_backlog": 0.10,
    "executor_backlog_only": 0.10,
    "combined_degraded": 0.80,
    "emulated_delay_jitter_loss": 0.50,
    "fiveg_profile": 0.05,
}


def clip01(value) -> float:
    try:
        return max(0.0, min(1.0, float(value)))
    except (TypeError, ValueError):
        return 0.0


def as_float(row: dict, key: str):
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _first_float(row: dict, keys: tuple[str, ...]):
    for key in keys:
        value = as_float(row, key)
        if value is not None:
            return value
    return None


def _time_delta_ms(row: dict, end_key: str, start_key: str):
    end = as_float(row, end_key)
    start = as_float(row, start_key)
    if end is None or start is None:
        return None
    return max(0.0, (end - start) * 1000.0)


def delta_proxy_ms(row: dict) -> tuple[float | None, float | None, float | None]:
    delta_net = as_float(row, "delta_net_proxy_ms")
    if delta_net is None:
        delta_net = _time_delta_ms(row, "t_rx", "t_source")

    delta_exec = as_float(row, "delta_exec_proxy_ms")
    if delta_exec is None:
        delta_exec = _time_delta_ms(row, "t_watchdog", "t_rx")
    if delta_exec is None:
        delta_exec = as_float(row, "queue_delay_proxy_ms")

    delta_eff = as_float(row, "delta_eff_proxy_ms")
    if delta_eff is None and delta_net is not None and delta_exec is not None:
        delta_eff = delta_net + delta_exec
    if delta_eff is None:
        delta_eff = _time_delta_ms(row, "t_watchdog", "t_source")
    if delta_eff is None:
        age_ms = as_float(row, "age_ms")
        if age_ms is None:
            age_sec = _first_float(row, ("age", "age_est"))
            age_ms = age_sec * 1000.0 if age_sec is not None else None
        delta_eff = age_ms

    return delta_net, delta_exec, delta_eff


def _phase_risk(row: dict) -> float:
    direct = _first_float(row, ("phase_risk", "risk_score", "risk_proxy"))
    if direct is not None:
        return clip01(direct)
    phase = str(row.get("task_phase") or row.get("phase") or "").strip().lower()
    prior = PHASE_RISK_PRIORS.get(phase, 0.30)
    precision = _first_float(row, ("precision_mode",))
    if precision is not None and precision > 0.0:
        prior += 0.10
    return clip01(prior)


def _communication_uncertainty(row: dict, normalizers: dict) -> float:
    direct = as_float(row, "communication_uncertainty")
    if direct is not None:
        return clip01(direct)

    delay_injected = as_float(row, "delay_injected_ms") or 0.0
    jitter = as_float(row, "profile_jitter_ms") or 0.0
    loss = as_float(row, "profile_loss_rate") or 0.0
    burst = as_float(row, "profile_burst_loss_rate") or 0.0
    loss_injected = as_float(row, "loss_injected") or 0.0
    profile = str(row.get("link_profile") or row.get("communication_profile_mode") or "").strip().lower()
    prior = PROFILE_UNCERTAINTY_PRIORS.get(profile, 0.0)
    delay_term = delay_injected / max(1e-6, float(normalizers["communication_delay_norm_ms"]))
    profile_term = jitter / 100.0 + loss * 5.0 + burst * 5.0 + min(1.0, loss_injected)
    return clip01(max(prior, delay_term, profile_term))


def compute_staleness_risk(row: dict, weights: dict | None = None, normalizers: dict | None = None) -> dict:
    weights = {**DEFAULT_WEIGHTS, **(weights or {})}
    normalizers = {**DEFAULT_NORMALIZERS, **(normalizers or {})}
    delta_net, delta_exec, delta_eff = delta_proxy_ms(row)

    normalized_delta_eff = as_float(row, "normalized_delta_eff")
    if normalized_delta_eff is None:
        normalized_delta_eff = clip01((delta_eff or 0.0) / max(1e-6, float(normalizers["delta_eff_norm_ms"])))
    else:
        normalized_delta_eff = clip01(normalized_delta_eff)

    voi = _first_float(row, ("VoI", "voi_score", "voi"))
    voi = clip01(voi if voi is not None else 0.0)

    phase_risk = _phase_risk(row)

    execution_residual = as_float(row, "execution_residual")
    if execution_residual is None:
        execution_residual = clip01((delta_exec or 0.0) / max(1e-6, float(normalizers["delta_exec_norm_ms"])))
    else:
        execution_residual = clip01(execution_residual)

    communication_uncertainty = _communication_uncertainty(row, normalizers)

    action_correction_uncertainty = as_float(row, "action_correction_uncertainty")
    if action_correction_uncertainty is None:
        action_correction_uncertainty = _first_float(
            row,
            ("target_uncertainty", "vision_uncertainty", "qr_uncertainty", "correction_uncertainty"),
        )
    action_correction_uncertainty = clip01(action_correction_uncertainty if action_correction_uncertainty is not None else 0.0)

    s_eff = (
        float(weights["w_delta"]) * normalized_delta_eff
        + float(weights["w_V"]) * voi
        + float(weights["w_R"]) * phase_risk
        + float(weights["w_E"]) * execution_residual
        + float(weights["w_C"]) * communication_uncertainty
        + float(weights["w_A"]) * action_correction_uncertainty
    )
    f_eff = 1.0 - clip01(s_eff)

    return {
        "delta_net_proxy_ms": delta_net,
        "delta_exec_proxy_ms": delta_exec,
        "delta_eff_proxy_ms": delta_eff,
        "normalized_delta_eff": normalized_delta_eff,
        "VoI": voi,
        "phase_risk": phase_risk,
        "execution_residual": execution_residual,
        "communication_uncertainty": communication_uncertainty,
        "action_correction_uncertainty": action_correction_uncertainty,
        "S_eff": s_eff,
        "F_eff": f_eff,
    }


def format_staleness_risk_row(values: dict) -> dict:
    out = {}
    for key, value in values.items():
        if value is None:
            out[key] = ""
        else:
            out[key] = f"{float(value):.6f}"
    return out
