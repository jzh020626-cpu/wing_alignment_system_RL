from __future__ import annotations

import math
from typing import Mapping


def resolve_tau_ms(phase: str | None, default_tau_ms: float, phase_tau_ms: Mapping[str, float] | None) -> float:
    if phase and phase_tau_ms and phase in phase_tau_ms:
        return float(phase_tau_ms[phase])
    return float(default_tau_ms)


def compute_aoi_and_effective_freshness(
    *,
    sender_timestamp_ns: int | None,
    receiver_timestamp_ns: int | None,
    phase: str | None,
    default_tau_ms: float,
    phase_tau_ms: Mapping[str, float] | None,
) -> dict[str, float | None]:
    tau_ms = resolve_tau_ms(phase, default_tau_ms, phase_tau_ms)
    if sender_timestamp_ns is None or receiver_timestamp_ns is None:
        return {"AoI_ms": None, "Effective_Freshness": None, "tau_ms": tau_ms}
    aoi_ms = max(float(receiver_timestamp_ns - sender_timestamp_ns) / 1_000_000.0, 0.0)
    effective = math.exp(-aoi_ms / max(tau_ms, 1e-9))
    return {
        "AoI_ms": aoi_ms,
        "Effective_Freshness": effective,
        "tau_ms": tau_ms,
    }
