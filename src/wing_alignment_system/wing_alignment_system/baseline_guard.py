# -*- coding: utf-8 -*-
"""Logging-only guard for unsafe freshness baselines.

This module does not implement scheduler, watchdog, or mission behavior
switches. It only rejects unsafe baseline requests before hardware execution.
"""

from __future__ import annotations


class UnsafeBaselineError(RuntimeError):
    pass


ALLOWED_UNSAFE_ENVIRONMENTS = {
    "offline",
    "replay",
    "sim",
    "simulation",
    "synthetic",
    "bench",
}

BENCH_REPLAY_ENVIRONMENTS = {
    "offline",
    "replay",
    "sim",
    "simulation",
    "synthetic",
    "bench",
}

DANGEROUS_MODE_TOKENS = {
    "watchdog_off",
    "off",
    "fixed_periodic",
    "fixed_periodic_stale_pass_through",
    "stale_pass_through",
    "dangerous_baseline",
    "b1",
}

BASELINE_MODE_METADATA = {
    "current_safe_default": {
        "baseline_candidate_status": "safe_default",
        "baseline_candidate_note": "current conservative scheduler/watchdog/mission behavior",
        "watchdog_required": True,
    },
    "freshness_aware_transmission_only": {
        "baseline_candidate_status": "transmission_only",
        "baseline_candidate_note": "bench/replay transmission-only entry; watchdog remains enabled",
        "watchdog_required": True,
    },
    "full_method_candidate": {
        "baseline_candidate_status": "candidate_not_yet_final",
        "baseline_candidate_note": "bench/replay candidate; authority behavior not yet final",
        "watchdog_required": True,
    },
}

BASELINE_GUARD_CSV_FIELDS = [
    "allow_unsafe_baseline",
    "baseline_mode",
    "scheduler_mode",
    "watchdog_mode",
    "baseline_execution_environment",
    "unsafe_baseline_requested",
    "baseline_guard_status",
    "baseline_guard_reason",
    "baseline_candidate_status",
    "baseline_candidate_note",
    "watchdog_required",
]


def _norm(value, default: str) -> str:
    text = str(default if value in ("", None) else value).strip().lower()
    return text.replace("-", "_").replace(" ", "_")


def _bool(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _dangerous_values(values: tuple[str, ...]) -> list[str]:
    dangerous = []
    for value in values:
        mode = _norm(value, "")
        if not mode:
            continue
        if mode in DANGEROUS_MODE_TOKENS:
            dangerous.append(mode)
            continue
        if "watchdog_off" in mode or "stale_pass_through" in mode or "dangerous" in mode:
            dangerous.append(mode)
            continue
        if "fixed_periodic" in mode:
            dangerous.append(mode)
    return dangerous


def check_baseline_guard(
    allow_unsafe_baseline=False,
    baseline_mode: str = "current_safe_default",
    scheduler_mode: str = "current_hybrid",
    watchdog_mode: str = "decay_then_stop",
    execution_environment: str = "hardware",
) -> dict:
    allow = _bool(allow_unsafe_baseline)
    baseline = _norm(baseline_mode, "current_safe_default")
    scheduler = _norm(scheduler_mode, "current_hybrid")
    watchdog = _norm(watchdog_mode, "decay_then_stop")
    environment = _norm(execution_environment, "hardware")
    dangerous = _dangerous_values((baseline, scheduler, watchdog))
    metadata = BASELINE_MODE_METADATA.get(baseline)

    if metadata is None and not dangerous:
        raise UnsafeBaselineError(f"unsupported baseline_mode: {baseline}")

    if baseline == "full_method_candidate" and environment not in BENCH_REPLAY_ENVIRONMENTS:
        raise UnsafeBaselineError(
            "full_method_candidate is a bench/replay candidate entry, not a hardware baseline"
        )

    if dangerous and environment not in ALLOWED_UNSAFE_ENVIRONMENTS:
        raise UnsafeBaselineError(
            "unsafe baseline modes are limited to offline/replay/sim/bench: "
            + ",".join(dangerous)
        )
    if dangerous and not allow:
        raise UnsafeBaselineError(
            "unsafe baseline requested but allow_unsafe_baseline is false: "
            + ",".join(dangerous)
        )

    return {
        "allow_unsafe_baseline": allow,
        "baseline_mode": baseline,
        "scheduler_mode": scheduler,
        "watchdog_mode": watchdog,
        "baseline_execution_environment": environment,
        "unsafe_baseline_requested": bool(dangerous),
        "baseline_guard_status": "allowed_unsafe_non_hardware" if dangerous else "safe_default",
        "baseline_guard_reason": ",".join(dangerous),
        "baseline_candidate_status": str((metadata or {}).get("baseline_candidate_status", "unsafe")),
        "baseline_candidate_note": str((metadata or {}).get("baseline_candidate_note", "")),
        "watchdog_required": bool((metadata or {}).get("watchdog_required", False)),
    }


def baseline_guard_csv_row(guard: dict) -> dict:
    return {
        "allow_unsafe_baseline": int(bool(guard.get("allow_unsafe_baseline", False))),
        "baseline_mode": str(guard.get("baseline_mode", "current_safe_default")),
        "scheduler_mode": str(guard.get("scheduler_mode", "current_hybrid")),
        "watchdog_mode": str(guard.get("watchdog_mode", "decay_then_stop")),
        "baseline_execution_environment": str(guard.get("baseline_execution_environment", "hardware")),
        "unsafe_baseline_requested": int(bool(guard.get("unsafe_baseline_requested", False))),
        "baseline_guard_status": str(guard.get("baseline_guard_status", "safe_default")),
        "baseline_guard_reason": str(guard.get("baseline_guard_reason", "")),
        "baseline_candidate_status": str(guard.get("baseline_candidate_status", "safe_default")),
        "baseline_candidate_note": str(guard.get("baseline_candidate_note", "")),
        "watchdog_required": int(bool(guard.get("watchdog_required", True))),
    }


def declare_baseline_guard(node, node_role: str = "") -> dict:
    allow = node.declare_parameter("allow_unsafe_baseline", False).value
    baseline_mode = node.declare_parameter("baseline_mode", "current_safe_default").value
    scheduler_mode = node.declare_parameter("scheduler_mode", "current_hybrid").value
    watchdog_mode = node.declare_parameter("watchdog_mode", "decay_then_stop").value
    environment = node.declare_parameter("baseline_execution_environment", "hardware").value
    try:
        guard = check_baseline_guard(
            allow_unsafe_baseline=allow,
            baseline_mode=baseline_mode,
            scheduler_mode=scheduler_mode,
            watchdog_mode=watchdog_mode,
            execution_environment=environment,
        )
    except UnsafeBaselineError as exc:
        logger = getattr(node, "get_logger", lambda: None)()
        if logger is not None:
            prefix = f"[{node_role}] " if node_role else ""
            logger.error(prefix + str(exc))
        raise
    return guard
