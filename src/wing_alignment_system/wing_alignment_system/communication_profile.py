# -*- coding: utf-8 -*-

ALLOWED_COMMUNICATION_PROFILE_MODES = {
    "wifi_nominal",
    "wifi_degraded",
    "fiveg_profile",
    "emulated_delay_jitter_loss",
    "executor_backlog",
    "combined_degraded",
}


def _declare_str(node, name: str, default: str) -> str:
    return str(node.declare_parameter(name, default).value).strip()


def _declare_float(node, name: str, default: float) -> float:
    return float(node.declare_parameter(name, float(default)).value)


def declare_communication_profile(node) -> dict:
    """Read logging-only communication profile parameters.

    These parameters are intentionally not used by controllers. They describe
    the run profile for offline/replay analysis and paper evidence boundaries.
    """
    mode = _declare_str(node, "communication_profile.mode", "wifi_nominal") or "wifi_nominal"
    profile_source = _declare_str(node, "communication_profile.profile_source", "configured") or "configured"
    delay_ms_mean = _declare_float(node, "communication_profile.delay_ms_mean", 0.0)
    jitter_ms = _declare_float(node, "communication_profile.jitter_ms", 0.0)
    loss_rate = _declare_float(node, "communication_profile.loss_rate", 0.0)
    burst_loss_rate = _declare_float(node, "communication_profile.burst_loss_rate", 0.0)
    executor_delay_ms = _declare_float(node, "communication_profile.executor_delay_ms", 0.0)
    notes = _declare_str(
        node,
        "communication_profile.notes",
        "Configured profile for logging and offline/replay experiments; not a claim of measured 5G/Wi-Fi performance.",
    )

    legacy_link_profile = _declare_str(node, "link_profile", "")
    legacy_profile_source = _declare_str(node, "profile_source", "")
    if legacy_link_profile and legacy_link_profile != "unspecified":
        mode = legacy_link_profile
    if legacy_profile_source and legacy_profile_source != "assumed":
        profile_source = legacy_profile_source

    if mode not in ALLOWED_COMMUNICATION_PROFILE_MODES:
        logger = getattr(node, "get_logger", lambda: None)()
        if logger is not None:
            logger.warn(
                f"[COMM_PROFILE] invalid mode={mode}; fallback to wifi_nominal"
            )
        mode = "wifi_nominal"

    return {
        "mode": mode,
        "profile_source": profile_source,
        "delay_ms_mean": delay_ms_mean,
        "jitter_ms": jitter_ms,
        "loss_rate": loss_rate,
        "burst_loss_rate": burst_loss_rate,
        "executor_delay_ms": executor_delay_ms,
        "notes": notes,
    }


COMMUNICATION_PROFILE_CSV_FIELDS = [
    "communication_profile_mode",
    "link_profile",
    "profile_source",
    "profile_delay_ms_mean",
    "profile_jitter_ms",
    "profile_loss_rate",
    "profile_burst_loss_rate",
    "profile_executor_delay_ms",
    "profile_notes",
]


def communication_profile_csv_row(profile: dict) -> dict:
    mode = str(profile.get("mode", "wifi_nominal"))
    return {
        "communication_profile_mode": mode,
        "link_profile": mode,
        "profile_source": str(profile.get("profile_source", "configured")),
        "profile_delay_ms_mean": float(profile.get("delay_ms_mean", 0.0)),
        "profile_jitter_ms": float(profile.get("jitter_ms", 0.0)),
        "profile_loss_rate": float(profile.get("loss_rate", 0.0)),
        "profile_burst_loss_rate": float(profile.get("burst_loss_rate", 0.0)),
        "profile_executor_delay_ms": float(profile.get("executor_delay_ms", 0.0)),
        "profile_notes": str(profile.get("notes", "")),
    }
