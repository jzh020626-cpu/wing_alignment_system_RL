from __future__ import annotations

import json
from typing import Any

from std_msgs.msg import String

try:
    from rclpy.serialization import serialize_message
except Exception:  # pragma: no cover - fallback only
    serialize_message = None

try:
    from rosidl_runtime_py.utilities import get_message
except Exception:  # pragma: no cover - fallback only
    get_message = None


def resolve_message_class(msg_type: str):
    if get_message is not None:
        return get_message(msg_type)
    package_name, _, tail = msg_type.partition("/msg/")
    module = __import__(f"{package_name}.msg", fromlist=[tail])
    return getattr(module, tail)


def _payload_json(msg: Any, warnings: list[str]) -> dict[str, Any] | None:
    if not isinstance(msg, String):
        return None
    try:
        return json.loads(msg.data)
    except Exception as exc:
        warnings.append(f"payload_json_parse_error:{type(exc).__name__}")
        return None


def _walk_value(root: Any, path: list[str]) -> Any:
    current = root
    for part in path:
        if current is None:
            return None
        if isinstance(current, dict):
            current = current.get(part)
        else:
            current = getattr(current, part, None)
    return current


def _stamp_to_ns(stamp: Any) -> int | None:
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def _read_source(msg: Any, source: str | None, payload: dict[str, Any] | None, warnings: list[str]) -> Any:
    if not source:
        return None
    if source == "fallback_counter":
        return None
    if source.startswith("payload_json."):
        if payload is None:
            warnings.append(f"missing_payload_json_for:{source}")
            return None
        value = _walk_value(payload, source.split(".")[1:])
        if value is None:
            warnings.append(f"payload_field_missing:{source}")
        return value
    if source == "header.stamp":
        header = getattr(msg, "header", None)
        value = _stamp_to_ns(getattr(header, "stamp", None)) if header is not None else None
        if value is None:
            warnings.append("header_stamp_missing")
        return value
    if source.startswith("header."):
        header = getattr(msg, "header", None)
        value = _walk_value(header, source.split(".")[1:]) if header is not None else None
        if value is None:
            warnings.append(f"header_field_missing:{source}")
        return value
    value = getattr(msg, source, None)
    if value is None:
        warnings.append(f"message_field_missing:{source}")
    return value


def _coerce_bool(value: Any) -> bool | str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    return "n/a"


def estimate_message_size_bytes(msg: Any, msg_size_source: str | None, warnings: list[str]) -> int:
    if msg_size_source == "serialized" and serialize_message is not None:
        try:
            return len(serialize_message(msg))
        except Exception as exc:
            warnings.append(f"serialized_size_fallback:{type(exc).__name__}")
    if isinstance(msg, String):
        return len(msg.data.encode("utf-8"))
    return len(repr(msg).encode("utf-8"))


def extract_message_fields(msg: Any, topic_cfg: dict[str, Any], fallback_seq_id: int) -> dict[str, Any]:
    warnings: list[str] = []
    payload = _payload_json(msg, warnings)
    seq_value = _read_source(msg, topic_cfg.get("seq_source"), payload, warnings)
    sender_timestamp = _read_source(msg, topic_cfg.get("sender_timestamp_source"), payload, warnings)

    try:
        seq_id = int(seq_value) if seq_value is not None else int(fallback_seq_id)
    except Exception:
        warnings.append("seq_id_parse_error")
        seq_id = int(fallback_seq_id)

    try:
        sender_timestamp_ns = int(sender_timestamp) if sender_timestamp is not None else None
    except Exception:
        warnings.append("sender_timestamp_parse_error")
        sender_timestamp_ns = None

    if seq_value is None:
        warnings.append("seq_id_fallback_counter_used")
    if sender_timestamp is None:
        warnings.append("sender_timestamp_missing")

    return {
        "seq_id": seq_id,
        "sender_timestamp": sender_timestamp_ns,
        "phase": _read_source(msg, topic_cfg.get("phase_source"), payload, warnings) or "n/a",
        "task_progress": _read_source(msg, topic_cfg.get("task_progress_source"), payload, warnings),
        "control_mode": _read_source(msg, topic_cfg.get("control_mode_source"), payload, warnings) or "n/a",
        "emergency_stop": _coerce_bool(_read_source(msg, topic_cfg.get("emergency_stop_source"), payload, warnings)),
        "fallback_flag": _coerce_bool(_read_source(msg, topic_cfg.get("fallback_flag_source"), payload, warnings)),
        "done_reason": _read_source(msg, topic_cfg.get("done_reason_source"), payload, warnings) or "n/a",
        "msg_size_bytes": estimate_message_size_bytes(msg, topic_cfg.get("msg_size_source"), warnings),
        "warnings": warnings,
    }
