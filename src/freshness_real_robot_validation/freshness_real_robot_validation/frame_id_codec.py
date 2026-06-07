from __future__ import annotations


def encode_validation_frame_id(
    *,
    seq_id: int,
    transmission_mode: str,
    payload_bytes: int,
    method_id: str,
    task_phase: str = "",
    task_progress: float = 0.0,
    execution_mode: str = "normal",
    aoi_ms: float | None = None,
    effective_freshness: float | None = None,
) -> str:
    encoded = (
        f"seq={int(seq_id)}|tx={str(transmission_mode)}|bytes={int(payload_bytes)}"
        f"|method={str(method_id)}|phase={str(task_phase)}|progress={float(task_progress):.6f}"
        f"|exec={str(execution_mode)}"
    )
    if aoi_ms is not None:
        encoded += f"|aoi={float(aoi_ms):.3f}"
    if effective_freshness is not None:
        encoded += f"|eff={float(effective_freshness):.6f}"
    return encoded


def decode_validation_frame_id(frame_id: str) -> dict:
    raw = str(frame_id or "").strip()
    if not raw:
        return {
            "seq_id": 0,
            "transmission_mode": "full_update",
            "payload_bytes": 256,
            "method_id": "unknown",
            "task_phase": "",
            "task_progress": 0.0,
            "execution_mode": "normal",
            "aoi_ms": None,
            "effective_freshness": None,
        }
    if raw.isdigit():
        return {
            "seq_id": int(raw),
            "transmission_mode": "full_update",
            "payload_bytes": 256,
            "method_id": "unknown",
            "task_phase": "",
            "task_progress": 0.0,
            "execution_mode": "normal",
            "aoi_ms": None,
            "effective_freshness": None,
        }

    result = {
        "seq_id": 0,
        "transmission_mode": "full_update",
        "payload_bytes": 256,
        "method_id": "unknown",
        "task_phase": "",
        "task_progress": 0.0,
        "execution_mode": "normal",
        "aoi_ms": None,
        "effective_freshness": None,
    }
    for item in raw.split("|"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        if key == "seq":
            result["seq_id"] = int(value or 0)
        elif key == "tx":
            result["transmission_mode"] = value or "full_update"
        elif key == "bytes":
            result["payload_bytes"] = int(value or 256)
        elif key == "method":
            result["method_id"] = value or "unknown"
        elif key == "phase":
            result["task_phase"] = value or ""
        elif key == "progress":
            result["task_progress"] = float(value or 0.0)
        elif key == "exec":
            result["execution_mode"] = value or "normal"
        elif key == "aoi":
            result["aoi_ms"] = float(value) if value else None
        elif key == "eff":
            result["effective_freshness"] = float(value) if value else None
    return result
