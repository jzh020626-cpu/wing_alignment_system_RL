from __future__ import annotations

import json

from std_msgs.msg import String


def string_message_from_payload(payload: dict) -> String:
    msg = String()
    msg.data = json.dumps(payload, ensure_ascii=True, sort_keys=True)
    return msg


def payload_from_string_message(msg: String) -> dict:
    text = str(getattr(msg, "data", "") or "").strip()
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}
