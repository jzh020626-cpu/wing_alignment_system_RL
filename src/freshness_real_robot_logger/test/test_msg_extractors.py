from __future__ import annotations

from std_msgs.msg import String

from freshness_real_robot_logger.msg_extractors import extract_message_fields


def _string_message(payload: str) -> String:
    msg = String()
    msg.data = payload
    return msg


def test_string_json_payload_is_parsed():
    msg = _string_message(
        '{"seq_id": 7, "sender_timestamp": 1234567890, "phase": "dispatch", "task_progress": 0.2, "control_mode": "manual", "emergency_stop": false, "fallback_flag": false, "done_reason": "running"}'
    )
    topic_cfg = {
        "seq_source": "payload_json.seq_id",
        "sender_timestamp_source": "payload_json.sender_timestamp",
        "phase_source": "payload_json.phase",
        "task_progress_source": "payload_json.task_progress",
        "control_mode_source": "payload_json.control_mode",
        "emergency_stop_source": "payload_json.emergency_stop",
        "fallback_flag_source": "payload_json.fallback_flag",
        "done_reason_source": "payload_json.done_reason",
        "msg_size_source": "serialized",
    }

    extracted = extract_message_fields(msg, topic_cfg, fallback_seq_id=99)

    assert extracted["seq_id"] == 7
    assert extracted["sender_timestamp"] == 1234567890
    assert extracted["phase"] == "dispatch"
    assert extracted["task_progress"] == 0.2
    assert extracted["control_mode"] == "manual"
    assert extracted["emergency_stop"] is False
    assert extracted["fallback_flag"] is False
    assert extracted["done_reason"] == "running"
    assert extracted["warnings"] == []
    assert extracted["msg_size_bytes"] > 0


def test_missing_fields_do_not_crash():
    msg = _string_message('{"phase": "dispatch"}')
    topic_cfg = {
        "seq_source": "payload_json.seq_id",
        "sender_timestamp_source": "payload_json.sender_timestamp",
        "msg_size_source": "serialized",
    }

    extracted = extract_message_fields(msg, topic_cfg, fallback_seq_id=5)

    assert extracted["seq_id"] == 5
    assert extracted["sender_timestamp"] is None
    assert extracted["warnings"]


def test_malformed_json_does_not_crash_and_records_warning():
    msg = _string_message("{not-json}")
    topic_cfg = {
        "seq_source": "payload_json.seq_id",
        "sender_timestamp_source": "payload_json.sender_timestamp",
        "msg_size_source": "serialized",
    }

    extracted = extract_message_fields(msg, topic_cfg, fallback_seq_id=3)

    assert extracted["seq_id"] == 3
    assert extracted["sender_timestamp"] is None
    assert any("payload_json_parse_error" in warning for warning in extracted["warnings"])
