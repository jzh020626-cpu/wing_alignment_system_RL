from __future__ import annotations

from freshness_real_robot_validation.frame_id_codec import decode_validation_frame_id
from freshness_real_robot_validation.observe_only_traffic import build_synthetic_frame_spec


def test_synthetic_frame_spec_marks_observe_only_traffic():
    spec = build_synthetic_frame_spec(
        seq_id=7,
        robot_name="tracer1",
        task_phase="standby",
        task_progress=0.0,
        payload_bytes=96,
    )

    decoded = decode_validation_frame_id(spec["frame_id"])

    assert spec["source_mode"] == "synthetic_observe_only"
    assert decoded["seq_id"] == 7
    assert decoded["transmission_mode"] == "synthetic_heartbeat"
    assert decoded["payload_bytes"] == 96
    assert decoded["method_id"] == "SYNTHETIC_OBSERVE_ONLY"
