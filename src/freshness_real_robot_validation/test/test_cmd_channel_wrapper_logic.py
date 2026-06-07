from __future__ import annotations

import random

from freshness_real_robot_validation.cmd_channel_wrapper_logic import (
    ChannelWrapperEngine,
    MessageEnvelope,
)
from freshness_real_robot_validation.frame_id_codec import decode_validation_frame_id, encode_validation_frame_id


def _envelope(*, robot_id: str = "tracer1", task_phase: str = "transport", task_progress: float = 0.55) -> MessageEnvelope:
    return MessageEnvelope(
        robot_id=robot_id,
        seq_id=17,
        sender_timestamp=123456789,
        payload_bytes=256,
        task_phase=task_phase,
        task_progress=task_progress,
        scenario_id="real-main",
        method_id="FR-TPO",
    )


def test_observe_mode_passthrough_keeps_single_forward_and_zero_retry():
    engine = ChannelWrapperEngine(
        scenario_id="real-nominal",
        wrapper_mode="observe",
        delay_ms_mean=0.0,
        jitter_ms=0.0,
        loss_rate=0.0,
        burst_loss_rate=0.0,
        duplicate_on_critical=False,
        rng=random.Random(7),
    )

    plan = engine.build_plan(_envelope())

    assert plan.forward_count == 1
    assert plan.retry_count == 0
    assert plan.delivery_expected is True
    assert plan.delay_ms >= 0.0
    assert plan.metadata["source_mode"] == "wrapped_cmd_vel_stamped"
    assert plan.metadata["scenario_id"] == "real-nominal"
    assert plan.metadata["method_id"] == "FR-TPO"


def test_combined_degraded_duplicates_critical_phase_messages_without_touching_execution_mode():
    engine = ChannelWrapperEngine(
        scenario_id="real-boundary-onset",
        wrapper_mode="active",
        delay_ms_mean=40.0,
        jitter_ms=5.0,
        loss_rate=0.0,
        burst_loss_rate=0.0,
        duplicate_on_critical=True,
        rng=random.Random(11),
    )

    plan = engine.build_plan(_envelope(task_phase="narrow_passage", task_progress=0.65))

    assert plan.forward_count == 2
    assert plan.retry_count == 1
    assert plan.metadata["task_phase"] == "narrow_passage"
    assert plan.metadata["payload_bytes"] == 256
    assert plan.metadata["execution_mode"] == "normal"


def test_loss_mode_can_drop_message_and_reports_non_delivery():
    engine = ChannelWrapperEngine(
        scenario_id="real-main",
        wrapper_mode="active",
        delay_ms_mean=25.0,
        jitter_ms=0.0,
        loss_rate=1.0,
        burst_loss_rate=0.0,
        duplicate_on_critical=False,
        rng=random.Random(3),
    )

    plan = engine.build_plan(_envelope())

    assert plan.forward_count == 0
    assert plan.delivery_expected is False
    assert plan.retry_count == 1


def test_validation_frame_id_codec_preserves_phase_and_progress_metadata():
    encoded = encode_validation_frame_id(
        seq_id=23,
        transmission_mode="compact_update",
        payload_bytes=128,
        method_id="FR-TPO",
        task_phase="transport",
        task_progress=0.42,
        execution_mode="degraded",
        aoi_ms=210.5,
        effective_freshness=0.88,
    )

    decoded = decode_validation_frame_id(encoded)

    assert decoded["seq_id"] == 23
    assert decoded["transmission_mode"] == "compact_update"
    assert decoded["payload_bytes"] == 128
    assert decoded["method_id"] == "FR-TPO"
    assert decoded["task_phase"] == "transport"
    assert decoded["task_progress"] == 0.42
    assert decoded["execution_mode"] == "degraded"
    assert decoded["aoi_ms"] == 210.5
    assert decoded["effective_freshness"] == 0.88
