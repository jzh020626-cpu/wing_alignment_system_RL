from __future__ import annotations

from freshness_real_robot_validation.wrapper_metadata import TRACE_FIELDS, WrapperMetadataTracker


def test_missing_source_stamp_still_gets_receive_timestamp_and_delay_stays_na():
    tracker = WrapperMetadataTracker(receiver_clock_type="rclpy_node_clock")

    row = tracker.build_row(
        seq_id=1,
        payload_type="geometry_msgs/msg/TwistStamped",
        payload_bytes=96,
        source_send_timestamp_ns=None,
        source_clock_type="n/a",
        wrapper_receive_timestamp_ns=1_000_000_000,
        transmission_mode="synthetic_heartbeat",
        phase="standby",
        task_progress=0.0,
        retry_count=0,
        execution_mode="normal",
        aoi_ms=None,
        effective_freshness=None,
        deadline_met="n/a",
        scenario_id="observe-only",
        method_id="SYNTHETIC_OBSERVE_ONLY",
        robot_id="tracer1",
        source_mode="synthetic_observe_only",
        delivery_expected=True,
        wrapper_mode="observe",
        wrapper_emit_timestamp_ns=None,
        allow_true_one_way_delay=False,
    )

    assert row["wrapper_receive_timestamp"] == 1_000_000_000
    assert row["receiver_node_time_ns"] == 1_000_000_000
    assert row["source_send_timestamp"] == "n/a"
    assert row["true_one_way_delay_ms"] == "n/a"
    assert row["receiver_side_aoi_proxy_ms"] == "n/a"


def test_inter_arrival_and_proxy_are_computed_and_seq_is_monotonic():
    tracker = WrapperMetadataTracker(receiver_clock_type="rclpy_node_clock")

    first = tracker.build_row(
        seq_id=10,
        payload_type="geometry_msgs/msg/TwistStamped",
        payload_bytes=96,
        source_send_timestamp_ns=900_000_000,
        source_clock_type="message_header_stamp",
        wrapper_receive_timestamp_ns=1_000_000_000,
        transmission_mode="synthetic_heartbeat",
        phase="standby",
        task_progress=0.0,
        retry_count=0,
        execution_mode="normal",
        aoi_ms=100.0,
        effective_freshness=0.9,
        deadline_met="n/a",
        scenario_id="observe-only",
        method_id="SYNTHETIC_OBSERVE_ONLY",
        robot_id="tracer1",
        source_mode="synthetic_observe_only",
        delivery_expected=True,
        wrapper_mode="observe",
        wrapper_emit_timestamp_ns=None,
        allow_true_one_way_delay=False,
    )
    second = tracker.build_row(
        seq_id=11,
        payload_type="geometry_msgs/msg/TwistStamped",
        payload_bytes=96,
        source_send_timestamp_ns=940_000_000,
        source_clock_type="message_header_stamp",
        wrapper_receive_timestamp_ns=1_050_000_000,
        transmission_mode="synthetic_heartbeat",
        phase="standby",
        task_progress=0.0,
        retry_count=0,
        execution_mode="degraded",
        aoi_ms=110.0,
        effective_freshness=0.82,
        deadline_met="n/a",
        scenario_id="observe-only",
        method_id="SYNTHETIC_OBSERVE_ONLY",
        robot_id="tracer1",
        source_mode="synthetic_observe_only",
        delivery_expected=True,
        wrapper_mode="observe",
        wrapper_emit_timestamp_ns=None,
        allow_true_one_way_delay=False,
    )

    assert first["seq_monotonic"] is True
    assert second["seq_monotonic"] is True
    assert second["inter_arrival_ms"] == 50.0
    assert second["receiver_side_aoi_proxy_ms"] == 110.0
    assert second["true_one_way_delay_ms"] == "n/a"
    assert second["execution_mode"] == "degraded"
    assert second["aoi_ms"] == 110.0
    assert second["effective_freshness"] == 0.82


def test_trace_fields_include_required_proxy_and_timestamp_columns():
    required = {
        "seq_id",
        "payload_type",
        "payload_bytes",
        "source_send_timestamp",
        "wrapper_receive_timestamp",
        "wrapper_emit_timestamp",
        "inter_arrival_ms",
        "receiver_side_aoi_proxy_ms",
        "true_one_way_delay_ms",
        "deadline_met",
        "retry_count",
        "transmission_mode",
        "execution_mode",
        "aoi_ms",
        "effective_freshness",
        "phase",
        "task_progress",
    }

    assert required.issubset(set(TRACE_FIELDS))
