from __future__ import annotations

from freshness_real_robot_logger.topic_state import TopicState


def test_seq_id_without_gap_has_no_loss():
    state = TopicState(topic_name="/demo", robot_id="r1", peer_id="base")

    first = state.update(
        seq_id=1,
        receive_time_ns=1_000_000_000,
        msg_size_bytes=100,
        sender_timestamp_ns=900_000_000,
    )
    second = state.update(
        seq_id=2,
        receive_time_ns=1_100_000_000,
        msg_size_bytes=100,
        sender_timestamp_ns=1_000_000_000,
    )

    assert first["packet_loss_flag"] is False
    assert second["packet_loss_flag"] is False
    assert state.missing_count == 0


def test_seq_id_gap_sets_packet_loss_flag():
    state = TopicState(topic_name="/demo", robot_id="r1", peer_id="base")
    state.update(seq_id=1, receive_time_ns=1_000_000_000, msg_size_bytes=100, sender_timestamp_ns=900_000_000)
    result = state.update(seq_id=4, receive_time_ns=1_100_000_000, msg_size_bytes=100, sender_timestamp_ns=1_000_000_000)

    assert result["packet_loss_flag"] is True
    assert state.missing_count == 2
    assert state.loss_events == 1


def test_inter_arrival_and_bandwidth_are_computable():
    state = TopicState(topic_name="/demo", robot_id="r1", peer_id="base")
    state.update(seq_id=1, receive_time_ns=1_000_000_000, msg_size_bytes=200, sender_timestamp_ns=900_000_000)
    result = state.update(seq_id=2, receive_time_ns=1_250_000_000, msg_size_bytes=300, sender_timestamp_ns=1_150_000_000)

    assert result["inter_arrival_ms"] == 250.0
    assert result["estimated_bandwidth_kbps"] >= 0.0
