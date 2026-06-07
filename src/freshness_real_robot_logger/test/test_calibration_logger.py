from __future__ import annotations

from pathlib import Path

import rclpy

from freshness_real_robot_logger.calibration_logger import CalibrationLoggerNode


def test_logger_node_initializes_with_use_sim_time_configuration(tmp_path: Path):
    config = {
        "ros": {
            "domain_id": 36,
            "node_name": "real_robot_network_calibration_logger_test",
            "use_sim_time": False,
        },
        "output": {
            "csv_path": str(tmp_path / "sample.csv"),
            "summary_json_path": str(tmp_path / "summary.json"),
            "flush_every_n_rows": 1,
        },
        "time_sync": {
            "assume_synchronized_clocks": False,
            "max_reasonable_one_way_delay_ms": 5000.0,
            "record_delay_as_proxy_when_unsynced": True,
        },
        "freshness": {
            "default_tau_ms": 1000.0,
            "phase_tau_ms": {
                "dispatch": 1000.0,
            },
        },
        "topics": [
            {
                "topic_name": "/calib/test_status",
                "msg_type": "std_msgs/msg/String",
                "robot_id": "robot_1",
                "peer_id": "base",
                "seq_source": "payload_json.seq_id",
                "sender_timestamp_source": "payload_json.sender_timestamp",
                "msg_size_source": "serialized",
                "qos": {
                    "reliability": "BEST_EFFORT",
                    "durability": "VOLATILE",
                    "history": "KEEP_LAST",
                    "depth": 10,
                },
            }
        ],
    }

    rclpy.init(args=[])
    node = None
    try:
        node = CalibrationLoggerNode(config)
        assert node.has_parameter("use_sim_time")
        assert node.subscriber_ready_at is not None
        assert node.topic_qos_profiles["/calib/test_status"]["reliability"] == "BEST_EFFORT"
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
