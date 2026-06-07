from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "ros": {
        "domain_id": 36,
        "node_name": "real_robot_network_calibration_logger",
        "use_sim_time": False,
    },
    "output": {
        "csv_path": "artifacts/real_robot/calibration/real_robot_network_calibration_sample.csv",
        "summary_json_path": "artifacts/real_robot/calibration/real_robot_network_calibration_summary.json",
        "flush_every_n_rows": 10,
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
            "cooperative_transport": 800.0,
            "narrow_passage": 500.0,
            "final_alignment": 300.0,
            "release_exit": 1000.0,
        },
    },
    "topics": [],
}


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _validate_topic(topic: dict[str, Any], index: int) -> None:
    required = ["topic_name", "msg_type", "robot_id", "peer_id", "msg_size_source"]
    missing = [field for field in required if not topic.get(field)]
    if missing:
        raise ValueError(f"topic[{index}] missing required fields: {missing}")


def load_logger_config(path: str | Path) -> dict[str, Any]:
    cfg_path = Path(path)
    payload = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    config = _deep_merge(DEFAULT_CONFIG, payload)
    if not isinstance(config.get("topics"), list) or not config["topics"]:
        raise ValueError("config must contain at least one topic entry")
    for index, topic in enumerate(config["topics"]):
        if not isinstance(topic, dict):
            raise ValueError(f"topic[{index}] must be a mapping")
        _validate_topic(topic, index)
    return config
