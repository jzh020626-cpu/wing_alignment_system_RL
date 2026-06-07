from __future__ import annotations

import argparse
import json
import os
import signal
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from freshness_real_robot_logger.config_loader import load_logger_config
from freshness_real_robot_logger.csv_writer import CalibrationCsvWriter
from freshness_real_robot_logger.freshness_metrics import compute_aoi_and_effective_freshness
from freshness_real_robot_logger.msg_extractors import extract_message_fields, resolve_message_class
from freshness_real_robot_logger.topic_state import TopicState


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat()


def _format_optional(value: Any, *, digits: int = 6) -> Any:
    if value is None:
        return None
    if isinstance(value, float):
        return round(value, digits)
    return value


def _normalize_qos_token(value: Any, *, default: str) -> str:
    if value is None:
        return default
    return str(value).strip().upper()


def _history_policy(name: str) -> HistoryPolicy:
    if name == "KEEP_ALL":
        return HistoryPolicy.KEEP_ALL
    return HistoryPolicy.KEEP_LAST


def _reliability_policy(name: str) -> ReliabilityPolicy:
    if name == "BEST_EFFORT":
        return ReliabilityPolicy.BEST_EFFORT
    return ReliabilityPolicy.RELIABLE


def _durability_policy(name: str) -> DurabilityPolicy:
    if name == "TRANSIENT_LOCAL":
        return DurabilityPolicy.TRANSIENT_LOCAL
    return DurabilityPolicy.VOLATILE


def _topic_qos_profile(topic_cfg: dict[str, Any]) -> QoSProfile:
    qos_cfg = dict(topic_cfg.get("qos") or {})
    history_name = _normalize_qos_token(qos_cfg.get("history"), default="KEEP_LAST")
    reliability_name = _normalize_qos_token(qos_cfg.get("reliability"), default="RELIABLE")
    durability_name = _normalize_qos_token(qos_cfg.get("durability"), default="VOLATILE")
    depth = max(int(qos_cfg.get("depth", 10)), 1)
    return QoSProfile(
        history=_history_policy(history_name),
        depth=depth,
        reliability=_reliability_policy(reliability_name),
        durability=_durability_policy(durability_name),
    )


def _qos_profile_summary(profile: QoSProfile) -> dict[str, Any]:
    return {
        "history": profile.history.name,
        "depth": int(profile.depth),
        "reliability": profile.reliability.name,
        "durability": profile.durability.name,
    }


class CalibrationLoggerNode(Node):
    def __init__(self, config: dict[str, Any], *, shutdown_after_inactive_sec: float | None = None):
        super().__init__(str(config["ros"]["node_name"]))
        self.config = config
        self.shutdown_after_inactive_sec = shutdown_after_inactive_sec
        self.warning_counter: Counter[str] = Counter()
        self.rows_written = 0
        self.start_wall_ns = time.time_ns()
        self.last_activity_ns = self.start_wall_ns
        self._finalized = False
        self.topic_states: dict[str, TopicState] = {}
        self.topic_configs: dict[str, dict[str, Any]] = {}
        self.fallback_seq_ids: dict[str, int] = {}
        self.topic_qos_profiles: dict[str, dict[str, Any]] = {}
        self.subscriber_ready_at: str | None = None
        self.summary_json_path = Path(config["output"]["summary_json_path"])
        self.summary_markdown_path = self.summary_json_path.with_suffix(".md")
        self.csv_writer = CalibrationCsvWriter(
            config["output"]["csv_path"],
            flush_every_n_rows=int(config["output"]["flush_every_n_rows"]),
        )
        use_sim_time = bool(config["ros"].get("use_sim_time", False))
        if self.has_parameter("use_sim_time"):
            self.set_parameters([Parameter("use_sim_time", value=use_sim_time)])
        else:
            self.declare_parameter("use_sim_time", use_sim_time)
        for topic_cfg in config["topics"]:
            topic_name = str(topic_cfg["topic_name"])
            message_class = resolve_message_class(str(topic_cfg["msg_type"]))
            state = TopicState(
                topic_name=topic_name,
                robot_id=str(topic_cfg["robot_id"]),
                peer_id=str(topic_cfg["peer_id"]),
            )
            self.topic_states[topic_name] = state
            self.topic_configs[topic_name] = topic_cfg
            self.fallback_seq_ids[topic_name] = 0
            qos_profile = _topic_qos_profile(topic_cfg)
            self.topic_qos_profiles[topic_name] = _qos_profile_summary(qos_profile)
            self.create_subscription(message_class, topic_name, self._build_callback(topic_name), qos_profile)
        self.subscriber_ready_at = _now_iso()
        self.get_logger().info(
            f"subscriber_ready timestamp={self.subscriber_ready_at} subscriptions={len(self.topic_states)}"
        )
        if self.shutdown_after_inactive_sec is not None:
            self.create_timer(0.5, self._check_inactivity)

    def _build_callback(self, topic_name: str):
        def _callback(msg: Any) -> None:
            self._handle_message(topic_name, msg)

        return _callback

    def _next_fallback_seq_id(self, topic_name: str) -> int:
        value = self.fallback_seq_ids[topic_name]
        self.fallback_seq_ids[topic_name] += 1
        return value

    def _time_sync_values(
        self,
        *,
        sender_timestamp_ns: int | None,
        receiver_timestamp_ns: int,
    ) -> tuple[str, float | None, float | None, list[str]]:
        warnings: list[str] = []
        sync_cfg = self.config["time_sync"]
        if sender_timestamp_ns is None:
            warnings.append("sender_timestamp_missing")
            return ("missing_sender_timestamp", None, None, warnings)
        delta_ms = max((receiver_timestamp_ns - sender_timestamp_ns) / 1_000_000.0, 0.0)
        max_reasonable = float(sync_cfg["max_reasonable_one_way_delay_ms"])
        if bool(sync_cfg["assume_synchronized_clocks"]):
            if delta_ms > max_reasonable:
                warnings.append("one_way_delay_out_of_range")
            return ("synchronized", delta_ms, None, warnings)
        if bool(sync_cfg["record_delay_as_proxy_when_unsynced"]):
            warnings.append("unsynchronized_proxy")
            return ("unsynchronized_proxy", None, delta_ms, warnings)
        warnings.append("unsynchronized_delay_not_recorded")
        return ("unsynchronized_proxy", None, None, warnings)

    def _normalize_scalar(self, value: Any) -> Any:
        return "n/a" if value is None else value

    def _handle_message(self, topic_name: str, msg: Any) -> None:
        receive_wall_ns = time.time_ns()
        receive_ros_ns = self.get_clock().now().nanoseconds
        fallback_seq_id = self._next_fallback_seq_id(topic_name)
        topic_cfg = self.topic_configs[topic_name]
        extracted = extract_message_fields(msg, topic_cfg, fallback_seq_id=fallback_seq_id)
        time_sync_mode, one_way_delay_ms, proxy_ms, sync_warnings = self._time_sync_values(
            sender_timestamp_ns=extracted["sender_timestamp"],
            receiver_timestamp_ns=receive_wall_ns,
        )
        warnings = list(extracted["warnings"]) + sync_warnings
        state_metrics = self.topic_states[topic_name].update(
            seq_id=int(extracted["seq_id"]),
            receive_time_ns=receive_wall_ns,
            msg_size_bytes=int(extracted["msg_size_bytes"]),
            sender_timestamp_ns=extracted["sender_timestamp"],
        )
        freshness = compute_aoi_and_effective_freshness(
            sender_timestamp_ns=extracted["sender_timestamp"],
            receiver_timestamp_ns=receive_wall_ns,
            phase=None if extracted["phase"] == "n/a" else str(extracted["phase"]),
            default_tau_ms=float(self.config["freshness"]["default_tau_ms"]),
            phase_tau_ms=dict(self.config["freshness"]["phase_tau_ms"]),
        )
        if state_metrics["packet_loss_flag"]:
            warnings.append("packet_loss_gap_detected")
        row = {
            "timestamp_wall": _now_iso(),
            "timestamp_ros": int(receive_ros_ns),
            "robot_id": str(topic_cfg["robot_id"]),
            "peer_id": str(topic_cfg["peer_id"]),
            "topic_name": topic_name,
            "seq_id": int(extracted["seq_id"]),
            "msg_size_bytes": int(extracted["msg_size_bytes"]),
            "sender_timestamp": extracted["sender_timestamp"],
            "receiver_timestamp": int(receive_wall_ns),
            "one_way_delay_ms": _format_optional(one_way_delay_ms),
            "receiver_side_latency_proxy_ms": _format_optional(proxy_ms),
            "inter_arrival_ms": _format_optional(state_metrics["inter_arrival_ms"]),
            "packet_loss_flag": bool(state_metrics["packet_loss_flag"]),
            "estimated_bandwidth_kbps": _format_optional(state_metrics["estimated_bandwidth_kbps"]),
            "AoI_ms": _format_optional(freshness["AoI_ms"]),
            "Effective_Freshness": _format_optional(freshness["Effective_Freshness"]),
            "phase": extracted["phase"],
            "task_progress": self._normalize_scalar(extracted["task_progress"]),
            "control_mode": extracted["control_mode"],
            "emergency_stop": extracted["emergency_stop"],
            "fallback_flag": extracted["fallback_flag"],
            "done_reason": extracted["done_reason"],
            "time_sync_mode": time_sync_mode,
            "warning_flags": "|".join(sorted(set(warnings))) if warnings else "",
        }
        self.csv_writer.write_row(row)
        self.rows_written += 1
        self.last_activity_ns = receive_wall_ns
        for warning in warnings:
            self.warning_counter[warning] += 1

    def _check_inactivity(self) -> None:
        if self.shutdown_after_inactive_sec is None or self._finalized:
            return
        idle_sec = (time.time_ns() - self.last_activity_ns) / 1_000_000_000.0
        if self.rows_written > 0 and idle_sec >= float(self.shutdown_after_inactive_sec):
            self.get_logger().info("Shutdown after inactivity threshold reached.")
            self.finalize()
            rclpy.shutdown()

    def _summary_payload(self) -> dict[str, Any]:
        return {
            "generated_at": _now_iso(),
            "node_name": self.get_name(),
            "rows_written": self.rows_written,
            "csv_path": str(Path(self.config["output"]["csv_path"]).resolve()),
            "summary_json_path": str(self.summary_json_path.resolve()),
            "subscriber_ready_at": self.subscriber_ready_at,
            "time_sync_assume_synchronized_clocks": bool(self.config["time_sync"]["assume_synchronized_clocks"]),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", str(self.config["ros"]["domain_id"])),
            "warning_counts": dict(self.warning_counter),
            "topic_qos_profiles": self.topic_qos_profiles,
            "topics": {name: state.summary() for name, state in self.topic_states.items()},
        }

    def _write_summary_markdown(self, payload: dict[str, Any]) -> None:
        lines = [
            "# Calibration Logger Summary",
            "",
            f"- node_name: `{payload['node_name']}`",
            f"- rows_written: `{payload['rows_written']}`",
            f"- csv_path: `{payload['csv_path']}`",
            f"- ros_domain_id: `{payload['ros_domain_id']}`",
            f"- synchronized_clocks_assumed: `{payload['time_sync_assume_synchronized_clocks']}`",
            f"- subscriber_ready_at: `{payload['subscriber_ready_at']}`",
            "",
            "## Topic summaries",
            "",
        ]
        for topic_name, topic_summary in payload["topics"].items():
            lines.append(f"- {topic_name}: `{topic_summary}`")
        lines.extend(["", "## QoS profiles", ""])
        for topic_name, qos_profile in payload["topic_qos_profiles"].items():
            lines.append(f"- {topic_name}: `{qos_profile}`")
        lines.extend(["", "## Warning counts", "", f"- `{payload['warning_counts']}`", ""])
        self.summary_markdown_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_markdown_path.write_text("\n".join(lines), encoding="utf-8")

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.csv_writer.flush()
        payload = self._summary_payload()
        self.summary_json_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        self._write_summary_markdown(payload)
        self.csv_writer.close()


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--shutdown-after-inactive-sec", type=float, default=None)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_argument_parser().parse_args(argv)
    config = load_logger_config(args.config)
    os.environ.setdefault("ROS_DOMAIN_ID", str(config["ros"]["domain_id"]))
    rclpy.init(args=[])
    node = CalibrationLoggerNode(config, shutdown_after_inactive_sec=args.shutdown_after_inactive_sec)

    def _handle_signal(signum, _frame):
        del signum
        node.finalize()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
