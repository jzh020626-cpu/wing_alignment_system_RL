from __future__ import annotations

import copy
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.cmd_channel_wrapper_logic import ChannelWrapperEngine, MessageEnvelope
from freshness_real_robot_validation.config_loader import load_yaml_config
from freshness_real_robot_validation.frame_id_codec import decode_validation_frame_id
from freshness_real_robot_validation.json_topics import string_message_from_payload
from freshness_real_robot_validation.wrapper_metadata import WrapperMetadataCsvWriter, WrapperMetadataTracker


class CmdChannelWrapperNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_channel_wrapper")
        self.robot_name = str(self.declare_parameter("robot_name", "tracer1").value)
        self.scenario_id = str(self.declare_parameter("scenario_id", "real-nominal").value)
        self.method_id = str(self.declare_parameter("method_id", "FR-TPO").value)
        self.wrapper_mode = str(self.declare_parameter("wrapper_mode", "observe").value)
        self.publish_passthrough = bool(self.declare_parameter("publish_passthrough", True).value)
        self.run_id = str(self.declare_parameter("run_id", "wrapper_run").value)
        self.payload_type = str(
            self.declare_parameter("payload_type", "geometry_msgs/msg/TwistStamped").value
        )
        self.receiver_clock_type = str(self.declare_parameter("receiver_clock_type", "rclpy_node_clock").value)
        self.source_clock_type_default = str(
            self.declare_parameter("source_clock_type_default", "message_header_stamp").value
        )
        self.allow_true_one_way_delay = bool(self.declare_parameter("allow_true_one_way_delay", False).value)
        metadata_log_root = str(
            self.declare_parameter("metadata_log_root", "~/.ros/freshness_real_robot_validation/wrapper_metadata").value
        )
        metadata_csv_path = str(self.declare_parameter("metadata_csv_path", "").value)
        config_path = str(self.declare_parameter("config_path", "").value)
        config = load_yaml_config(config_path)
        scenario_cfg = dict(config.get("scenarios", {}).get(self.scenario_id, {}))

        delay_ms_mean = float(self.declare_parameter("delay_ms_mean", scenario_cfg.get("delay_ms_mean", 0.0)).value)
        jitter_ms = float(self.declare_parameter("jitter_ms", scenario_cfg.get("jitter_ms", 0.0)).value)
        loss_rate = float(self.declare_parameter("loss_rate", scenario_cfg.get("loss_rate", 0.0)).value)
        burst_loss_rate = float(self.declare_parameter("burst_loss_rate", scenario_cfg.get("burst_loss_rate", 0.0)).value)
        duplicate_on_critical = bool(
            self.declare_parameter("duplicate_on_critical", scenario_cfg.get("duplicate_on_critical", False)).value
        )

        self.input_topic = str(
            self.declare_parameter("input_topic", f"/fr_validation/{self.robot_name}/cmd_vel_stamped_tx").value
        )
        self.output_topic = str(self.declare_parameter("output_topic", f"/{self.robot_name}/cmd_vel_stamped").value)
        self.meta_topic = str(self.declare_parameter("meta_topic", f"/fr_validation/{self.robot_name}/cmd_channel_meta").value)

        self.engine = ChannelWrapperEngine(
            scenario_id=self.scenario_id,
            wrapper_mode=self.wrapper_mode,
            delay_ms_mean=delay_ms_mean,
            jitter_ms=jitter_ms,
            loss_rate=loss_rate,
            burst_loss_rate=burst_loss_rate,
            duplicate_on_critical=duplicate_on_critical,
        )
        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.meta_publisher = self.create_publisher(String, self.meta_topic, 10)
        self.create_subscription(TwistStamped, self.input_topic, self._cmd_cb, 10)
        self._pending: list[tuple[float, TwistStamped]] = []
        csv_path = (
            Path(metadata_csv_path).expanduser()
            if metadata_csv_path.strip()
            else Path(metadata_log_root).expanduser() / self.run_id / f"{self.robot_name}_cmd_channel_meta.csv"
        )
        self.metadata_writer = WrapperMetadataCsvWriter(csv_path)
        self.metadata_tracker = WrapperMetadataTracker(receiver_clock_type=self.receiver_clock_type)
        self.create_timer(0.01, self._flush_pending)

    def _cmd_cb(self, msg: TwistStamped) -> None:
        decoded = decode_validation_frame_id(str(msg.header.frame_id))
        wrapper_receive_timestamp_ns = int(self.get_clock().now().nanoseconds)
        source_send_timestamp_ns = self._stamp_to_ns(msg)
        source_clock_type = self.source_clock_type_default if source_send_timestamp_ns is not None else "n/a"
        envelope = MessageEnvelope(
            robot_id=self.robot_name,
            seq_id=int(decoded["seq_id"]),
            sender_timestamp=int(source_send_timestamp_ns or 0),
            payload_bytes=int(decoded["payload_bytes"]),
            task_phase=str(decoded.get("task_phase", "")) or "transport",
            task_progress=float(decoded.get("task_progress", 0.0)),
            scenario_id=self.scenario_id,
            method_id=str(decoded["method_id"] or self.method_id),
            transmission_mode=str(decoded["transmission_mode"]),
            execution_mode=str(decoded.get("execution_mode", "normal") or "normal"),
            aoi_ms=decoded.get("aoi_ms"),
            effective_freshness=decoded.get("effective_freshness"),
        )
        plan = self.engine.build_plan(envelope)
        row = self.metadata_tracker.build_row(
            seq_id=envelope.seq_id,
            payload_type=self.payload_type,
            payload_bytes=envelope.payload_bytes,
            source_send_timestamp_ns=source_send_timestamp_ns,
            source_clock_type=source_clock_type,
            wrapper_receive_timestamp_ns=wrapper_receive_timestamp_ns,
            transmission_mode=envelope.transmission_mode,
            phase=envelope.task_phase,
            task_progress=envelope.task_progress,
            retry_count=plan.retry_count,
            execution_mode=str(plan.metadata.get("execution_mode", envelope.execution_mode)),
            aoi_ms=envelope.aoi_ms,
            effective_freshness=envelope.effective_freshness,
            deadline_met="n/a",
            scenario_id=self.scenario_id,
            method_id=envelope.method_id,
            robot_id=self.robot_name,
            source_mode=str(plan.metadata.get("source_mode", "wrapped_cmd_vel_stamped")),
            delivery_expected=bool(plan.delivery_expected),
            wrapper_mode=self.wrapper_mode,
            wrapper_emit_timestamp_ns=None,
            allow_true_one_way_delay=self.allow_true_one_way_delay,
        )
        self.metadata_writer.write_row(row)
        self.meta_publisher.publish(string_message_from_payload(row))
        if not self.publish_passthrough or plan.forward_count <= 0:
            return

        for offset_idx in range(plan.forward_count):
            outgoing = copy.deepcopy(msg)
            outgoing.header.frame_id = str(msg.header.frame_id or envelope.seq_id)
            due_time = time.time() + (plan.delay_ms / 1000.0) + (0.005 * offset_idx)
            self._pending.append((due_time, outgoing))

    def _flush_pending(self) -> None:
        if not self._pending:
            return
        now = time.time()
        ready = [item for item in self._pending if item[0] <= now]
        self._pending = [item for item in self._pending if item[0] > now]
        for _, msg in ready:
            self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        self.metadata_writer.close()
        return super().destroy_node()

    @staticmethod
    def _stamp_to_ns(msg: TwistStamped) -> int | None:
        sec = int(getattr(msg.header.stamp, "sec", 0))
        nanosec = int(getattr(msg.header.stamp, "nanosec", 0))
        if sec == 0 and nanosec == 0:
            return None
        return sec * 1_000_000_000 + nanosec


def main() -> None:
    rclpy.init(args=None)
    node = CmdChannelWrapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
