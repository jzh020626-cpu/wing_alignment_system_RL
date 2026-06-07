from __future__ import annotations

import copy
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.frame_id_codec import decode_validation_frame_id
from freshness_real_robot_validation.frame_id_codec import encode_validation_frame_id
from freshness_real_robot_validation.json_topics import payload_from_string_message, string_message_from_payload
from freshness_real_robot_validation.policy_logic import decide_tx_mode


class TxPolicySidecarNode(Node):
    def __init__(self) -> None:
        super().__init__("tx_policy_sidecar")
        self.robot_name = str(self.declare_parameter("robot_name", "tracer1").value)
        self.method_id = str(self.declare_parameter("method_id", "FR-TPO").value)
        self.input_topic = str(self.declare_parameter("input_topic", f"/fr_validation/{self.robot_name}/cmd_vel_stamped_source").value)
        self.output_topic = str(self.declare_parameter("output_topic", f"/fr_validation/{self.robot_name}/cmd_vel_stamped_tx").value)
        self.meta_topic = str(self.declare_parameter("meta_topic", f"/fr_validation/{self.robot_name}/tx_policy_meta").value)
        self.derived_phase_topic = str(self.declare_parameter("derived_phase_topic", "/fr_validation/derived_phase_status").value)
        self.compact_min_gap_ms = float(self.declare_parameter("compact_min_gap_ms", 120.0).value)
        self.enable_execution_mode = bool(self.declare_parameter("enable_execution_mode", False).value)
        self.default_effective_freshness = float(self.declare_parameter("default_effective_freshness", 0.90).value)
        self.default_aoi_ms = float(self.declare_parameter("default_aoi_ms", 100.0).value)
        self.default_stale_indicator = float(self.declare_parameter("default_stale_indicator", 0.0).value)

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.meta_publisher = self.create_publisher(String, self.meta_topic, 10)
        self.create_subscription(TwistStamped, self.input_topic, self._cmd_cb, 10)
        self.create_subscription(String, self.derived_phase_topic, self._phase_cb, 10)

        self._latest_phase = {"task_phase": "standby", "task_progress": 0.0}
        self._last_forward_wall = 0.0

    def _phase_cb(self, msg: String) -> None:
        payload = payload_from_string_message(msg)
        if payload:
            self._latest_phase = payload

    def _cmd_cb(self, msg: TwistStamped) -> None:
        phase = str(self._latest_phase.get("task_phase", "standby"))
        progress = float(self._latest_phase.get("task_progress", 0.0))
        decision = decide_tx_mode(
            method_id=self.method_id,
            task_phase=phase,
            task_progress=progress,
            effective_freshness=float(self._latest_phase.get("Effective_Freshness", self.default_effective_freshness)),
            aoi_ms=float(self._latest_phase.get("AoI_ms", self.default_aoi_ms)),
            stale_indicator=float(self._latest_phase.get("stale_indicator", self.default_stale_indicator)),
            enable_execution_mode=self.enable_execution_mode,
        )
        effective_freshness = float(self._latest_phase.get("Effective_Freshness", self.default_effective_freshness))
        aoi_ms = float(self._latest_phase.get("AoI_ms", self.default_aoi_ms))
        stale_indicator = float(self._latest_phase.get("stale_indicator", self.default_stale_indicator))

        forward = True
        now_wall = time.time()
        if decision["transmission_mode"] == "compact_update":
            min_gap = max(0.0, self.compact_min_gap_ms) / 1000.0
            if now_wall - self._last_forward_wall < min_gap:
                forward = False
        elif decision["transmission_mode"] == "skip_update":
            forward = False

        decoded = decode_validation_frame_id(str(getattr(msg.header, "frame_id", "") or ""))
        seq_id = int(decoded.get("seq_id", 0) or 0)
        meta = {
            "seq_id": seq_id,
            "sender_timestamp": int(time.time_ns()),
            "robot_id": self.robot_name,
            "task_phase": phase,
            "task_progress": progress,
            "transmission_mode": str(decision["transmission_mode"]),
            "execution_mode": str(decision["execution_mode"]),
            "retry_count": 0,
            "payload_bytes": int(decision["payload_bytes"]),
            "aoi_ms": aoi_ms,
            "effective_freshness": effective_freshness,
            "stale_indicator": stale_indicator,
            "source_mode": "tx_policy_sidecar",
            "scenario_id": str(self._latest_phase.get("scenario_id", "")),
            "method_id": str(self.method_id),
            "forwarded": bool(forward),
        }
        self.meta_publisher.publish(string_message_from_payload(meta))
        if not forward:
            return

        outgoing = copy.deepcopy(msg)
        outgoing.header.frame_id = encode_validation_frame_id(
            seq_id=seq_id,
            transmission_mode=str(decision["transmission_mode"]),
            payload_bytes=int(decision["payload_bytes"]),
            method_id=self.method_id,
            task_phase=phase,
            task_progress=progress,
            execution_mode=str(decision["execution_mode"]),
            aoi_ms=aoi_ms,
            effective_freshness=effective_freshness,
        )
        self.publisher.publish(outgoing)
        self._last_forward_wall = now_wall


def main() -> None:
    rclpy.init(args=None)
    node = TxPolicySidecarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
