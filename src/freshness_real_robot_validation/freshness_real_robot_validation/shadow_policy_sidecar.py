from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.json_topics import payload_from_string_message, string_message_from_payload
from freshness_real_robot_validation.policy_logic import decide_shadow_pair


def _robots(value: str) -> list[str]:
    return [part.strip() for part in str(value).split(",") if part.strip()]


class ShadowPolicySidecarNode(Node):
    def __init__(self) -> None:
        super().__init__("shadow_policy_sidecar")
        self.robot_ids = _robots(str(self.declare_parameter("robot_ids", "tracer1,tracer2,tracer3").value))
        self.derived_phase_topic = str(self.declare_parameter("derived_phase_topic", "/fr_validation/derived_phase_status").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/fr_validation/shadow_policy_decisions").value)
        self.enable_execution_mode = bool(self.declare_parameter("enable_execution_mode", False).value)
        self.default_effective_freshness = float(self.declare_parameter("default_effective_freshness", 0.90).value)
        self.default_aoi_ms = float(self.declare_parameter("default_aoi_ms", 100.0).value)
        self.default_stale_indicator = float(self.declare_parameter("default_stale_indicator", 0.0).value)
        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(String, self.derived_phase_topic, self._phase_cb, 10)

    def _phase_cb(self, msg: String) -> None:
        phase_payload = payload_from_string_message(msg)
        if not phase_payload:
            return

        decisions = {}
        for robot_id in self.robot_ids:
            decisions[robot_id] = decide_shadow_pair(
                task_phase=str(phase_payload.get("task_phase", "standby")),
                task_progress=float(phase_payload.get("task_progress", 0.0)),
                effective_freshness=float(phase_payload.get("Effective_Freshness", self.default_effective_freshness)),
                aoi_ms=float(phase_payload.get("AoI_ms", self.default_aoi_ms)),
                stale_indicator=float(phase_payload.get("stale_indicator", self.default_stale_indicator)),
                enable_execution_mode=self.enable_execution_mode,
            )

        payload = {
            "seq_id": int(time.time() * 1000),
            "sender_timestamp": int(time.time_ns()),
            "robot_id": "fleet",
            "task_phase": str(phase_payload.get("task_phase", "standby")),
            "task_progress": float(phase_payload.get("task_progress", 0.0)),
            "transmission_mode": "shadow_pair",
            "retry_count": 0,
            "payload_bytes": 0,
            "source_mode": "shadow_policy_sidecar",
            "scenario_id": str(phase_payload.get("scenario_id", "")),
            "method_id": "DGWS|FR-TPO",
            "decisions": decisions,
        }
        self.publisher.publish(string_message_from_payload(payload))


def main() -> None:
    rclpy.init(args=None)
    node = ShadowPolicySidecarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
