from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.json_topics import string_message_from_payload
from freshness_real_robot_validation.observe_only_traffic import build_synthetic_frame_spec


class ObserveOnlyTrafficSourceNode(Node):
    def __init__(self) -> None:
        super().__init__("observe_only_traffic_source")
        self.robot_name = str(self.declare_parameter("robot_name", "tracer1").value)
        self.output_topic = str(
            self.declare_parameter("output_topic", f"/fr_validation/{self.robot_name}/cmd_vel_stamped_tx").value
        )
        self.meta_topic = str(
            self.declare_parameter("meta_topic", f"/fr_validation/{self.robot_name}/observe_only_source_meta").value
        )
        self.publish_rate_hz = max(0.5, float(self.declare_parameter("publish_rate_hz", 5.0).value))
        self.payload_bytes = max(16, int(self.declare_parameter("payload_bytes", 96).value))
        self.task_phase = str(self.declare_parameter("task_phase", "standby").value)
        self.task_progress = float(self.declare_parameter("task_progress", 0.0).value)
        self.method_id = str(self.declare_parameter("method_id", "SYNTHETIC_OBSERVE_ONLY").value)
        self.transmission_mode = str(self.declare_parameter("transmission_mode", "synthetic_heartbeat").value)
        self.scenario_id = str(self.declare_parameter("scenario_id", "observe-only").value)

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.meta_publisher = self.create_publisher(String, self.meta_topic, 10)
        self._seq_id = 0
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_once)

    def _publish_once(self) -> None:
        self._seq_id += 1
        spec = build_synthetic_frame_spec(
            seq_id=self._seq_id,
            robot_name=self.robot_name,
            task_phase=self.task_phase,
            task_progress=self.task_progress,
            payload_bytes=self.payload_bytes,
            method_id=self.method_id,
            transmission_mode=self.transmission_mode,
        )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(spec["frame_id"])
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)

        self.meta_publisher.publish(
            string_message_from_payload(
                {
                    "seq_id": int(spec["seq_id"]),
                    "sender_timestamp": int(time.time_ns()),
                    "robot_id": str(spec["robot_id"]),
                    "task_phase": str(spec["task_phase"]),
                    "task_progress": float(spec["task_progress"]),
                    "transmission_mode": str(spec["transmission_mode"]),
                    "retry_count": 0,
                    "payload_bytes": int(spec["payload_bytes"]),
                    "source_mode": str(spec["source_mode"]),
                    "scenario_id": self.scenario_id,
                    "method_id": str(spec["method_id"]),
                    "payload_type": "geometry_msgs/msg/TwistStamped",
                }
            )
        )


def main() -> None:
    rclpy.init(args=None)
    node = ObserveOnlyTrafficSourceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
