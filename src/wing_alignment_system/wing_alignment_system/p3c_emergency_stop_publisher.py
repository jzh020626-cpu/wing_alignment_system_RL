#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _flag_asserted(flag_path: str) -> bool:
    if not str(flag_path or "").strip():
        return False
    return Path(flag_path).expanduser().exists()


class P3CEmergencyStopPublisher(Node):
    def __init__(self):
        super().__init__("p3c_emergency_stop_publisher")
        self.topic = str(self.declare_parameter("topic", "/wing_alignment/emergency_stop").value)
        self.publish_hz = max(1.0, float(self.declare_parameter("publish_hz", 5.0).value))
        self.default_state = bool(self.declare_parameter("default_state", False).value)
        self.assert_stop_on_start = bool(self.declare_parameter("assert_stop_on_start", False).value)
        self.shutdown_publish_true = bool(self.declare_parameter("shutdown_publish_true", True).value)
        self.stop_file = str(self.declare_parameter("stop_file", "/tmp/p3c_emergency_stop.flag").value)
        self._last_state = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Bool, self.topic, qos)
        self.timer = self.create_timer(1.0 / self.publish_hz, self._tick)
        self._tick()

    def _resolved_state(self) -> bool:
        if self.assert_stop_on_start:
            return True
        if _flag_asserted(self.stop_file):
            return True
        return self.default_state

    def _publish_state(self, asserted: bool) -> None:
        msg = Bool()
        msg.data = bool(asserted)
        self.pub.publish(msg)
        self._last_state = bool(asserted)

    def _tick(self) -> None:
        self._publish_state(self._resolved_state())

    def destroy_node(self):
        if self.shutdown_publish_true:
            try:
                self._publish_state(True)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = P3CEmergencyStopPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
