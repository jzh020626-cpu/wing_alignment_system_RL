from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SyntheticCalibrationPublisher(Node):
    def __init__(self, *, rate_hz: float, duration_sec: float, drop_every_n: int, phase_cycle: list[str]):
        super().__init__("synthetic_calibration_publisher")
        self.publisher = self.create_publisher(String, "/calib/test_status", 10)
        self.period_sec = 1.0 / max(rate_hz, 1e-6)
        self.duration_sec = max(duration_sec, 0.1)
        self.drop_every_n = max(drop_every_n, 0)
        self.phase_cycle = phase_cycle or ["dispatch", "cooperative_transport", "narrow_passage", "final_alignment", "release_exit"]
        self.start_monotonic = time.monotonic()
        self.finished = False
        self.seq_id = 0
        self.sent_count = 0
        self.timer = self.create_timer(self.period_sec, self._publish_tick)

    def _publish_tick(self) -> None:
        if time.monotonic() - self.start_monotonic >= self.duration_sec:
            self.timer.cancel()
            self.finished = True
            return
        current_seq = self.seq_id
        self.seq_id += 1
        if self.drop_every_n > 0 and current_seq > 0 and current_seq % self.drop_every_n == 0:
            return
        payload = {
            "seq_id": current_seq,
            "sender_timestamp": time.time_ns(),
            "phase": self.phase_cycle[current_seq % len(self.phase_cycle)],
            "task_progress": round((current_seq % 20) / 20.0, 3),
            "control_mode": "calibration",
            "emergency_stop": False,
            "fallback_flag": bool(current_seq % 7 == 0 and current_seq > 0),
            "done_reason": "running",
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(msg)
        self.sent_count += 1


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument("--drop-every-n", type=int, default=0)
    parser.add_argument("--phase-cycle", default="dispatch,cooperative_transport,narrow_passage,final_alignment,release_exit")
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_argument_parser().parse_args(argv)
    os.environ.setdefault("ROS_DOMAIN_ID", "36")
    phases = [phase.strip() for phase in str(args.phase_cycle).split(",") if phase.strip()]
    rclpy.init(args=[])
    node = SyntheticCalibrationPublisher(
        rate_hz=float(args.rate_hz),
        duration_sec=float(args.duration_sec),
        drop_every_n=int(args.drop_every_n),
        phase_cycle=phases,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
