#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3-D0b Replay Phase Source: Timed replay of mission phases for shadow validation.

Produces /fr_validation/derived_phase_status on a deterministic timeline,
explicitly marked phase_source=replay. No real mission_coordinator dependency.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.json_topics import string_message_from_payload

# Deterministic phase timeline for a 60-second D0b replay
REPLAY_PHASES = [
    # (offset_sec, mission_state, task_phase, task_progress, phase_progress_proxy)
    (0.0,   "STANDBY",          "standby",        0.00, 0.00),
    (4.0,   "WAIT_WING",        "approach",       0.10, 0.15),
    (8.0,   "SYNC_APPROACH_X",  "approach",       0.28, 0.40),
    (13.0,  "SYNC_APPROACH_Y",  "approach",       0.50, 0.70),
    (17.0,  "RUN_ALIGNMENT",    "approach",       0.80, 0.95),
    (22.0,  "SYNC_SLIDE_ALIGN", "slide_align",    0.40, 0.60),
    (28.0,  "ALL_READY_HOLD",   "slide_align",    0.60, 1.00),
    (33.0,  "SYNC_LEVEL_Z",     "level_recenter", 0.30, 0.40),
    (38.0,  "SYNC_RECENTER",    "level_recenter", 0.55, 0.75),
    (43.0,  "LOAD_STABLE_HOLD", "level_recenter", 0.75, 1.00),
    (48.0,  "TRANSPORT_PRECHECK","transport",      0.10, 0.20),
    (53.0,  "SYNC_TRANSPORT",   "transport",       0.50, 0.70),
    (58.0,  "TRANSPORT_SETTLE", "transport",       0.75, 0.90),
    (63.0,  "DONE",             "transport",       0.90, 1.00),
]

class ReplayPhaseSourceNode(Node):
    def __init__(self):
        super().__init__("p3d_replay_phase_source")
        self.output_topic = str(
            self.declare_parameter("output_topic", "/fr_validation/derived_phase_status").value
        )
        self.run_id = str(self.declare_parameter("run_id", "p3d_shadow").value)
        self.replay_speed = float(self.declare_parameter("replay_speed", 1.0).value)
        self.loop = bool(self.declare_parameter("replay_loop", False).value)
        self.hz = float(self.declare_parameter("publish_rate_hz", 5.0).value)

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self._start_monotonic = time.monotonic()
        self._phase_index = 0
        self._phase_count = len(REPLAY_PHASES)
        self._timer = self.create_timer(1.0 / self.hz, self._tick)

        self.get_logger().info(
            f"[P3-D0b ReplayPhaseSource] output={self.output_topic} "
            f"run_id={self.run_id} speed={self.replay_speed} loop={self.loop} hz={self.hz}"
        )

    def _tick(self):
        elapsed = (time.monotonic() - self._start_monotonic) * self.replay_speed
        if self.loop and REPLAY_PHASES[-1][0] > 0:
            elapsed = elapsed % REPLAY_PHASES[-1][0]

        # Linear scan for current phase
        phase_idx = 0
        for i, (t, *_rest) in enumerate(REPLAY_PHASES):
            if elapsed >= t:
                phase_idx = i
            else:
                break

        offset_sec, mission_state, task_phase, task_progress, phase_progress_proxy = REPLAY_PHASES[phase_idx]

        payload = {
            "mission_state": mission_state,
            "task_phase": task_phase,
            "task_progress": float(task_progress),
            "phase_progress_proxy": float(phase_progress_proxy),
            "source_mode": "replay",
            "confidence": 0.99,
            "run_id": str(self.run_id),
            "aborted": False,
            "Effective_Freshness": 0.70,
            "AoI_ms": 180.0,
            "stale_indicator": 0.0,
            "scenario_id": "p3d_shadow",
            "phase_source": "replay",
        }
        msg = string_message_from_payload(payload)
        self.publisher.publish(msg)


def main():
    rclpy.init(args=None)
    node = ReplayPhaseSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
