#!/usr/bin/env python3
"""Read-only Stage 0 topic safety monitor.

The script subscribes to command topics and writes JSON/Markdown summaries. It
does not publish any ROS messages.
"""

from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from base_interfaces_demo.msg import MotorCommand


ROBOTS = ("tracer1", "tracer2", "tracer3")
SLIDES = ("huatai1", "huatai2", "huatai3")


def _topic_entry(topic_type: str) -> dict:
    return {
        "type": topic_type,
        "count": 0,
        "nonzero_count": 0,
        "true_count": 0,
        "max_abs": 0.0,
        "last_sample": None,
    }


class Stage0TopicSafetyMonitor:
    def __init__(self, duration_sec: float):
        self.deadline = time.time() + max(0.0, duration_sec)
        self.stop_requested = False
        self.summary = {
            "started_at": time.time(),
            "duration_sec": duration_sec,
            "topics": {},
            "boundary": "read-only monitor; no ROS topic publications",
        }
        self.node = rclpy.create_node("stage0_topic_safety_monitor")
        self._subscriptions = []
        for robot in ROBOTS:
            self._watch_twist(f"/{robot}/cmd_vel")
            self._watch_twist(f"/{robot}/cmd_goal")
            self._watch_bool(f"/{robot}/cmd_stop")
            self._watch_bool(f"/{robot}/precision_mode")
        for slide in SLIDES:
            self._watch_motor(f"/{slide}_pos_spe_pd")
            self._watch_motor(f"/{slide}_compensation_ref")

    def request_stop(self, *_args) -> None:
        self.stop_requested = True

    def _entry(self, topic: str, topic_type: str) -> dict:
        return self.summary["topics"].setdefault(topic, _topic_entry(topic_type))

    def _watch_twist(self, topic: str) -> None:
        entry = self._entry(topic, "geometry_msgs/msg/Twist")

        def cb(msg: Twist) -> None:
            values = [
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.linear.z),
                float(msg.angular.x),
                float(msg.angular.y),
                float(msg.angular.z),
            ]
            magnitude = max(abs(value) for value in values)
            entry["count"] += 1
            entry["max_abs"] = max(float(entry["max_abs"]), magnitude)
            if magnitude > 1e-9:
                entry["nonzero_count"] += 1
            entry["last_sample"] = {
                "linear": values[:3],
                "angular": values[3:],
            }

        self._subscriptions.append(self.node.create_subscription(Twist, topic, cb, 10))

    def _watch_bool(self, topic: str) -> None:
        entry = self._entry(topic, "std_msgs/msg/Bool")

        def cb(msg: Bool) -> None:
            value = bool(msg.data)
            entry["count"] += 1
            if value:
                entry["true_count"] += 1
            entry["last_sample"] = {"data": value}

        self._subscriptions.append(self.node.create_subscription(Bool, topic, cb, 10))

    def _watch_motor(self, topic: str) -> None:
        entry = self._entry(topic, "base_interfaces_demo/msg/MotorCommand")

        def cb(msg: MotorCommand) -> None:
            values = [
                float(msg.x),
                float(msg.y),
                float(msg.z),
                float(msg.vx),
                float(msg.vy),
                float(msg.vz),
            ]
            magnitude = max(abs(value) for value in values)
            entry["count"] += 1
            entry["max_abs"] = max(float(entry["max_abs"]), magnitude)
            if magnitude > 1e-9:
                entry["nonzero_count"] += 1
            entry["last_sample"] = {
                "command_type": str(msg.command_type),
                "xyz": values[:3],
                "vxyz": values[3:],
                "time": float(msg.time),
                "is_relative": bool(msg.is_relative),
                "can_id": str(msg.can_id),
            }

        self._subscriptions.append(self.node.create_subscription(MotorCommand, topic, cb, 10))

    def run(self) -> dict:
        while rclpy.ok() and (not self.stop_requested) and time.time() < self.deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.summary["finished_at"] = time.time()
        self.summary["elapsed_sec"] = self.summary["finished_at"] - self.summary["started_at"]
        self.summary["checks"] = _evaluate(self.summary["topics"])
        self.node.destroy_node()
        return self.summary


def _evaluate(topics: dict) -> dict:
    cmd_vel_nonzero = []
    cmd_goal_messages = []
    slide_command_messages = []
    for robot in ROBOTS:
        vel = topics.get(f"/{robot}/cmd_vel", {})
        if int(vel.get("nonzero_count", 0)) > 0:
            cmd_vel_nonzero.append(f"/{robot}/cmd_vel")
        goal = topics.get(f"/{robot}/cmd_goal", {})
        if int(goal.get("count", 0)) > 0:
            cmd_goal_messages.append(f"/{robot}/cmd_goal")
    for slide in SLIDES:
        for suffix in ("pos_spe_pd", "compensation_ref"):
            topic = f"/{slide}_{suffix}"
            entry = topics.get(topic, {})
            if int(entry.get("count", 0)) > 0:
                slide_command_messages.append(topic)
    passed = not (cmd_vel_nonzero or cmd_goal_messages or slide_command_messages)
    return {
        "passed": passed,
        "cmd_vel_only_zero": not cmd_vel_nonzero,
        "cmd_vel_nonzero_topics": cmd_vel_nonzero,
        "cmd_goal_absent": not cmd_goal_messages,
        "cmd_goal_topics_with_messages": cmd_goal_messages,
        "slide_command_absent": not slide_command_messages,
        "slide_command_topics_with_messages": slide_command_messages,
    }


def _write_markdown(path: str, summary: dict) -> None:
    if not path:
        return
    checks = summary.get("checks", {})
    lines = [
        "# Stage 0 topic safety summary",
        "",
        f"- passed: {checks.get('passed')}",
        f"- cmd_vel_only_zero: {checks.get('cmd_vel_only_zero')}",
        f"- cmd_goal_absent: {checks.get('cmd_goal_absent')}",
        f"- slide_command_absent: {checks.get('slide_command_absent')}",
        "",
        "This is a read-only topic monitor summary. It is not a safety proof.",
        "",
        "## Observed messages",
        "",
    ]
    for topic, entry in sorted(summary.get("topics", {}).items()):
        if entry.get("count", 0):
            lines.append(
                f"- `{topic}`: count={entry.get('count')} "
                f"nonzero={entry.get('nonzero_count')} true={entry.get('true_count')} "
                f"max_abs={entry.get('max_abs')}"
            )
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor Stage 0 command topics without publishing.")
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--json-out", required=True)
    parser.add_argument("--md-out", default="")
    args = parser.parse_args()

    rclpy.init()
    monitor = Stage0TopicSafetyMonitor(args.duration_sec)
    signal.signal(signal.SIGINT, monitor.request_stop)
    signal.signal(signal.SIGTERM, monitor.request_stop)
    try:
        summary = monitor.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    json_path = Path(args.json_out)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    _write_markdown(args.md_out, summary)
    print(json.dumps(summary["checks"], indent=2, ensure_ascii=False))
    return 0 if summary["checks"]["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
