#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish isolated desired commands for scheduler/watchdog artifact smoke tests.

This script only publishes to an isolated namespace by default:
`/p1_nominal_isolated/<robot>/cmd_vel_desired`.
It does not publish to `/tracer*/cmd_vel` or any chassis driver topic.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def _robots(value: str) -> list[str]:
    return [part.strip() for part in str(value).split(",") if part.strip()]


def _topic(prefix: str, robot: str) -> str:
    clean_prefix = "/" + str(prefix).strip("/")
    return f"{clean_prefix}/{robot}/cmd_vel_desired"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Publish isolated desired Twist commands for nominal artifact smoke tests."
    )
    parser.add_argument("--run-id", required=True, help="Run ID recorded in the manifest.")
    parser.add_argument("--robots", default="tracer1,tracer2,tracer3")
    parser.add_argument("--isolated-prefix", default="/p1_nominal_isolated")
    parser.add_argument("--duration-sec", type=float, default=4.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--linear-x", type=float, default=0.03)
    parser.add_argument("--angular-z", type=float, default=0.02)
    parser.add_argument("--baseline-mode", default="current_safe_default")
    parser.add_argument("--profile", default="nominal")
    args = parser.parse_args()

    if args.baseline_mode != "current_safe_default":
        raise SystemExit("only current_safe_default is allowed for this isolated replay smoke")
    if args.profile != "nominal":
        raise SystemExit("only nominal profile is allowed for this isolated replay smoke")
    if "/tracer" in str(args.isolated_prefix):
        raise SystemExit("isolated-prefix must not be a real /tracer* control namespace")

    robots = _robots(args.robots)
    if not robots:
        raise SystemExit("at least one robot is required")

    rclpy.init()
    node = rclpy.create_node("p1_nominal_isolated_cmd_replay")
    publishers = {
        robot: node.create_publisher(Twist, _topic(args.isolated_prefix, robot), 10)
        for robot in robots
    }

    try:
        period = 1.0 / max(float(args.rate_hz), 1e-6)
        end_time = time.monotonic() + max(float(args.duration_sec), 0.0)
        tick = 0
        while time.monotonic() < end_time:
            msg = Twist()
            msg.linear.x = float(args.linear_x)
            msg.angular.z = float(args.angular_z)
            for pub in publishers.values():
                pub.publish(msg)
            tick += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
        print(
            f"published {tick} isolated desired-command ticks for run_id={args.run_id} "
            f"robots={','.join(robots)} prefix={args.isolated_prefix}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
