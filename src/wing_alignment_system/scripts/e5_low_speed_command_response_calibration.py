#!/usr/bin/env python3
"""E5 Stage 2A: tracer1 low-speed command-response calibration.

Supports three feedback modes:
  pose_stamped    -- velocity from PoseStamped position deltas (/Rigid17/pose)
  odometry        -- velocity from Odometry twist (/tracer1/odom)
  command_loopback -- echo-only, no motion measurement

Safety: --dry-run is default.  --execute required for real cmd_vel.
         Rejects linear_x > 0.05 m/s.
         Only robot-id=tracer1 unless explicitly authorized.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import signal
import sys
import time
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

MAX_LINEAR_SPEED = 0.05
DEFAULT_LINEAR_SPEED = 0.03
ALLOWED_ROBOTS = {"tracer1", "tracer2", "tracer3"}


def script_sha256(path: str) -> str:
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except Exception:
        return "unavailable"


class CalibrationNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("e5_stage2a_calibration")
        self._args = args
        self._execute = args.execute
        self._dry_run = args.dry_run

        self._fb_type = args.feedback_type
        self._fb_topic = args.feedback_topic
        self._fb_samples: List[dict] = []
        self._fb_last_pose = None
        self._fb_last_time_ros = None
        self._fb_last_time_wall = None
        self._fb_sample_idx = 0
        self._previous_intervals: List[float] = []

        if self._fb_type == "pose_stamped":
            self._fb_sub = self.create_subscription(
                PoseStamped, self._fb_topic, self._fb_pose_cb, 10)
        elif self._fb_type == "odometry":
            self._fb_sub = self.create_subscription(
                Odometry, self._fb_topic, self._fb_odom_cb, 10)
        elif self._fb_type == "command_loopback":
            self._fb_sub = self.create_subscription(
                Twist, args.cmd_topic, self._fb_loopback_cb, 10)
        else:
            self.get_logger().error(f"Unknown feedback_type: {self._fb_type}")
            sys.exit(1)

        self._cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self._shutdown = False
        signal.signal(signal.SIGINT, self._sigint_handler)

        if self._dry_run:
            self.get_logger().warn("DRY RUN: no command will be published")
        elif self._execute:
            self.get_logger().warn(
                f"EXECUTE MODE: low-speed cmd_vel will be published to {args.cmd_topic}")

    def _sigint_handler(self, signum, frame):
        self._shutdown = True
        self.get_logger().info("SIGINT received, shutting down")

    def _fb_pose_cb(self, msg: PoseStamped):
        now_ros = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        now_wall = time.time()
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        frame_id = msg.header.frame_id
        speed, dt_ms, outlier = None, None, 0

        if self._fb_last_pose is not None and self._fb_last_time_ros is not None:
            dt = now_ros - self._fb_last_time_ros
            if dt > 1e-9:
                dx = px - self._fb_last_pose[0]
                dy = py - self._fb_last_pose[1]
                dz = pz - self._fb_last_pose[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > self._args.pose_jump_threshold:
                    outlier = 1
                speed = dist / dt
                dt_ms = dt * 1000.0
            if dt_ms is not None and dt_ms > 0:
                self._previous_intervals.append(dt_ms)

        self._fb_last_pose = (px, py, pz)
        self._fb_last_time_ros = now_ros
        self._fb_last_time_wall = now_wall
        interval_ms = round(dt_ms, 3) if dt_ms is not None else None
        self._fb_samples.append({
            "run_id": self._args.run_id, "robot_id": self._args.robot_id,
            "feedback_type": self._fb_type,
            "feedback_time_ros": round(now_ros, 6),
            "feedback_time_wall": round(now_wall, 6),
            "frame_id": frame_id,
            "pose_x": round(px, 6), "pose_y": round(py, 6), "pose_z": round(pz, 6),
            "linear_x": "", "linear_y": "", "angular_z": "",
            "speed_abs": round(speed, 6) if speed is not None else "",
            "dt_ms": interval_ms if interval_ms is not None else "",
            "sample_index": self._fb_sample_idx,
            "interval_ms": interval_ms if interval_ms is not None else "",
            "outlier_flag": outlier,
        })
        self._fb_sample_idx += 1

    def _fb_odom_cb(self, msg: Odometry):
        now_ros = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        now_wall = time.time()
        lx = msg.twist.twist.linear.x
        ly = msg.twist.twist.linear.y
        az = msg.twist.twist.angular.z
        speed = math.sqrt(lx*lx + ly*ly)
        frame_id = msg.header.frame_id
        dt_ms = None

        if self._fb_last_time_wall is not None:
            dt = now_wall - self._fb_last_time_wall
            if dt > 1e-9:
                dt_ms = dt * 1000.0
                self._previous_intervals.append(dt_ms)

        self._fb_last_time_ros = now_ros
        self._fb_last_time_wall = now_wall
        self._fb_samples.append({
            "run_id": self._args.run_id, "robot_id": self._args.robot_id,
            "feedback_type": self._fb_type,
            "feedback_time_ros": round(now_ros, 6),
            "feedback_time_wall": round(now_wall, 6),
            "frame_id": frame_id,
            "pose_x": "", "pose_y": "", "pose_z": "",
            "linear_x": round(lx, 6), "linear_y": round(ly, 6),
            "angular_z": round(az, 6),
            "speed_abs": round(speed, 6),
            "dt_ms": round(dt_ms, 3) if dt_ms is not None else "",
            "sample_index": self._fb_sample_idx,
            "interval_ms": round(dt_ms, 3) if dt_ms is not None else "",
            "outlier_flag": 0,
        })
        self._fb_sample_idx += 1

    def _fb_loopback_cb(self, msg: Twist):
        now_wall = time.time()
        speed = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        self._fb_samples.append({
            "run_id": self._args.run_id, "robot_id": self._args.robot_id,
            "feedback_type": self._fb_type,
            "feedback_time_ros": "", "feedback_time_wall": round(now_wall, 6),
            "frame_id": "",
            "pose_x": "", "pose_y": "", "pose_z": "",
            "linear_x": round(msg.linear.x, 6), "linear_y": round(msg.linear.y, 6),
            "angular_z": round(msg.angular.z, 6),
            "speed_abs": round(speed, 6),
            "dt_ms": "", "sample_index": self._fb_sample_idx,
            "interval_ms": "", "outlier_flag": 0,
        })
        self._fb_sample_idx += 1

    def _publish_cmd(self, vx: float):
        if not self._execute:
            return
        twist = Twist()
        twist.linear.x = float(vx)
        self._cmd_pub.publish(twist)

    def _publish_zero_repeated(self):
        if not self._execute:
            return
        for _ in range(10):
            self._publish_cmd(0.0)
            time.sleep(0.1)

    def _analyse_motion(self, summary: dict, samples: list, cmd_start: float, cmd_stop: float):
        args = self._args
        speeds = [s["speed_abs"] for s in samples
                  if isinstance(s.get("speed_abs"), (int, float)) and s["speed_abs"] > 0]
        if speeds:
            summary["max_observed_speed"] = round(max(speeds), 6)
            summary["mean_observed_speed"] = round(sum(speeds) / len(speeds), 6)
        else:
            summary["max_observed_speed"] = ""
            summary["mean_observed_speed"] = ""

        consecutive = 0
        first_motion = None
        for s in samples:
            sp = s.get("speed_abs", 0)
            t = s.get("feedback_time_ros", 0)
            if (isinstance(sp, (int, float)) and sp > args.motion_threshold
                    and not s.get("outlier_flag")):
                consecutive += 1
                if consecutive >= args.min_consecutive_motion_samples and first_motion is None:
                    first_motion = t
            else:
                consecutive = 0

        if first_motion is not None:
            summary["first_motion_time_ros"] = round(first_motion, 6)
            summary["response_lag_ms"] = round((first_motion - cmd_start) * 1000.0, 1)
        else:
            summary["first_motion_time_ros"] = ""
            summary["response_lag_ms"] = "missing"
            if summary.get("status") != "command_loopback_only":
                summary["status"] = "no_motion_detected"

        consecutive = 0
        near_zero = None
        for s in samples:
            sp = s.get("speed_abs", 0)
            t = s.get("feedback_time_ros", 0)
            if isinstance(sp, (int, float)) and isinstance(t, (int, float)) and t >= cmd_stop:
                if sp <= args.zero_threshold:
                    consecutive += 1
                    if consecutive >= args.min_consecutive_zero_samples and near_zero is None:
                        near_zero = t
                else:
                    consecutive = 0

        if near_zero is not None:
            summary["near_zero_time_ros"] = round(near_zero, 6)
            summary["stop_latency_ms"] = round((near_zero - cmd_stop) * 1000.0, 1)
        else:
            summary["near_zero_time_ros"] = ""
            summary["stop_latency_ms"] = "missing"

        if summary.get("status") == "pending":
            summary["status"] = "completed"

    def run(self) -> dict:
        args = self._args
        summary = {
            "run_id": args.run_id, "robot_id": args.robot_id,
            "cmd_topic": args.cmd_topic, "feedback_topic": args.feedback_topic,
            "feedback_type": args.feedback_type,
            "dry_run": args.dry_run, "execute": args.execute,
            "linear_x": args.linear_x,
            "command_duration_sec": args.command_duration_sec,
            "status": "pending", "notes": "",
        }

        self.get_logger().info(f"Pre-wait {args.pre_wait_sec}s (collecting feedback)")
        if self._execute:
            self._publish_zero_repeated()
        self._spin_for(args.pre_wait_sec)
        samples_before = len(self._fb_samples)

        if self._execute:
            self.get_logger().info(f"Pre-zero {args.pre_zero_sec}s")
            self._publish_zero_repeated()
        self._spin_for(args.pre_zero_sec)

        cmd_start_wall = time.time()
        cmd_start_ros = self._fb_last_time_ros or 0.0
        self.get_logger().info(f"CMD vx={args.linear_x} for {args.command_duration_sec}s")
        if self._execute:
            self._publish_cmd(args.linear_x)
        self._spin_for(args.command_duration_sec)

        cmd_stop_wall = time.time()
        cmd_stop_ros = self._fb_last_time_ros or 0.0
        self.get_logger().info("CMD zero stop")
        if self._execute:
            self._publish_cmd(0.0)

        self.get_logger().info(f"Post-stop {args.post_stop_sec}s")
        self._spin_for(args.post_stop_sec)

        if self._execute:
            self._publish_zero_repeated()
        self._spin_for(0.5)

        summary["feedback_sample_count"] = len(self._fb_samples) - samples_before
        summary["cmd_start_time_ros"] = round(cmd_start_ros, 6)
        summary["cmd_start_time_wall"] = round(cmd_start_wall, 6)
        summary["cmd_stop_time_ros"] = round(cmd_stop_ros, 6)
        summary["cmd_stop_time_wall"] = round(cmd_stop_wall, 6)

        if self._fb_type == "command_loopback":
            summary["response_lag_ms"] = "missing"
            summary["stop_latency_ms"] = "missing"
            summary["status"] = "command_loopback_only"
            summary["notes"] = "command-loopback is not hardware motion response calibration"
            echoes = [s for s in self._fb_samples
                      if isinstance(s.get("speed_abs"), (int, float)) and s["speed_abs"] > args.motion_threshold]
            if echoes:
                first_echo = echoes[0]["feedback_time_wall"]
                summary["cmd_echo_delay_ms"] = round((first_echo - cmd_start_wall) * 1000.0, 1)
            else:
                summary["cmd_echo_delay_ms"] = "missing"
        else:
            summary["cmd_echo_delay_ms"] = ""
            self._analyse_motion(summary, self._fb_samples, cmd_start_ros, cmd_stop_ros)

        intervals = sorted(self._previous_intervals)
        if intervals:
            n = len(intervals)
            summary["feedback_jitter_p50_ms"] = round(intervals[int(n*0.50)], 2)
            summary["feedback_jitter_p95_ms"] = round(intervals[min(int(n*0.95), n-1)], 2)
            summary["feedback_jitter_p99_ms"] = round(intervals[min(int(n*0.99), n-1)], 2)
        else:
            summary["feedback_jitter_p50_ms"] = ""
            summary["feedback_jitter_p95_ms"] = ""
            summary["feedback_jitter_p99_ms"] = ""

        os.makedirs(args.out_dir, exist_ok=True)
        self._write_csv_summary(summary)
        self._write_csv_samples(self._fb_samples)
        self._write_manifest(summary)
        self.get_logger().info("Calibration complete")
        return summary

    def _spin_for(self, duration_sec: float):
        start = time.time()
        while time.time() - start < duration_sec and not self._shutdown:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _write_csv_summary(self, summary: dict):
        path = os.path.join(self._args.out_dir, "e5_stage2a_tracer1_command_response.csv")
        fields = [
            "run_id", "robot_id", "cmd_topic", "feedback_topic", "feedback_type",
            "dry_run", "execute", "linear_x", "command_duration_sec", "pre_wait_sec",
            "pre_zero_sec", "post_stop_sec",
            "cmd_start_time_ros", "cmd_start_time_wall", "cmd_stop_time_ros",
            "cmd_stop_time_wall", "first_feedback_time_ros", "first_motion_time_ros",
            "near_zero_time_ros", "response_lag_ms", "stop_latency_ms",
            "cmd_echo_delay_ms", "max_observed_speed", "mean_observed_speed",
            "feedback_sample_count", "feedback_jitter_p50_ms", "feedback_jitter_p95_ms",
            "feedback_jitter_p99_ms", "status", "notes",
        ]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            row = {k: summary.get(k, "") for k in fields}
            row["pre_wait_sec"] = self._args.pre_wait_sec
            row["pre_zero_sec"] = self._args.pre_zero_sec
            row["post_stop_sec"] = self._args.post_stop_sec
            row["first_feedback_time_ros"] = (
                round(self._fb_samples[0]["feedback_time_ros"], 6)
                if self._fb_samples and self._fb_samples[0].get("feedback_time_ros") else "")
            w.writerow(row)

    def _write_csv_samples(self, samples: list):
        path = os.path.join(self._args.out_dir, "e5_stage2a_tracer1_feedback_samples.csv")
        fields = [
            "run_id", "robot_id", "feedback_type", "feedback_time_ros",
            "feedback_time_wall", "frame_id", "pose_x", "pose_y", "pose_z",
            "linear_x", "linear_y", "angular_z", "speed_abs", "dt_ms",
            "sample_index", "interval_ms", "outlier_flag",
        ]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            for s in samples:
                w.writerow({k: s.get(k, "") for k in fields})

    def _write_manifest(self, summary: dict):
        path = os.path.join(self._args.out_dir, "e5_stage2a_tracer1_run_manifest.json")
        manifest = {
            "run_id": self._args.run_id,
            "robot_id": self._args.robot_id,
            "cmd_topic": self._args.cmd_topic,
            "feedback_topic": self._args.feedback_topic,
            "feedback_type": self._args.feedback_type,
            "linear_x": self._args.linear_x,
            "command_duration_sec": self._args.command_duration_sec,
            "pre_wait_sec": self._args.pre_wait_sec,
            "pre_zero_sec": self._args.pre_zero_sec,
            "post_stop_sec": self._args.post_stop_sec,
            "motion_threshold": self._args.motion_threshold,
            "zero_threshold": self._args.zero_threshold,
            "pose_jump_threshold": self._args.pose_jump_threshold,
            "dry_run": self._args.dry_run,
            "execute": self._args.execute,
            "script_sha256": script_sha256(__file__),
            "start_time": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "output_files": [
                "e5_stage2a_tracer1_command_response.csv",
                "e5_stage2a_tracer1_feedback_samples.csv",
            ],
            "operator_confirmations": {},
            "notes": summary.get("notes", ""),
        }
        with open(path, "w") as f:
            json.dump(manifest, f, indent=2)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="E5 Stage 2A: tracer1 low-speed cmd-response calibration")
    p.add_argument("--robot-id", default="tracer1")
    p.add_argument("--cmd-topic", default="/tracer1/cmd_vel")
    p.add_argument("--feedback-topic", default="/Rigid17/pose")
    p.add_argument("--feedback-type", default="pose_stamped",
                   choices=["pose_stamped", "odometry", "command_loopback"])
    p.add_argument("--run-id", default="")
    p.add_argument("--out-dir", default="/tmp/e5_stage2a")
    p.add_argument("--linear-x", type=float, default=DEFAULT_LINEAR_SPEED)
    p.add_argument("--command-duration-sec", type=float, default=1.0)
    p.add_argument("--pre-wait-sec", type=float, default=1.0)
    p.add_argument("--pre-zero-sec", type=float, default=0.5)
    p.add_argument("--post-stop-sec", type=float, default=2.0)
    p.add_argument("--motion-threshold", type=float, default=0.01)
    p.add_argument("--zero-threshold", type=float, default=0.01)
    p.add_argument("--pose-jump-threshold", type=float, default=0.5)
    p.add_argument("--min-consecutive-motion-samples", type=int, default=3)
    p.add_argument("--min-consecutive-zero-samples", type=int, default=5)
    p.add_argument("--dry-run", action="store_true", default=True)
    p.add_argument("--execute", action="store_true", default=False)
    return p.parse_args()


def main():
    args = parse_args()

    if args.robot_id not in ALLOWED_ROBOTS:
        print(f"ERROR: robot-id {args.robot_id} not authorized. Allowed: {ALLOWED_ROBOTS}")
        return 1

    if args.linear_x > MAX_LINEAR_SPEED:
        print(f"ERROR: linear_x {args.linear_x} exceeds max {MAX_LINEAR_SPEED} m/s. Refusing.")
        return 1

    if not args.run_id:
        args.run_id = f"e5_stage2a_{args.robot_id}_{time.strftime('%Y%m%d_%H%M%S')}"

    if args.execute:
        args.dry_run = False
        args.execute = True

    if args.dry_run:
        print("DRY RUN: no command will be published")
    elif args.execute:
        print(f"EXECUTE MODE: low-speed cmd_vel will be published to {args.cmd_topic}")

    rclpy.init()
    node = CalibrationNode(args)
    summary = node.run()
    node.destroy_node()
    rclpy.shutdown()

    print("\n=== CALIBRATION SUMMARY ===")
    for k in ["status", "response_lag_ms", "stop_latency_ms", "cmd_echo_delay_ms",
              "max_observed_speed", "feedback_sample_count",
              "feedback_jitter_p50_ms", "feedback_jitter_p95_ms"]:
        print(f"  {k}: {summary.get(k, '')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
