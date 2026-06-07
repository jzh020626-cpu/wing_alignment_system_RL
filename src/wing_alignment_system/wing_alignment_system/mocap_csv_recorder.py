#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from datetime import datetime
from functools import partial
from typing import Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from wing_alignment_system.common_async_csv import AsyncCsvLogger
from wing_alignment_system.mission_geometry import extract_mocap_yaw_rad, map_mocap_xy


MM_TO_M = 0.001
DEFAULT_ROBOT_NAMES = ["tracer1", "tracer2", "tracer3"]
DEFAULT_MOCAP_TOPICS = {
    "tracer1": "/Rigid17/pose",
    "tracer2": "/Rigid14/pose",
    "tracer3": "/Rigid15/pose",
}
CSV_FIELDS = [
    "ros_time_sec",
    "recv_wall_time_sec",
    "robot_name",
    "topic",
    "seq",
    "raw_x_mm",
    "raw_y_mm",
    "raw_z_mm",
    "raw_qx",
    "raw_qy",
    "raw_qz",
    "raw_qw",
    "world_x_m",
    "world_y_m",
    "yaw_rad",
    "yaw_deg",
]


def default_csv_path(timestamp_str: str | None = None) -> str:
    stamp = timestamp_str or datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"/tmp/three_tracer_mocap_{stamp}.csv"


def _normalize_csv_path(path: str) -> str:
    path = str(path or "").strip()
    if not path:
        return default_csv_path()
    return os.path.abspath(path)


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0.0)) + float(getattr(stamp, "nanosec", 0.0)) * 1e-9


def build_csv_row(
    *,
    robot_name: str,
    topic: str,
    msg,
    seq: int,
    recv_wall_time_sec: float | None = None,
    mocap_yaw_mode: str = "legacy_deg_y",
    flip_heading_sign: bool = False,
    heading_deg_bias: float = 0.0,
    swap_xz: bool = False,
    negate_x: bool = False,
    negate_z: bool = True,
) -> dict:
    recv_sec = float(time.time() if recv_wall_time_sec is None else recv_wall_time_sec)
    raw_x_mm = float(msg.pose.position.x)
    raw_y_mm = float(msg.pose.position.y)
    raw_z_mm = float(msg.pose.position.z)
    raw_qx = float(msg.pose.orientation.x)
    raw_qy = float(msg.pose.orientation.y)
    raw_qz = float(msg.pose.orientation.z)
    raw_qw = float(msg.pose.orientation.w)

    world_x_m, world_y_m = map_mocap_xy(
        raw_x_mm * MM_TO_M,
        raw_z_mm * MM_TO_M,
        swap_xz=swap_xz,
        negate_x=negate_x,
        negate_z=negate_z,
    )
    yaw_rad = extract_mocap_yaw_rad(
        raw_qx,
        raw_qy,
        raw_qz,
        raw_qw,
        mode=mocap_yaw_mode,
        flip_heading_sign=flip_heading_sign,
        heading_deg_bias=heading_deg_bias,
    )
    return {
        "ros_time_sec": _stamp_to_sec(getattr(getattr(msg, "header", None), "stamp", None)),
        "recv_wall_time_sec": recv_sec,
        "robot_name": str(robot_name),
        "topic": str(topic),
        "seq": int(seq),
        "raw_x_mm": raw_x_mm,
        "raw_y_mm": raw_y_mm,
        "raw_z_mm": raw_z_mm,
        "raw_qx": raw_qx,
        "raw_qy": raw_qy,
        "raw_qz": raw_qz,
        "raw_qw": raw_qw,
        "world_x_m": world_x_m,
        "world_y_m": world_y_m,
        "yaw_rad": yaw_rad,
        "yaw_deg": yaw_rad * 180.0 / 3.141592653589793,
    }


class MocapCsvRecorder(Node):
    def __init__(self):
        super().__init__("mocap_csv_recorder")

        self.declare_parameter("robot_names", DEFAULT_ROBOT_NAMES)
        self.declare_parameter(
            "robot_mocap_topics",
            [DEFAULT_MOCAP_TOPICS[rn] for rn in DEFAULT_ROBOT_NAMES],
        )
        self.declare_parameter("csv_path", "")
        self.declare_parameter("mocap_yaw_mode", "legacy_deg_y")
        self.declare_parameter("flip_heading_sign", False)
        self.declare_parameter("heading_deg_bias", 0.0)
        self.declare_parameter("swap_xz", False)
        self.declare_parameter("negate_x", False)
        self.declare_parameter("negate_z", True)

        names = [str(x) for x in self.get_parameter("robot_names").value]
        if not names:
            names = list(DEFAULT_ROBOT_NAMES)
        raw_topics = [str(x) for x in self.get_parameter("robot_mocap_topics").value]
        if len(raw_topics) != len(names):
            self.get_logger().warn(
                f"[MOCAP_CSV] robot_mocap_topics len={len(raw_topics)} != robot_names len={len(names)}; "
                "fallback to default topic map"
            )
            raw_topics = [DEFAULT_MOCAP_TOPICS.get(rn, "") for rn in names]

        self.robot_names = names
        self.robot_topics = {rn: tp for rn, tp in zip(names, raw_topics) if tp}
        self.csv_path = _normalize_csv_path(str(self.get_parameter("csv_path").value))
        self.mocap_yaw_mode = str(self.get_parameter("mocap_yaw_mode").value).strip().lower() or "legacy_deg_y"
        self.flip_heading_sign = bool(self.get_parameter("flip_heading_sign").value)
        self.heading_deg_bias = float(self.get_parameter("heading_deg_bias").value)
        self.swap_xz = bool(self.get_parameter("swap_xz").value)
        self.negate_x = bool(self.get_parameter("negate_x").value)
        self.negate_z = bool(self.get_parameter("negate_z").value)

        self.csv_logger = AsyncCsvLogger(self.csv_path, CSV_FIELDS)
        self._closed = False
        self._seq_by_robot: Dict[str, int] = {rn: 0 for rn in self.robot_names}
        self._subs: List = []

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for robot_name, topic in self.robot_topics.items():
            self._subs.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    partial(self._pose_cb, robot_name, topic),
                    qos_best_effort,
                )
            )

        self.get_logger().warn(
            f"[MOCAP_CSV] recording {len(self.robot_topics)} topics to {self.csv_path}"
        )

    def _pose_cb(self, robot_name: str, topic: str, msg: PoseStamped):
        self._seq_by_robot[robot_name] += 1
        row = build_csv_row(
            robot_name=robot_name,
            topic=topic,
            msg=msg,
            seq=self._seq_by_robot[robot_name],
            recv_wall_time_sec=time.time(),
            mocap_yaw_mode=self.mocap_yaw_mode,
            flip_heading_sign=self.flip_heading_sign,
            heading_deg_bias=self.heading_deg_bias,
            swap_xz=self.swap_xz,
            negate_x=self.negate_x,
            negate_z=self.negate_z,
        )
        self.csv_logger.log(row)

    def _close_csv_logger(self):
        if self._closed:
            return
        self._closed = True
        self.csv_logger.close()
        self.get_logger().warn(f"[MOCAP_CSV] saved to {self.csv_path}")

    def destroy_node(self):
        self._close_csv_logger()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MocapCsvRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
