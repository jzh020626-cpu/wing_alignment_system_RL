#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass
import select
import sys
import termios
import time
import tty
from typing import Dict, Iterable, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover - allows importing helpers without ROS installed
    rclpy = None
    Node = object  # type: ignore[misc,assignment]
    Twist = None


DEFAULT_ROBOTS: Tuple[str, str, str] = ("tracer1", "tracer2", "tracer3")
DEFAULT_TOPIC_SUFFIX = "cmd_vel_desired"
DEFAULT_LINEAR_SPEED = 0.08
DEFAULT_ANGULAR_SPEED = 0.80
DEFAULT_KEY_HOLD_TIMEOUT_SEC = 0.25


@dataclass(frozen=True)
class MotionCommand:
    linear: float
    angular: float


MOVE_BINDINGS: Dict[str, MotionCommand] = {
    "u": MotionCommand(1.0, 1.0),
    "i": MotionCommand(1.0, 0.0),
    "o": MotionCommand(1.0, -1.0),
    "j": MotionCommand(0.0, 1.0),
    "k": MotionCommand(0.0, 0.0),
    "l": MotionCommand(0.0, -1.0),
    "m": MotionCommand(-1.0, -1.0),
    ",": MotionCommand(-1.0, 0.0),
    ".": MotionCommand(-1.0, 1.0),
}

SPEED_BINDINGS: Dict[str, Tuple[float, float]] = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


def build_topic_names(
    robots: Iterable[str],
    topic_suffix: str = DEFAULT_TOPIC_SUFFIX,
) -> Tuple[str, ...]:
    suffix = str(topic_suffix).strip().strip("/")
    return tuple(f"/{robot}/{suffix}" for robot in robots)


def key_to_motion(key: str) -> Optional[MotionCommand]:
    return MOVE_BINDINGS.get(key)


def read_key(settings, timeout_sec: float = 0.1) -> str:
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], max(0.0, float(timeout_sec)))
    key = sys.stdin.read(1) if readable else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def make_help_text() -> str:
    return (
        "同时控制 tracer1 / tracer2 / tracer3\n"
        "--------------------------------------------------\n"
        "移动按键:\n"
        "   u    i    o\n"
        "   j    k    l\n"
        "   m    ,    .\n"
        "\n"
        "i/, 前进/后退\n"
        "j/l 左转/右转\n"
        "u/o/m/. 组合移动\n"
        "松开移动键约 0.25s 后自动停车\n"
        "k 或 空格 立即停车\n"
        "\n"
        "速度调节:\n"
        "q/z 同时增减线速度和角速度\n"
        "w/x 只增减线速度\n"
        "e/c 只增减角速度\n"
        "\n"
        "Ctrl-C 退出\n"
    )


class MultiTracerKeyboardControl(Node):
    def __init__(self) -> None:
        super().__init__("multi_tracer_keyboard_control")

        robots_param = self.declare_parameter("robots", list(DEFAULT_ROBOTS)).value
        self.robots = tuple(str(robot) for robot in robots_param)
        self.topic_suffix = str(
            self.declare_parameter("topic_suffix", DEFAULT_TOPIC_SUFFIX).value
        ).strip().strip("/")
        self.linear_speed = float(
            self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED).value
        )
        self.angular_speed = float(
            self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED).value
        )
        self.key_hold_timeout_sec = float(
            self.declare_parameter("key_hold_timeout_sec", DEFAULT_KEY_HOLD_TIMEOUT_SEC).value
        )
        self.publish_hz = float(self.declare_parameter("publish_hz", 10.0).value)

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_motion_time_sec: Optional[float] = None

        self.cmd_publishers = {
            robot: self.create_publisher(Twist, f"/{robot}/{self.topic_suffix}", 10)
            for robot in self.robots
        }
        self.timer = self.create_timer(
            1.0 / max(1.0, self.publish_hz),
            self.publish_current_command,
        )

        topic_text = ", ".join(build_topic_names(self.robots, self.topic_suffix))
        self.get_logger().info(
            f"键盘三车控制已启动，目标话题: {topic_text}, "
            f"key_hold_timeout={self.key_hold_timeout_sec:.2f}s"
        )

    def update_motion(self, motion: MotionCommand) -> None:
        self.current_linear = float(motion.linear) * self.linear_speed
        self.current_angular = float(motion.angular) * self.angular_speed
        self.last_motion_time_sec = time.monotonic()
        self.publish_current_command()

    def scale_speeds(self, linear_scale: float, angular_scale: float) -> None:
        self.linear_speed *= float(linear_scale)
        self.angular_speed *= float(angular_scale)
        self.get_logger().info(
            f"当前速度: linear={self.linear_speed:.3f} m/s, "
            f"angular={self.angular_speed:.3f} rad/s"
        )

    def stop(self) -> None:
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_motion_time_sec = None
        self.publish_current_command()

    @staticmethod
    def is_motion_active(
        last_motion_time_sec: Optional[float],
        now_sec: float,
        key_hold_timeout_sec: float,
    ) -> bool:
        if last_motion_time_sec is None:
            return False
        return (float(now_sec) - float(last_motion_time_sec)) <= float(key_hold_timeout_sec)

    def publish_current_command(self) -> None:
        motion_active = self.is_motion_active(
            self.last_motion_time_sec,
            time.monotonic(),
            self.key_hold_timeout_sec,
        )
        msg = Twist()
        if motion_active:
            msg.linear.x = float(self.current_linear)
            msg.angular.z = float(self.current_angular)
        else:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.last_motion_time_sec = None
        for publisher in self.cmd_publishers.values():
            publisher.publish(msg)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run multi_tracer_keyboard_control")

    settings = termios.tcgetattr(sys.stdin)
    print(make_help_text())

    rclpy.init(args=args)
    node = MultiTracerKeyboardControl()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = read_key(settings)

            if key == "\x03":
                break

            if key in ("k", " "):
                node.stop()
                continue

            motion = key_to_motion(key)
            if motion is not None:
                node.update_motion(motion)
                continue

            speed_scale = SPEED_BINDINGS.get(key)
            if speed_scale is not None:
                node.scale_speeds(*speed_scale)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
