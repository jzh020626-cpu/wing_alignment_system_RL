# -*- coding: utf-8 -*-

import math
from typing import Tuple

from wing_alignment_system.drive_types import Pose2D, Goal2D


def wrap_angle(a: float) -> float:
    if math.isnan(a) or math.isinf(a):
        return 0.0
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def distance_xy(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def world_error(pose: Pose2D, goal: Goal2D) -> Tuple[float, float]:
    return goal.x - pose.x, goal.y - pose.y


def body_frame_error(pose: Pose2D, goal: Goal2D) -> Tuple[float, float]:
    dx = goal.x - pose.x
    dy = goal.y - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    dx_local = c * dx + s * dy
    dy_local = -s * dx + c * dy
    return dx_local, dy_local


def yaw_error_to_goal(pose: Pose2D, goal: Goal2D) -> float:
    dx = goal.x - pose.x
    dy = goal.y - pose.y
    if math.hypot(dx, dy) < 1e-9:
        return 0.0
    return wrap_angle(math.atan2(dy, dx) - pose.yaw)


def yaw_error_final(pose: Pose2D, goal: Goal2D) -> float:
    return wrap_angle(goal.yaw - pose.yaw)
