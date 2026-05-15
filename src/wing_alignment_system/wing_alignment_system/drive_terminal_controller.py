# -*- coding: utf-8 -*-

import math

from wing_alignment_system.drive_math import distance_xy, yaw_error_final
from wing_alignment_system.drive_types import DriverConfig, Goal2D, Pose2D


class FinalStopController:
    """
    终端停车锁区：
    一旦进入停车判据，连续 final_stop_hold_sec 保持零速，
    避免“到点后还轻轻拱/摆一下”。
    """

    def __init__(self, cfg: DriverConfig):
        self.cfg = cfg
        self._hold_until = 0.0

    def reset(self):
        self._hold_until = 0.0

    def should_hold(self, pose: Pose2D, goal: Goal2D, now_sec: float, align_yaw_at_goal: bool) -> bool:
        if now_sec < self._hold_until:
            return True

        dist = distance_xy(pose.x, pose.y, goal.x, goal.y)
        yaw_err = yaw_error_final(pose, goal)

        yaw_ok = (not align_yaw_at_goal) or (abs(yaw_err) <= math.radians(self.cfg.final_stop_yaw_tol_deg))
        if dist <= self.cfg.final_stop_radius and yaw_ok:
            self._hold_until = now_sec + self.cfg.final_stop_hold_sec
            return True

        return False
