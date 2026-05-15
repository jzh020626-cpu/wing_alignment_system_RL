# -*- coding: utf-8 -*-

import math
from typing import Tuple

from wing_alignment_system.drive_math import body_frame_error, clamp, distance_xy, wrap_angle, yaw_error_final
from wing_alignment_system.drive_terminal_controller import FinalStopController
from wing_alignment_system.drive_types import DriveCommand, DriverConfig, Goal2D, Pose2D


class PrecisionController:
    """
    终端精定位控制器（被 precision_mode 触发）：
    - 比 coarse 更柔和
    - 不使用近区最小非零速度
    - 加入最终停车锁区
    """

    def __init__(self, cfg: DriverConfig):
        self.cfg = cfg
        self.stopper = FinalStopController(cfg)

    def reset(self):
        self.stopper.reset()

    def compute(self, pose: Pose2D, goal: Goal2D, now_sec: float, align_yaw_at_goal: bool) -> Tuple[DriveCommand, bool, bool]:
        dx_local, dy_local = body_frame_error(pose, goal)
        dx_world = goal.x - pose.x
        dy_world = goal.y - pose.y
        dist = distance_xy(pose.x, pose.y, goal.x, goal.y)
        relaxed_active = dist <= max(self.cfg.pos_tol * 3.0, self.cfg.final_stop_radius * 4.0)

        if self.stopper.should_hold(pose, goal, now_sec, align_yaw_at_goal):
            return DriveCommand(v=0.0, w=0.0), True, relaxed_active

        heading_to_goal = math.atan2(dy_world, dx_world) if dist > 1e-9 else pose.yaw
        heading_err = wrap_angle(heading_to_goal - pose.yaw)

        yaw_err = yaw_error_final(pose, goal) if align_yaw_at_goal else 0.0

        if dx_local < -0.02 and not relaxed_active:
            w = clamp(self.cfg.terminal_k_heading * heading_err, -self.cfg.terminal_w_max, self.cfg.terminal_w_max)
            if abs(heading_err) < math.radians(self.cfg.terminal_deadband_yaw_deg):
                w = 0.0
            return DriveCommand(v=0.0, w=w), False, relaxed_active

        rotate_only = (not relaxed_active) and abs(heading_err) > math.radians(self.cfg.terminal_rotate_only_deg)

        if rotate_only:
            v = 0.0
        else:
            v = self.cfg.terminal_k_x * dx_local
            v = clamp(v, 0.0, self.cfg.terminal_v_max)

            if not relaxed_active:
                heading_scale = max(0.0, math.cos(min(abs(heading_err), math.pi / 2.0)))
                v *= heading_scale

            if abs(dx_local) < self.cfg.terminal_deadband_x:
                v = 0.0

        w = self.cfg.terminal_k_y * dy_local + self.cfg.terminal_k_yaw * yaw_err
        w = clamp(w, -self.cfg.terminal_w_max, self.cfg.terminal_w_max)

        if abs(dy_local) < self.cfg.terminal_deadband_y and abs(yaw_err) < math.radians(self.cfg.terminal_deadband_yaw_deg):
            w = 0.0

        return DriveCommand(v=v, w=w), False, relaxed_active
