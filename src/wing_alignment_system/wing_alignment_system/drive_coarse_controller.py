# -*- coding: utf-8 -*-

import math
from typing import Tuple

from wing_alignment_system.drive_math import clamp, distance_xy, yaw_error_final, yaw_error_to_goal
from wing_alignment_system.drive_types import DriveCommand, DriverConfig, Goal2D, Pose2D


class CoarseController:
    """
    粗导航控制器：
    phase 0: 先转向到大致朝向
    phase 1: 朝目标点前进
    phase 2: 到点后补最终 yaw
    phase 3: reached
    """

    def __init__(self, cfg: DriverConfig):
        self.cfg = cfg

    def reset(self):
        pass

    def compute(self, pose: Pose2D, goal: Goal2D, phase: int) -> Tuple[DriveCommand, int, bool]:
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        dist = distance_xy(pose.x, pose.y, goal.x, goal.y)
        near = dist <= self.cfg.slow_radius

        pos_tol = self.cfg.pos_tol
        yaw_to_goal_err = yaw_error_to_goal(pose, goal)

        if dist < max(pos_tol * 1.5, 0.03):
            yaw_to_goal_err = 0.0

        if near and abs(yaw_to_goal_err) > math.radians(self.cfg.near_rotate_only_deg):
            phase = 0

        if phase == 0:
            if dist <= max(pos_tol, 0.02):
                phase = 2 if self.cfg.align_yaw_at_goal else 3
            elif abs(yaw_to_goal_err) < math.radians(self.cfg.coarse_heading_enter_deg):
                phase = 1
            else:
                w_cap = self.cfg.w_slow_max if near else self.cfg.w_max
                w = clamp(self.cfg.k_yaw * yaw_to_goal_err, -w_cap, w_cap)
                return DriveCommand(v=0.0, w=w), phase, False

        if phase == 1:
            if dist <= pos_tol:
                phase = 2 if self.cfg.align_yaw_at_goal else 3
            else:
                lookahead = self.cfg.Ld_near if near else self.cfg.Ld
                lookahead = max(1e-6, lookahead)

                v_cap = self.cfg.v_slow_max if near else self.cfg.v_max
                w_cap = self.cfg.w_slow_max if near else self.cfg.w_max

                k_dist = self.cfg.coarse_k_dist * self.cfg.v_nominal / lookahead
                v = clamp(k_dist * dist, 0.0, v_cap)

                v_min = self.cfg.v_min_near if near else self.cfg.v_min_far
                if v > 0.0 and v < v_min:
                    v = v_min

                heading_scale = max(0.15, math.cos(min(abs(yaw_to_goal_err), math.pi / 2.0)))
                v *= heading_scale

                w = clamp(self.cfg.k_yaw * yaw_to_goal_err, -w_cap, w_cap)
                return DriveCommand(v=v, w=w), phase, False

        if phase == 2:
            yaw_err = yaw_error_final(pose, goal)
            if abs(yaw_err) <= math.radians(self.cfg.yaw_tol_deg):
                phase = 3
            else:
                w_cap = self.cfg.w_slow_max if near else self.cfg.w_max
                w = clamp(self.cfg.k_yaw * yaw_err, -w_cap, w_cap)
                return DriveCommand(v=0.0, w=w), phase, False

        if phase == 3:
            return DriveCommand(v=0.0, w=0.0), 3, True

        return DriveCommand(v=0.0, w=0.0), phase, False
