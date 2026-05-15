# -*- coding: utf-8 -*-

from dataclasses import dataclass


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Goal2D:
    x: float
    y: float
    yaw: float


@dataclass
class PendingGoal:
    x: float
    y: float
    yaw_rad: float
    yaw_deg_in_msg: float


@dataclass
class DriveCommand:
    v: float = 0.0
    w: float = 0.0


@dataclass
class DriverConfig:
    v_nominal: float
    v_max: float
    w_max: float
    Ld: float
    Ld_near: float
    pos_tol: float
    k_yaw: float
    yaw_tol_deg: float
    slow_radius: float
    v_slow_max: float
    w_slow_max: float
    v_min_far: float
    v_min_near: float
    near_rotate_only_deg: float
    coarse_heading_enter_deg: float
    coarse_k_dist: float
    align_yaw_at_goal: bool
    goal_reached_latch_sec: float
    stop_hold_sec: float
    pose_timeout_sec: float
    final_stop_radius: float
    final_stop_yaw_tol_deg: float
    final_stop_hold_sec: float
    terminal_v_max: float
    terminal_w_max: float
    terminal_k_x: float
    terminal_k_y: float
    terminal_k_yaw: float
    terminal_k_heading: float
    terminal_rotate_only_deg: float
    terminal_deadband_x: float
    terminal_deadband_y: float
    terminal_deadband_yaw_deg: float
    dv_max: float
    dw_max: float
