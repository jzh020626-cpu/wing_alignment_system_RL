# -*- coding: utf-8 -*-

from dataclasses import dataclass, field
from typing import Optional, Tuple, TYPE_CHECKING, List

if TYPE_CHECKING:
    from wing_alignment_system.target_estimator import TargetEstimator


@dataclass
class ToolOffset:
    x: float
    y: float
    yaw_deg: float


@dataclass
class RobotRuntime:
    reached: bool = False
    confirmed: bool = False
    last_goal_epoch: float = 0.0

    local_state: str = "IDLE"
    align_done: bool = False
    align_done_epoch: float = 0.0
    ready_to_lift: bool = False
    ready_epoch: float = 0.0
    lifting: bool = False
    transporting: bool = False
    transport_dispatched: bool = False
    transport_arrived: bool = False
    transport_settled: bool = False
    transport_failed: bool = False
    transport_start_stamp: float = 0.0
    transport_arrive_stamp: float = 0.0
    transport_settle_stamp: float = 0.0
    formation_error_m: float = 0.0
    formation_error_yaw_deg: float = 0.0
    group_stop_reason: str = ""
    finished: bool = False
    faulted: bool = False
    fault_reason: str = ""

    micro_i: int = 0
    micro_attempts: int = 0
    seg_i: int = 0
    segs: Optional[List[Tuple[float, float]]] = None
    dwell_start: float = 0.0
    dwell_locked: bool = False

    final_target: Optional[Tuple[float, float]] = None
    staging_target: Optional[Tuple[float, float]] = None
    transport_target: Optional[Tuple[float, float]] = None
    goal_kind: str = "FINAL"
    staged: bool = False
    entered: bool = False

    delta_armed_since: float = 0.0
    last_delta_stamp: Optional[float] = None
    delta_latest: Optional[Tuple[float, float, float]] = None
    te: Optional["TargetEstimator"] = None

    fine_active: bool = False
    fine_goal_inflight: bool = False
    fine_settle_until: float = 0.0

    gate_stopped: bool = False
    gate_hold_until: float = 0.0
    slow_mode: bool = False

    qr_zero_pending: bool = False
    qr_zero_done: bool = False
    qr_zero_req_epoch: float = 0.0

    locked_yaw: Optional[float] = None
    xy_stable_count: int = 0

    last_comp_pub_epoch: float = 0.0
    comp_active: bool = False
    comp_last_vx: float = 0.0
    comp_last_vy: float = 0.0
    comp_last_vz: float = 0.0

    slide_pos: Optional[Tuple[float, float, float]] = None
    slide_vel: Optional[Tuple[float, float, float]] = None
    slide_reached: bool = False
    slide_pos_stamp: float = 0.0
    odom_twist_body: Optional[Tuple[float, float, float]] = None
    odom_stamp: float = 0.0
    imu_wz: Optional[float] = None
    imu_stamp: float = 0.0
    mocap_twist_world: Optional[Tuple[float, float]] = None
    mocap_twist_body: Optional[Tuple[float, float]] = None
    mocap_wz: Optional[float] = None
    mocap_twist_stamp: float = 0.0
    last_mocap_xy: Optional[Tuple[float, float]] = None
    last_mocap_yaw: Optional[float] = None
    last_mocap_stamp: float = 0.0

    recenter_done: bool = False
    recenter_target: Optional[Tuple[float, float, float]] = None
    transport_center_ref: Optional[Tuple[float, float, float]] = None

    first_qr_locked: bool = False
    first_qr_lock_epoch: float = 0.0

    raw_qr_seen_stamp: float = 0.0
    raw_qr_hit_count: int = 0
    raw_qr_last_hit_stamp: float = 0.0
    raw_qr_pose: Optional[Tuple[float, float, float]] = None

    direct_align_phase: str = "idle"
    direct_align_retry: int = 0
    direct_align_epoch: float = 0.0
    direct_align_xy_time: float = 0.0
    direct_align_z_time: float = 0.0
    direct_align_pending_z_mm: float = 0.0
    direct_align_started_at: float = 0.0
    direct_align_xy_start_pos: Optional[Tuple[float, float, float]] = None
    direct_align_xy_cmd_x_mm: float = 0.0
    direct_align_xy_cmd_y_mm: float = 0.0
    direct_align_z_cmd_mm: float = 0.0
    direct_align_positive_z_attempted: bool = False
    direct_align_small_residual_count: int = 0
    direct_align_z_done: bool = False
    direct_align_force_monitor_latched: bool = False
    direct_align_force_contact_latched: bool = False
    direct_align_force_contact_count: int = 0
    direct_align_force_last_eval_stamp: float = 0.0
    direct_align_post_contact_hold_start: float = 0.0
    direct_align_post_contact_z_ref_mm: Optional[float] = None

    sync_wait_qr: bool = False
    sync_wait_qr_epoch: float = 0.0

    force_filtered: Optional[Tuple[float, float, float, float]] = None
    force_f: Optional[Tuple[float, float, float, float]] = None
    force_stamp: float = 0.0
    force_hist: List[Tuple[float, float, float, float, float]] = field(default_factory=list)
    contact_confirmed: bool = False
    force_contact_epoch: float = 0.0
    post_contact_latched: bool = False
    post_contact_epoch: float = 0.0
    post_contact_delta: Optional[Tuple[float, float, float]] = None

    level_active: bool = False
    level_done: bool = False
    level_target_z_mm: Optional[float] = None
    level_z_done: bool = False
    level_z_target_mm: Optional[float] = None
    level_z_start_epoch: float = 0.0

    loaded_ref_captured: bool = False
    chassis_check_last_xy: Optional[Tuple[float, float]] = None
    chassis_check_last_stamp: float = 0.0
    last_cmd_v: float = 0.0
    last_cmd_w: float = 0.0

    raw_qr_last_reject_reason: str = ""
    # transport 底盘前馈是否激活（用于 freshness 状态快照）
    # odom / imu 来源的时间戳（由 _apply_odom_sample_locked / _apply_imu_sample_locked 写入）
    last_delta_receive_stamp: float = 0.0
    last_delta_stamp_missing: bool = False
    raw_qr_receive_stamp: float = 0.0
    raw_qr_last_source_stamp: float = 0.0
    raw_qr_stamp_missing: bool = False

    load_stable_vel_ok: bool = False
    load_stable_force_fresh_ok: bool = False
    load_stable_delta_fresh_ok: bool = False
    load_stable_force_slope_ok: bool = False
    load_stable_residual_ok: bool = False
