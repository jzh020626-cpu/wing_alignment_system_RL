#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import time
import sys
import threading
import traceback
from collections import deque
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from base_interfaces_demo.msg import MotorCommand, MotorStatus

from wing_alignment_system.target_estimator import TargetEstimator
from wing_alignment_system.mission_types import ToolOffset, RobotRuntime
from wing_alignment_system.mission_geometry import (
    wrap_angle_rad,
    map_mocap_xy,
    micro_offsets,
    _now_sec,
    extract_mocap_yaw_rad,
)
from wing_alignment_system.mission_state_helpers import MissionStateHelpersMixin
from wing_alignment_system.mission_dispatcher import MissionDispatcherMixin
from wing_alignment_system.mission_gate_manager import MissionGateManagerMixin
from wing_alignment_system.mission_slide_manager import MissionSlideManagerMixin
from wing_alignment_system.mission_robot_step import MissionRobotStepMixin
from wing_alignment_system.common_rt import EventQueue, FixedRateLoop, LatestValueBuffer
from wing_alignment_system.common_async_csv import AsyncCsvLogger

try:
    from wing_mechanism_bench.trace_utils import (
        TRACE_EVENT_PATH_CONTROL,
        build_trace_publisher,
    )
except Exception:  # pragma: no cover - bench package is optional
    TRACE_EVENT_PATH_CONTROL = "transport_control_events"

    class _NullTracePublisher:
        def emit(self, *args, **kwargs):
            return None

    def build_trace_publisher(*args, **kwargs):
        return _NullTracePublisher()


MISSION_RUNTIME_FIELDS = [
    'run_id',
    'timestamp',
    'mission_state',
    'task_phase',
    'precision_mode',
    'robot_id',
    'team_scope',
    'Delta_eff_proxy_ms',
    'S_eff',
    'F_eff',
    'base_authority_weight',
    'slide_authority_weight',
    'authority_policy_mode',
    'freeze_state',
    'watchdog_or_safe_state',
    'docking_residual_proxy',
    'slide_residual_proxy',
    'support_residual_proxy',
    'safe_abort_reason',
    'event_type',
    'event_note',
]


class MissionCoordinator(
    Node,
    MissionStateHelpersMixin,
    MissionDispatcherMixin,
    MissionGateManagerMixin,
    MissionSlideManagerMixin,
    MissionRobotStepMixin,
):
    def __init__(self):
        super().__init__('mission_coordinator')
        self.get_logger().warn('>>> MISSION COORDINATOR STARTING - STANDBY FOR MOCAP <<<')
        sys.stdout.flush()

        # ===== 基础参数 =====
        self.declare_parameter('wing_mocap_topic', '/Rigid8/pose')
        self.declare_parameter('mm_to_m', 0.001)
        self.declare_parameter('swap_xz', False)
        self.declare_parameter('negate_x', False)
        self.declare_parameter('negate_z', True)
        self.declare_parameter('mocap_yaw_mode', 'legacy_deg_y')
        self.declare_parameter('flip_heading_sign', False)
        self.declare_parameter('heading_deg_bias', 0.0)
        self.declare_parameter('freeze_wing_on_start', True)
        self.declare_parameter('use_goal_yaw', True)
        self.declare_parameter('mocap_timeout_sec', 0.50)

        self.wing_mocap_topic = str(self.get_parameter('wing_mocap_topic').value)
        self.mm_to_m = float(self.get_parameter('mm_to_m').value)
        self.swap_xz = bool(self.get_parameter('swap_xz').value)
        self.negate_x = bool(self.get_parameter('negate_x').value)
        self.negate_z = bool(self.get_parameter('negate_z').value)
        self.mocap_yaw_mode = str(self.get_parameter('mocap_yaw_mode').value).strip().lower() or 'legacy_deg_y'
        if self.mocap_yaw_mode not in ('legacy_deg_y', 'quaternion', 'quat_auto'):
            self.get_logger().warn(
                f'[MOCAP_YAW] invalid mocap_yaw_mode={self.mocap_yaw_mode}, fallback to legacy_deg_y'
            )
            self.mocap_yaw_mode = 'legacy_deg_y'
        self.flip_heading_sign = bool(self.get_parameter('flip_heading_sign').value)
        self.heading_deg_bias = float(self.get_parameter('heading_deg_bias').value)
        self.freeze_wing_on_start = bool(self.get_parameter('freeze_wing_on_start').value)
        self.use_goal_yaw = bool(self.get_parameter('use_goal_yaw').value)
        self.mocap_timeout_sec = float(self.get_parameter('mocap_timeout_sec').value)

        # ===== 机器人列表 =====
        self.declare_parameter('robots', ['tracer1'])
        self.declare_parameter('dispatch_order', ['tracer1'])
        self.declare_parameter('robot_mocap_topics', [''])
        self.declare_parameter('robot_odom_topics', [''])
        self.declare_parameter('robot_imu_topics', [''])

        self.robots = [str(x) for x in self.get_parameter('robots').value]
        self.dispatch_order = [str(x) for x in self.get_parameter('dispatch_order').value]
        self.dispatch_order = [x for x in self.dispatch_order if x in self.robots] or list(self.robots)

        mocap_raw = [str(x) for x in self.get_parameter('robot_mocap_topics').value]
        self.robot_mocap_topics = [x for x in mocap_raw if x]
        topic_map = {'tracer1': '/Rigid17/pose', 'tracer2': '/Rigid14/pose', 'tracer3': '/Rigid15/pose'}
        if len(self.robot_mocap_topics) != len(self.robots):
            self.robot_mocap_topics = [topic_map.get(rn, '/RigidBody1/pose') for rn in self.robots]

        # Odom / IMU input path is disabled for the current setup.
        # odom_raw = [str(x) for x in self.get_parameter('robot_odom_topics').value]
        # imu_raw = [str(x) for x in self.get_parameter('robot_imu_topics').value]
        # if len(odom_raw) == len(self.robots):
        #     self.robot_odom_topics = odom_raw
        # else:
        #     self.robot_odom_topics = [''] * len(self.robots)
        #
        # if len(imu_raw) == len(self.robots):
        #     self.robot_imu_topics = imu_raw
        # else:
        #     self.robot_imu_topics = [''] * len(self.robots)
        self.robot_odom_topics = [''] * len(self.robots)
        self.robot_imu_topics = [''] * len(self.robots)

        # ===== 承载点相对机翼偏置 =====
        self.declare_parameter('tool_offsets_flat', [0.0])
        flat = [float(x) for x in self.get_parameter('tool_offsets_flat').value]
        n = len(self.robots)
        if len(flat) == 1 and abs(flat[0]) < 1e-12:
            flat = [0.0] * (3 * n)

        self.tool_offsets: Dict[str, ToolOffset] = {}
        for i, rn in enumerate(self.robots):
            self.tool_offsets[rn] = ToolOffset(
                flat[3 * i + 0],
                flat[3 * i + 1],
                flat[3 * i + 2]
            )

        # ===== staging / entry / path =====
        self.declare_parameter('staging_enable', True)
        self.declare_parameter('staging_offsets_flat', [0.0])
        self.declare_parameter('entry_enable', True)
        self.declare_parameter('entry_zone_r', 2.2)
        self.declare_parameter('path_mode', 'x_first')
        self.declare_parameter('staging_path_mode', '')
        self.declare_parameter('final_path_mode', '')
        self.declare_parameter('transport_path_mode', '')
        self.declare_parameter('path_min_seg_m', 0.03)
        self.declare_parameter('reach_min_delay_sec', 0.25)
        self.declare_parameter('waypoint_dwell_sec', 2.0)
        self.declare_parameter('dwell_lock_stop', True)
        self.declare_parameter('final_precision_enable', True)
        self.declare_parameter('final_precision_robots', ['tracer1', 'tracer2', 'tracer3'])
        self.declare_parameter('final_precision_window_m', 0.35)
        self.declare_parameter('micro_mode', 'cross')
        self.declare_parameter('micro_radii_m', [0.0, 0.08, 0.15])
        self.declare_parameter('micro_max_attempts', 4)
        self.declare_parameter('micro_min_move_m', 0.07)
        self.declare_parameter('base_micro_search_enable', False)
        self.declare_parameter('wait_qr_fail_timeout_sec', 20.0)
        self.declare_parameter('start_mode', 'parallel')
        self.declare_parameter('cooperative_common_x_backoff_m', 0.80)
        self.declare_parameter('cooperative_anchor_robot', 'tracer1')
        self.declare_parameter('cooperative_start_line_y', [-3.320, -2.320, -1.320])
        self.declare_parameter('cooperative_wait_tracer1_offset_y_m', 1.5)
        self.declare_parameter('cooperative_wait_side_mode', 'follow_tracer1')

        self.staging_enable = bool(self.get_parameter('staging_enable').value)
        sflat = [float(x) for x in self.get_parameter('staging_offsets_flat').value]
        if len(sflat) == 1 and abs(sflat[0]) < 1e-12:
            self.declare_parameter('staging_auto_r', 1.6)
            r = float(self.get_parameter('staging_auto_r').value)
            angs = [math.radians(30.0 + 120.0 * i) for i in range(n)]
            sflat = []
            for a in angs:
                sflat.extend([-r * math.cos(a), r * math.sin(a)])

        self.staging_offsets: Dict[str, Tuple[float, float]] = {}
        for i, rn in enumerate(self.robots):
            self.staging_offsets[rn] = (float(sflat[2 * i + 0]), float(sflat[2 * i + 1]))

        self.entry_enable = bool(self.get_parameter('entry_enable').value)
        self.entry_zone_r = float(self.get_parameter('entry_zone_r').value)

        self.path_mode = str(self.get_parameter('path_mode').value).lower().strip()
        self.staging_path_mode = str(self.get_parameter('staging_path_mode').value).lower().strip()
        self.final_path_mode = str(self.get_parameter('final_path_mode').value).lower().strip()
        self.transport_path_mode = str(self.get_parameter('transport_path_mode').value).lower().strip()
        self.path_min_seg_m = float(self.get_parameter('path_min_seg_m').value)
        self.reach_min_delay_sec = float(self.get_parameter('reach_min_delay_sec').value)
        self.waypoint_dwell_sec = float(self.get_parameter('waypoint_dwell_sec').value)
        self.dwell_lock_stop = bool(self.get_parameter('dwell_lock_stop').value)
        self.final_precision_enable = bool(self.get_parameter('final_precision_enable').value)
        self.final_precision_robots = [str(x) for x in self.get_parameter('final_precision_robots').value]
        self.final_precision_window_m = max(0.05, float(self.get_parameter('final_precision_window_m').value))
        self.micro_list = micro_offsets(
            str(self.get_parameter('micro_mode').value),
            [float(x) for x in self.get_parameter('micro_radii_m').value]
        )
        self.micro_max_attempts = int(self.get_parameter('micro_max_attempts').value)
        self.micro_min_move_m = float(self.get_parameter('micro_min_move_m').value)
        self.base_micro_search_enable = bool(self.get_parameter('base_micro_search_enable').value)
        self.wait_qr_fail_timeout_sec = float(self.get_parameter('wait_qr_fail_timeout_sec').value)
        self.start_mode = str(self.get_parameter('start_mode').value).lower().strip()
        self.cooperative_common_x_backoff_m = float(self.get_parameter('cooperative_common_x_backoff_m').value)
        self.cooperative_anchor_robot = str(self.get_parameter('cooperative_anchor_robot').value)
        start_line_y = [float(x) for x in self.get_parameter('cooperative_start_line_y').value]
        self.cooperative_wait_tracer1_offset_y_m = float(
            self.get_parameter('cooperative_wait_tracer1_offset_y_m').value
        )
        self.cooperative_wait_side_mode = str(
            self.get_parameter('cooperative_wait_side_mode').value
        ).lower().strip() or 'follow_tracer1'
        if len(start_line_y) != len(self.robots):
            self.get_logger().warn(
                f'[COOP_APPROACH] cooperative_start_line_y len={len(start_line_y)} '
                f'!= robots len={len(self.robots)}; fallback to zeros'
            )
            start_line_y = [0.0] * len(self.robots)
        if self.cooperative_anchor_robot not in self.robots:
            fallback_anchor = self.robots[0] if self.robots else 'tracer1'
            self.get_logger().warn(
                f'[COOP_APPROACH] invalid cooperative_anchor_robot={self.cooperative_anchor_robot}; '
                f'fallback to {fallback_anchor}'
            )
            self.cooperative_anchor_robot = fallback_anchor
        anchor_idx = self.robots.index(self.cooperative_anchor_robot) if self.robots else 0
        anchor_y0 = float(start_line_y[anchor_idx]) if start_line_y else 0.0
        self.cooperative_start_line_y_map = {
            rn: float(start_line_y[i]) for i, rn in enumerate(self.robots)
        }
        self.cooperative_line_offsets_m = {
            rn: float(start_line_y[i]) - anchor_y0 for i, rn in enumerate(self.robots)
        }
        if self.cooperative_wait_side_mode not in ('follow_tracer1',):
            self.get_logger().warn(
                f'[COOP_APPROACH] invalid cooperative_wait_side_mode={self.cooperative_wait_side_mode}; '
                f'fallback to follow_tracer1'
            )
            self.cooperative_wait_side_mode = 'follow_tracer1'
        self.entry_owner: Optional[str] = None

        self.path_mode = self._normalize_path_mode(self.path_mode, default='x_first')
        self.staging_path_mode = self._normalize_path_mode(self.staging_path_mode, default=self.path_mode)
        self.final_path_mode = self._normalize_path_mode(self.final_path_mode, default=self.path_mode)
        self.transport_path_mode = self._normalize_path_mode(self.transport_path_mode, default=self.path_mode)
        self.get_logger().warn(
            f'[PATH_MODE] default={self.path_mode} staging={self.staging_path_mode} '
            f'final={self.final_path_mode} transport={self.transport_path_mode}'
        )

        # ===== topic 模板 =====
        self.declare_parameter('delta_topic_template', '/{robot}/wing_alignment/delta')
        self.declare_parameter('raw_qr_topic_template', '/{robot}/object_position')
        self.declare_parameter('precision_topic_template', '/{robot}/precision_mode')
        self.declare_parameter('emergency_stop_topic', '/wing_alignment/emergency_stop')
        self.declare_parameter('force_contact_enable', False)
        self.declare_parameter('force_contact_topic_template', '/{slide}/force_contact')
        self.declare_parameter('alignment_vision_enable', True)

        self.delta_topic_template = str(self.get_parameter('delta_topic_template').value)
        self.raw_qr_topic_template = str(self.get_parameter('raw_qr_topic_template').value)
        self.precision_topic_template = str(self.get_parameter('precision_topic_template').value)
        self.emergency_stop_topic = str(self.get_parameter('emergency_stop_topic').value)
        self.force_contact_enable = bool(self.get_parameter('force_contact_enable').value)
        self.force_contact_topic_template = str(self.get_parameter('force_contact_topic_template').value)
        self.alignment_vision_enable = bool(self.get_parameter('alignment_vision_enable').value)

        # ===== delta / estimator =====
        self.declare_parameter('delta_arm_delay_sec', 0.3)
        self.declare_parameter('delta_max_age_sec', 2.0)
        self.declare_parameter('delta_frame_gap_max_sec', 0.25)
        self.declare_parameter('delta_stable_eps', 0.005)
        self.declare_parameter('delta_threshold_frames', 2)
        self.declare_parameter('delta_abs_max_m', 2.0)

        self.delta_arm_delay_sec = float(self.get_parameter('delta_arm_delay_sec').value)
        self.delta_max_age_sec = float(self.get_parameter('delta_max_age_sec').value)
        self.delta_frame_gap_max_sec = float(self.get_parameter('delta_frame_gap_max_sec').value)
        self.delta_stable_eps = float(self.get_parameter('delta_stable_eps').value)
        self.delta_threshold_frames = int(self.get_parameter('delta_threshold_frames').value)
        self.delta_abs_max_m = float(self.get_parameter('delta_abs_max_m').value)

        self.declare_parameter('te_min_confidence', 0.55)
        self.declare_parameter('te_ema_alpha', 0.45)
        self.declare_parameter('te_lost_timeout_sec', 0.8)
        self.declare_parameter('te_freeze_timeout_sec', 4.0)
        self.declare_parameter('te_jump_sigma', 0.02)

        self.te_min_confidence = float(self.get_parameter('te_min_confidence').value)
        self.te_ema_alpha = float(self.get_parameter('te_ema_alpha').value)
        self.te_lost_timeout_sec = float(self.get_parameter('te_lost_timeout_sec').value)
        self.te_freeze_timeout_sec = float(self.get_parameter('te_freeze_timeout_sec').value)
        self.te_jump_sigma = float(self.get_parameter('te_jump_sigma').value)

        # ===== raw QR 判据 =====
        self.declare_parameter('raw_qr_seen_timeout_sec', 0.6)
        self.declare_parameter('raw_qr_arm_delay_sec', 0.8)
        self.declare_parameter('raw_qr_accept_radius_m', 0.18)
        self.declare_parameter('raw_qr_min_hits', 6)
        self.declare_parameter('raw_qr_hit_timeout_sec', 0.20)
        self.declare_parameter('vision_stamp_offset_fallback_sec', 60.0)

        self.declare_parameter('raw_qr_min_x_m', -0.03)
        self.declare_parameter('raw_qr_max_x_m', 0.06)
        self.declare_parameter('raw_qr_max_abs_y_m', 0.04)
        self.declare_parameter('raw_qr_min_z_m', 0.10)
        self.declare_parameter('raw_qr_max_z_m', 0.35)

        self.raw_qr_seen_timeout_sec = float(self.get_parameter('raw_qr_seen_timeout_sec').value)
        self.raw_qr_arm_delay_sec = float(self.get_parameter('raw_qr_arm_delay_sec').value)
        self.raw_qr_accept_radius_m = float(self.get_parameter('raw_qr_accept_radius_m').value)
        self.raw_qr_min_hits = int(self.get_parameter('raw_qr_min_hits').value)
        self.raw_qr_hit_timeout_sec = float(self.get_parameter('raw_qr_hit_timeout_sec').value)
        self.vision_stamp_offset_fallback_sec = max(0.0, float(self.get_parameter('vision_stamp_offset_fallback_sec').value))

        self.raw_qr_min_x_m = float(self.get_parameter('raw_qr_min_x_m').value)
        self.raw_qr_max_x_m = float(self.get_parameter('raw_qr_max_x_m').value)
        self.raw_qr_max_abs_y_m = float(self.get_parameter('raw_qr_max_abs_y_m').value)
        self.raw_qr_min_z_m = float(self.get_parameter('raw_qr_min_z_m').value)
        self.raw_qr_max_z_m = float(self.get_parameter('raw_qr_max_z_m').value)

        # ===== 对位容差 =====
        self.declare_parameter('fine_enable', True)
        self.declare_parameter('fine_xy_tol_m', 0.005)
        self.declare_parameter('fine_z_tol_m', 0.005)
        self.declare_parameter('fine_xy_stable_frames', 5)

        self.fine_enable = bool(self.get_parameter('fine_enable').value)
        self.fine_xy_tol_m = float(self.get_parameter('fine_xy_tol_m').value)
        self.fine_z_tol_m = float(self.get_parameter('fine_z_tol_m').value)
        self.fine_xy_stable_frames = int(self.get_parameter('fine_xy_stable_frames').value)

        # ===== QR zero =====
        self.declare_parameter('qr_zero_enable', True)
        self.declare_parameter('qr_zero_wait_max_sec', 4.0)
        self.declare_parameter('qr_zero_retry_sec', 0.5)
        self.declare_parameter('wait_qr_hold_on_raw_qr_sec', 10.0)

        self.qr_zero_enable = bool(self.get_parameter('qr_zero_enable').value)
        self.qr_zero_wait_max_sec = float(self.get_parameter('qr_zero_wait_max_sec').value)
        self.qr_zero_retry_sec = float(self.get_parameter('qr_zero_retry_sec').value)
        self.wait_qr_hold_on_raw_qr_sec = float(self.get_parameter('wait_qr_hold_on_raw_qr_sec').value)

        # ===== approach slow =====
        self.declare_parameter('approach_slow_enable', False)
        self.declare_parameter('approach_slow_r', 1.5)
        self.declare_parameter('approach_slow_hyst', 0.3)

        self.approach_slow_enable = bool(self.get_parameter('approach_slow_enable').value)
        self.approach_slow_r = float(self.get_parameter('approach_slow_r').value)
        self.approach_slow_hyst = float(self.get_parameter('approach_slow_hyst').value)

        # ===== collision gate =====
        self.declare_parameter('gate_enable', True)
        self.declare_parameter('gate_near_wing_r', 2.0)
        self.declare_parameter('gate_dmin_far', 0.85)
        self.declare_parameter('gate_dresume_far', 1.10)
        self.declare_parameter('gate_dmin_near', 0.65)
        self.declare_parameter('gate_dresume_near', 0.80)
        self.declare_parameter('gate_hold_sec', 0.50)
        self.declare_parameter('gate_keep_one_moving', True)

        self.gate_enable = bool(self.get_parameter('gate_enable').value)
        self.gate_near_wing_r = float(self.get_parameter('gate_near_wing_r').value)
        self.gate_dmin_far = float(self.get_parameter('gate_dmin_far').value)
        self.gate_dresume_far = float(self.get_parameter('gate_dresume_far').value)
        self.gate_dmin_near = float(self.get_parameter('gate_dmin_near').value)
        self.gate_dresume_near = float(self.get_parameter('gate_dresume_near').value)
        self.gate_hold_sec = float(self.get_parameter('gate_hold_sec').value)
        self.gate_keep_one_moving = bool(self.get_parameter('gate_keep_one_moving').value)

        # ===== global mission =====
        self.declare_parameter('all_ready_hold_sec', 0.80)
        self.declare_parameter('sync_lift_settle_sec', 0.60)
        self.declare_parameter('transport_enable', True)
        self.declare_parameter('transport_goal_offset_xyyaw', [0.60, 0.0, 0.0])
        self.declare_parameter('transport_goal_refresh_sec', 0.50)
        self.declare_parameter('transport_settle_sec', 1.00)
        self.declare_parameter('transport_finish_pos_tol_m', 0.08)
        self.declare_parameter('transport_finish_yaw_tol_deg', 6.0)
        self.declare_parameter('transport_max_center_error_m', 0.20)
        self.declare_parameter('transport_abort_on_wing_pose_lost', True)
        self.declare_parameter('abort_on_any_fault', True)
        self.declare_parameter('loop_hz', 20.0)

        self.all_ready_hold_sec = float(self.get_parameter('all_ready_hold_sec').value)
        self.sync_lift_settle_sec = float(self.get_parameter('sync_lift_settle_sec').value)
        self.transport_enable = bool(self.get_parameter('transport_enable').value)

        tgoal = [float(x) for x in self.get_parameter('transport_goal_offset_xyyaw').value]
        while len(tgoal) < 3:
            tgoal.append(0.0)
        self.transport_goal_dx_m = float(tgoal[0])
        self.transport_goal_dy_m = float(tgoal[1])
        self.transport_goal_dyaw_deg = float(tgoal[2])
        self.transport_goal_refresh_sec = max(0.10, float(self.get_parameter('transport_goal_refresh_sec').value))
        self.transport_settle_sec = max(0.10, float(self.get_parameter('transport_settle_sec').value))
        self.transport_finish_pos_tol_m = max(0.01, float(self.get_parameter('transport_finish_pos_tol_m').value))
        self.transport_finish_yaw_tol_deg = max(0.5, float(self.get_parameter('transport_finish_yaw_tol_deg').value))
        self.transport_max_center_error_m = max(0.01, float(self.get_parameter('transport_max_center_error_m').value))
        self.transport_abort_on_wing_pose_lost = bool(self.get_parameter('transport_abort_on_wing_pose_lost').value)

        self.abort_on_any_fault = bool(self.get_parameter('abort_on_any_fault').value)
        self.loop_hz = max(1.0, float(self.get_parameter('loop_hz').value))

        # ===== workflow mode =====
        self.declare_parameter('workflow', 'full')
        workflow_raw = str(self.get_parameter('workflow').value).lower().strip()
        if workflow_raw == 'approach':
            self.get_logger().warn('[WORKFLOW] approach 已废弃，自动回退为 full')
            workflow_raw = 'full'
        if workflow_raw not in ('full', 'lift', 'transport'):
            self.get_logger().warn(f'[WORKFLOW] Unknown workflow={workflow_raw}, fallback to full')
            workflow_raw = 'full'
        self.workflow = workflow_raw
        self.declare_parameter('resume_phase', '')
        self.resume_phase = str(self.get_parameter('resume_phase').value).lower().strip()
        self.declare_parameter('start_state', '')
        self.start_state = str(self.get_parameter('start_state').value).lower().strip()
        self._start_state_handled = False
        self._start_state_override_active = False
        self._start_state_wait_last_log = 0.0
        self.declare_parameter('managed_phase_mode', False)
        self.managed_phase_mode = bool(self.get_parameter('managed_phase_mode').value)
        self.managed_active_phase = ''
        self.managed_completed_phases = set()
        if self.managed_phase_mode and workflow_raw != 'full':
            self.get_logger().warn(
                f'[MANAGED_PHASE] workflow={workflow_raw} ignored while managed_phase_mode=true; forcing workflow=full'
            )
            workflow_raw = 'full'
            self.workflow = workflow_raw
        self.startup_resume_phase = 'none'
        self.startup_resume_pending = False
        self._apply_resume_phase_override()
        self.declare_parameter('skip_preflight', False)
        self.skip_preflight = bool(self.get_parameter('skip_preflight').value)
        if self.workflow == 'transport' and self.skip_preflight:
            self.get_logger().warn('[WORKFLOW=transport] skip_preflight 被忽略：transport-only 必须通过预检后才允许启动')
            self.skip_preflight = False
        self.preflight_ok = bool(
            self.managed_phase_mode and
            (not self.start_state) and
            (not self.startup_resume_pending)
        )
        self.preflight_last_log = 0.0
        self.workflow_boundary_reached = False

        # ===== loaded z-leveling & load stable barrier =====
        self.declare_parameter('load_level_enable', True)
        self.declare_parameter('load_level_only_raise_z', True)
        self.declare_parameter('load_level_contact_force_n', 18.0)
        self.declare_parameter('load_level_force_fresh_timeout_sec', 0.40)
        self.declare_parameter('load_level_z_tol_mm', 1.0)
        self.declare_parameter('load_level_vel_tol_mmps', 1.0)
        self.declare_parameter('load_level_time_sec', 2.0)
        self.declare_parameter('load_level_z_plane_max_mm', 180.0)
        self.declare_parameter('load_level_z_plane_min_mm', 80.0)

        self.declare_parameter('load_stable_hold_sec', 1.5)
        self.declare_parameter('load_stable_slide_vel_tol_mmps', 1.0)
        self.declare_parameter('load_stable_force_window_sec', 1.0)
        self.declare_parameter('load_stable_force_slope_tol_nps', 3.0)
        self.declare_parameter('load_stable_force_fresh_timeout_sec', 0.40)
        self.declare_parameter('load_stable_delta_fresh_timeout_sec', 0.40)
        self.declare_parameter('load_stable_xy_tol_m', 0.012)
        self.declare_parameter('load_stable_z_tol_m', 0.10)

        self.load_level_enable = bool(self.get_parameter('load_level_enable').value)
        self.load_level_only_raise_z = bool(self.get_parameter('load_level_only_raise_z').value)
        self.load_level_contact_force_n = float(self.get_parameter('load_level_contact_force_n').value)
        self.load_level_force_fresh_timeout_sec = float(self.get_parameter('load_level_force_fresh_timeout_sec').value)
        self.load_level_z_tol_mm = float(self.get_parameter('load_level_z_tol_mm').value)
        self.load_level_vel_tol_mmps = float(self.get_parameter('load_level_vel_tol_mmps').value)
        self.load_level_time_sec = float(self.get_parameter('load_level_time_sec').value)
        self.load_level_z_plane_max_mm = float(self.get_parameter('load_level_z_plane_max_mm').value)
        self.load_level_z_plane_min_mm = float(self.get_parameter('load_level_z_plane_min_mm').value)

        self.load_stable_hold_sec = float(self.get_parameter('load_stable_hold_sec').value)
        self.load_stable_slide_vel_tol_mmps = float(self.get_parameter('load_stable_slide_vel_tol_mmps').value)
        self.load_stable_force_window_sec = float(self.get_parameter('load_stable_force_window_sec').value)
        self.load_stable_force_slope_tol_nps = float(self.get_parameter('load_stable_force_slope_tol_nps').value)
        self.load_stable_force_fresh_timeout_sec = float(self.get_parameter('load_stable_force_fresh_timeout_sec').value)
        self.load_stable_delta_fresh_timeout_sec = float(self.get_parameter('load_stable_delta_fresh_timeout_sec').value)
        self.load_stable_xy_tol_m = float(self.get_parameter('load_stable_xy_tol_m').value)
        self.load_stable_z_tol_m = float(self.get_parameter('load_stable_z_tol_m').value)

        # ===== slide compensation =====
        self.declare_parameter('slide_comp_enable', True)
        self.declare_parameter('slide_comp_alignment_enable', True)
        self.declare_parameter('slide_comp_transport_enable', True)
        self.declare_parameter('slide_comp_cmd_period_sec', 0.05)

        self.declare_parameter('slide_comp_x_sign', 1.0)
        self.declare_parameter('slide_comp_y_sign', 1.0)
        self.declare_parameter('slide_comp_z_sign', 1.0)

        self.declare_parameter('slide_comp_vx_gain_mmps_per_m', 600.0)
        self.declare_parameter('slide_comp_vy_gain_mmps_per_m', 600.0)
        self.declare_parameter('slide_comp_vz_gain_mmps_per_m', 400.0)

        self.declare_parameter('slide_comp_vx_limit_mmps', 20.0)
        self.declare_parameter('slide_comp_vy_limit_mmps', 20.0)
        self.declare_parameter('slide_comp_vz_limit_mmps', 10.0)

        self.declare_parameter('slide_comp_dx_deadband_m', 0.002)
        self.declare_parameter('slide_comp_dy_deadband_m', 0.002)
        self.declare_parameter('slide_comp_dz_deadband_m', 0.002)

        self.declare_parameter('slide_comp_hold_zero_on_lost', True)
        self.declare_parameter('slide_comp_abort_transport_on_hard_lost', False)

        self.slide_comp_enable = bool(self.get_parameter('slide_comp_enable').value)
        self.slide_comp_alignment_enable = bool(self.get_parameter('slide_comp_alignment_enable').value)
        self.slide_comp_transport_enable = bool(self.get_parameter('slide_comp_transport_enable').value)
        self.slide_comp_cmd_period_sec = float(self.get_parameter('slide_comp_cmd_period_sec').value)

        def _norm_sign(v: float) -> float:
            v = float(v)
            if abs(v) < 1e-9:
                return 1.0
            return 1.0 if v > 0.0 else -1.0

        self.slide_comp_x_sign = _norm_sign(self.get_parameter('slide_comp_x_sign').value)
        self.slide_comp_y_sign = _norm_sign(self.get_parameter('slide_comp_y_sign').value)
        self.slide_comp_z_sign = _norm_sign(self.get_parameter('slide_comp_z_sign').value)

        self.slide_comp_vx_gain_mmps_per_m = float(self.get_parameter('slide_comp_vx_gain_mmps_per_m').value)
        self.slide_comp_vy_gain_mmps_per_m = float(self.get_parameter('slide_comp_vy_gain_mmps_per_m').value)
        self.slide_comp_vz_gain_mmps_per_m = float(self.get_parameter('slide_comp_vz_gain_mmps_per_m').value)

        self.slide_comp_vx_limit_mmps = float(self.get_parameter('slide_comp_vx_limit_mmps').value)
        self.slide_comp_vy_limit_mmps = float(self.get_parameter('slide_comp_vy_limit_mmps').value)
        self.slide_comp_vz_limit_mmps = float(self.get_parameter('slide_comp_vz_limit_mmps').value)

        self.slide_comp_dx_deadband_m = float(self.get_parameter('slide_comp_dx_deadband_m').value)
        self.slide_comp_dy_deadband_m = float(self.get_parameter('slide_comp_dy_deadband_m').value)
        self.slide_comp_dz_deadband_m = float(self.get_parameter('slide_comp_dz_deadband_m').value)

        self.slide_comp_hold_zero_on_lost = bool(self.get_parameter('slide_comp_hold_zero_on_lost').value)
        self.slide_comp_abort_transport_on_hard_lost = bool(self.get_parameter('slide_comp_abort_transport_on_hard_lost').value)

        # ===== transport-specific compliant slide params =====
        self.declare_parameter('slide_transport_vx_gain_mmps_per_m', 120.0)
        self.declare_parameter('slide_transport_vy_gain_mmps_per_m', 120.0)
        self.declare_parameter('slide_transport_vz_gain_mmps_per_m', 120.0)

        self.declare_parameter('slide_transport_vx_limit_mmps', 10.0)
        self.declare_parameter('slide_transport_vy_limit_mmps', 10.0)
        self.declare_parameter('slide_transport_vz_limit_mmps', 10.0)

        self.declare_parameter('slide_transport_dx_deadband_m', 0.003)
        self.declare_parameter('slide_transport_dy_deadband_m', 0.003)
        self.declare_parameter('slide_transport_dz_deadband_m', 0.006)
        self.declare_parameter('slide_transport_dz_hold_tol_m', 0.012)

        self.declare_parameter('slide_transport_force_yield_deadband_n', 12.0)
        self.declare_parameter('slide_transport_force_yield_full_n', 30.0)
        self.declare_parameter('slide_transport_force_yield_min_scale', 0.25)

        self.slide_transport_vx_gain_mmps_per_m = float(self.get_parameter('slide_transport_vx_gain_mmps_per_m').value)
        self.slide_transport_vy_gain_mmps_per_m = float(self.get_parameter('slide_transport_vy_gain_mmps_per_m').value)
        self.slide_transport_vz_gain_mmps_per_m = float(self.get_parameter('slide_transport_vz_gain_mmps_per_m').value)

        self.slide_transport_vx_limit_mmps = float(self.get_parameter('slide_transport_vx_limit_mmps').value)
        self.slide_transport_vy_limit_mmps = float(self.get_parameter('slide_transport_vy_limit_mmps').value)
        self.slide_transport_vz_limit_mmps = float(self.get_parameter('slide_transport_vz_limit_mmps').value)

        self.slide_transport_dx_deadband_m = float(self.get_parameter('slide_transport_dx_deadband_m').value)
        self.slide_transport_dy_deadband_m = float(self.get_parameter('slide_transport_dy_deadband_m').value)
        self.slide_transport_dz_deadband_m = float(self.get_parameter('slide_transport_dz_deadband_m').value)
        self.slide_transport_dz_hold_tol_m = float(self.get_parameter('slide_transport_dz_hold_tol_m').value)

        self.slide_transport_force_yield_deadband_n = float(self.get_parameter('slide_transport_force_yield_deadband_n').value)
        self.slide_transport_force_yield_full_n = float(self.get_parameter('slide_transport_force_yield_full_n').value)
        self.slide_transport_force_yield_min_scale = float(self.get_parameter('slide_transport_force_yield_min_scale').value)

        # ===== transport slide recenter =====
        self.declare_parameter('slide_transport_recenter_enable', True)
        self.declare_parameter('slide_transport_recenter_targets_flat', [125.0, 135.0, 126.0, 141.0, 126.0, 134.0])
        self.declare_parameter('slide_transport_recenter_gain_mmps_per_mm', 0.8)
        self.declare_parameter('slide_transport_recenter_vxy_limit_mmps', 8.0)
        self.declare_parameter('slide_transport_recenter_xy_deadband_mm', 2.0)

        self.slide_transport_recenter_enable = bool(self.get_parameter('slide_transport_recenter_enable').value)
        self.slide_transport_recenter_gain_mmps_per_mm = float(self.get_parameter('slide_transport_recenter_gain_mmps_per_mm').value)
        self.slide_transport_recenter_vxy_limit_mmps = float(self.get_parameter('slide_transport_recenter_vxy_limit_mmps').value)
        self.slide_transport_recenter_xy_deadband_mm = float(self.get_parameter('slide_transport_recenter_xy_deadband_mm').value)

        self.declare_parameter('transport_chassis_fusion_enable', True)
        self.declare_parameter('transport_ff_enable', True)
        self.declare_parameter('transport_ff_use_mocap', True)
        self.declare_parameter('transport_ff_use_odom', True)
        self.declare_parameter('transport_ff_use_imu', True)
        self.declare_parameter('transport_chassis_fresh_timeout_sec', 0.40)
        self.declare_parameter('transport_mocap_twist_alpha', 0.35)
        self.declare_parameter('transport_ff_vx_gain_mmps_per_mps', 220.0)
        self.declare_parameter('transport_ff_vy_gain_mmps_per_mps', 220.0)
        self.declare_parameter('transport_ff_yaw_enable', False)
        self.declare_parameter('transport_ff_yaw_gain_scale', 0.25)
        self.declare_parameter('transport_chassis_stable_body_vel_tol_mps', 0.015)
        self.declare_parameter('transport_chassis_stable_yaw_rate_tol_rps', 0.08)

        self.transport_chassis_fusion_enable = bool(self.get_parameter('transport_chassis_fusion_enable').value)
        self.transport_ff_enable = bool(self.get_parameter('transport_ff_enable').value)
        self.transport_ff_use_mocap = bool(self.get_parameter('transport_ff_use_mocap').value)
        # Odom / IMU feedforward branches are disabled for the current setup.
        # self.transport_ff_use_odom = bool(self.get_parameter('transport_ff_use_odom').value)
        # self.transport_ff_use_imu = bool(self.get_parameter('transport_ff_use_imu').value)
        self.transport_ff_use_odom = False
        self.transport_ff_use_imu = False
        self.transport_chassis_fresh_timeout_sec = float(self.get_parameter('transport_chassis_fresh_timeout_sec').value)
        self.transport_mocap_twist_alpha = float(self.get_parameter('transport_mocap_twist_alpha').value)
        self.transport_ff_vx_gain_mmps_per_mps = float(self.get_parameter('transport_ff_vx_gain_mmps_per_mps').value)
        self.transport_ff_vy_gain_mmps_per_mps = float(self.get_parameter('transport_ff_vy_gain_mmps_per_mps').value)
        self.transport_ff_yaw_enable = bool(self.get_parameter('transport_ff_yaw_enable').value)
        self.transport_ff_yaw_gain_scale = float(self.get_parameter('transport_ff_yaw_gain_scale').value)
        self.transport_chassis_stable_body_vel_tol_mps = float(self.get_parameter('transport_chassis_stable_body_vel_tol_mps').value)
        self.transport_chassis_stable_yaw_rate_tol_rps = float(self.get_parameter('transport_chassis_stable_yaw_rate_tol_rps').value)

        targets_flat = [float(x) for x in self.get_parameter('slide_transport_recenter_targets_flat').value]
        self.slide_transport_recenter_targets = {}
        if len(targets_flat) >= 6:
            self.slide_transport_recenter_targets['tracer1'] = (targets_flat[0], targets_flat[1])
            self.slide_transport_recenter_targets['tracer2'] = (targets_flat[2], targets_flat[3])
            self.slide_transport_recenter_targets['tracer3'] = (targets_flat[4], targets_flat[5])
        else:
            self.get_logger().warn(f'slide_transport_recenter_targets_flat has {len(targets_flat)} elements, expected 6')
            self.slide_transport_recenter_targets = {
                'tracer1': (125.0, 135.0),
                'tracer2': (126.0, 141.0),
                'tracer3': (126.0, 134.0)
            }
        self.get_logger().warn(
            f'[SLIDE_RECENTER_TARGETS] enable={self.slide_transport_recenter_enable} '
            f'targets={self.slide_transport_recenter_targets}'
        )

        # ===== slide realtime loop + smoothing =====
        self.declare_parameter('slide_rt_hz', 20.0)
        self.declare_parameter('slide_rt_apply_inputs', False)
        self.declare_parameter('slide_status_fresh_timeout_sec', 0.5)
        self.declare_parameter('slide_comp_ax_limit_mmps2', 120.0)
        self.declare_parameter('slide_comp_ay_limit_mmps2', 120.0)
        self.declare_parameter('slide_comp_az_limit_mmps2', 60.0)
        self.declare_parameter('slide_comp_vx_min_mmps', 2.0)
        self.declare_parameter('slide_comp_vy_min_mmps', 2.0)
        self.declare_parameter('slide_comp_vz_min_mmps', 1.0)

        self.slide_rt_hz = max(5.0, float(self.get_parameter('slide_rt_hz').value))
        self.slide_rt_apply_inputs = bool(self.get_parameter('slide_rt_apply_inputs').value)
        self.slide_status_fresh_timeout_sec = float(self.get_parameter('slide_status_fresh_timeout_sec').value)
        self.slide_comp_ax_limit_mmps2 = float(self.get_parameter('slide_comp_ax_limit_mmps2').value)
        self.slide_comp_ay_limit_mmps2 = float(self.get_parameter('slide_comp_ay_limit_mmps2').value)
        self.slide_comp_az_limit_mmps2 = float(self.get_parameter('slide_comp_az_limit_mmps2').value)
        self.slide_comp_vx_min_mmps = float(self.get_parameter('slide_comp_vx_min_mmps').value)
        self.slide_comp_vy_min_mmps = float(self.get_parameter('slide_comp_vy_min_mmps').value)
        self.slide_comp_vz_min_mmps = float(self.get_parameter('slide_comp_vz_min_mmps').value)

        # ===== direct align fallback =====
        self.declare_parameter('slide_direct_align_enable', True)
        self.declare_parameter('slide_direct_align_speed_mmps', 20.0)
        self.declare_parameter('slide_direct_align_min_time_sec', 0.10)
        self.declare_parameter('slide_direct_align_settle_margin_sec', 0.25)
        self.declare_parameter('slide_direct_align_fresh_timeout_sec', 0.50)
        self.declare_parameter('slide_direct_align_trigger_wait_sec', 0.80)
        self.declare_parameter('slide_direct_align_max_retry', 3)
        self.declare_parameter('slide_direct_align_pos_deadband_mm', 1.0)
        self.declare_parameter('slide_direct_align_xy_move_confirm_mm', 0.8)
        self.declare_parameter('slide_direct_align_xy_move_timeout_margin_sec', 2.0)
        self.declare_parameter('slide_direct_align_ack_drop_timeout_sec', 0.8)
        self.declare_parameter('slide_direct_align_contact_enable', True)
        self.declare_parameter('slide_post_contact_hold_sec', 5.0)
        self.declare_parameter('slide_direct_align_contact_seek_mm', 140.0)

        self.slide_direct_align_enable = bool(self.get_parameter('slide_direct_align_enable').value)
        self.slide_direct_align_speed_mmps = float(self.get_parameter('slide_direct_align_speed_mmps').value)
        self.slide_direct_align_min_time_sec = float(self.get_parameter('slide_direct_align_min_time_sec').value)
        self.slide_direct_align_settle_margin_sec = float(self.get_parameter('slide_direct_align_settle_margin_sec').value)
        self.slide_direct_align_fresh_timeout_sec = float(self.get_parameter('slide_direct_align_fresh_timeout_sec').value)
        self.slide_direct_align_trigger_wait_sec = float(self.get_parameter('slide_direct_align_trigger_wait_sec').value)
        self.slide_direct_align_max_retry = int(self.get_parameter('slide_direct_align_max_retry').value)
        self.slide_direct_align_pos_deadband_mm = float(self.get_parameter('slide_direct_align_pos_deadband_mm').value)
        self.slide_direct_align_xy_move_confirm_mm = float(self.get_parameter('slide_direct_align_xy_move_confirm_mm').value)
        self.slide_direct_align_xy_move_timeout_margin_sec = float(self.get_parameter('slide_direct_align_xy_move_timeout_margin_sec').value)
        self.slide_direct_align_ack_drop_timeout_sec = float(self.get_parameter('slide_direct_align_ack_drop_timeout_sec').value)
        self.slide_direct_align_contact_enable = bool(self.get_parameter('slide_direct_align_contact_enable').value)
        self.slide_post_contact_hold_sec = float(self.get_parameter('slide_post_contact_hold_sec').value)
        self.slide_direct_align_contact_seek_mm = float(self.get_parameter('slide_direct_align_contact_seek_mm').value)

        # ===== alignment strategy =====
        self.declare_parameter('slide_align_mode', 'direct_only')
        self.slide_align_mode = str(self.get_parameter('slide_align_mode').value).lower().strip()
        if self.slide_align_mode not in ('direct_only', 'speed_only', 'direct_then_speed'):
            self.get_logger().warn(
                f'Invalid slide_align_mode={self.slide_align_mode}, fallback to direct_only'
            )
            self.slide_align_mode = 'direct_only'
        self.get_logger().warn(f'[SLIDE_ALIGN_MODE] {self.slide_align_mode}')

        # ===== per-robot axis capability map (x,y,z for each robot) =====
        self.declare_parameter('slide_axis_enable_flat', [1, 1, 1, 1, 1, 1, 1, 1, 1])
        axis_flat = [float(x) for x in self.get_parameter('slide_axis_enable_flat').value]
        self.slide_axis_enable_map = None
        if len(axis_flat) >= 3 * len(self.robots):
            cap_map = {}
            for i, rn in enumerate(self.robots):
                x_on = bool(axis_flat[3 * i + 0] >= 0.5)
                y_on = bool(axis_flat[3 * i + 1] >= 0.5)
                z_on = bool(axis_flat[3 * i + 2] >= 0.5)
                cap_map[rn] = {'x': x_on, 'y': y_on, 'z': z_on}
                if rn.startswith('tracer'):
                    suffix = rn[len('tracer'):]
                    if suffix.isdigit():
                        cap_map[f'huatai{suffix}'] = {'x': x_on, 'y': y_on, 'z': z_on}
            self.slide_axis_enable_map = cap_map
            self.get_logger().warn(f'[SLIDE_AXIS_CAP] loaded from slide_axis_enable_flat: {self.slide_axis_enable_map}')
        else:
            self.get_logger().warn(
                f'[SLIDE_AXIS_CAP] slide_axis_enable_flat length={len(axis_flat)} is too short; fallback to default capability map'
            )

        # ===== slide recenter =====
        self.declare_parameter('slide_recenter_enable', True)
        self.declare_parameter('slide_recenter_mode', 'virtual')
        self.declare_parameter('slide_center_xyz_mm', [139.0, 138.5, 100.0])
        self.declare_parameter('slide_recenter_time_sec', 3.0)
        self.declare_parameter('slide_recenter_tol_mm', 1.0)
        self.declare_parameter('slide_recenter_hold_sec', 0.50)
        self.declare_parameter('slide_recenter_abort_on_hard_lost', False)

        self.slide_recenter_enable = bool(self.get_parameter('slide_recenter_enable').value)
        self.slide_recenter_mode = str(self.get_parameter('slide_recenter_mode').value).strip().lower()

        center_xyz = [float(x) for x in self.get_parameter('slide_center_xyz_mm').value]
        while len(center_xyz) < 3:
            center_xyz.append(0.0)
        self.slide_center_x_mm = float(center_xyz[0])
        self.slide_center_y_mm = float(center_xyz[1])
        self.slide_center_z_mm = float(center_xyz[2])

        self.slide_recenter_time_sec = float(self.get_parameter('slide_recenter_time_sec').value)
        self.slide_recenter_tol_mm = float(self.get_parameter('slide_recenter_tol_mm').value)
        self.slide_recenter_hold_sec = float(self.get_parameter('slide_recenter_hold_sec').value)
        self.slide_recenter_abort_on_hard_lost = bool(self.get_parameter('slide_recenter_abort_on_hard_lost').value)

        self.get_logger().warn(
            f"[SLIDE_SIGN_CONFIG] x_sign={self.slide_comp_x_sign:.1f}, "
            f"y_sign={self.slide_comp_y_sign:.1f}, z_sign={self.slide_comp_z_sign:.1f}"
        )

        # ===== QoS =====
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_reliable_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ===== world state =====
        self.wing_x: Optional[float] = None
        self.wing_y: Optional[float] = None
        self.wing_yaw: Optional[float] = None
        self._wing_frozen = False

        self.wing_pose_stamp: float = 0.0
        self.robot_pose_stamp: Dict[str, float] = {rn: 0.0 for rn in self.robots}

        self.robot_xy: Dict[str, Tuple[float, float]] = {}
        self.robot_yaw: Dict[str, float] = {}

        self.rt: Dict[str, RobotRuntime] = {rn: RobotRuntime() for rn in self.robots}
        for rn in self.robots:
            self.rt[rn].force_f = None
            self.rt[rn].force_stamp = 0.0
            self.rt[rn].force_hist = deque()

            self.rt[rn].level_active = False
            self.rt[rn].level_done = False
            self.rt[rn].level_target_z_mm = None
            self.rt[rn].loaded_ref_captured = False

            self.rt[rn].sync_wait_qr = False
            self.rt[rn].sync_wait_qr_epoch = 0.0

            self.rt[rn].te = TargetEstimator(
                arm_delay_sec=self.delta_arm_delay_sec,
                max_age_sec=self.delta_max_age_sec,
                frame_gap_max_sec=self.delta_frame_gap_max_sec,
                abs_max_m=self.delta_abs_max_m,
                stable_eps=self.delta_stable_eps,
                stable_frames=self.delta_threshold_frames,
                min_confidence=self.te_min_confidence,
                jump_sigma=self.te_jump_sigma,
                lost_timeout_sec=self.te_lost_timeout_sec,
                freeze_timeout_sec=self.te_freeze_timeout_sec,
                ema_alpha=self.te_ema_alpha,
                gain=1.0,
                step_cap_m=0.02,
                delta_in_world=False,
                error_sign=1.0,
                dx_sign=1.0,
                dy_sign=1.0,
                # duplicate_eps, max_jump_suppress, jump_decay_alpha 已移除，保持与TargetEstimator签名一致
            )

        self._raw_qr_diag_counter: Dict[str, int] = {rn: 0 for rn in self.robots}
        self._wing_pose_rx = LatestValueBuffer()
        self._robot_mocap_rx = {rn: LatestValueBuffer() for rn in self.robots}
        # Odom / IMU buffering is disabled for the current setup.
        # self._odom_rx = {rn: LatestValueBuffer() for rn in self.robots}
        # self._imu_rx = {rn: LatestValueBuffer() for rn in self.robots}
        self._odom_rx = {}
        self._imu_rx = {}
        self._reached_rx = {rn: LatestValueBuffer() for rn in self.robots}
        self._force_rx = {rn: EventQueue(maxsize=1024) for rn in self.robots}
        self._raw_qr_rx = {rn: EventQueue(maxsize=512) for rn in self.robots}
        self._delta_rx = {rn: EventQueue(maxsize=2048) for rn in self.robots}
        self._slide_status_rx = {rn: LatestValueBuffer() for rn in self.robots}
        self._force_contact_rx = {rn: EventQueue(maxsize=64) for rn in self.robots}

        # ===== ROS I/O =====
        self.wing_sub = self.create_subscription(PoseStamped, self.wing_mocap_topic, self.wing_cb, qos_be)

        self.goal_pub: Dict[str, object] = {}
        self.stop_pub: Dict[str, object] = {}
        self.resume_pub: Dict[str, object] = {}
        self.precision_pub: Dict[str, object] = {}
        self.qr_zero_cli: Dict[str, object] = {}
        self.qr_reset_cli: Dict[str, object] = {}

        self.slide_cmd_pub: Dict[str, object] = {}
        self.slide_comp_pub: Dict[str, object] = {}
        self.slide_status_sub: Dict[str, object] = {}
        self.raw_qr_sub: Dict[str, object] = {}
        self.force_sub: Dict[str, object] = {}

        for rn in self.robots:
            self.goal_pub[rn] = self.create_publisher(Twist, f'/{rn}/cmd_goal', qos_reliable_transient)
            self.stop_pub[rn] = self.create_publisher(Bool, f'/{rn}/cmd_stop', qos_volatile)
            self.resume_pub[rn] = self.create_publisher(Bool, f'/{rn}/cmd_resume', qos_volatile)
            self.precision_pub[rn] = self.create_publisher(Bool, self._fmt(self.precision_topic_template, rn), 10)

            huatai_id = rn.replace('tracer', 'huatai')

            self.slide_cmd_pub[rn] = self.create_publisher(MotorCommand, f'/{huatai_id}_pos_spe_pd', 10)
            self.slide_comp_pub[rn] = self.create_publisher(MotorCommand, f'/{huatai_id}_compensation_ref', 10)
            self.slide_status_sub[rn] = self.create_subscription(
                MotorStatus,
                f'/{huatai_id}_pos_spe_p',
                self._mk_slide_status_cb(rn),
                10
            )

            self.force_sub[rn] = self.create_subscription(
                Float32MultiArray,
                f'/{huatai_id}_force_filtered',
                self._mk_force_cb(rn),
                qos_be
            )

            self.create_subscription(Bool, f'/{rn}/goal_reached', self._mk_reached_cb(rn), 10)
            self.create_subscription(Vector3Stamped, self._fmt(self.delta_topic_template, rn), self._mk_delta_cb(rn), qos_be)

            self.raw_qr_sub[rn] = self.create_subscription(
                PoseStamped,
                self._fmt(self.raw_qr_topic_template, rn),
                self._mk_raw_qr_cb(rn),
                qos_be
            )

            srv = f'/{rn}/qr_delta/zero'
            self.qr_zero_cli[rn] = self.create_client(Trigger, srv)

            srv_reset = f'/{rn}/qr_delta/reset_tracking'
            self.qr_reset_cli[rn] = self.create_client(Trigger, srv_reset)

        self._bench_trace = build_trace_publisher(self, source_node="mission_coordinator", default_enabled=False)
        self._bench_precision_state = {rn: False for rn in self.robots}
        self.declare_parameter('run_id', '')
        self.declare_parameter('mission_log_dir', '~/.ros/mission_bench_logs')
        self.run_id = str(self.get_parameter('run_id').value).strip() or time.strftime('%Y%m%d_%H%M%S')
        self.mission_log_dir = os.path.expanduser(str(self.get_parameter('mission_log_dir').value).strip() or '~/.ros/mission_bench_logs')
        self._mission_runtime_log_dir = os.path.join(self.mission_log_dir, self.run_id)
        self.mission_runtime_logger = AsyncCsvLogger(
            os.path.join(self._mission_runtime_log_dir, 'mission_runtime_events.csv'),
            MISSION_RUNTIME_FIELDS,
        )
        self.declare_parameter('bench_snapshot_period_sec', 0.25)
        self._bench_snapshot_period_sec = max(0.05, float(self.get_parameter('bench_snapshot_period_sec').value))
        self._bench_last_snapshot_sec = 0.0

        for rn, tp in zip(self.robots, self.robot_mocap_topics):
            self.create_subscription(PoseStamped, tp, self._mk_robot_mocap_cb(rn), qos_be)
        # Odom / IMU subscriptions are disabled for the current setup.
        # for rn, tp in zip(self.robots, self.robot_odom_topics):
        #     if tp:
        #         self.create_subscription(Odometry, tp, self._mk_odom_cb(rn), qos_be)
        # for rn, tp in zip(self.robots, self.robot_imu_topics):
        #     if tp:
        #         self.create_subscription(Imu, tp, self._mk_imu_cb(rn), qos_be)

        self.emergency = False
        qos_emg = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Bool, self.emergency_stop_topic, self._emergency_cb, qos_emg)
        if self.force_contact_enable:
            for rn in self.robots:
                topic = self._force_contact_topic_for_robot(rn)
                self.create_subscription(Bool, topic, self._mk_force_contact_cb(rn), qos_emg)

        # ===== mission state =====
        self.state = (
            'WAIT_WING'
            if self.startup_resume_pending
            else ('STANDBY' if (self.managed_phase_mode and (not self.start_state)) else 'WAIT_WING')
        )
        self._pending_initial_dispatch = False
        self._dispatch_epoch = 0.0
        self.all_ready_since = 0.0
        self.sync_lift_since = 0.0
        self.recenter_all_done_since = 0.0
        self.transport_dispatch_epoch = 0.0
        self.transport_precheck_last_log = 0.0
        self.load_stable_since = 0.0
        self.loaded_z_plane_mm: Optional[float] = None

        now_ros = _now_sec(self)
        self._wait_wing_log_time = now_ros
        self._state_lock = threading.RLock()

        self._mission_start_approach_srv = self.create_service(
            Trigger,
            '/mission/start_approach',
            self._handle_start_approach,
        )
        self._mission_start_slide_align_srv = self.create_service(
            Trigger,
            '/mission/start_slide_align',
            self._handle_start_slide_align,
        )
        self._mission_start_level_recenter_srv = self.create_service(
            Trigger,
            '/mission/start_level_recenter',
            self._handle_start_level_recenter,
        )
        self._mission_start_transport_srv = self.create_service(
            Trigger,
            '/mission/start_transport',
            self._handle_start_transport,
        )
        self._mission_reset_to_standby_srv = self.create_service(
            Trigger,
            '/mission/reset_to_standby',
            self._handle_reset_to_standby,
        )
        self._mission_get_status_srv = self.create_service(
            Trigger,
            '/mission/get_status',
            self._handle_get_status,
        )

        self.timer = self.create_timer(1.0 / self.loop_hz, self._loop_tick_locked)
        self._last_slide_rt_overrun_log_wall = 0.0
        self._slide_executor = FixedRateLoop(
            name='mission_slide_rt',
            hz=self.slide_rt_hz,
            tick_fn=self._slide_tick_locked,
            on_error=self._on_slide_rt_error,
            on_overrun=self._on_slide_rt_overrun,
        )
        self._slide_executor.start()

        self.get_logger().warn(f'>>> MISSION COORDINATOR INITIALIZED | WORKFLOW={self.workflow.upper()} <<<')
        sys.stdout.flush()

    # ------------------------------------------------------------------
    # buffered input apply
    # ------------------------------------------------------------------
    def _apply_pending_inputs_locked(self):
        wing_sample = self._wing_pose_rx.pop()
        if wing_sample is not None:
            self._apply_wing_pose_sample_locked(wing_sample)

        for rn in self.robots:
            mocap_sample = self._robot_mocap_rx[rn].pop()
            if mocap_sample is not None:
                self._apply_robot_mocap_sample_locked(rn, mocap_sample)

            # Odom / IMU sample application is disabled for the current setup.
            # odom_sample = self._odom_rx[rn].pop()
            # if odom_sample is not None:
            #     self._apply_odom_sample_locked(rn, odom_sample)
            #
            # imu_sample = self._imu_rx[rn].pop()
            # if imu_sample is not None:
            #     self._apply_imu_sample_locked(rn, imu_sample)

            reached_sample = self._reached_rx[rn].pop()
            if reached_sample is not None:
                self._apply_reached_sample_locked(rn, reached_sample)

            for force_sample in self._force_rx[rn].drain():
                self._apply_force_sample_locked(rn, force_sample)

            slide_status_sample = self._slide_status_rx[rn].pop()
            if slide_status_sample is not None:
                self._apply_slide_status_sample_locked(rn, slide_status_sample)

            for delta_sample in self._delta_rx[rn].drain():
                self._apply_delta_sample_locked(rn, delta_sample)

            for raw_qr_sample in self._raw_qr_rx[rn].drain():
                self._apply_raw_qr_sample_locked(rn, raw_qr_sample)

            for force_contact_sample in self._force_contact_rx[rn].drain():
                self._apply_force_contact_sample_locked(rn, force_contact_sample)

    def _apply_wing_pose_sample_locked(self, sample):
        if self.freeze_wing_on_start and self._wing_frozen:
            return

        px, pz, ox, oy, oz, ow, stamp = sample
        mx = float(px) * self.mm_to_m
        mz = float(pz) * self.mm_to_m
        xw, yw = map_mocap_xy(mx, mz, self.swap_xz, self.negate_x, self.negate_z)

        self.wing_x = xw
        self.wing_y = yw
        self.wing_yaw = extract_mocap_yaw_rad(
            float(ox),
            float(oy),
            float(oz),
            float(ow),
            mode=self.mocap_yaw_mode,
            flip_heading_sign=self.flip_heading_sign,
            heading_deg_bias=self.heading_deg_bias,
        )
        self.wing_pose_stamp = float(stamp)

    def _apply_robot_mocap_sample_locked(self, rn: str, sample):
        px, pz, ox, oy, oz, ow, now = sample
        mx = float(px) * self.mm_to_m
        mz = float(pz) * self.mm_to_m
        xw, yw = map_mocap_xy(mx, mz, self.swap_xz, self.negate_x, self.negate_z)
        yaw_rad = extract_mocap_yaw_rad(
            float(ox),
            float(oy),
            float(oz),
            float(ow),
            mode=self.mocap_yaw_mode,
            flip_heading_sign=self.flip_heading_sign,
            heading_deg_bias=self.heading_deg_bias,
        )

        ctx = self.rt[rn]
        prev_xy = ctx.last_mocap_xy
        prev_yaw = ctx.last_mocap_yaw
        prev_stamp = float(ctx.last_mocap_stamp)

        self.robot_xy[rn] = (xw, yw)
        self.robot_yaw[rn] = yaw_rad
        self.robot_pose_stamp[rn] = float(now)

        if prev_xy is not None and prev_stamp > 0.0 and float(now) > prev_stamp:
            dt = float(now) - prev_stamp
            if dt >= 1e-3:
                vx_w = (xw - float(prev_xy[0])) / dt
                vy_w = (yw - float(prev_xy[1])) / dt

                c = math.cos(yaw_rad)
                s = math.sin(yaw_rad)
                vx_b = c * vx_w + s * vy_w
                vy_b = -s * vx_w + c * vy_w

                ctx.mocap_twist_world = (vx_w, vy_w)
                ctx.mocap_twist_body = (vx_b, vy_b)
                if prev_yaw is not None:
                    ctx.mocap_wz = wrap_angle_rad(yaw_rad - float(prev_yaw)) / dt
                else:
                    ctx.mocap_wz = 0.0
                ctx.mocap_twist_stamp = float(now)

        ctx.last_mocap_xy = (xw, yw)
        ctx.last_mocap_yaw = yaw_rad
        ctx.last_mocap_stamp = float(now)

    def _apply_odom_sample_locked(self, rn: str, sample):
        vx, vy, wz, stamp = sample
        ctx = self.rt[rn]
        ctx.odom_twist_body = (float(vx), float(vy), float(wz))
        ctx.odom_stamp = float(stamp)

    def _apply_imu_sample_locked(self, rn: str, sample):
        wz, stamp = sample
        ctx = self.rt[rn]
        ctx.imu_wz = float(wz)
        ctx.imu_stamp = float(stamp)

    def _apply_reached_sample_locked(self, rn: str, sample):
        self.rt[rn].reached = bool(sample)

    def _apply_force_sample_locked(self, rn: str, sample):
        now, fx, fy, fz, fn = sample
        ctx = self.rt[rn]
        ctx.force_f = (float(fx), float(fy), float(fz), float(fn))
        ctx.force_stamp = float(now)

        if ctx.force_hist is None:
            ctx.force_hist = deque()
        ctx.force_hist.append((float(now), float(fz)))

        hist_keep = max(1.0, self.load_stable_force_window_sec * 2.0)
        while len(ctx.force_hist) >= 2 and (float(now) - ctx.force_hist[0][0]) > hist_keep:
            ctx.force_hist.popleft()

    def _apply_raw_qr_sample_locked(self, rn: str, sample):
        x, y, z, source_stamp, receive_stamp, source_valid = sample
        ctx = self.rt[rn]
        source_stamp = float(source_stamp)
        receive_stamp = float(receive_stamp)
        source_valid = bool(source_valid)
        if source_valid:
            source_stamp, source_valid = self._sanitize_vision_source_stamp(
                rn, source_stamp, receive_stamp, 'RAW_QR'
            )

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            ctx.raw_qr_last_reject_reason = 'NON_FINITE'
            return
        if abs(x) < 1e-9 and abs(y) < 1e-9 and abs(z) < 1e-9:
            ctx.raw_qr_last_reject_reason = 'ZERO_VECTOR'
            return

        ctx.raw_qr_pose = (float(x), float(y), float(z))
        ctx.raw_qr_receive_stamp = receive_stamp
        ctx.raw_qr_last_source_stamp = source_stamp if source_valid else 0.0
        ctx.raw_qr_stamp_missing = not source_valid

        in_wait_qr = (ctx.dwell_start > 0.0 and ctx.goal_kind == 'FINAL' and not ctx.first_qr_locked)
        pose_ok = self._raw_qr_pose_reasonable_xyz(x, y, z)

        if not pose_ok:
            ctx.raw_qr_last_reject_reason = 'OUT_OF_BOUNDS'
            self._raw_qr_diag_counter[rn] += 1
            if in_wait_qr and self._raw_qr_diag_counter[rn] % 30 == 1:
                self.get_logger().warn(
                    f'[RAW_QR_REJECT][{rn}] pose=({x:.4f},{y:.4f},{z:.4f}) '
                    f'bounds: x=[{self.raw_qr_min_x_m},{self.raw_qr_max_x_m}] '
                    f'|y|<={self.raw_qr_max_abs_y_m} z=[{self.raw_qr_min_z_m},{self.raw_qr_max_z_m}]'
                )
            return

        if self.state not in ('RUN_ALIGNMENT', 'SYNC_SLIDE_ALIGN'):
            ctx.raw_qr_last_reject_reason = 'NOT_IN_ALIGNMENT_STATE'
            return
        if ctx.goal_kind != 'FINAL':
            ctx.raw_qr_last_reject_reason = 'NOT_FINAL_GOAL'
            return
        if not ctx.entered:
            ctx.raw_qr_last_reject_reason = 'NOT_ENTERED'
            return

        dwell_t0 = 0.0
        if ctx.dwell_start > 0.0:
            dwell_t0 = ctx.dwell_start
        elif getattr(ctx, 'sync_wait_qr', False):
            dwell_t0 = getattr(ctx, 'sync_wait_qr_epoch', 0.0)
        else:
            ctx.raw_qr_last_reject_reason = 'PRE_ARM'
            return

        now = receive_stamp
        if dwell_t0 <= 0.0 or (now - dwell_t0) < self.raw_qr_arm_delay_sec:
            ctx.raw_qr_last_reject_reason = 'PRE_ARM'
            return

        if ctx.final_target is None:
            ctx.raw_qr_last_reject_reason = 'NO_FINAL_TARGET'
            return
        if rn not in self.robot_xy:
            ctx.raw_qr_last_reject_reason = 'NO_MOCAP'
            return

        rx, ry = self.robot_xy[rn]
        xt, yt = ctx.final_target
        dist_to_final = math.hypot(rx - xt, ry - yt)
        if dist_to_final > self.raw_qr_accept_radius_m:
            ctx.raw_qr_last_reject_reason = 'DIST_TOO_FAR'
            if in_wait_qr and self._raw_qr_diag_counter[rn] % 30 == 1:
                self.get_logger().warn(
                    f'[RAW_QR_REJECT][{rn}] dist_to_final={dist_to_final:.3f} > {self.raw_qr_accept_radius_m}'
                )
            return

        if ctx.raw_qr_last_hit_stamp > 0.0 and (now - ctx.raw_qr_last_hit_stamp) <= self.raw_qr_hit_timeout_sec:
            ctx.raw_qr_hit_count += 1
        else:
            ctx.raw_qr_hit_count = 1

        ctx.raw_qr_last_hit_stamp = now
        ctx.raw_qr_seen_stamp = now

    def _apply_delta_sample_locked(self, rn: str, sample):
        msg_t, dx, dy, dz, now_ros, now_wall, source_valid = sample
        ctx = self.rt[rn]
        if ctx.faulted or ctx.finished:
            return
        source_valid = bool(source_valid)
        if source_valid:
            msg_t, source_valid = self._sanitize_vision_source_stamp(
                rn, msg_t, now_ros, 'DELTA'
            )
        else:
            msg_t = float(now_ros)

        allowed_states = (
            'RUN_ALIGNMENT',
            'SYNC_SLIDE_ALIGN',
            'ALL_READY_HOLD',
            'SYNC_LEVEL_Z',
            'LOAD_STABLE_HOLD',
            'SYNC_TRANSPORT',
            'SYNC_LIFT',
        )
        if self.workflow == 'transport':
            allowed_states = allowed_states + ('WAIT_WING',)
        if self.state not in allowed_states:
            return
        if self.entry_enable and (not ctx.entered) and self.state == 'RUN_ALIGNMENT':
            return

        ctx.delta_latest = (float(dx), float(dy), float(dz))
        ctx.last_delta_stamp = float(msg_t)
        ctx.last_delta_receive_stamp = float(now_ros)
        ctx.last_delta_stamp_missing = not source_valid

        if ctx.te is not None:
            ctx.te.observe_delta(float(dx), float(dy), float(dz), float(msg_t), float(now_ros), float(now_wall))

    @staticmethod
    def _valid_source_stamp(stamp_sec: float) -> bool:
        return math.isfinite(stamp_sec) and stamp_sec > 0.0

    def _sanitize_vision_source_stamp(self, rn: str, source_stamp: float, receive_stamp: float, tag: str):
        source_stamp = float(source_stamp)
        receive_stamp = float(receive_stamp)
        source_valid = self._valid_source_stamp(source_stamp)
        if not source_valid:
            return 0.0, False

        fallback_sec = float(getattr(self, 'vision_stamp_offset_fallback_sec', 0.0))
        if fallback_sec <= 0.0:
            return source_stamp, True

        offset_sec = receive_stamp - source_stamp
        if math.isfinite(offset_sec) and abs(offset_sec) > fallback_sec:
            ctx = self.rt.get(rn)
            if ctx is not None and self._diag_ok(ctx, f'_diag_{tag.lower()}_clock_offset_epoch', 2.0):
                self.get_logger().warn(
                    f'[{tag}_CLOCK_OFFSET][{rn}] source_stamp={source_stamp:.6f} '
                    f'receive_stamp={receive_stamp:.6f} '
                    f'offset={offset_sec:.3f}s(>{fallback_sec:.3f}s) -> fallback_to_receive_stamp'
                )
            return receive_stamp, True

        return source_stamp, True

    def _apply_slide_status_sample_locked(self, rn: str, sample):
        x, y, z, vx, vy, vz, reached, stamp = sample
        ctx = self.rt[rn]
        prev_reached = bool(getattr(ctx, '_diag_slide_reached_prev', False))
        prev_pos = getattr(ctx, '_diag_slide_pos_prev', None)
        prev_movement_confirm = getattr(ctx, '_diag_slide_movement_confirm_prev', None)

        ctx.slide_pos = (float(x), float(y), float(z))
        ctx.slide_vel = (float(vx), float(vy), float(vz))
        ctx.slide_reached = bool(reached)
        ctx.slide_pos_stamp = float(stamp)

        if rn == 'tracer1' and self.state == 'SYNC_SLIDE_ALIGN' and bool(getattr(ctx, 'fine_active', False)):
            phase = str(getattr(ctx, 'direct_align_phase', ''))
            if bool(reached) != prev_reached:
                self.get_logger().warn(
                    f'[SLIDE_TRACE][tracer1] bottom_ack phase={phase} '
                    f'reached_target={int(bool(reached))} '
                    f'pos=({float(x):.1f},{float(y):.1f},{float(z):.1f}) '
                    f'vel=({float(vx):.1f},{float(vy):.1f},{float(vz):.1f})'
                )

            if phase in ('xy_sent', 'xy_barrier'):
                status = self._sync_slide_xy_phase_status('tracer1')
                movement_confirm = bool(status['movement_confirm'])
                pos_changed = (
                    prev_pos is None or
                    math.hypot(float(x) - float(prev_pos[0]), float(y) - float(prev_pos[1])) >= 0.5 or
                    abs(float(z) - float(prev_pos[2])) >= 0.5
                )
                confirm_changed = (prev_movement_confirm is None) or (bool(prev_movement_confirm) != movement_confirm)
                blocked_periodic = (not movement_confirm) and self._diag_ok(ctx, '_diag_tracer1_slide_blocked_epoch', 0.5)
                if confirm_changed or blocked_periodic or (pos_changed and self._diag_ok(ctx, '_diag_tracer1_slide_feedback_epoch', 0.25)):
                    self.get_logger().warn(
                        f'[SLIDE_TRACE][tracer1] status_feedback phase={phase} '
                        f'pos=({float(x):.1f},{float(y):.1f},{float(z):.1f}) '
                        f'dxy=({float(status["move_dx_mm"]):.1f},{float(status["move_dy_mm"]):.1f}) '
                        f'pos_fresh={int(bool(status["pos_fresh"]))} '
                        f'reached_target={int(bool(reached))} '
                        f'movement_confirm={int(movement_confirm)} '
                        f'confirm_reason={status["confirm_reason"]}'
                    )
                ctx._diag_slide_movement_confirm_prev = movement_confirm
            else:
                ctx._diag_slide_movement_confirm_prev = None

        ctx._diag_slide_reached_prev = bool(reached)
        ctx._diag_slide_pos_prev = (float(x), float(y), float(z))

    def _apply_force_contact_sample_locked(self, rn: str, sample):
        if not bool(sample):
            return
        try:
            if not self._force_contact_ok(rn):
                return
        except Exception:
            return

        ctx = self.rt[rn]
        now = _now_sec(self)
        newly_latched = not bool(getattr(ctx, 'contact_confirmed', False))
        ctx.contact_confirmed = True
        if newly_latched:
            ctx.force_contact_epoch = now
        if self.state == 'SYNC_SLIDE_ALIGN' and ctx.fine_active:
            phase = str(getattr(ctx, 'direct_align_phase', ''))
            if phase in ('z_sent', 'done'):
                self._latch_post_contact_state(rn, ctx, source='force_contact_topic')
                self.stop_slide_position(rn)
                self.stop_slide_comp(rn)
        if newly_latched and self._diag_ok(ctx, "_diag_force_contact_epoch", 0.5):
            self.get_logger().warn(
                f"[FORCE_CONTACT][{rn}] force_monitor contact latched; contact_confirmed=True"
            )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def wing_cb(self, msg: PoseStamped):
        self._wing_pose_rx.set((
            float(msg.pose.position.x),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
            _now_sec(self),
        ))

    def _mk_robot_mocap_cb(self, rn: str):
        def cb(msg: PoseStamped):
            self._robot_mocap_rx[rn].set((
                float(msg.pose.position.x),
                float(msg.pose.position.z),
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
                _now_sec(self),
            ))
        return cb

    # Odom / IMU callbacks are intentionally left unused in the current setup.
    # def _mk_odom_cb(self, rn: str):
    #     def cb(msg: Odometry):
    #         tw = msg.twist.twist
    #         self._odom_rx[rn].set((
    #             float(tw.linear.x),
    #             float(tw.linear.y),
    #             float(tw.angular.z),
    #             _now_sec(self),
    #         ))
    #     return cb

    # def _mk_imu_cb(self, rn: str):
    #     def cb(msg: Imu):
    #         self._imu_rx[rn].set((float(msg.angular_velocity.z), _now_sec(self)))
    #     return cb

    def _mk_reached_cb(self, rn: str):
        def cb(msg: Bool):
            self._reached_rx[rn].set(bool(msg.data))
        return cb

    def _mk_force_cb(self, rn: str):
        def cb(msg: Float32MultiArray):
            if len(msg.data) < 4:
                return
            now = _now_sec(self)
            self._force_rx[rn].put((
                now,
                float(msg.data[0]),
                float(msg.data[1]),
                float(msg.data[2]),
                float(msg.data[3]),
            ))
        return cb

    def _mk_raw_qr_cb(self, rn: str):
        def cb(msg: PoseStamped):
            p = msg.pose.position
            receive_stamp = _now_sec(self)
            source_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            source_valid = self._valid_source_stamp(source_stamp)
            self._raw_qr_rx[rn].put((
                float(p.x),
                float(p.y),
                float(p.z),
                source_stamp,
                receive_stamp,
                source_valid,
            ))
        return cb

    def _mk_delta_cb(self, rn: str):
        def cb(msg: Vector3Stamped):
            now_wall = time.time()
            now_ros = _now_sec(self)
            msg_t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self._delta_rx[rn].put((
                msg_t,
                float(msg.vector.x),
                float(msg.vector.y),
                float(msg.vector.z),
                now_ros,
                now_wall,
                self._valid_source_stamp(msg_t),
            ))
        return cb

    def _mk_slide_status_cb(self, rn: str):
        def cb(msg: MotorStatus):
            self._slide_status_rx[rn].set((
                float(msg.x),
                float(msg.y),
                float(msg.z),
                float(msg.vx),
                float(msg.vy),
                float(msg.vz),
                bool(msg.reached_target),
                _now_sec(self),
            ))
        return cb

    def _force_contact_topic_for_robot(self, rn: str) -> str:
        slide_key = self._slide_actor_key(rn)
        return (
            str(self.force_contact_topic_template)
            .replace('{robot}', str(rn))
            .replace('{slide}', str(slide_key))
        )

    def _mk_force_contact_cb(self, rn: str):
        def cb(msg: Bool):
            if bool(msg.data):
                self._force_contact_rx[rn].put(True)
        return cb

    def _emergency_cb(self, msg: Bool):
        with self._state_lock:
            if bool(msg.data) and not self.emergency:
                self.emergency = True
                self.get_logger().error('[EMERGENCY] emergency_stop=True -> stop_all()')
                self.stop_all()
                self.stop_all_slide_comp()

    def _workflow_allows_state(self, target_state: str) -> bool:
        if bool(getattr(self, '_start_state_override_active', False)):
            return True
        return MissionStateHelpersMixin._workflow_allows_state(self, target_state)

    def _valid_start_states(self):
        return {
            'wait_wing',
            'run_alignment',
            'phase1_done_hold',
            'sync_slide_align',
            'all_ready_hold',
            'sync_level_z',
            'sync_recenter',
            'load_stable_hold',
            'transport_precheck',
            'sync_transport',
            'transport_settle',
        }

    @staticmethod
    def _normalize_resume_phase(phase_name: str) -> str:
        phase = str(phase_name or '').lower().strip()
        if phase in ('', 'approach', 'slide_align', 'level_recenter', 'transport'):
            return phase
        return ''

    def _apply_resume_phase_override(self):
        normalized = MissionCoordinator._normalize_resume_phase(getattr(self, 'resume_phase', ''))
        raw = str(getattr(self, 'resume_phase', '') or '').lower().strip()
        if raw and not normalized:
            self.get_logger().warn(
                f'[RESUME_PHASE] invalid resume_phase={raw}; fallback to full startup flow'
            )
        self.resume_phase = normalized
        self.startup_resume_phase = normalized or 'none'
        self.startup_resume_pending = bool(normalized)
        if not normalized:
            return

        if str(getattr(self, 'start_state', '') or '').strip():
            self.get_logger().warn(
                f'[RESUME_PHASE] resume_phase={normalized} overrides start_state={self.start_state}'
            )
            self.start_state = ''

        if not bool(getattr(self, 'managed_phase_mode', False)):
            self.get_logger().warn(
                f'[RESUME_PHASE] resume_phase={normalized} forces managed_phase_mode=true'
            )
        self.managed_phase_mode = True

    def _resume_slide_align_precheck_ok(self):
        ready, reason = self._start_state_mocap_ready()
        if not ready:
            return False, reason

        bad = []
        threshold = max(
            float(getattr(self, 'raw_qr_accept_radius_m', 0.18)),
            float(getattr(self, 'transport_finish_pos_tol_m', 0.08)),
        )
        for rn in self.robots:
            if rn not in self.robot_xy:
                bad.append(f'{rn}:no_mocap')
                continue
            xt, yt, _ = self.predict_target_world(rn, 0.0, 0.0)
            rx, ry = self.robot_xy[rn]
            dist = math.hypot(float(rx) - float(xt), float(ry) - float(yt))
            if dist > threshold:
                bad.append(f'{rn}:dist={dist:.3f}>{threshold:.3f}')

        if bad:
            return False, f'coarse final pose missing: {bad}'
        return True, ''

    def _prime_resume_slide_align_context(self):
        now = _now_sec(self)
        for rn in self.robots:
            ctx = self.rt[rn]
            self._reset_runtime_for_new_mission_leg(rn, clear_alignment=True)
            xt, yt, yaw_rad = self.predict_target_world(rn, 0.0, 0.0)
            ctx.final_target = (xt, yt)
            ctx.goal_kind = 'FINAL'
            ctx.locked_yaw = yaw_rad
            ctx.staged = False
            ctx.entered = True
            ctx.first_qr_locked = True
            ctx.first_qr_lock_epoch = now
            ctx.local_state = 'WAIT_ALL_QR_LOCK'

    def _resume_level_recenter_precheck_ok(self):
        ok, reason = self._resume_slide_align_precheck_ok()
        if not ok:
            return False, reason

        missing = []
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue
            if ctx.slide_pos is None or ctx.slide_vel is None:
                missing.append(f'{rn}:slide_status_missing')
                continue
            if not self._force_msg_fresh(rn, self.load_level_force_fresh_timeout_sec):
                missing.append(f'{rn}:force_stale')
        if missing:
            return False, '; '.join(missing)
        return True, ''

    def _prime_resume_ready_context(self):
        self._prime_resume_slide_align_context()
        now = _now_sec(self)
        for rn in self.robots:
            ctx = self.rt[rn]
            ctx.confirmed = True
            ctx.align_done = True
            ctx.align_done_epoch = now
            ctx.ready_to_lift = True
            ctx.ready_epoch = now
            ctx.local_state = 'READY_TO_LIFT'

    def _resume_transport_precheck_ok(self):
        ok, reason = self._resume_level_recenter_precheck_ok()
        if not ok:
            return False, reason
        self._prime_resume_ready_context()
        return self._transport_precheck_ok()

    def _begin_approach_phase(self, reason: str):
        self.managed_active_phase = 'approach'
        self._wing_frozen = False
        self._pending_initial_dispatch = False
        self._set_global_state('WAIT_WING', reason)
        return True, MissionCoordinator._managed_status_summary(self)

    def _begin_slide_align_phase(self, reason: str):
        self.managed_active_phase = 'slide_align'
        self.stop_all()
        self.stop_all_slide_comp()
        self._start_sync_slide_align()
        self._set_global_state('SYNC_SLIDE_ALIGN', reason)
        return True, MissionCoordinator._managed_status_summary(self)

    def _begin_level_recenter_phase(self, reason: str):
        self.managed_active_phase = 'level_recenter'
        self.stop_all()
        self.stop_all_slide_comp()
        self.load_stable_since = 0.0
        if self.transport_enable and self.load_level_enable:
            self._start_level_z_all()
            self._set_global_state('SYNC_LEVEL_Z', reason)
            return True, MissionCoordinator._managed_status_summary(self)
        if self.transport_enable and getattr(self, 'slide_recenter_enable', False):
            self.recenter_all_done_since = 0.0
            self._start_recenter_all()
            self._set_global_state('SYNC_RECENTER', reason)
            return True, MissionCoordinator._managed_status_summary(self)
        self._set_global_state('LOAD_STABLE_HOLD', f'{reason} (hold only)')
        return True, MissionCoordinator._managed_status_summary(self)

    def _begin_transport_phase(self, reason: str):
        self.managed_active_phase = 'transport'
        self.stop_all()
        self.stop_all_slide_comp()
        self.transport_precheck_last_log = 0.0
        self._set_global_state('TRANSPORT_PRECHECK', reason)
        return True, MissionCoordinator._managed_status_summary(self)

    def _startup_resume_request(self):
        phase = MissionCoordinator._normalize_resume_phase(
            getattr(self, 'startup_resume_phase', getattr(self, 'resume_phase', ''))
        )
        if not phase:
            return False, 'startup resume phase not set'

        if phase == 'approach':
            self.startup_resume_pending = False
            self.managed_active_phase = 'approach'
            self._begin_approach_phase('startup resume_phase=approach')
            return True, f'resume_phase=approach | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'slide_align':
            ok, reason = self._resume_slide_align_precheck_ok()
            if not ok:
                return False, reason
            self._prime_resume_slide_align_context()
            self.startup_resume_pending = False
            self._begin_slide_align_phase('startup resume_phase=slide_align')
            return True, f'resume_phase=slide_align | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'level_recenter':
            ok, reason = self._resume_level_recenter_precheck_ok()
            if not ok:
                return False, reason
            self._prime_resume_ready_context()
            self.startup_resume_pending = False
            self._begin_level_recenter_phase('startup resume_phase=level_recenter')
            return True, f'resume_phase=level_recenter | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'transport':
            ok, reason = self._resume_transport_precheck_ok()
            if not ok:
                return False, reason
            self.startup_resume_pending = False
            self._begin_transport_phase('startup resume_phase=transport')
            return True, f'resume_phase=transport | {MissionCoordinator._managed_status_summary(self)}'

        return False, f'unsupported resume_phase={phase}'

    def _cooperative_common_x(self) -> float:
        return float(self.wing_x) - float(self.cooperative_common_x_backoff_m)

    def _cooperative_wait_side_sign(self) -> float:
        anchor = str(getattr(self, 'cooperative_anchor_robot', '') or '')
        if anchor not in getattr(self, 'robots', []):
            robots = getattr(self, 'robots', [])
            anchor = robots[0] if robots else 'tracer1'

        deadband_m = max(float(getattr(self, 'path_min_seg_m', 0.03)), 0.05)
        wing_y = float(getattr(self, 'wing_y', 0.0) or 0.0)

        if (
            str(getattr(self, 'cooperative_wait_side_mode', 'follow_tracer1') or 'follow_tracer1')
            == 'follow_tracer1'
        ):
            robot_xy = getattr(self, 'robot_xy', {})
            if anchor in robot_xy:
                delta_live = float(robot_xy[anchor][1]) - wing_y
                if abs(delta_live) > deadband_m:
                    return 1.0 if delta_live > 0.0 else -1.0

        start_line_map = getattr(self, 'cooperative_start_line_y_map', {})
        if anchor in start_line_map:
            delta_start = float(start_line_map[anchor]) - wing_y
            if abs(delta_start) > deadband_m:
                return 1.0 if delta_start > 0.0 else -1.0

        return -1.0

    def _cooperative_wait_targets(self):
        common_x = self._cooperative_common_x()
        wait_y_anchor = (
            float(self.wing_y)
            + float(self._cooperative_wait_side_sign()) * float(self.cooperative_wait_tracer1_offset_y_m)
        )
        return {
            rn: (common_x, float(wait_y_anchor) + float(self.cooperative_line_offsets_m[rn]))
            for rn in self.robots
        }

    def _cooperative_x_targets(self):
        common_x = self._cooperative_common_x()
        return {
            rn: (common_x, float(self.robot_xy[rn][1]))
            for rn in self.robots
            if rn in self.robot_xy
        }

    def _all_cooperative_leg_complete(self, goal_kind: str) -> bool:
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                return False
            if ctx.goal_kind != goal_kind:
                return False
            if ctx.segs is not None:
                return False
        return True

    def _dispatch_cooperative_leg(self, goal_kind: str, targets, local_state: str, reason: str, clear_alignment: bool):
        dispatched = False
        for rn in self.robots:
            if rn not in targets:
                continue
            ctx = self.rt[rn]
            if ctx.faulted or ctx.finished:
                continue

            if clear_alignment:
                self._reset_runtime_for_new_mission_leg(rn, clear_alignment=True)
                ctx.micro_i = 0
                ctx.micro_attempts = 0
                ctx.staged = False
                ctx.entered = False
            else:
                ctx.segs = None
                ctx.seg_i = 0
                ctx.dwell_start = 0.0
                ctx.dwell_locked = False
                ctx.fine_active = False
                ctx.sync_wait_qr = False
                ctx.sync_wait_qr_epoch = 0.0
                ctx.gate_stopped = False
                ctx.gate_hold_until = 0.0

            xt, yt = targets[rn]
            ctx.goal_kind = goal_kind
            ctx.locked_yaw = None
            ctx.staging_target = (float(xt), float(yt))
            ctx.segs = [(float(xt), float(yt))]
            ctx.seg_i = 0

            self._set_local_state(rn, local_state, reason)
            self.precision_on(rn, False)
            self.resume_one(rn)
            self._send_current_segment(rn, tag=goal_kind)
            dispatched = True

        return dispatched

    def _mark_cooperative_wait_line_ready(self):
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue
            ctx.staged = True
            ctx.entered = False
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            self._set_local_state(rn, 'WAIT_ENTRY', 'cooperative waiting line reached')

    def _managed_boundary_state_for_phase(self, phase_name: str) -> str:
        phase = str(phase_name or '').lower().strip()
        return {
            'approach': 'PHASE1_DONE_HOLD',
            'slide_align': 'ALL_READY_HOLD',
            'level_recenter': 'LOAD_STABLE_HOLD',
            'transport': 'DONE',
        }.get(phase, '')

    def _managed_status_summary(self) -> str:
        active = str(getattr(self, 'managed_active_phase', '') or 'idle')
        completed_phases = sorted(str(x) for x in getattr(self, 'managed_completed_phases', set()))
        completed = ','.join(completed_phases) if completed_phases else 'none'
        enabled = int(bool(getattr(self, 'managed_phase_mode', False)))
        startup_resume_phase = str(getattr(self, 'startup_resume_phase', 'none') or 'none')
        return (
            f'managed_phase_mode={enabled} '
            f'startup_resume_phase={startup_resume_phase} '
            f'current_state={getattr(self, "state", "")} '
            f'active_phase={active} '
            f'completed={completed}'
        )

    def _managed_should_hold_boundary(self, state_name: str) -> bool:
        if not bool(getattr(self, 'managed_phase_mode', False)):
            return False
        return str(state_name or '') in {
            'STANDBY',
            'PHASE1_DONE_HOLD',
            'ALL_READY_HOLD',
            'LOAD_STABLE_HOLD',
        }

    def _managed_mark_phase_complete(self, phase_name: str, boundary_state: str):
        if not bool(getattr(self, 'managed_phase_mode', False)):
            return
        phase = str(phase_name or '').lower().strip()
        if not phase:
            return
        completed = getattr(self, 'managed_completed_phases', None)
        if completed is None:
            completed = set()
            self.managed_completed_phases = completed
        completed.add(phase)
        if str(getattr(self, 'managed_active_phase', '')).lower().strip() == phase:
            self.managed_active_phase = ''
        self.get_logger().warn(
            f'[MANAGED_PHASE] phase={phase} completed at {boundary_state} | {self._managed_status_summary()}'
        )

    def _reset_managed_runtime_state(self):
        self._pending_initial_dispatch = False
        self._dispatch_epoch = 0.0
        self.all_ready_since = 0.0
        self.sync_lift_since = 0.0
        self.recenter_all_done_since = 0.0
        self.transport_dispatch_epoch = 0.0
        self.transport_precheck_last_log = 0.0
        self.load_stable_since = 0.0
        self.loaded_z_plane_mm = None
        self.workflow_boundary_reached = False
        self._start_state_handled = False
        self._start_state_override_active = False
        self._start_state_wait_last_log = 0.0
        self.preflight_last_log = 0.0
        self.preflight_ok = bool(
            getattr(self, 'managed_phase_mode', False) and
            (not getattr(self, 'start_state', '')) and
            (not getattr(self, 'startup_resume_pending', False))
        )
        self._wing_frozen = False

        for rn in self.robots:
            ctx = self.rt[rn]
            self._reset_runtime_for_new_mission_leg(rn, clear_alignment=True)
            ctx.reached = False
            ctx.confirmed = False
            ctx.last_goal_epoch = 0.0
            ctx.local_state = 'IDLE'
            ctx.final_target = None
            ctx.staging_target = None
            ctx.transport_target = None
            ctx.goal_kind = 'FINAL'
            ctx.staged = False
            ctx.entered = False
            ctx.locked_yaw = None
            ctx.micro_i = 0
            ctx.micro_attempts = 0
            ctx.recenter_done = False
            ctx.recenter_target = None
            ctx.level_z_done = False
            ctx.level_z_target_mm = None
            ctx.level_z_start_epoch = 0.0
            ctx.level_active = False
            ctx.level_done = False
            ctx.level_target_z_mm = None
            ctx.transport_center_ref = None
            ctx.loaded_ref_captured = False
            ctx.chassis_check_last_xy = None
            ctx.chassis_check_last_stamp = 0.0
            ctx.last_cmd_v = 0.0
            ctx.last_cmd_w = 0.0

    def _managed_enter_standby(self, reason: str, clear_runtime: bool = False, clear_phase_progress: bool = False):
        self.stop_all()
        self.stop_all_slide_comp()
        self.managed_active_phase = ''
        if clear_phase_progress:
            self.managed_completed_phases.clear()
        if clear_runtime:
            self._reset_managed_runtime_state()
        self._set_global_state('STANDBY', reason)

    def _managed_phase_request(self, phase_name: str):
        phase = str(phase_name or '').lower().strip()
        status_prefix = MissionCoordinator._managed_status_summary(self)
        if not bool(getattr(self, 'managed_phase_mode', False)):
            return False, f'managed phase mode disabled | {status_prefix}'
        if bool(getattr(self, 'emergency', False)):
            return False, f'phase={phase} rejected emergency stop active | {status_prefix}'
        has_fault = getattr(self, '_has_any_fault', lambda: False)()
        if has_fault:
            faulted = [rn for rn in self.robots if getattr(self.rt[rn], 'faulted', False)]
            return False, f'phase={phase} rejected faulted robots={faulted} | {status_prefix}'

        boundary_state = MissionCoordinator._managed_boundary_state_for_phase(self, phase)
        if not boundary_state:
            return False, f'unsupported phase={phase} | {status_prefix}'

        if phase in getattr(self, 'managed_completed_phases', set()) and self.state == boundary_state:
            return True, f'phase={phase} already completed | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'approach':
            if self.state != 'STANDBY':
                return False, f'phase=approach rejected current_state={self.state}; expected STANDBY | {status_prefix}'
            self._begin_approach_phase('managed phase request: approach')
            return True, f'phase=approach accepted -> WAIT_WING | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'slide_align':
            if self.state != 'PHASE1_DONE_HOLD':
                return False, (
                    f'phase=slide_align rejected current_state={self.state}; '
                    f'expected PHASE1_DONE_HOLD | {status_prefix}'
                )
            if not self._all_first_qr_locked():
                return False, f'phase=slide_align rejected QR locks incomplete | {status_prefix}'
            self._begin_slide_align_phase('managed phase request: slide_align')
            return True, f'phase=slide_align accepted -> SYNC_SLIDE_ALIGN | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'level_recenter':
            if self.state != 'ALL_READY_HOLD':
                return False, (
                    f'phase=level_recenter rejected current_state={self.state}; '
                    f'expected ALL_READY_HOLD | {status_prefix}'
                )
            if not self._all_ready_to_lift():
                return False, f'phase=level_recenter rejected ready_to_lift incomplete | {status_prefix}'
            self._begin_level_recenter_phase('managed phase request: level_recenter')
            if self.state == 'SYNC_LEVEL_Z':
                return True, f'phase=level_recenter accepted -> SYNC_LEVEL_Z | {MissionCoordinator._managed_status_summary(self)}'
            if self.state == 'SYNC_RECENTER':
                return True, f'phase=level_recenter accepted -> SYNC_RECENTER | {MissionCoordinator._managed_status_summary(self)}'
            return True, f'phase=level_recenter accepted -> LOAD_STABLE_HOLD | {MissionCoordinator._managed_status_summary(self)}'

        if phase == 'transport':
            if self.state != 'LOAD_STABLE_HOLD':
                return False, (
                    f'phase=transport rejected current_state={self.state}; '
                    f'expected LOAD_STABLE_HOLD | {status_prefix}'
                )
            if not self._all_load_stable():
                return False, f'phase=transport rejected load not stable | {status_prefix}'
            self._begin_transport_phase('managed phase request: transport')
            return True, f'phase=transport accepted -> TRANSPORT_PRECHECK | {MissionCoordinator._managed_status_summary(self)}'

        return False, f'unsupported phase={phase} | {status_prefix}'

    def _handle_phase_request(self, phase_name: str, response):
        with self._state_lock:
            success, message = self._managed_phase_request(phase_name)
            response.success = bool(success)
            response.message = message
            return response

    def _handle_start_approach(self, request, response):
        del request
        return self._handle_phase_request('approach', response)

    def _handle_start_slide_align(self, request, response):
        del request
        return self._handle_phase_request('slide_align', response)

    def _handle_start_level_recenter(self, request, response):
        del request
        return self._handle_phase_request('level_recenter', response)

    def _handle_start_transport(self, request, response):
        del request
        return self._handle_phase_request('transport', response)

    def _handle_reset_to_standby(self, request, response):
        del request
        with self._state_lock:
            if not self.managed_phase_mode:
                response.success = False
                response.message = f'managed phase mode disabled | {self._managed_status_summary()}'
                return response
            self._managed_enter_standby(
                'managed phase request: reset_to_standby',
                clear_runtime=True,
                clear_phase_progress=True,
            )
            response.success = True
            response.message = self._managed_status_summary()
            return response

    def _handle_get_status(self, request, response):
        del request
        with self._state_lock:
            response.success = True
            response.message = self._managed_status_summary()
            return response

    def _start_state_requires_mocap_ready(self) -> bool:
        requested = str(getattr(self, 'start_state', '') or '').lower().strip()
        return (
            (not self._start_state_handled)
            and (requested in self._valid_start_states())
            and (requested != 'wait_wing')
        )

    def _start_state_mocap_ready(self):
        if self.wing_x is None or self.wing_y is None or self.wing_yaw is None:
            return False, 'wing pose/yaw unavailable'

        missing = [
            rn for rn in self.robots
            if (rn not in self.robot_xy) or (rn not in self.robot_yaw)
        ]
        if missing:
            return False, f'robot mocap missing: {missing}'

        fresh, bad = self._mocap_fresh()
        if not fresh:
            return False, f'mocap stale: {bad}'

        return True, ''

    def _enter_start_state_after_preflight(self) -> bool:
        if self._start_state_handled:
            return False
        self._start_state_handled = True

        requested = str(getattr(self, 'start_state', '') or '').lower().strip()
        self.get_logger().warn(f'[START_STATE] requested={requested}')

        if not requested:
            return False

        valid_states = self._valid_start_states()
        if requested not in valid_states:
            self.get_logger().warn(
                f'[START_STATE] requested={requested} invalid; fallback to default workflow logic'
            )
            return False

        self._start_state_override_active = True
        now_ros = _now_sec(self)

        if requested == 'wait_wing':
            self._pending_initial_dispatch = False
            self._set_global_state('WAIT_WING', 'start_state=wait_wing after preflight')
            self.get_logger().warn('[START_STATE] entered WAIT_WING')
            return True

        if requested == 'run_alignment':
            if self.freeze_wing_on_start:
                self._wing_frozen = True
            self._pending_initial_dispatch = True
            self._set_global_state('RUN_ALIGNMENT', 'start_state=run_alignment after preflight')
            self.get_logger().warn('[START_STATE] entered RUN_ALIGNMENT')
            return True

        if requested == 'phase1_done_hold':
            self.stop_all()
            self.stop_all_slide_comp()
            self._set_global_state('PHASE1_DONE_HOLD', 'start_state=phase1_done_hold after preflight')
            self.get_logger().warn('[START_STATE] entered PHASE1_DONE_HOLD')
            return True

        if requested == 'sync_slide_align':
            self._start_sync_slide_align()
            self._set_global_state('SYNC_SLIDE_ALIGN', 'start_state=sync_slide_align after preflight')
            self.get_logger().warn('[START_STATE] entered SYNC_SLIDE_ALIGN')
            return True

        if requested == 'all_ready_hold':
            self.stop_all()
            self.stop_all_slide_comp()
            self.all_ready_since = now_ros
            self._set_global_state('ALL_READY_HOLD', 'start_state=all_ready_hold after preflight')
            self.get_logger().warn('[START_STATE] entered ALL_READY_HOLD')
            return True

        if requested == 'sync_level_z':
            self.load_stable_since = 0.0
            self._start_level_z_all()
            self._set_global_state('SYNC_LEVEL_Z', 'start_state=sync_level_z after preflight')
            self.get_logger().warn('[START_STATE] entered SYNC_LEVEL_Z')
            return True

        if requested == 'sync_recenter':
            self.recenter_all_done_since = 0.0
            self._start_recenter_all()
            self._set_global_state('SYNC_RECENTER', 'start_state=sync_recenter after preflight')
            self.get_logger().warn('[START_STATE] entered SYNC_RECENTER')
            return True

        if requested == 'load_stable_hold':
            self.stop_all()
            self.stop_all_slide_comp()
            self.load_stable_since = 0.0
            self._set_global_state('LOAD_STABLE_HOLD', 'start_state=load_stable_hold after preflight')
            self.get_logger().warn('[START_STATE] entered LOAD_STABLE_HOLD')
            return True

        if requested == 'transport_precheck':
            self.stop_all()
            self.stop_all_slide_comp()
            self.transport_precheck_last_log = 0.0
            self._set_global_state('TRANSPORT_PRECHECK', 'start_state=transport_precheck after preflight')
            self.get_logger().warn('[START_STATE] entered TRANSPORT_PRECHECK')
            return True

        if requested == 'sync_transport':
            self.stop_all()
            self.stop_all_slide_comp()
            ok, reason = self._transport_precheck_ok()
            if not ok:
                self.get_logger().warn(
                    f'[START_STATE] requested=sync_transport precheck failed: {reason}; fallback to TRANSPORT_PRECHECK'
                )
                self.transport_precheck_last_log = 0.0
                self._set_global_state('TRANSPORT_PRECHECK', 'start_state=sync_transport fallback after precheck failure')
                return True
            self._capture_transport_center_ref()
            dispatched = self.dispatch_transport_all(force_refresh=False)
            if not dispatched:
                self.get_logger().warn(
                    '[START_STATE] requested=sync_transport dispatch failed; fallback to TRANSPORT_PRECHECK'
                )
                self.transport_precheck_last_log = 0.0
                self._set_global_state('TRANSPORT_PRECHECK', 'start_state=sync_transport fallback after dispatch failure')
                return True
            self.transport_dispatch_epoch = now_ros
            self._set_global_state('SYNC_TRANSPORT', 'start_state=sync_transport after preflight')
            self.get_logger().warn('[START_STATE] entered SYNC_TRANSPORT')
            return True

        if requested == 'transport_settle':
            self.stop_all()
            self.stop_all_slide_comp()
            self._set_global_state('TRANSPORT_SETTLE', 'start_state=transport_settle after preflight')
            self.get_logger().warn('[START_STATE] entered TRANSPORT_SETTLE')
            return True

        return False

    def _sync_slide_xy_motion_flags(self, ctx):
        deadband_mm = self._direct_align_pos_deadband_mm()
        need_x = abs(float(getattr(ctx, 'direct_align_xy_cmd_x_mm', 0.0))) >= deadband_mm
        need_y = abs(float(getattr(ctx, 'direct_align_xy_cmd_y_mm', 0.0))) >= deadband_mm
        return bool(need_x), bool(need_y)

    def _sync_slide_wait_qr_blocked(self, ctx) -> bool:
        return bool(getattr(ctx, 'sync_wait_qr', False)) or (str(getattr(ctx, 'local_state', '')) == 'WAIT_QR')

    def _sync_slide_xy_phase_status(self, rn: str):
        ctx = self.rt[rn]
        phase = str(getattr(ctx, 'direct_align_phase', ''))
        local_state = str(getattr(ctx, 'local_state', ''))
        wait_qr_blocked = self._sync_slide_wait_qr_blocked(ctx)
        need_x, need_y = self._sync_slide_xy_motion_flags(ctx)
        pos = getattr(ctx, 'slide_pos', None)
        start = getattr(ctx, 'direct_align_xy_start_pos', None)
        pos_fresh = self._slide_pos_fresh(ctx) if pos is not None else False

        move_dx_mm = 0.0
        move_dy_mm = 0.0
        if pos is not None and start is not None:
            move_dx_mm = abs(float(pos[0]) - float(start[0]))
            move_dy_mm = abs(float(pos[1]) - float(start[1]))

        if bool(getattr(ctx, 'ready_to_lift', False)):
            movement_confirm = True
            confirm_reason = 'ready_to_lift'
        elif wait_qr_blocked:
            movement_confirm = False
            confirm_reason = 'wait_qr'
        elif phase in ('xy_barrier', 'z_sent', 'done', 'ready_barrier'):
            movement_confirm = True
            confirm_reason = 'passed_xy'
        elif not (need_x or need_y):
            movement_confirm = False
            confirm_reason = 'no_xy_command'
        elif start is None:
            movement_confirm = False
            confirm_reason = 'xy_start_pos_missing'
        elif not pos_fresh:
            movement_confirm = False
            confirm_reason = 'slide_pos_stale_or_missing'
        else:
            confirm_mm = self._direct_align_xy_move_confirm_mm()
            x_ok = (not need_x) or (move_dx_mm >= confirm_mm)
            y_ok = (not need_y) or (move_dy_mm >= confirm_mm)
            movement_confirm = bool(x_ok and y_ok)
            if movement_confirm:
                confirm_reason = 'movement_confirmed'
            elif need_x and need_y and (move_dx_mm < confirm_mm) and (move_dy_mm < confirm_mm):
                confirm_reason = 'x_and_y_no_effective_motion'
            elif need_x and (move_dx_mm < confirm_mm):
                confirm_reason = 'x_no_effective_motion'
            elif need_y and (move_dy_mm < confirm_mm):
                confirm_reason = 'y_no_effective_motion'
            else:
                confirm_reason = 'movement_confirm_pending'

        ack = bool(getattr(ctx, 'slide_reached', False))
        elapsed_xy = max(0.0, _now_sec(self) - float(getattr(ctx, 'direct_align_epoch', 0.0)))
        expected_finish = float(getattr(ctx, 'direct_align_xy_time', 0.0)) + self._direct_align_settle_margin_sec()
        xy_reached_time = elapsed_xy >= expected_finish
        xy_phase_done = bool(ack or xy_reached_time) and bool(movement_confirm)

        if bool(getattr(ctx, 'ready_to_lift', False)):
            barrier_ready = True
        elif wait_qr_blocked:
            barrier_ready = False
        elif phase in ('xy_barrier', 'z_sent', 'done', 'ready_barrier'):
            barrier_ready = True
        elif phase == 'xy_sent':
            barrier_ready = bool(xy_phase_done)
        else:
            barrier_ready = False

        ack_early = (phase == 'xy_sent') and ack and (need_x or need_y) and (not movement_confirm)
        if ack_early:
            confirm_reason = 'ack_without_effective_motion'
        return {
            'rn': rn,
            'phase': phase,
            'local_state': local_state,
            'wait_qr_blocked': bool(wait_qr_blocked),
            'need_x': bool(need_x),
            'need_y': bool(need_y),
            'pos_fresh': bool(pos_fresh),
            'movement_confirm': bool(movement_confirm),
            'confirm_reason': str(confirm_reason),
            'ack': bool(ack),
            'xy_reached_time': bool(xy_reached_time),
            'xy_phase_done': bool(xy_phase_done),
            'barrier_ready': bool(barrier_ready),
            'ack_early': bool(ack_early),
            'move_dx_mm': float(move_dx_mm),
            'move_dy_mm': float(move_dy_mm),
        }

    def _sync_slide_trace_xy_cmd_sent(self, rn: str, ctx, x_mm: float, y_mm: float, move_time: float):
        if rn != 'tracer1':
            return
        start = getattr(ctx, 'direct_align_xy_start_pos', None)
        start_str = 'None'
        if start is not None:
            start_str = f'({float(start[0]):.1f},{float(start[1]):.1f},{float(start[2]):.1f})'
        self.get_logger().warn(
            f'[SLIDE_TRACE][tracer1] cmd_sent phase=xy_sent '
            f'relative_cmd=({float(x_mm):.1f},{float(y_mm):.1f},0.0) '
            f'start_pos={start_str} time={float(move_time):.2f}s'
        )

    def _sync_slide_enter_xy_barrier(self, rn: str, ctx, z_mm: float, reason: str):
        ctx.direct_align_phase = 'xy_barrier'
        ctx.direct_align_epoch = _now_sec(self)
        ctx.direct_align_pending_z_mm = float(z_mm)

        status = self._sync_slide_xy_phase_status(rn)
        if self._diag_ok(ctx, '_diag_slide_xy_barrier_ready_epoch', 0.5):
            self.get_logger().warn(
                f'[SLIDE_BARRIER_READY][{rn}] reason={reason} '
                f'ack={int(bool(status["ack"]))} '
                f'pos_fresh={int(bool(status["pos_fresh"]))} '
                f'movement_confirm={int(bool(status["movement_confirm"]))} '
                f'confirm_reason={status["confirm_reason"]} '
                f'pending_z_mm={float(z_mm):.1f}'
            )

    def _sync_slide_wait_for_xy_barrier(self, rn: str, source: str = 'xy_barrier') -> bool:
        if self.state != 'SYNC_SLIDE_ALIGN':
            return True
        if not self._alignment_vision_enabled():
            return True

        active = [
            robot for robot in self.robots
            if (not bool(getattr(self.rt[robot], 'faulted', False)))
            and (not bool(getattr(self.rt[robot], 'finished', False)))
        ]
        if not active:
            return True

        snapshot = [self._sync_slide_xy_phase_status(robot) for robot in active]
        blockers = [status for status in snapshot if not bool(status['barrier_ready'])]
        ctx = self.rt[rn]

        if blockers:
            if self._diag_ok(ctx, '_diag_slide_xy_barrier_wait_epoch', 0.5):
                blocker_text = ', '.join(
                    f'{status["rn"]}:phase={status["phase"]}'
                    f'/local={status["local_state"]}'
                    f'/wait_qr={int(bool(status["wait_qr_blocked"]))}'
                    f'/ack={int(bool(status["ack"]))}'
                    f'/pos_fresh={int(bool(status["pos_fresh"]))}'
                    f'/movement_confirm={int(bool(status["movement_confirm"]))}'
                    f'/reason={status["confirm_reason"]}'
                    for status in blockers
                )
                self.get_logger().warn(
                    f'[SLIDE_BARRIER] waiting requester={rn} source={source} blockers=[{blocker_text}]'
                )
            return False

        if self._diag_ok(ctx, '_diag_slide_xy_barrier_release_epoch', 1.0):
            ready_text = ', '.join(
                f'{status["rn"]}:phase={status["phase"]}'
                f'/ack={int(bool(status["ack"]))}'
                f'/movement_confirm={int(bool(status["movement_confirm"]))}'
                f'/reason={status["confirm_reason"]}'
                for status in snapshot
            )
            self.get_logger().warn(
                f'[SLIDE_BARRIER] released requester={rn} source={source} ready=[{ready_text}]'
            )
        return True

    def _sync_slide_ready_phase_status(self, rn: str):
        ctx = self.rt[rn]
        phase = str(getattr(ctx, 'direct_align_phase', ''))
        local_state = str(getattr(ctx, 'local_state', ''))
        wait_qr_blocked = self._sync_slide_wait_qr_blocked(ctx)

        if bool(getattr(ctx, 'ready_to_lift', False)):
            barrier_ready = True
            ready_reason = 'ready_to_lift'
        elif wait_qr_blocked:
            barrier_ready = False
            ready_reason = 'wait_qr'
        elif phase == 'ready_barrier':
            barrier_ready = True
            ready_reason = 'ready_barrier'
        else:
            barrier_ready = False
            ready_reason = f'phase={phase or "idle"}'

        return {
            'rn': rn,
            'phase': phase,
            'local_state': local_state,
            'wait_qr_blocked': bool(wait_qr_blocked),
            'ready_to_lift': bool(getattr(ctx, 'ready_to_lift', False)),
            'barrier_ready': bool(barrier_ready),
            'ready_reason': str(ready_reason),
        }

    def _sync_slide_enter_ready_barrier(self, rn: str, ctx, dz: float, reason: str):
        ctx.direct_align_phase = 'ready_barrier'
        ctx.direct_align_epoch = _now_sec(self)
        ctx.direct_align_ready_dz_m = float(dz)
        if self._diag_ok(ctx, '_diag_slide_ready_barrier_epoch', 0.5):
            self.get_logger().warn(
                f'[SLIDE_READY_BARRIER_READY][{rn}] reason={reason} '
                f'dz={float(dz):.4f}m local_state={ctx.local_state}'
            )

    def _sync_slide_wait_for_ready_barrier(self, rn: str, source: str = 'ready_barrier') -> bool:
        if self.state != 'SYNC_SLIDE_ALIGN':
            return True

        active = [
            robot for robot in self.robots
            if (not bool(getattr(self.rt[robot], 'faulted', False)))
            and (not bool(getattr(self.rt[robot], 'finished', False)))
        ]
        if not active:
            return True

        snapshot = [self._sync_slide_ready_phase_status(robot) for robot in active]
        blockers = [status for status in snapshot if not bool(status['barrier_ready'])]
        ctx = self.rt[rn]

        if blockers:
            if self._diag_ok(ctx, '_diag_slide_ready_barrier_wait_epoch', 0.5):
                blocker_text = ', '.join(
                    f'{status["rn"]}:phase={status["phase"]}'
                    f'/local={status["local_state"]}'
                    f'/wait_qr={int(bool(status["wait_qr_blocked"]))}'
                    f'/ready={int(bool(status["ready_to_lift"]))}'
                    f'/reason={status["ready_reason"]}'
                    for status in blockers
                )
                self.get_logger().warn(
                    f'[SLIDE_READY_BARRIER] waiting requester={rn} source={source} blockers=[{blocker_text}]'
                )
            return False

        if self._diag_ok(ctx, '_diag_slide_ready_barrier_release_epoch', 1.0):
            ready_text = ', '.join(
                f'{status["rn"]}:phase={status["phase"]}'
                f'/ready={int(bool(status["ready_to_lift"]))}'
                f'/reason={status["ready_reason"]}'
                for status in snapshot
            )
            self.get_logger().warn(
                f'[SLIDE_READY_BARRIER] released requester={rn} source={source} ready=[{ready_text}]'
            )
        return True

    # ------------------------------------------------------------------
    # common helpers
    # ------------------------------------------------------------------
    def _mission_task_phase(self, state_name: str = '') -> str:
        state = str(state_name or self.state or '').upper()
        active = str(getattr(self, 'managed_active_phase', '') or '').lower().strip()
        if active:
            return active
        if state in {'STANDBY'}:
            return 'standby'
        if state in {'WAIT_WING', 'RUN_ALIGNMENT', 'PHASE1_DONE_HOLD'}:
            return 'approach'
        if state in {'SYNC_SLIDE_ALIGN', 'ALL_READY_HOLD'}:
            return 'slide_align'
        if state in {'SYNC_LEVEL_Z', 'SYNC_RECENTER', 'LOAD_STABLE_HOLD'}:
            return 'level_recenter'
        if state in {'TRANSPORT_PRECHECK', 'SYNC_TRANSPORT', 'TRANSPORT_SETTLE', 'DONE'}:
            return 'transport'
        if state == 'ABORT':
            return 'abort'
        return state.lower()

    @staticmethod
    def _mission_fmt_optional(value) -> str:
        if value in ('', None):
            return ''
        try:
            return f'{float(value):.6f}'
        except (TypeError, ValueError):
            return str(value)

    def _mission_safe_state(self, rn: str = '') -> str:
        if bool(getattr(self, 'emergency', False)):
            return 'emergency_stop'
        if self.state == 'ABORT':
            return 'abort'
        if rn and rn in getattr(self, 'rt', {}):
            ctx = self.rt[rn]
            if bool(getattr(ctx, 'faulted', False)):
                return 'robot_fault'
            if bool(getattr(ctx, 'transport_failed', False)):
                return 'transport_failed'
            reason = str(getattr(ctx, 'group_stop_reason', '') or '').strip()
            if reason:
                return f'group_stop:{reason}'
        return ''

    def _mission_runtime_row(
        self,
        *,
        event_type: str,
        robot_id: str = '',
        phase: str = '',
        reason: str = '',
        note: str = '',
        **extra,
    ) -> dict:
        rn = '' if robot_id == 'fleet' else str(robot_id or '')
        team_scope = 'fleet' if robot_id == 'fleet' else ''
        alpha = self._bench_alpha(rn) if rn in getattr(self, 'rt', {}) else None
        freeze_state = self._bench_freeze_flag(rn) if rn in getattr(self, 'rt', {}) else ''
        dock_proxy = extra.get('docking_residual_proxy', extra.get('e_dock_proxy', ''))
        slide_proxy = extra.get('slide_residual_proxy', extra.get('e_slide', ''))
        support_proxy = extra.get('support_residual_proxy', extra.get('e_sup_proxy', ''))
        outcome = str(extra.get('outcome', '') or '')
        safe_abort_reason = ''
        if event_type == 'SAFE_ABORT' or outcome == 'safe_abort':
            safe_abort_reason = str(reason or extra.get('safe_abort_reason', '') or '')
        return {
            'run_id': self.run_id,
            'timestamp': f'{_now_sec(self):.6f}',
            'mission_state': str(self.state),
            'task_phase': phase or self._mission_task_phase(),
            'precision_mode': int(bool(self._bench_precision_state.get(rn, False))) if rn else '',
            'robot_id': rn,
            'team_scope': team_scope,
            'Delta_eff_proxy_ms': self._mission_fmt_optional(self._bench_delta_eff_proxy_ms(rn)) if rn else '',
            'S_eff': self._mission_fmt_optional(self._bench_s_eff(rn)) if rn else '',
            'F_eff': self._mission_fmt_optional(self._bench_f_eff(rn)) if rn else '',
            'base_authority_weight': self._mission_fmt_optional(1.0 - alpha) if alpha is not None else '',
            'slide_authority_weight': self._mission_fmt_optional(alpha) if alpha is not None else '',
            'authority_policy_mode': 'precision_mode_proxy' if alpha is not None else '',
            'freeze_state': freeze_state,
            'watchdog_or_safe_state': self._mission_safe_state(rn),
            'docking_residual_proxy': self._mission_fmt_optional(dock_proxy),
            'slide_residual_proxy': self._mission_fmt_optional(slide_proxy),
            'support_residual_proxy': self._mission_fmt_optional(support_proxy),
            'safe_abort_reason': safe_abort_reason,
            'event_type': event_type,
            'event_note': note or reason,
        }

    def _log_mission_runtime_event(self, **kwargs):
        logger = getattr(self, 'mission_runtime_logger', None)
        if logger is None:
            return
        logger.log(self._mission_runtime_row(**kwargs))

    def _set_global_state(self, new_state: str, reason: str = ''):
        old_state = str(getattr(self, 'state', ''))
        MissionStateHelpersMixin._set_global_state(self, new_state, reason)
        current_state = str(getattr(self, 'state', ''))
        if old_state == current_state:
            return
        event_type = 'STATE_TRANSITION'
        if current_state == 'ABORT':
            event_type = 'SAFE_ABORT' if self._bench_fleet_outcome() == 'safe_abort' else 'MISSION_ABORT'
        elif current_state == 'STANDBY' and 'reset_to_standby' in str(reason):
            event_type = 'RECOVERY_RESET'
        self._log_mission_runtime_event(
            event_type=event_type,
            robot_id='fleet',
            phase=self._mission_task_phase(current_state),
            reason=reason,
            note=f'{old_state}->{current_state}',
            state_from=old_state,
            state_to=current_state,
        )

    def stop_all(self):
        for rn in self.robots:
            self.stop_pub[rn].publish(Bool(data=True))
            self.precision_on(rn, False)

    def resume_one(self, rn: str):
        self.resume_pub[rn].publish(Bool(data=True))

    def precision_on(self, rn: str, on: bool):
        desired = bool(on)
        previous = bool(self._bench_precision_state.get(rn, False))
        if desired != previous:
            self._bench_precision_state[rn] = desired
            self._bench_emit_event(
                event_type='AUTH_SWITCH',
                robot_id=rn,
                phase=self.state.lower(),
                reason='precision_mode_on' if desired else 'precision_mode_off',
                note='mission_coordinator precision_mode publish',
                state_from='precision' if previous else 'coarse',
                state_to='precision' if desired else 'coarse',
            )
        self.precision_pub[rn].publish(Bool(data=bool(on)))

    def _bench_emit_event(
        self,
        *,
        event_type: str,
        robot_id: str = "",
        phase: str = "",
        reason: str = "",
        note: str = "",
        state_from: str = "",
        state_to: str = "",
        **extra,
    ):
        self._bench_trace.emit(
            event_type=event_type,
            event_path_id=TRACE_EVENT_PATH_CONTROL,
            robot_id=robot_id,
            phase=phase or self.state.lower(),
            reason=reason,
            note=note,
            state_from=state_from,
            state_to=state_to,
            **extra,
        )
        self._log_mission_runtime_event(
            event_type=event_type,
            robot_id=robot_id,
            phase=phase or self.state.lower(),
            reason=reason,
            note=note,
            state_from=state_from,
            state_to=state_to,
            **extra,
        )

    def _bench_freeze_flag(self, rn: str) -> int:
        global_freeze_states = {
            'PHASE1_DONE_HOLD',
            'ALL_READY_HOLD',
            'SYNC_LEVEL_Z',
            'SYNC_RECENTER',
            'LOAD_STABLE_HOLD',
            'TRANSPORT_PRECHECK',
            'TRANSPORT_SETTLE',
        }
        local_freeze_states = {'WAIT_QR', 'TRANSPORT_WAIT_SETTLE'}
        ctx = self.rt[rn]
        return int((self.state in global_freeze_states) or (ctx.local_state in local_freeze_states))

    def _bench_alpha(self, rn: str) -> float:
        return 1.0 if bool(self._bench_precision_state.get(rn, False)) else 0.0

    def _bench_mocap_age_sec(self, rn: str):
        ts = self.robot_pose_stamp.get(rn, 0.0)
        if ts <= 0.0:
            return float('nan')
        return _now_sec(self) - ts

    def _bench_delta_age_sec(self, rn: str):
        ctx = self.rt[rn]
        ts = getattr(ctx, 'last_delta_receive_stamp', None)
        if ts is None:
            ts = getattr(ctx, 'last_delta_stamp', None)
        if ts is None or float(ts) <= 0.0:
            return float('nan')
        return _now_sec(self) - float(ts)

    def _bench_qr_age_sec(self, rn: str):
        ctx = self.rt[rn]
        ts = getattr(ctx, 'raw_qr_seen_stamp', 0.0)
        if float(ts) <= 0.0:
            return float('nan')
        return _now_sec(self) - float(ts)

    def _bench_force_age_sec(self, rn: str):
        ctx = self.rt[rn]
        ts = getattr(ctx, 'force_stamp', 0.0)
        if float(ts) <= 0.0:
            return float('nan')
        return _now_sec(self) - float(ts)

    def _bench_slide_age_sec(self, rn: str):
        ctx = self.rt[rn]
        ts = getattr(ctx, 'slide_pos_stamp', 0.0)
        if float(ts) <= 0.0:
            return float('nan')
        return _now_sec(self) - float(ts)

    def _bench_chassis_twist_age_sec(self, rn: str):
        ctx = self.rt[rn]
        ts = getattr(ctx, 'mocap_twist_stamp', 0.0)
        if float(ts) <= 0.0:
            return float('nan')
        return _now_sec(self) - float(ts)

    def _bench_phase_sources(self):
        phase = self._mission_task_phase()
        if phase == 'approach':
            return {'mocap', 'qr'}
        if phase == 'slide_align':
            return {'delta', 'qr', 'slide', 'force'}
        if phase == 'level_recenter':
            return {'force', 'slide', 'delta'}
        if phase == 'transport':
            return {'mocap', 'delta', 'slide', 'force'}
        return set()

    def _bench_delta_eff_proxy_ms(self, rn: str):
        sources = self._bench_phase_sources()
        ages = []
        if 'mocap' in sources:
            ages.append(self._bench_mocap_age_sec(rn))
        if 'delta' in sources:
            ages.append(self._bench_delta_age_sec(rn))
        if 'qr' in sources:
            ages.append(self._bench_qr_age_sec(rn))
        if 'force' in sources:
            ages.append(self._bench_force_age_sec(rn))
        if 'slide' in sources:
            ages.append(self._bench_slide_age_sec(rn))
        valid = [a for a in ages if not (isinstance(a, float) and (a != a))]
        if not valid:
            return float('nan')
        return max(valid) * 1000.0

    def _bench_s_eff(self, rn: str):
        ctx = self.rt[rn]
        components = []
        sources = self._bench_phase_sources()
        now = _now_sec(self)

        if 'mocap' in sources:
            ts = self.robot_pose_stamp.get(rn, 0.0)
            timeout = float(getattr(self, 'mocap_timeout_sec', 0.5))
            if ts > 0.0 and timeout > 0.0:
                components.append(max(0.0, (now - ts)) / timeout)

        if 'delta' in sources:
            sts = getattr(ctx, 'last_delta_receive_stamp', None)
            if sts is None:
                sts = getattr(ctx, 'last_delta_stamp', None)
            timeout = float(getattr(self, 'delta_max_age_sec', 2.0))
            if sts is not None and float(sts) > 0.0 and timeout > 0.0:
                components.append(max(0.0, (now - float(sts))) / timeout)

        if 'qr' in sources:
            ts = getattr(ctx, 'raw_qr_seen_stamp', 0.0)
            timeout = float(getattr(self, 'raw_qr_seen_timeout_sec', 0.6))
            if float(ts) > 0.0 and timeout > 0.0:
                components.append(max(0.0, (now - float(ts))) / timeout)

        if 'force' in sources:
            ts = getattr(ctx, 'force_stamp', 0.0)
            timeout = float(getattr(self, 'load_level_force_fresh_timeout_sec', 0.4))
            if float(ts) > 0.0 and timeout > 0.0:
                components.append(max(0.0, (now - float(ts))) / timeout)

        if 'slide' in sources:
            ts = getattr(ctx, 'slide_pos_stamp', 0.0)
            timeout = float(getattr(self, 'slide_status_fresh_timeout_sec', 0.5))
            if float(ts) > 0.0 and timeout > 0.0:
                components.append(max(0.0, (now - float(ts))) / timeout)

        dock_proxy = self._bench_e_dock_proxy(rn)
        if dock_proxy is not None:
            tol = max(1e-6, float(getattr(self, 'fine_xy_tol_m', 0.005)))
            components.append(float(dock_proxy) / tol)

        slide_res = self._bench_e_slide(rn)
        if slide_res is not None:
            ref_tol = max(1e-6, float(getattr(self, 'slide_recenter_tol_mm', 1.0)))
            components.append(float(slide_res) / ref_tol)

        sup_proxy = self._bench_e_sup_proxy(rn)
        if sup_proxy is not None:
            components.append(float(sup_proxy))

        if self._bench_freeze_flag(rn):
            components.append(1.0)
        if bool(getattr(ctx, 'transport_failed', False)):
            components.append(1.5)
        if bool(getattr(ctx, 'faulted', False)):
            components.append(2.0)

        if not components:
            return 0.0
        return max(float(v) for v in components)

    def _bench_f_eff(self, rn: str):
        s = self._bench_s_eff(rn)
        return 1.0 / (1.0 + s)

    def _bench_v_base(self, rn: str):
        twist = self._transport_chassis_twist_body(rn)
        if twist is None:
            return None
        vx_body, vy_body, _ = twist
        return math.hypot(float(vx_body), float(vy_body))

    def _bench_e_slide(self, rn: str):
        ctx = self.rt[rn]
        cur = getattr(ctx, 'slide_pos', None)
        ref = getattr(ctx, 'recenter_target', None) or getattr(ctx, 'transport_center_ref', None)
        if cur is None or ref is None:
            return None
        dx = float(cur[0]) - float(ref[0])
        dy = float(cur[1]) - float(ref[1])
        dz = float(cur[2]) - float(ref[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _bench_e_dock_proxy(self, rn: str):
        ctx = self.rt[rn]
        delta = getattr(ctx, 'post_contact_delta', None)
        if delta is None:
            delta = self._get_stable_delta(rn)
        if delta is None:
            delta = getattr(ctx, 'delta_latest', None)
        if delta is None:
            return None
        if getattr(ctx, 'last_delta_stamp', None) is not None:
            max_age = max(
                float(getattr(self, 'delta_max_age_sec', 0.0)),
                float(getattr(self, 'load_stable_delta_fresh_timeout_sec', 0.0)),
                0.5,
            )
            if (_now_sec(self) - float(ctx.last_delta_stamp)) > max_age:
                return None
        dx, dy, _ = delta
        return math.hypot(float(dx), float(dy))

    def _bench_e_sup_proxy(self, rn: str):
        ctx = self.rt[rn]
        components = []
        if getattr(ctx, 'slide_vel', None) is not None:
            vx, vy, vz = ctx.slide_vel
            vel_tol = max(1e-6, float(getattr(self, 'load_stable_slide_vel_tol_mmps', 1.0)))
            components.append(math.hypot(float(vx), float(vy)) / vel_tol)
            components.append(abs(float(vz)) / vel_tol)

        force_timeout = float(getattr(self, 'load_stable_force_fresh_timeout_sec', 0.5))
        if self._force_msg_fresh(rn, force_timeout):
            slope_tol = max(1e-6, float(getattr(self, 'load_stable_force_slope_tol_nps', 1.0)))
            components.append(float(self._force_slope_abs_nps(rn)) / slope_tol)

        delta = self._get_stable_delta(rn)
        if delta is None:
            delta = getattr(ctx, 'delta_latest', None)
        if delta is not None and getattr(ctx, 'last_delta_stamp', None) is not None:
            delta_timeout = float(getattr(self, 'load_stable_delta_fresh_timeout_sec', 0.5))
            if (_now_sec(self) - float(ctx.last_delta_stamp)) <= delta_timeout:
                dx, dy, dz = delta
                xy_tol = max(1e-6, float(getattr(self, 'load_stable_xy_tol_m', 0.01)))
                z_tol = max(1e-6, float(getattr(self, 'load_stable_z_tol_m', 0.01)))
                components.append(max(abs(float(dx)) / xy_tol, abs(float(dy)) / xy_tol, abs(float(dz)) / z_tol))

        if not components:
            return None
        return max(float(value) for value in components)

    def _bench_risk_proxy(self, rn: str):
        ctx = self.rt[rn]
        components = []
        formation_tol = max(1e-6, float(getattr(self, 'transport_max_center_error_m', 0.20)))
        if float(getattr(ctx, 'formation_error_m', 0.0)) > 0.0:
            components.append(float(ctx.formation_error_m) / formation_tol)

        dock_proxy = self._bench_e_dock_proxy(rn)
        if dock_proxy is not None:
            dock_tol = max(1e-6, float(getattr(self, 'fine_xy_tol_m', 0.01)))
            components.append(float(dock_proxy) / dock_tol)

        sup_proxy = self._bench_e_sup_proxy(rn)
        if sup_proxy is not None:
            components.append(float(sup_proxy))

        if bool(getattr(ctx, 'transport_failed', False)):
            components.append(1.5)
        if bool(getattr(ctx, 'faulted', False)):
            components.append(2.0)
        if self._bench_freeze_flag(rn):
            components.append(1.0)

        if not components:
            return 0.0
        return max(float(value) for value in components)

    def _bench_outcome_one(self, rn: str) -> str:
        ctx = self.rt[rn]
        if bool(getattr(ctx, 'finished', False)) or self.state == 'DONE':
            return 'success'
        if bool(getattr(ctx, 'faulted', False)) or bool(getattr(ctx, 'transport_failed', False)):
            return 'unsafe_fail'
        if self.state == 'ABORT':
            if bool(getattr(self, 'emergency', False)):
                return 'safe_abort'
            if getattr(ctx, 'group_stop_reason', '') or getattr(ctx, 'fault_reason', ''):
                return 'unsafe_fail'
            return 'safe_abort'
        return 'unknown'

    def _bench_fleet_outcome(self) -> str:
        outcomes = [self._bench_outcome_one(rn) for rn in self.robots]
        if outcomes and all(outcome == 'success' for outcome in outcomes):
            return 'success'
        if 'unsafe_fail' in outcomes:
            return 'unsafe_fail'
        if self.state == 'ABORT' or 'safe_abort' in outcomes:
            return 'safe_abort'
        return 'unknown'

    def _bench_emit_runtime_snapshot(self):
        phase_m = self.state.lower()
        for rn in self.robots:
            ctx = self.rt[rn]
            alpha = self._bench_alpha(rn)
            freeze_flag = int(self._bench_freeze_flag(rn))
            v_base = self._bench_v_base(rn)
            e_slide = self._bench_e_slide(rn)
            e_sup_proxy = self._bench_e_sup_proxy(rn)
            e_dock_proxy = self._bench_e_dock_proxy(rn)
            risk_proxy = self._bench_risk_proxy(rn)
            mocap_age = self._bench_mocap_age_sec(rn)
            delta_age = self._bench_delta_age_sec(rn)
            qr_age = self._bench_qr_age_sec(rn)
            force_age = self._bench_force_age_sec(rn)
            slide_age = self._bench_slide_age_sec(rn)
            delta_eff_ms = self._bench_delta_eff_proxy_ms(rn)
            s_eff = self._bench_s_eff(rn)
            f_eff = self._bench_f_eff(rn)
            load_stable_vel_ok = int(bool(getattr(ctx, 'load_stable_vel_ok', False)))
            load_stable_force_fresh_ok = int(bool(getattr(ctx, 'load_stable_force_fresh_ok', False)))
            load_stable_delta_fresh_ok = int(bool(getattr(ctx, 'load_stable_delta_fresh_ok', False)))
            load_stable_force_slope_ok = int(bool(getattr(ctx, 'load_stable_force_slope_ok', False)))
            load_stable_residual_ok = int(bool(getattr(ctx, 'load_stable_residual_ok', False)))
            self._bench_emit_event(
                event_type='TASK_SNAPSHOT',
                robot_id=rn,
                phase=phase_m,
                reason='periodic_runtime_snapshot',
                note='platform_b_p1 runtime snapshot',
                alpha=f'{alpha:.3f}',
                freeze_flag=freeze_flag,
                phase_m=phase_m,
                local_state=str(getattr(ctx, 'local_state', '')),
                v_base=f'{float(v_base):.6f}' if v_base is not None else None,
                e_slide=f'{float(e_slide):.6f}' if e_slide is not None else None,
                e_sup_proxy=f'{float(e_sup_proxy):.6f}' if e_sup_proxy is not None else None,
                e_dock_proxy=f'{float(e_dock_proxy):.6f}' if e_dock_proxy is not None else None,
                risk_proxy=f'{float(risk_proxy):.6f}',
                mocap_age_sec=f'{float(mocap_age):.6f}' if not (isinstance(mocap_age, float) and (mocap_age != mocap_age)) else None,
                delta_age_sec=f'{float(delta_age):.6f}' if not (isinstance(delta_age, float) and (delta_age != delta_age)) else None,
                qr_age_sec=f'{float(qr_age):.6f}' if not (isinstance(qr_age, float) and (qr_age != qr_age)) else None,
                force_age_sec=f'{float(force_age):.6f}' if not (isinstance(force_age, float) and (force_age != force_age)) else None,
                slide_age_sec=f'{float(slide_age):.6f}' if not (isinstance(slide_age, float) and (slide_age != slide_age)) else None,
                Delta_eff_proxy_ms=f'{float(delta_eff_ms):.6f}' if not (isinstance(delta_eff_ms, float) and (delta_eff_ms != delta_eff_ms)) else None,
                S_eff=f'{float(s_eff):.6f}',
                F_eff=f'{float(f_eff):.6f}',
                load_stable_vel_ok=load_stable_vel_ok,
                load_stable_force_fresh_ok=load_stable_force_fresh_ok,
                load_stable_delta_fresh_ok=load_stable_delta_fresh_ok,
                load_stable_force_slope_ok=load_stable_force_slope_ok,
                load_stable_residual_ok=load_stable_residual_ok,
                outcome=self._bench_outcome_one(rn),
                transport_failed=int(bool(getattr(ctx, 'transport_failed', False))),
                faulted=int(bool(getattr(ctx, 'faulted', False))),
                finished=int(bool(getattr(ctx, 'finished', False))),
                group_stop_reason=str(getattr(ctx, 'group_stop_reason', '')),
            )

    def _maybe_bench_snapshot_locked(self):
        now_ros = _now_sec(self)
        if (now_ros - float(self._bench_last_snapshot_sec)) < self._bench_snapshot_period_sec:
            return
        self._bench_last_snapshot_sec = now_ros
        self._bench_emit_runtime_snapshot()

    def _bench_emit_outcome(self, *, reason: str = '', state_from: str = '', state_to: str = ''):
        phase_m = (state_to or self.state).lower()
        self._bench_emit_event(
            event_type='TASK_OUTCOME',
            robot_id='fleet',
            phase=phase_m,
            reason=reason,
            note='fleet terminal outcome',
            state_from=state_from,
            state_to=state_to,
            outcome=self._bench_fleet_outcome(),
        )
        for rn in self.robots:
            ctx = self.rt[rn]
            self._bench_emit_event(
                event_type='TASK_OUTCOME',
                robot_id=rn,
                phase=phase_m,
                reason=reason or str(getattr(ctx, 'group_stop_reason', '') or getattr(ctx, 'fault_reason', '')),
                note='robot terminal outcome',
                state_from=state_from,
                state_to=state_to,
                outcome=self._bench_outcome_one(rn),
                freeze_flag=int(self._bench_freeze_flag(rn)),
                alpha=f'{self._bench_alpha(rn):.3f}',
                phase_m=phase_m,
            )

    def arm_delta(self, rn: str):
        ctx = self.rt[rn]
        ctx.delta_armed_since = _now_sec(self)
        ctx.delta_latest = None
        ctx.last_delta_stamp = None
        if ctx.te is not None:
            ctx.te.arm(time.time())

    def reached_ok(self, rn: str) -> bool:
        ctx = self.rt[rn]
        return ctx.reached and (_now_sec(self) - ctx.last_goal_epoch) >= self.reach_min_delay_sec

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def loop(self):
        now_ros = _now_sec(self)

        if self.managed_phase_mode and self.state == 'STANDBY':
            self.stop_all()
            self.stop_all_slide_comp()
            return

        if not self.preflight_ok:
            if self._check_workflow_preflight():
                self.preflight_ok = True
                self.get_logger().warn(f'[WORKFLOW={self.workflow}] Preflight checks PASSED')

                requested_start = str(getattr(self, 'start_state', '') or '').lower().strip()
                valid_start_states = self._valid_start_states()

                if requested_start == 'wait_wing':
                    if self._enter_start_state_after_preflight():
                        return
                elif requested_start and requested_start not in valid_start_states:
                    if self._enter_start_state_after_preflight():
                        return

                if not requested_start or requested_start not in valid_start_states:
                    if self.workflow == 'lift':
                        self._start_sync_slide_align()
                        self._set_global_state('SYNC_SLIDE_ALIGN', f'workflow={self.workflow} preflight passed; starting slide alignment')
                    elif self.workflow == 'transport':
                        self._set_global_state('TRANSPORT_PRECHECK', f'workflow={self.workflow} preflight passed; starting transport precheck')
            else:
                return

        if self.state == 'WAIT_WING' and self._start_state_requires_mocap_ready():
            ready, reason = self._start_state_mocap_ready()
            if not ready:
                if (
                    self._start_state_wait_last_log <= 0.0
                    or (now_ros - self._start_state_wait_last_log) > 3.0
                ):
                    self.get_logger().warn(
                        f'[START_STATE] waiting for mocap readiness before entering {self.start_state}: {reason}'
                    )
                    self._start_state_wait_last_log = now_ros
                return

            if self._enter_start_state_after_preflight():
                return

        if self.state == 'WAIT_WING':
            fresh, bad = self._mocap_fresh()

            if self.wing_x is None:
                if now_ros - self._wait_wing_log_time > 5.0:
                    self.get_logger().warn(f'Waiting for Wing Mocap data on topic: {self.wing_mocap_topic}')
                    self._wait_wing_log_time = now_ros
                return

            missing = [rn for rn in self.robots if rn not in self.robot_xy]
            if missing:
                if now_ros - self._wait_wing_log_time > 3.0:
                    self.get_logger().warn(f'Waiting for Mocap of robots: {missing}')
                    self._wait_wing_log_time = now_ros
                return

            if not fresh:
                if now_ros - self._wait_wing_log_time > 3.0:
                    self.get_logger().warn(f'Mocap topics exist but data is stale: {bad}')
                    self._wait_wing_log_time = now_ros
                return

            if self.startup_resume_pending:
                if self.freeze_wing_on_start:
                    self._wing_frozen = True
                ok, reason = self._startup_resume_request()
                if ok:
                    self.get_logger().warn(f'[RESUME_PHASE] startup accepted | {reason}')
                    return
                self.get_logger().error(f'[RESUME_PHASE] startup rejected | {reason}')
                self._managed_enter_standby(f'startup resume rejected: {reason}')
                return

            if self.freeze_wing_on_start:
                self._wing_frozen = True

            self._set_global_state('SYNC_APPROACH_X', 'wing and robot mocap available and fresh')
            self._pending_initial_dispatch = True
            return

        if self.state in ('ABORT', 'DONE'):
            return

        if self._pending_initial_dispatch:
            self._pending_initial_dispatch = False
            self._dispatch_epoch = now_ros
            self._set_global_state('SYNC_APPROACH_X', 'initial cooperative approach dispatch')
            dispatched = self._dispatch_cooperative_leg(
                'APPROACH_X',
                self._cooperative_x_targets(),
                'SYNC_APPROACH_X',
                'dispatch cooperative x leg to common approach line',
                clear_alignment=True,
            )
            if not dispatched:
                self._set_global_state('ABORT', 'cooperative approach x dispatch failed')
                return

        if self.state == 'WAIT_ENTRY_RELEASE':
            self.update_entry_owner()
            self._set_global_state('RUN_ALIGNMENT', 'cooperative approach complete; start sequential final entry')

        if self.state == 'RUN_ALIGNMENT':
            self.update_entry_owner()
            self.apply_collision_gate()

        for rn in self.robots:
            self.step_robot(rn)

        if self.emergency:
            self.stop_all()
            self.stop_all_slide_comp()
            self._set_global_state('ABORT', 'emergency stop asserted')
            return

        if self._has_any_fault() and self.abort_on_any_fault:
            self.stop_all()
            self.stop_all_slide_comp()
            self._set_global_state('ABORT', 'robot fault detected')
            return

        if self.state == 'SYNC_APPROACH_X':
            if self._all_cooperative_leg_complete('APPROACH_X'):
                dispatched = self._dispatch_cooperative_leg(
                    'APPROACH_Y',
                    self._cooperative_wait_targets(),
                    'SYNC_APPROACH_Y',
                    'common x line reached; dispatch cooperative y leg',
                    clear_alignment=False,
                )
                if not dispatched:
                    self._set_global_state('ABORT', 'cooperative approach y dispatch failed')
                    return
                self._set_global_state('SYNC_APPROACH_Y', 'all robots reached common x target; start coordinated y advance')
            return

        if self.state == 'SYNC_APPROACH_Y':
            if self._all_cooperative_leg_complete('APPROACH_Y'):
                self._mark_cooperative_wait_line_ready()
                self._set_global_state('WAIT_ENTRY_RELEASE', 'cooperative waiting line reached; hold and release robots sequentially')
            return

        if self.state == 'RUN_ALIGNMENT':
            if self._all_first_qr_locked():
                if self.workflow == 'approach':
                    self.stop_all()
                    self.stop_all_slide_comp()
                    self._set_global_state('PHASE1_DONE_HOLD', 'workflow=approach complete; all robots QR locked and parked')
                    self._managed_mark_phase_complete('approach', 'PHASE1_DONE_HOLD')
                    return

                self.stop_all()
                self.stop_all_slide_comp()
                if self.managed_phase_mode:
                    self._set_global_state('PHASE1_DONE_HOLD', 'managed phase boundary reached; all robots QR locked and parked')
                    self._managed_mark_phase_complete('approach', 'PHASE1_DONE_HOLD')
                else:
                    self._start_sync_slide_align()
                    self._set_global_state('SYNC_SLIDE_ALIGN', 'all robots achieved first QR lock; start synchronized slide alignment')
            return

        if self.state == 'PHASE1_DONE_HOLD':
            self.stop_all()
            self.stop_all_slide_comp()
            return

        if self.state == 'SYNC_SLIDE_ALIGN':
            if self._all_ready_to_lift():
                self.all_ready_since = now_ros
                self.stop_all()
                self.stop_all_slide_comp()
                self._set_global_state('ALL_READY_HOLD', 'all robots reached READY_TO_LIFT')
                self._managed_mark_phase_complete('slide_align', 'ALL_READY_HOLD')
            return

        if self.state == 'ALL_READY_HOLD':
            if self.managed_phase_mode and self._managed_should_hold_boundary('ALL_READY_HOLD'):
                self.stop_all()
                self.stop_all_slide_comp()
                return

            if not self._all_ready_to_lift():
                self.all_ready_since = 0.0
                self._set_global_state('SYNC_SLIDE_ALIGN', 'ready barrier broken before loaded leveling')
                return

            self.stop_all()
            self.stop_all_slide_comp()

            if (now_ros - self.all_ready_since) >= self.all_ready_hold_sec:
                if self.transport_enable and self.load_level_enable:
                    self._start_level_z_all()
                    self.load_stable_since = 0.0
                    self._set_global_state('SYNC_LEVEL_Z', 'all robots ready; start contact-gated loaded z-plane leveling')
                elif self.transport_enable:
                    self._set_global_state('TRANSPORT_PRECHECK', 'all robots ready; start transport precheck')
                else:
                    for rn in self.robots:
                        self.rt[rn].finished = True
                        self._set_local_state(rn, 'FINISHED', 'transport disabled; mission complete')
                    self.stop_all()
                    self.stop_all_slide_comp()
                    self._set_global_state('DONE', 'transport disabled')
            return

        if self.state == 'SYNC_LEVEL_Z':
            self.stop_all()
            self.stop_all_slide_comp()

            if self._all_level_z_done():
                self.load_stable_since = 0.0
                if self.slide_recenter_enable:
                    self.recenter_all_done_since = 0.0
                    self._start_recenter_all()
                    self._set_global_state('SYNC_RECENTER', 'loaded z-plane leveling done; start slide recenter before transport')
                else:
                    self._set_global_state('LOAD_STABLE_HOLD', 'loaded z-plane leveling done; wait for force/velocity stabilization before transport')
                    self._managed_mark_phase_complete('level_recenter', 'LOAD_STABLE_HOLD')
            return

        if self.state == 'SYNC_RECENTER':
            self.stop_all()
            self.stop_all_slide_comp()

            if self._all_recenter_done():
                if self.recenter_all_done_since <= 0.0:
                    self.recenter_all_done_since = now_ros

                if (now_ros - self.recenter_all_done_since) >= self.slide_recenter_hold_sec:
                    self.load_stable_since = 0.0
                    self._set_global_state('LOAD_STABLE_HOLD', 'slide recenter done; wait for stability before transport')
                    self._managed_mark_phase_complete('level_recenter', 'LOAD_STABLE_HOLD')
            else:
                self.recenter_all_done_since = 0.0
            return

        if self.state == 'LOAD_STABLE_HOLD':
            self.stop_all()
            self.stop_all_slide_comp()

            if self.managed_phase_mode and self._managed_should_hold_boundary('LOAD_STABLE_HOLD'):
                return

            if self._all_load_stable():
                if self.load_stable_since <= 0.0:
                    self.load_stable_since = now_ros

                if (now_ros - self.load_stable_since) >= self.load_stable_hold_sec:
                    if self.workflow == 'lift':
                        if not self.workflow_boundary_reached:
                            self.workflow_boundary_reached = True
                            self.get_logger().warn(f'[WORKFLOW=lift] Load stable hold complete. Workflow boundary reached.')
                        return

                    self._set_global_state('TRANSPORT_PRECHECK', 'loaded transport barrier passed; start transport precheck')
            else:
                self.load_stable_since = 0.0
            return

        if self.state == 'TRANSPORT_PRECHECK':
            self.stop_all()
            self.stop_all_slide_comp()

            ok, reason = self._transport_precheck_ok()
            if not ok:
                if (now_ros - self.transport_precheck_last_log) >= 2.0:
                    self.get_logger().warn(f'[TRANSPORT_PRECHECK] waiting: {reason}')
                    self.transport_precheck_last_log = now_ros
                return

            self._capture_transport_center_ref()
            dispatched = self.dispatch_transport_all(force_refresh=False)
            if not dispatched:
                if (now_ros - self.transport_precheck_last_log) >= 2.0:
                    self.get_logger().warn('[TRANSPORT_PRECHECK] transport dispatch skipped; waiting for valid refs/targets')
                    self.transport_precheck_last_log = now_ros
                return

            self.transport_dispatch_epoch = now_ros
            self._set_global_state('SYNC_TRANSPORT', 'transport precheck passed; transport goals dispatched')
            return

        if self.state == 'SYNC_TRANSPORT':
            ok, reason = self._transport_consistency_ok()
            if not ok:
                for rn in self.robots:
                    ctx = self.rt[rn]
                    if ctx.faulted:
                        continue
                    ctx.transport_failed = True
                    ctx.group_stop_reason = reason
                self.stop_all()
                self.stop_all_slide_comp()
                self._set_global_state('ABORT', f'transport consistency failed: {reason}')
                return

            if (now_ros - self.transport_dispatch_epoch) >= self.transport_goal_refresh_sec:
                if self.dispatch_transport_all(force_refresh=True):
                    self.transport_dispatch_epoch = now_ros

            if self._all_transport_arrived():
                self.stop_all()
                self.stop_all_slide_comp()
                self._set_global_state('TRANSPORT_SETTLE', 'all robots reached transport targets; start settle confirmation')
            return

        if self.state == 'TRANSPORT_SETTLE':
            self.stop_all()
            self.stop_all_slide_comp()

            ok, reason = self._transport_consistency_ok()
            if not ok:
                for rn in self.robots:
                    ctx = self.rt[rn]
                    if ctx.faulted:
                        continue
                    ctx.transport_failed = True
                    ctx.group_stop_reason = reason
                self._set_global_state('ABORT', f'transport settle failed: {reason}')
                return

            if self._all_transport_settled():
                self.stop_all()
                self.stop_all_slide_comp()
                self._set_global_state('DONE', 'all robots settled at transport targets')
                self._managed_mark_phase_complete('transport', 'DONE')
            return

    def _loop_tick_locked(self):
        with self._state_lock:
            self._apply_pending_inputs_locked()
            self.loop()
            self._maybe_bench_snapshot_locked()

    def _slide_tick_locked(self):
        with self._state_lock:
            if self.slide_rt_apply_inputs:
                self._apply_pending_inputs_locked()
            self.slide_rt_loop()

    def _on_slide_rt_error(self, exc: BaseException):
        self.get_logger().error(
            f'[mission_coordinator] slide executor crashed: {exc}\n{traceback.format_exc()}'
        )

    def _on_slide_rt_overrun(self, loop_name: str, tick_sec: float, overrun_sec: float, count: int):
        wall = time.time()
        if wall - self._last_slide_rt_overrun_log_wall < 5.0 and int(count) % 100 != 1:
            return
        self._last_slide_rt_overrun_log_wall = wall
        period_ms = 1000.0 / max(5.0, float(self.slide_rt_hz))
        self.get_logger().warn(
            f'[mission_coordinator] {loop_name} overrun count={int(count)} '
            f'tick={float(tick_sec) * 1000.0:.2f}ms period={period_ms:.2f}ms '
            f'late={float(overrun_sec) * 1000.0:.2f}ms'
        )

    def destroy_node(self):
        if hasattr(self, '_slide_executor'):
            self._slide_executor.stop()
        if hasattr(self, 'mission_runtime_logger'):
            self.mission_runtime_logger.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
