#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from wing_alignment_system.common_rt import FixedRateLoop
from wing_alignment_system.drive_precision_controller import PrecisionController
from wing_alignment_system.drive_types import DriverConfig, Goal2D, Pose2D
from wing_alignment_system.mission_geometry import extract_mocap_yaw_rad


def wrap_angle(a: float) -> float:
    if math.isnan(a) or math.isinf(a):
        return 0.0
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _now_sec(node: Node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


def clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


@dataclass
class PendingGoal:
    x: float
    y: float
    yaw_rad: float
    yaw_deg_in_msg: float
    profile_code: float


@dataclass
class DriverStepAction:
    cmd_v: Optional[float] = None
    cmd_w: Optional[float] = None
    status_value: Optional[bool] = None
    log_level: Optional[str] = None
    log_text: str = ''


class GotoPoseDriver(Node):
    def __init__(self):
        super().__init__('goto_pose_node')

        self.declare_parameter('robot_name', 'tracer1')
        self.robot_name = str(self.get_parameter('robot_name').value)

        self.get_logger().warn(f'>>> GOTO POSE DRIVER STARTING for {self.robot_name} <<<')
        sys.stdout.flush()

        topic_map = {
            'tracer1': '/Rigid17/pose',
            'tracer2': '/Rigid14/pose',
            'tracer3': '/Rigid15/pose'
        }
        default_mocap = topic_map.get(self.robot_name, '/RigidBody1/pose')
        self.declare_parameter('mocap_topic', default_mocap)
        self.mocap_topic = str(self.get_parameter('mocap_topic').value)

        self.mm_to_m = 0.001
        self.declare_parameter('swap_xz', False)
        self.declare_parameter('negate_x', False)
        self.declare_parameter('negate_z', True)
        self.declare_parameter('mocap_yaw_mode', 'legacy_deg_y')
        self.declare_parameter('flip_heading_sign', False)
        self.declare_parameter('heading_deg_bias', 0.0)
        self.declare_parameter('angular_cmd_sign', 1.0)

        self.swap_xz = bool(self.get_parameter('swap_xz').value)
        self.negate_x = bool(self.get_parameter('negate_x').value)
        self.negate_z = bool(self.get_parameter('negate_z').value)
        self.mocap_yaw_mode = str(self.get_parameter('mocap_yaw_mode').value).strip().lower() or 'legacy_deg_y'
        if self.mocap_yaw_mode not in ('legacy_deg_y', 'quaternion', 'quat_auto'):
            self.get_logger().warn(
                f'[{self.robot_name}] invalid mocap_yaw_mode={self.mocap_yaw_mode}, fallback to legacy_deg_y'
            )
            self.mocap_yaw_mode = 'legacy_deg_y'
        self.flip_heading_sign = bool(self.get_parameter('flip_heading_sign').value)
        self.heading_deg_bias = float(self.get_parameter('heading_deg_bias').value)
        angular_cmd_sign = float(self.get_parameter('angular_cmd_sign').value)
        self.angular_cmd_sign = -1.0 if angular_cmd_sign < 0.0 else 1.0

        self.declare_parameter('cmd_topic', '')
        cmd_topic_param = str(self.get_parameter('cmd_topic').value)
        default_cmd_topic = f'/{self.robot_name}/cmd_vel_desired'
        self.cmd_topic = cmd_topic_param if cmd_topic_param else default_cmd_topic
        if not cmd_topic_param:
            self.get_logger().warn(
                f'[{self.robot_name}] cmd_topic unset -> default to scheduler input {self.cmd_topic}'
            )
        self.get_logger().warn(
            f'[{self.robot_name}] mocap_topic={self.mocap_topic}, cmd_topic={self.cmd_topic}, '
            f'swap_xz={self.swap_xz}, negate_x={self.negate_x}, negate_z={self.negate_z}, '
            f'mocap_yaw_mode={self.mocap_yaw_mode}, '
            f'flip_heading_sign={self.flip_heading_sign}, heading_deg_bias={self.heading_deg_bias}, '
            f'angular_cmd_sign={self.angular_cmd_sign:+.1f}'
        )

        # ------------------------------------------------------------------
        # default / final-stable profile
        # ------------------------------------------------------------------
        self.declare_parameter('v_nominal', 0.20)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 0.65)

        self.declare_parameter('pos_tol', 0.05)
        self.declare_parameter('yaw_tol_deg', 3.0)
        self.declare_parameter('k_yaw', 1.5)

        self.declare_parameter('slow_radius', 0.30)
        self.declare_parameter('v_slow_max', 0.08)
        self.declare_parameter('w_slow_max', 0.30)

        self.declare_parameter('v_min_far', 0.04)
        self.declare_parameter('near_rotate_only_deg', 12.0)
        self.declare_parameter('rotate_only_deg', 20.0)

        self.declare_parameter('align_yaw_at_goal', True)
        self.declare_parameter('treat_zero_yaw_as_keep', False)

        self.declare_parameter('goal_reached_latch_sec', 1.0)
        self.declare_parameter('stop_hold_sec', 1.0)
        self.declare_parameter('pose_timeout_sec', 0.25)
        self.declare_parameter('final_stop_hold_sec', 0.35)
        self.declare_parameter('phase2_enter_dist', 0.12)
        self.declare_parameter('near_rotate_only_exit_deg', 6.0)

        self.declare_parameter('dv_max', 0.20)
        self.declare_parameter('dw_max', 0.80)
        self.declare_parameter('stall_cmd_v_th', 0.03)
        self.declare_parameter('stall_cmd_w_th', 0.20)
        self.declare_parameter('stall_pos_eps_m', 0.02)
        self.declare_parameter('stall_yaw_eps_deg', 5.0)
        self.declare_parameter('stall_warn_sec', 1.5)
        self.declare_parameter('stall_fail_stop', True)
        self.declare_parameter('stall_abort_sec', 6.0)

        self.declare_parameter('precision_v_max', 0.06)
        self.declare_parameter('precision_w_max', 0.25)
        self.declare_parameter('precision_k_x', 1.20)
        self.declare_parameter('precision_k_y', 2.20)
        self.declare_parameter('precision_k_yaw', 1.00)
        self.declare_parameter('precision_k_heading', 1.10)
        self.declare_parameter('precision_rotate_only_deg', 9.0)
        self.declare_parameter('precision_deadband_x', 0.006)
        self.declare_parameter('precision_deadband_y', 0.004)
        self.declare_parameter('precision_deadband_yaw_deg', 2.0)
        self.declare_parameter('precision_pos_tol', 0.025)
        self.declare_parameter('precision_yaw_tol_deg', 2.0)
        self.declare_parameter('precision_final_stop_radius', 0.020)
        self.declare_parameter('precision_final_stop_yaw_tol_deg', 2.5)

        self.v_nominal = float(self.get_parameter('v_nominal').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.yaw_tol_deg = float(self.get_parameter('yaw_tol_deg').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)

        self.slow_radius = float(self.get_parameter('slow_radius').value)
        self.v_slow_max = float(self.get_parameter('v_slow_max').value)
        self.w_slow_max = float(self.get_parameter('w_slow_max').value)

        self.v_min_far = float(self.get_parameter('v_min_far').value)
        self.near_rotate_only_deg = float(self.get_parameter('near_rotate_only_deg').value)
        self.rotate_only_deg = float(self.get_parameter('rotate_only_deg').value)

        self.align_yaw_at_goal = bool(self.get_parameter('align_yaw_at_goal').value)
        self.treat_zero_yaw_as_keep = bool(self.get_parameter('treat_zero_yaw_as_keep').value)

        self.goal_reached_latch_sec = float(self.get_parameter('goal_reached_latch_sec').value)
        self.stop_hold_sec = float(self.get_parameter('stop_hold_sec').value)
        self.pose_timeout_sec = float(self.get_parameter('pose_timeout_sec').value)
        self.final_stop_hold_sec = float(self.get_parameter('final_stop_hold_sec').value)
        self.phase2_enter_dist = float(self.get_parameter('phase2_enter_dist').value)
        self.near_rotate_only_exit_deg = float(self.get_parameter('near_rotate_only_exit_deg').value)

        self.dv_max = float(self.get_parameter('dv_max').value)
        self.dw_max = float(self.get_parameter('dw_max').value)
        self.stall_cmd_v_th = float(self.get_parameter('stall_cmd_v_th').value)
        self.stall_cmd_w_th = float(self.get_parameter('stall_cmd_w_th').value)
        self.stall_pos_eps_m = float(self.get_parameter('stall_pos_eps_m').value)
        self.stall_yaw_eps_deg = float(self.get_parameter('stall_yaw_eps_deg').value)
        self.stall_warn_sec = float(self.get_parameter('stall_warn_sec').value)
        self.stall_fail_stop = bool(self.get_parameter('stall_fail_stop').value)
        self.stall_abort_sec = max(0.0, float(self.get_parameter('stall_abort_sec').value))
        self.get_logger().warn(
            f'[{self.robot_name}] stall_fail_stop={self.stall_fail_stop}, stall_abort_sec={self.stall_abort_sec:.1f}s'
        )

        self.precision_v_max = float(self.get_parameter('precision_v_max').value)
        self.precision_w_max = float(self.get_parameter('precision_w_max').value)
        self.precision_k_x = float(self.get_parameter('precision_k_x').value)
        self.precision_k_y = float(self.get_parameter('precision_k_y').value)
        self.precision_k_yaw = float(self.get_parameter('precision_k_yaw').value)
        self.precision_k_heading = float(self.get_parameter('precision_k_heading').value)
        self.precision_rotate_only_deg = float(self.get_parameter('precision_rotate_only_deg').value)
        self.precision_deadband_x = float(self.get_parameter('precision_deadband_x').value)
        self.precision_deadband_y = float(self.get_parameter('precision_deadband_y').value)
        self.precision_deadband_yaw_deg = float(self.get_parameter('precision_deadband_yaw_deg').value)
        self.precision_pos_tol = float(self.get_parameter('precision_pos_tol').value)
        self.precision_yaw_tol_deg = float(self.get_parameter('precision_yaw_tol_deg').value)
        self.precision_final_stop_radius = float(self.get_parameter('precision_final_stop_radius').value)
        self.precision_final_stop_yaw_tol_deg = float(self.get_parameter('precision_final_stop_yaw_tol_deg').value)

        # ------------------------------------------------------------------
        # staging-fast profile
        # ------------------------------------------------------------------
        self.declare_parameter('staging_profile_code', 1.0)
        self.declare_parameter('staging_v_nominal', 0.25)
        self.declare_parameter('staging_v_max', 0.25)
        self.declare_parameter('staging_w_max', 0.65)

        self.declare_parameter('staging_pos_tol', 0.06)
        self.declare_parameter('staging_yaw_tol_deg', 8.0)

        self.declare_parameter('staging_slow_radius', 0.18)
        self.declare_parameter('staging_v_slow_max', 0.12)
        self.declare_parameter('staging_w_slow_max', 0.35)

        self.declare_parameter('staging_v_min_far', 0.06)
        self.declare_parameter('staging_near_rotate_only_deg', 8.0)
        self.declare_parameter('staging_rotate_only_deg', 15.0)

        self.declare_parameter('staging_align_yaw_at_goal', False)

        self.staging_profile_code = float(self.get_parameter('staging_profile_code').value)
        self.staging_v_nominal = float(self.get_parameter('staging_v_nominal').value)
        self.staging_v_max = float(self.get_parameter('staging_v_max').value)
        self.staging_w_max = float(self.get_parameter('staging_w_max').value)

        self.staging_pos_tol = float(self.get_parameter('staging_pos_tol').value)
        self.staging_yaw_tol_deg = float(self.get_parameter('staging_yaw_tol_deg').value)

        self.staging_slow_radius = float(self.get_parameter('staging_slow_radius').value)
        self.staging_v_slow_max = float(self.get_parameter('staging_v_slow_max').value)
        self.staging_w_slow_max = float(self.get_parameter('staging_w_slow_max').value)

        self.staging_v_min_far = float(self.get_parameter('staging_v_min_far').value)
        self.staging_near_rotate_only_deg = float(self.get_parameter('staging_near_rotate_only_deg').value)
        self.staging_rotate_only_deg = float(self.get_parameter('staging_rotate_only_deg').value)

        self.staging_align_yaw_at_goal = bool(self.get_parameter('staging_align_yaw_at_goal').value)

        # ------------------------------------------------------------------
        # transport-low-speed profile
        # ------------------------------------------------------------------
        self.declare_parameter('transport_profile_code', 2.0)
        self.declare_parameter('transport_v_nominal', 0.10)
        self.declare_parameter('transport_v_max', 0.12)
        self.declare_parameter('transport_w_max', 0.30)

        self.declare_parameter('transport_pos_tol', 0.06)
        self.declare_parameter('transport_yaw_tol_deg', 4.0)

        self.declare_parameter('transport_slow_radius', 0.40)
        self.declare_parameter('transport_v_slow_max', 0.05)
        self.declare_parameter('transport_w_slow_max', 0.18)

        self.declare_parameter('transport_v_min_far', 0.03)
        self.declare_parameter('transport_near_rotate_only_deg', 10.0)
        self.declare_parameter('transport_rotate_only_deg', 18.0)

        self.declare_parameter('transport_align_yaw_at_goal', True)

        self.transport_profile_code = float(self.get_parameter('transport_profile_code').value)
        self.transport_v_nominal = float(self.get_parameter('transport_v_nominal').value)
        self.transport_v_max = float(self.get_parameter('transport_v_max').value)
        self.transport_w_max = float(self.get_parameter('transport_w_max').value)

        self.transport_pos_tol = float(self.get_parameter('transport_pos_tol').value)
        self.transport_yaw_tol_deg = float(self.get_parameter('transport_yaw_tol_deg').value)

        self.transport_slow_radius = float(self.get_parameter('transport_slow_radius').value)
        self.transport_v_slow_max = float(self.get_parameter('transport_v_slow_max').value)
        self.transport_w_slow_max = float(self.get_parameter('transport_w_slow_max').value)

        self.transport_v_min_far = float(self.get_parameter('transport_v_min_far').value)
        self.transport_near_rotate_only_deg = float(self.get_parameter('transport_near_rotate_only_deg').value)
        self.transport_rotate_only_deg = float(self.get_parameter('transport_rotate_only_deg').value)

        self.transport_align_yaw_at_goal = bool(self.get_parameter('transport_align_yaw_at_goal').value)

        # ------------------------------------------------------------------
        # runtime states
        # ------------------------------------------------------------------
        self.precision_mode_requested = False
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_pose_rx_sec = 0.0
        self._last_update_sec = None
        self.dt_nominal = 1.0 / 50.0

        self._last_v = 0.0
        self._last_w = 0.0

        self.mission_active = False
        self.phase = 0  # 0: rotate-to-goal, 1: drive, 2: final-yaw, 3: reached

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_profile_code = 0.0
        self.goal_profile_name = 'default'

        self.goal_reached_until = 0.0
        self.final_stop_until = 0.0

        self.e_stop_latched = False
        self.fault_latched = False
        self.stop_hold_until = 0.0
        self.pending_goal: Optional[PendingGoal] = None
        self._fault_reason = ''

        self._reached_log_epoch = 0.0
        self._last_logged_goal = None
        self._last_debug_log_sec = 0.0
        self._stall_ref_pose = None
        self._stall_since = 0.0
        self._last_stall_log_sec = 0.0
        self._near_rotate_latched = False
        self._precision_relaxed_logged = False
        self._precision_reached_logged = False
        self._last_control_overrun_log_wall = 0.0
        self._state_lock = threading.RLock()
        self._precision_controller = PrecisionController(self._build_precision_driver_config())

        # ------------------------------------------------------------------
        # pubs/subs
        # ------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(Bool, f'/{self.robot_name}/goal_reached', 10)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.fault_pub = self.create_publisher(String, f'/{self.robot_name}/driver_fault', qos_reliable_transient)
        qos_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(PoseStamped, self.mocap_topic, self.mocap_cb, qos_best_effort)
        self.create_subscription(Twist, f'/{self.robot_name}/cmd_goal', self.goal_cb, qos_reliable_transient)
        self.create_subscription(Bool, f'/{self.robot_name}/cmd_stop', self.stop_cb, qos_volatile)
        self.create_subscription(Bool, f'/{self.robot_name}/cmd_resume', self.resume_cb, qos_volatile)
        self.create_subscription(Bool, f'/{self.robot_name}/precision_mode', self.precision_cb, 10)

        self._control_loop = FixedRateLoop(
            name=f'{self.robot_name}_goto_pose_rt',
            hz=1.0 / self.dt_nominal,
            tick_fn=self._control_tick,
            on_error=self._on_control_error,
            on_overrun=self._on_control_overrun,
        )
        self._control_loop.start()

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def precision_cb(self, msg: Bool):
        with self._state_lock:
            requested = bool(msg.data)
            if requested != self.precision_mode_requested:
                self._precision_controller.reset()
                self._precision_relaxed_logged = False
                self._precision_reached_logged = False
                if requested:
                    self._reset_near_rotate_latch()
                    self.get_logger().warn(f'[{self.robot_name}] enter final precision mode')
            self.precision_mode_requested = requested

    def _extract_yaw_rad(self, msg: PoseStamped) -> float:
        return extract_mocap_yaw_rad(
            ox=float(msg.pose.orientation.x),
            oy=float(msg.pose.orientation.y),
            oz=float(msg.pose.orientation.z),
            ow=float(msg.pose.orientation.w),
            mode=self.mocap_yaw_mode,
            flip_heading_sign=self.flip_heading_sign,
            heading_deg_bias=self.heading_deg_bias,
        )

    def mocap_cb(self, msg: PoseStamped):
        with self._state_lock:
            self.last_pose_rx_sec = _now_sec(self)

            mx = msg.pose.position.x * self.mm_to_m
            mz = msg.pose.position.z * self.mm_to_m
            xw, yw = (mz, mx) if self.swap_xz else (mx, mz)
            if self.negate_x:
                xw = -xw
            if self.negate_z:
                yw = -yw

            self.x = float(xw)
            self.y = float(yw)
            self.yaw = self._extract_yaw_rad(msg)
            self.have_pose = True

    def stop_cb(self, msg: Bool):
        if not bool(msg.data):
            return

        with self._state_lock:
            now = _now_sec(self)

            if self.mission_active and (self.pending_goal is None):
                self.pending_goal = PendingGoal(
                    x=float(self.goal_x),
                    y=float(self.goal_y),
                    yaw_rad=float(self.goal_yaw),
                    yaw_deg_in_msg=float(math.degrees(self.goal_yaw)),
                    profile_code=float(self.goal_profile_code),
                )

            already = self.e_stop_latched
            self.e_stop_latched = True
            self.stop_hold_until = max(self.stop_hold_until, now + self.stop_hold_sec)

            self.mission_active = False
            self.phase = 3
            self.goal_reached_until = 0.0
            self.final_stop_until = 0.0

            self._last_v = 0.0
            self._last_w = 0.0
            self._precision_controller.reset()

            self.cmd_pub.publish(Twist())
            self._publish_goal_status(now, reached=False)

            if not already:
                self.get_logger().warn(f'[{self.robot_name}] STOP (latched=True)')

    def resume_cb(self, msg: Bool):
        if not bool(msg.data):
            return

        with self._state_lock:
            if self.fault_latched:
                self.get_logger().error(
                    f'[{self.robot_name}] RESUME ignored because driver fault is latched: {self._fault_reason or "unknown"}'
                )
                return

            now = _now_sec(self)
            if (not self.e_stop_latched) and (now >= self.stop_hold_until):
                return

            self.stop_hold_until = 0.0
            self.e_stop_latched = False
            self.get_logger().warn(f'[{self.robot_name}] RESUME')

            if self.pending_goal is not None:
                pg = self.pending_goal
                self.pending_goal = None
                self._start_new_goal(pg.x, pg.y, pg.yaw_rad, pg.yaw_deg_in_msg, pg.profile_code, from_pending=True)

    def goal_cb(self, msg: Twist):
        with self._state_lock:
            if self.fault_latched:
                return

            gx = float(msg.linear.x)
            gy = float(msg.linear.y)
            profile_code = float(msg.linear.z)
            yaw_deg_in = float(msg.angular.z)

            if self.treat_zero_yaw_as_keep and abs(yaw_deg_in) < 1e-6 and self.have_pose:
                goal_yaw = self.yaw
            else:
                goal_yaw = wrap_angle(math.radians(yaw_deg_in))

            if self.e_stop_latched:
                self.pending_goal = PendingGoal(
                    x=gx,
                    y=gy,
                    yaw_rad=goal_yaw,
                    yaw_deg_in_msg=yaw_deg_in,
                    profile_code=profile_code,
                )
                return

            self._start_new_goal(gx, gy, goal_yaw, yaw_deg_in, profile_code, from_pending=False)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _profile_name_from_code(self, code: float) -> str:
        if abs(code - self.staging_profile_code) < 0.25:
            return 'staging'
        if abs(code - self.transport_profile_code) < 0.25:
            return 'transport'
        return 'default'

    def _get_active_profile(self):
        if self.goal_profile_name == 'staging':
            return {
                'name': 'staging',
                'v_nominal': self.staging_v_nominal,
                'v_max': self.staging_v_max,
                'w_max': self.staging_w_max,
                'pos_tol': self.staging_pos_tol,
                'yaw_tol_deg': self.staging_yaw_tol_deg,
                'slow_radius': self.staging_slow_radius,
                'v_slow_max': self.staging_v_slow_max,
                'w_slow_max': self.staging_w_slow_max,
                'v_min_far': self.staging_v_min_far,
                'near_rotate_only_deg': self.staging_near_rotate_only_deg,
                'rotate_only_deg': self.staging_rotate_only_deg,
                'align_yaw_at_goal': self.staging_align_yaw_at_goal,
            }
        if self.goal_profile_name == 'transport':
            return {
                'name': 'transport',
                'v_nominal': self.transport_v_nominal,
                'v_max': self.transport_v_max,
                'w_max': self.transport_w_max,
                'pos_tol': self.transport_pos_tol,
                'yaw_tol_deg': self.transport_yaw_tol_deg,
                'slow_radius': self.transport_slow_radius,
                'v_slow_max': self.transport_v_slow_max,
                'w_slow_max': self.transport_w_slow_max,
                'v_min_far': self.transport_v_min_far,
                'near_rotate_only_deg': self.transport_near_rotate_only_deg,
                'rotate_only_deg': self.transport_rotate_only_deg,
                'align_yaw_at_goal': self.transport_align_yaw_at_goal,
            }
        return {
            'name': 'default',
            'v_nominal': self.v_nominal,
            'v_max': self.v_max,
            'w_max': self.w_max,
            'pos_tol': self.pos_tol,
            'yaw_tol_deg': self.yaw_tol_deg,
            'slow_radius': self.slow_radius,
            'v_slow_max': self.v_slow_max,
            'w_slow_max': self.w_slow_max,
            'v_min_far': self.v_min_far,
            'near_rotate_only_deg': self.near_rotate_only_deg,
            'rotate_only_deg': self.rotate_only_deg,
            'align_yaw_at_goal': self.align_yaw_at_goal,
        }

    def _build_precision_driver_config(self) -> DriverConfig:
        return DriverConfig(
            v_nominal=self.v_nominal,
            v_max=self.v_max,
            w_max=self.w_max,
            Ld=max(self.slow_radius, 0.05),
            Ld_near=max(self.slow_radius * 0.5, 0.03),
            pos_tol=self.precision_pos_tol,
            k_yaw=self.k_yaw,
            yaw_tol_deg=self.precision_yaw_tol_deg,
            slow_radius=self.slow_radius,
            v_slow_max=self.precision_v_max,
            w_slow_max=self.precision_w_max,
            v_min_far=0.0,
            v_min_near=0.0,
            near_rotate_only_deg=self.near_rotate_only_deg,
            coarse_heading_enter_deg=6.0,
            coarse_k_dist=0.0,
            align_yaw_at_goal=True,
            goal_reached_latch_sec=self.goal_reached_latch_sec,
            stop_hold_sec=self.stop_hold_sec,
            pose_timeout_sec=self.pose_timeout_sec,
            final_stop_radius=self.precision_final_stop_radius,
            final_stop_yaw_tol_deg=self.precision_final_stop_yaw_tol_deg,
            final_stop_hold_sec=self.final_stop_hold_sec,
            terminal_v_max=self.precision_v_max,
            terminal_w_max=self.precision_w_max,
            terminal_k_x=self.precision_k_x,
            terminal_k_y=self.precision_k_y,
            terminal_k_yaw=self.precision_k_yaw,
            terminal_k_heading=self.precision_k_heading,
            terminal_rotate_only_deg=self.precision_rotate_only_deg,
            terminal_deadband_x=self.precision_deadband_x,
            terminal_deadband_y=self.precision_deadband_y,
            terminal_deadband_yaw_deg=self.precision_deadband_yaw_deg,
            dv_max=self.dv_max,
            dw_max=self.dw_max,
        )

    def _start_new_goal(self, gx: float, gy: float, gyaw: float, yaw_deg_in: float, profile_code: float, from_pending: bool):
        self.goal_x = gx
        self.goal_y = gy
        self.goal_yaw = gyaw
        self.goal_profile_code = float(profile_code)
        self.goal_profile_name = self._profile_name_from_code(profile_code)

        self.goal_reached_until = 0.0
        self.final_stop_until = 0.0
        self.mission_active = True
        self.phase = 0
        self._near_rotate_latched = False

        self._last_v = 0.0
        self._last_w = 0.0
        self._precision_controller.reset()
        self._precision_relaxed_logged = False
        self._precision_reached_logged = False

        current_goal = (
            round(gx, 3),
            round(gy, 3),
            round(math.degrees(gyaw), 1),
            self.goal_profile_name,
        )
        if self._last_logged_goal != current_goal:
            src = "pending" if from_pending else "new"
            self.get_logger().info(
                f'[{self.robot_name}] {src} goal ({gx:.3f},{gy:.3f}) '
                f'yaw={yaw_deg_in:.1f} profile={self.goal_profile_name}'
            )
            self._last_logged_goal = current_goal

    def _publish_goal_status(self, now: float, reached: bool):
        if reached:
            self.goal_reached_until = max(self.goal_reached_until, now + self.goal_reached_latch_sec)
        self.status_pub.publish(Bool(data=bool(now <= self.goal_reached_until)))

    def _goal_status_value(self, now: float, reached: bool) -> bool:
        if reached:
            self.goal_reached_until = max(self.goal_reached_until, now + self.goal_reached_latch_sec)
        return bool(now <= self.goal_reached_until)

    def _reset_near_rotate_latch(self):
        self._near_rotate_latched = False

    def _precision_goal_reached(
        self,
        dist: float,
        yaw_err_final: float,
        align_yaw_at_goal: bool,
        holding: bool,
        relaxed_active: bool,
    ):
        strict_yaw_ok = (not align_yaw_at_goal) or (abs(yaw_err_final) <= math.radians(self.precision_yaw_tol_deg))
        if holding:
            return True, 'hold_zone'

        if (dist <= self.precision_pos_tol) and strict_yaw_ok:
            return True, 'strict'

        # Keep a small relaxed band for noisy real-world precision docking,
        # but do not allow the previous ~7 cm acceptance window.
        relaxed_dist_tol = max(self.precision_pos_tol * 1.5, self.precision_final_stop_radius * 1.5)
        relaxed_yaw_tol_deg = max(self.precision_yaw_tol_deg, self.precision_final_stop_yaw_tol_deg)
        relaxed_yaw_ok = (not align_yaw_at_goal) or (abs(yaw_err_final) <= math.radians(relaxed_yaw_tol_deg))
        if relaxed_active and (dist <= relaxed_dist_tol) and relaxed_yaw_ok:
            return True, 'relaxed'

        return False, ''

    def _precision_command(self, prof, dist: float, yaw_err_final: float, now: float):
        pose = Pose2D(x=float(self.x), y=float(self.y), yaw=float(self.yaw))
        goal = Goal2D(x=float(self.goal_x), y=float(self.goal_y), yaw=float(self.goal_yaw))
        cmd, holding, relaxed_active = self._precision_controller.compute(
            pose=pose,
            goal=goal,
            now_sec=float(now),
            align_yaw_at_goal=bool(prof['align_yaw_at_goal']),
        )
        reached, reached_reason = self._precision_goal_reached(
            dist=dist,
            yaw_err_final=yaw_err_final,
            align_yaw_at_goal=bool(prof['align_yaw_at_goal']),
            holding=holding,
            relaxed_active=relaxed_active,
        )
        if relaxed_active and not self._precision_relaxed_logged:
            self._precision_relaxed_logged = True
            self.get_logger().warn(
                f'[{self.robot_name}] final precision relaxed reached active '
                f'dist={dist:.3f} yaw_final_err={math.degrees(yaw_err_final):.1f}deg'
            )
        if reached and not self._precision_reached_logged:
            if reached_reason == 'relaxed':
                self.get_logger().warn(
                    f'[{self.robot_name}] final precision reached via relaxed criterion '
                    f'dist={dist:.3f} yaw_final_err={math.degrees(yaw_err_final):.1f}deg'
                )
            elif reached_reason == 'hold_zone':
                self.get_logger().info(
                    f'[{self.robot_name}] final precision reached via hold zone '
                    f'dist={dist:.3f} yaw_final_err={math.degrees(yaw_err_final):.1f}deg'
                )
            else:
                self.get_logger().info(
                    f'[{self.robot_name}] final precision reached '
                    f'dist={dist:.3f} yaw_final_err={math.degrees(yaw_err_final):.1f}deg'
                )
            self._precision_reached_logged = True
        return float(cmd.v), float(cmd.w), bool(reached)

    def _reset_stall_watch(self):
        self._stall_ref_pose = None
        self._stall_since = 0.0

    def _check_stall(self, now: float, cmd_v: float, cmd_w: float):
        moving_cmd = (abs(cmd_v) >= self.stall_cmd_v_th) or (abs(cmd_w) >= self.stall_cmd_w_th)

        if (not moving_cmd) or (not self.have_pose):
            self._reset_stall_watch()
            return None

        if self._stall_ref_pose is None:
            self._stall_ref_pose = (self.x, self.y, self.yaw)
            self._stall_since = now
            return None

        rx, ry, ryaw = self._stall_ref_pose
        dpos = math.hypot(self.x - rx, self.y - ry)
        dyaw_deg = abs(math.degrees(wrap_angle(self.yaw - ryaw)))
        stall_duration = now - self._stall_since

        if dpos <= self.stall_pos_eps_m and dyaw_deg <= self.stall_yaw_eps_deg:
            if stall_duration >= self.stall_warn_sec and (now - self._last_stall_log_sec) > 2.0:
                self._last_stall_log_sec = now
                self.get_logger().error(
                    f'[{self.robot_name}] STALL_SUSPECT cmd=({cmd_v:.3f},{cmd_w:.3f}) '
                    f'but pose nearly unchanged for {stall_duration:.1f}s '
                    f'| dpos={dpos:.3f}m dyaw={dyaw_deg:.1f}deg phase={self.phase} '
                    f'| mocap_topic={self.mocap_topic} cmd_topic={self.cmd_topic}'
                )
            return stall_duration

        self._stall_ref_pose = (self.x, self.y, self.yaw)
        self._stall_since = now
        return None

    def _publish_fault(self, reason: str):
        fault_text = str(reason or '').strip()
        if not fault_text:
            return
        if fault_text == self._fault_reason:
            return
        self._fault_reason = fault_text
        self.fault_pub.publish(String(data=fault_text))

    def _engage_stall_fail_stop(self, now: float, stall_duration: float, cmd_v: float, cmd_w: float) -> str:
        self.e_stop_latched = True
        self.fault_latched = True
        self.stop_hold_until = max(self.stop_hold_until, now + self.stop_hold_sec)
        self.mission_active = False
        self.phase = 3
        self.goal_reached_until = 0.0
        self.final_stop_until = 0.0
        self.pending_goal = None
        self._last_v = 0.0
        self._last_w = 0.0
        self._precision_controller.reset()
        self._reset_near_rotate_latch()
        self._reset_stall_watch()

        reason = (
            f'[{self.robot_name}] STALL_FAIL_STOP engaged after {stall_duration:.1f}s '
            f'cmd=({cmd_v:.3f},{cmd_w:.3f}) goal=({self.goal_x:.3f},{self.goal_y:.3f}) '
            f'yaw_goal={math.degrees(self.goal_yaw):.1f}deg'
        )
        self._publish_fault(reason)
        return reason

    def _rate_limit(self, target_v: float, target_w: float, dt: float):
        safe_v = clamp(target_v, self._last_v - self.dv_max * dt, self._last_v + self.dv_max * dt)
        safe_w = clamp(target_w, self._last_w - self.dw_max * dt, self._last_w + self.dw_max * dt)

        if abs(safe_v) < 1e-4:
            safe_v = 0.0
        if abs(safe_w) < 1e-4:
            safe_w = 0.0

        self._last_v = safe_v
        self._last_w = safe_w
        return safe_v, safe_w

    def _finish_goal(self, now: float) -> str:
        self.mission_active = False
        self.phase = 3
        self.final_stop_until = now + self.final_stop_hold_sec

        self._last_v = 0.0
        self._last_w = 0.0
        self._precision_controller.reset()
        self._reset_near_rotate_latch()
        self._reset_stall_watch()

        if (now - self._reached_log_epoch) > 2.0:
            self._reached_log_epoch = now
            return f'[{self.robot_name}] reached'
        return ''

    def _control_tick(self):
        with self._state_lock:
            action = self.update()

        if action is None:
            return

        if action.cmd_v is not None and action.cmd_w is not None:
            cmd = Twist()
            cmd.linear.x = float(action.cmd_v)
            cmd.angular.z = float(self.angular_cmd_sign * action.cmd_w)
            self.cmd_pub.publish(cmd)

        if action.status_value is not None:
            self.status_pub.publish(Bool(data=bool(action.status_value)))

        if action.log_text:
            if action.log_level == 'warn':
                self.get_logger().warn(action.log_text)
            elif action.log_level == 'error':
                self.get_logger().error(action.log_text)
            else:
                self.get_logger().info(action.log_text)

    def _on_control_error(self, exc: BaseException):
        reason = f'[{self.robot_name}] control loop crashed: {exc}'
        self._publish_fault(reason)
        self.get_logger().error(f'{reason}\n{traceback.format_exc()}')

    def _on_control_overrun(self, loop_name: str, tick_sec: float, overrun_sec: float, count: int):
        wall = time.time()
        if wall - self._last_control_overrun_log_wall < 5.0 and int(count) % 100 != 1:
            return
        self._last_control_overrun_log_wall = wall
        self.get_logger().warn(
            f'[{self.robot_name}] {loop_name} overrun count={int(count)} '
            f'tick={float(tick_sec) * 1000.0:.2f}ms period={self.dt_nominal * 1000.0:.2f}ms '
            f'late={float(overrun_sec) * 1000.0:.2f}ms'
        )

    def destroy_node(self):
        if hasattr(self, '_control_loop'):
            self._control_loop.stop()
        if hasattr(self, 'cmd_pub'):
            self.cmd_pub.publish(Twist())
        return super().destroy_node()

    # ------------------------------------------------------------------
    # main update
    # ------------------------------------------------------------------
    def update(self):
        now = _now_sec(self)

        if self._last_update_sec is None:
            actual_dt = self.dt_nominal
        else:
            actual_dt = clamp(now - self._last_update_sec, 0.005, 0.05)
        self._last_update_sec = now

        if not self.have_pose:
            return None

        if self.e_stop_latched or (now < self.stop_hold_until):
            self._last_v = 0.0
            self._last_w = 0.0
            self.goal_reached_until = 0.0
            self._reset_stall_watch()
            status_value = self._goal_status_value(now, reached=False)
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value)

        if now < self.final_stop_until:
            self._last_v = 0.0
            self._last_w = 0.0
            self._reset_near_rotate_latch()
            self._reset_stall_watch()
            status_value = self._goal_status_value(now, reached=True)
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value)

        if not self.mission_active:
            self._reset_near_rotate_latch()
            self._reset_stall_watch()
            status_value = self._goal_status_value(now, reached=False)
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value)

        if (now - self.last_pose_rx_sec) > self.pose_timeout_sec:
            self._last_v = 0.0
            self._last_w = 0.0
            self.goal_reached_until = 0.0
            self._reset_stall_watch()
            status_value = self._goal_status_value(now, reached=False)

            log_text = ''
            if (now - self._last_debug_log_sec) > 2.0:
                self._last_debug_log_sec = now
                log_text = f'[{self.robot_name}] pose timeout -> force stop'
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value, log_level='warn', log_text=log_text)

        prof = self._get_active_profile()

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        yaw_to_goal = math.atan2(dy, dx) if dist > 1e-9 else self.yaw
        yaw_err_to_goal = wrap_angle(yaw_to_goal - self.yaw)
        yaw_err_final = wrap_angle(self.goal_yaw - self.yaw)

        near = dist <= prof['slow_radius']
        if not near:
            self._reset_near_rotate_latch()

        target_v = 0.0
        target_w = 0.0
        control_mode = f'phase={self.phase}'

        if self.precision_mode_requested:
            self._reset_near_rotate_latch()
            target_v, target_w, precision_reached = self._precision_command(prof, dist, yaw_err_final, now)
            control_mode = 'precision'
            if precision_reached:
                self.phase = 3
        else:
            if self.phase == 0:
                if dist <= prof['pos_tol']:
                    self.phase = 2 if prof['align_yaw_at_goal'] else 3
                elif abs(yaw_err_to_goal) < math.radians(6.0):
                    self.phase = 1
                else:
                    w_cap = prof['w_slow_max'] if near else prof['w_max']
                    target_w = clamp(self.k_yaw * yaw_err_to_goal, -w_cap, w_cap)

            if self.phase == 1:
                # Phase 2 only rotates in place. Entering it early freezes the chassis
                # several centimeters short of the target, so it must stay tied to
                # the actual positional tolerance rather than a larger yaw-prealign radius.
                phase2_enter_dist = prof['pos_tol']

                if dist <= phase2_enter_dist:
                    self._reset_near_rotate_latch()
                    self.phase = 2 if prof['align_yaw_at_goal'] else 3
                else:
                    v_cap = prof['v_slow_max'] if near else prof['v_max']
                    w_cap = prof['w_slow_max'] if near else prof['w_max']

                    rotate_only = False
                    if near:
                        enter_deg = float(prof['near_rotate_only_deg'])
                        exit_deg = min(enter_deg, self.near_rotate_only_exit_deg)
                        if self._near_rotate_latched:
                            self._near_rotate_latched = abs(yaw_err_to_goal) > math.radians(exit_deg)
                        else:
                            self._near_rotate_latched = abs(yaw_err_to_goal) > math.radians(enter_deg)
                        rotate_only = self._near_rotate_latched
                    else:
                        rotate_only = abs(yaw_err_to_goal) > math.radians(prof['rotate_only_deg'])

                    if rotate_only:
                        target_v = 0.0
                        target_w = clamp(self.k_yaw * yaw_err_to_goal, -w_cap, w_cap)
                    else:
                        if near:
                            gain_near = 0.08 if prof['name'] == 'transport' else 0.12
                            target_v = clamp(gain_near * dist, 0.0, v_cap)
                            # Avoid falling into the chassis dead zone in the 10-20 cm band.
                            if prof['name'] == 'transport':
                                if dist > max(prof['pos_tol'] * 2.0, 0.14):
                                    target_v = max(target_v, min(v_cap, 0.03))
                            else:
                                if dist > max(prof['pos_tol'] * 2.0, 0.10):
                                    target_v = max(target_v, min(v_cap, 0.05))
                        else:
                            gain_far = 0.28 if prof['name'] == 'transport' else 0.45
                            target_v = clamp(gain_far * dist, prof['v_min_far'], v_cap)

                        heading_scale = max(0.15, math.cos(min(abs(yaw_err_to_goal), math.pi / 2.0)))
                        target_v *= heading_scale

                        if prof['name'] == 'default' and near and dist < 0.08:
                            target_v = min(target_v, 0.018)
                        elif prof['name'] == 'transport' and near and dist < 0.12:
                            target_v = min(target_v, 0.010)

                        target_w = clamp(self.k_yaw * yaw_err_to_goal, -w_cap, w_cap)

            if self.phase == 2:
                if abs(yaw_err_final) < math.radians(prof['yaw_tol_deg']):
                    self.phase = 3
                else:
                    w_cap = prof['w_slow_max'] if near else prof['w_max']
                    target_v = 0.0
                    target_w = clamp(self.k_yaw * yaw_err_final, -w_cap, w_cap)
            control_mode = f'phase={self.phase}'

        if self.phase == 3:
            log_text = self._finish_goal(now)
            status_value = self._goal_status_value(now, reached=True)
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value, log_level='info', log_text=log_text)

        safe_v, safe_w = self._rate_limit(target_v, target_w, actual_dt)
        status_value = self._goal_status_value(now, reached=False)
        stall_duration = self._check_stall(now, safe_v, safe_w)
        if (
            self.stall_fail_stop and
            self.stall_abort_sec > 0.0 and
            stall_duration is not None and
            stall_duration >= self.stall_abort_sec
        ):
            log_text = self._engage_stall_fail_stop(now, stall_duration, safe_v, safe_w)
            status_value = self._goal_status_value(now, reached=False)
            return DriverStepAction(cmd_v=0.0, cmd_w=0.0, status_value=status_value, log_level='error', log_text=log_text)

        log_text = ''
        if (now - self._last_debug_log_sec) > 2.0:
            self._last_debug_log_sec = now
            log_text = (
                f'[{self.robot_name}] profile={prof["name"]} mode={control_mode} dist={dist:.3f} '
                f'yaw_to_goal_err={math.degrees(yaw_err_to_goal):.1f}deg '
                f'yaw_final_err={math.degrees(yaw_err_final):.1f}deg '
                f'precision={int(self.precision_mode_requested)} '
                f'near_rotate_latched={int(self._near_rotate_latched)} '
                f'cmd=({safe_v:.3f},{safe_w:.3f})'
            )
        return DriverStepAction(
            cmd_v=safe_v,
            cmd_w=safe_w,
            status_value=status_value,
            log_level='info',
            log_text=log_text,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
