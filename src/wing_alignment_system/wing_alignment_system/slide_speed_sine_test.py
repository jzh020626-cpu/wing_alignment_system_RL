#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from base_interfaces_demo.msg import MotorCommand


@dataclass
class SlideStatus:
    x_mm: float = 0.0
    y_mm: float = 0.0
    z_mm: float = 0.0
    vx_mmps: float = 0.0
    vy_mmps: float = 0.0
    vz_mmps: float = 0.0
    stamp_sec: float = 0.0
    valid: bool = False


class SlideSpeedSineTest(Node):
    def __init__(self):
        super().__init__('slide_speed_sine_test_single')

        # ------------------------------------------------------------
        # 基本参数（单机版）
        # ------------------------------------------------------------
        self.declare_parameter('mode', 'sine')  # sine | step_sweep
        self.declare_parameter('robot_name', 'huatai1')
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('run_sec', 20.0)
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('ramp_in_sec', 1.0)
        self.declare_parameter('ramp_out_sec', 1.0)
        self.declare_parameter('use_compensation_topics', True)

        # topic 可覆盖；为空时自动生成
        self.declare_parameter('cmd_topic', '')
        self.declare_parameter('status_topic', '')

        # ------------------------------------------------------------
        # sine 模式参数
        # ------------------------------------------------------------
        self.declare_parameter('sine_freq_hz', 1.0)
        self.declare_parameter('vx_amp_mmps', 0.0)
        self.declare_parameter('vy_amp_mmps', 0.0)
        self.declare_parameter('vz_amp_mmps', 0.0)
        self.declare_parameter('vy_axis_phase_offset_rad', 0.0)
        self.declare_parameter('vz_axis_phase_offset_rad', 0.0)
        self.declare_parameter('log_ref_interval_sec', 0.37)

        # sine 结果落盘
        self.declare_parameter('save_sine_cycle_csv', True)
        self.declare_parameter('save_sine_summary_csv', True)
        self.declare_parameter('sine_cycle_csv_path', '')
        self.declare_parameter('sine_summary_csv_path', '')
        self.declare_parameter('sine_case_tag', '')
        self.declare_parameter('span_ref_x_mm', -1.0)
        self.declare_parameter('span_ref_y_mm', -1.0)
        self.declare_parameter('span_ref_z_mm', -1.0)

        # ------------------------------------------------------------
        # step sweep 模式参数
        # ------------------------------------------------------------
        self.declare_parameter('step_axis', 'x')  # x | y | z
        self.declare_parameter('step_cmds_mmps', [5.0, -5.0, 10.0, -10.0, 15.0, -15.0, 20.0, -20.0])
        self.declare_parameter('step_hold_sec', 2.6)
        self.declare_parameter('step_measure_ignore_head_sec', 0.10)
        self.declare_parameter('step_measure_ignore_tail_sec', 0.10)
        self.declare_parameter('step_return_to_baseline', True)
        self.declare_parameter('step_return_timeout_sec', 8.0)
        self.declare_parameter('step_return_tol_mm', 1.0)
        self.declare_parameter('step_return_kp_mmps_per_mm', 0.8)
        self.declare_parameter('step_return_v_limit_mmps', 10.0)
        self.declare_parameter('step_csv_path', '')

        # ------------------------------------------------------------
        # 安全/诊断参数
        # ------------------------------------------------------------
        self.declare_parameter('sign_check_after_sec', 0.60)
        self.declare_parameter('sign_mismatch_delta_mm', 1.0)

        # 基于 baseline 的相对安全窗
        self.declare_parameter('safe_window_x_mm', 70.0)
        self.declare_parameter('safe_window_y_mm', 35.0)
        self.declare_parameter('safe_window_z_mm', 25.0)

        # 绝对软限位，默认给极大范围，相当于不用
        self.declare_parameter('abs_soft_min_x_mm', -1.0e9)
        self.declare_parameter('abs_soft_max_x_mm',  1.0e9)
        self.declare_parameter('abs_soft_min_y_mm', -1.0e9)
        self.declare_parameter('abs_soft_max_y_mm',  1.0e9)
        self.declare_parameter('abs_soft_min_z_mm', -1.0e9)
        self.declare_parameter('abs_soft_max_z_mm',  1.0e9)

        # 单机各轴使能
        self.declare_parameter('enable_x', True)
        self.declare_parameter('enable_y', True)
        self.declare_parameter('enable_z', True)

        # ------------------------------------------------------------
        # 读取参数
        # ------------------------------------------------------------
        self.mode = str(self.get_parameter('mode').value).strip()
        self.robot_name = str(self.get_parameter('robot_name').value).strip()
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.dt = 1.0 / max(1.0, self.publish_hz)
        self.run_sec = float(self.get_parameter('run_sec').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.ramp_in_sec = float(self.get_parameter('ramp_in_sec').value)
        self.ramp_out_sec = float(self.get_parameter('ramp_out_sec').value)
        self.use_compensation_topics = bool(self.get_parameter('use_compensation_topics').value)

        cmd_topic_param = str(self.get_parameter('cmd_topic').value).strip()
        status_topic_param = str(self.get_parameter('status_topic').value).strip()

        if cmd_topic_param:
            self.cmd_topic = cmd_topic_param
        else:
            if self.use_compensation_topics:
                self.cmd_topic = f'/{self.robot_name}_compensation_ref'
            else:
                self.cmd_topic = f'/{self.robot_name}_speed_ref'

        if status_topic_param:
            self.status_topic = status_topic_param
        else:
            self.status_topic = f'/{self.robot_name}_pos_spe_p_std'

        # sine
        self.sine_freq_hz = float(self.get_parameter('sine_freq_hz').value)
        self.vx_amp_mmps = float(self.get_parameter('vx_amp_mmps').value)
        self.vy_amp_mmps = float(self.get_parameter('vy_amp_mmps').value)
        self.vz_amp_mmps = float(self.get_parameter('vz_amp_mmps').value)
        self.vy_axis_phase_offset_rad = float(self.get_parameter('vy_axis_phase_offset_rad').value)
        self.vz_axis_phase_offset_rad = float(self.get_parameter('vz_axis_phase_offset_rad').value)
        self.log_ref_interval_sec = float(self.get_parameter('log_ref_interval_sec').value)

        self.save_sine_cycle_csv = bool(self.get_parameter('save_sine_cycle_csv').value)
        self.save_sine_summary_csv = bool(self.get_parameter('save_sine_summary_csv').value)
        self.sine_cycle_csv_path = str(self.get_parameter('sine_cycle_csv_path').value).strip()
        self.sine_summary_csv_path = str(self.get_parameter('sine_summary_csv_path').value).strip()
        self.sine_case_tag = str(self.get_parameter('sine_case_tag').value).strip()
        self.span_ref_x_mm = float(self.get_parameter('span_ref_x_mm').value)
        self.span_ref_y_mm = float(self.get_parameter('span_ref_y_mm').value)
        self.span_ref_z_mm = float(self.get_parameter('span_ref_z_mm').value)

        # step
        self.step_axis = str(self.get_parameter('step_axis').value).strip().lower()
        self.step_cmds_mmps = [float(v) for v in list(self.get_parameter('step_cmds_mmps').value)]
        self.step_hold_sec = float(self.get_parameter('step_hold_sec').value)
        self.step_measure_ignore_head_sec = float(self.get_parameter('step_measure_ignore_head_sec').value)
        self.step_measure_ignore_tail_sec = float(self.get_parameter('step_measure_ignore_tail_sec').value)
        self.step_return_to_baseline = bool(self.get_parameter('step_return_to_baseline').value)
        self.step_return_timeout_sec = float(self.get_parameter('step_return_timeout_sec').value)
        self.step_return_tol_mm = float(self.get_parameter('step_return_tol_mm').value)
        self.step_return_kp_mmps_per_mm = float(self.get_parameter('step_return_kp_mmps_per_mm').value)
        self.step_return_v_limit_mmps = float(self.get_parameter('step_return_v_limit_mmps').value)
        self.step_csv_path = str(self.get_parameter('step_csv_path').value).strip()

        # safety
        self.sign_check_after_sec = float(self.get_parameter('sign_check_after_sec').value)
        self.sign_mismatch_delta_mm = float(self.get_parameter('sign_mismatch_delta_mm').value)

        self.safe_window_x_mm = float(self.get_parameter('safe_window_x_mm').value)
        self.safe_window_y_mm = float(self.get_parameter('safe_window_y_mm').value)
        self.safe_window_z_mm = float(self.get_parameter('safe_window_z_mm').value)

        self.abs_soft_min_x_mm = float(self.get_parameter('abs_soft_min_x_mm').value)
        self.abs_soft_max_x_mm = float(self.get_parameter('abs_soft_max_x_mm').value)
        self.abs_soft_min_y_mm = float(self.get_parameter('abs_soft_min_y_mm').value)
        self.abs_soft_max_y_mm = float(self.get_parameter('abs_soft_max_y_mm').value)
        self.abs_soft_min_z_mm = float(self.get_parameter('abs_soft_min_z_mm').value)
        self.abs_soft_max_z_mm = float(self.get_parameter('abs_soft_max_z_mm').value)

        self.enable_x = bool(self.get_parameter('enable_x').value)
        self.enable_y = bool(self.get_parameter('enable_y').value)
        self.enable_z = bool(self.get_parameter('enable_z').value)

        # ------------------------------------------------------------
        # topic / status / publisher
        # ------------------------------------------------------------
        self.status = SlideStatus()

        self.cmd_pub = self.create_publisher(MotorCommand, self.cmd_topic, 10)

        self.status_sub = self.create_subscription(
            Float64MultiArray,
            self.status_topic,
            self._status_cb,
            10
        )

        # ------------------------------------------------------------
        # 状态机公共变量
        # ------------------------------------------------------------
        self.start_wall_sec = self._now()
        self.baseline_captured = False
        self.baseline_pos: Optional[Tuple[float, float, float]] = None

        self.last_ref_log_sec = -1.0e9
        self.shutdown_requested = False
        self.stop_sent = False
        self.shutdown_timer = None

        # sine mode runtime
        self.sine_cycle_samples: Dict[int, List[Tuple[float, float, float]]] = {}
        self.last_completed_cycle_idx = -1
        self.sine_cycle_rows: List[Dict[str, object]] = []
        self.sine_summary_written = False
        self.sine_test_start_sec: Optional[float] = None
        self.sine_test_end_sec: Optional[float] = None

        # step mode runtime
        self.step_axis_enabled = self._axis_enabled(self.step_axis)
        self.step_state = 'WAIT_BASELINE'
        self.step_segment_idx = -1
        self.step_segment_cmd_mmps = 0.0
        self.step_segment_start_sec = 0.0
        self.step_segment_samples: List[Dict[str, float]] = []
        self.step_segment_pos0: Optional[Tuple[float, float, float]] = None
        self.step_results_rows: List[Dict[str, object]] = []
        self.step_return_start_sec = 0.0
        self.step_abort_reason = ''

        # ------------------------------------------------------------
        # 启动日志
        # ------------------------------------------------------------
        self.get_logger().warn('>>> SLIDE SPEED TEST SINGLE START <<<')
        self.get_logger().warn(f'mode={self.mode}')
        self.get_logger().warn(f'robot_name={self.robot_name}')
        self.get_logger().warn(f'publish_hz={self.publish_hz:.3f} Hz, dt={self.dt:.4f} s')
        self.get_logger().warn(f'run_sec={self.run_sec:.2f}, settle_sec={self.settle_sec:.2f}')
        self.get_logger().warn(f'ramp_in_sec={self.ramp_in_sec:.2f}, ramp_out_sec={self.ramp_out_sec:.2f}')
        self.get_logger().warn(f'cmd_topic={self.cmd_topic}')
        self.get_logger().warn(f'status_topic={self.status_topic}')
        self.get_logger().warn(
            f'enabled_axes: x={self.enable_x}, y={self.enable_y}, z={self.enable_z}'
        )

        if self.mode == 'sine':
            cycle_sec = 1.0 / max(1.0e-6, self.sine_freq_hz)
            self.get_logger().warn(f'sine_freq_hz={self.sine_freq_hz:.4f} Hz, cycle_sec={cycle_sec:.4f} s')
            self.get_logger().warn(
                f'amp(mm/s): vx={self.vx_amp_mmps:.3f}, vy={self.vy_amp_mmps:.3f}, vz={self.vz_amp_mmps:.3f}'
            )
            self.get_logger().warn(
                f'axis_phase(rad): vy_offset={self.vy_axis_phase_offset_rad:.3f}, '
                f'vz_offset={self.vz_axis_phase_offset_rad:.3f}'
            )
            self.get_logger().warn(f'log_ref_interval_sec={self.log_ref_interval_sec:.3f}')
            self.get_logger().warn(f'[SINE_NOTE] PRIMARY judgement should use span_vs_theory and drift.')
            self.get_logger().warn(
                f'save_sine_cycle_csv={self.save_sine_cycle_csv}, '
                f'save_sine_summary_csv={self.save_sine_summary_csv}'
            )
            self.get_logger().warn(
                f'sine_cycle_csv_path={self._effective_sine_cycle_csv_path()}'
            )
            self.get_logger().warn(
                f'sine_summary_csv_path={self._effective_sine_summary_csv_path()}'
            )
            self.get_logger().warn(
                f'sine_case_tag={self.sine_case_tag if self.sine_case_tag else "(empty)"}'
            )
            self.get_logger().warn(
                f'span_ref(mm): x={self.span_ref_x_mm:.3f}, '
                f'y={self.span_ref_y_mm:.3f}, z={self.span_ref_z_mm:.3f}'
            )

        elif self.mode == 'step_sweep':
            csv_path = self._effective_step_csv_path()
            self.get_logger().warn(f'step_axis={self.step_axis}')
            self.get_logger().warn(f'step_cmds_mmps={self.step_cmds_mmps}')
            self.get_logger().warn(f'step_hold_sec={self.step_hold_sec:.3f}')
            self.get_logger().warn(
                f'measure_ignore_head_sec={self.step_measure_ignore_head_sec:.3f}, '
                f'measure_ignore_tail_sec={self.step_measure_ignore_tail_sec:.3f}'
            )
            self.get_logger().warn(f'step_return_to_baseline={self.step_return_to_baseline}')
            self.get_logger().warn(f'step_axis_enabled={self.step_axis_enabled}')
            self.get_logger().warn(f'step_csv_path={csv_path}')
            self.get_logger().warn(
                '[STEP_NOTE] PRIMARY judgement should use mean_speed_from_pos_mmps, '
                'NOT raw feedback speed field alone.'
            )

        else:
            self.get_logger().error(f'未知 mode={self.mode}，只支持 sine / step_sweep')
            self._request_shutdown('INVALID_MODE')
            return

        self.timer = self.create_timer(self.dt, self._on_timer)

    # ------------------------------------------------------------------
    # 基础工具
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _status_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 6:
            return

        self.status.x_mm = float(data[0])
        self.status.y_mm = float(data[1])
        self.status.z_mm = float(data[2])
        self.status.vx_mmps = float(data[3])
        self.status.vy_mmps = float(data[4])
        self.status.vz_mmps = float(data[5])
        self.status.stamp_sec = self._now()
        self.status.valid = True

    def _status_ready(self) -> bool:
        return self.status.valid

    def _capture_baseline_if_needed(self) -> bool:
        if self.baseline_captured:
            return True

        elapsed = self._now() - self.start_wall_sec
        if elapsed < self.settle_sec:
            if int(elapsed * 2.0) != int((elapsed - self.dt) * 2.0):
                self.get_logger().info(
                    f'[SETTLE] t={elapsed:.2f}s/{self.settle_sec:.2f}s | '
                    f'status_ready={self._status_ready()}'
                )
            return False

        if not self._status_ready():
            self.get_logger().warn('[BASELINE] waiting for status ready')
            return False

        self.baseline_pos = (self.status.x_mm, self.status.y_mm, self.status.z_mm)
        self.baseline_captured = True
        self.get_logger().warn(
            f'[BASELINE] captured | {self.robot_name}=('
            f'{self.baseline_pos[0]:.3f}, {self.baseline_pos[1]:.3f}, {self.baseline_pos[2]:.3f})'
        )
        return True

    def _axis_enabled(self, axis: str) -> bool:
        if axis == 'x':
            return self.enable_x
        if axis == 'y':
            return self.enable_y
        if axis == 'z':
            return self.enable_z
        return False

    def _pub_speed(self, vx: float, vy: float, vz: float):
        msg = MotorCommand()
        msg.command_type = 'speed'
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        self.cmd_pub.publish(msg)

    def _pub_stop(self):
        msg = MotorCommand()
        msg.command_type = 'stop'
        self.cmd_pub.publish(msg)

    def _stop_all(self, reason: str):
        if self.stop_sent:
            return
        self.stop_sent = True
        self._pub_speed(0.0, 0.0, 0.0)
        self._pub_stop()
        self.get_logger().warn(reason)

    def _request_shutdown(self, reason: str):
        if self.shutdown_requested:
            return

        # 在发停机前先把结果尽可能 flush 完
        if self.mode == 'sine':
            self._finalize_all_pending_sine_cycles()
            self._flush_sine_cycle_csv()
            self._flush_sine_summary_csv()

        if self.mode == 'step_sweep':
            self._flush_step_csv()

        self.shutdown_requested = True
        self._stop_all(reason)
        if self.shutdown_timer is None:
            self.shutdown_timer = self.create_timer(0.20, self._shutdown_once)

    def _shutdown_once(self):
        try:
            if self.shutdown_timer is not None:
                self.shutdown_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            self.destroy_node()
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # 安全检查
    # ------------------------------------------------------------------
    def _inside_relative_window(self, st: SlideStatus) -> bool:
        if not self.baseline_captured or self.baseline_pos is None:
            return True
        bx, by, bz = self.baseline_pos
        if abs(st.x_mm - bx) > self.safe_window_x_mm:
            return False
        if abs(st.y_mm - by) > self.safe_window_y_mm:
            return False
        if abs(st.z_mm - bz) > self.safe_window_z_mm:
            return False
        return True

    def _inside_absolute_limits(self, st: SlideStatus) -> bool:
        if not (self.abs_soft_min_x_mm <= st.x_mm <= self.abs_soft_max_x_mm):
            return False
        if not (self.abs_soft_min_y_mm <= st.y_mm <= self.abs_soft_max_y_mm):
            return False
        if not (self.abs_soft_min_z_mm <= st.z_mm <= self.abs_soft_max_z_mm):
            return False
        return True

    def _safety_check(self) -> bool:
        if not self.baseline_captured:
            return True

        st = self.status
        if not st.valid:
            return True

        if not self._inside_relative_window(st):
            self.get_logger().error(
                f'[SAFETY] exceeded relative safe window: '
                f'pos=({st.x_mm:.3f},{st.y_mm:.3f},{st.z_mm:.3f}) '
                f'baseline=({self.baseline_pos[0]:.3f},{self.baseline_pos[1]:.3f},{self.baseline_pos[2]:.3f})'
            )
            return False

        if not self._inside_absolute_limits(st):
            self.get_logger().error(
                f'[SAFETY] exceeded absolute soft limit: '
                f'pos=({st.x_mm:.3f},{st.y_mm:.3f},{st.z_mm:.3f})'
            )
            return False

        return True

    # ------------------------------------------------------------------
    # sine 模式：辅助工具
    # ------------------------------------------------------------------
    def _effective_sine_cycle_csv_path(self) -> str:
        if self.sine_cycle_csv_path:
            return self.sine_cycle_csv_path
        return f'/tmp/{self.robot_name}_sine_cycles.csv'

    def _effective_sine_summary_csv_path(self) -> str:
        if self.sine_summary_csv_path:
            return self.sine_summary_csv_path
        return f'/tmp/{self.robot_name}_sine_summary.csv'

    @staticmethod
    def _safe_div(num: float, den: float) -> float:
        if abs(den) < 1.0e-12:
            return float('nan')
        return num / den

    def _ideal_span_from_speed_sine(self, amp_mmps: float) -> float:
        if self.sine_freq_hz <= 1.0e-12:
            return float('nan')
        # 对 v(t)=A*sin(2*pi*f*t)，位移峰峰值 span = A / (pi*f)
        return abs(amp_mmps) / (math.pi * self.sine_freq_hz)

    def _flush_sine_cycle_csv(self):
        if not self.save_sine_cycle_csv:
            return

        csv_path = self._effective_sine_cycle_csv_path()
        csv_dir = os.path.dirname(csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        fieldnames = [
            'case_tag',
            'robot',
            'freq_hz',
            'vx_amp_mmps',
            'vy_amp_mmps',
            'vz_amp_mmps',
            'publish_hz',
            'run_sec',
            'settle_sec',
            'cycle_idx',
            'n',
            'cycle_start_t_run_sec',
            'cycle_end_t_run_sec',
            'baseline_x_mm',
            'baseline_y_mm',
            'baseline_z_mm',
            'drift_x_mm',
            'drift_y_mm',
            'drift_z_mm',
            'span_x_mm',
            'span_y_mm',
            'span_z_mm',
            'ideal_span_x_mm',
            'ideal_span_y_mm',
            'ideal_span_z_mm',
            'gain_vs_ideal_x',
            'gain_vs_ideal_y',
            'gain_vs_ideal_z',
            'span_ref_x_mm',
            'span_ref_y_mm',
            'span_ref_z_mm',
            'gain_vs_ref_x',
            'gain_vs_ref_y',
            'gain_vs_ref_z',
        ]

        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.sine_cycle_rows:
                writer.writerow(row)

        self.get_logger().warn(f'[SINE_CSV] cycle rows saved to {csv_path}')

    def _flush_sine_summary_csv(self):
        if not self.save_sine_summary_csv:
            return
        if self.sine_summary_written:
            return
        if len(self.sine_cycle_rows) <= 0:
            return

        csv_path = self._effective_sine_summary_csv_path()
        csv_dir = os.path.dirname(csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        xs = [row['drift_x_mm'] for row in self.sine_cycle_rows]
        ys = [row['drift_y_mm'] for row in self.sine_cycle_rows]
        zs = [row['drift_z_mm'] for row in self.sine_cycle_rows]

        span_xs = [row['span_x_mm'] for row in self.sine_cycle_rows]
        span_ys = [row['span_y_mm'] for row in self.sine_cycle_rows]
        span_zs = [row['span_z_mm'] for row in self.sine_cycle_rows]

        gain_ideal_xs = [row['gain_vs_ideal_x'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ideal_x'])]
        gain_ideal_ys = [row['gain_vs_ideal_y'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ideal_y'])]
        gain_ideal_zs = [row['gain_vs_ideal_z'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ideal_z'])]

        gain_ref_xs = [row['gain_vs_ref_x'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ref_x'])]
        gain_ref_ys = [row['gain_vs_ref_y'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ref_y'])]
        gain_ref_zs = [row['gain_vs_ref_z'] for row in self.sine_cycle_rows if math.isfinite(row['gain_vs_ref_z'])]

        def mean_or_nan(arr: List[float]) -> float:
            if len(arr) <= 0:
                return float('nan')
            return sum(arr) / len(arr)

        summary_row = {
            'case_tag': self.sine_case_tag,
            'robot': self.robot_name,
            'freq_hz': self.sine_freq_hz,
            'vx_amp_mmps': self.vx_amp_mmps,
            'vy_amp_mmps': self.vy_amp_mmps,
            'vz_amp_mmps': self.vz_amp_mmps,
            'publish_hz': self.publish_hz,
            'run_sec': self.run_sec,
            'settle_sec': self.settle_sec,
            'cycle_count': len(self.sine_cycle_rows),
            'mean_drift_x_mm': mean_or_nan(xs),
            'mean_drift_y_mm': mean_or_nan(ys),
            'mean_drift_z_mm': mean_or_nan(zs),
            'max_abs_drift_x_mm': max(abs(v) for v in xs) if xs else float('nan'),
            'max_abs_drift_y_mm': max(abs(v) for v in ys) if ys else float('nan'),
            'max_abs_drift_z_mm': max(abs(v) for v in zs) if zs else float('nan'),
            'mean_span_x_mm': mean_or_nan(span_xs),
            'mean_span_y_mm': mean_or_nan(span_ys),
            'mean_span_z_mm': mean_or_nan(span_zs),
            'mean_gain_vs_ideal_x': mean_or_nan(gain_ideal_xs),
            'mean_gain_vs_ideal_y': mean_or_nan(gain_ideal_ys),
            'mean_gain_vs_ideal_z': mean_or_nan(gain_ideal_zs),
            'mean_gain_vs_ref_x': mean_or_nan(gain_ref_xs),
            'mean_gain_vs_ref_y': mean_or_nan(gain_ref_ys),
            'mean_gain_vs_ref_z': mean_or_nan(gain_ref_zs),
            'span_ref_x_mm': self.span_ref_x_mm,
            'span_ref_y_mm': self.span_ref_y_mm,
            'span_ref_z_mm': self.span_ref_z_mm,
        }

        fieldnames = list(summary_row.keys())
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerow(summary_row)

        self.sine_summary_written = True
        self.get_logger().warn(f'[SINE_CSV] summary row saved to {csv_path}')

        self.get_logger().warn(
            f'[SINE_SUMMARY] cycles={summary_row["cycle_count"]} '
            f'| mean_drift_x={summary_row["mean_drift_x_mm"]:.3f} mm '
            f'| max_abs_drift_x={summary_row["max_abs_drift_x_mm"]:.3f} mm '
            f'| mean_span_x={summary_row["mean_span_x_mm"]:.3f} mm '
            f'| mean_gain_vs_ideal_x={summary_row["mean_gain_vs_ideal_x"]:.3f} '
            f'| mean_gain_vs_ref_x={summary_row["mean_gain_vs_ref_x"]:.3f}'
        )

    def _finalize_all_pending_sine_cycles(self):
        if len(self.sine_cycle_samples) <= 0:
            return

        pending_keys = sorted(list(self.sine_cycle_samples.keys()))
        for k in pending_keys:
            if k > self.last_completed_cycle_idx:
                self._finalize_sine_cycle(k)
                self.last_completed_cycle_idx = k

        self.sine_cycle_samples.clear()

    # ------------------------------------------------------------------
    # sine 模式
    # ------------------------------------------------------------------
    def _sine_envelope(self, t_run: float) -> float:
        if t_run < 0.0:
            return 0.0

        s_in = 1.0
        if self.ramp_in_sec > 1.0e-6:
            s_in = min(1.0, max(0.0, t_run / self.ramp_in_sec))

        t_remain = self.run_sec - t_run
        s_out = 1.0
        if self.ramp_out_sec > 1.0e-6:
            s_out = min(1.0, max(0.0, t_remain / self.ramp_out_sec))

        return min(s_in, s_out)

    def _finalize_sine_cycle(self, cycle_idx: int):
        if cycle_idx not in self.sine_cycle_samples:
            return

        samples = self.sine_cycle_samples[cycle_idx]
        if len(samples) < 2 or self.baseline_pos is None:
            return

        bx, by, bz = self.baseline_pos
        x_vals = [p[0] for p in samples]
        y_vals = [p[1] for p in samples]
        z_vals = [p[2] for p in samples]

        drift_x = sum(x_vals) / len(x_vals) - bx
        drift_y = sum(y_vals) / len(y_vals) - by
        drift_z = sum(z_vals) / len(z_vals) - bz

        span_x = max(x_vals) - min(x_vals)
        span_y = max(y_vals) - min(y_vals)
        span_z = max(z_vals) - min(z_vals)

        cycle_sec = 1.0 / max(1.0e-6, self.sine_freq_hz)
        cycle_start_t_run = cycle_idx * cycle_sec
        cycle_end_t_run = (cycle_idx + 1) * cycle_sec

        ideal_span_x = self._ideal_span_from_speed_sine(self.vx_amp_mmps) if self._axis_enabled('x') else float('nan')
        ideal_span_y = self._ideal_span_from_speed_sine(self.vy_amp_mmps) if self._axis_enabled('y') else float('nan')
        ideal_span_z = self._ideal_span_from_speed_sine(self.vz_amp_mmps) if self._axis_enabled('z') else float('nan')

        gain_vs_ideal_x = self._safe_div(span_x, ideal_span_x) if math.isfinite(ideal_span_x) else float('nan')
        gain_vs_ideal_y = self._safe_div(span_y, ideal_span_y) if math.isfinite(ideal_span_y) else float('nan')
        gain_vs_ideal_z = self._safe_div(span_z, ideal_span_z) if math.isfinite(ideal_span_z) else float('nan')

        gain_vs_ref_x = self._safe_div(span_x, self.span_ref_x_mm) if self.span_ref_x_mm > 0.0 else float('nan')
        gain_vs_ref_y = self._safe_div(span_y, self.span_ref_y_mm) if self.span_ref_y_mm > 0.0 else float('nan')
        gain_vs_ref_z = self._safe_div(span_z, self.span_ref_z_mm) if self.span_ref_z_mm > 0.0 else float('nan')

        row = {
            'case_tag': self.sine_case_tag,
            'robot': self.robot_name,
            'freq_hz': self.sine_freq_hz,
            'vx_amp_mmps': self.vx_amp_mmps,
            'vy_amp_mmps': self.vy_amp_mmps,
            'vz_amp_mmps': self.vz_amp_mmps,
            'publish_hz': self.publish_hz,
            'run_sec': self.run_sec,
            'settle_sec': self.settle_sec,
            'cycle_idx': cycle_idx,
            'n': len(samples),
            'cycle_start_t_run_sec': cycle_start_t_run,
            'cycle_end_t_run_sec': cycle_end_t_run,
            'baseline_x_mm': bx,
            'baseline_y_mm': by,
            'baseline_z_mm': bz,
            'drift_x_mm': drift_x,
            'drift_y_mm': drift_y,
            'drift_z_mm': drift_z,
            'span_x_mm': span_x,
            'span_y_mm': span_y,
            'span_z_mm': span_z,
            'ideal_span_x_mm': ideal_span_x,
            'ideal_span_y_mm': ideal_span_y,
            'ideal_span_z_mm': ideal_span_z,
            'gain_vs_ideal_x': gain_vs_ideal_x,
            'gain_vs_ideal_y': gain_vs_ideal_y,
            'gain_vs_ideal_z': gain_vs_ideal_z,
            'span_ref_x_mm': self.span_ref_x_mm,
            'span_ref_y_mm': self.span_ref_y_mm,
            'span_ref_z_mm': self.span_ref_z_mm,
            'gain_vs_ref_x': gain_vs_ref_x,
            'gain_vs_ref_y': gain_vs_ref_y,
            'gain_vs_ref_z': gain_vs_ref_z,
        }
        self.sine_cycle_rows.append(row)

        self.get_logger().warn(
            f'[CYCLE {cycle_idx}] {self.robot_name}: '
            f'drift=({drift_x:.3f},{drift_y:.3f},{drift_z:.3f}) '
            f'span=({span_x:.3f},{span_y:.3f},{span_z:.3f}) '
            f'| gain_vs_ideal_x={gain_vs_ideal_x:.3f} '
            f'| gain_vs_ref_x={gain_vs_ref_x:.3f}'
        )

    def _collect_sine_cycle_samples(self, t_run: float):
        if not self.baseline_captured:
            return

        cycle_sec = 1.0 / max(1.0e-6, self.sine_freq_hz)
        cycle_idx = int(max(0.0, t_run) / cycle_sec)

        if cycle_idx not in self.sine_cycle_samples:
            self.sine_cycle_samples[cycle_idx] = []

        if self.status.valid:
            self.sine_cycle_samples[cycle_idx].append(
                (self.status.x_mm, self.status.y_mm, self.status.z_mm)
            )

        while self.last_completed_cycle_idx + 1 <= cycle_idx - 1:
            finished_idx = self.last_completed_cycle_idx + 1
            self._finalize_sine_cycle(finished_idx)
            self.last_completed_cycle_idx = finished_idx

            old_keys = [k for k in self.sine_cycle_samples.keys() if k < finished_idx]
            for k in old_keys:
                self.sine_cycle_samples.pop(k, None)

    def _run_sine_once(self):
        now = self._now()
        elapsed = now - self.start_wall_sec

        if not self._capture_baseline_if_needed():
            self._pub_speed(0.0, 0.0, 0.0)
            return

        if self.sine_test_start_sec is None:
            self.sine_test_start_sec = now

        if not self._safety_check():
            self._request_shutdown('SINE_ABORT_BY_SAFETY')
            return

        t_run = elapsed - self.settle_sec
        if t_run >= self.run_sec:
            self.sine_test_end_sec = now
            self._request_shutdown('速度正弦测试结束，已向滑台发送 0 速度停机命令。')
            return

        env = self._sine_envelope(t_run)
        omega = 2.0 * math.pi * self.sine_freq_hz

        vx = 0.0
        vy = 0.0
        vz = 0.0

        if self._axis_enabled('x'):
            vx = env * self.vx_amp_mmps * math.sin(omega * t_run)
        if self._axis_enabled('y'):
            vy = env * self.vy_amp_mmps * math.sin(
                omega * t_run + self.vy_axis_phase_offset_rad
            )
        if self._axis_enabled('z'):
            vz = env * self.vz_amp_mmps * math.sin(
                omega * t_run + self.vz_axis_phase_offset_rad
            )

        self._pub_speed(vx, vy, vz)

        if int(t_run * self.publish_hz) % max(1, int(self.publish_hz * 2.0)) == 0:
            self.get_logger().info(f'[PUB_STAT] approx={self.publish_hz:.2f} Hz, dt={self.dt:.4f}s')

        if (now - self.last_ref_log_sec) >= self.log_ref_interval_sec:
            self.last_ref_log_sec = now
            self.get_logger().info(
                f'[REF] t={t_run:.2f}s | {self.robot_name}: ({vx:.2f}, {vy:.2f}, {vz:.2f}) mm/s'
            )

        self._collect_sine_cycle_samples(t_run)

    # ------------------------------------------------------------------
    # step sweep 模式
    # ------------------------------------------------------------------
    def _effective_step_csv_path(self) -> str:
        if self.step_csv_path:
            return self.step_csv_path
        return f'/tmp/{self.robot_name}_slide_step_{self.step_axis}.csv'

    def _start_next_step_segment(self):
        self.step_segment_idx += 1
        if self.step_segment_idx >= len(self.step_cmds_mmps):
            self._flush_step_csv()
            self._request_shutdown('step sweep 结束，已向滑台发送 0 速度停机命令。')
            return

        self.step_segment_cmd_mmps = float(self.step_cmds_mmps[self.step_segment_idx])
        self.step_segment_start_sec = self._now()
        self.step_segment_samples = []
        self.step_segment_pos0 = None

        if self.status.valid:
            self.step_segment_pos0 = (self.status.x_mm, self.status.y_mm, self.status.z_mm)

        self.step_state = 'SEGMENT_ACTIVE'
        self.get_logger().warn(
            f'[STEP] segment={self.step_segment_idx} axis={self.step_axis} '
            f'cmd_mmps={self.step_segment_cmd_mmps:.3f} enabled={self.step_axis_enabled}'
        )

    def _record_step_samples(self):
        now = self._now()
        seg_t = now - self.step_segment_start_sec

        st = self.status
        if not st.valid:
            return

        if seg_t < self.step_measure_ignore_head_sec:
            return
        if seg_t > max(0.0, self.step_hold_sec - self.step_measure_ignore_tail_sec):
            return

        self.step_segment_samples.append({
            't': now,
            'x': st.x_mm,
            'y': st.y_mm,
            'z': st.z_mm,
            'vx': st.vx_mmps,
            'vy': st.vy_mmps,
            'vz': st.vz_mmps,
        })

    def _command_step_axis(self, cmd_mmps: float):
        vx = 0.0
        vy = 0.0
        vz = 0.0

        if self.step_axis_enabled:
            if self.step_axis == 'x':
                vx = cmd_mmps
            elif self.step_axis == 'y':
                vy = cmd_mmps
            else:
                vz = cmd_mmps

        self._pub_speed(vx, vy, vz)

    def _check_step_sign_mismatch(self) -> bool:
        seg_t = self._now() - self.step_segment_start_sec
        if seg_t < self.sign_check_after_sec:
            return False

        cmd = self.step_segment_cmd_mmps
        if abs(cmd) < 1.0e-9:
            return False
        if not self.step_axis_enabled:
            return False
        if self.step_segment_pos0 is None or not self.status.valid:
            return False

        if self.step_axis == 'x':
            delta = self.status.x_mm - self.step_segment_pos0[0]
        elif self.step_axis == 'y':
            delta = self.status.y_mm - self.step_segment_pos0[1]
        else:
            delta = self.status.z_mm - self.step_segment_pos0[2]

        if cmd > 0.0 and delta < -abs(self.sign_mismatch_delta_mm):
            self.step_abort_reason = (
                f'[STEP_ABORT] SIGN_MISMATCH {self.robot_name} axis={self.step_axis} '
                f'cmd={cmd:.3f} but delta={delta:.3f} mm'
            )
            return True

        if cmd < 0.0 and delta > abs(self.sign_mismatch_delta_mm):
            self.step_abort_reason = (
                f'[STEP_ABORT] SIGN_MISMATCH {self.robot_name} axis={self.step_axis} '
                f'cmd={cmd:.3f} but delta={delta:.3f} mm'
            )
            return True

        return False

    def _compute_step_result_rows(self):
        samples = self.step_segment_samples
        st_now = self.status

        row = {
            'segment_idx': float(self.step_segment_idx),
            'cmd_mmps': float(self.step_segment_cmd_mmps),
            'robot': self.robot_name,
            'n': 0.0,
            'mean_vx_mmps': float('nan'),
            'mean_vy_mmps': float('nan'),
            'mean_vz_mmps': float('nan'),
            'mean_meas_axis_mmps': float('nan'),
            'window_dt_sec': float('nan'),
            'delta_axis_mm': float('nan'),
            'mean_speed_from_pos_mmps': float('nan'),
            'gain_from_pos': float('nan'),
            'start_x_mm': float('nan'),
            'start_y_mm': float('nan'),
            'start_z_mm': float('nan'),
            'end_x_mm': float('nan'),
            'end_y_mm': float('nan'),
            'end_z_mm': float('nan'),
            'status': 'NO_DATA',
        }

        if len(samples) >= 2:
            row['n'] = float(len(samples))

            mean_vx = sum(s['vx'] for s in samples) / len(samples)
            mean_vy = sum(s['vy'] for s in samples) / len(samples)
            mean_vz = sum(s['vz'] for s in samples) / len(samples)

            row['mean_vx_mmps'] = mean_vx
            row['mean_vy_mmps'] = mean_vy
            row['mean_vz_mmps'] = mean_vz

            if self.step_axis == 'x':
                mean_axis = mean_vx
                delta_axis = samples[-1]['x'] - samples[0]['x']
            elif self.step_axis == 'y':
                mean_axis = mean_vy
                delta_axis = samples[-1]['y'] - samples[0]['y']
            else:
                mean_axis = mean_vz
                delta_axis = samples[-1]['z'] - samples[0]['z']

            dt = max(1.0e-6, samples[-1]['t'] - samples[0]['t'])
            mean_speed_from_pos = delta_axis / dt

            row['mean_meas_axis_mmps'] = mean_axis
            row['window_dt_sec'] = dt
            row['delta_axis_mm'] = delta_axis
            row['mean_speed_from_pos_mmps'] = mean_speed_from_pos
            if abs(self.step_segment_cmd_mmps) > 1.0e-9:
                row['gain_from_pos'] = mean_speed_from_pos / self.step_segment_cmd_mmps

            row['start_x_mm'] = samples[0]['x']
            row['start_y_mm'] = samples[0]['y']
            row['start_z_mm'] = samples[0]['z']
            row['end_x_mm'] = samples[-1]['x']
            row['end_y_mm'] = samples[-1]['y']
            row['end_z_mm'] = samples[-1]['z']
            row['status'] = 'OK'

            self.get_logger().warn(
                f'[STEP_RESULT] seg={self.step_segment_idx} {self.robot_name} axis={self.step_axis} '
                f'cmd={self.step_segment_cmd_mmps:.2f} '
                f'PRIMARY mean_speed_from_pos={mean_speed_from_pos:.4f} mm/s '
                f'| delta_axis={delta_axis:.4f} mm dt={dt:.3f} '
                f'| gain_from_pos={row["gain_from_pos"]:.4f} '
                f'| feedback_only mean_meas_axis={mean_axis:.4f} mm/s'
            )
        else:
            if st_now.valid:
                row['start_x_mm'] = st_now.x_mm
                row['start_y_mm'] = st_now.y_mm
                row['start_z_mm'] = st_now.z_mm
                row['end_x_mm'] = st_now.x_mm
                row['end_y_mm'] = st_now.y_mm
                row['end_z_mm'] = st_now.z_mm

        self.step_results_rows.append(row)

    def _flush_step_csv(self):
        csv_path = self._effective_step_csv_path()
        csv_dir = os.path.dirname(csv_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)

        fieldnames = [
            'segment_idx',
            'cmd_mmps',
            'robot',
            'n',
            'mean_vx_mmps',
            'mean_vy_mmps',
            'mean_vz_mmps',
            'mean_meas_axis_mmps',
            'window_dt_sec',
            'delta_axis_mm',
            'mean_speed_from_pos_mmps',
            'gain_from_pos',
            'start_x_mm',
            'start_y_mm',
            'start_z_mm',
            'end_x_mm',
            'end_y_mm',
            'end_z_mm',
            'status',
        ]

        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.step_results_rows:
                writer.writerow(row)

        self.get_logger().warn(f'[CSV] saved to {csv_path}')

    def _return_to_baseline_step(self):
        if not self.step_return_to_baseline:
            self.step_state = 'SEGMENT_GAP'
            return

        now = self._now()
        if self.step_return_start_sec <= 0.0:
            self.step_return_start_sec = now

        if (now - self.step_return_start_sec) > self.step_return_timeout_sec:
            self.step_abort_reason = '[STEP_ABORT] RETURN_TO_BASELINE_TIMEOUT'
            self._flush_step_csv()
            self._request_shutdown(self.step_abort_reason)
            return

        if not self.step_axis_enabled or self.baseline_pos is None or not self.status.valid:
            self._pub_speed(0.0, 0.0, 0.0)
            self.step_state = 'SEGMENT_GAP'
            self.step_return_start_sec = 0.0
            return

        bx, by, bz = self.baseline_pos
        all_ok = True

        if self.step_axis == 'x':
            err = bx - self.status.x_mm
            if abs(err) <= self.step_return_tol_mm:
                vx = 0.0
            else:
                vx = max(-self.step_return_v_limit_mmps,
                         min(self.step_return_v_limit_mmps, self.step_return_kp_mmps_per_mm * err))
                all_ok = False
            self._pub_speed(vx, 0.0, 0.0)

        elif self.step_axis == 'y':
            err = by - self.status.y_mm
            if abs(err) <= self.step_return_tol_mm:
                vy = 0.0
            else:
                vy = max(-self.step_return_v_limit_mmps,
                         min(self.step_return_v_limit_mmps, self.step_return_kp_mmps_per_mm * err))
                all_ok = False
            self._pub_speed(0.0, vy, 0.0)

        else:
            err = bz - self.status.z_mm
            if abs(err) <= self.step_return_tol_mm:
                vz = 0.0
            else:
                vz = max(-self.step_return_v_limit_mmps,
                         min(self.step_return_v_limit_mmps, self.step_return_kp_mmps_per_mm * err))
                all_ok = False
            self._pub_speed(0.0, 0.0, vz)

        if all_ok:
            self._pub_speed(0.0, 0.0, 0.0)
            self.step_return_start_sec = 0.0
            self.step_state = 'SEGMENT_GAP'

    def _run_step_once(self):
        if not self._capture_baseline_if_needed():
            self._pub_speed(0.0, 0.0, 0.0)
            return

        if not self.step_axis_enabled:
            self._request_shutdown(f'[STEP_ABORT] axis {self.step_axis} disabled for {self.robot_name}')
            return

        if not self._safety_check():
            self._flush_step_csv()
            self._request_shutdown('STEP_ABORT_BY_SAFETY')
            return

        if self.step_state == 'WAIT_BASELINE':
            self._start_next_step_segment()
            return

        if self.step_state == 'SEGMENT_ACTIVE':
            seg_t = self._now() - self.step_segment_start_sec
            self._command_step_axis(self.step_segment_cmd_mmps)
            self._record_step_samples()

            if self._check_step_sign_mismatch():
                self._compute_step_result_rows()
                self._flush_step_csv()
                self._request_shutdown(self.step_abort_reason)
                return

            if seg_t >= self.step_hold_sec:
                self._pub_speed(0.0, 0.0, 0.0)
                self._compute_step_result_rows()
                self.step_state = 'RETURN_BASELINE'
                self.step_return_start_sec = 0.0
                return

        elif self.step_state == 'RETURN_BASELINE':
            self._return_to_baseline_step()
            return

        elif self.step_state == 'SEGMENT_GAP':
            self._start_next_step_segment()
            return

    # ------------------------------------------------------------------
    # 主 timer
    # ------------------------------------------------------------------
    def _on_timer(self):
        if self.shutdown_requested:
            return

        if self.mode == 'sine':
            self._run_sine_once()
        elif self.mode == 'step_sweep':
            self._run_step_once()


def main(args=None):
    rclpy.init(args=args)
    node = SlideSpeedSineTest()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                if node.mode == 'sine':
                    node._finalize_all_pending_sine_cycles()
                    node._flush_sine_cycle_csv()
                    node._flush_sine_summary_csv()
                elif node.mode == 'step_sweep':
                    node._flush_step_csv()
            except Exception:
                pass

            if rclpy.ok():
                try:
                    node.destroy_node()
                except Exception:
                    pass
                rclpy.shutdown()


if __name__ == '__main__':
    main()