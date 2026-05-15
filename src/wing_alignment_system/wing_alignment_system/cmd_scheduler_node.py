#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from std_msgs.msg import UInt32, Bool

from wing_alignment_system.common_utils import now_sec, expanduser
from wing_alignment_system.common_async_csv import AsyncCsvLogger
from wing_alignment_system.communication_profile import (
    COMMUNICATION_PROFILE_CSV_FIELDS,
    communication_profile_csv_row,
    declare_communication_profile,
)
from wing_alignment_system.baseline_guard import (
    BASELINE_GUARD_CSV_FIELDS,
    baseline_guard_csv_row,
    declare_baseline_guard,
)
from wing_alignment_system.cmd_scheduler_types import SchedulerConfig
from wing_alignment_system.cmd_scheduler_policy import SchedulerPolicy


class CmdScheduler(Node):
    def __init__(self):
        super().__init__("cmd_scheduler")

        self.robots = [str(x) for x in self.declare_parameter("robots", ["tracer1"]).value]
        self.tick_hz = max(10.0, float(self.declare_parameter("tick_hz", 50.0).value))
        self.v_max = max(1e-6, float(self.declare_parameter("v_max", 0.16).value))
        self.w_max = max(1e-6, float(self.declare_parameter("w_max", 0.55).value))
        self.bytes_per_command = max(1, int(self.declare_parameter("bytes_per_command", 96).value))
        self.run_id = str(self.declare_parameter("run_id", "").value).strip() or time.strftime("%Y%m%d_%H%M%S")
        self.communication_profile = declare_communication_profile(self)
        self._communication_profile_row = communication_profile_csv_row(self.communication_profile)
        self.baseline_guard = declare_baseline_guard(self, "cmd_scheduler")
        self._baseline_guard_row = baseline_guard_csv_row(self.baseline_guard)

        self.base_period = max(0.0, float(self.declare_parameter("base_period_ms", 60.0).value) * 1e-3)
        self.jitter = max(0.0, float(self.declare_parameter("jitter_ms", 10.0).value) * 1e-3)
        self.t_min = max(0.01, float(self.declare_parameter("T_min_ms", 15.0).value) * 1e-3)
        self.t_max = max(0.05, float(self.declare_parameter("T_max_ms", 120.0).value) * 1e-3)
        self.age_th = max(0.0, float(self.declare_parameter("age_th_ms", 80.0).value) * 1e-3)

        self.prec_tmax_scale = float(self.declare_parameter("precision_T_max_scale", 0.60).value)
        self.prec_age_scale = float(self.declare_parameter("precision_age_th_scale", 0.60).value)
        self.voi_th = float(self.declare_parameter("voi_th", 0.08).value)
        self.voi_high_th = float(self.declare_parameter("voi_high_th", 0.25).value)
        self.dup_delay = max(0.0, float(self.declare_parameter("dup_delay_ms", 18.0).value) * 1e-3)

        self.enable_eps = bool(self.declare_parameter("enable_eps_trigger", True).value)
        self.eps_th = float(self.declare_parameter("eps_th", 0.10).value)
        self.eps_high_th = float(self.declare_parameter("eps_high_th", 0.25).value)

        self.robot_mocap_topics = [str(x) for x in self.declare_parameter("robot_mocap_topics", [""]).value]
        self.mm_to_m = float(self.declare_parameter("mm_to_m", 0.001).value)
        self.swap_xz = bool(self.declare_parameter("swap_xz", False).value)
        self.negate_x = bool(self.declare_parameter("negate_x", False).value)
        self.negate_z = bool(self.declare_parameter("negate_z", True).value)

        self.declare_parameter("override_timeout_ms", 150.0)

        base_dir = expanduser(str(self.declare_parameter("log_dir", "~/.ros/cmd_safety_logs").value))
        self.log_dir = os.path.join(base_dir, self.run_id)

        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_ack = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_local = QoSProfile(depth=10)

        cfg = SchedulerConfig(
            robots=self.robots, tick_hz=self.tick_hz, v_max=self.v_max, w_max=self.w_max,
            base_period=self.base_period, jitter=self.jitter, t_min=self.t_min, t_max=self.t_max, age_th=self.age_th,
            prec_tmax_scale=self.prec_tmax_scale, prec_age_scale=self.prec_age_scale, voi_th=self.voi_th,
            voi_high_th=self.voi_high_th, dup_delay=self.dup_delay, enable_eps=self.enable_eps,
            eps_th=self.eps_th, eps_high_th=self.eps_high_th, t0=now_sec(self),
        )
        self.policy = SchedulerPolicy(cfg)

        self.pub_cmd: Dict[str, object] = {}
        self.pub_seq: Dict[str, object] = {}

        for rn in self.robots:
            self.pub_cmd[rn] = self.create_publisher(TwistStamped, f'/{rn}/cmd_vel_stamped', qos_cmd)
            self.pub_seq[rn] = self.create_publisher(UInt32, f'/{rn}/cmd_seq', qos_cmd)
            self.create_subscription(Twist, f'/{rn}/cmd_vel_desired', self._mk_desired_cb(rn), qos_local)
            self.create_subscription(UInt32, f'/{rn}/last_cmd_seq', self._mk_ack_cb(rn), qos_ack)
            self.create_subscription(Bool, f'/{rn}/precision_mode', self._mk_precision_cb(rn), qos_local)
            if self.enable_eps:
                self.create_subscription(Twist, f'/{rn}/cmd_goal', self._mk_goal_cb(rn), qos_local)

        if self.enable_eps:
            for rn, tp in zip(self.robots, self.robot_mocap_topics):
                if tp:
                    self.create_subscription(PoseStamped, tp, self._mk_mocap_cb(rn), qos_cmd)

        self.events_logger = AsyncCsvLogger(
            os.path.join(self.log_dir, 'events.csv'),
            [
                'run_id', 'robot_id', 'task_phase', 't_tx', 'robot', 'seq',
                'command_id', 'command_type', 'v', 'w', 'reason',
                'communication_load_bytes', 'scheduler_decision',
                'voi', 'eps', 'age_est', 'unacked', 'precision_mode',
                *COMMUNICATION_PROFILE_CSV_FIELDS,
                *BASELINE_GUARD_CSV_FIELDS,
            ]
        )

        self.timer = self.create_timer(1.0 / self.tick_hz, self._tick)

    def _mk_desired_cb(self, rn: str):
        def cb(msg: Twist):
            self.policy.on_desired(rn, float(msg.linear.x), float(msg.angular.z))
        return cb

    def _mk_ack_cb(self, rn: str):
        def cb(msg: UInt32):
            self.policy.on_ack(rn, int(msg.data), now_sec(self))
        return cb

    def _mk_precision_cb(self, rn: str):
        def cb(msg: Bool):
            self.policy.on_precision(rn, bool(msg.data))
        return cb

    def _mk_goal_cb(self, rn: str):
        def cb(msg: Twist):
            self.policy.on_goal(rn, float(msg.linear.x), float(msg.linear.y))
        return cb

    def _mk_mocap_cb(self, rn: str):
        def cb(msg: PoseStamped):
            mx = msg.pose.position.x * self.mm_to_m
            mz = msg.pose.position.z * self.mm_to_m
            xw, yw = (mz, mx) if self.swap_xz else (mx, mz)
            if self.negate_x:
                xw = -xw
            if self.negate_z:
                yw = -yw
            self.policy.on_pose(rn, xw, yw)
        return cb

    def _publish(self, rn: str, seq: int, v: float, w: float, reason: str, age_est: float):
        s = self.policy.st[rn]
        self.pub_seq[rn].publish(UInt32(data=int(seq)))
        cm = TwistStamped()
        cm.header.stamp = self.get_clock().now().to_msg()
        cm.header.frame_id = str(seq)
        cm.twist.linear.x = float(v)
        cm.twist.angular.z = float(w)
        self.pub_cmd[rn].publish(cm)
        self.events_logger.log({
            'run_id': self.run_id, 'robot_id': rn, 'task_phase': 'unknown',
            't_tx': f'{now_sec(self):.6f}', 'robot': rn, 'seq': int(seq),
            'command_id': int(seq), 'command_type': 'cmd_vel',
            'v': float(v), 'w': float(w), 'reason': reason,
            'communication_load_bytes': int(seq) * self.bytes_per_command,
            'scheduler_decision': reason,
            'voi': float(s.voi), 'eps': float(s.eps), 'age_est': float(age_est),
            'unacked': int(s.unacked_streak), 'precision_mode': int(s.precision_mode),
            **self._communication_profile_row,
            **self._baseline_guard_row,
        })

    def _tick(self):
        decisions = self.policy.tick(now_sec(self))
        for rn, dec in decisions:
            self._publish(rn, dec.seq, dec.v, dec.w, dec.reason, dec.age_est)


def main(args=None):
    rclpy.init(args=args)
    node = CmdScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.events_logger.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
