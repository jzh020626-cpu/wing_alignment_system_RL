#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import UInt32, Bool

from wing_alignment_system.common_utils import now_sec, expanduser
from wing_alignment_system.common_async_csv import AsyncCsvLogger
from wing_alignment_system.common_rt import EventQueue, FixedRateLoop, LatestValueBuffer
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
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig
from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy


class CmdWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_watchdog")
        self.robot = str(self.declare_parameter('robot_name', 'tracer1').value)
        self.run_id = str(self.declare_parameter('run_id', '').value).strip() or time.strftime('%Y%m%d_%H%M%S')
        self.communication_profile = declare_communication_profile(self)
        self._communication_profile_row = communication_profile_csv_row(self.communication_profile)
        self.baseline_guard = declare_baseline_guard(self, "cmd_watchdog")
        self._baseline_guard_row = baseline_guard_csv_row(self.baseline_guard)
        self.watchdog_hz = float(self.declare_parameter('watchdog_hz', 80.0).value)
        self.age_safe = float(self.declare_parameter('age_safe_ms', 120.0).value) * 1e-3
        self.age_stop = float(self.declare_parameter('age_stop_ms', 300.0).value) * 1e-3
        self.decay_mode = str(self.declare_parameter('decay_mode', 'linear').value).lower().strip()
        self.decay_k = float(self.declare_parameter('decay_k', 3.0).value)
        self.pair_window = float(self.declare_parameter('pair_window_ms', 60.0).value) * 1e-3
        self.publish_before_first_cmd = bool(self.declare_parameter('publish_before_first_cmd', False).value)
        self.topic_cmd_stamped = f'/{self.robot}/cmd_vel_stamped'
        self.topic_ack = f'/{self.robot}/last_cmd_seq'
        self.topic_cmd_out = f'/{self.robot}/cmd_vel'
        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_out = QoSProfile(depth=10)
        qos_ack = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_emg = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        qos_volatile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=5)
        cfg = WatchdogConfig(watchdog_hz=self.watchdog_hz, age_safe=self.age_safe, age_stop=self.age_stop, decay_mode=self.decay_mode, decay_k=self.decay_k)
        self.policy = WatchdogPolicy(cfg)
        self._cmd_rx = EventQueue(maxsize=4000)
        self._fallback_rx_seq = 0
        self._stop_req = LatestValueBuffer()
        self._resume_req = LatestValueBuffer()
        self._emergency_req = LatestValueBuffer(False)
        self._last_stop_req_ts = 0.0
        self._last_resume_req_ts = 0.0
        self._last_ctrl_apply_ts = 0.0
        self._last_cmd_source_ts = 0.0
        self._last_cmd_rx_ts = 0.0
        self._last_cmd_seq = 0
        self._have_accepted_cmd = False
        self._last_published_state = None
        self.pub_cmd = self.create_publisher(Twist, self.topic_cmd_out, qos_out)
        self.pub_ack = self.create_publisher(UInt32, self.topic_ack, qos_ack)
        self.create_subscription(TwistStamped, self.topic_cmd_stamped, self._cmd_cb, qos_in)
        self.create_subscription(Bool, f'/{self.robot}/cmd_stop', self._stop_cb, qos_volatile)
        self.create_subscription(Bool, f'/{self.robot}/cmd_resume', self._resume_cb, qos_volatile)
        emg_topic = str(self.declare_parameter('emergency_stop_topic', '/wing_alignment/emergency_stop').value)
        self.create_subscription(Bool, emg_topic, self._emg_cb, qos_emg)
        base_dir = expanduser(str(self.declare_parameter('log_dir', '~/.ros/cmd_safety_logs').value))
        self.log_dir = os.path.join(base_dir, self.run_id)
        self.rx_logger = AsyncCsvLogger(
            os.path.join(self.log_dir, f'rx_{self.robot}.csv'),
            [
                'run_id', 'robot_id', 'command_id', 'command_type',
                't_source', 't_rx', 'delta_net_proxy_ms',
                *COMMUNICATION_PROFILE_CSV_FIELDS,
                *BASELINE_GUARD_CSV_FIELDS,
            ],
        )
        self.ts_logger = AsyncCsvLogger(
            os.path.join(self.log_dir, f'ts_{self.robot}.csv'),
            [
                'run_id', 'robot_id', 'command_id', 'command_type',
                't_source', 't_rx', 't_watchdog', 't', 'age', 'age_ms',
                'delta_net_proxy_ms', 'delta_exec_proxy_ms', 'delta_eff_proxy_ms',
                'queue_delay_proxy_ms', 'v', 'w', 'state', 'watchdog_action',
                'stale_reason', 'emg',
                *COMMUNICATION_PROFILE_CSV_FIELDS,
                *BASELINE_GUARD_CSV_FIELDS,
            ],
        )
        self._rt_loop = FixedRateLoop(
            name=f'{self.robot}_watchdog_rt',
            hz=max(1.0, self.watchdog_hz),
            tick_fn=self._watchdog,
            on_error=self._on_rt_error,
        )
        self._rt_loop.start()

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        try:
            value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        except AttributeError:
            return 0.0
        return value if value > 0.0 else 0.0

    @staticmethod
    def _fmt_sec(value: float) -> str:
        return f'{float(value):.6f}' if float(value) > 0.0 else ''

    @staticmethod
    def _fmt_ms(value) -> str:
        return f'{float(value) * 1e3:.6f}' if value is not None else ''

    def _cmd_cb(self, msg: TwistStamped):
        try:
            seq = int(msg.header.frame_id)
        except ValueError:
            self._fallback_rx_seq += 1
            seq = self._fallback_rx_seq
        t_rx = now_sec(self)
        t_source = self._stamp_to_sec(msg.header.stamp)
        self._cmd_rx.put((int(seq), float(msg.twist.linear.x), float(msg.twist.angular.z), t_rx, t_source))

    def _stop_cb(self, msg: Bool):
        if bool(msg.data):
            self._stop_req.set(now_sec(self))

    def _resume_cb(self, msg: Bool):
        if bool(msg.data):
            self._resume_req.set(now_sec(self))

    def _emg_cb(self, msg: Bool):
        self._emergency_req.set(bool(msg.data))

    def _watchdog(self):
        stop_req = self._stop_req.pop()
        if stop_req is not None:
            self._last_stop_req_ts = float(stop_req)

        resume_req = self._resume_req.pop()
        if resume_req is not None:
            self._last_resume_req_ts = float(resume_req)

        ctrl_events = []
        if self._last_stop_req_ts > self._last_ctrl_apply_ts:
            ctrl_events.append((self._last_stop_req_ts, 'stop'))
        if self._last_resume_req_ts > self._last_ctrl_apply_ts:
            ctrl_events.append((self._last_resume_req_ts, 'resume'))
        ctrl_events.sort(key=lambda item: item[0])

        # Treat closely-spaced stop/resume chatter as one paired control update,
        # so pair_window_ms keeps its original debounce meaning.
        if (
            len(ctrl_events) >= 2 and
            abs(ctrl_events[-1][0] - ctrl_events[-2][0]) <= self.pair_window and
            ctrl_events[-1][1] != ctrl_events[-2][1]
        ):
            ctrl_events = [ctrl_events[-1]]

        for ts, kind in ctrl_events:
            if kind == 'stop':
                self.policy.on_stop(True)
            elif kind == 'resume':
                self.policy.on_resume(True)
            self._last_ctrl_apply_ts = max(self._last_ctrl_apply_ts, float(ts))

        emergency_level = self._emergency_req.get()
        if emergency_level is not None:
            self.policy.on_emergency(bool(emergency_level))

        for seq, v, w, t_rx, t_source in self._cmd_rx.drain():
            accepted = self.policy.on_cmd(seq, v, w, t_rx)
            if not accepted:
                continue
            self._have_accepted_cmd = True
            self._last_cmd_source_ts = float(t_source)
            self._last_cmd_rx_ts = float(t_rx)
            self._last_cmd_seq = int(seq)
            self.pub_ack.publish(UInt32(data=int(seq)))
            delta_net_proxy = (float(t_rx) - float(t_source)) if float(t_source) > 0.0 else None
            self.rx_logger.log({
                'run_id': self.run_id,
                'robot_id': self.robot,
                'command_id': int(seq),
                'command_type': 'cmd_vel',
                't_source': self._fmt_sec(t_source),
                't_rx': f'{float(t_rx):.6f}',
                'delta_net_proxy_ms': self._fmt_ms(delta_net_proxy),
                **self._communication_profile_row,
                **self._baseline_guard_row,
            })

        now = now_sec(self)
        out = self.policy.compute(now)
        should_publish = self.publish_before_first_cmd or self._have_accepted_cmd
        if (not should_publish) and (out.state in ('CMD_STOP', 'EMERGENCY_STOP')):
            should_publish = (self._last_published_state != out.state)
        if not should_publish:
            return
        cmd = Twist()
        cmd.linear.x = float(out.applied_v)
        cmd.angular.z = float(out.applied_w)
        self.pub_cmd.publish(cmd)
        self._last_published_state = out.state
        delta_net_proxy = (
            float(self._last_cmd_rx_ts) - float(self._last_cmd_source_ts)
            if self._last_cmd_source_ts > 0.0 and self._last_cmd_rx_ts > 0.0
            else None
        )
        delta_exec_proxy = (
            float(now) - float(self._last_cmd_rx_ts)
            if self._last_cmd_rx_ts > 0.0
            else None
        )
        delta_eff_proxy = (
            float(now) - float(self._last_cmd_source_ts)
            if self._last_cmd_source_ts > 0.0
            else None
        )
        if out.state == 'DECAY':
            stale_reason = 'age_safe_exceeded'
        elif out.state == 'AGE_STOP':
            stale_reason = 'age_stop_exceeded'
        elif out.state == 'CMD_STOP':
            stale_reason = 'cmd_stop_latched'
        elif out.state == 'EMERGENCY_STOP':
            stale_reason = 'emergency_latched'
        else:
            stale_reason = ''
        self.ts_logger.log({
            'run_id': self.run_id,
            'robot_id': self.robot,
            'command_id': int(self._last_cmd_seq),
            'command_type': 'cmd_vel',
            't_source': self._fmt_sec(self._last_cmd_source_ts),
            't_rx': self._fmt_sec(self._last_cmd_rx_ts),
            't_watchdog': f'{now:.6f}',
            't': f'{now:.6f}',
            'age': float(out.age),
            'age_ms': f'{float(out.age) * 1e3:.6f}',
            'delta_net_proxy_ms': self._fmt_ms(delta_net_proxy),
            'delta_exec_proxy_ms': self._fmt_ms(delta_exec_proxy),
            'delta_eff_proxy_ms': self._fmt_ms(delta_eff_proxy),
            'queue_delay_proxy_ms': self._fmt_ms(delta_exec_proxy),
            'v': float(out.applied_v),
            'w': float(out.applied_w),
            'state': out.state,
            'watchdog_action': out.state,
            'stale_reason': stale_reason,
            'emg': int(self.policy.st.emergency_latched),
            **self._communication_profile_row,
            **self._baseline_guard_row,
        })

    def _on_rt_error(self, exc: BaseException):
        self.get_logger().error(
            f'[{self.robot}] watchdog fixed-rate loop crashed: {exc}\n{traceback.format_exc()}'
        )

    def destroy_node(self):
        if hasattr(self, '_rt_loop'):
            self._rt_loop.stop()
        if hasattr(self, 'pub_cmd'):
            self.pub_cmd.publish(Twist())
        if hasattr(self, 'rx_logger'):
            self.rx_logger.close()
        if hasattr(self, 'ts_logger'):
            self.ts_logger.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
