#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
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
        self.freshness_tau_ms = float(self.declare_parameter('freshness_tau_ms', 1000.0).value)
        self.publish_before_first_cmd = bool(self.declare_parameter('publish_before_first_cmd', False).value)
        self.enable_execution_mode_output = bool(self.declare_parameter('enable_execution_mode_output', False).value)
        self.degraded_linear_scale = float(self.declare_parameter('degraded_linear_scale', 0.5).value)
        self.degraded_angular_scale = float(self.declare_parameter('degraded_angular_scale', 0.25).value)
        self.topic_cmd_stamped = f'/{self.robot}/cmd_vel_stamped'
        self.topic_ack = f'/{self.robot}/last_cmd_seq'
        self.topic_cmd_out = f'/{self.robot}/cmd_vel'
        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_out = QoSProfile(depth=10)
        qos_ack = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_emg = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        qos_volatile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, depth=5)
        cfg = WatchdogConfig(
            watchdog_hz=self.watchdog_hz,
            age_safe=self.age_safe,
            age_stop=self.age_stop,
            decay_mode=self.decay_mode,
            decay_k=self.decay_k,
            enable_execution_mode_output=self.enable_execution_mode_output,
            degraded_linear_scale=self.degraded_linear_scale,
            degraded_angular_scale=self.degraded_angular_scale,
        )
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
        self._last_cmd_metadata = self._empty_cmd_metadata()
        self._have_accepted_cmd = False
        self._last_published_state = None
        self._last_ctrl_log_kind = None
        self._last_ctrl_log_state = None
        self._last_ctrl_log_ts = 0.0
        self._last_rt_overrun_log_wall = 0.0
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
        self.mode_logger = AsyncCsvLogger(
            os.path.join(self.log_dir, f'mode_timeline_{self.robot}.csv'),
            [
                'run_id', 'timestamp', 'robot_id', 'seq',
                'transmission_mode', 'execution_mode', 'AoI_ms', 'effective_freshness', 'phase',
                'output_scale', 'stop_reason', 'watchdog_state',
                'cmd_v_in', 'cmd_w_in', 'cmd_v_out', 'cmd_w_out',
                't_source', 't_rx', 't_watchdog',
            ],
        )
        self._rt_loop = FixedRateLoop(
            name=f'{self.robot}_watchdog_rt',
            hz=max(1.0, self.watchdog_hz),
            tick_fn=self._watchdog,
            on_error=self._on_rt_error,
            on_overrun=self._on_rt_overrun,
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

    @staticmethod
    def _empty_cmd_metadata() -> dict:
        return {
            'seq_id': 0,
            'transmission_mode': 'full_update',
            'execution_mode': 'normal',
            'aoi_ms': None,
            'effective_freshness': None,
            'phase': 'standby',
        }

    @staticmethod
    def _decode_cmd_frame_id(frame_id: str) -> dict:
        raw = str(frame_id or '').strip()
        if not raw:
            return CmdWatchdog._empty_cmd_metadata()
        if raw.isdigit():
            metadata = CmdWatchdog._empty_cmd_metadata()
            metadata['seq_id'] = int(raw)
            return metadata
        metadata = CmdWatchdog._empty_cmd_metadata()
        for item in raw.split('|'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            if key == 'seq':
                metadata['seq_id'] = int(value or 0)
            elif key == 'tx':
                metadata['transmission_mode'] = value or 'full_update'
            elif key == 'exec':
                metadata['execution_mode'] = value or 'normal'
            elif key == 'aoi':
                metadata['aoi_ms'] = float(value) if value else None
            elif key == 'eff':
                metadata['effective_freshness'] = float(value) if value else None
            elif key == 'phase':
                metadata['phase'] = value or 'standby'
        return metadata

    @staticmethod
    def _should_log_control_event(self, now: float, kind: str, stop_latched: bool, emergency_latched: bool) -> bool:
        state = (bool(stop_latched), bool(emergency_latched))
        last_kind = getattr(self, '_last_ctrl_log_kind', None)
        last_state = getattr(self, '_last_ctrl_log_state', None)
        last_ts = float(getattr(self, '_last_ctrl_log_ts', 0.0) or 0.0)
        if last_kind == kind and last_state == state and float(now) - last_ts < 2.0:
            return False
        self._last_ctrl_log_kind = kind
        self._last_ctrl_log_state = state
        self._last_ctrl_log_ts = float(now)
        return True

    def _cmd_cb(self, msg: TwistStamped):
        metadata = self._decode_cmd_frame_id(str(msg.header.frame_id))
        seq = int(metadata['seq_id'])
        if seq <= 0:
            self._fallback_rx_seq += 1
            seq = self._fallback_rx_seq
        t_rx = now_sec(self)
        t_source = self._stamp_to_sec(msg.header.stamp)
        self._last_cmd_metadata = metadata
        self._cmd_rx.put((int(seq), float(msg.twist.linear.x), float(msg.twist.angular.z), t_rx, t_source, metadata))

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

        for seq, v, w, t_rx, t_source, metadata in self._cmd_rx.drain():
            accepted = self.policy.on_cmd(
                seq,
                v,
                w,
                t_rx,
                execution_mode=str(metadata.get('execution_mode', 'normal')),
            )
            if not accepted:
                continue
            self._have_accepted_cmd = True
            self._last_cmd_source_ts = float(t_source)
            self._last_cmd_rx_ts = float(t_rx)
            self._last_cmd_seq = int(seq)
            self._last_cmd_metadata = dict(metadata)
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
        aoi_ms = self._last_cmd_metadata.get('aoi_ms')
        if aoi_ms is None and self._last_cmd_source_ts > 0.0:
            aoi_ms = max(0.0, (float(now) - self._last_cmd_source_ts) * 1000.0)
        eff = None
        if aoi_ms is not None:
            eff = math.exp(-float(aoi_ms) / max(1.0, self.freshness_tau_ms))
            eff = max(0.0, min(1.0, eff))
        self.mode_logger.log({
            'run_id': self.run_id,
            'timestamp': f'{now:.6f}',
            'robot_id': self.robot,
            'seq': int(self._last_cmd_seq),
            'transmission_mode': str(self._last_cmd_metadata.get('transmission_mode', 'full_update')),
            'execution_mode': str(self._last_cmd_metadata.get('execution_mode', 'normal')),
            'AoI_ms': '' if aoi_ms is None else f'{float(aoi_ms):.3f}',
            'effective_freshness': '' if eff is None else f'{float(eff):.6f}',
            'phase': str(self._last_cmd_metadata.get('phase', 'standby')),
            'output_scale': f'{float(getattr(out, "output_scale", 1.0)):.6f}',
            'stop_reason': str(getattr(out, "stop_reason", "") or stale_reason),
            'watchdog_state': out.state,
            'cmd_v_in': f'{float(self.policy.st.last_v):.6f}',
            'cmd_w_in': f'{float(self.policy.st.last_w):.6f}',
            'cmd_v_out': f'{float(out.applied_v):.6f}',
            'cmd_w_out': f'{float(out.applied_w):.6f}',
            't_source': self._fmt_sec(self._last_cmd_source_ts),
            't_rx': self._fmt_sec(self._last_cmd_rx_ts),
            't_watchdog': f'{now:.6f}',
        })

    def _on_rt_error(self, exc: BaseException):
        self.get_logger().error(
            f'[{self.robot}] watchdog fixed-rate loop crashed: {exc}\n{traceback.format_exc()}'
        )

    def _on_rt_overrun(self, loop_name: str, tick_sec: float, overrun_sec: float, count: int):
        wall = time.time()
        if wall - self._last_rt_overrun_log_wall < 5.0 and int(count) % 100 != 1:
            return
        self._last_rt_overrun_log_wall = wall
        period_ms = 1000.0 / max(1.0, float(self.watchdog_hz))
        self.get_logger().warn(
            f'[{self.robot}] {loop_name} overrun count={int(count)} '
            f'tick={float(tick_sec) * 1000.0:.2f}ms period={period_ms:.2f}ms '
            f'late={float(overrun_sec) * 1000.0:.2f}ms'
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
        if hasattr(self, 'mode_logger'):
            self.mode_logger.close()
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
