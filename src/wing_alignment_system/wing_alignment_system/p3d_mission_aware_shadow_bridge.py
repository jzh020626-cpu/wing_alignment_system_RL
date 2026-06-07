#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3-D0b Mission-Aware Shadow Cmd Bridge.

Inputs:
  /{robot}/cmd_vel_desired   (geometry_msgs/Twist)  -- from goto_pose_driver
  /fr_validation/derived_phase_status (std_msgs/String JSON)  -- from phase source

Output:
  /{robot}/cmd_vel_stamped   (geometry_msgs/TwistStamped)
    with frame_id metadata: seq, tx_mode, exec_mode, aoi_ms, effective_freshness, phase

Constraints:
  - safe_idle_no_publish=true   (cmd_watchdog does not publish /cmd_vel)
  - enable_execution_mode_output=false
  - output=false (no real chassis intervention)
  - No delay/loss/jitter injection
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
import json


# Phase-risk: higher urgency in critical phases, lower in standby
LOW_RISK_PHASES = {"standby", "approach"}
CRITICAL_PHASES = {"level_recenter", "transport"}


def _pick_exec_mode(*, task_phase: str, effective_freshness: float, aoi_ms: float, stale_indicator: float) -> str:
    """Compute execution_mode from phase + freshness state. Never 'normal' for
    real phases unless freshness is excellent; standby stays normal."""
    if task_phase == "standby":
        return "normal"
    if stale_indicator >= 1.0 or aoi_ms >= 500.0:
        return "safe_stop"
    if task_phase in CRITICAL_PHASES and (effective_freshness < 0.40 or aoi_ms >= 350.0):
        return "hold"
    if effective_freshness < 0.65 or aoi_ms >= 220.0:
        return "degraded"
    return "normal"


def _pick_tx_mode(*, task_phase: str, task_progress: float, effective_freshness: float, aoi_ms: float, stale_indicator: float) -> str:
    """Compute transmission_mode from phase + freshness state."""
    if task_phase == "standby":
        return "compact"
    low_fresh = effective_freshness < 0.45
    high_aoi = aoi_ms >= 300.0
    stale = stale_indicator >= 1.0
    critical = task_phase in CRITICAL_PHASES
    if stale or (critical and (low_fresh or high_aoi)):
        return "urgent" if stale or aoi_ms >= 400.0 else "full"
    if task_phase in LOW_RISK_PHASES and effective_freshness >= 0.85 and aoi_ms <= 150.0 and task_progress < 0.85:
        return "compact"
    return "full"


def _build_frame_id(*, seq: int, tx: str, exec_mode: str, aoi: float, eff: float, phase: str) -> str:
    return f"seq={seq}|tx={tx}|exec={exec_mode}|aoi={aoi:.1f}|eff={eff:.3f}|phase={phase}"


_FULL_SWEEP_MODES = ["normal", "degraded", "hold", "safe_stop"]


def _stress_exec_mode(*, task_phase: str, profile: str, sweep_counter: int = 0) -> str:
    """Apply forced execution profile, bypassing freshness-based logic."""
    if profile == "degraded_only":
        return "normal" if task_phase == "standby" else "degraded"
    if profile == "full_sweep":
        idx = sweep_counter % len(_FULL_SWEEP_MODES)
        return _FULL_SWEEP_MODES[idx]
    return "normal"


class P3DMissionAwareShadowBridge(Node):
    def __init__(self):
        super().__init__("p3d_mission_aware_shadow_bridge")
        robots_raw = str(self.declare_parameter("robots", "tracer1,tracer2,tracer3").value)
        self.robots = [r.strip() for r in robots_raw.split(",") if r.strip()]
        self.phase_topic = str(self.declare_parameter("phase_topic", "/fr_validation/derived_phase_status").value)
        self.force_exec_profile = str(self.declare_parameter("force_exec_profile", "none").value)

        # Phase state
        self._task_phase = "standby"
        self._task_progress = 0.0
        self._phase_source = "unavailable"

        # Freshness / AoI state (receiver-side proxy)
        self._effective_freshness = 0.70
        self._aoi_ms = 180.0
        self._stale_indicator = 0.0
        self._last_phase_update_monotonic = 0.0

        qos_in = QoSProfile(depth=10)
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._seq = {rn: 0 for rn in self.robots}
        self._sweep_counter = {rn: 0 for rn in self.robots}
        self._pubs = {}
        for rn in self.robots:
            self._pubs[rn] = self.create_publisher(TwistStamped, f"/{rn}/cmd_vel_stamped", qos_out)
            self.create_subscription(Twist, f"/{rn}/cmd_vel_desired", self._mk_desired_cb(rn), qos_in)

        # Phase subscriber
        self.create_subscription(String, self.phase_topic, self._phase_cb, qos_in)

        # Age timer -- AoI increases when no phase update
        self._age_timer = self.create_timer(0.5, self._age_tick)

        self.get_logger().info(
            f"[P3-D0b MissionAwareShadowBridge] robots={self.robots} "
            f"phase_topic={self.phase_topic} force_exec_profile={self.force_exec_profile}"
        )

    def _phase_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        self._task_phase = str(payload.get("task_phase", "standby"))
        self._task_progress = float(payload.get("task_progress", 0.0))
        self._phase_source = str(payload.get("source_mode", payload.get("phase_source", "unknown")))
        self._effective_freshness = float(payload.get("Effective_Freshness", 0.70))
        self._aoi_ms = float(payload.get("AoI_ms", 180.0))
        self._stale_indicator = float(payload.get("stale_indicator", 0.0))
        self._last_phase_update_monotonic = now

    def _age_tick(self):
        """Tick AoI: grow linearly when no phase updates arrive."""
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self._last_phase_update_monotonic
        if dt > 0.0 and self._last_phase_update_monotonic > 0.0:
            self._aoi_ms = max(self._aoi_ms, dt * 1000.0)

    def _mk_desired_cb(self, rn: str):
        def cb(msg: Twist):
            self._seq[rn] += 1
            seq = self._seq[rn]

            tx_mode = _pick_tx_mode(
                task_phase=self._task_phase,
                task_progress=self._task_progress,
                effective_freshness=self._effective_freshness,
                aoi_ms=self._aoi_ms,
                stale_indicator=self._stale_indicator,
            )
            exec_mode = _pick_exec_mode(
                task_phase=self._task_phase,
                effective_freshness=self._effective_freshness,
                aoi_ms=self._aoi_ms,
                stale_indicator=self._stale_indicator,
            )
            # Stress profile override: inject non-normal exec modes
            if self.force_exec_profile != "none":
                exec_mode = _stress_exec_mode(
                    task_phase=self._task_phase,
                    profile=self.force_exec_profile,
                    sweep_counter=self._sweep_counter[rn],
                )
                self._sweep_counter[rn] += 1

            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = _build_frame_id(
                seq=seq,
                tx=tx_mode,
                exec_mode=exec_mode,
                aoi=self._aoi_ms,
                eff=self._effective_freshness,
                phase=self._task_phase,
            )
            ts.twist.linear.x = float(msg.linear.x)
            ts.twist.linear.y = float(msg.linear.y)
            ts.twist.linear.z = float(msg.linear.z)
            ts.twist.angular.x = float(msg.angular.x)
            ts.twist.angular.y = float(msg.angular.y)
            ts.twist.angular.z = float(msg.angular.z)
            self._pubs[rn].publish(ts)
        return cb


def main():
    rclpy.init(args=None)
    node = P3DMissionAwareShadowBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
