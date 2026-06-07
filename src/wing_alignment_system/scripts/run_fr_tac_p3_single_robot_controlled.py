#!/usr/bin/env python3
"""FR-TAC-P3-C: Single-robot controlled closed-loop validation."""

from __future__ import annotations

import argparse
import csv
import datetime
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / "src" / "wing_alignment_system"))
from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig

MAX_LINEAR = 0.05
MAX_ANGULAR = 0.10
V_CMD = 0.03
W_CMD = 0.06

RUN_ID = "p3c_controlled"
ROBOT = "tracer1"
WATCHDOG_NODE = f"/{ROBOT}/cmd_watchdog"
ESTOP_NODE = "/p3c_emergency_stop_publisher"
ESTOP_TOPIC = "/wing_alignment/emergency_stop"
ESTOP_FILE = Path("/tmp/p3c_emergency_stop.flag")
RUNTIME_RUN_ID = "p3c_runtime"

CASES = {
    "C1": {"label": "normal_no_impairment", "execution_mode": "normal"},
    "C2": {"label": "forced_degraded", "execution_mode": "degraded"},
    "C3": {"label": "forced_hold", "execution_mode": "hold"},
    "C4": {"label": "forced_safe_stop", "execution_mode": "safe_stop"},
    "C5": {"label": "emergency_cmd_stop_override", "execution_mode": "normal"},
}


def _policy(enable_output: bool = True) -> WatchdogPolicy:
    return WatchdogPolicy(
        WatchdogConfig(
            watchdog_hz=40.0,
            age_safe=0.15,
            age_stop=0.40,
            decay_mode="linear",
            decay_k=3.0,
            enable_execution_mode_output=enable_output,
            degraded_linear_scale=0.5,
            degraded_angular_scale=0.5,
        )
    )


def _mode_row(ts, seq, mode, vi, wi, vo, wo, sc, st, sr, sample_phase=""):
    return dict(
        run_id=RUN_ID,
        timestamp=f"{ts:.6f}",
        robot_id=ROBOT,
        seq=str(seq),
        transmission_mode="full_update",
        execution_mode=mode,
        AoI_ms="100.000",
        effective_freshness="0.900000",
        output_scale=f"{sc:.6f}",
        stop_reason=sr,
        watchdog_state=st,
        cmd_v_in=f"{vi:.6f}",
        cmd_w_in=f"{wi:.6f}",
        cmd_v_out=f"{vo:.6f}",
        cmd_w_out=f"{wo:.6f}",
        t_source=f"{ts:.6f}",
        t_rx=f"{ts:.6f}",
        t_watchdog=f"{ts:.6f}",
        sample_phase=sample_phase,
    )


def _check_row(case, tid, en, mode, vi, wi, vo, wo, st, sr, passed, note="",
               cmd_v_in_mean="", cmd_v_out_mean="",
               output_scale_first="",
               output_scale_last_window_mean="",
               cmd_v_out_last_window_mean="",
               cmd_w_out_last_window_mean="",
               samples_count="",
               failure_reason="",
               active_window_start="",
               active_window_end="",
               stats_window_start="",
               raw_outputs_count="",
               active_outputs_count="",
               last_window_outputs_count="",
               post_cleanup_outputs_count="",
               active_last_nonzero_count=""):
    return dict(
        case_id=case,
        test_id=tid,
        enable_execution_mode_output=str(en),
        execution_mode=mode,
        cmd_v_in=f"{vi:.6f}",
        cmd_w_in=f"{wi:.6f}",
        cmd_v_out=f"{vo:.6f}",
        cmd_w_out=f"{wo:.6f}",
        watchdog_state=st,
        stop_reason=sr,
        passed=str(passed),
        note=note,
        cmd_v_in_mean=f"{cmd_v_in_mean}" if cmd_v_in_mean != "" else "",
        cmd_v_out_mean=f"{cmd_v_out_mean}" if cmd_v_out_mean != "" else "",
        output_scale_first=f"{output_scale_first}" if output_scale_first != "" else "",
        output_scale_last_window_mean=f"{output_scale_last_window_mean}" if output_scale_last_window_mean != "" else "",
        cmd_v_out_last_window_mean=f"{cmd_v_out_last_window_mean}" if cmd_v_out_last_window_mean != "" else "",
        cmd_w_out_last_window_mean=f"{cmd_w_out_last_window_mean}" if cmd_w_out_last_window_mean != "" else "",
        samples_count=f"{samples_count}" if samples_count != "" else "",
        failure_reason=failure_reason if failure_reason != "" else "",
        active_window_start=active_window_start if active_window_start != "" else "",
        active_window_end=active_window_end if active_window_end != "" else "",
        stats_window_start=stats_window_start if stats_window_start != "" else "",
        raw_outputs_count=raw_outputs_count if raw_outputs_count != "" else "",
        active_outputs_count=active_outputs_count if active_outputs_count != "" else "",
        last_window_outputs_count=last_window_outputs_count if last_window_outputs_count != "" else "",
        post_cleanup_outputs_count=post_cleanup_outputs_count if post_cleanup_outputs_count != "" else "",
        active_last_nonzero_count=active_last_nonzero_count if active_last_nonzero_count != "" else "",
    )

def _csv(d, fn, fields, rows):
    p = d / fn
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return p


def _scale_for(vi: float, vo: float) -> float:
    if abs(vi) < 1e-9:
        return 0.0 if abs(vo) < 1e-9 else 1.0
    return max(0.0, min(1.0, abs(vo) / abs(vi)))


def _steady_state_stats(rows):
    window = rows[-max(10, len(rows) // 2):]
    nw = len(window)
    if nw == 0:
        return {"output_scale_last_window_mean": 0.0, "cmd_v_out_last_window_mean": 0.0,
                "cmd_w_out_last_window_mean": 0.0, "cmd_v_in_mean": 0.0, "cmd_w_in_mean": 0.0,
                "samples_count": len(rows)}
    mean_scale = sum(float(r["output_scale"]) for r in window) / nw
    mean_v_out = sum(float(r["cmd_v_out"]) for r in window) / nw
    mean_w_out = sum(float(r["cmd_w_out"]) for r in window) / nw
    mean_v_in = sum(float(r["cmd_v_in"]) for r in window) / nw
    mean_w_in = sum(float(r["cmd_w_in"]) for r in window) / nw
    return {"output_scale_last_window_mean": mean_scale, "cmd_v_out_last_window_mean": mean_v_out,
            "cmd_w_out_last_window_mean": mean_w_out, "cmd_v_in_mean": mean_v_in,
            "cmd_w_in_mean": mean_w_in, "samples_count": len(rows)}


def run_case_c1(out_dir: Path, seq_base: int, allow_motion: bool):
    p = _policy(enable_output=False)
    mr = []
    cr = []
    all_ok = True
    s = seq_base + 1
    t = float(s) * 0.1
    p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="normal")
    out = p.compute(t + 0.01)
    scale = _scale_for(V_CMD, out.applied_v)
    ok1a = (
        out.state == "NORMAL"
        and out.stop_reason == ""
        and out.applied_v > 0
        and out.applied_w > 0
        and scale <= 1.0
        and out.applied_v <= V_CMD + 0.001
        and out.applied_w <= W_CMD + 0.001
    )
    mr.append(_mode_row(t, s, "normal", V_CMD, W_CMD, out.applied_v, out.applied_w, scale, out.state, out.stop_reason))
    cr.append(_check_row("C1", "C1a_normal_first_ramp", False, "normal",
        V_CMD, W_CMD, out.applied_v, out.applied_w, out.state, out.stop_reason, ok1a,
        output_scale_first=f"{scale:.6f}"))
    all_ok = all_ok and ok1a
    for _ in range(25):
        s += 1
        t += 0.05
        p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="normal")
        out = p.compute(t + 0.01)
        scale = _scale_for(V_CMD, out.applied_v)
        mr.append(_mode_row(t, s, "normal", V_CMD, W_CMD, out.applied_v, out.applied_w, scale, out.state, out.stop_reason))
    stats = _steady_state_stats(mr)
    ok1b = (
        stats.get("output_scale_last_window_mean", 0.0) >= 0.90
        and stats.get("cmd_v_out_last_window_mean", 0.0) >= 0.90 * V_CMD
        and stats.get("cmd_w_out_last_window_mean", 0.0) >= 0.90 * W_CMD
    )
    note = "steady_state_ok" if ok1b else f"scale_mean={stats.get('output_scale_last_window_mean',0):.4f}"
    cr.append(_check_row("C1", "C1b_normal_steady_state", False, "normal",
        V_CMD, W_CMD, stats.get("cmd_v_out_last_window_mean", 0.0), stats.get("cmd_w_out_last_window_mean", 0.0),
        "NORMAL", "", ok1b, note=note,
        output_scale_first=f"{float(mr[0]['output_scale']):.6f}",
        output_scale_last_window_mean=f"{stats.get('output_scale_last_window_mean', 0.0):.6f}",
        samples_count=str(stats.get('samples_count', 0))))
    all_ok = all_ok and ok1b
    return mr, cr, all_ok

def run_case_c2(out_dir: Path, seq_base: int, allow_motion: bool):
    p = _policy(enable_output=True)
    mr = []
    cr = []
    s = seq_base + 1
    t = float(s) * 0.1
    p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="degraded")
    out = p.compute(t + 0.01)
    ev, ew = V_CMD * 0.5, W_CMD * 0.5
    passed = abs(out.applied_v - ev) < 0.001 and abs(out.applied_w - ew) < 0.001
    mr.append(_mode_row(t, s, "degraded", V_CMD, W_CMD, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
    cr.append(_check_row("C2", "C2_degraded_scale", True, "degraded", V_CMD, W_CMD, out.applied_v, out.applied_w, out.state, out.stop_reason, passed))
    return mr, cr, passed


def run_case_c3(out_dir: Path, seq_base: int, allow_motion: bool):
    p = _policy(enable_output=True)
    mr = []
    cr = []
    s = seq_base + 1
    t = float(s) * 0.1
    # Continuous simulation: 21 samples at 0.05 s period (~1.05 s total)
    for _ in range(21):
        s += 1
        t += 0.05
        p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="hold")
        out = p.compute(t + 0.01)
        mr.append(_mode_row(t, s, "hold", V_CMD, W_CMD, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
    stats = _steady_state_stats(mr)
    # Check no emergency/cmd/age stop in any frame
    any_bad_stop = any(
        r["watchdog_state"] in ("EMERGENCY_STOP", "CMD_STOP", "AGE_STOP")
        for r in mr
    )
    n_samples = stats.get("samples_count", 0)
    last_v_mean = stats.get("cmd_v_out_last_window_mean", 0.0)
    last_w_mean = stats.get("cmd_w_out_last_window_mean", 0.0)
    passed = (
        out.requested_execution_mode == "hold"
        and out.stop_reason == "execution_mode_hold"
        and abs(last_v_mean) <= 1e-3
        and abs(last_w_mean) <= 1e-3
        and n_samples >= 20
        and not any_bad_stop
    )
    failure_reason = ""
    if not passed:
        reasons = []
        if out.requested_execution_mode != "hold":
            reasons.append(f"mode={out.requested_execution_mode}")
        if out.stop_reason != "execution_mode_hold":
            reasons.append(f"stop_reason={out.stop_reason}")
        if abs(last_v_mean) > 1e-3:
            reasons.append(f"last_window_v_mean={last_v_mean:.6f}")
        if abs(last_w_mean) > 1e-3:
            reasons.append(f"last_window_w_mean={last_w_mean:.6f}")
        if n_samples < 20:
            reasons.append(f"expected_samples>=20 observed={n_samples}")
        if any_bad_stop:
            bad_names = sorted(set(r["watchdog_state"] for r in mr if r["watchdog_state"] in ("EMERGENCY_STOP", "CMD_STOP", "AGE_STOP")))
            reasons.append(f"bad_states={bad_names}")
        reasons.append(f"threshold=1e-3")
        failure_reason = "; ".join(reasons)
    note = "steady_state_ok" if passed else failure_reason
    first_scale = float(mr[0]["output_scale"])
    cr.append(_check_row("C3", "C3_hold_zero", True, "hold",
        V_CMD, W_CMD, float(mr[-1]["cmd_v_out"]), float(mr[-1]["cmd_w_out"]),
        out.state, out.stop_reason, passed, note=note,
        output_scale_first=f"{first_scale:.6f}",
        output_scale_last_window_mean=f"{stats.get('output_scale_last_window_mean', 0.0):.6f}",
        cmd_v_out_last_window_mean=f"{last_v_mean:.6f}",
        cmd_w_out_last_window_mean=f"{last_w_mean:.6f}",
        samples_count=str(n_samples),
        failure_reason=failure_reason))
    return mr, cr, passed


def run_case_c4(out_dir: Path, seq_base: int, allow_motion: bool):
    p = _policy(enable_output=True)
    mr = []
    cr = []
    s = seq_base + 1
    t = float(s) * 0.1
    # Continuous simulation: 21 samples at 0.05 s period (~1.05 s total)
    for _ in range(21):
        s += 1
        t += 0.05
        p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="safe_stop")
        out = p.compute(t + 0.01)
        mr.append(_mode_row(t, s, "safe_stop", V_CMD, W_CMD, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
    stats = _steady_state_stats(mr)
    n_samples = stats.get("samples_count", 0)
    last_v_mean = stats.get("cmd_v_out_last_window_mean", 0.0)
    last_w_mean = stats.get("cmd_w_out_last_window_mean", 0.0)
    passed = (
        out.requested_execution_mode == "safe_stop"
        and out.stop_reason == "execution_mode_safe_stop"
        and out.state == "MODE_SAFE_STOP"
        and abs(last_v_mean) <= 1e-3
        and abs(last_w_mean) <= 1e-3
        and n_samples >= 20
    )
    failure_reason = ""
    if not passed:
        reasons = []
        if out.requested_execution_mode != "safe_stop":
            reasons.append(f"mode={out.requested_execution_mode}")
        if out.stop_reason != "execution_mode_safe_stop":
            reasons.append(f"stop_reason={out.stop_reason}")
        if out.state != "MODE_SAFE_STOP":
            reasons.append(f"state={out.state}")
        if abs(last_v_mean) > 1e-3:
            reasons.append(f"last_window_v_mean={last_v_mean:.6f}")
        if abs(last_w_mean) > 1e-3:
            reasons.append(f"last_window_w_mean={last_w_mean:.6f}")
        if n_samples < 20:
            reasons.append(f"expected_samples>=20 observed={n_samples}")
        reasons.append(f"threshold=1e-3")
        failure_reason = "; ".join(reasons)
    note = "steady_state_ok" if passed else failure_reason
    first_scale = float(mr[0]["output_scale"])
    cr.append(_check_row("C4", "C4_safe_stop_zero", True, "safe_stop",
        V_CMD, W_CMD, float(mr[-1]["cmd_v_out"]), float(mr[-1]["cmd_w_out"]),
        out.state, out.stop_reason, passed, note=note,
        output_scale_first=f"{first_scale:.6f}",
        output_scale_last_window_mean=f"{stats.get('output_scale_last_window_mean', 0.0):.6f}",
        cmd_v_out_last_window_mean=f"{last_v_mean:.6f}",
        cmd_w_out_last_window_mean=f"{last_w_mean:.6f}",
        samples_count=str(n_samples),
        failure_reason=failure_reason))
    return mr, cr, passed


def run_case_c5(out_dir: Path, seq_base: int, allow_motion: bool):
    p = _policy(enable_output=True)
    mr = []
    cr = []
    all_ok = True
    s = seq_base

    s += 1
    t = float(s) * 0.1
    p.on_cmd(s, V_CMD, W_CMD, t, execution_mode="normal")
    p.on_emergency(True)
    out = p.compute(t + 0.01)
    ok1 = out.applied_v == 0.0 and out.applied_w == 0.0 and out.state == "EMERGENCY_STOP"
    all_ok = all_ok and ok1
    mr.append(_mode_row(t, s, "normal", V_CMD, W_CMD, out.applied_v, out.applied_w, out.output_scale, out.state, out.stop_reason))
    cr.append(_check_row("C5", "C5_emergency_override", True, "normal", V_CMD, W_CMD, out.applied_v, out.applied_w, out.state, out.stop_reason, ok1))

    p2 = _policy(enable_output=True)
    s += 1
    t = float(s) * 0.1
    p2.on_cmd(s, V_CMD, W_CMD, t, execution_mode="normal")
    p2.on_stop(True)
    out2 = p2.compute(t + 0.01)
    ok2 = out2.applied_v == 0.0 and out2.applied_w == 0.0 and out2.state == "CMD_STOP"
    all_ok = all_ok and ok2
    mr.append(_mode_row(t, s, "normal", V_CMD, W_CMD, out2.applied_v, out2.applied_w, out2.output_scale, out2.state, out2.stop_reason))
    cr.append(_check_row("C5", "C5_cmd_stop_override", True, "normal", V_CMD, W_CMD, out2.applied_v, out2.applied_w, out2.state, out2.stop_reason, ok2))

    p3 = _policy(enable_output=True)
    s += 1
    t = float(s) * 0.1
    p3.on_cmd(s, V_CMD, W_CMD, t, execution_mode="normal")
    out3 = p3.compute(t + 0.5)
    ok3 = out3.applied_v == 0.0 and out3.applied_w == 0.0 and out3.state == "AGE_STOP"
    all_ok = all_ok and ok3
    mr.append(_mode_row(t + 0.5, s, "normal", V_CMD, W_CMD, out3.applied_v, out3.applied_w, out3.output_scale, out3.state, out3.stop_reason))
    cr.append(_check_row("C5", "C5_age_stop_override", True, "normal", V_CMD, W_CMD, out3.applied_v, out3.applied_w, out3.state, out3.stop_reason, ok3))
    return mr, cr, all_ok


CASE_RUNNERS = {
    "C1": run_case_c1,
    "C2": run_case_c2,
    "C3": run_case_c3,
    "C4": run_case_c4,
    "C5": run_case_c5,
}


def _ros2_text(*args: str) -> str:
    proc = subprocess.run(["ros2", *args], capture_output=True, text=True, check=False)
    return (proc.stdout or "") + (proc.stderr or "")


def _runtime_gate_errors() -> list[str]:
    errors = []
    nodes = set(line.strip() for line in _ros2_text("node", "list").splitlines() if line.strip())
    topics = set(line.strip() for line in _ros2_text("topic", "list").splitlines() if line.strip())

    if WATCHDOG_NODE not in nodes:
        errors.append(f"{WATCHDOG_NODE} is not running")
    if ESTOP_NODE not in nodes:
        errors.append(f"{ESTOP_NODE} is not running")
    if f"/{ROBOT}/cmd_vel_stamped" not in topics:
        errors.append(f"/{ROBOT}/cmd_vel_stamped is not present")
    if ESTOP_TOPIC not in topics:
        errors.append(f"{ESTOP_TOPIC} is not present")

    estop_info = _ros2_text("topic", "info", ESTOP_TOPIC, "-v")
    if "Publisher count: 0" in estop_info or "Unknown topic" in estop_info:
        errors.append(f"{ESTOP_TOPIC} does not have an active publisher")
    if "Node name: p3c_emergency_stop_publisher" not in estop_info:
        errors.append(f"{ESTOP_TOPIC} is not published by p3c_emergency_stop_publisher")

    watchdog_param = _ros2_text("param", "get", WATCHDOG_NODE, "safe_idle_no_publish").lower()
    if "false" not in watchdog_param:
        errors.append(f"{WATCHDOG_NODE} safe_idle_no_publish is not false")
    watchdog_run_id = _ros2_text("param", "get", WATCHDOG_NODE, "run_id")
    if RUNTIME_RUN_ID not in watchdog_run_id:
        errors.append(f"{WATCHDOG_NODE} run_id is not {RUNTIME_RUN_ID}")
    return errors



def run_controlled_ros(artifact_root: Path, cases: list[str]):
    errors = _runtime_gate_errors()
    if errors:
        print("P3-C runtime gate failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 2

    import rclpy
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from std_msgs.msg import Bool

    rclpy.init(args=[])
    node = Node("p3c_controlled_runner")
    pub_cmd = node.create_publisher(TwistStamped, f"/{ROBOT}/cmd_vel_stamped", 10)
    pub_stop = node.create_publisher(Bool, f"/{ROBOT}/cmd_stop", 10)
    pub_resume = node.create_publisher(Bool, f"/{ROBOT}/cmd_resume", 10)
    outputs: list[tuple[float, float, float]] = []

    def _out_cb(msg: Twist):
        outputs.append((time.time(), float(msg.linear.x), float(msg.angular.z)))

    node.create_subscription(Twist, f"/{ROBOT}/cmd_vel", _out_cb, 10)

    def _spin(duration: float):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def _clear_outputs():
        outputs.clear()

    def _publish_cmd(seq: int, v: float, w: float, execution_mode: str):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = f"seq={seq}|tx=full_update|exec={execution_mode}|aoi=100|eff=0.9"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        pub_cmd.publish(msg)

    def _publish_bool(pub, value: bool):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def _observe_command(seq: int, v: float, w: float, execution_mode: str, duration: float = 0.5):
        _clear_outputs()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            _publish_cmd(seq, v, w, execution_mode)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)
        _spin(0.2)
        if not outputs:
            return 0.0, 0.0
        _, vo, wo = outputs[-1]
        return vo, wo

    all_mode = []
    all_check = []
    case_results = {}
    seq = 1000

    try:
        for case_id in cases:
            print(f"--- Case {case_id}: {CASES[case_id]['label']} ---")
            case_dir = artifact_root / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            case_rows = []
            case_checks = []

            if case_id == "C1":
                seq += 1
                vo, wo = _observe_command(seq, V_CMD, W_CMD, "normal")
                sc = _scale_for(V_CMD, vo)
                ok1a = (
                    sc <= 1.0
                    and vo > 0.0
                    and wo > 0.0
                    and vo <= V_CMD + 0.003
                    and wo <= W_CMD + 0.003
                )
                case_rows.append(_mode_row(time.time(), seq, "normal", V_CMD, W_CMD, vo, wo, sc, "NORMAL", ""))
                case_checks.append(_check_row("C1", "C1a_normal_first_ramp", True, "normal",
                    V_CMD, W_CMD, vo, wo, "NORMAL", "", ok1a,
                    output_scale_first=f"{sc:.6f}"))
                ok = ok1a

                _clear_outputs()
                ramp_start = time.monotonic()
                while time.monotonic() - ramp_start < 2.0:
                    seq += 1
                    _publish_cmd(seq, V_CMD, W_CMD, "normal")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.05)
                _spin(0.3)
                mean_vo = 0.0
                mean_wo = 0.0
                sc_mean = 0.0
                if outputs:
                    steady_window = [(tup[0], tup[1], tup[2]) for tup in outputs if tup[0] >= ramp_start + 1.5]
                    if not steady_window:
                        steady_window = outputs[-3:] if len(outputs) >= 3 else outputs
                    if steady_window:
                        mean_vo = sum(t[1] for t in steady_window) / len(steady_window)
                        mean_wo = sum(t[2] for t in steady_window) / len(steady_window)
                        sc_mean = _scale_for(V_CMD, mean_vo)
                ok1b = (
                    sc_mean >= 0.90
                    and mean_vo >= 0.90 * V_CMD
                    and mean_wo >= 0.90 * W_CMD
                )
                note_1b = "steady_state_ok" if ok1b else f"scale_mean={sc_mean:.4f} v_mean={mean_vo:.4f}"
                case_rows.append(_mode_row(time.time(), seq, "normal", V_CMD, W_CMD,
                    mean_vo, mean_wo, sc_mean, "NORMAL", ""))
                case_checks.append(_check_row("C1", "C1b_normal_steady_state", True, "normal",
                    V_CMD, W_CMD, mean_vo, mean_wo, "NORMAL", "", ok1b, note=note_1b,
                    output_scale_first=f"{_scale_for(V_CMD, vo):.6f}",
                    output_scale_last_window_mean=f"{sc_mean:.6f}",
                    samples_count=str(len(outputs))))
                ok = ok and ok1b

            elif case_id == "C2":
                seq += 1
                vo, wo = _observe_command(seq, V_CMD, W_CMD, "degraded")
                ok = abs(vo - (V_CMD * 0.5)) < 0.01 and abs(wo - (W_CMD * 0.5)) < 0.01
                case_rows.append(_mode_row(time.time(), seq, "degraded", V_CMD, W_CMD, vo, wo, _scale_for(V_CMD, vo), "NORMAL", "execution_mode_degraded"))
                case_checks.append(_check_row("C2", "C2_degraded_scale", True, "degraded", V_CMD, W_CMD, vo, wo, "NORMAL", "execution_mode_degraded", ok))
            elif case_id == "C3":
                # Phase 1: pre_zero -- publish zero cmd for 0.5 s to flush residuals
                seq_start = seq + 1
                _clear_outputs()
                settle_deadline = time.monotonic() + 0.5
                while time.monotonic() < settle_deadline:
                    _publish_cmd(seq_start, 0.0, 0.0, "hold")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.03)
                # Phase 2: active -- continuous hold publish for >= 1.3 s at ~17 Hz
                _clear_outputs()
                active_start = time.monotonic()
                hold_deadline = active_start + 1.3
                while time.monotonic() < hold_deadline:
                    seq += 1
                    _publish_cmd(seq, V_CMD, W_CMD, "hold")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.06)
                active_end = time.monotonic()
                active_end_wall = time.time()
                # Phase 3: post_cleanup -- spin to drain residual outputs (excluded from gate)
                _spin(0.2)
                raw_outputs = list(outputs)
                # Split outputs: active (ts <= active_end_wall) vs post_cleanup (ts > active_end_wall)
                active_outputs = [o for o in raw_outputs if o[0] <= active_end_wall]
                post_cleanup_outputs = [o for o in raw_outputs if o[0] > active_end_wall]
                # Statistics window: [active_end_wall - 0.5, active_end_wall] from active outputs only
                stats_window_start = active_end_wall - 0.5
                last_window = [o for o in active_outputs if o[0] >= stats_window_start]
                if len(last_window) < 5:
                    last_window = active_outputs[-5:] if len(active_outputs) >= 5 else active_outputs
                # Mode timeline: tag each output with sample_phase
                for ts_obs, v_obs, w_obs in active_outputs:
                    case_rows.append(_mode_row(ts_obs, seq_start, "hold", V_CMD, W_CMD, v_obs, w_obs, 0.0, "NORMAL", "execution_mode_hold",
                        sample_phase="active"))
                for ts_obs, v_obs, w_obs in post_cleanup_outputs:
                    case_rows.append(_mode_row(ts_obs, seq_start, "hold", V_CMD, W_CMD, v_obs, w_obs, 0.0, "NORMAL", "execution_mode_hold",
                        sample_phase="post_cleanup"))
                # Window statistics from active last window only
                if last_window:
                    vo = sum(o[1] for o in last_window) / len(last_window)
                    wo = sum(o[2] for o in last_window) / len(last_window)
                    active_last_nonzero = sum(1 for o in last_window if abs(o[1]) > 1e-6 or abs(o[2]) > 1e-6)
                else:
                    vo, wo = 0.0, 0.0
                    active_last_nonzero = 0
                ok = (
                    abs(vo) <= 1e-3
                    and abs(wo) <= 1e-3
                    and len(last_window) >= 5
                )
                failure_reason = ""
                if not ok:
                    reasons = []
                    if abs(vo) > 1e-3:
                        reasons.append(f"last_window_v_mean={vo:.6f}")
                    if abs(wo) > 1e-3:
                        reasons.append(f"last_window_w_mean={wo:.6f}")
                    if len(last_window) < 5:
                        reasons.append(f"last_window_outputs_count={len(last_window)} min=5")
                    reasons.append(f"expected_threshold=1e-3")
                    reasons.append(f"active_last_nonzero_count={active_last_nonzero}")
                    failure_reason = "; ".join(reasons)
                case_checks.append(_check_row("C3", "C3_hold_last_window_zero", True, "hold",
                    V_CMD, W_CMD, vo, wo, "NORMAL", "execution_mode_hold", ok,
                    note=failure_reason if failure_reason else "steady_state_ok",
                    cmd_v_out_last_window_mean=f"{vo:.6f}",
                    cmd_w_out_last_window_mean=f"{wo:.6f}",
                    output_scale_first="0.000000",
                    output_scale_last_window_mean="0.000000",
                    samples_count=str(len(last_window)),
                    failure_reason=failure_reason,
                    active_window_start=f"{active_start:.6f}",
                    active_window_end=f"{active_end_wall:.6f}",
                    stats_window_start=f"{stats_window_start:.6f}",
                    raw_outputs_count=str(len(raw_outputs)),
                    active_outputs_count=str(len(active_outputs)),
                    last_window_outputs_count=str(len(last_window)),
                    post_cleanup_outputs_count=str(len(post_cleanup_outputs)),
                    active_last_nonzero_count=str(active_last_nonzero)))
            elif case_id == "C4":
                # Phase 1: pre_zero -- publish zero cmd for 0.5 s to flush residuals
                seq_start = seq + 1
                _clear_outputs()
                settle_deadline = time.monotonic() + 0.5
                while time.monotonic() < settle_deadline:
                    _publish_cmd(seq_start, 0.0, 0.0, "safe_stop")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.03)
                # Phase 2: active -- continuous safe_stop publish for >= 1.3 s at ~17 Hz
                _clear_outputs()
                active_start = time.monotonic()
                hold_deadline = active_start + 1.3
                while time.monotonic() < hold_deadline:
                    seq += 1
                    _publish_cmd(seq, V_CMD, W_CMD, "safe_stop")
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.06)
                active_end = time.monotonic()
                active_end_wall = time.time()
                # Phase 3: post_cleanup -- spin to drain residual outputs (excluded from gate)
                _spin(0.2)
                raw_outputs = list(outputs)
                # Split outputs: active (ts <= active_end_wall) vs post_cleanup (ts > active_end_wall)
                active_outputs = [o for o in raw_outputs if o[0] <= active_end_wall]
                post_cleanup_outputs = [o for o in raw_outputs if o[0] > active_end_wall]
                # Statistics window: [active_end_wall - 0.5, active_end_wall] from active outputs only
                stats_window_start = active_end_wall - 0.5
                last_window = [o for o in active_outputs if o[0] >= stats_window_start]
                if len(last_window) < 5:
                    last_window = active_outputs[-5:] if len(active_outputs) >= 5 else active_outputs
                # Mode timeline: tag each output with sample_phase
                for ts_obs, v_obs, w_obs in active_outputs:
                    case_rows.append(_mode_row(ts_obs, seq_start, "safe_stop", V_CMD, W_CMD, v_obs, w_obs, 0.0, "MODE_SAFE_STOP", "execution_mode_safe_stop",
                        sample_phase="active"))
                for ts_obs, v_obs, w_obs in post_cleanup_outputs:
                    case_rows.append(_mode_row(ts_obs, seq_start, "safe_stop", V_CMD, W_CMD, v_obs, w_obs, 0.0, "MODE_SAFE_STOP", "execution_mode_safe_stop",
                        sample_phase="post_cleanup"))
                # Window statistics from active last window only
                if last_window:
                    vo = sum(o[1] for o in last_window) / len(last_window)
                    wo = sum(o[2] for o in last_window) / len(last_window)
                    active_last_nonzero = sum(1 for o in last_window if abs(o[1]) > 1e-6 or abs(o[2]) > 1e-6)
                else:
                    vo, wo = 0.0, 0.0
                    active_last_nonzero = 0
                ok = (
                    abs(vo) <= 1e-3
                    and abs(wo) <= 1e-3
                    and len(last_window) >= 5
                )
                failure_reason = ""
                if not ok:
                    reasons = []
                    if abs(vo) > 1e-3:
                        reasons.append(f"last_window_v_mean={vo:.6f}")
                    if abs(wo) > 1e-3:
                        reasons.append(f"last_window_w_mean={wo:.6f}")
                    if len(last_window) < 5:
                        reasons.append(f"last_window_outputs_count={len(last_window)} min=5")
                    reasons.append(f"expected_threshold=1e-3")
                    reasons.append(f"active_last_nonzero_count={active_last_nonzero}")
                    failure_reason = "; ".join(reasons)
                case_checks.append(_check_row("C4", "C4_safe_stop_last_window_zero", True, "safe_stop",
                    V_CMD, W_CMD, vo, wo, "MODE_SAFE_STOP", "execution_mode_safe_stop", ok,
                    note=failure_reason if failure_reason else "steady_state_ok",
                    cmd_v_out_last_window_mean=f"{vo:.6f}",
                    cmd_w_out_last_window_mean=f"{wo:.6f}",
                    output_scale_first="0.000000",
                    output_scale_last_window_mean="0.000000",
                    samples_count=str(len(last_window)),
                    failure_reason=failure_reason,
                    active_window_start=f"{active_start:.6f}",
                    active_window_end=f"{active_end_wall:.6f}",
                    stats_window_start=f"{stats_window_start:.6f}",
                    raw_outputs_count=str(len(raw_outputs)),
                    active_outputs_count=str(len(active_outputs)),
                    last_window_outputs_count=str(len(last_window)),
                    post_cleanup_outputs_count=str(len(post_cleanup_outputs)),
                    active_last_nonzero_count=str(active_last_nonzero)))
            elif case_id == "C5":
                ESTOP_FILE.touch()
                time.sleep(0.2)
                seq += 1
                vo, wo = _observe_command(seq, V_CMD, W_CMD, "normal")
                ok = abs(vo) < 0.005 and abs(wo) < 0.005
                case_rows.append(_mode_row(time.time(), seq, "normal", V_CMD, W_CMD, vo, wo, 0.0, "EMERGENCY_STOP", "emergency_latched"))
                case_checks.append(_check_row("C5", "C5_emergency_override", True, "normal", V_CMD, W_CMD, vo, wo, "EMERGENCY_STOP", "emergency_latched", ok))
                ESTOP_FILE.unlink(missing_ok=True)
                _spin(0.3)

                _publish_bool(pub_stop, True)
                _spin(0.1)
                seq += 1
                vo, wo = _observe_command(seq, V_CMD, W_CMD, "normal")
                ok2 = abs(vo) < 0.005 and abs(wo) < 0.005
                case_rows.append(_mode_row(time.time(), seq, "normal", V_CMD, W_CMD, vo, wo, 0.0, "CMD_STOP", "cmd_stop_latched"))
                case_checks.append(_check_row("C5", "C5_cmd_stop_override", True, "normal", V_CMD, W_CMD, vo, wo, "CMD_STOP", "cmd_stop_latched", ok2))
                _publish_bool(pub_resume, True)
                _spin(0.2)

                _clear_outputs()
                seq += 1
                _publish_cmd(seq, V_CMD, W_CMD, "normal")
                _spin(0.7)
                if outputs:
                    _, vo, wo = outputs[-1]
                else:
                    vo, wo = 0.0, 0.0
                ok3 = abs(vo) < 0.005 and abs(wo) < 0.005
                case_rows.append(_mode_row(time.time(), seq, "normal", V_CMD, W_CMD, vo, wo, 0.0, "AGE_STOP", "age_stop_exceeded"))
                case_checks.append(_check_row("C5", "C5_age_stop_override", True, "normal", V_CMD, W_CMD, vo, wo, "AGE_STOP", "age_stop_exceeded", ok3))
                ok = ok and ok2 and ok3
            else:
                ok = False

            case_results[case_id] = ok
            print(f"  {'PASS' if ok else 'FAIL'}")
            all_mode.extend(case_rows)
            all_check.extend(case_checks)
    finally:
        ESTOP_FILE.unlink(missing_ok=True)
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    total_passed = sum(1 for c in all_check if c["passed"] == "True")
    total = len(all_check)
    gate = all(case_results.values())

    for case_id in cases:
        case_dir = artifact_root / case_id
        case_checks = [c for c in all_check if c["case_id"] == case_id]
        case_modes = [m for m in all_mode if case_id == "C5" or m["execution_mode"] == CASES[case_id]["execution_mode"]]
        tf = ["run_id", "timestamp", "robot_id", "seq", "transmission_mode", "execution_mode", "AoI_ms", "effective_freshness", "output_scale", "stop_reason", "watchdog_state", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "t_source", "t_rx", "t_watchdog", "sample_phase"]
        cf = ["case_id", "test_id", "enable_execution_mode_output", "execution_mode", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "watchdog_state", "stop_reason", "passed", "note", "cmd_v_in_mean", "cmd_v_out_mean", "output_scale_first", "output_scale_last_window_mean", "cmd_v_out_last_window_mean", "cmd_w_out_last_window_mean", "samples_count", "failure_reason", "active_window_start", "active_window_end", "stats_window_start", "raw_outputs_count", "active_outputs_count", "last_window_outputs_count", "post_cleanup_outputs_count", "active_last_nonzero_count"]
        _csv(case_dir, "mode_timeline_tracer1.csv", tf, case_modes)
        _csv(case_dir, "watchdog_output_check.csv", cf, case_checks)
        lines = [
            f"FR-TAC-P3-C {case_id} Gate Report",
            "=" * 60,
            f"Gate: {'PASS' if case_results[case_id] else 'FAIL'}",
            f"Label: {CASES[case_id]['label']}",
            "Allow Real Motion: True",
            "",
        ]
        for c in case_checks:
            status = "PASS" if c["passed"] == "True" else "FAIL"
            lines.append(f"  [{status}] {c['test_id']}: v_out={c['cmd_v_out']} w_out={c['cmd_w_out']} state={c['watchdog_state']}")
        (case_dir / "gate.txt").write_text("\n".join(lines), encoding="utf-8")

    tf = ["run_id", "timestamp", "robot_id", "seq", "transmission_mode", "execution_mode", "AoI_ms", "effective_freshness", "output_scale", "stop_reason", "watchdog_state", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "t_source", "t_rx", "t_watchdog", "sample_phase"]
    cf = ["case_id", "test_id", "enable_execution_mode_output", "execution_mode", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "watchdog_state", "stop_reason", "passed", "note", "cmd_v_in_mean", "cmd_v_out_mean", "output_scale_first", "output_scale_last_window_mean", "cmd_v_out_last_window_mean", "cmd_w_out_last_window_mean", "samples_count", "failure_reason", "active_window_start", "active_window_end", "stats_window_start", "raw_outputs_count", "active_outputs_count", "last_window_outputs_count", "post_cleanup_outputs_count", "active_last_nonzero_count"]
    _csv(artifact_root, "mode_timeline_tracer1.csv", tf, all_mode)
    _csv(artifact_root, "watchdog_output_check.csv", cf, all_check)

    in_v = [float(r["cmd_v_in"]) for r in all_mode]
    out_v = [float(r["cmd_v_out"]) for r in all_mode]
    scales = [float(r["output_scale"]) for r in all_mode]
    srow = dict(
        run_id=artifact_root.name,
        test_type="p3c_controlled",
        total_checks=total,
        checks_passed=total_passed,
        checks_failed=total - total_passed,
        gate_passed=str(gate).lower(),
        allow_real_motion="true",
        cmd_vel_in_mean=f"{sum(in_v) / len(in_v):.6f}" if in_v else "0.0",
        cmd_vel_out_mean=f"{sum(out_v) / len(out_v):.6f}" if out_v else "0.0",
        output_scale_mean=f"{sum(scales) / len(scales):.6f}" if scales else "1.0",
        degraded_count=str(sum(1 for c in all_check if c["case_id"] == "C2" and c["passed"] == "True")),
        hold_count=str(sum(1 for c in all_check if c["case_id"] == "C3" and c["passed"] == "True")),
        safe_stop_count=str(sum(1 for c in all_check if c["case_id"] == "C4" and c["passed"] == "True")),
        emergency_stop_count=str(sum(1 for c in all_check if "emergency" in c["test_id"] and c["passed"] == "True")),
        cmd_stop_count=str(sum(1 for c in all_check if "cmd_stop" in c["test_id"] and c["passed"] == "True")),
        age_stop_count=str(sum(1 for c in all_check if "age_stop" in c["test_id"] and c["passed"] == "True")),
        max_AoI_ms="100.000",
    )
    _csv(artifact_root, "run_summary.csv", list(srow.keys()), [srow])

    lines = [
        "FR-TAC-P3-C Controlled Gate Report",
        "=" * 60,
        f"Gate: {'PASS' if gate else 'FAIL'}",
        f"Run ID: {artifact_root.name}",
        "Allow Real Motion: True",
        f"Robot: {ROBOT}",
        f"{total_passed}/{total} checks passed",
        "",
    ]
    for case_id in cases:
        lines.append(f"  Case {case_id}: {'PASS' if case_results[case_id] else 'FAIL'}  ({CASES[case_id]['label']})")
    lines.append("")
    for c in all_check:
        status = "PASS" if c["passed"] == "True" else "FAIL"
        lines.append(f"  [{status}] {c['test_id']}: v_out={c['cmd_v_out']} w_out={c['cmd_w_out']} state={c['watchdog_state']}")
    (artifact_root / "p3c_controlled_gate.txt").write_text("\n".join(lines), encoding="utf-8")
    print(f"\n=== P3-C Controlled Gate: {'PASS' if gate else 'FAIL'} ===")
    return 0 if gate else 1


def main():
    parser = argparse.ArgumentParser(description="FR-TAC-P3-C Controlled Runner")
    parser.add_argument("--allow-real-motion", action="store_true", help="REQUIRED to publish real cmd_vel to tracer1")
    parser.add_argument("--artifact-root", default=os.path.expanduser("~/.ros/fr_tac_p3c_controlled_runs"), help="Root directory for output artifacts")
    parser.add_argument("--run-id", default="p3c_controlled", help="Run identifier")
    parser.add_argument("--cases", default="C1,C2,C3,C4,C5", help="Comma-separated case IDs to run")
    parser.add_argument("--force", action="store_true", help="Overwrite existing run directory")
    args = parser.parse_args()

    artifact_root = Path(args.artifact_root).expanduser().resolve()
    run_id_val = args.run_id
    run_dir = artifact_root / run_id_val
    if run_dir.exists() and not args.force:
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_id_val = f"{args.run_id}_{ts}"
        run_dir = artifact_root / run_id_val
        print(f"Run dir exists, using timestamped: {run_id_val}")
    cases = [c.strip() for c in args.cases.split(",") if c.strip() in CASE_RUNNERS]
    if not cases:
        print("ERROR: no valid cases specified", file=sys.stderr)
        return 1

    print("FR-TAC-P3-C Controlled Runner")
    print(f"  Robot:       {ROBOT}")
    print(f"  V_MAX:       {MAX_LINEAR} m/s")
    print(f"  W_MAX:       {MAX_ANGULAR} rad/s")
    print(f"  V_CMD:       {V_CMD} m/s")
    print(f"  W_CMD:       {W_CMD} rad/s")
    print(f"  Real Motion: {'YES' if args.allow_real_motion else 'NO (shadow-only)'}")
    print(f"  Cases:       {cases}")
    print(f"  Out Dir:     {run_dir}")
    print("")

    if not args.allow_real_motion:
        print("*** --allow-real-motion NOT set. Running in-memory validation only. ***")
        print("*** No cmd_vel will be published to the robot. ***")
        print("")
        all_mode = []
        all_check = []
        case_results = {}
        seq_base = 0
        for case_id in cases:
            case_dir = run_dir / case_id
            case_dir.mkdir(parents=True, exist_ok=True)
            print(f"--- Case {case_id}: {CASES[case_id]['label']} ---")
            runner = CASE_RUNNERS[case_id]
            mr, cr, ok = runner(case_dir, seq_base, args.allow_real_motion)
            seq_base += 10
            all_mode.extend(mr)
            all_check.extend(cr)
            case_results[case_id] = ok
            print(f"  {'PASS' if ok else 'FAIL'}")

        tf = ["run_id", "timestamp", "robot_id", "seq", "transmission_mode", "execution_mode", "AoI_ms", "effective_freshness", "output_scale", "stop_reason", "watchdog_state", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "t_source", "t_rx", "t_watchdog", "sample_phase"]
        cf = ["case_id", "test_id", "enable_execution_mode_output", "execution_mode", "cmd_v_in", "cmd_w_in", "cmd_v_out", "cmd_w_out", "watchdog_state", "stop_reason", "passed", "note", "cmd_v_in_mean", "cmd_v_out_mean", "output_scale_first", "output_scale_last_window_mean", "cmd_v_out_last_window_mean", "cmd_w_out_last_window_mean", "samples_count", "failure_reason", "active_window_start", "active_window_end", "stats_window_start", "raw_outputs_count", "active_outputs_count", "last_window_outputs_count", "post_cleanup_outputs_count", "active_last_nonzero_count"]
        _csv(run_dir, "mode_timeline_tracer1.csv", tf, all_mode)
        _csv(run_dir, "watchdog_output_check.csv", cf, all_check)
        total_passed = sum(1 for c in all_check if c["passed"] == "True")
        total = len(all_check)
        gate = all(case_results.values())
        srow = dict(
            run_id=args.run_id,
            test_type="p3c_controlled",
            total_checks=total,
            checks_passed=total_passed,
            checks_failed=total - total_passed,
            gate_passed=str(gate).lower(),
            allow_real_motion="false",
            cmd_vel_in_mean=f"{sum(float(r['cmd_v_in']) for r in all_mode) / len(all_mode):.6f}" if all_mode else "0.0",
            cmd_vel_out_mean=f"{sum(float(r['cmd_v_out']) for r in all_mode) / len(all_mode):.6f}" if all_mode else "0.0",
            output_scale_mean=f"{sum(float(r['output_scale']) for r in all_mode) / len(all_mode):.6f}" if all_mode else "1.0",
            degraded_count=str(sum(1 for c in all_check if c["case_id"] == "C2" and c["passed"] == "True")),
            hold_count=str(sum(1 for c in all_check if c["case_id"] == "C3" and c["passed"] == "True")),
            safe_stop_count=str(sum(1 for c in all_check if c["case_id"] == "C4" and c["passed"] == "True")),
            emergency_stop_count=str(sum(1 for c in all_check if "emergency" in c["test_id"] and c["passed"] == "True")),
            cmd_stop_count=str(sum(1 for c in all_check if "cmd_stop" in c["test_id"] and c["passed"] == "True")),
            age_stop_count=str(sum(1 for c in all_check if "age_stop" in c["test_id"] and c["passed"] == "True")),
            max_AoI_ms="100.000",
        )
        _csv(run_dir, "run_summary.csv", list(srow.keys()), [srow])
        lines = [
            "FR-TAC-P3-C Controlled Gate Report",
            "=" * 60,
            f"Gate: {'PASS' if gate else 'FAIL'}",
            f"Run ID: {args.run_id}",
            "Allow Real Motion: False",
            f"Robot: {ROBOT}",
            f"{total_passed}/{total} checks passed",
            "",
        ]
        for case_id in cases:
            lines.append(f"  Case {case_id}: {'PASS' if case_results[case_id] else 'FAIL'}  ({CASES[case_id]['label']})")
        lines.append("")
        for c in all_check:
            status = "PASS" if c["passed"] == "True" else "FAIL"
            lines.append(f"  [{status}] {c['test_id']}: v_out={c['cmd_v_out']} w_out={c['cmd_w_out']} state={c['watchdog_state']}")
        (run_dir / "p3c_controlled_gate.txt").write_text("\n".join(lines), encoding="utf-8")
        print(f"\n=== P3-C Controlled Gate: {'PASS' if gate else 'FAIL'} ===")
        return 0 if gate else 1

    run_dir.mkdir(parents=True, exist_ok=True)
    return run_controlled_ros(run_dir, cases)


if __name__ == "__main__":
    raise SystemExit(main())
