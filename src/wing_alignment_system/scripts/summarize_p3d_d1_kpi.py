#!/usr/bin/env python3
"""
FR-TAC-P3-D1 Degraded-Only Controlled KPI Summarizer
=====================================================
Reads mode_timeline CSVs per robot, generates run_summary.csv,
phasewise_summary.csv, and p3d_d1_gate.txt.

D1-specific KPIs:
  - cmd_vel_in/out ratio
  - degraded_time_ratio
  - output_scale_mean
  - safety_override_count
  - emergency_stop_count
  - cmd_stop_count
  - final cmd_vel zero confirmation

Usage:
  python3 summarize_p3d_d1_kpi.py --log-dir <log_dir> --robots tracer1,tracer2,tracer3 [--run-id ID]
"""
from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path


def _safe_float(val, default=0.0):
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def read_mode_timeline(csv_path: Path) -> list[dict]:
    rows = []
    if not csv_path.exists():
        return rows
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)
    return rows


def _extract_phase_from_frame_id(frame_id: str) -> str:
    if not frame_id:
        return "unknown"
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return parts.get("phase", "unknown")


def compute_robot_kpi(rows: list[dict], robot_id: str) -> dict:
    """Compute P3-D1 specific KPIs for a single robot."""
    if not rows:
        return {"robot_id": robot_id, "n_samples": 0}

    n = len(rows)
    first_ts = _safe_float(rows[0].get("timestamp"), 0)
    last_ts = _safe_float(rows[-1].get("timestamp"), 0)
    duration_s = max(0.0, last_ts - first_ts)

    exec_modes = defaultdict(int)
    phases = defaultdict(int)
    wd_states = defaultdict(int)
    stop_reasons = defaultdict(int)

    cmd_v_in_sum = 0.0
    cmd_v_out_sum = 0.0
    cmd_w_in_sum = 0.0
    cmd_w_out_sum = 0.0
    output_scale_sum = 0.0
    degraded_samples = 0
    normal_samples = 0
    hold_samples = 0
    safe_stop_samples = 0
    emergency_stop_count = 0
    cmd_stop_count = 0
    age_stop_count = 0
    safety_override_count = 0
    nonzero_v_out = 0

    last_v_out = 0.0
    last_w_out = 0.0

    for r in rows:
        frame_id = r.get("frame_id", "")
        if frame_id and "phase=" in frame_id:
            phase = _extract_phase_from_frame_id(frame_id)
        else:
            phase = r.get("task_phase", r.get("phase", ""))

        exec_mode = r.get("execution_mode", "unknown")
        state = r.get("watchdog_state", "unknown")
        sr = r.get("stop_reason", "")

        v_in = _safe_float(r.get("cmd_v_in", 0))
        v_out = _safe_float(r.get("cmd_v_out", 0))
        w_in = _safe_float(r.get("cmd_w_in", 0))
        w_out = _safe_float(r.get("cmd_w_out", 0))
        scale = _safe_float(r.get("output_scale", 1.0))

        cmd_v_in_sum += v_in
        cmd_v_out_sum += v_out
        cmd_w_in_sum += w_in
        cmd_w_out_sum += w_out
        output_scale_sum += scale

        if phase:
            phases[phase] += 1
        exec_modes[exec_mode] += 1
        wd_states[state] += 1

        if exec_mode == "degraded":
            degraded_samples += 1
        elif exec_mode == "normal":
            normal_samples += 1
        elif exec_mode == "hold":
            hold_samples += 1
        elif exec_mode == "safe_stop":
            safe_stop_samples += 1

        if state == "EMERGENCY_STOP":
            emergency_stop_count += 1
        elif state == "CMD_STOP":
            cmd_stop_count += 1
        elif state == "AGE_STOP":
            age_stop_count += 1

        if sr:
            stop_reasons[sr] += 1

        if state not in ("NORMAL", "DECAY"):
            safety_override_count += 1

        if abs(v_out) > 1e-6:
            nonzero_v_out += 1

        last_v_out = v_out
        last_w_out = w_out

    # Derived KPIs
    cmd_vel_in_ratio = (cmd_v_out_sum / cmd_v_in_sum) if cmd_v_in_sum > 1e-9 else 1.0
    degraded_time_ratio = degraded_samples / n if n > 0 else 0.0
    output_scale_mean = output_scale_sum / n if n > 0 else 1.0
    nonzero_ratio = nonzero_v_out / n if n > 0 else 0.0
    final_cmd_vel_zero = abs(last_v_out) < 1e-6 and abs(last_w_out) < 1e-6

    # D1-specific: verify no hold/safe_stop
    has_forbidden_modes = hold_samples > 0 or safe_stop_samples > 0

    return {
        "robot_id": robot_id,
        "n_samples": n,
        "duration_s": f"{duration_s:.3f}",
        "execution_mode_dist": dict(exec_modes),
        "phase_dist": dict(phases),
        "watchdog_state_dist": dict(wd_states),
        "stop_reason_dist": dict(stop_reasons),
        "cmd_v_in_mean": f"{cmd_v_in_sum / n:.6f}" if n > 0 else "0.0",
        "cmd_v_out_mean": f"{cmd_v_out_sum / n:.6f}" if n > 0 else "0.0",
        "cmd_w_in_mean": f"{cmd_w_in_sum / n:.6f}" if n > 0 else "0.0",
        "cmd_w_out_mean": f"{cmd_w_out_sum / n:.6f}" if n > 0 else "0.0",
        "cmd_vel_in_out_ratio": f"{cmd_vel_in_ratio:.6f}",
        "degraded_samples": degraded_samples,
        "normal_samples": normal_samples,
        "hold_samples": hold_samples,
        "safe_stop_samples": safe_stop_samples,
        "degraded_time_ratio": f"{degraded_time_ratio:.6f}",
        "output_scale_mean": f"{output_scale_mean:.6f}",
        "nonzero_v_out_ratio": f"{nonzero_ratio:.6f}",
        "emergency_stop_count": emergency_stop_count,
        "cmd_stop_count": cmd_stop_count,
        "age_stop_count": age_stop_count,
        "safety_override_count": safety_override_count,
        "final_cmd_vel_zero": str(final_cmd_vel_zero),
        "has_forbidden_modes": str(has_forbidden_modes),
        "last_cmd_v_out": f"{last_v_out:.6f}",
        "last_cmd_w_out": f"{last_w_out:.6f}",
    }


def _compute_phasewise(rows: list[dict], robot_id: str) -> list[dict]:
    """Compute per-phase KPIs from mode_timeline rows."""
    if not rows:
        return []

    phase_buckets = defaultdict(list)
    for r in rows:
        frame_id = r.get("frame_id", "")
        if frame_id and "phase=" in frame_id:
            phase = _extract_phase_from_frame_id(frame_id)
        else:
            phase = r.get("task_phase", r.get("phase", "unknown"))
        phase_buckets[phase].append(r)

    result = []
    for phase, bucket in sorted(phase_buckets.items()):
        if not bucket:
            continue
        n = len(bucket)
        first_ts = _safe_float(bucket[0].get("timestamp"), 0)
        last_ts = _safe_float(bucket[-1].get("timestamp"), 0)
        dur = max(0.0, last_ts - first_ts)

        exec_modes = defaultdict(int)
        v_in_sum = 0.0
        v_out_sum = 0.0
        output_scale_sum = 0.0
        degraded_n = 0
        safety_n = 0

        for r in bucket:
            exec_mode = r.get("execution_mode", "unknown")
            exec_modes[exec_mode] += 1
            v_in_sum += _safe_float(r.get("cmd_v_in", 0))
            v_out_sum += _safe_float(r.get("cmd_v_out", 0))
            output_scale_sum += _safe_float(r.get("output_scale", 1.0))
            if exec_mode == "degraded":
                degraded_n += 1
            state = r.get("watchdog_state", "")
            if state not in ("NORMAL", "DECAY", ""):
                safety_n += 1

        result.append({
            "robot_id": robot_id,
            "phase": phase,
            "phase_duration_s": f"{dur:.3f}",
            "n_samples": n,
            "degraded_samples": degraded_n,
            "degraded_ratio": f"{degraded_n / n:.6f}" if n > 0 else "0.0",
            "output_scale_mean": f"{output_scale_sum / n:.6f}" if n > 0 else "1.0",
            "cmd_v_in_mean": f"{v_in_sum / n:.6f}" if n > 0 else "0.0",
            "cmd_v_out_mean": f"{v_out_sum / n:.6f}" if n > 0 else "0.0",
            "cmd_vel_in_out_ratio": f"{v_out_sum / v_in_sum:.6f}" if v_in_sum > 1e-9 else "1.0",
            "execution_mode_dist": dict(exec_modes),
            "safety_override_count": safety_n,
        })

    return result


def generate_gate(run_dir: Path, robot_kpis: dict, robots: list[str]) -> str:
    """Generate P3-D1 gate report."""
    lines = []
    lines.append("=" * 60)
    lines.append("FR-TAC-P3-D1 Degraded-Only Controlled Gate Report")
    lines.append("=" * 60)
    lines.append(f"Timestamp: {run_dir}")
    lines.append("")

    errors = []
    warnings = []

    for rid in robots:
        kpi = robot_kpis.get(rid, {})
        if not kpi or kpi.get("n_samples", 0) == 0:
            errors.append(f"{rid}: no samples (mode_timeline empty or missing)")
            # --- Enhanced no-samples diagnostics (Task A) ---
            lines.append(f"--- {rid} (NO SAMPLES DIAGNOSTIC) ---")
            lines.append(f"  n_samples: 0")
            mt_path = run_dir / f"mode_timeline_{rid}.csv"
            if mt_path.exists():
                lines.append(f"  mode_timeline file: exists (but empty or header-only)")
            else:
                lines.append(f"  mode_timeline file: MISSING at {mt_path}")
            rts_path = run_dir / "recorder_topic_status.csv"
            if rts_path.exists():
                import csv as _csv
                key_topics = {
                    f"/{rid}/cmd_vel_desired": "cmd_vel_desired",
                    f"/{rid}/cmd_vel_stamped": "cmd_vel_stamped",
                    f"/{rid}/cmd_vel": "cmd_vel",
                }
                topic_rows = {}
                try:
                    with rts_path.open("r", encoding="utf-8", newline="") as fh:
                        reader = _csv.DictReader(fh)
                        for row in reader:
                            t = row.get("topic", "")
                            if t in key_topics:
                                topic_rows[t] = row
                except Exception:
                    pass
                lines.append(f"  recorder_topic_status.csv sample counts:")
                for tpath, tlabel in key_topics.items():
                    tr = topic_rows.get(tpath, {})
                    observed = tr.get("observed", "N/A")
                    row_count = tr.get("row_count", "N/A")
                    lines.append(f"    {tlabel}: observed={observed}, row_count={row_count}")
                all_zero = all(
                    topic_rows.get(tp, {}).get("row_count", "N/A") in ("0", 0, "N/A")
                    for tp in key_topics
                )
                if all_zero:
                    lines.append(f"  ROOT CAUSE: All cmd_vel topics have zero samples in recorder.")
                    lines.append(f"    cmd_vel_desired not published -> bridge has nothing to forward.")
                    lines.append(f"    cmd_vel_stamped never generated -> cmd_watchdog never records.")
                    lines.append(f"    mode_timeline cannot be produced without cmd_watchdog input.")
            else:
                lines.append(f"  recorder_topic_status.csv: MISSING at {rts_path}")
            lines.append("")
            continue

        lines.append(f"--- {rid} ---")
        lines.append(f"  Samples:  {kpi['n_samples']}")
        lines.append(f"  Duration: {kpi['duration_s']}s")
        lines.append(f"  Exec modes: {kpi['execution_mode_dist']}")
        lines.append(f"  Phases: {kpi['phase_dist']}")
        lines.append(f"  Watchdog states: {kpi['watchdog_state_dist']}")
        lines.append(f"  cmd_vel in/out ratio: {kpi['cmd_vel_in_out_ratio']}")
        lines.append(f"  degraded_time_ratio: {kpi['degraded_time_ratio']}")
        lines.append(f"  output_scale_mean: {kpi['output_scale_mean']}")
        lines.append(f"  emergency_stop_count: {kpi['emergency_stop_count']}")
        lines.append(f"  cmd_stop_count: {kpi['cmd_stop_count']}")
        lines.append(f"  age_stop_count: {kpi['age_stop_count']}")
        lines.append(f"  safety_override_count: {kpi['safety_override_count']}")
        lines.append(f"  final cmd_vel: v={kpi['last_cmd_v_out']} w={kpi['last_cmd_w_out']}")
        lines.append(f"  final cmd_vel zero: {kpi['final_cmd_vel_zero']}")
        lines.append("")

        # Gate checks
        # D1-C1: No hold/safe_stop in execution_mode distribution
        if kpi.get("has_forbidden_modes") == "True":
            errors.append(f"{rid}: forbidden modes detected (hold={kpi['hold_samples']}, safe_stop={kpi['safe_stop_samples']})")
        else:
            lines.append(f"  [PASS] No hold/safe_stop in execution_mode distribution for {rid}")

        # D1-C2: degraded_time_ratio should be > 0 (some degradation present)
        dtr = _safe_float(kpi.get("degraded_time_ratio", "0"))
        if dtr <= 0.0:
            warnings.append(f"{rid}: degraded_time_ratio is zero (all normal)")
        else:
            lines.append(f"  [PASS] degraded_time_ratio={kpi['degraded_time_ratio']} > 0")

        # D1-C3: output_scale_mean should be <= 1.0
        osm = _safe_float(kpi.get("output_scale_mean", "1.0"))
        if osm > 1.0:
            errors.append(f"{rid}: output_scale_mean={kpi['output_scale_mean']} exceeds 1.0")
        lines.append(f"  [PASS] output_scale_mean={kpi['output_scale_mean']} <= 1.0")

        # D1-C4: cmd_vel in/out ratio should be <= 1.0 (degraded scales down)
        cvr = _safe_float(kpi.get("cmd_vel_in_out_ratio", "1.0"))
        if cvr > 1.0:
            warnings.append(f"{rid}: cmd_vel_in_out_ratio={kpi['cmd_vel_in_out_ratio']} > 1.0 (output exceeds input)")

        # D1-C5: final cmd_vel must be zero
        if kpi.get("final_cmd_vel_zero") != "True":
            errors.append(f"{rid}: final cmd_vel is non-zero (v={kpi['last_cmd_v_out']}, w={kpi['last_cmd_w_out']})")
        else:
            lines.append(f"  [PASS] final cmd_vel zero confirmed")

        # D1-C6: emergency_stop_count MUST be 0 (pre-real readiness)
        if int(kpi.get("emergency_stop_count", 0)) > 0:
            errors.append(f"{rid}: emergency_stop_count={kpi['emergency_stop_count']} (emergency stops detected)")
        else:
            lines.append(f"  [PASS] emergency_stop_count=0")

        # D1-C7: cmd_stop_count MUST be 0 (pre-real readiness)
        if int(kpi.get("cmd_stop_count", 0)) > 0:
            errors.append(f"{rid}: cmd_stop_count={kpi['cmd_stop_count']} (cmd stops detected)")
        else:
            lines.append(f"  [PASS] cmd_stop_count=0")

        # D1-C8: safety_override_count MUST be 0 (pre-real readiness)
        soc = int(kpi.get("safety_override_count", 0))
        if soc > 0:
            errors.append(f"{rid}: safety_override_count={soc} (safety overrides detected)")
        else:
            lines.append(f"  [PASS] safety_override_count=0")

        # D1-C9: no CMD_STOP watchdog state (pre-real readiness)
        wd_dist = kpi.get("watchdog_state_dist", {})
        if isinstance(wd_dist, str):
            import ast
            try: wd_dist = ast.literal_eval(wd_dist)
            except: wd_dist = {}
        if wd_dist.get("CMD_STOP", 0) > 0:
            errors.append(f"{rid}: CMD_STOP watchdog state detected ({wd_dist})")

    lines.append("")
    n_err = len(errors)
    n_warn = len(warnings)
    lines.append(f"Gate errors:   {n_err}")
    lines.append(f"Gate warnings: {n_warn}")

    if n_err > 0:
        lines.append("GATE: FAIL")
        for e in errors:
            lines.append(f"  ERROR: {e}")
        # Task B: Factual infrastructure note when no cmd_watchdog samples exist
        no_sample_errors = [e for e in errors if "no samples" in e]
        if no_sample_errors:
            lines.append("")
            lines.append("  NOTE: Real-motion gate infrastructure (G11/G14a/G14b) may have passed,")
            lines.append("  but real telemetry evidence is unavailable because no cmd_watchdog")
            lines.append("  samples were generated. The message flow")
            lines.append("  (cmd_vel_desired -> bridge -> cmd_vel_stamped -> watchdog -> mode_timeline)")
            lines.append("  was never activated. See the NO SAMPLES DIAGNOSTIC section above.")
    else:
        lines.append("GATE: PASS")
    for w in warnings:
        lines.append(f"  WARN: {w}")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="FR-TAC-P3-D1 Degraded-Only Controlled KPI Summarizer",
    )
    parser.add_argument("--log-dir", required=True, help="Log directory with mode_timeline CSVs")
    parser.add_argument("--robots", default="tracer1,tracer2,tracer3",
                        help="Comma-separated robot IDs")
    parser.add_argument("--run-id", default="p3d_d1_controlled", help="Run identifier")

    args = parser.parse_args()
    run_dir = Path(args.log_dir).expanduser().resolve()
    if not run_dir.is_dir():
        print(f"ERROR: directory not found: {run_dir}", file=sys.stderr)
        return 2

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    run_id = args.run_id

    # Read per-robot mode timelines
    robot_kpis = {}
    all_phasewise = []
    for rid in robots:
        mt_path = run_dir / f"mode_timeline_{rid}.csv"
        rows = read_mode_timeline(mt_path)
        kpi = compute_robot_kpi(rows, rid)
        robot_kpis[rid] = kpi
        pw = _compute_phasewise(rows, rid)
        all_phasewise.extend(pw)

    # --- Write run_summary.csv ---
    sum_path = run_dir / "run_summary.csv"
    sum_fields = [
        "robot_id", "n_samples", "duration_s",
        "execution_mode_dist", "phase_dist", "watchdog_state_dist",
        "stop_reason_dist",
        "cmd_v_in_mean", "cmd_v_out_mean",
        "cmd_w_in_mean", "cmd_w_out_mean",
        "cmd_vel_in_out_ratio",
        "degraded_samples", "normal_samples",
        "hold_samples", "safe_stop_samples",
        "degraded_time_ratio",
        "output_scale_mean",
        "nonzero_v_out_ratio",
        "emergency_stop_count", "cmd_stop_count", "age_stop_count",
        "safety_override_count",
        "final_cmd_vel_zero",
        "has_forbidden_modes",
        "last_cmd_v_out", "last_cmd_w_out",
    ]
    with sum_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=sum_fields)
        writer.writeheader()
        for rid in robots:
            kpi = robot_kpis.get(rid, {})
            writer.writerow({k: str(kpi.get(k, "")) for k in sum_fields})

    # --- Write phasewise_summary.csv ---
    pw_path = run_dir / "phasewise_summary.csv"
    pw_fields = [
        "robot_id", "phase", "phase_duration_s",
        "n_samples", "degraded_samples", "degraded_ratio",
        "output_scale_mean",
        "cmd_v_in_mean", "cmd_v_out_mean",
        "cmd_vel_in_out_ratio",
        "execution_mode_dist",
        "safety_override_count",
    ]
    with pw_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=pw_fields)
        writer.writeheader()
        for row in all_phasewise:
            writer.writerow(row)
    if not all_phasewise:
        with pw_path.open("a", encoding="utf-8") as fh:
            fh.write("\n# No phasewise data (all standby or no phase_source)\n")

    # --- Write gate file ---
    gate_content = generate_gate(run_dir, robot_kpis, robots)
    gate_path = run_dir / "p3d_d1_gate.txt"
    gate_path.write_text(gate_content, encoding="utf-8")

    # --- Print summary ---
    print(f"run_summary.csv:           {sum_path}")
    print(f"phasewise_summary.csv:     {pw_path}")
    print(f"p3d_d1_gate.txt:           {gate_path}")
    for rid in robots:
        kpi = robot_kpis.get(rid, {})
        n = kpi.get("n_samples", 0)
        dur = kpi.get("duration_s", "0")
        cvr = kpi.get("cmd_vel_in_out_ratio", "0")
        dtr = kpi.get("degraded_time_ratio", "0")
        osm = kpi.get("output_scale_mean", "1.0")
        exec_dist = kpi.get("execution_mode_dist", {})
        phase_dist = kpi.get("phase_dist", {})
        print(f"  {rid}: {n} samples, {dur}s, "
              f"cv_ratio={cvr}, deg_ratio={dtr}, "
              f"out_scale={osm}")
        if exec_dist:
            print(f"    exec_modes: {exec_dist}")
        if phase_dist:
            print(f"    phases: {phase_dist}")

    if gate_content and "GATE: PASS" in gate_content:
        print("\nGate: PASS")
        return 0
    else:
        print("\nGate: FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
