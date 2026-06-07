#!/usr/bin/env python3
"""
FR-TAC-P3-D0 Shadow KPI Summarizer
===================================
Reads mode_timeline CSVs per robot and mission_runtime_events CSV,
generates run_summary.csv, phasewise_summary.csv, p3d_shadow_gate.txt.

P3-D0b adds: phase-aware gate checks (phase not all standby,
execution_mode not all normal, tx_mode not all shadow_bridge,
AoI/eff not all fixed placeholder).

Usage:
  python3 summarize_p3d_shadow_kpi.py <log_dir> [--robots tracer1,tracer2,tracer3]
  python3 summarize_p3d_shadow_kpi.py --log-dir <log_dir> [--robots ...] [--mission-aware-shadow]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from glob import glob
from pathlib import Path


def _safe_float(val, default=0.0):
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _safe_int(val, default=0):
    try:
        return int(val)
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


def read_mission_events(csv_path: Path) -> list[dict]:
    rows = []
    if not csv_path.exists():
        return rows
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append(row)
    return rows


def _extract_phase_from_frame_id(frame_id: str) -> str:
    """Extract phase from frame_id metadata string like 'seq=1|tx_mode=full|...|phase=transport'"""
    if not frame_id:
        return "unknown"
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return parts.get("phase", "unknown")


def _extract_exec_mode_from_frame_id(frame_id: str) -> str:
    if not frame_id:
        return "unknown"
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return parts.get("exec_mode", "unknown")


def _extract_tx_mode_from_frame_id(frame_id: str) -> str:
    if not frame_id:
        return "unknown"
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return parts.get("tx_mode", parts.get("tx", "unknown"))


def _extract_aoi_from_frame_id(frame_id: str) -> float:
    if not frame_id:
        return 0.0
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return _safe_float(parts.get("aoi_ms", parts.get("aoi", "0.0")))


def _extract_eff_from_frame_id(frame_id: str) -> float:
    if not frame_id:
        return 0.0
    parts = dict(p.split("=", 1) for p in str(frame_id).split("|") if "=" in p)
    return _safe_float(parts.get("effective_freshness", parts.get("eff", "1.0")))


def compute_robot_kpi(rows: list[dict], robot_id: str, mission_aware: bool = False) -> dict:
    if not rows:
        return {}

    aoi_vals = []
    eff_vals = []
    exec_modes = defaultdict(int)
    tx_modes = defaultdict(int)
    phases = defaultdict(int)
    stop_reasons = defaultdict(int)
    wd_states = defaultdict(int)

    first_ts = _safe_float(rows[0].get("timestamp"), 0)
    last_ts = _safe_float(rows[-1].get("timestamp"), 0)
    duration = max(0, last_ts - first_ts)

    tx_count = 0
    payload_bytes = 0
    payload_map = {
        "skip_update": 0, "skip": 0,
        "compact_update": 128, "compact": 128,
        "full_update": 256, "full": 256,
        "urgent_refresh": 320, "urgent": 320,
        "shadow_bridge": 256,
    }

    for r in rows:
        # Try to get phase/exec/tx from frame_id if present (P3-D0b)
        frame_id = r.get("frame_id", "")
        if mission_aware and frame_id and "phase=" in frame_id:
            phase = _extract_phase_from_frame_id(frame_id)
            exec_mode = _extract_exec_mode_from_frame_id(frame_id)
            tx_mode = _extract_tx_mode_from_frame_id(frame_id)
            aoi_val = _extract_aoi_from_frame_id(frame_id)
            eff_val = _extract_eff_from_frame_id(frame_id)
        else:
            phase = r.get("task_phase", r.get("phase", ""))
            exec_mode = r.get("execution_mode", "unknown")
            tx_mode = r.get("transmission_mode", "unknown")
            aoi_val = _safe_float(r.get("AoI_ms", r.get("aoi_ms", 0)))
            eff_val = _safe_float(r.get("effective_freshness", r.get("eff", 1.0)))

        if phase:
            phases[phase] += 1
        aoi_vals.append(aoi_val)
        eff_vals.append(eff_val)
        exec_modes[exec_mode] += 1
        tx_modes[tx_mode] += 1
        if tx_mode not in ("unknown", "skip_update", "skip"):
            tx_count += 1
            payload_bytes += payload_map.get(tx_mode, 256)
        sr = r.get("stop_reason", "") or ""
        if sr:
            stop_reasons[sr] += 1
        wd_states[r.get("watchdog_state", "unknown")] += 1

    n = len(rows)
    avg_aoi = sum(aoi_vals) / len(aoi_vals) if aoi_vals else 0.0
    max_aoi = max(aoi_vals) if aoi_vals else 0.0
    avg_eff = sum(eff_vals) / len(eff_vals) if eff_vals else 0.0

    emergency_count = wd_states.get("EMERGENCY_STOP", 0)
    cmd_stop_count = wd_states.get("CMD_STOP", 0)
    age_stop_count = wd_states.get("AGE_STOP", 0)
    safe_stop_count = wd_states.get("MODE_SAFE_STOP", 0)
    safety_override_count = emergency_count + cmd_stop_count + age_stop_count

    output_passthrough = True
    mismatches = 0
    for r in rows:
        vi = _safe_float(r.get("cmd_v_in"), 0)
        vo = _safe_float(r.get("cmd_v_out"), 0)
        state = r.get("watchdog_state", "")
        if state in ("EMERGENCY_STOP", "CMD_STOP", "AGE_STOP", "MODE_SAFE_STOP"):
            if abs(vo) > 0.001:
                mismatches += 1
        elif abs(vi - vo) > 0.001:
            mismatches += 1

    output_ok = mismatches == 0

    return {
        "robot_id": robot_id,
        "phase_duration_s": round(duration, 3),
        "n_samples": n,
        "tx_count": tx_count,
        "payload_bytes": payload_bytes,
        "avg_AoI_ms": round(avg_aoi, 3),
        "max_AoI_ms": round(max_aoi, 3),
        "avg_effective_freshness": round(avg_eff, 6),
        "execution_mode_dist": dict(exec_modes),
        "transmission_mode_dist": dict(tx_modes),
        "phase_dist": dict(phases),
        "stop_reason_dist": dict(stop_reasons),
        "watchdog_state_dist": dict(wd_states),
        "emergency_stop_count": emergency_count,
        "cmd_stop_count": cmd_stop_count,
        "age_stop_count": age_stop_count,
        "safe_stop_count": safe_stop_count,
        "safety_override_count": safety_override_count,
        "output_passthrough_ok": output_ok,
        "output_mismatches": mismatches,
        "aoi_all_fixed_placeholder": _is_fixed_placeholder(aoi_vals, 0.0),
        "eff_all_fixed_placeholder": _is_fixed_placeholder(eff_vals, 1.0),
    }


def _is_fixed_placeholder(vals: list[float], placeholder: float) -> bool:
    """True if ALL values equal the placeholder (or list is empty)."""
    if not vals:
        return True
    return all(abs(v - placeholder) < 0.001 for v in vals)


def _compute_phasewise_from_mode_timelines(run_dir, robots):
    """Build per-robot per-phase KPI from mode_timeline CSVs.

    Groups mode_timeline rows by (robot_id, phase), computes
    timestamp-based phase_duration_s, and all per-group KPIs.
    Returns list of dicts ready for CSV writer.
    """
    rows_out = []
    payload_map = {
        "skip_update": 0, "skip": 0,
        "compact_update": 128, "compact": 128,
        "full_update": 256, "full": 256,
        "urgent_refresh": 320, "urgent": 320,
        "shadow_bridge": 256,
    }

    for rid in robots:
        mt_path = run_dir / f"mode_timeline_{rid}.csv"
        mt_rows = read_mode_timeline(mt_path)
        if not mt_rows:
            continue

        # Group rows by phase
        phase_groups = defaultdict(list)
        for r in mt_rows:
            frame_id = r.get("frame_id", "")
            if frame_id and "phase=" in frame_id:
                phase = _extract_phase_from_frame_id(frame_id)
            else:
                phase = r.get("phase", r.get("task_phase", "unknown"))
            phase_groups[phase].append(r)

        for phase, p_rows in sorted(phase_groups.items()):
            timestamps = []
            aoi_vals = []
            eff_vals = []
            exec_modes = defaultdict(int)
            tx_modes = defaultdict(int)
            wd_states = defaultdict(int)
            tx_count = 0
            payload_bytes = 0
            mismatches = 0

            for r in p_rows:
                ts = _safe_float(r.get("timestamp"), 0)
                if ts > 0:
                    timestamps.append(ts)

                frame_id = r.get("frame_id", "")
                if frame_id and "exec=" in frame_id:
                    exec_mode = _extract_exec_mode_from_frame_id(frame_id)
                    tx_mode = _extract_tx_mode_from_frame_id(frame_id)
                    aoi_val = _extract_aoi_from_frame_id(frame_id)
                    eff_val = _extract_eff_from_frame_id(frame_id)
                else:
                    exec_mode = r.get("execution_mode", "unknown")
                    tx_mode = r.get("transmission_mode", "unknown")
                    aoi_val = _safe_float(r.get("AoI_ms", 0))
                    eff_val = _safe_float(r.get("effective_freshness", 1.0))

                exec_modes[exec_mode] += 1
                tx_modes[tx_mode] += 1
                aoi_vals.append(aoi_val)
                eff_vals.append(eff_val)

                if tx_mode not in ("unknown", "skip_update", "skip"):
                    tx_count += 1
                    payload_bytes += payload_map.get(tx_mode, 256)

                wd_state = r.get("watchdog_state", "unknown")
                wd_states[wd_state] += 1

                vi = _safe_float(r.get("cmd_v_in"), 0)
                vo = _safe_float(r.get("cmd_v_out"), 0)
                if wd_state in ("EMERGENCY_STOP", "CMD_STOP", "AGE_STOP", "MODE_SAFE_STOP"):
                    if abs(vo) > 0.001:
                        mismatches += 1
                elif abs(vi - vo) > 0.001:
                    mismatches += 1

            n = len(p_rows)
            duration = round(max(0, max(timestamps) - min(timestamps)), 3) if timestamps else 0.0
            avg_aoi = round(sum(aoi_vals) / len(aoi_vals), 3) if aoi_vals else 0.0
            max_aoi = round(max(aoi_vals), 3) if aoi_vals else 0.0
            avg_eff = round(sum(eff_vals) / len(eff_vals), 6) if eff_vals else 0.0

            emergency_count = wd_states.get("EMERGENCY_STOP", 0)
            cmd_stop_count = wd_states.get("CMD_STOP", 0)
            age_stop_count = wd_states.get("AGE_STOP", 0)
            safety_override = emergency_count + cmd_stop_count + age_stop_count

            rows_out.append({
                "robot_id": rid,
                "phase": phase,
                "phase_duration_s": duration,
                "n_samples": n,
                "tx_count": tx_count,
                "payload_bytes": payload_bytes,
                "avg_AoI_ms": avg_aoi,
                "max_AoI_ms": max_aoi,
                "avg_effective_freshness": avg_eff,
                "execution_mode_dist": str(dict(exec_modes)),
                "transmission_mode_dist": str(dict(tx_modes)),
                "safety_override_count": safety_override,
                "output_passthrough_ok": str(mismatches == 0),
            })

    return rows_out


def generate_gate(run_dir: Path, robot_kpis: dict, robots: list[str], mission_aware: bool = False) -> str:
    lines = []
    lines.append("=== P3-D0 Shadow Gate ===")
    lines.append(f"Run Dir: {run_dir}")
    lines.append(f"Mode: {'P3-D0b mission-aware' if mission_aware else 'P3-D0a basic bridge'}")

    errors = []
    warnings = []

    for rid in robots:
        kpi = robot_kpis.get(rid, {})
        n = kpi.get("n_samples", 0)
        ok = kpi.get("output_passthrough_ok", False)
        mismatches = kpi.get("output_mismatches", 0)
        exec_dist = kpi.get("execution_mode_dist", {})
        tx_dist = kpi.get("transmission_mode_dist", {})
        phase_dist = kpi.get("phase_dist", {})
        aoi_fixed = kpi.get("aoi_all_fixed_placeholder", True)
        eff_fixed = kpi.get("eff_all_fixed_placeholder", True)
        avg_aoi = kpi.get("avg_AoI_ms", 0)
        avg_eff = kpi.get("avg_effective_freshness", 0)

        lines.append(f"\n--- {rid} ---")
        lines.append(f"  n_samples: {n}")
        lines.append(f"  output_passthrough_ok: {ok} (mismatches={mismatches})")
        lines.append(f"  execution_mode_dist: {exec_dist}")
        lines.append(f"  transmission_mode_dist: {tx_dist}")
        lines.append(f"  phase_dist: {phase_dist}")
        lines.append(f"  avg_AoI_ms: {avg_aoi}, aoi_fixed_placeholder: {aoi_fixed}")
        lines.append(f"  avg_effective_freshness: {avg_eff}, eff_fixed_placeholder: {eff_fixed}")

        # P3-D0b specific gates
        if mission_aware:
            # Gate: phase not all standby
            if phase_dist and len(phase_dist) <= 1 and "standby" in phase_dist:
                warnings.append(f"{rid}: phase_dist is all standby, phase_source may be replay")
            elif not phase_dist:
                warnings.append(f"{rid}: no phase data in mode_timeline")

            # Gate: execution_mode not all normal
            if exec_dist and len(exec_dist) == 1 and "normal" in exec_dist:
                warnings.append(f"{rid}: execution_mode_dist is all normal (may be baseline B0/B1)")

            # Gate: transmission_mode not all shadow_bridge
            if tx_dist and len(tx_dist) == 1 and "shadow_bridge" in tx_dist:
                warnings.append(f"{rid}: transmission_mode_dist is all shadow_bridge (bridge-only baseline)")

            # Gate: AoI/eff not all fixed placeholder
            if aoi_fixed:
                warnings.append(f"{rid}: AoI values are all fixed placeholder (0.0)")
            if eff_fixed:
                warnings.append(f"{rid}: effective_freshness values are all fixed placeholder (1.0)")

        # Always: output passthrough must be ok
        if not ok:
            errors.append(f"{rid}: output_passthrough_ok=False (mismatches={mismatches})")

        # Always: must have samples
        if n == 0:
            errors.append(f"{rid}: no samples in mode_timeline")

    # Emergency flag check (Task B): shadow mode = auto-clean + WARN
    # real-motion mode check is handled by run script
    emergency_flag_path = "/tmp/p3c_emergency_stop.flag"
    import os as _os
    if _os.path.exists(emergency_flag_path):
        warnings.append(f"emergency_stop_flag still present at {emergency_flag_path}")
        lines.append(f"  WARN: emergency_stop.flag found (should be cleaned in shadow mode)")
    else:
        lines.append("  [PASS] emergency_stop.flag not present")

    lines.append("")
    lines.append(f"Gate errors:   {len(errors)}")
    lines.append(f"Gate warnings: {len(warnings)}")

    for e in errors:
        lines.append(f"  ERROR: {e}")
    for w in warnings:
        lines.append(f"  WARN: {w}")

    if errors:
        lines.append("\nGate: FAIL")
    else:
        lines.append("\nGate: PASS")

    if mission_aware:
        lines.append("\n[P3-D0b] phase_source=replay (deterministic timeline, no real mission_coordinator dependency)")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="FR-TAC-P3-D0 Shadow KPI Summarizer",
        add_help=False,
    )
    parser.add_argument("log_dir", nargs="?", default=None, help="Log directory with mode_timeline CSVs")
    parser.add_argument("--log-dir", dest="log_dir_flag", default=None, help="Log directory (named arg)")
    parser.add_argument("--robots", dest="robots", default="tracer1,tracer2,tracer3", help="Comma-separated robot IDs")
    parser.add_argument("--mission-aware-shadow", dest="mission_aware", action="store_true", default=False,
                        help="Enable P3-D0b mission-aware gate checks")
    parser.add_argument("--help", "-h", action="store_true", help="Show help")

    if "--help" in sys.argv or "-h" in sys.argv:
        parser.print_help()
        return 0

    args = parser.parse_args()

    log_dir = args.log_dir or args.log_dir_flag
    if not log_dir:
        parser.print_help()
        print("\nERROR: log directory required (positional or --log-dir)", file=sys.stderr)
        return 2

    run_dir = Path(log_dir).expanduser().resolve()
    if not run_dir.is_dir():
        print(f"ERROR: directory not found: {run_dir}", file=sys.stderr)
        return 2

    robots = [r.strip() for r in args.robots.split(",") if r.strip()]
    mission_aware = args.mission_aware

    # Read per-robot mode timelines
    robot_kpis = {}
    for rid in robots:
        mt_path = run_dir / f"mode_timeline_{rid}.csv"
        rows = read_mode_timeline(mt_path)
        kpi = compute_robot_kpi(rows, rid, mission_aware=mission_aware)
        robot_kpis[rid] = kpi

    # Build phasewise from mode_timeline CSVs (per-robot per-phase)
    phasewise_rows = _compute_phasewise_from_mode_timelines(run_dir, robots)

    # --- Write run_summary.csv ---
    sum_path = run_dir / "run_summary.csv"
    sum_fields = [
        "robot_id", "phase_duration_s", "n_samples",
        "tx_count", "payload_bytes",
        "avg_AoI_ms", "max_AoI_ms", "avg_effective_freshness",
        "emergency_stop_count", "cmd_stop_count", "age_stop_count",
        "safe_stop_count", "safety_override_count",
        "output_passthrough_ok", "output_mismatches",
        "execution_mode_dist", "transmission_mode_dist",
        "phase_dist",
        "stop_reason_dist", "watchdog_state_dist",
        "aoi_all_fixed_placeholder", "eff_all_fixed_placeholder",
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
        "n_samples", "tx_count", "payload_bytes",
        "avg_AoI_ms", "max_AoI_ms", "avg_effective_freshness",
        "execution_mode_dist", "transmission_mode_dist",
        "safety_override_count", "output_passthrough_ok",
    ]
    with pw_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=pw_fields)
        writer.writeheader()
        phase_source_note = ""
        if phasewise_rows:
            for row in phasewise_rows:
                writer.writerow(row)
        else:
            phase_source_note = "phase_source=unavailable"

    if phase_source_note:
        with pw_path.open("a", encoding="utf-8") as fh:
            fh.write(f"\n# {phase_source_note}\n")

    # --- Write gate file ---
    gate_content = generate_gate(run_dir, robot_kpis, robots, mission_aware=mission_aware)
    gate_path = run_dir / "p3d_shadow_gate.txt"
    gate_path.write_text(gate_content, encoding="utf-8")

    # --- Print summary ---
    print(f"run_summary.csv: {sum_path}")
    print(f"phasewise_summary.csv: {pw_path}")
    print(f"p3d_shadow_gate.txt: {gate_path}")
    for rid in robots:
        kpi = robot_kpis.get(rid, {})
        n = kpi.get("n_samples", 0)
        dur = kpi.get("phase_duration_s", 0)
        avg_aoi = kpi.get("avg_AoI_ms", 0)
        tx_dist = kpi.get("transmission_mode_dist", {})
        exec_dist = kpi.get("execution_mode_dist", {})
        phase_dist = kpi.get("phase_dist", {})
        print(f"  {rid}: {n} samples, {dur}s, avg_AoI={avg_aoi}ms, output_ok={kpi.get('output_passthrough_ok', False)}")
        if tx_dist:
            print(f"    tx_modes: {tx_dist}")
        if exec_dist:
            print(f"    exec_modes: {exec_dist}")
        if phase_dist:
            print(f"    phases: {phase_dist}")

    if gate_content and "Gate: PASS" in gate_content:
        print("\nGate: PASS")
        return 0
    else:
        print("\nGate: FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
