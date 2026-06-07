#!/usr/bin/env python3
"""Offline communication policy replay over mode_timeline CSV.

VoI_proxy is logging-only task-value proxy, not a validated physical VoI model.
"""

import argparse
import csv
import math
import os
import sys
from collections import Counter


PHASE_WEIGHTS = {
    "standby": 0.2,
    "approach": 1.0,
    "transport": 1.2,
    "align": 1.5,
    "slide_align": 1.5,
    "docking": 2.0,
    "final_align": 2.0,
}

FRESHNESS_TAU_MS = 1000.0
HIGH_DELTA_THRESHOLD = 0.005
HIGH_VOI_THRESHOLD = 0.001


def safe_float(value):
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return None


def percentile(values, pct):
    if not values:
        return None
    s = sorted(values)
    idx = int(math.ceil(pct / 100.0 * len(s))) - 1
    return s[max(0, min(idx, len(s) - 1))]


def load_mode_timeline(path):
    records = []
    if not os.path.isfile(path):
        return records
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            return records
        col = {name.strip(): i for i, name in enumerate(header)}
        for raw in reader:
            if len(raw) < 19:
                continue
            ts = safe_float(raw[col.get("t_source", 16)]) or 0.0
            tr = safe_float(raw[col.get("t_rx", 17)]) or 0.0
            tw = safe_float(raw[col.get("t_watchdog", 18)]) or 0.0
            aoi = safe_float(raw[col.get("AoI_ms", 6)])
            vi = safe_float(raw[col.get("cmd_v_in", 12)]) or 0.0
            wi = safe_float(raw[col.get("cmd_w_in", 13)]) or 0.0
            ef = safe_float(raw[col.get("effective_freshness", 7)]) or 1.0
            ph = raw[col.get("phase", 8)].strip()
            records.append({
                "t_source": ts,
                "t_rx": tr,
                "t_watchdog": tw,
                "aoi_ms": aoi,
                "cmd_v_in": vi,
                "cmd_w_in": wi,
                "freshness": ef,
                "phase": ph,
            })
    return records


def apply_policy_full_update(records):
    return list(range(len(records)))


def apply_policy_periodic_k(records, k):
    kept = []
    for i in range(0, len(records), max(1, k)):
        kept.append(i)
    return kept


def apply_policy_delta_trigger(records, threshold):
    kept = []
    last_vi, last_wi = 0.0, 0.0
    for i, r in enumerate(records):
        dv = abs(r["cmd_v_in"] - last_vi)
        dw = abs(r["cmd_w_in"] - last_wi)
        delta_norm = dv + 0.5 * dw
        if delta_norm > threshold or i == 0:
            kept.append(i)
            last_vi = r["cmd_v_in"]
            last_wi = r["cmd_w_in"]
    return kept


def apply_policy_aoi_threshold(records, aoi_thresh, delta_thresh):
    kept = []
    last_vi, last_wi = 0.0, 0.0
    for i, r in enumerate(records):
        aoi = r.get("aoi_ms")
        dv = abs(r["cmd_v_in"] - last_vi)
        dw = abs(r["cmd_w_in"] - last_wi)
        delta_norm = dv + 0.5 * dw
        exceed = (aoi is not None and aoi > aoi_thresh) or delta_norm > delta_thresh
        if exceed or i == 0:
            kept.append(i)
            last_vi = r["cmd_v_in"]
            last_wi = r["cmd_w_in"]
    return kept


def apply_policy_delta_hold(records, delta_threshold, max_hold_ms):
    kept = []
    last_vi, last_wi = 0.0, 0.0
    last_sent_tw = -1e9
    for i, r in enumerate(records):
        dv = abs(r["cmd_v_in"] - last_vi)
        dw = abs(r["cmd_w_in"] - last_wi)
        delta_norm = dv + 0.5 * dw
        age = (r["t_watchdog"] - last_sent_tw) * 1000.0
        if i == 0 or delta_norm > delta_threshold or age >= max_hold_ms:
            kept.append(i)
            last_vi = r["cmd_v_in"]
            last_wi = r["cmd_w_in"]
            last_sent_tw = r["t_watchdog"]
    return kept


def apply_policy_freshness_risk(records, threshold):
    kept = []
    last_vi, last_wi = 0.0, 0.0
    for i, r in enumerate(records):
        dv = abs(r["cmd_v_in"] - last_vi)
        dw = abs(r["cmd_w_in"] - last_wi)
        delta_norm = dv + 0.5 * dw
        pw = PHASE_WEIGHTS.get(r["phase"], 1.0)
        ef = r.get("freshness", 1.0)
        score = delta_norm * pw * (1.0 - ef)
        if score > threshold or i == 0:
            kept.append(i)
            last_vi = r["cmd_v_in"]
            last_wi = r["cmd_w_in"]
    return kept


def compute_kpi(records, kept_indices, payload_bytes, run_id, robot, policy, params):
    total = len(records)
    kept_count = len(kept_indices)
    dropped = max(total - kept_count, 0)
    kept_ratio = kept_count / total if total > 0 else 1.0
    dropped_ratio = dropped / total if total > 0 else 0.0

    if kept_count >= 2 and len(records) >= 2:
        interval = records[-1]["t_watchdog"] - records[0]["t_watchdog"]
        tx_rate = kept_count / interval if interval > 0.01 else 0.0
        traffic_bytes = kept_count * payload_bytes
        traffic_rate = traffic_bytes / interval if interval > 0.01 else 0.0
    else:
        tx_rate = 0.0
        traffic_bytes = 0
        traffic_rate = 0.0

    # --- receiver-held-command timeline reconstruction ---
    kept_set = set(kept_indices)
    last_kept_idx = 0
    receiver_aoi_vals = []
    receiver_ef_vals = []
    held_error_norms = []
    held_cmd_deltas = []
    last_held_vi = 0.0
    last_held_wi = 0.0
    voi_full = []
    delta_full = []
    voi_kept_set = set()
    delta_kept_set = set()

    for i, r in enumerate(records):
        if i in kept_set:
            last_kept_idx = i
            last_held_vi = r["cmd_v_in"]
            last_held_wi = r["cmd_w_in"]
        held = records[last_kept_idx]
        age = r["t_watchdog"] - held["t_source"]
        if age > 0 and held["t_source"] > 0:
            receiver_aoi_vals.append(age * 1000.0)
        else:
            receiver_aoi_vals.append(0.0)
        receiver_ef_vals.append(math.exp(-receiver_aoi_vals[-1] / max(1.0, FRESHNESS_TAU_MS)))

        err = abs(r["cmd_v_in"] - last_held_vi) + 0.5 * abs(r["cmd_w_in"] - last_held_wi)
        held_error_norms.append(err)

        dv = abs(r["cmd_v_in"] - (records[i-1]["cmd_v_in"] if i > 0 else 0.0))
        dw = abs(r["cmd_w_in"] - (records[i-1]["cmd_w_in"] if i > 0 else 0.0))
        held_cmd_deltas.append(dv + 0.5 * dw)

        pw = PHASE_WEIGHTS.get(r["phase"], 1.0)
        ef = r.get("freshness", 1.0)
        vp = pw * held_cmd_deltas[-1] * ef
        voi_full.append(vp)
        if vp > HIGH_VOI_THRESHOLD:
            voi_full[-1] = vp
            if i in kept_set:
                voi_kept_set.add(i)

        dn = held_cmd_deltas[-1]
        delta_full.append(dn)
        if dn > HIGH_DELTA_THRESHOLD:
            if i in kept_set:
                delta_kept_set.add(i)

    high_voi_full = sum(1 for v in voi_full if v > HIGH_VOI_THRESHOLD)
    high_voi_kept = len(voi_kept_set)
    high_voi_recall = high_voi_kept / max(high_voi_full, 1)

    high_delta_full = sum(1 for v in delta_full if v > HIGH_DELTA_THRESHOLD)
    high_delta_kept = len(delta_kept_set)
    high_delta_recall = high_delta_kept / max(high_delta_full, 1)

    # --- aggregate receiver-side KPIs ---
    stale_c = len(receiver_aoi_vals)
    stale_50 = sum(1 for v in receiver_aoi_vals if v > 50) / max(stale_c, 1)
    stale_100 = sum(1 for v in receiver_aoi_vals if v > 100) / max(stale_c, 1)
    stale_200 = sum(1 for v in receiver_aoi_vals if v > 200) / max(stale_c, 1)

    phases = [records[i]["phase"] for i in range(total) if records[i].get("phase")]
    nonstandby = sum(1 for p in phases if p and p != "standby")
    phase_note = "standby_only_phase_label" if nonstandby == 0 else ("phase_varies" if phases else "no_phase_data")

    return {
        "run_id": run_id,
        "robot_id": robot,
        "policy": policy,
        "policy_params": str(params),
        "tx_count": kept_count,
        "tx_rate_hz": f"{tx_rate:.2f}",
        "tx_reduction_ratio": f"{dropped_ratio:.4f}",
        "traffic_bytes_est": traffic_bytes,
        "traffic_rate_Bps_est": f"{traffic_rate:.2f}",
        "receiver_AoI_ms_mean": f"{sum(receiver_aoi_vals) / stale_c:.2f}" if stale_c else "N/A",
        "receiver_AoI_ms_p95": f"{percentile(receiver_aoi_vals, 95):.2f}" if stale_c else "N/A",
        "receiver_stale_ratio_50ms": f"{stale_50:.4f}",
        "receiver_stale_ratio_100ms": f"{stale_100:.4f}",
        "receiver_stale_ratio_200ms": f"{stale_200:.4f}",
        "receiver_effective_freshness_mean": f"{sum(receiver_ef_vals) / len(receiver_ef_vals):.4f}" if receiver_ef_vals else "N/A",
        "receiver_effective_freshness_p05": f"{percentile(receiver_ef_vals, 5):.4f}" if receiver_ef_vals else "N/A",
        "held_cmd_error_norm_mean": f"{sum(held_error_norms) / len(held_error_norms):.6f}" if held_error_norms else "N/A",
        "held_cmd_error_norm_p95": f"{percentile(held_error_norms, 95):.6f}" if held_error_norms else "N/A",
        "held_cmd_error_norm_max": f"{max(held_error_norms):.6f}" if held_error_norms else "N/A",
        "cmd_delta_norm_mean": f"{sum(held_cmd_deltas) / len(held_cmd_deltas):.6f}" if held_cmd_deltas else "N/A",
        "kept_update_ratio": f"{kept_ratio:.4f}",
        "dropped_update_ratio": f"{dropped_ratio:.4f}",
        "phase_validity_note": phase_note,
        "high_VoI_recall": f"{high_voi_recall:.4f}",
        "missed_high_VoI_count": high_voi_full - high_voi_kept,
        "high_delta_recall": f"{high_delta_recall:.4f}",
        "missed_high_delta_count": high_delta_full - high_delta_kept,
        "VoI_proxy_sum": f"{sum(voi_full):.6f}" if voi_full else "N/A",
    }


def main():
    parser = argparse.ArgumentParser(description="Offline communication policy replay")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--robot", default="tracer1")
    parser.add_argument("--payload-bytes", type=int, default=128)
    parser.add_argument("--delay-ms", type=int, default=0)
    parser.add_argument("--jitter-ms", type=int, default=0)
    parser.add_argument("--loss-rate", type=float, default=0)
    parser.add_argument("--bandwidth-bps", type=int, default=0)
    args = parser.parse_args()

    mt_path = os.path.join(args.run_dir, f"mode_timeline_{args.robot}.csv")
    if not os.path.isfile(mt_path):
        print(f"FATAL: mode_timeline file not found: {mt_path}", file=sys.stderr)
        sys.exit(1)

    records = load_mode_timeline(mt_path)
    if not records:
        print("FATAL: no valid records in mode_timeline", file=sys.stderr)
        sys.exit(1)

    policies = []
    policies.append(("full_update", lambda r: apply_policy_full_update(r), {}))
    for k in [2, 5, 10]:
        policies.append((f"periodic_{k}", lambda r, k=k: apply_policy_periodic_k(r, k), {"k": k}))
    for thresh in [0.001, 0.005, 0.01]:
        policies.append((f"delta_{thresh:.3f}", lambda r, t=thresh: apply_policy_delta_trigger(r, t), {"threshold": thresh}))
    for (d, h) in [(0.001, 100), (0.001, 200), (0.001, 500), (0.005, 100), (0.005, 200), (0.005, 500), (0.010, 200), (0.010, 500)]:
        policies.append((f"delta_{d:.3f}_hold_{h}", lambda r, d=d, h=h: apply_policy_delta_hold(r, d, h), {"delta_threshold": d, "max_hold_ms": h}))
    for aoi_t, delta_t in [(50, 0.005), (100, 0.005), (200, 0.01)]:
        policies.append((f"aoi_{aoi_t}_delta_{delta_t:.3f}", lambda r, a=aoi_t, d=delta_t: apply_policy_aoi_threshold(r, a, d), {"aoi_thresh": aoi_t, "delta_thresh": delta_t}))
    for risk in [0.0001, 0.001, 0.005]:
        policies.append((f"freshness_risk_{risk:.4f}", lambda r, t=risk: apply_policy_freshness_risk(r, t), {"threshold": risk}))

    rows = []
    for policy_name, policy_fn, params in policies:
        kept = policy_fn(records)
        row = compute_kpi(records, kept, args.payload_bytes, args.run_id, args.robot, policy_name, params)
        rows.append(row)

    out_path = os.path.join(args.run_dir, "communication_policy_replay_summary.csv")
    columns = [
        "run_id", "robot_id", "policy", "policy_params",
        "tx_count", "tx_rate_hz", "tx_reduction_ratio",
        "traffic_bytes_est", "traffic_rate_Bps_est",
        "receiver_AoI_ms_mean", "receiver_AoI_ms_p95",
        "receiver_stale_ratio_50ms", "receiver_stale_ratio_100ms", "receiver_stale_ratio_200ms",
        "receiver_effective_freshness_mean", "receiver_effective_freshness_p05",
        "held_cmd_error_norm_mean", "held_cmd_error_norm_p95", "held_cmd_error_norm_max",
        "high_VoI_recall", "missed_high_VoI_count",
        "high_delta_recall", "missed_high_delta_count",
        "VoI_proxy_sum", "cmd_delta_norm_mean",
        "kept_update_ratio", "dropped_update_ratio",
        "phase_validity_note",
    ]
    with open(out_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    for row in rows:
        print(
            f"{row['policy']:22s}  tx={row['tx_count']:4d}  "
            f"drop={row['tx_reduction_ratio']}  "
            f"AoI_p95={row['receiver_AoI_ms_p95']}  "
            f"stale50={row['receiver_stale_ratio_50ms']}  "
            f"err_p95={row['held_cmd_error_norm_p95']}  "
            f"d_recall={row['high_delta_recall']}"
        )


if __name__ == "__main__":
    main()
