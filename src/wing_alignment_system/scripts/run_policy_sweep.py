#!/usr/bin/env python3
"""Server-side log-driven communication policy sweep simulator.

Reads real mode_timeline logs, applies policies + channel perturbations,
computes receiver-held-command timeline KPIs in parallel.

VoI_proxy is logging-only command-change freshness proxy, not validated VoI.
"""

import argparse
import csv
import math
import os
import random
import sys
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from itertools import product

PHASE_WEIGHTS = {
    "standby": 0.2, "approach": 1.0, "transport": 1.2,
    "align": 1.5, "slide_align": 1.5, "docking": 2.0, "final_align": 2.0,
}
PAYLOAD_BYTES = 128
FRESHNESS_TAU_MS = 1000.0
HIGH_DELTA_TH = 0.005
HIGH_VOI_TH = 0.001


def safe_float(v):
    try:
        return float(str(v).strip())
    except (ValueError, TypeError):
        return None


def pct(vals, p):
    if not vals:
        return 0.0
    return sorted(vals)[max(0, min(len(vals) - 1, int(math.ceil(p / 100.0 * len(vals))) - 1))]


def load_records(path):
    recs = []
    if not os.path.isfile(path):
        return recs
    with open(path, newline="") as f:
        r = csv.reader(f)
        hdr = next(r, None)
        if hdr is None:
            return recs
        col = {n.strip(): i for i, n in enumerate(hdr)}
        for row in r:
            if len(row) < 19:
                continue
            ts = safe_float(row[col.get("t_source", 16)]) or 0.0
            tw = safe_float(row[col.get("t_watchdog", 18)]) or 0.0
            vi = safe_float(row[col.get("cmd_v_in", 12)]) or 0.0
            wi = safe_float(row[col.get("cmd_w_in", 13)]) or 0.0
            ef = safe_float(row[col.get("effective_freshness", 7)]) or 1.0
            ph = row[col.get("phase", 8)].strip()
            recs.append({
                "t_source": ts, "t_watchdog": tw, "cmd_v": vi, "cmd_w": wi,
                "freshness": ef, "phase": ph,
            })
    return recs


# --- policies ---

def policy_full(recs):
    return set(range(len(recs)))


def policy_periodic(recs, k):
    return set(range(0, len(recs), max(1, k)))


def policy_delta(recs, threshold):
    kept = set(); kept.add(0)
    lv, lw = recs[0]["cmd_v"], recs[0]["cmd_w"]
    for i in range(1, len(recs)):
        dv = abs(recs[i]["cmd_v"] - lv) + 0.5 * abs(recs[i]["cmd_w"] - lw)
        if dv > threshold:
            kept.add(i); lv, lw = recs[i]["cmd_v"], recs[i]["cmd_w"]
    return kept


def policy_delta_hold(recs, threshold, max_hold_ms):
    kept = set(); kept.add(0)
    lv, lw = recs[0]["cmd_v"], recs[0]["cmd_w"]
    lt = recs[0]["t_watchdog"]
    for i in range(1, len(recs)):
        dv = abs(recs[i]["cmd_v"] - lv) + 0.5 * abs(recs[i]["cmd_w"] - lw)
        age = (recs[i]["t_watchdog"] - lt) * 1000.0
        if dv > threshold or age >= max_hold_ms:
            kept.add(i); lv, lw = recs[i]["cmd_v"], recs[i]["cmd_w"]; lt = recs[i]["t_watchdog"]
    return kept


def policy_aoi_delta(recs, aoi_th, delta_th):
    kept = set(); kept.add(0)
    lv, lw = recs[0]["cmd_v"], recs[0]["cmd_w"]
    lt = recs[0]["t_watchdog"]
    for i in range(1, len(recs)):
        dv = abs(recs[i]["cmd_v"] - lv) + 0.5 * abs(recs[i]["cmd_w"] - lw)
        age = (recs[i]["t_watchdog"] - lt) * 1000.0
        if dv > delta_th or age > aoi_th:
            kept.add(i); lv, lw = recs[i]["cmd_v"], recs[i]["cmd_w"]; lt = recs[i]["t_watchdog"]
    return kept


# --- channel simulation ---

def apply_channel(recs, kept, delay_ms, jitter_ms, loss_rate, bandwidth_bps, seed):
    rng = random.Random(seed)
    sent = sorted(kept)
    delivered = {}
    queue = []
    t_now = 0.0
    tx_count = len(sent)
    loss_count = 0
    q_delays = []
    ch_delays = []

    last_deliver_t = 0.0
    bw_interval = PAYLOAD_BYTES / bandwidth_bps if bandwidth_bps > 0 else 0.0

    for s_idx in sent:
        r = recs[s_idx]
        ch_d = delay_ms / 1000.0 + abs(rng.gauss(0, jitter_ms / 1000.0))
        ch_delays.append(ch_d * 1000.0)
        t_avail = r["t_watchdog"] + ch_d
        if rng.random() < loss_rate:
            loss_count += 1
            continue
        queue.append((s_idx, t_avail))

    queue.sort(key=lambda x: x[1])

    for s_idx, t_avail in queue:
        actual = max(t_avail, last_deliver_t + bw_interval)
        q_delays.append((actual - t_avail) * 1000.0)
        delivered[s_idx] = actual
        last_deliver_t = actual

    return delivered, loss_count, q_delays, ch_delays, tx_count


# --- KPI computation ---

def compute_kpis(recs, delivered, tx_count, loss_count, q_delays, ch_delays, kept):
    total = len(recs)
    kept_c = len(kept)
    delivered_c = len(delivered)

    if kept_c >= 2:
        tspan = recs[max(kept)]["t_watchdog"] - recs[min(kept)]["t_watchdog"]
        tx_rate = kept_c / tspan if tspan > 0.01 else 0.0
    else:
        tx_rate = 0.0

    # receiver-held timeline
    raoi = []; ref_vals = []; errs = []; deltas = []
    last_del_idx = 0
    last_held_v, last_held_w = 0.0, 0.0
    for i in range(total):
        if i in delivered:
            last_del_idx = i
            last_held_v, last_held_w = recs[i]["cmd_v"], recs[i]["cmd_w"]
        h = recs[last_del_idx]
        age = recs[i]["t_watchdog"] - h["t_source"]
        raoi.append(max(0.0, age * 1000.0))
        ref_vals.append(math.exp(-raoi[-1] / FRESHNESS_TAU_MS))
        errs.append(abs(recs[i]["cmd_v"] - last_held_v) + 0.5 * abs(recs[i]["cmd_w"] - last_held_w))
        dv = abs(recs[i]["cmd_v"] - (recs[i - 1]["cmd_v"] if i > 0 else 0.0))
        dw = abs(recs[i]["cmd_w"] - (recs[i - 1]["cmd_w"] if i > 0 else 0.0))
        deltas.append(dv + 0.5 * dw)

    stale_c = max(len(raoi), 1)
    stale50 = sum(1 for v in raoi if v > 50) / stale_c
    stale100 = sum(1 for v in raoi if v > 100) / stale_c
    stale200 = sum(1 for v in raoi if v > 200) / stale_c

    # high delta/VoI recall
    d_hi_full = sum(1 for v in deltas if v > HIGH_DELTA_TH)
    d_hi_kept = sum(1 for i in range(total) if deltas[i] > HIGH_DELTA_TH and i in delivered)
    vo_hi_full = 0; vo_hi_kept = 0
    for i in range(total):
        pw = PHASE_WEIGHTS.get(recs[i]["phase"], 1.0)
        ef = recs[i].get("freshness", 1.0)
        vp = pw * deltas[i] * ef
        if vp > HIGH_VOI_TH:
            vo_hi_full += 1
            if i in delivered:
                vo_hi_kept += 1

    phases = [recs[i]["phase"] for i in range(total) if recs[i].get("phase")]
    nonstd = sum(1 for p in phases if p and p != "standby")
    ph_note = "standby_only_phase_label" if nonstd == 0 else ("phase_varies" if phases else "no_phase_data")

    return {
        "tx_count": kept_c,
        "tx_reduction_ratio": round(1.0 - kept_c / max(total, 1), 4),
        "payload_bytes_est": PAYLOAD_BYTES,
        "traffic_bytes_est": kept_c * PAYLOAD_BYTES,
        "traffic_rate_Bps_est": round(tx_rate * PAYLOAD_BYTES, 2),
        "receiver_AoI_ms_mean": round(sum(raoi) / stale_c, 2),
        "receiver_AoI_ms_p95": round(pct(raoi, 95), 2),
        "receiver_AoI_ms_max": round(max(raoi), 2),
        "stale_ratio_50ms": round(stale50, 4),
        "stale_ratio_100ms": round(stale100, 4),
        "stale_ratio_200ms": round(stale200, 4),
        "receiver_effective_freshness_mean": round(sum(ref_vals) / len(ref_vals), 4),
        "receiver_effective_freshness_p05": round(pct(ref_vals, 5), 4),
        "held_cmd_error_norm_mean": round(sum(errs) / len(errs), 6),
        "held_cmd_error_norm_p95": round(pct(errs, 95), 6),
        "held_cmd_error_norm_max": round(max(errs), 6),
        "high_delta_recall": round(d_hi_kept / max(d_hi_full, 1), 4),
        "missed_high_delta_count": d_hi_full - d_hi_kept,
        "high_VoI_recall": round(vo_hi_kept / max(vo_hi_full, 1), 4),
        "missed_high_VoI_count": vo_hi_full - vo_hi_kept,
        "delivered_count": delivered_c,
        "dropped_count": tx_count - delivered_c,
        "loss_observed": round(loss_count / max(tx_count, 1), 4),
        "queue_delay_ms_mean": round(sum(q_delays) / max(len(q_delays), 1), 2) if q_delays else 0.0,
        "queue_delay_ms_p95": round(pct(q_delays, 95), 2) if q_delays else 0.0,
        "channel_delay_ms_mean": round(sum(ch_delays) / max(len(ch_delays), 1), 2) if ch_delays else 0.0,
        "channel_delay_ms_p95": round(pct(ch_delays, 95), 2) if ch_delays else 0.0,
        "VoI_proxy_sum": round(sum(deltas[i] * PHASE_WEIGHTS.get(recs[i]["phase"], 1.0) * recs[i].get("freshness", 1.0) for i in range(total) if i in delivered), 6),
        "VoI_proxy_mean": 0.0,
        "VoI_proxy_p95": 0.0,
        "phase_validity_note": ph_note,
    }


# --- policy dispatch ---

def apply_policy_by_name(recs, policy_name, params):
    if policy_name == "full_update":
        return policy_full(recs)
    if policy_name.startswith("periodic_"):
        return policy_periodic(recs, params["k"])
    if policy_name.startswith("delta_hold_"):
        return policy_delta_hold(recs, params["threshold"], params["max_hold_ms"])
    if policy_name.startswith("delta_"):
        return policy_delta(recs, params["threshold"])
    if policy_name.startswith("aoi_"):
        return policy_aoi_delta(recs, params["aoi_th"], params["delta_th"])
    return policy_full(recs)


def run_one(args_tuple):
    (run_name, recs), policy_spec, channel, seed = args_tuple
    policy_name, policy_params = policy_spec
    kept = apply_policy_by_name(recs, policy_name, policy_params)
    delay, jitter, loss, bw = channel
    delivered, loss_c, qd, chd, tx_c = apply_channel(recs, kept, delay, jitter, loss, bw, seed)
    kpi = compute_kpis(recs, delivered, tx_c, loss_c, qd, chd, kept)
    kpi["run_name"] = run_name
    kpi["policy"] = policy_name
    kpi["delay_ms"] = delay
    kpi["jitter_ms"] = jitter
    kpi["loss_rate"] = loss
    kpi["bandwidth_bps"] = bw
    kpi["seed"] = seed
    return kpi


POLICY_COLS = [
    "run_name", "policy", "delay_ms", "jitter_ms", "loss_rate", "bandwidth_bps", "seed",
    "tx_count", "tx_reduction_ratio", "payload_bytes_est", "traffic_bytes_est", "traffic_rate_Bps_est",
    "receiver_AoI_ms_mean", "receiver_AoI_ms_p95", "receiver_AoI_ms_max",
    "stale_ratio_50ms", "stale_ratio_100ms", "stale_ratio_200ms",
    "receiver_effective_freshness_mean", "receiver_effective_freshness_p05",
    "held_cmd_error_norm_mean", "held_cmd_error_norm_p95", "held_cmd_error_norm_max",
    "high_delta_recall", "missed_high_delta_count", "high_VoI_recall", "missed_high_VoI_count",
    "delivered_count", "dropped_count", "loss_observed",
    "queue_delay_ms_mean", "queue_delay_ms_p95",
    "channel_delay_ms_mean", "channel_delay_ms_p95",
    "VoI_proxy_sum", "VoI_proxy_mean", "VoI_proxy_p95", "phase_validity_note",
]


def build_policies(smoke):
    policies = [("full_update", {})]
    if smoke:
        policies += [
            ("periodic_2", {"k": 2}),
            ("delta_hold_0.001_100", {"threshold": 0.001, "max_hold_ms": 100}),
            ("delta_hold_0.005_100", {"threshold": 0.005, "max_hold_ms": 100}),
        ]
        return policies
    for k in [2, 3, 5, 10]:
        policies.append((f"periodic_{k}", {"k": k}))
    for th in [0.001, 0.005, 0.010]:
        policies.append((f"delta_{th}", {"threshold": th}))
    for th in [0.001, 0.005, 0.010]:
        for h in [50, 100, 150, 200, 500]:
            policies.append((f"delta_hold_{th:.3f}_{h}", {"threshold": th, "max_hold_ms": h}))
    for a, d in [(50, 0.001), (50, 0.005), (100, 0.001), (100, 0.005), (200, 0.001), (200, 0.005)]:
        policies.append((f"aoi_{a}_delta_{d}", {"aoi_th": a, "delta_th": d}))
    return policies


def build_channels(smoke):
    if smoke:
        return [(0, 0, 0.0, 0)]
    ch = []
    for d in [0, 20, 50, 100]:
        for j in [0, 10, 30]:
            for l in [0.0, 0.05, 0.10]:
                for bw in [0, 8000, 16000, 32000]:
                    ch.append((d, j, l, bw))
    return ch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-root", default="/workspace/fr_tac_sim/data/real_logs")
    ap.add_argument("--runs", default="")
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--full", action="store_true")
    ap.add_argument("--workers", type=int, default=0)
    ap.add_argument("--output-dir", default="")
    args = ap.parse_args()

    if not args.smoke and not args.full:
        print("Use --smoke or --full", file=sys.stderr)
        sys.exit(1)

    run_names = [r.strip() for r in (args.runs or "").split(",") if r.strip()]
    if not run_names:
        print("No --runs specified", file=sys.stderr)
        sys.exit(1)

    # load data
    runs_data = {}
    for rn in run_names:
        p = os.path.join(args.data_root, rn, "mode_timeline_tracer1.csv")
        if not os.path.isfile(p):
            print(f"FATAL: missing {p}", file=sys.stderr)
            sys.exit(1)
        recs = load_records(p)
        if not recs:
            print(f"FATAL: empty data for {rn}", file=sys.stderr)
            sys.exit(1)
        runs_data[rn] = recs
        print(f"Loaded {rn}: {len(recs)} records")

    policies = build_policies(args.smoke)
    channels = build_channels(args.smoke)
    seeds = [0] if args.smoke else list(range(10))
    workers = args.workers
    if workers <= 0:
        workers = min(120, max(1, os.cpu_count() - 4))

    out_dir = args.output_dir
    if not out_dir:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_dir = f"/workspace/fr_tac_sim/outputs/policy_sweep_{ts}"
    os.makedirs(out_dir, exist_ok=True)

    print(f"Runs: {len(runs_data)}  Policies: {len(policies)}  Channels: {len(channels)}  Seeds: {len(seeds)}")
    print(f"Workers: {workers}  Output: {out_dir}")

    tasks = []
    for (rn, recs) in runs_data.items():
        for pol in policies:
            for ch in channels:
                for s in seeds:
                    tasks.append(((rn, list(recs)), (pol[0], pol[1]), ch, s))

    t0 = time.time()
    done = 0
    results = []
    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(run_one, t): t for t in tasks}
        for fut in as_completed(futs):
            results.append(fut.result())
            done += 1
            if done % 500 == 0:
                print(f"  {done}/{len(tasks)} ({time.time() - t0:.0f}s)")

    elapsed = time.time() - t0
    print(f"Completed {len(results)} tasks in {elapsed:.0f}s")

    # write summary
    sum_path = os.path.join(out_dir, "policy_sweep_summary.csv")
    with open(sum_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=POLICY_COLS)
        w.writeheader()
        for r in results:
            w.writerow(r)

    # top candidates
    top_path = os.path.join(out_dir, "top_candidates.csv")
    candidates = []
    for r in results:
        if (r.get("tx_reduction_ratio", 0) >= 0.50 and
            r.get("receiver_AoI_ms_p95", 999) <= 200 and
            r.get("stale_ratio_200ms", 1.0) <= 0.05 and
            r.get("high_delta_recall", 0) >= 0.95 and
            r.get("high_VoI_recall", 0) >= 0.95 and
            r.get("delivered_count", 0) >= 10):
            candidates.append(r)
    candidates.sort(key=lambda r: (
        -r.get("tx_reduction_ratio", 0),
        r.get("receiver_AoI_ms_p95", 999),
        r.get("held_cmd_error_norm_p95", 999),
    ))
    with open(top_path, "w", newline="") as f:
        if candidates:
            w = csv.DictWriter(f, fieldnames=POLICY_COLS)
            w.writeheader()
            for c in candidates:
                w.writerow(c)

    print(f"Candidates: {len(candidates)} (top 5 shown)")
    for c in candidates[:5]:
        print(f"  {c['policy']:30s}  drop={c['tx_reduction_ratio']:.2f}  "
              f"AoI_p95={c['receiver_AoI_ms_p95']:.1f}  "
              f"err_p95={c['held_cmd_error_norm_p95']:.5f}  "
              f"dR={c['high_delta_recall']:.2f}  vR={c['high_VoI_recall']:.2f}")

    # manifest
    manifest = os.path.join(out_dir, "run_manifest.txt")
    with open(manifest, "w") as f:
        f.write(f"code: run_policy_sweep.py\n")
        f.write(f"runs: {run_names}\n")
        f.write(f"policies: {len(policies)}\n")
        f.write(f"channels: {len(channels)}\n")
        f.write(f"seeds: {seeds[0]}..{seeds[-1]}\n")
        f.write(f"workers: {workers}\n")
        f.write(f"start: {time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(t0))}\n")
        f.write(f"end: {time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(time.time()))}\n")
        f.write(f"elapsed_s: {elapsed:.0f}\n")
        f.write(f"candidates: {len(candidates)}\n")

    print(f"Done. Output: {out_dir}")
    print(f"  {sum_path}")
    print(f"  {top_path}")
    print(f"  {manifest}")


if __name__ == "__main__":
    main()
