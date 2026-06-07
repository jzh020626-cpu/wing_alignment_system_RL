#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR-TAC-P3-D1c-R4a: Goal-Trigger Readiness KPI Collector
========================================================
Reads a rosbag2 sqlite3 db3 file directly to compute accurate per-topic
statistics and non-zero Twist/TwistStamped counts across all 6 fields.

Usage:
  python3 summarize_d1c_r4_goal_trigger_kpi.py \
    --bag-dir /path/to/bag \
    [--watchdog-dir /path/to/cmd_safety_logs/run_id] \
    [--robot tracer1] \
    [--output json|csv]
"""

import argparse
import json
import os
import sqlite3
import sys
from collections import OrderedDict

EPSILON = 1e-6


def _get_msg_type(cur, topic_name):
    row = cur.execute("SELECT type FROM topics WHERE name=?", (topic_name,)).fetchone()
    return row[0] if row else None


def _count_twist_generic(cur, topic_name, msg_type_str):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    row = cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,)).fetchone()
    if not row:
        return 0, 0, 0.0

    Cls = get_message(msg_type_str)
    is_stamped = "TwistStamped" in msg_type_str

    total = 0
    nonzero = 0
    max_abs = 0.0
    for ts, data in cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (row[0],)):
        total += 1
        msg = deserialize_message(data, Cls)
        t = msg.twist if is_stamped else msg
        vals = [t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z]
        local_max = max(abs(v) for v in vals)
        max_abs = max(max_abs, local_max)
        if local_max > EPSILON:
            nonzero += 1
    return total, nonzero, max_abs


def _count_bool(cur, topic_name):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Bool
    row = cur.execute("SELECT id FROM topics WHERE name=?", (topic_name,)).fetchone()
    if not row:
        return 0, 0
    total = 0
    true_count = 0
    for ts, data in cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (row[0],)):
        total += 1
        msg = deserialize_message(data, Bool)
        if msg.data:
            true_count += 1
    return total, true_count


def find_watchdog_artifacts(watchdog_dir, robot):
    found = []
    mt_lines = 0
    if not watchdog_dir or not os.path.isdir(watchdog_dir):
        return found, mt_lines
    for label in ["mode_timeline", "rx", "ts"]:
        fname = f"{label}_{robot}.csv"
        fpath = os.path.join(watchdog_dir, fname)
        if os.path.isfile(fpath):
            found.append(fpath)
            if label == "mode_timeline":
                with open(fpath) as f:
                    raw = sum(1 for _ in f)
                if raw > 0:
                    with open(fpath) as f:
                        first = f.readline().strip()
                    mt_lines = max(0, raw - 1) if any(
                        kw in first.lower() for kw in ["timestamp", "run_id", "phase", "mode"]
                    ) else raw
    return found, mt_lines


def main():
    parser = argparse.ArgumentParser(description="D1c-R4a Goal-Trigger KPI Collector")
    parser.add_argument("--bag-dir", required=True)
    parser.add_argument("--watchdog-dir", default="")
    parser.add_argument("--robot", default="tracer1")
    parser.add_argument("--output", default="json", choices=["json", "csv"])
    args = parser.parse_args()

    bag_dir = args.bag_dir
    db_file = None
    if os.path.isfile(bag_dir):
        db_file = bag_dir
    else:
        for f in sorted(os.listdir(bag_dir)):
            if f.endswith(".db3"):
                db_file = os.path.join(bag_dir, f)
                break
    if not db_file or not os.path.isfile(db_file):
        print(f"ERROR: No .db3 in {bag_dir}", file=sys.stderr)
        sys.exit(1)

    db = sqlite3.connect(db_file)
    cur = db.cursor()
    robot = args.robot
    r = OrderedDict()

    twist_topics = [
        ("cmd_goal_samples", f"/{robot}/cmd_goal"),
        ("cmd_vel_desired_samples", f"/{robot}/cmd_vel_desired"),
        ("cmd_vel_stamped_samples", f"/{robot}/cmd_vel_stamped"),
        ("cmd_vel_output_samples", f"/{robot}/cmd_vel"),
    ]
    for key, topic in twist_topics:
        mtype = _get_msg_type(cur, topic)
        if mtype:
            total, nz, mx = _count_twist_generic(cur, topic, mtype)
            r[key] = total
            base = key.replace("_samples", "")
            r[f"{base}_nonzero_samples"] = nz
            r[f"{base}_max_abs"] = round(mx, 6)

    bool_topics = [
        ("emergency_stop_samples", "/wing_alignment/emergency_stop"),
        ("cmd_stop_samples", f"/{robot}/cmd_stop"),
    ]
    for key, topic in bool_topics:
        mtype = _get_msg_type(cur, topic)
        if mtype:
            total, tc = _count_bool(cur, topic)
            r[key] = total
            r[f"{key}_true_count"] = tc

    wd_found, mt_lines = find_watchdog_artifacts(args.watchdog_dir, robot)
    r["watchdog_artifacts_found"] = [os.path.basename(p) for p in wd_found]
    r["mode_timeline_lines"] = mt_lines
    r["mode_timeline_samples"] = max(0, mt_lines - 1) if mt_lines > 0 else 0

    db.close()

    if args.output == "csv":
        for k, v in r.items():
            if isinstance(v, list):
                print(f"{k},{','.join(str(x) for x in v)}")
            else:
                print(f"{k},{v}")
    else:
        print(json.dumps(r, indent=2))


if __name__ == "__main__":
    main()

