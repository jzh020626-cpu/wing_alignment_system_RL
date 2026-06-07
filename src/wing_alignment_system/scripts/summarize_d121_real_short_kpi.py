#!/usr/bin/env python3
"""summarize_d121_real_short_kpi.py — clean KPI parsing for D1-2-1 short runs.

Reads CSV artifacts from run_dir and outputs a single-line summary:
  stamped_samples nonzero_stamped max_abs_linear max_abs_angular
  duration_active_s cmd_vel_samples cmd_vel_nonzero emergency_count
  cmd_stop_count age_stop_count final_zero_int tracer2_nonzero tracer3_nonzero

All int fields are sanitized to decimal integers. No bare "0" lines.
"""

import csv
import sys
import os


def count_nonempty_lines(path):
    """Count non-empty lines in a CSV file."""
    if not os.path.isfile(path):
        return 0
    count = 0
    with open(path, 'r') as f:
        for line in f:
            if line.strip():
                count += 1
    return count


def parse_twist_csv(path, col_vx=7, col_wz=8, ts_col=0):
    """Parse a topic-echo CSV of Twist or TwistStamped.

    Returns (sample_count, nonzero_count, max_abs_v, max_abs_w, duration_s, final_is_zero).
    All six Twist components (linear.x/y/z, angular.x/y/z) are scanned.
    For TwistStamped CSV, col offsets shift.
    """
    if not os.path.isfile(path):
        return 0, 0, 0.0, 0.0, 0.0, 1

    samples = 0
    nonzero = 0
    max_v = 0.0
    max_w = 0.0
    first_ts = None
    last_ts = 0.0
    last_vx = 0.0
    last_vy = 0.0
    last_vz = 0.0
    last_wx = 0.0
    last_wy = 0.0
    last_wz = 0.0

    with open(path, 'r') as f:
        for row in csv.reader(f):
            if not row or len(row) < 2:
                continue
            try:
                ts = float(row[ts_col])
                if ts_col == 0:
                    # header guard: first column must be numeric
                    pass
                if first_ts is None:
                    first_ts = ts
                last_ts = ts

                # Scan all 6 twist components
                def safe_float(r, i):
                    try:
                        return float(r[i]) if i < len(r) else 0.0
                    except (ValueError, IndexError):
                        return 0.0

                vx = safe_float(row, col_vx)
                vy = safe_float(row, col_vx + 1)
                vz = safe_float(row, col_vx + 2)
                wx = safe_float(row, col_vx + 3)
                wy = safe_float(row, col_vx + 4)
                wz = safe_float(row, col_vx + 5)

                # Nonzero check: any of 6 components > 1e-6
                if (abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(vz) > 1e-6 or
                        abs(wx) > 1e-6 or abs(wy) > 1e-6 or abs(wz) > 1e-6):
                    nonzero += 1

                max_v = max(max_v, abs(vx), abs(vy), abs(vz))
                max_w = max(max_w, abs(wx), abs(wy), abs(wz))
                last_vx, last_vy, last_vz = vx, vy, vz
                last_wx, last_wy, last_wz = wx, wy, wz
                samples += 1
            except (ValueError, IndexError):
                continue

    dur = max(0.0, last_ts - first_ts) if first_ts is not None else 0.0
    final_zero = 1 if (abs(last_vx) < 1e-6 and abs(last_vy) < 1e-6 and
                       abs(last_vz) < 1e-6 and abs(last_wx) < 1e-6 and
                       abs(last_wy) < 1e-6 and abs(last_wz) < 1e-6) else 0
    return samples, nonzero, max_v, max_w, dur, final_zero


def parse_stamped_csv(path):
    """Parse TwistStamped CSV. Returns (samples, nonzero, _, _, _, _)."""
    # TwistStamped has header.stamp.sec, header.stamp.nanosec, frame_id, then twist fields
    # Topic echo CSV varies; use the generic parser with appropriate column offsets
    # For ros2 topic echo --csv of TwistStamped, columns are roughly:
    #   sec, nanosec, frame_id, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
    if not os.path.isfile(path):
        return 0, 0

    samples = 0
    nonzero = 0
    with open(path, 'r') as f:
        for row in csv.reader(f):
            if not row or len(row) < 3:
                continue
            try:
                # Check if this is a timestamped row
                if len(row) >= 9:
                    # Try columns 3-8 for twist components
                    vx = float(row[3]) if row[3].strip() else 0.0
                    vy = float(row[4]) if len(row) > 4 and row[4].strip() else 0.0
                    vz = float(row[5]) if len(row) > 5 and row[5].strip() else 0.0
                    wx = float(row[6]) if len(row) > 6 and row[6].strip() else 0.0
                    wy = float(row[7]) if len(row) > 7 and row[7].strip() else 0.0
                    wz = float(row[8]) if len(row) > 8 and row[8].strip() else 0.0
                elif len(row) >= 6:
                    # Shorter: columns 0-5 are twist
                    vx = float(row[0]) if row[0].strip() else 0.0
                    vy = float(row[1]) if len(row) > 1 and row[1].strip() else 0.0
                    vz = float(row[2]) if len(row) > 2 and row[2].strip() else 0.0
                    wx = float(row[3]) if len(row) > 3 and row[3].strip() else 0.0
                    wy = float(row[4]) if len(row) > 4 and row[4].strip() else 0.0
                    wz = float(row[5]) if len(row) > 5 and row[5].strip() else 0.0
                else:
                    continue

                if (abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(vz) > 1e-6 or
                        abs(wx) > 1e-6 or abs(wy) > 1e-6 or abs(wz) > 1e-6):
                    nonzero += 1
                samples += 1
            except (ValueError, IndexError):
                continue

    return samples, nonzero


def count_boolean_csv(path):
    """Count lines containing True/true/1 in a CSV."""
    if not os.path.isfile(path):
        return 0
    count = 0
    with open(path, 'r') as f:
        for line in f:
            stripped = line.strip().lower()
            if stripped in ('true', '1') or 'true' in stripped:
                count += 1
    return count


def sanitize_int(val):
    """Return decimal integer string, or '0' on failure."""
    try:
        return str(int(float(str(val).strip())))
    except (ValueError, TypeError):
        return '0'


def main():
    if len(sys.argv) < 2:
        print('Usage: summarize_d121_real_short_kpi.py <run_dir>', file=sys.stderr)
        sys.exit(2)

    run_dir = sys.argv[1]

    # ---- stamped samples ----
    stamped_path = os.path.join(run_dir, 'cmd_vel_stamped_tracer1.csv')
    stamped_samples, stamped_nonzero = parse_stamped_csv(stamped_path)

    # ---- cmd_vel output ----
    cmd_vel_path = os.path.join(run_dir, 'cmd_vel_tracer1.csv')
    cmd_samples, cmd_nonzero, max_v, max_w, dur, final_zero = parse_twist_csv(cmd_vel_path)

    # ---- emergency / cmd_stop / age_stop ----
    emg_path = os.path.join(run_dir, 'emergency_stop.csv')
    emg_count = count_boolean_csv(emg_path)

    stop_path = os.path.join(run_dir, 'cmd_stop_tracer1.csv')
    stop_count = count_boolean_csv(stop_path)

    age_stop_count = 0

    # ---- tracer2/tracer3 crosscheck ----
    t2_path = os.path.join(run_dir, 'cmd_vel_tracer2_crosscheck.csv')
    _, t2_nonzero, _, _, _, _ = parse_twist_csv(t2_path)

    t3_path = os.path.join(run_dir, 'cmd_vel_tracer3_crosscheck.csv')
    _, t3_nonzero, _, _, _, _ = parse_twist_csv(t3_path)

    # ---- mode_timeline ----
    mt_path = os.path.join(run_dir, 'mode_timeline_tracer1.csv')
    mt_samples = count_nonempty_lines(mt_path)
    if mt_samples > 0:
        mt_samples -= 1  # header row

    # ---- Output: single sanitized line ----
    fields = [
        sanitize_int(stamped_samples),
        sanitize_int(stamped_nonzero),
        '{:.6f}'.format(max_v),
        '{:.6f}'.format(max_w),
        '{:.3f}'.format(dur),
        sanitize_int(cmd_samples),
        sanitize_int(cmd_nonzero),
        sanitize_int(emg_count),
        sanitize_int(stop_count),
        sanitize_int(age_stop_count),
        sanitize_int(final_zero),
        sanitize_int(t2_nonzero),
        sanitize_int(t3_nonzero),
        sanitize_int(mt_samples),
    ]
    print(' '.join(fields))


if __name__ == '__main__':
    main()
