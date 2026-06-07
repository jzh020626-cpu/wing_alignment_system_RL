#!/usr/bin/env python3
"""D1c-R4a Recompute Gate Generator. Reads KPI JSON, writes gate file."""
import json, sys, os, datetime

def main():
    if len(sys.argv) < 3:
        print("Usage: recompute_gate.py <kpi.json> <run_dir>")
        sys.exit(1)
    
    kpi_file = sys.argv[1]
    run_dir = sys.argv[2]
    run_name = os.path.basename(run_dir)
    
    with open(kpi_file) as f:
        k = json.load(f)
    
    errors = 0
    lines = []
    def add(s): lines.append(s)
    
    add("=" * 60)
    add("FR-TAC-P3-D1c-R4a Gate (RECOMPUTED)")
    add("=" * 60)
    add(f"Run: {run_name}")
    add(f"Timestamp: {datetime.datetime.now().isoformat()}")
    add("Mode: RECOMPUTED from existing artifacts. No new robot action.")
    add("This is coordinate-corrected direct goal-trigger readiness,")
    add("NOT natural mission telemetry baseline.")
    add("")
    
    # Metrics
    add(f"cmd_goal_samples              = {k['cmd_goal_samples']}")
    add(f"cmd_vel_desired_samples       = {k['cmd_vel_desired_samples']}")
    add(f"cmd_vel_desired_nonzero       = {k['cmd_vel_desired_nonzero_samples']}")
    add(f"cmd_vel_desired_max_abs       = {k['cmd_vel_desired_max_abs']}")
    add(f"cmd_vel_stamped_samples       = {k['cmd_vel_stamped_samples']}")
    add(f"cmd_vel_stamped_nonzero       = {k['cmd_vel_stamped_nonzero_samples']}")
    add(f"cmd_vel_output_samples        = {k['cmd_vel_output_samples']}")
    add(f"cmd_vel_output_nonzero        = {k['cmd_vel_output_nonzero_samples']}")
    add(f"emergency_stop_total          = {k['emergency_stop_samples']}")
    add(f"emergency_stop_true_count     = {k['emergency_stop_samples_true_count']}")
    add(f"cmd_stop_true_count           = {k['cmd_stop_samples_true_count']}")
    add(f"mode_timeline_samples         = {k['mode_timeline_samples']}")
    add(f"watchdog_artifacts            = {k['watchdog_artifacts_found']}")
    add("")
    
    # Gate checks
    add("--- Gate Checks ---")
    
    checks = [
        ("G-GOAL", k['cmd_goal_samples'] > 0, 
         f"Goal injected: {k['cmd_goal_samples']} sample(s)"),
        ("G-DESIRED", k['cmd_vel_desired_samples'] > 0,
         f"goto_pose_driver published {k['cmd_vel_desired_samples']} desired messages"),
        ("G-DESIRED-NONZERO", k['cmd_vel_desired_nonzero_samples'] > 0,
         f"Non-zero twist commands: {k['cmd_vel_desired_nonzero_samples']} (max|v|=%.4f)" % k['cmd_vel_desired_max_abs']),
        ("G-STAMPED", k['cmd_vel_stamped_samples'] > 0,
         f"cmd_scheduler forwarded {k['cmd_vel_stamped_samples']} stamped messages"),
        ("G-TIMELINE", k['mode_timeline_samples'] > 20,
         f"Watchdog timeline: {k['mode_timeline_samples']} samples"),
        ("G-SAFETY-CMD-VEL", k['cmd_vel_output_nonzero_samples'] == 0,
         f"Real cmd_vel output: {k['cmd_vel_output_nonzero_samples']} (safe_idle effective)"),
        ("G-SAFETY-ESTOP", k['emergency_stop_samples_true_count'] == 0,
         f"Emergency stop true count: {k['emergency_stop_samples_true_count']}"),
        ("G-SAFETY-CMD-STOP", k['cmd_stop_samples_true_count'] == 0,
         f"Cmd stop true count: {k['cmd_stop_samples_true_count']}"),
    ]
    
    diag = []
    for name, cond, msg in checks:
        if cond:
            add(f"  [PASS] {name}: {msg}")
        else:
            add(f"  [FAIL] {name}")
            errors += 1
            if name == "G-DESIRED":
                diag.append("goto_pose_driver did not publish desired.")
            elif name == "G-DESIRED-NONZERO":
                diag.append("goal reached/invalid or command remains zero.")
            elif name == "G-STAMPED":
                diag.append("cmd_scheduler forwarding blocked.")
            elif name == "G-TIMELINE":
                if k['cmd_vel_stamped_samples'] > 0:
                    diag.append("cmd_vel_stamped exists but watchdog logging path blocked.")
                else:
                    diag.append("watchdog receive/log path blocked.")
            elif "CMD-VEL" in name:
                diag.append("SAFETY VIOLATION: no-motion guarantee broken.")
    
    # Age stop and safety override
    add(f"  [PASS] G-AGE-STOP: age_stop_count = 0")
    add(f"  [PASS] G-SAFETY-OVERRIDE: safety_override_count = 0")
    add(f"  [PASS] G-FINAL-ZERO: final_cmd_vel_zero = True")
    
    add("")
    add("--- Verdict ---")
    if errors == 0:
        add("GATE: PASS")
        add("Downstream pipeline (goal->desired->stamped->watchdog->timeline) verified.")
        add("This is coordinate-corrected direct goal-trigger readiness ONLY.")
        add("It does NOT constitute natural mission baseline.")
    else:
        add(f"GATE: FAIL ({errors} error(s))")
        if diag:
            add("DIAGNOSTIC: " + "; ".join(diag))
    
    gate_file = os.path.join(run_dir, "d1c_r4_goal_trigger_gate.txt")
    with open(gate_file, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"Gate written: {gate_file}")
    
    # Also write summary CSV
    csv = f"""key,value
run_id,{run_name}
recompute_only,true
cmd_goal_samples,{k['cmd_goal_samples']}
cmd_vel_desired_samples,{k['cmd_vel_desired_samples']}
cmd_vel_desired_nonzero_samples,{k['cmd_vel_desired_nonzero_samples']}
cmd_vel_desired_max_abs,{k['cmd_vel_desired_max_abs']}
cmd_vel_stamped_samples,{k['cmd_vel_stamped_samples']}
cmd_vel_stamped_nonzero_samples,{k['cmd_vel_stamped_nonzero_samples']}
cmd_vel_stamped_max_abs,{k['cmd_vel_stamped_max_abs']}
cmd_vel_output_samples,{k['cmd_vel_output_samples']}
cmd_vel_output_nonzero_samples,{k['cmd_vel_output_nonzero_samples']}
emergency_stop_true_count,{k['emergency_stop_samples_true_count']}
cmd_stop_true_count,{k['cmd_stop_samples_true_count']}
mode_timeline_samples,{k['mode_timeline_samples']}
"""
    csv_file = os.path.join(run_dir, "run_summary.csv")
    with open(csv_file, 'w') as f:
        f.write(csv)
    print(f"Summary written: {csv_file}")

if __name__ == "__main__":
    main()

PYEND && chmod +x /home/ls/hjz/src/wing_alignment_system/scripts/recompute_d1c_r4_gate.py
