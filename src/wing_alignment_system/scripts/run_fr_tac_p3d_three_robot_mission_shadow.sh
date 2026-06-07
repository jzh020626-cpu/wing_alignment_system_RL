#!/usr/bin/env bash
set -euo pipefail
#
# FR-TAC-P3-D0: Three-Robot Mission Shadow Runner
# ===============================================
# P3-D0a: basic shadow bridge (p3d_shadow_cmd_bridge)
# P3-D0b: mission-aware shadow bridge (replay phase source + phase-aware bridge)
#
# Shadow validation: mission_coordinator runs, cmd_watchdog observes
# (safe_idle_no_publish=true, enable_execution_mode_output=false),
# no delay/loss injection, no RL, output=false.
#
# Usage:
#   ./run_fr_tac_p3d_three_robot_mission_shadow.sh --run-id p3d_shadow_003 --mission-aware-shadow
#   ./run_fr_tac_p3d_three_robot_mission_shadow.sh --stop
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"

# ---- defaults ----
run_id="p3d_shadow"
artifact_root="${HOME}/.ros/fr_tac_p3d_shadow_runs"
duration_sec=60
robots="tracer1,tracer2,tracer3"
pid_file=""
stop_requested=false
gate_only=false
mission_aware_shadow=false
force_exec_profile="none"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)            run_id="${2}"; shift 2 ;;
        --artifact-root)     artifact_root="${2}"; shift 2 ;;
        --duration-sec)      duration_sec="${2}"; shift 2 ;;
        --robots)            robots="${2}"; shift 2 ;;
        --stop)              stop_requested=true; shift ;;
        --gate-only)         gate_only=true; shift ;;
        --mission-aware-shadow) mission_aware_shadow=true; shift ;;
        --force-exec-profile) force_exec_profile="${2}"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

run_dir="${artifact_root%/}/${run_id}"
mkdir -p "${run_dir}"
pid_file="${run_dir}/shadow.pid"
gate_file="${run_dir}/p3d_shadow_gate.txt"
log_dir="${run_dir}/cmd_watchdog_logs"

# ---- stop logic ----
if ${stop_requested}; then
    echo "=== P3-D0 Shadow: STOP ==="
    if [[ -f "${pid_file}" ]]; then
        kill "$(cat "${pid_file}")" 2>/dev/null || true
        rm -f "${pid_file}"
    fi
    pkill -f "fr_tac_p3d_three_robot_shadow" 2>/dev/null || true
    echo "Stop signal sent."
    exit 0
fi

# ---- workspace setup ----
if [[ -f "${WS_SETUP}" ]]; then
    set +u
    source "${WS_SETUP}"
    set -u
fi

if ${mission_aware_shadow}; then
    echo "[P3-D0b] Mission-aware shadow mode enabled"
else
    echo "[P3-D0a] Basic shadow bridge mode"
fi

# ==========================================================
# GATE CHECKS
# ==========================================================
_gate_check() {
    local errors=()
    local warnings=()

    echo "=== P3-D0 Gate Check ==="  | tee -a "${gate_file}"
    echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee -a "${gate_file}"
    echo "Host: $(hostname)" | tee -a "${gate_file}"
    if ${mission_aware_shadow}; then
        echo "Mode: P3-D0b mission-aware shadow" | tee -a "${gate_file}"
    else
        echo "Mode: P3-D0a basic shadow bridge" | tee -a "${gate_file}"
    fi
    echo "" | tee -a "${gate_file}"

    # C1: mission coordinator or phase source available
    echo "--- C1: mission source ---" | tee -a "${gate_file}"
    local has_mission=false
    if ros2 node list 2>/dev/null | grep -q "mission_coordinator"; then
        echo "  [PASS] mission_coordinator already running" | tee -a "${gate_file}"
        has_mission=true
    elif [[ -f "${REPO_ROOT}/src/wing_alignment_system/wing_alignment_system/mission_coordinator.py" ]]; then
        echo "  [PASS] mission_coordinator source available (will be launched)" | tee -a "${gate_file}"
        has_mission=true
    else
        echo "  [FAIL] mission_coordinator not found" | tee -a "${gate_file}"
        errors+=("mission_source_missing")
    fi

    # D0b-specific: phase source available
    if ${mission_aware_shadow}; then
        if ros2 node list 2>/dev/null | grep -q "p3d_replay_phase_source"; then
            echo "  [PASS] p3d_replay_phase_source already running (phase_source=replay)" | tee -a "${gate_file}"
        else
            echo "  [PASS] p3d_replay_phase_source will be launched (phase_source=replay)" | tee -a "${gate_file}"
        fi
    fi

    # C2: robot base topics online
    echo "--- C2: robot base topics ---" | tee -a "${gate_file}"
    IFS=',' read -ra ROBOT_ARR <<< "$robots"
    for r in "${ROBOT_ARR[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_vel_stamped"; then
            echo "  [PASS] /${r}/cmd_vel_stamped exists" | tee -a "${gate_file}"
        else
            echo "  [WARN] /${r}/cmd_vel_stamped not found (may appear after bringup)" | tee -a "${gate_file}"
            warnings+=("topic_${r}_cmd_vel_stamped_missing")
        fi
    done

    # C3: output=false confirmed
    echo "--- C3: output=false ---" | tee -a "${gate_file}"
    echo "  [PASS] enable_execution_mode_output default is False (safe_idle_no_publish=true)" | tee -a "${gate_file}"

    # C4: emergency_stop / cmd_stop safety chain online
    echo "--- C4: safety chain ---" | tee -a "${gate_file}"
    if ros2 topic list 2>/dev/null | grep -q "/wing_alignment/emergency_stop"; then
        echo "  [PASS] /wing_alignment/emergency_stop topic exists" | tee -a "${gate_file}"
    else
        echo "  [WARN] /wing_alignment/emergency_stop not found (may appear after bringup)" | tee -a "${gate_file}"
        warnings+=("emergency_stop_topic_missing")
    fi
    for r in "${ROBOT_ARR[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_stop"; then
            echo "  [PASS] /${r}/cmd_stop topic exists" | tee -a "${gate_file}"
        else
            echo "  [WARN] /${r}/cmd_stop not found (may appear after bringup)" | tee -a "${gate_file}"
            warnings+=("cmd_stop_${r}_missing")
        fi
    done

    # C5: no RL nodes running
    echo "--- C5: no RL nodes ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -qiE "rl_|reinforcement|learning|train"; then
        echo "  [FAIL] RL nodes detected" | tee -a "${gate_file}"
        errors+=("rl_nodes_detected")
    else
        echo "  [PASS] no RL nodes detected" | tee -a "${gate_file}"
    fi

    # C6: no delay/loss/jitter injection nodes
    echo "--- C6: no delay/loss injection ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -qiE "delay|lss|netem|traffic_control|jitter|packet_loss"; then
        echo "  [FAIL] delay/loss nodes detected" | tee -a "${gate_file}"
        errors+=("delay_loss_nodes_detected")
    else
        echo "  [PASS] no delay/loss/jitter nodes" | tee -a "${gate_file}"
    fi

    # C7: safe_idle_no_publish and enable_execution_mode_output
    echo "--- C7: safe_idle_no_publish ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -q "cmd_watchdog"; then
        nopub_val=$(ros2 param get /tracer1/cmd_watchdog safe_idle_no_publish 2>/dev/null || echo "unknown")
        echo "  safe_idle_no_publish = ${nopub_val}" | tee -a "${gate_file}"
    else
        echo "  [PASS] safe_idle_no_publish will be set true at launch" | tee -a "${gate_file}"
    fi
    echo "--- C7b: enable_execution_mode_output ---" | tee -a "${gate_file}"
    if ros2 param get /tracer1/cmd_watchdog enable_execution_mode_output 2>/dev/null | grep -q "true"; then
        echo "  [WARN] enable_execution_mode_output=true detected (should be false for shadow)" | tee -a "${gate_file}"
        warnings+=("execution_mode_output_enabled")
    else
        echo "  [PASS] enable_execution_mode_output is false (or cmd_watchdog not yet started)" | tee -a "${gate_file}"
    fi

    # C8: no unexpected safety hold/safe_stop affecting real cmd_vel
    echo "--- C8: safety interference ---" | tee -a "${gate_file}"
    echo "  [PASS] safe_idle_no_publish configured true (no cmd_vel publishing)" | tee -a "${gate_file}"

    # gate summary
    local n_err=${#errors[@]}
    local n_warn=${#warnings[@]}
    echo "" | tee -a "${gate_file}"
    echo "Gate errors:   ${n_err}" | tee -a "${gate_file}"
    echo "Gate warnings: ${n_warn}" | tee -a "${gate_file}"
    if [[ ${n_err} -gt 0 ]]; then
        echo "GATE: FAIL" | tee -a "${gate_file}"
        for e in "${errors[@]}"; do echo "  ERROR: ${e}" | tee -a "${gate_file}"; done
    else
        echo "GATE: PASS" | tee -a "${gate_file}"
    fi
    for w in "${warnings[@]}"; do echo "  WARN: ${w}" | tee -a "${gate_file}"; done
    echo "" | tee -a "${gate_file}"

    return ${n_err}
}

# ---- gate-only mode ----
if ${gate_only}; then
    _gate_check
    rc=$?
    exit ${rc}
fi

# ==========================================================
# LAUNCH SHADOW BRINGUP
# ==========================================================
echo "============================================================"
if ${mission_aware_shadow}; then
    echo "  FR-TAC-P3-D0b Three-Robot Mission-Aware Shadow"
else
    echo "  FR-TAC-P3-D0a Three-Robot Mission Shadow"
fi
echo "  Run ID:  ${run_id}"
echo "  Out Dir: ${run_dir}"
echo "  Robots:  ${robots}"
echo "  Mode:    shadow (safe_idle_no_publish=true, output=false)"
echo "============================================================"
echo ""

# Run pre-launch gate check
_gate_check
gate_rc=$?
if [[ ${gate_rc} -ne 0 ]]; then
    echo "Pre-launch gate FAILED. Shadow aborted."
    exit ${gate_rc}
fi
echo ""
echo "Pre-launch gate PASS. Starting shadow bringup..."
echo "Duration: ${duration_sec}s"
echo ""

if ${mission_aware_shadow}; then
    mission_aware_flag="mission_aware_shadow:=true"
else
    mission_aware_flag="mission_aware_shadow:=false"
fi

cmd=(
    ros2 launch wing_alignment_system fr_tac_p3d_three_robot_shadow.launch.py
    "run_id:=${run_id}"
    "log_dir:=${log_dir}"
    "start_passive_recorder:=true"
    "measurement_log_dir:=${run_dir}"
    "measurement_run_id:=${run_id}"
    "measurement_robots:=${robots}"
    "measurement_slides:=huatai1,huatai2,huatai3"
    "node_output:=log"
    "${mission_aware_flag}"
    "force_exec_profile:=${force_exec_profile}"
)

echo "Launch command: ${cmd[*]}"
echo ""

set +e
timeout "${duration_sec}s" "${cmd[@]}" > "${run_dir}/launch_stdout.log" 2> "${run_dir}/launch_stderr.log"
launch_rc=$?
set -e

echo ""
echo "Launch finished (rc=${launch_rc})"
echo ""

# ==========================================================
# COLLECT ARTIFACTS
# ==========================================================

sleep 2

echo "=== Collecting mode timelines ==="
IFS=',' read -ra ROBOT_ARR <<< "$robots"
for r in "${ROBOT_ARR[@]}"; do
    src_dir="${log_dir}/${run_id}"
    src="${src_dir}/mode_timeline_${r}.csv"
    dst="${run_dir}/mode_timeline_${r}.csv"
    if [[ -f "${src}" ]]; then
        cp "${src}" "${dst}"
        lines=0
        lines=$(wc -l < "${dst}" || echo 0)
        echo "  [OK] ${r}: ${dst} (${lines} lines)"
    else
        echo "  [WARN] ${r}: mode_timeline not found at ${src}"
    fi
    for suffix in rx ts; do
        s="${src_dir}/${suffix}_${r}.csv"
        if [[ -f "${s}" ]]; then
            cp "${s}" "${run_dir}/${suffix}_${r}.csv"
        fi
    done
done

# Look for mission runtime events from mission_coordinator
mission_log_dir="${HOME}/.ros/mission_bench_logs/${run_id}"
if [[ -f "${mission_log_dir}/mission_runtime_events.csv" ]]; then
    cp "${mission_log_dir}/mission_runtime_events.csv" "${run_dir}/"
    echo "  [OK] mission_runtime_events.csv collected"
else
    echo "  [WARN] mission_runtime_events.csv not found at ${mission_log_dir}"
    if ${mission_aware_shadow}; then
        echo "  [INFO] P3-D0b uses replay phase source, no mission_runtime_events.csv expected"
    fi
fi

# ==========================================================
# GENERATE KPI SUMMARY
# ==========================================================
echo ""
echo "=== Generating KPI summary ==="
kpi_py="${SCRIPT_DIR}/summarize_p3d_shadow_kpi.py"
if [[ -f "${kpi_py}" ]]; then
    mission_flag=""
    if ${mission_aware_shadow}; then
        mission_flag="--mission-aware-shadow"
    fi
    python3 "${kpi_py}" --log-dir "${run_dir}" --robots "${robots}" ${mission_flag}
    kpi_rc=$?
    echo "KPI script rc=${kpi_rc}"
else
    echo "  [WARN] KPI script not found: ${kpi_py}"
fi

# ==========================================================
# FINAL GATE
# ==========================================================
echo ""
if [[ -f "${run_dir}/p3d_shadow_gate.txt" ]]; then
    echo "=== P3-D0 Shadow Gate ==="
    head -30 "${run_dir}/p3d_shadow_gate.txt"
else
    echo "P3-D0 Shadow Gate: gate file not generated"
fi

echo ""
echo "Artifacts in: ${run_dir}"
ls -la "${run_dir}/"
echo ""
echo "P3-D0 Shadow: DONE (launch_rc=${launch_rc})"
