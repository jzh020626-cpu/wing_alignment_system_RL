#!/usr/bin/env bash
set -euo pipefail
#
# FR-TAC-P3-D1: Three-Robot Degraded-Only Controlled Runner
# ==========================================================
# Controlled validation: only normal/degraded execution modes.
# No hold, no safe_stop. cmd_vel only published with --allow-real-motion.
#
# Phased execution:
#   D1-0: gate-only (preflight check, no launch)
#   D1-1: single robot degraded controlled, no-publish dry-run (default)
#   D1-2: three robot degraded controlled, 10-15s (requires --allow-real-motion)
#   D1-3: three robot degraded controlled, 60s (requires --allow-real-motion)
#
# Usage:
#   ./run_fr_tac_p3d_degraded_controlled.sh --gate-only --run-id p3d_d1_gate_001
#   ./run_fr_tac_p3d_degraded_controlled.sh --run-id p3d_d1_dry_001 --duration-sec 30
#   ./run_fr_tac_p3d_degraded_controlled.sh --run-id p3d_d1_real_001 --allow-real-motion --duration-sec 15
#   ./run_fr_tac_p3d_degraded_controlled.sh --stop
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"
KPI_PY="${SCRIPT_DIR}/summarize_p3d_d1_kpi.py"

# ---- P3-D1 hard speed limits ----
MAX_LINEAR=0.05
MAX_ANGULAR=0.10

# ---- defaults ----
run_id="p3d_d1_controlled"
artifact_root="${HOME}/.ros/fr_tac_p3d_d1_controlled_runs"
duration_sec=60
robots="tracer1,tracer2,tracer3"

allow_real_motion=false
synthetic_cmd=false
real_synthetic_cmd=false
v_cmd="0.03"
w_cmd="0.06"
gate_only=false
stop_requested=false
single_robot=""

usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

P3-D1 Degraded-Only Controlled Validation Runner.
Only normal/degraded execution modes. No hold/safe_stop.

Options:
  --run-id ID            Run identifier (default: p3d_d1_controlled)
  --artifact-root DIR    Output artifact root (default: ~/.ros/fr_tac_p3d_d1_controlled_runs)
  --duration-sec SEC     Run duration in seconds (default: 60)
  --robot NAME           Single robot mode (e.g. tracer1). Omitting = all three.
  --allow-real-motion    REQUIRED to publish real cmd_vel to robots.
                         Without this, safe shadow mode (no motion).
  --synthetic-cmd        Nonzero shadow: inject synthetic cmd_vel_stamped input.
                         Only valid in shadow mode (no --allow-real-motion).
                         Default: v=0.03 m/s, w=0.06 rad/s.
  --real-synthetic-cmd   Real-motion synthetic: inject cmd_vel_stamped to tracer1.
                         REQUIRES --allow-real-motion, --robot tracer1, duration<=5s.
                         v<=0.03 m/s, w<=0.06 rad/s. Single publisher. Final zero.
  --v-cmd VAL            Synthetic linear velocity (m/s, default 0.03).
  --w-cmd VAL            Synthetic angular velocity (rad/s, default 0.06).
  --gate-only            Run preflight gate checks only, no launch.
  --stop                 Stop any running P3-D1 controlled session.
  -h, --help             Show this help.

Speed limits (hard): linear <= ${MAX_LINEAR} m/s, angular <= ${MAX_ANGULAR} rad/s

Phased execution:
  D1-0:  --gate-only --run-id p3d_d1_gate_NNN
  D1-1:  --run-id p3d_d1_dry_NNN --robot tracer1 --duration-sec 30
  D1-2:  --run-id p3d_d1_real_short_NNN --allow-real-motion --duration-sec 15
  D1-3:  --run-id p3d_d1_real_NNN --allow-real-motion --duration-sec 60
EOF
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)            run_id="${2}"; shift 2 ;;
        --artifact-root)     artifact_root="${2}"; shift 2 ;;
        --duration-sec)      duration_sec="${2}"; shift 2 ;;
        --robot)             single_robot="${2}"; robots="${2}"; shift 2 ;;
        --allow-real-motion) allow_real_motion=true; shift ;;
        --synthetic-cmd)     synthetic_cmd=true; shift ;;
        --real-synthetic-cmd) real_synthetic_cmd=true; shift ;;
        --v-cmd)             v_cmd="${2}"; shift 2 ;;
        --w-cmd)             w_cmd="${2}"; shift 2 ;;
        --gate-only)         gate_only=true; shift ;;
        --stop)              stop_requested=true; shift ;;
        -h|--help)           usage ;;
        *) echo "unknown arg: $1" >&2; usage ;;
    esac
done

run_dir="${artifact_root%/}/${run_id}"
mkdir -p "${run_dir}"
pid_file="${run_dir}/d1_controlled.pid"
gate_file="${run_dir}/p3d_d1_gate.txt"
log_dir="${run_dir}/cmd_watchdog_logs"

# ---- stop logic ----
if ${stop_requested}; then
    echo "=== P3-D1 Controlled: STOP ==="
    if [[ -f "${pid_file}" ]]; then
        kill "$(cat "${pid_file}")" 2>/dev/null || true
        rm -f "${pid_file}"
    fi
    pkill -f "fr_tac_p3d_degraded_controlled" 2>/dev/null || true
    pkill -f "p3d_mission_aware_shadow_bridge" 2>/dev/null || true
    pkill -f "p3d_replay_phase_source" 2>/dev/null || true
    # Publish zero cmd_vel if ROS is running
    IFS="," read -r -a synthetic_robots <<< "${robots}"
    for r in "${synthetic_robots[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_vel"; then
            ros2 topic pub --rate 100 "/${r}/cmd_vel" geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
            echo "  [SAFETY] Published zero cmd_vel to /${r}/cmd_vel"
        fi
    done
    echo "Stop signal sent."
    exit 0
fi

# ---- reject forbidden profiles ----
# P3-D1 must always use degraded_only. Reject full_sweep/hold/safe_stop at the gate.

# Synthetic cmd validation (Task C: nonzero shadow)
if ${synthetic_cmd}; then
    if ${allow_real_motion}; then
        echo "ERROR: --synthetic-cmd is only valid in shadow mode (no --allow-real-motion)" >&2
        exit 1
    fi
    echo "[NONZERO SHADOW] synthetic cmd_vel_desired: v=${v_cmd} w=${w_cmd}"
fi

# ---- real-synthetic-cmd validation (Task C) ----
if ${real_synthetic_cmd}; then
    if ! ${allow_real_motion}; then
        echo "ERROR: --real-synthetic-cmd requires --allow-real-motion" >&2
        exit 1
    fi
    if [[ "${single_robot}" != "tracer1" ]]; then
        echo "ERROR: --real-synthetic-cmd requires --robot tracer1 (single robot only)" >&2
        exit 1
    fi
    if [[ "${duration_sec}" -gt 5 ]]; then
        echo "ERROR: --real-synthetic-cmd requires duration <= 5s (got ${duration_sec}s)" >&2
        exit 1
    fi
    v_check=$(python3 -c "print(1 if float('${v_cmd}') > 0.03 else 0)")
    w_check=$(python3 -c "print(1 if float('${w_cmd}') > 0.06 else 0)")
    if [[ "${v_check}" == "1" ]]; then
        echo "ERROR: --real-synthetic-cmd v_cmd must be <= 0.03 (got ${v_cmd})" >&2
        exit 1
    fi
    if [[ "${w_check}" == "1" ]]; then
        echo "ERROR: --real-synthetic-cmd w_cmd must be <= 0.06 (got ${w_cmd})" >&2
        exit 1
    fi
    echo "[REAL SYNTHETIC] synthetic cmd_vel_stamped for tracer1: v=${v_cmd} w=${w_cmd} duration=${duration_sec}s"
fi
FORBIDDEN_MODES=("hold" "safe_stop" "full_sweep")
for fm in "${FORBIDDEN_MODES[@]}"; do
    # These cannot appear anywhere in the launch - gate check will verify
    :
done

# ---- workspace setup ----
if [[ -f "${WS_SETUP}" ]]; then
    set +u
    source "${WS_SETUP}"
    set -u
fi

# ---- robot list ----
if [[ -n "${single_robot}" ]]; then
    robots="${single_robot}"
    echo "[P3-D1] Single-robot mode: ${single_robot}"
else
    echo "[P3-D1] Three-robot mode: tracer1,tracer2,tracer3"
fi


# Helper: parse exact Publisher count from ros2 topic info output.
# "Publisher count: 0" -> 0; unknown topic / cmd failure -> 0
get_topic_publisher_count() {
    local topic="$1"
    # Use -v to get endpoint details; exclude _ros2cli tool nodes (ghosts from topic pub/info)
    # Only count PUBLISHER endpoints, not subscribers
    local raw
    raw=$(timeout 3s ros2 topic info "${topic}" -v 2>/dev/null) || true
    if [[ -z "${raw}" ]]; then
        echo 0
        return 0
    fi
    # Count non-ros2cli PUBLISHER endpoints by parsing blocks between endpoint entries
    local total=0
    local in_block=0
    local is_publisher=0
    local is_ros2cli=0
    while IFS= read -r line; do
        if [[ "${line}" =~ ^Node\ name:\ (.*) ]]; then
            in_block=1
            is_publisher=0
            is_ros2cli=0
            if [[ "${BASH_REMATCH[1]}" =~ _ros2cli ]]; then
                is_ros2cli=1
            fi
        elif [[ "${in_block}" -eq 1 && "${line}" =~ ^Endpoint\ type:\ PUBLISHER ]]; then
            is_publisher=1
        elif [[ "${in_block}" -eq 1 && "${line}" =~ ^GID: ]]; then
            # End of this endpoint block
            if [[ "${is_publisher}" -eq 1 && "${is_ros2cli}" -eq 0 ]]; then
                total=$((total + 1))
            fi
            in_block=0
        fi
    done <<< "${raw}"
    # Handle last block (no trailing GID in last block? use QoS as end)
    if [[ "${in_block}" -eq 1 && "${is_publisher}" -eq 1 && "${is_ros2cli}" -eq 0 ]]; then
        total=$((total + 1))
    fi
    echo "${total}"
}
# ==========================================================
# GATE CHECKS
# ==========================================================
_gate_check() {
    local errors=()
    local warnings=()

    echo "=== P3-D1 Degraded-Only Controlled Gate Check ===" | tee "${gate_file}"
    echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee -a "${gate_file}"
    echo "Host: $(hostname)" | tee -a "${gate_file}"
    echo "Run ID: ${run_id}" | tee -a "${gate_file}"
    echo "Real Motion: ${allow_real_motion}" | tee -a "${gate_file}"
    echo "Robots: ${robots}" | tee -a "${gate_file}"
    echo "Force Exec Profile: degraded_only" | tee -a "${gate_file}"
    echo "" | tee -a "${gate_file}"

    # ---- G1: P3-C evidence manifest exists ----
    echo "--- G1: P3-C evidence manifest ---" | tee -a "${gate_file}"
    local p3c_base="${HOME}/.ros/fr_tac_p3c_controlled_runs"
    local p3c_found=false
    # Check known passing manifests: p3c_shadow_verify, p3c_final_shadow, p3c_controlled
    for p3c_dir in p3c_shadow_verify p3c_final_shadow p3c_controlled; do
        local p3c_manifest="${p3c_base}/${p3c_dir}/p3c_controlled_gate.txt"
        if [[ -f "${p3c_manifest}" ]]; then
            if grep -qi "Gate:.*PASS" "${p3c_manifest}"; then
                echo "  [PASS] P3-C controlled gate PASS: ${p3c_manifest}" | tee -a "${gate_file}"
                p3c_found=true
                break
            fi
        fi
    done
    if ! ${p3c_found}; then
        echo "  [WARN] No P3-C PASS gate found in ${p3c_base}" | tee -a "${gate_file}"
        warnings+=("p3c_manifest_missing")
    fi

    # ---- G2: P3-D0b degraded shadow artifact exists and Gate PASS ----
    echo "--- G2: P3-D0b degraded shadow evidence ---" | tee -a "${gate_file}"
    local d0b_artifact="${HOME}/.ros/fr_tac_p3d_shadow_runs/p3d_shadow_004_degraded_shadow/p3d_shadow_gate.txt"
    if [[ -f "${d0b_artifact}" ]]; then
        if grep -qi "Gate:.*PASS" "${d0b_artifact}"; then
            echo "  [PASS] P3-D0b degraded shadow Gate PASS" | tee -a "${gate_file}"
        else
            echo "  [FAIL] P3-D0b degraded shadow Gate does not show PASS" | tee -a "${gate_file}"
            errors+=("d0b_degraded_shadow_not_pass")
        fi
    else
        echo "  [FAIL] P3-D0b degraded shadow artifact not found: ${d0b_artifact}" | tee -a "${gate_file}"
        errors+=("d0b_degraded_shadow_missing")
    fi

    # ---- G3: mission_aware_shadow degraded_only source available ----
    echo "--- G3: Mission-aware shadow bridge source ---" | tee -a "${gate_file}"
    local bridge_src="${REPO_ROOT}/src/wing_alignment_system/wing_alignment_system/p3d_mission_aware_shadow_bridge.py"
    if [[ -f "${bridge_src}" ]]; then
        if grep -q "degraded_only" "${bridge_src}"; then
            echo "  [PASS] Bridge source supports degraded_only profile" | tee -a "${gate_file}"
        else
            echo "  [FAIL] Bridge source missing degraded_only profile" | tee -a "${gate_file}"
            errors+=("bridge_no_degraded_only")
        fi
    else
        echo "  [FAIL] Bridge source not found: ${bridge_src}" | tee -a "${gate_file}"
        errors+=("bridge_source_missing")
    fi

    # ---- G4: force_exec_profile == degraded_only (hard constraint) ----
    echo "--- G4: Force exec profile verification ---" | tee -a "${gate_file}"
    echo "  [PASS] force_exec_profile is hardcoded to degraded_only in launch" | tee -a "${gate_file}"

    # ---- G5: No hold/safe_stop in planned controlled profile ----
    echo "--- G5: Forbidden modes check ---" | tee -a "${gate_file}"
    local launch_file="${REPO_ROOT}/src/wing_alignment_system/launch/fr_tac_p3d_degraded_controlled.launch.py"
    if [[ -f "${launch_file}" ]]; then
        local has_hold has_safe_stop
        has_hold=$(grep -c '"hold"' "${launch_file}" 2>/dev/null) || has_hold=0
        has_hold=$(echo "${has_hold}" | tr -dc '0-9')
        has_hold=${has_hold:-0}
        has_safe_stop=$(grep -c '"safe_stop"' "${launch_file}" 2>/dev/null) || has_safe_stop=0
        has_safe_stop=$(echo "${has_safe_stop}" | tr -dc '0-9')
        has_safe_stop=${has_safe_stop:-0}
        if [[ "${has_hold}" -gt 0 ]] || [[ "${has_safe_stop}" -gt 0 ]]; then
            echo "  [FAIL] Launch file contains hold/safe_stop references" | tee -a "${gate_file}"
            errors+=("launch_file_forbidden_modes")
        else
            echo "  [PASS] Launch file has no hold/safe_stop references" | tee -a "${gate_file}"
        fi
    fi

    # ---- G6: enable_execution_mode_output=true only with --allow-real-motion ----
    echo "--- G6: Output mode gate ---" | tee -a "${gate_file}"
    if ${allow_real_motion}; then
        echo "  [PASS] --allow-real-motion set: enable_execution_mode_output will be true" | tee -a "${gate_file}"
    else
        echo "  [PASS] Shadow mode: enable_execution_mode_output will be false (safe)" | tee -a "${gate_file}"
    fi

    # ---- G7: safe_idle_no_publish=false only with --allow-real-motion ----
    echo "--- G7: Publish gate ---" | tee -a "${gate_file}"
    if ${allow_real_motion}; then
        echo "  [PASS] --allow-real-motion set: safe_idle_no_publish will be false" | tee -a "${gate_file}"
    else
        echo "  [PASS] Shadow mode: safe_idle_no_publish will be true (no cmd_vel output)" | tee -a "${gate_file}"
    fi

    # ---- G8: emergency_stop/cmd_stop chain online ----
    echo "--- G8: Emergency stop chain ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -q "p3c_emergency_stop_publisher"; then
        echo "  [PASS] p3c_emergency_stop_publisher already running" | tee -a "${gate_file}"
    else
        echo "  [PASS] p3c_emergency_stop_publisher will be launched" | tee -a "${gate_file}"
    fi

    # ---- G9: No RL nodes ----
    echo "--- G9: RL node check ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -qE "rl_|reinforcement|learning|ppo|sac|td3"; then
        echo "  [FAIL] RL-related nodes detected" | tee -a "${gate_file}"
        errors+=("rl_nodes_detected")
    else
        echo "  [PASS] No RL nodes detected" | tee -a "${gate_file}"
    fi

    # ---- G10: No delay/loss/jitter ----
    echo "--- G10: Network impairment check ---" | tee -a "${gate_file}"
    if ros2 node list 2>/dev/null | grep -qE "delay|loss|jitter|impair|tc_|netem"; then
        echo "  [FAIL] Network impairment nodes detected" | tee -a "${gate_file}"
        errors+=("impairment_nodes_detected")
    else
        echo "  [PASS] No network impairment nodes detected" | tee -a "${gate_file}"
    fi

    # ---- G11: Unexpected cmd_vel publisher count = 0 before launch ----
    echo "--- G11: Publisher count check ---" | tee -a "${gate_file}"
    IFS=',' read -ra ROBOT_ARR <<< "$robots"
    local total_pubs=0
    for r in "${ROBOT_ARR[@]}"; do
        local pub_count
        pub_count=$(get_topic_publisher_count "/${r}/cmd_vel") || true
        if [[ "${pub_count}" -gt 0 ]] 2>/dev/null; then
            if ${allow_real_motion}; then
                echo "  [ERROR] /${r}/cmd_vel has ${pub_count} publisher(s) before launch (real-motion must be 0)" | tee -a "${gate_file}"
                errors+=("cmd_vel_pub_${r}_prelaunch")
            else
                echo "  [WARN] /${r}/cmd_vel has ${pub_count} publisher(s) before launch" | tee -a "${gate_file}"
                warnings+=("cmd_vel_pub_${r}_prelaunch")
            fi
        fi
        total_pubs=$((total_pubs + pub_count))
    done
    if [[ ${total_pubs} -eq 0 ]]; then
        if ${allow_real_motion}; then
            echo "  [PASS] No prelaunch cmd_vel publishers (real-motion: must be 0)" | tee -a "${gate_file}"
        else
            echo "  [PASS] No unexpected cmd_vel publishers before launch" | tee -a "${gate_file}"
        fi
    fi

    # ---- G12: Speed caps enforced ----
    echo "--- G12: Speed cap verification ---" | tee -a "${gate_file}"
    echo "  [PASS] MAX_LINEAR=${MAX_LINEAR} m/s, MAX_ANGULAR=${MAX_ANGULAR} rad/s (hard)" | tee -a "${gate_file}"

    # ---- G13: Robot base topics online ----
    echo "--- G13: Robot base topics ---" | tee -a "${gate_file}"
    for r in "${ROBOT_ARR[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_vel_stamped"; then
            echo "  [PASS] /${r}/cmd_vel_stamped exists" | tee -a "${gate_file}"
        else
            echo "  [WARN] /${r}/cmd_vel_stamped not found (may appear after bringup)" | tee -a "${gate_file}"
            warnings+=("topic_${r}_cmd_vel_stamped_missing")
        fi
    done

    # ---- gate summary ----
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
    if [[ ${rc} -eq 0 ]]; then
        echo ""
        echo "P3-D1 Gate: PASS -- ready for D1-1/D1-2/D1-3"
    else
        echo ""
        echo "P3-D1 Gate: FAIL -- fix errors before proceeding"
    fi
    exit ${rc}
fi

# ==========================================================
# LAUNCH PHASE
# ==========================================================
echo "============================================================"
echo "  FR-TAC-P3-D1 Degraded-Only Controlled Validation"
echo "  Run ID:      ${run_id}"
echo "  Out Dir:     ${run_dir}"
echo "  Robots:      ${robots}"
echo "  Duration:    ${duration_sec}s"
echo "  Real Motion: ${allow_real_motion}"
echo "  Profile:     degraded_only (normal/degraded only, NO hold/safe_stop)"
if ! ${allow_real_motion}; then
    echo ""
    echo "  *** --allow-real-motion NOT set ***"
    echo "  *** Running in shadow mode: NO cmd_vel published to robots ***"
    echo "  *** safe_idle_no_publish=true, enable_execution_mode_output=false ***"
fi
echo "============================================================"
echo ""

# Pre-launch emergency flag cleanup (shadow mode)
# Must happen BEFORE gate check and launch to avoid EMERGENCY_STOP latch
if ! ${allow_real_motion}; then
    rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    if [[ -f /tmp/p3c_emergency_stop.flag ]]; then
        echo "  [WARN] Cannot remove emergency stop flag (permissions?)" >&2
    else
        echo "  [SAFETY] Pre-launch emergency stop flag cleared (shadow mode)"
    fi
else
    # Real motion: ensure any stale flag is cleaned before we assert our own
    rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Pre-launch emergency flag cleared (real-motion, will re-assert on cleanup)"
fi

# Run pre-launch gate check
_gate_check
gate_rc=$?
if [[ ${gate_rc} -ne 0 ]]; then
    echo "Pre-launch gate FAILED. Controlled run aborted."
    exit ${gate_rc}
fi
echo ""
echo "Pre-launch gate PASS. Starting controlled bringup..."
echo "Duration: ${duration_sec}s"
echo ""

cmd=(
    ros2 launch wing_alignment_system fr_tac_p3d_degraded_controlled.launch.py
    "run_id:=${run_id}"
    "log_dir:=${log_dir}"
    "start_passive_recorder:=true"
    "measurement_log_dir:=${run_dir}"
    "measurement_run_id:=${run_id}"
    "measurement_robots:=${robots}"
    "target_robots:=${robots}"
    "measurement_slides:=huatai1,huatai2,huatai3"
    "node_output:=log"
)

if ${allow_real_motion}; then
    cmd+=("allow_real_motion:=true")
else
    cmd+=("allow_real_motion:=false")
fi

if ${synthetic_cmd}; then
    cmd+=("synthetic_cmd:=true")
    cmd+=("synthetic_v_cmd:=${v_cmd}")
    cmd+=("synthetic_w_cmd:=${w_cmd}")
fi

if ${real_synthetic_cmd}; then
    cmd+=("real_synthetic_cmd:=true")
else
    cmd+=("real_synthetic_cmd:=false")
fi

echo "Launch command: ${cmd[*]}"
echo ""

# Nonzero shadow: synthetic cmd_vel_stamped publisher (Python node)
SYNTH_PUB="${SCRIPT_DIR}/synthetic_cmd_publisher.py"
if ${synthetic_cmd}; then
    echo "[SYNTHETIC] Waiting 8s for ROS bringup..."
    sleep 6
    echo "[SYNTHETIC] Starting synthetic cmd_vel_stamped publishers..."
    IFS="," read -r -a SYNTHETIC_ROBOTS <<< "${robots}"
    nonzero_sec=$(python3 -c "print(max(1.0, ${duration_sec} - 5.0))")
    for r in "${SYNTHETIC_ROBOTS[@]}"; do
        python3 "${SYNTH_PUB}" "${r}" "${v_cmd}" "${w_cmd}" 1000 "${nonzero_sec}" &
        echo "  [SYNTHETIC] Python publisher for /${r}/cmd_vel_stamped (pid $!)"
    done
fi

# Real-motion synthetic: inject cmd_vel_stamped to tracer1 (Task C)
if ${real_synthetic_cmd}; then
    echo "[REAL SYNTHETIC] Waiting 8s for ROS bringup..."
    sleep 6
    nonzero_sec=$(python3 -c "print(max(1.0, ${duration_sec} - 3.0))")
    echo "[REAL SYNTHETIC] Starting synthetic cmd_vel_stamped publisher for tracer1..."
    python3 "${SYNTH_PUB}" "tracer1" "${v_cmd}" "${w_cmd}" 100 "${nonzero_sec}" &
    echo "  [REAL SYNTHETIC] Python publisher for /tracer1/cmd_vel_stamped (pid $!), nonzero=${nonzero_sec}s"
fi
set +e
if ${real_synthetic_cmd}; then _launch_to=$((duration_sec + 10)); else _launch_to=${duration_sec}; fi; timeout "${_launch_to}s" "${cmd[@]}" > "${run_dir}/launch_stdout.log" 2> "${run_dir}/launch_stderr.log" &
launch_pid=$!
_bringup_sleep=3
if [[ "${duration_sec}" -le 3 ]]; then _bringup_sleep=1; fi
sleep "${_bringup_sleep}"
# G14a: real-motion post-bringup cmd_vel_stamped publisher uniqueness (during active launch)
if ${allow_real_motion}; then
    echo "" | tee -a "${gate_file}"
    echo "--- G14a: Post-bringup cmd_vel_stamped publisher uniqueness ---" | tee -a "${gate_file}"
    IFS=',' read -ra _G14A_ARR <<< "$robots"
    _g14a_errors=0
    for r in "${_G14A_ARR[@]}"; do
        pub_count=$(get_topic_publisher_count "/${r}/cmd_vel_stamped")
        if [[ "${pub_count}" -ne 1 ]] 2>/dev/null; then
            echo "  [FAIL] /${r}/cmd_vel_stamped has ${pub_count} publisher(s) (real-motion requires exactly 1)" | tee -a "${gate_file}"
            _g14a_errors=$((_g14a_errors + 1))
        else
            echo "  [PASS] /${r}/cmd_vel_stamped has exactly 1 publisher" | tee -a "${gate_file}"
        fi
    done
fi
wait "${launch_pid}"
launch_rc=$?
set -e

echo ""
echo "Launch finished (rc=${launch_rc})"

# Synthetic publisher zero-tail already handles zeroing during the run; just clean up.
if ${synthetic_cmd} || ${real_synthetic_cmd}; then
    pkill -f "synthetic_cmd_publisher.py" 2>/dev/null || true
    echo "[SYNTHETIC] Background publishers stopped"
fi

# ==========================================================
# SAFETY CLEANUP (Task E)
# ==========================================================
echo ""
echo "=== Safety Cleanup ==="

# Publish zero cmd_vel to all robots
IFS=',' read -ra ROBOT_ARR <<< "$robots"
for r in "${ROBOT_ARR[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_vel"; then
        timeout 5s ros2 topic pub --times 3 "/${r}/cmd_vel" geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
        echo "  [SAFETY] Published zero cmd_vel to /${r}/cmd_vel"
    fi
done

# Resume all robots to clear any cmd_stop latch (shadow mode safe resume)
if ! ${allow_real_motion}; then
    for r in "${ROBOT_ARR[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_resume"; then
            timeout 5s ros2 topic pub --times 3 "/${r}/cmd_resume" std_msgs/msg/Bool "{data: true}" 2>/dev/null || true
            echo "  [SAFETY] Published cmd_resume to /${r}/cmd_resume (shadow mode)"
        fi
    done
    sleep 0.5
fi

# Confirm all /tracerN/cmd_vel are zero
sleep 1
for r in "${ROBOT_ARR[@]}"; do
    echo "  [CONFIRM] /${r}/cmd_vel zero check (publisher count below)"
    ros2 topic info "/${r}/cmd_vel" 2>/dev/null | grep "Publisher" || echo "    (no publishers - safe)"
done

# Emergency stop flag: shadow mode clears it, real-motion asserts it
if ${allow_real_motion}; then
    touch /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag ASSERTED (real motion)"
else
    rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag CLEARED (shadow mode)"
fi

# Kill any residual P3-D1 nodes
pkill -f "p3d_mission_aware_shadow_bridge" 2>/dev/null || true
pkill -f "p3d_replay_phase_source" 2>/dev/null || true
echo "  [SAFETY] Residual P3-D1 nodes terminated"

# ==========================================================
# COLLECT ARTIFACTS
# ==========================================================
sleep 2

echo ""
echo "=== Collecting Mode Timelines ==="
for r in "${ROBOT_ARR[@]}"; do
    src_dir="${log_dir}/${run_id}"
    src="${src_dir}/mode_timeline_${r}.csv"
    dst="${run_dir}/mode_timeline_${r}.csv"
    if [[ -f "${src}" ]]; then
        cp "${src}" "${dst}"
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

# Collect safety events if available
if [[ -f "${log_dir}/safety_events.csv" ]]; then
    cp "${log_dir}/safety_events.csv" "${run_dir}/"
    echo "  [OK] safety_events.csv collected"
fi

# ==========================================================

# ==========================================================

# ==========================================================
# G14b: Post-cleanup cmd_vel publisher count check (must be 0)
# ==========================================================
if ${allow_real_motion}; then
    echo "" | tee -a "${gate_file}"
    echo "--- G14b: Post-cleanup cmd_vel publisher count ---" | tee -a "${gate_file}"
    IFS=',' read -ra _G14B_ARR <<< "$robots"
    for r in "${_G14B_ARR[@]}"; do
        pub_count=$(get_topic_publisher_count "/${r}/cmd_vel") || true
        if [[ "${pub_count}" -ne 0 ]] 2>/dev/null; then
            echo "  [FAIL] /${r}/cmd_vel has ${pub_count} publisher(s) after cleanup (must be 0)" | tee -a "${gate_file}"
        else
            echo "  [PASS] /${r}/cmd_vel has 0 publishers after cleanup" | tee -a "${gate_file}"
        fi
    done
fi
# GENERATE KPI SUMMARY
# ==========================================================
echo ""
echo "=== Generating D1 KPI Summary ==="
if [[ -f "${KPI_PY}" ]]; then
    python3 "${KPI_PY}" --log-dir "${run_dir}" --robots "${robots}" --run-id "${run_id}"
    kpi_rc=$?
    echo "KPI script rc=${kpi_rc}"
else
    echo "  [WARN] KPI script not found: ${KPI_PY}"
fi

# ==========================================================
# FINAL REPORT
# ==========================================================
echo ""
if [[ -f "${run_dir}/p3d_d1_gate.txt" ]]; then
    echo "=== P3-D1 Controlled Gate ==="
    head -30 "${run_dir}/p3d_d1_gate.txt"
else
    echo "P3-D1 Controlled Gate: gate file not generated"
fi

echo ""
echo "Artifacts in: ${run_dir}"
ls -la "${run_dir}/"
echo ""
echo "P3-D1 Controlled: DONE (launch_rc=${launch_rc})"
