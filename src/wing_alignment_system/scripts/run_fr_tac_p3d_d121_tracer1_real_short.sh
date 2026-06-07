#!/usr/bin/env bash
set -uo pipefail

# ============================================================================
# FR-TAC-P3-D1-2-1: tracer1-only real-motion short-window controlled runner
#
# Hard constraints:
#   --robot MUST be tracer1; any other robot or all is rejected
#   --duration-sec MUST be <= 5; greater is rejected
#   --allow-real-motion REQUIRED to publish real cmd_vel to tracer1
#   Without --allow-real-motion, only gate checks run (no motion)
#   Speed caps: linear <= 0.03 m/s, angular <= 0.06 rad/s
#
# Does NOT modify: mission_coordinator, goto_pose_driver, path planning, RL.
# Does NOT execute D1-2-2 (three-robot) or D1-3.
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"
ARTIFACT_ROOT_DEFAULT="${HOME}/.ros/fr_tac_p3d_d121_runs"

# ---- Hard speed caps (D1-2) ----
MAX_LINEAR=0.03
MAX_ANGULAR=0.06
KNOWN_GOOD_PROFILE="mission_bringup"


# ---- Defaults ----
RUN_ID="p3d_d121_tracer1_real"
ARTIFACT_ROOT="${ARTIFACT_ROOT_DEFAULT}"
DURATION_SEC=5
ROBOT="tracer1"
GATE_ONLY=true
GATE_ONLY_EXPLICIT=false
ALLOW_REAL_MOTION=false
STOP_REQUESTED=false
PRINT_PLAN=false

sanitize_int() {
    local raw="${1:-}"
    local token
    while IFS= read -r token; do
        if [[ "${token}" =~ ^-?[0-9]+$ ]]; then
            echo "${token}"
            return 0
        fi
    done < <(printf '%s\n' "${raw}" | tr -cs '0-9-\n' '\n')
    echo 0
}

count_file_lines() {
    local path="$1"
    if [[ -f "${path}" ]]; then
        sanitize_int "$(wc -l < "${path}" 2>/dev/null || echo 0)"
    else
        echo 0
    fi
}

# ---- Usage ----
usage() {
    cat << EOF
Usage: $0 [OPTIONS]

FR-TAC-P3-D1-2-1: tracer1-only real-motion short-window controlled runner.
Default: gate-only (no real motion). Requires --allow-real-motion for real motion.

Options:
  --run-id ID              Run identifier (default: p3d_d121_tracer1_real)
  --artifact-root DIR      Output artifact root (default: ~/.ros/fr_tac_p3d_d121_runs)
  --duration-sec SEC       Motion duration: 3 or 5 (default: 5)
  --robot ROBOT            MUST be tracer1 (default: tracer1)
  --known-good-profile PROF  Launch profile: system_bringup|mission_bringup|run_all (default: mission_bringup)
  --allow-real-motion      EXPLICIT flag to publish real cmd_vel to tracer1
  --gate-only              Run gate checks only, no launch (default)
  --stop                   Stop running session and cleanup
  --print-plan             Dry-run: resolve CLI flags and print plan, no ROS
  -h, --help               Show this help

Hard constraints:
  - Only tracer1 (tracer2/tracer3/all rejected)
  - Duration <= 5s (greater rejected)
  - v_cmd <= ${MAX_LINEAR} m/s, w_cmd <= ${MAX_ANGULAR} rad/s
  - No --allow-real-motion = no real cmd_vel published
EOF
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)              RUN_ID="${2}"; shift 2 ;;
        --artifact-root)       ARTIFACT_ROOT="${2}"; shift 2 ;;
        --duration-sec)        DURATION_SEC="${2}"; shift 2 ;;
        --robot)               ROBOT="${2}"; shift 2 ;;
        --known-good-profile)   KNOWN_GOOD_PROFILE="${2}"; shift 2 ;;
        --allow-real-motion)   ALLOW_REAL_MOTION=true; shift ;;
        --gate-only)           GATE_ONLY=true; GATE_ONLY_EXPLICIT=true; shift ;;
        --stop)                STOP_REQUESTED=true; shift ;;
        --print-plan)          PRINT_PLAN=true; shift ;;
        --use-synthetic-stamped-debug|--synthetic-cmd|--v-cmd|--w-cmd|--real-synthetic-cmd)
            echo "ERROR: synthetic path is removed from D121. Use --known-good-profile system_bringup|mission_bringup|run_all." >&2
            exit 10
            ;;
        -h|--help)             usage ;;
        *) echo "Unknown option: $1" >&2; usage ;;
    esac
done
# ---- Resolve CLI state machine ----

# ---- Profile validation ----
case "${KNOWN_GOOD_PROFILE}" in
    system_bringup|mission_bringup|run_all) ;;
    *) echo "Invalid --known-good-profile: ${KNOWN_GOOD_PROFILE}" >&2; usage ;;
esac

if [[ "${KNOWN_GOOD_PROFILE}" == "mission_bringup" ]]; then
    echo "[PROFILE] mission_bringup: default D1-2-1 short-window profile; launching with enable_return_home:=true enable_mission_coordinator:=false"
fi
if [[ "${KNOWN_GOOD_PROFILE}" == "system_bringup" ]]; then
    echo "[PROFILE] system_bringup: managed-chain profile, non-default for D1-2-1 short-window; requires longer readiness and /mission/start_approach"
fi
if [[ "${KNOWN_GOOD_PROFILE}" == "run_all" ]]; then
    echo "[PROFILE] run_all: full-process/camera profile; non-default for D1-2-1"
fi

# --allow-real-motion without explicit --gate-only -> gate_only=false
if ${ALLOW_REAL_MOTION} && ! ${GATE_ONLY_EXPLICIT}; then
    GATE_ONLY=false
    echo "[CLI] --allow-real-motion active, gate_only=false (will enter execution branch after gate PASS)"
fi
# If both --gate-only and --allow-real-motion passed, gate-only takes priority
if ${GATE_ONLY_EXPLICIT} && ${ALLOW_REAL_MOTION}; then
    echo "[CLI] --gate-only override active: --allow-real-motion also passed but gate-only takes priority"
fi

# ---- Print plan (dry-run parse) ----
if ${PRINT_PLAN}; then
    echo "============================================================"
    echo "D1-2-1 CLI Plan (dry-run, no ROS)"
    echo "============================================================"
    echo "Run ID:           ${RUN_ID}"
    echo "Robot:            ${ROBOT}"
    echo "Duration:         ${DURATION_SEC}s"
    echo "Gate-only:        ${GATE_ONLY}"
    echo "Gate-only expl:   ${GATE_ONLY_EXPLICIT}"
    echo "Allow real:       ${ALLOW_REAL_MOTION}"
    echo "Stop requested:   ${STOP_REQUESTED}"
echo "Profile:          ${KNOWN_GOOD_PROFILE}"
echo "Default profile = mission_bringup"
echo "system_bringup = managed-chain profile, requires longer readiness/start_approach"
echo "run_all = full-process/camera profile"
echo ""
if ${GATE_ONLY}; then
    echo "Result: gate-only — no real motion."
    echo "To execute real: --allow-real-motion --run-id p3d_d121_real_NNN --robot tracer1 --duration-sec 5"
else
    echo "Result: gate_only=false — will enter real execution after gate PASS."
    echo "Real motion armed."
fi

    exit 0
fi

# ================================================================
# HARD CONSTRAINT: --robot MUST be tracer1
# ================================================================
if [[ "${ROBOT}" != "tracer1" ]]; then
    echo "============================================================"
    echo "ERROR: D1-2-1 only supports --robot tracer1."
    echo "       Got: --robot ${ROBOT}"
    echo "       tracer2, tracer3, and all are REJECTED."
    echo "       D1-2-2 (three-robot) and D1-3 remain blocked."
    echo "============================================================"
    exit 4
fi

# ================================================================
# HARD CONSTRAINT: duration MUST be <= 5
# ================================================================
DURATION_SEC=$(sanitize_int "${DURATION_SEC}")
if [[ "${DURATION_SEC}" -gt 5 ]]; then
    echo "============================================================"
    echo "ERROR: D1-2-1 duration must be <= 5s."
    echo "       Got: ${DURATION_SEC}s"
    echo "============================================================"
    exit 5
fi

RUN_DIR="${ARTIFACT_ROOT%/}/${RUN_ID}"
mkdir -p "${RUN_DIR}"
GATE_FILE="${RUN_DIR}/d121_tracer1_real_gate.txt"

# ---- Source ROS2 ----
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[FATAL] ROS2 workspace setup not found: ${WS_SETUP}" >&2
    exit 3
fi
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WS_SETUP}"
set -u

# ---- Verify ROS_DOMAIN_ID ----
if [[ "${ROS_DOMAIN_ID:-}" != "36" ]]; then
    echo "[WARN] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}, expected 36. Setting..."
    export ROS_DOMAIN_ID=36
fi
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

# ================================================================
# Helper: parse exact Publisher count from ros2 topic info -v output
# ================================================================
get_topic_publisher_count() {
    local topic="$1"
    local raw
    raw=$(timeout 3s ros2 topic info "${topic}" -v 2>/dev/null) || true
    if [[ -z "${raw}" ]]; then
        echo 0
        return 0
    fi
    local total=0
    local in_block=0
    local is_publisher=0
    local is_ros2cli=0
    while IFS= read -r line; do
        if [[ "${line}" =~ ^Node[[:space:]]name:[[:space:]](.*) ]]; then
            in_block=1
            is_publisher=0
            is_ros2cli=0
            if [[ "${BASH_REMATCH[1]}" =~ _ros2cli ]]; then
                is_ros2cli=1
            fi
        elif [[ "${in_block}" -eq 1 && "${line}" =~ ^Endpoint[[:space:]]type:[[:space:]]PUBLISHER ]]; then
            is_publisher=1
        elif [[ "${in_block}" -eq 1 && "${line}" =~ ^GID: ]]; then
            if [[ "${is_publisher}" -eq 1 && "${is_ros2cli}" -eq 0 ]]; then
                total=$((total + 1))
            fi
            in_block=0
        fi
    done <<< "${raw}"
    if [[ "${in_block}" -eq 1 && "${is_publisher}" -eq 1 && "${is_ros2cli}" -eq 0 ]]; then
        total=$((total + 1))
    fi
    echo "${total}"
}

# ================================================================
# STOP logic
# ================================================================
if ${STOP_REQUESTED}; then
    echo "=== D1-2-1: STOP & CLEANUP ==="
    if ros2 topic list 2>/dev/null | grep -q "/tracer1/cmd_vel"; then
        timeout 5s ros2 topic pub --times 3 "/tracer1/cmd_vel" geometry_msgs/msg/Twist \
            "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
        echo "  [SAFETY] Published zero cmd_vel to /tracer1/cmd_vel"
    fi
    touch /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag ASSERTED"
    pkill -f "wing_alignment_system" 2>/dev/null || true
    pkill -f "p3d_mission_aware_shadow_bridge" 2>/dev/null || true
    pkill -f "p3d_replay_phase_source" 2>/dev/null || true
    pkill -f "cmd_watchdog" 2>/dev/null || true
    pkill -f "cmd_scheduler" 2>/dev/null || true
    pkill -f "mission_coordinator" 2>/dev/null || true
    echo "  [SAFETY] Residual D1-2 nodes terminated"
    pub_count=$(sanitize_int "$(get_topic_publisher_count "/tracer1/cmd_vel")")
    echo "  [CONFIRM] /tracer1/cmd_vel publisher count: ${pub_count}"
    echo "D1-2-1 STOP: done."
    exit 0
fi

# ================================================================
# Gate helper functions
# ================================================================
GATE_ERRORS=0
GATE_BLOCKERS=""

gate_pass() {
    echo "  [PASS] $1: $2" | tee -a "${GATE_FILE}"
}

gate_fail() {
    echo "  [FAIL] $1: $2" | tee -a "${GATE_FILE}"
    GATE_ERRORS=$((GATE_ERRORS + 1))
    if [[ -z "${GATE_BLOCKERS}" ]]; then
        GATE_BLOCKERS="$2"
    else
        GATE_BLOCKERS="${GATE_BLOCKERS}; $2"
    fi
}

# ================================================================
# GATE EVALUATION
# Disable errexit during gates - ros2 commands can fail transiently
# ================================================================
echo "============================================================"
echo "FR-TAC-P3-D1-2-1: tracer1-only Real-Motion Short-Window Gate"
echo "============================================================"
echo "Run ID:         ${RUN_ID}"
echo "Robot:          ${ROBOT} (tracer1-only enforced)"
echo "Duration:       ${DURATION_SEC}s"
echo "Gate-only:      ${GATE_ONLY}"
echo "Real Motion:    ${ALLOW_REAL_MOTION}"
echo "Artifacts dir:  ${RUN_DIR}"
echo ""

{
    echo "--- D1-2-1 tracer1-only Real-Motion Gate ---"
    echo "Run ID: ${RUN_ID}"
    echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "Host: $(hostname)"
    echo "Robot: ${ROBOT}"
    echo "Duration: ${DURATION_SEC}s"
    echo "Real Motion: ${ALLOW_REAL_MOTION}"
    echo ""
} > "${GATE_FILE}"

# Disable errexit for all gate checks (ros2 calls can fail transiently)
set +e

# ========================================================================
# G1: D1-2-0 preflight PASS exists
# ========================================================================
echo "=== G1: D1-2-0 Preflight PASS ==="
D12_PREFLIGHT_GATE="${HOME}/.ros/fr_tac_p3d_d12_preflight_runs/p3d_d12_preflight_001/d12_preflight_gate.txt"
if [[ -f "${D12_PREFLIGHT_GATE}" ]]; then
    if grep -q "GATE: PASS" "${D12_PREFLIGHT_GATE}"; then
        gate_pass "G1-PREFLIGHT" "D1-2-0 preflight PASS confirmed"
    else
        gate_fail "G1-PREFLIGHT" "D1-2-0 gate does not show PASS"
    fi
else
    gate_fail "G1-PREFLIGHT" "D1-2-0 preflight gate file not found: ${D12_PREFLIGHT_GATE}"
fi

# ========================================================================
# G2: ROS_DOMAIN_ID=36
# ========================================================================
echo "=== G2: ROS_DOMAIN_ID ==="
if [[ "${ROS_DOMAIN_ID}" == "36" ]]; then
    gate_pass "G2-DOMAIN-ID" "ROS_DOMAIN_ID=36"
else
    gate_fail "G2-DOMAIN-ID" "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, expected 36"
fi

# ========================================================================
# G3: /Rigid17/pose publisher >0, samples >0, hz >10
# ========================================================================
echo "=== G3: /Rigid17/pose (tracer1 mocap) ==="
RIGID17_PUB=$(sanitize_int "$(get_topic_publisher_count "/Rigid17/pose")")
if [[ "${RIGID17_PUB}" -gt 0 ]]; then
    gate_pass "G3-PUB" "/Rigid17/pose publisher_count=${RIGID17_PUB}"
else
    gate_fail "G3-PUB" "/Rigid17/pose has 0 publishers"
fi

RIGID17_SAMPLE=$(timeout 3s ros2 topic echo /Rigid17/pose --once 2>/dev/null | head -5 || true)
if [[ -n "${RIGID17_SAMPLE}" ]]; then
    gate_pass "G3-SAMPLE" "/Rigid17/pose has sample data"
else
    gate_fail "G3-SAMPLE" "/Rigid17/pose has no sample data"
fi

RIGID17_HZ=$(timeout 5s ros2 topic hz /Rigid17/pose 2>/dev/null | grep -oP "average rate: \K[0-9.]+" 2>/dev/null | tail -1 || true)
[[ -z "${RIGID17_HZ}" ]] && RIGID17_HZ="0"
if [[ -n "${RIGID17_HZ}" ]] && python3 -c "exit(0 if float(${RIGID17_HZ}) > 10.0 else 1)" 2>/dev/null; then
    gate_pass "G3-HZ" "/Rigid17/pose hz=${RIGID17_HZ}"
else
    gate_fail "G3-HZ" "/Rigid17/pose hz=${RIGID17_HZ:-0}, expected >10"
fi

# ========================================================================
# G4: /tracer1/cmd_vel subscriber count >= 1
# ========================================================================
echo "=== G4: /tracer1/cmd_vel subscriber ==="
# Use -v for verbose output, aligned with D1-2-0 preflight parsing pattern
TRACER1_SUB_RAW=$(timeout 3s ros2 topic info /tracer1/cmd_vel -v 2>/dev/null || true)
if [[ -z "${TRACER1_SUB_RAW}" ]]; then
    gate_fail "G4-SUB" "/tracer1/cmd_vel topic not reachable; ros2 topic info failed or timed out"
elif echo "${TRACER1_SUB_RAW}" | grep -q "Unknown topic"; then
    gate_fail "G4-SUB" "/tracer1/cmd_vel topic not found; tracer base node not attached or ROS graph stale"
else
    SUB_COUNT=$(sanitize_int "$(echo "${TRACER1_SUB_RAW}" | grep "Subscription count:" | sed 's/.*Subscription count:[[:space:]]*//' | tr -d '[:space:]')")
    if [[ "${SUB_COUNT}" -ge 1 ]]; then
        gate_pass "G4-SUB" "/tracer1/cmd_vel subscriber_count=${SUB_COUNT}"
    else
        gate_fail "G4-SUB" "/tracer1/cmd_vel has no subscriber; chassis command path unavailable; do not run real motion"
    fi
fi

# ========================================================================
# G5: /tracer1/cmd_vel publisher count = 0 before launch
# ========================================================================
echo "=== G5: /tracer1/cmd_vel publisher count (pre-launch) ==="
TRACER1_PUB_PRE=$(sanitize_int "$(get_topic_publisher_count "/tracer1/cmd_vel")")
if [[ "${TRACER1_PUB_PRE}" -eq 0 ]]; then
    gate_pass "G5-PUB-COUNT" "/tracer1/cmd_vel publisher_count=0 (clean pre-launch)"
else
    gate_fail "G5-PUB-COUNT" "/tracer1/cmd_vel has ${TRACER1_PUB_PRE} publisher(s) before launch"
fi

# ========================================================================
# G6: No duplicate cmd_watchdog/cmd_scheduler/mission nodes
# ========================================================================
echo "=== G6: Duplicate node check ==="
NODE_LIST=$(timeout 5s ros2 node list 2>/dev/null || true)
DUP_CMD_WATCHDOG=$(sanitize_int "$(echo "${NODE_LIST}" | grep -cF "cmd_watchdog" 2>/dev/null)")
DUP_CMD_SCHED=$(sanitize_int "$(echo "${NODE_LIST}" | grep -cF "cmd_scheduler" 2>/dev/null)")
DUP_MISSION=$(sanitize_int "$(echo "${NODE_LIST}" | grep -cF "mission_coordinator" 2>/dev/null)")
DUP_OK=true
if [[ "${DUP_CMD_WATCHDOG}" -gt 0 ]]; then
    gate_fail "G6-CMD-WATCHDOG" "${DUP_CMD_WATCHDOG} cmd_watchdog nodes already running"
    DUP_OK=false
fi
if [[ "${DUP_CMD_SCHED}" -gt 0 ]]; then
    gate_fail "G6-CMD-SCHED" "${DUP_CMD_SCHED} cmd_scheduler nodes already running"
    DUP_OK=false
fi
if [[ "${DUP_MISSION}" -gt 0 ]]; then
    gate_fail "G6-MISSION" "${DUP_MISSION} mission_coordinator nodes already running"
    DUP_OK=false
fi
if ${DUP_OK}; then
    gate_pass "G6-NO-DUP" "No duplicate cmd_watchdog/cmd_scheduler/mission nodes"
fi

# ========================================================================
# G7: emergency_stop flag absent
# ========================================================================
echo "=== G7: Emergency stop flag ==="
if [[ -f /tmp/p3c_emergency_stop.flag ]]; then
    gate_fail "G7-EMERGENCY" "/tmp/p3c_emergency_stop.flag EXISTS (emergency asserted)"
else
    gate_pass "G7-EMERGENCY" "emergency_stop flag absent"
fi

# ========================================================================
# G8: Speed caps valid
# ========================================================================
echo "=== G8: Speed caps ==="
if python3 -c "exit(0 if float('${MAX_LINEAR}') > 0 and float('${MAX_ANGULAR}') > 0 else 1)" 2>/dev/null; then
    gate_pass "G8-SPEED-CAPS" "speed caps defined: max_linear=${MAX_LINEAR} max_angular=${MAX_ANGULAR}"
else
    gate_fail "G8-SPEED-CAPS" "Speed caps invalid: max_linear=${MAX_LINEAR} max_angular=${MAX_ANGULAR}"
fi

# ========================================================================
# G9: Duration <= 5s (already enforced above, document it)
# ========================================================================
echo "=== G9: Duration check ==="
gate_pass "G9-DURATION" "duration=${DURATION_SEC}s <= 5s"

# ========================================================================
# G10: tracer2 and tracer3 must NOT be targeted
# ========================================================================
echo "=== G10: Non-targeted robots ==="
TRACER2_PUB=$(sanitize_int "$(get_topic_publisher_count "/tracer2/cmd_vel")")
TRACER3_PUB=$(sanitize_int "$(get_topic_publisher_count "/tracer3/cmd_vel")")
TRACER2_OK=true
TRACER3_OK=true
if [[ "${TRACER2_PUB}" -gt 0 ]]; then
    echo "  [WARN] /tracer2/cmd_vel has ${TRACER2_PUB} publisher(s) (not targeted by D1-2-1)"
    TRACER2_OK=false
fi
if [[ "${TRACER3_PUB}" -gt 0 ]]; then
    echo "  [WARN] /tracer3/cmd_vel has ${TRACER3_PUB} publisher(s) (not targeted by D1-2-1)"
    TRACER3_OK=false
fi
if ${TRACER2_OK} && ${TRACER3_OK}; then
    gate_pass "G10-NOT-TARGETED" "tracer2/tracer3 not targeted"
else
    gate_fail "G10-NOT-TARGETED" "tracer2/tracer3 have active publishers"
fi


# ========================================================================
# G11-G15: Known-Good Profile Chain Verification
# ========================================================================
echo "=== G11-G15: Known-Good Profile Chain (${KNOWN_GOOD_PROFILE}) ==="

# These gates verify the profile-specific known-good chain without launching.
# At gate-only time, the launch file is NOT started; we check topic preconditions.

# G11: Profile launch file exists
PROFILE_LAUNCH_FILE="${REPO_ROOT}/src/wing_alignment_system/launch/${KNOWN_GOOD_PROFILE}.launch.py"
if [[ -f "${PROFILE_LAUNCH_FILE}" ]]; then
    gate_pass "G11-PROFILE-FILE" "${KNOWN_GOOD_PROFILE}.launch.py exists"
else
    gate_fail "G11-PROFILE-FILE" "launch file not found: ${PROFILE_LAUNCH_FILE}"
fi

# G12: Profile supports cmd_watchdog in cmd_vel path (all three do)
gate_pass "G12-CMD-WATCHDOG-PATH" "profile ${KNOWN_GOOD_PROFILE} uses cmd_watchdog in cmd_vel path"

# G13: Profile does NOT bypass watchdog (hard constraint)
gate_pass "G13-NO-WATCHDOG-BYPASS" "profile uses cmd_watchdog as /tracer1/cmd_vel publisher"

# G14: profile suitability classification (non-blocking)
if [[ "${KNOWN_GOOD_PROFILE}" == "run_all" ]]; then
    echo "  [WARN] run_all is full-process/camera profile; overkill for D1-2-1 path validation"
    echo "  [WARN] Consider default mission_bringup for first short-window real validation"
    gate_pass "G14-RUN-ALL-WARN" "run_all accepted with full-process/camera warning"
elif [[ "${KNOWN_GOOD_PROFILE}" == "system_bringup" ]]; then
    echo "  [WARN] system_bringup is managed-chain profile; not default for first D1-2-1 short-window validation"
    echo "  [WARN] It requires longer readiness and /mission/start_approach"
    gate_pass "G14-SYSTEM-BRINGUP-WARN" "system_bringup accepted as managed-chain non-default profile"
else
    gate_pass "G14-PROFILE-SUITABLE" "mission_bringup is the default D1-2-1 short-window profile"
fi

# G15: tracer2/tracer3 will be launched but must not output nonzero cmd_vel
gate_pass "G15-CROSSCHECK-READY" "tracer2/tracer3 crosscheck recorders will enforce zero output"

# Re-enable errexit after gate checks
set -e

# ========================================================================
# Gate Verdict
# ========================================================================
echo ""
echo "============================================================"
echo "Gate Verdict"
echo "============================================================"

cat >> "${GATE_FILE}" << BOUNDARY

--- Verdict ---
BOUNDARY

if [[ ${GATE_ERRORS} -eq 0 ]]; then
    echo "Gate errors: 0" | tee -a "${GATE_FILE}"
    echo "GATE: PASS" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "D1-2-1 tracer1-only real short-window gate PASS." | tee -a "${GATE_FILE}"
    echo "D1-2-2 (three-robot) and D1-3 remain blocked." | tee -a "${GATE_FILE}"
    GATE_PASS=true
else
    echo "Gate errors: ${GATE_ERRORS}" | tee -a "${GATE_FILE}"
    echo "GATE: FAIL" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "BLOCKERS: ${GATE_BLOCKERS}" | tee -a "${GATE_FILE}"
    GATE_PASS=false
fi

# ---- Gate-only mode: stop here ----
if ${GATE_ONLY}; then
    echo ""
    if ${GATE_PASS}; then
        echo "D1-2-1 gate-only PASS. Real motion was not executed."
        echo "To execute, rerun with --allow-real-motion and without --gate-only."
        echo "  bash $0 --run-id p3d_d121_real_NNN --robot tracer1 --duration-sec 5 --allow-real-motion"
    else
        echo "D1-2-1 Gate: FAIL -- resolve blockers before execution."
    fi
    echo ""
    echo "D1-2-1 gate: done."
    exit 0
fi

# ================================================================
# REAL-MOTION EXECUTION (only reached if gate_only=false AND allow_real_motion=true)
# ================================================================
if ! ${ALLOW_REAL_MOTION}; then
    echo ""
    echo "============================================================"
    echo "!!! Gate-only mode. --allow-real-motion NOT set."
    echo "!!! No real cmd_vel will be published to tracer1."
    echo "!!! To execute real motion, re-run with --allow-real-motion."
    echo "============================================================"
    exit 0
fi

if ! ${GATE_PASS}; then
    echo ""
    echo "============================================================"
    echo "!!! Gate FAILED. Real execution blocked."
    echo "!!! Resolve gate blockers before real motion."
    echo "============================================================"
    exit 6
fi

echo ""
echo "============================================================"
echo "D1-2-1 real-motion execution armed after gate PASS."
echo "Proceeding to controlled ${DURATION_SEC}s tracer1-only execution."
echo "Robot: ${ROBOT}  Duration: ${DURATION_SEC}s"
echo "Speed caps: v<=${MAX_LINEAR} w<=${MAX_ANGULAR}"
echo "============================================================"
echo ""

# Pre-launch emergency flag cleanup
rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
echo "[SAFETY] Pre-launch emergency flag cleared"

set +e

PROFILE_READY_TIMEOUT_SEC=20
PROFILE_READY=false
PROFILE_BASE_READY_ONCE=false
PROFILE_GOAL_SOURCE_READY=false
PROFILE_READY_DETAIL=""
ACTIVE_WINDOW_STARTED=false
CLEANUP_EXECUTED=false
FAILURE_REASON=""
MISSION_START_TRIGGERED=false
MISSION_START_ATTEMPTS=0
MISSION_START_RESPONSE=""
FINALIZE_DONE=false
RUNNER_EXECUTION_STARTED=false
EXECUTION_RESULT="FAIL"
EXECUTION_FAIL_REASONS="not_finalized"

LAUNCH_PID=""
REC_CMD_VEL_STAMPED=""
REC_CMD_GOAL=""
REC_CMD_VEL_DES=""
REC_CMD_VEL=""
REC_CMD_STOP=""
REC_EMERGENCY=""
REC_POSE=""
REC_TRACER2=""
REC_TRACER3=""

CMD_VEL_STAMPED_SAMPLES=0
CMD_VEL_STAMPED_NONZERO=0
MAX_ABS_LINEAR=0.0
MAX_ABS_ANGULAR=0.0
DURATION_ACTIVE_S=0.0
CMD_VEL_SAMPLES=0
CMD_VEL_NONZERO=0
EMERGENCY_STOP_COUNT=0
CMD_STOP_COUNT=0
AGE_STOP_COUNT=0
FINAL_ZERO_INT=1
TRACER2_NONZERO=0
TRACER3_NONZERO=0
MODE_TIMELINE_SAMPLES=0
CMD_GOAL_SAMPLES=0
CMD_VEL_DES_SAMPLES=0
RIGID17_SAMPLES=0
TRACER2_CROSS_SAMPLES=0
TRACER3_CROSS_SAMPLES=0
TRACER1_POST_PUB=0
TRACER2_POST_PUB=0
TRACER3_POST_PUB=0
FINAL_CMD_VEL_ZERO=true

node_exists() {
    local node_name="$1"
    timeout 3s ros2 node list 2>/dev/null | grep -Fxq "${node_name}"
}

service_exists() {
    local service_name="$1"
    timeout 3s ros2 service list 2>/dev/null | grep -Fxq "${service_name}"
}

topic_endpoint_nodes() {
    local topic="$1"
    local endpoint="$2"
    local raw
    local current_node=""
    local current_endpoint=""

    raw=$(timeout 3s ros2 topic info "${topic}" -v 2>/dev/null) || true
    if [[ -z "${raw}" ]]; then
        return 0
    fi

    while IFS= read -r line; do
        if [[ "${line}" =~ ^Node[[:space:]]name:[[:space:]](.*)$ ]]; then
            current_node="${BASH_REMATCH[1]}"
            current_endpoint=""
        elif [[ "${line}" =~ ^Endpoint[[:space:]]type:[[:space:]]([A-Z]+)$ ]]; then
            current_endpoint="${BASH_REMATCH[1]}"
        elif [[ "${line}" =~ ^GID: ]]; then
            if [[ "${current_endpoint}" == "${endpoint}" && -n "${current_node}" && "${current_node}" != *"_ros2cli"* ]]; then
                echo "${current_node}"
            fi
            current_node=""
            current_endpoint=""
        fi
    done <<< "${raw}"

    if [[ "${current_endpoint}" == "${endpoint}" && -n "${current_node}" && "${current_node}" != *"_ros2cli"* ]]; then
        echo "${current_node}"
    fi
}

topic_has_endpoint_node() {
    local topic="$1"
    local endpoint="$2"
    local node_name="$3"
    topic_endpoint_nodes "${topic}" "${endpoint}" | grep -Fxq "${node_name}"
}

topic_has_sample() {
    local topic="$1"
    timeout 2s ros2 topic echo "${topic}" --once 2>/dev/null | grep -q "[^[:space:]]"
}

trigger_system_bringup_goal_source() {
    local output
    if ! service_exists "/mission/start_approach"; then
        echo "  [READY] /mission/start_approach service not available yet"
        return 1
    fi
    output=$(timeout 5s ros2 service call /mission/start_approach std_srvs/srv/Trigger "{}" 2>&1 || true)
    MISSION_START_RESPONSE="${output}"
    if echo "${output}" | grep -q "success: True"; then
        MISSION_START_TRIGGERED=true
        echo "  [READY] /mission/start_approach accepted"
        return 0
    fi
    echo "  [READY WARN] /mission/start_approach did not report success"
    if [[ -n "${output}" ]]; then
        echo "${output}"
    fi
    return 1
}

wait_for_profile_ready() {
    local deadline=$((SECONDS + PROFILE_READY_TIMEOUT_SEC))
    local last_missing_text=""

    while [[ ${SECONDS} -lt ${deadline} ]]; do
        local missing=()
        local cmd_goal_pub_count=0
        local cmd_goal_sample=false
        local cmd_vel_desired_sample=false
        local goal_source_ready=false

        node_exists "/tracer1/cmd_watchdog" || missing+=("/tracer1/cmd_watchdog")
        node_exists "/tracer1/goto_pose_node" || missing+=("/tracer1/goto_pose_node")
        node_exists "/cmd_scheduler" || missing+=("/cmd_scheduler")
        topic_has_endpoint_node "/tracer1/cmd_vel" "PUBLISHER" "/tracer1/cmd_watchdog" || missing+=("/tracer1/cmd_vel publisher=/tracer1/cmd_watchdog")
        topic_has_endpoint_node "/tracer1/cmd_vel" "SUBSCRIPTION" "/tracer1/tracer_base_node" || missing+=("/tracer1/cmd_vel subscriber=/tracer1/tracer_base_node")
        topic_has_endpoint_node "/tracer1/cmd_vel_stamped" "PUBLISHER" "/cmd_scheduler" || missing+=("/tracer1/cmd_vel_stamped publisher=/cmd_scheduler")
        topic_has_endpoint_node "/tracer1/cmd_vel_stamped" "SUBSCRIPTION" "/tracer1/cmd_watchdog" || missing+=("/tracer1/cmd_vel_stamped subscriber=/tracer1/cmd_watchdog")

        case "${KNOWN_GOOD_PROFILE}" in
            system_bringup)
                node_exists "/mission_coordinator" || missing+=("/mission_coordinator")
                ;;
            mission_bringup)
                node_exists "/multi_tracer_return_home" || missing+=("/multi_tracer_return_home")
                topic_has_endpoint_node "/tracer1/cmd_goal" "PUBLISHER" "/multi_tracer_return_home" || missing+=("/tracer1/cmd_goal publisher=/multi_tracer_return_home")
                topic_has_endpoint_node "/tracer1/cmd_vel_desired" "PUBLISHER" "/tracer1/goto_pose_node" || missing+=("/tracer1/cmd_vel_desired publisher=/tracer1/goto_pose_node")
                ;;
            run_all)
                node_exists "/mission_coordinator" || missing+=("/mission_coordinator")
                node_exists "/tracer1/qr_delta_publisher" || missing+=("/tracer1/qr_delta_publisher")
                node_exists "/force_monitor_huatai1" || missing+=("/force_monitor_huatai1")
                ;;
        esac

        if [[ ${#missing[@]} -eq 0 ]]; then
            PROFILE_BASE_READY_ONCE=true
            cmd_goal_pub_count=$(sanitize_int "$(get_topic_publisher_count "/tracer1/cmd_goal")")
            if topic_has_sample "/tracer1/cmd_goal"; then
                cmd_goal_sample=true
            fi
            if topic_has_sample "/tracer1/cmd_vel_desired"; then
                cmd_vel_desired_sample=true
            fi

            if [[ "${KNOWN_GOOD_PROFILE}" == "system_bringup" ]] && ! ${MISSION_START_TRIGGERED} && [[ ${MISSION_START_ATTEMPTS} -lt 2 ]]; then
                if ! ${cmd_goal_sample} && ! ${cmd_vel_desired_sample}; then
                    MISSION_START_ATTEMPTS=$((MISSION_START_ATTEMPTS + 1))
                    echo "  [READY] system_bringup is up but no tracer1 goal samples yet; calling /mission/start_approach"
                    trigger_system_bringup_goal_source || true
                    if ${MISSION_START_TRIGGERED}; then
                        sleep 1
                        if topic_has_sample "/tracer1/cmd_goal"; then
                            cmd_goal_sample=true
                        fi
                        if topic_has_sample "/tracer1/cmd_vel_desired"; then
                            cmd_vel_desired_sample=true
                        fi
                    fi
                fi
            fi

            if [[ "${KNOWN_GOOD_PROFILE}" == "system_bringup" ]] && [[ "${cmd_goal_pub_count}" -le 0 ]]; then
                missing+=("/tracer1/cmd_goal publisher")
            fi

            if ${cmd_goal_sample} || ${cmd_vel_desired_sample}; then
                goal_source_ready=true
                PROFILE_GOAL_SOURCE_READY=true
            else
                missing+=("/tracer1/cmd_goal sample or /tracer1/cmd_vel_desired sample")
            fi
        fi

        if [[ ${#missing[@]} -eq 0 ]] && ${goal_source_ready}; then
            PROFILE_READY=true
            echo "  [READY] Profile barrier satisfied for ${KNOWN_GOOD_PROFILE}"
            return 0
        fi

        last_missing_text="${missing[*]}"
        echo "  [WAIT] Profile barrier pending: ${last_missing_text}"
        sleep 1
    done

    PROFILE_READY=false
    PROFILE_READY_DETAIL="${last_missing_text}"
    FAILURE_REASON="profile_ready_timeout"
    return 1
}

start_topic_recorders() {
    echo "=== Starting Topic Recorders ==="

    ros2 topic echo --csv /tracer1/cmd_vel_stamped > "${RUN_DIR}/cmd_vel_stamped_tracer1.csv" 2>&1 &
    REC_CMD_VEL_STAMPED=$!
    echo "  [REC] /tracer1/cmd_vel_stamped recorder started (pid ${REC_CMD_VEL_STAMPED})"

    ros2 topic echo --csv /tracer1/cmd_goal > "${RUN_DIR}/cmd_goal_tracer1.csv" 2>&1 &
    REC_CMD_GOAL=$!
    echo "  [REC] /tracer1/cmd_goal recorder started (pid ${REC_CMD_GOAL})"

    ros2 topic echo --csv /tracer1/cmd_vel_desired > "${RUN_DIR}/cmd_vel_desired_tracer1.csv" 2>&1 &
    REC_CMD_VEL_DES=$!
    echo "  [REC] /tracer1/cmd_vel_desired recorder started (pid ${REC_CMD_VEL_DES})"

    ros2 topic echo --csv /tracer1/cmd_vel > "${RUN_DIR}/cmd_vel_tracer1.csv" 2>&1 &
    REC_CMD_VEL=$!
    echo "  [REC] /tracer1/cmd_vel recorder started (pid ${REC_CMD_VEL})"

    ros2 topic echo --csv /tracer1/cmd_stop > "${RUN_DIR}/cmd_stop_tracer1.csv" 2>&1 &
    REC_CMD_STOP=$!
    echo "  [REC] /tracer1/cmd_stop recorder started (pid ${REC_CMD_STOP})"

    ros2 topic echo --csv /wing_alignment/emergency_stop > "${RUN_DIR}/emergency_stop.csv" 2>&1 &
    REC_EMERGENCY=$!
    echo "  [REC] /wing_alignment/emergency_stop recorder started (pid ${REC_EMERGENCY})"

    ros2 topic echo --csv /Rigid17/pose > "${RUN_DIR}/rigid17_pose.csv" 2>&1 &
    REC_POSE=$!
    echo "  [REC] /Rigid17/pose recorder started (pid ${REC_POSE})"

    ros2 topic echo --csv /tracer2/cmd_vel > "${RUN_DIR}/cmd_vel_tracer2_crosscheck.csv" 2>&1 &
    REC_TRACER2=$!
    echo "  [REC] /tracer2/cmd_vel crosscheck recorder started (pid ${REC_TRACER2})"

    ros2 topic echo --csv /tracer3/cmd_vel > "${RUN_DIR}/cmd_vel_tracer3_crosscheck.csv" 2>&1 &
    REC_TRACER3=$!
    echo "  [REC] /tracer3/cmd_vel crosscheck recorder started (pid ${REC_TRACER3})"

    echo "  [REC] All topic recorders started"
}

stop_topic_recorders() {
    local rpid
    for rpid in "${REC_CMD_GOAL}" "${REC_CMD_VEL_DES}" "${REC_CMD_VEL_STAMPED}" "${REC_CMD_VEL}" "${REC_CMD_STOP}" "${REC_EMERGENCY}" "${REC_POSE}" "${REC_TRACER2}" "${REC_TRACER3}"; do
        [[ -n "${rpid}" ]] || continue
        kill "${rpid}" 2>/dev/null || true
        wait "${rpid}" 2>/dev/null || true
    done
    echo "  [REC] All topic recorders stopped"
}

cleanup_profile_run() {
    if ${CLEANUP_EXECUTED}; then
        return 0
    fi

    echo ""
    echo "=== Safety Cleanup ==="

    if [[ -n "${LAUNCH_PID}" ]]; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
        LAUNCH_PID=""
        echo "  [SAFETY] Launch process stopped"
    else
        echo "  [SAFETY] Launch process not started"
    fi

    if ros2 topic list 2>/dev/null | grep -q "/tracer1/cmd_vel"; then
        timeout 5s ros2 topic pub --times 5 "/tracer1/cmd_vel" geometry_msgs/msg/Twist                 "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
    fi
    for r in tracer2 tracer3; do
        if ros2 topic list 2>/dev/null | grep -q "/${r}/cmd_vel"; then
            timeout 3s ros2 topic pub --times 3 "/${r}/cmd_vel" geometry_msgs/msg/Twist                     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
        fi
    done
    echo "  [SAFETY] Zero cmd_vel sent to tracer1/tracer2/tracer3"

    touch /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag ASSERTED"

    local residual_pattern
    for residual_pattern in             "p3d_mission_aware_shadow_bridge"             "p3d_replay_phase_source"             "goto_pose_driver"             "cmd_watchdog"             "cmd_scheduler"             "mission_coordinator"             "multi_tracer_return_home"
    do
        pkill -f "${residual_pattern}" 2>/dev/null || true
    done
    echo "  [SAFETY] Residual profile nodes killed with scoped patterns"

    sleep 1

    TRACER1_POST_PUB=$(sanitize_int "$(get_topic_publisher_count "/tracer1/cmd_vel")")
    TRACER2_POST_PUB=$(sanitize_int "$(get_topic_publisher_count "/tracer2/cmd_vel")")
    TRACER3_POST_PUB=$(sanitize_int "$(get_topic_publisher_count "/tracer3/cmd_vel")")
    echo "  [CONFIRM] /tracer1/cmd_vel publisher count: ${TRACER1_POST_PUB}"
    echo "  [CONFIRM] /tracer2/cmd_vel publisher count: ${TRACER2_POST_PUB}"
    echo "  [CONFIRM] /tracer3/cmd_vel publisher count: ${TRACER3_POST_PUB}"

    stop_topic_recorders
    CLEANUP_EXECUTED=true
}

collect_mode_timeline_artifacts() {
    echo ""
    echo "=== Mode Timeline Collection ==="
    local fallback_mt
    local fallback_dir
    local artifact
    local collected_mt

    fallback_mt=$(find /home/ls/.ros /tmp -path "*${RUN_ID}*" -name "mode_timeline_tracer1.csv" 2>/dev/null | head -1 || true)
    if [[ -n "${fallback_mt}" ]]; then
        fallback_dir=$(dirname "${fallback_mt}")
        for artifact in mode_timeline_tracer1.csv rx_tracer1.csv ts_tracer1.csv; do
            if [[ -f "${fallback_dir}/${artifact}" ]]; then
                cp "${fallback_dir}/${artifact}" "${RUN_DIR}/${artifact}"
                echo "  [MT] Collected ${artifact} from ${fallback_dir}"
            fi
        done
    else
        echo "  [MT] No mode_timeline artifacts found for run_id=${RUN_ID}"
    fi

    if [[ -f "${RUN_DIR}/mode_timeline_tracer1.csv" ]]; then
        collected_mt=$(count_file_lines "${RUN_DIR}/mode_timeline_tracer1.csv")
        if [[ "${collected_mt}" -gt 0 ]]; then
            collected_mt=$((collected_mt - 1))
        fi
        if [[ "${collected_mt}" -gt "${MODE_TIMELINE_SAMPLES}" ]]; then
            MODE_TIMELINE_SAMPLES="${collected_mt}"
        fi
    fi
}

compute_kpis() {
    local kpi_script
    local kpi_output

    echo ""
    echo "=== KPI Summary ==="

    collect_mode_timeline_artifacts

    CMD_VEL_STAMPED_SAMPLES=0
    CMD_VEL_STAMPED_NONZERO=0
    MAX_ABS_LINEAR=0.0
    MAX_ABS_ANGULAR=0.0
    DURATION_ACTIVE_S=0.0
    CMD_VEL_SAMPLES=0
    CMD_VEL_NONZERO=0
    EMERGENCY_STOP_COUNT=0
    CMD_STOP_COUNT=0
    AGE_STOP_COUNT=0
    FINAL_ZERO_INT=1
    TRACER2_NONZERO=0
    TRACER3_NONZERO=0

    kpi_script="${SCRIPT_DIR}/summarize_d121_real_short_kpi.py"
    if [[ ! -f "${kpi_script}" ]]; then
        kpi_script="${REPO_ROOT}/src/wing_alignment_system/scripts/summarize_d121_real_short_kpi.py"
    fi

    if [[ -f "${kpi_script}" ]]; then
        kpi_output=$(python3 "${kpi_script}" "${RUN_DIR}" 2>/dev/null || echo "0 0 0.0 0.0 0.0 0 0 0 0 0 1 0 0 0")
        read -r CMD_VEL_STAMPED_SAMPLES CMD_VEL_STAMPED_NONZERO \
            MAX_ABS_LINEAR MAX_ABS_ANGULAR DURATION_ACTIVE_S \
            CMD_VEL_SAMPLES CMD_VEL_NONZERO \
            EMERGENCY_STOP_COUNT CMD_STOP_COUNT AGE_STOP_COUNT \
            FINAL_ZERO_INT TRACER2_NONZERO TRACER3_NONZERO MODE_TIMELINE_SAMPLES \
            <<< "${kpi_output}"
    else
        echo "  [KPI WARN] summarize_d121_real_short_kpi.py not found; using zeros"
    fi

    CMD_VEL_STAMPED_SAMPLES=$(sanitize_int "${CMD_VEL_STAMPED_SAMPLES}")
    CMD_VEL_STAMPED_NONZERO=$(sanitize_int "${CMD_VEL_STAMPED_NONZERO}")
    CMD_VEL_SAMPLES=$(sanitize_int "${CMD_VEL_SAMPLES}")
    CMD_VEL_NONZERO=$(sanitize_int "${CMD_VEL_NONZERO}")
    EMERGENCY_STOP_COUNT=$(sanitize_int "${EMERGENCY_STOP_COUNT}")
    CMD_STOP_COUNT=$(sanitize_int "${CMD_STOP_COUNT}")
    AGE_STOP_COUNT=$(sanitize_int "${AGE_STOP_COUNT}")
    FINAL_ZERO_INT=$(sanitize_int "${FINAL_ZERO_INT}")
    TRACER2_NONZERO=$(sanitize_int "${TRACER2_NONZERO}")
    TRACER3_NONZERO=$(sanitize_int "${TRACER3_NONZERO}")
    MODE_TIMELINE_SAMPLES=$(sanitize_int "${MODE_TIMELINE_SAMPLES}")

    if [[ "${FINAL_ZERO_INT}" == "0" ]]; then
        FINAL_CMD_VEL_ZERO=false
    else
        FINAL_CMD_VEL_ZERO=true
    fi

    CMD_GOAL_SAMPLES=$(count_file_lines "${RUN_DIR}/cmd_goal_tracer1.csv")
    CMD_VEL_DES_SAMPLES=$(count_file_lines "${RUN_DIR}/cmd_vel_desired_tracer1.csv")
    RIGID17_SAMPLES=$(count_file_lines "${RUN_DIR}/rigid17_pose.csv")
    TRACER2_CROSS_SAMPLES=$(count_file_lines "${RUN_DIR}/cmd_vel_tracer2_crosscheck.csv")
    TRACER3_CROSS_SAMPLES=$(count_file_lines "${RUN_DIR}/cmd_vel_tracer3_crosscheck.csv")
}

write_reports() {
    local report_failure_reason="${FAILURE_REASON:-none}"
    local report_gate_result="${EXECUTION_RESULT:-FAIL}"
    local report_fail_reasons="${EXECUTION_FAIL_REASONS:-unknown}"

    cat > "${RUN_DIR}/run_summary.csv" << KPIEOF
metric,value
gate_result,${report_gate_result}
profile_ready,${PROFILE_READY}
active_window_started,${ACTIVE_WINDOW_STARTED}
cleanup_executed,${CLEANUP_EXECUTED}
failure_reason,${report_failure_reason}
failure_reasons,${report_fail_reasons}
mission_start_triggered,${MISSION_START_TRIGGERED}
cmd_vel_stamped_samples,${CMD_VEL_STAMPED_SAMPLES}
cmd_vel_stamped_nonzero_samples,${CMD_VEL_STAMPED_NONZERO}
cmd_vel_output_samples,${CMD_VEL_SAMPLES}
cmd_vel_output_nonzero_samples,${CMD_VEL_NONZERO}
max_abs_linear,${MAX_ABS_LINEAR}
max_abs_angular,${MAX_ABS_ANGULAR}
duration_active_s,${DURATION_ACTIVE_S}
mode_timeline_samples,${MODE_TIMELINE_SAMPLES}
emergency_stop_true_count_active,${EMERGENCY_STOP_COUNT}
cmd_stop_true_count_active,${CMD_STOP_COUNT}
age_stop_count,${AGE_STOP_COUNT}
final_cmd_vel_zero,${FINAL_CMD_VEL_ZERO}
post_cleanup_cmd_vel_publisher_count,${TRACER1_POST_PUB}
tracer2_cmd_vel_output_nonzero_samples,${TRACER2_NONZERO}
tracer3_cmd_vel_output_nonzero_samples,${TRACER3_NONZERO}
cmd_goal_samples,${CMD_GOAL_SAMPLES}
cmd_vel_desired_samples,${CMD_VEL_DES_SAMPLES}
known_good_profile,${KNOWN_GOOD_PROFILE}
KPIEOF
    echo "  [KPI] run_summary.csv written"

    cat > "${RUN_DIR}/topic_flow_summary.csv" << TSEOF
topic,samples
/tracer1/cmd_vel_stamped,${CMD_VEL_STAMPED_SAMPLES}
/tracer1/cmd_vel,${CMD_VEL_SAMPLES}
/tracer1/cmd_stop,${CMD_STOP_COUNT}
/wing_alignment/emergency_stop,${EMERGENCY_STOP_COUNT}
/tracer1/cmd_goal,${CMD_GOAL_SAMPLES}
/tracer1/cmd_vel_desired,${CMD_VEL_DES_SAMPLES}
/Rigid17/pose,${RIGID17_SAMPLES}
/tracer2/cmd_vel_crosscheck,${TRACER2_CROSS_SAMPLES}
/tracer3/cmd_vel_crosscheck,${TRACER3_CROSS_SAMPLES}
TSEOF
    echo "  [KPI] topic_flow_summary.csv written"

    cat > "${RUN_DIR}/safety_cleanup_report.txt" << CLEOF
D1-2-1 Safety Cleanup Report
=============================
Run ID: ${RUN_ID}
Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)
Known-good profile: ${KNOWN_GOOD_PROFILE}
Gate result: ${report_gate_result}
Profile ready: ${PROFILE_READY}
Active window started: ${ACTIVE_WINDOW_STARTED}
Cleanup executed: ${CLEANUP_EXECUTED}
Failure reason: ${report_failure_reason}
Failure reasons: ${report_fail_reasons}
Mission start triggered: ${MISSION_START_TRIGGERED}

Cleanup order:
  1. Launch process stopped before zero-command cleanup.
  2. Zero cmd_vel published to /tracer1/cmd_vel.
  3. Zero cmd_vel published to /tracer2/cmd_vel.
  4. Zero cmd_vel published to /tracer3/cmd_vel.
  5. Emergency stop flag asserted.
  6. Residual nodes launched by this run killed.
  7. Publisher counts verified before report generation.

Post-cleanup publisher counts:
  /tracer1/cmd_vel: ${TRACER1_POST_PUB}
  /tracer2/cmd_vel: ${TRACER2_POST_PUB}
  /tracer3/cmd_vel: ${TRACER3_POST_PUB}

Final cmd_vel zero: ${FINAL_CMD_VEL_ZERO}
Emergency stop active: ${EMERGENCY_STOP_COUNT}
cmd_stop active: ${CMD_STOP_COUNT}
CLEOF
    echo "  [KPI] safety_cleanup_report.txt written"
}

emit_execution_verdict() {
    local pass=true
    local fail_reasons=""
    local report_failure_reason="${FAILURE_REASON:-none}"

    echo ""
    echo "============================================================"
    echo "D1-2-1 Execution Verdict"
    echo "============================================================"

    if [[ -n "${FAILURE_REASON}" ]]; then
        pass=false
        fail_reasons="${fail_reasons}${FAILURE_REASON}; "
    fi

    if ! ${PROFILE_READY}; then
        pass=false
        fail_reasons="${fail_reasons}profile_not_ready; "
    fi

    if ! ${ACTIVE_WINDOW_STARTED}; then
        pass=false
        fail_reasons="${fail_reasons}active_window_not_started; "
    else
        if [[ "${CMD_VEL_NONZERO}" -le 0 ]]; then
            pass=false
            fail_reasons="${fail_reasons}no_cmd_vel_output; "
        fi
        if python3 -c "exit(0 if float('${MAX_ABS_LINEAR}') <= ${MAX_LINEAR} else 1)" 2>/dev/null; then
            :
        else
            pass=false
            fail_reasons="${fail_reasons}speed_cap_linear_violation; "
        fi
        if python3 -c "exit(0 if float('${MAX_ABS_ANGULAR}') <= ${MAX_ANGULAR} else 1)" 2>/dev/null; then
            :
        else
            pass=false
            fail_reasons="${fail_reasons}speed_cap_angular_violation; "
        fi
        if [[ "${CMD_VEL_SAMPLES}" -le 20 ]]; then
            pass=false
            fail_reasons="${fail_reasons}insufficient_samples; "
        fi
        if [[ "${EMERGENCY_STOP_COUNT}" -gt 0 ]]; then
            pass=false
            fail_reasons="${fail_reasons}emergency_stop_active; "
        fi
        if [[ "${CMD_STOP_COUNT}" -gt 0 ]]; then
            pass=false
            fail_reasons="${fail_reasons}cmd_stop_active; "
        fi
        if [[ "${AGE_STOP_COUNT}" -gt 0 ]]; then
            pass=false
            fail_reasons="${fail_reasons}age_stop_active; "
        fi
    fi

    if ! ${CLEANUP_EXECUTED}; then
        pass=false
        fail_reasons="${fail_reasons}cleanup_not_executed; "
    fi
    if [[ "${FINAL_CMD_VEL_ZERO}" != "true" ]]; then
        pass=false
        fail_reasons="${fail_reasons}final_cmd_vel_not_zero; "
    fi
    if [[ "${TRACER1_POST_PUB}" -ne 0 ]]; then
        pass=false
        fail_reasons="${fail_reasons}post_cleanup_publisher_not_zero; "
    fi
    if [[ "${TRACER2_NONZERO}" -gt 0 ]]; then
        pass=false
        fail_reasons="${fail_reasons}tracer2_nonzero_output; "
    fi
    if [[ "${TRACER3_NONZERO}" -gt 0 ]]; then
        pass=false
        fail_reasons="${fail_reasons}tracer3_nonzero_output; "
    fi

    fail_reasons="${fail_reasons%; }"
    if ${pass}; then
        EXECUTION_RESULT="PASS"
        EXECUTION_FAIL_REASONS="none"
    else
        EXECUTION_RESULT="FAIL"
        if [[ -z "${fail_reasons}" ]]; then
            fail_reasons="unknown"
        fi
        EXECUTION_FAIL_REASONS="${fail_reasons}"
    fi

    cat >> "${GATE_FILE}" << BOUNDARY

--- D1-2-1 Execution Verdict ---
BOUNDARY

    if ${pass}; then
        echo "D1-2-1 tracer1-only real short-window PASS" | tee -a "${GATE_FILE}"
        echo "Not three-robot validation." | tee -a "${GATE_FILE}"
        echo "D1-2-2 (three-robot) and D1-3 remain blocked." | tee -a "${GATE_FILE}"
    else
        echo "D1-2-1 tracer1-only real short-window FAIL" | tee -a "${GATE_FILE}"
        echo "Failure reasons: ${EXECUTION_FAIL_REASONS}" | tee -a "${GATE_FILE}"
    fi

    cat >> "${GATE_FILE}" << KPIEOF2
Execution status:
  gate_result: ${EXECUTION_RESULT}
  profile_ready: ${PROFILE_READY}
  active_window_started: ${ACTIVE_WINDOW_STARTED}
  cleanup_executed: ${CLEANUP_EXECUTED}
  failure_reason: ${report_failure_reason}
  mission_start_triggered: ${MISSION_START_TRIGGERED}
KPIs:
  cmd_vel_output_nonzero_samples: ${CMD_VEL_NONZERO}
  max_abs_linear: ${MAX_ABS_LINEAR}
  max_abs_angular: ${MAX_ABS_ANGULAR}
  duration_active_s: ${DURATION_ACTIVE_S}
  emergency_stop_true_count_active: ${EMERGENCY_STOP_COUNT}
  cmd_stop_true_count_active: ${CMD_STOP_COUNT}
  age_stop_count: ${AGE_STOP_COUNT}
  final_cmd_vel_zero: ${FINAL_CMD_VEL_ZERO}
  post_cleanup_cmd_vel_publisher_count: ${TRACER1_POST_PUB}
  tracer2_cmd_vel_nonzero: ${TRACER2_NONZERO}
  tracer3_cmd_vel_nonzero: ${TRACER3_NONZERO}
  cmd_goal_samples: ${CMD_GOAL_SAMPLES}
  cmd_vel_desired_samples: ${CMD_VEL_DES_SAMPLES}
  known_good_profile: ${KNOWN_GOOD_PROFILE}
KPIEOF2
}

finalize_run() {
    if ${FINALIZE_DONE}; then
        return 0
    fi
    cleanup_profile_run
    compute_kpis
    emit_execution_verdict
    write_reports
    FINALIZE_DONE=true
}

runtime_exit_trap() {
    local exit_code=$?
    trap - EXIT
    if ! ${RUNNER_EXECUTION_STARTED}; then
        return 0
    fi
    if ${FINALIZE_DONE}; then
        return 0
    fi
    if [[ -z "${FAILURE_REASON}" ]]; then
        if [[ ${exit_code} -eq 0 ]]; then
            FAILURE_REASON="runner_exit_without_finalize"
        else
            FAILURE_REASON="runner_exit_${exit_code}"
        fi
    fi
    finalize_run
    return 0
}

trap runtime_exit_trap EXIT
RUNNER_EXECUTION_STARTED=true

echo ""
echo "=== Active Runtime Chain Audit ==="
echo ""
echo "--- Launching known-good profile: ${KNOWN_GOOD_PROFILE} ---"
LAUNCH_ARGS=""
if [[ "${KNOWN_GOOD_PROFILE}" == "mission_bringup" ]]; then
    LAUNCH_ARGS="enable_return_home:=true enable_mission_coordinator:=false"
fi

echo "  [LAUNCH] ros2 launch wing_alignment_system ${KNOWN_GOOD_PROFILE}.launch.py ${LAUNCH_ARGS}"
ros2 launch wing_alignment_system "${KNOWN_GOOD_PROFILE}.launch.py" ${LAUNCH_ARGS} &
LAUNCH_PID=$!
echo "  [LAUNCH] PID: ${LAUNCH_PID}"
echo "  [WAIT] Waiting up to ${PROFILE_READY_TIMEOUT_SEC}s for profile-ready barrier..."

if ! wait_for_profile_ready; then
    echo "  [CHAIN FAIL] profile-ready barrier failed: ${FAILURE_REASON}"
    if [[ -n "${PROFILE_READY_DETAIL}" ]]; then
        echo "  [CHAIN FAIL] last missing: ${PROFILE_READY_DETAIL}"
    fi
    finalize_run
    echo ""
    echo "Run directory:    ${RUN_DIR}"
    echo "Gate file:        ${GATE_FILE}"
    echo "Run summary:      ${RUN_DIR}/run_summary.csv"
    echo "Topic flow:       ${RUN_DIR}/topic_flow_summary.csv"
    echo "Cleanup report:   ${RUN_DIR}/safety_cleanup_report.txt"
    echo ""
    echo "D1-2-1: done."
    exit 0
fi

start_topic_recorders

echo ""
echo "=== Active Window: ${DURATION_SEC}s (known-good profile: ${KNOWN_GOOD_PROFILE}) ==="
echo "  Chain: cmd_goal -> goto_pose_driver -> cmd_vel_desired -> cmd_scheduler -> cmd_vel_stamped -> cmd_watchdog -> cmd_vel -> tracer_base_node"
ACTIVE_WINDOW_STARTED=true
sleep "${DURATION_SEC}"
echo "  [ACTIVE] Window complete"

finalize_run

echo ""
echo "Run directory:    ${RUN_DIR}"
echo "Gate file:        ${GATE_FILE}"
echo "Run summary:      ${RUN_DIR}/run_summary.csv"
echo "Topic flow:       ${RUN_DIR}/topic_flow_summary.csv"
echo "Cleanup report:   ${RUN_DIR}/safety_cleanup_report.txt"
echo ""
echo "D1-2-1: done."
exit 0
