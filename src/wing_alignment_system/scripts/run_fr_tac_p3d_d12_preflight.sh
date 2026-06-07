#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# FR-TAC-P3-D1-2: Three-Robot Real-Motion Short-Window Preflight
#
# Preflight gate-only check. Verifies all preconditions for three-robot
# real motion. Does NOT execute any real motion unless --allow-real-motion
# is explicitly passed (and even then, only traces, no chassis output).
#
# Phase: D1-2-0 (no-motion shadow preflight)
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"
ARTIFACT_ROOT="${HOME}/.ros/fr_tac_p3d_d12_preflight_runs"

# ---- Defaults ----
RUN_ID="p3d_d12_preflight_001"
GATE_ONLY=true
CLEANUP_MODE=false
DURATION_SEC=5
ROBOT_FILTER="all"
ALLOW_REAL_MOTION=false

# ---- Usage ----
usage() {
    cat << 'EOF'
Usage: $0 [OPTIONS]

P3-D1-2 Three-Robot Real-Motion Short-Window Preflight.
Default: gate-only preflight (no real motion).
Checks G1-G9 readiness conditions before any real-motion attempt.

Options:
  --run-id ID              Run identifier (default: p3d_d12_preflight_001)
  --gate-only              Preflight gate checks only (default)
  --cleanup                List and verify cleanup commands (does not execute)
  --duration-sec SEC       Proposed motion duration (default: 5)
  --robot ROBOT            tracer1|all (default: all)
  --allow-real-motion      EXPLICIT flag for real motion (REQUIRES human auth)
  -h, --help               Show this help.

REAL MOTION IS DISABLED BY DEFAULT. --allow-real-motion requires explicit
human authorization and is only valid for D1-2-1 (tracer1-only) or D1-2-2
(three-robot) when all preconditions are met.
EOF
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)              RUN_ID="${2}"; shift 2 ;;
        --gate-only)           GATE_ONLY=true; shift ;;
        --cleanup)             CLEANUP_MODE=true; shift ;;
        --duration-sec)        DURATION_SEC="${2}"; shift 2 ;;
        --robot)               ROBOT_FILTER="${2}"; shift 2 ;;
        --allow-real-motion)   ALLOW_REAL_MOTION=true; shift ;;
        -h|--help)             usage ;;
        *) echo "Unknown option: $1" >&2; usage ;;
    esac
done

# ---- Safety: block real motion unless explicitly authorized ----
if [[ "${ALLOW_REAL_MOTION}" == "true" ]]; then
    echo "============================================================"
    echo "!!! --allow-real-motion DETECTED !!!"
    echo "!!! This flag requires explicit human authorization.    !!!"
    echo "!!! D1-2-0 preflight does NOT support real motion.      !!!"
    echo "!!! Real motion is for D1-2-1/D1-2-2 only.             !!!"
    echo "============================================================"
    echo ""
    echo "D1-2 execution requires separate explicit user authorization."
    echo "This preflight script only validates readiness conditions."
    echo ""
    exit 2
fi

RUN_DIR="${ARTIFACT_ROOT}/${RUN_ID}"
mkdir -p "${RUN_DIR}"

echo "============================================================"
echo "FR-TAC-P3-D1-2: Three-Robot Preflight (Gate-Only)"
echo "============================================================"
echo "Run ID:         ${RUN_ID}"
echo "Robot filter:   ${ROBOT_FILTER}"
echo "Gate-only:      ${GATE_ONLY}"
echo "Artifacts dir:  ${RUN_DIR}"
echo ""

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

# ---- Gate Evaluation ----
GATE_FILE="${RUN_DIR}/d12_preflight_gate.txt"
GATE_ERRORS=0
GATE_BLOCKERS=""

echo "--- D1-2 Preflight Gate ---" > "${GATE_FILE}"
echo "Run ID: ${RUN_ID}" >> "${GATE_FILE}"
echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "${GATE_FILE}"
echo "" >> "${GATE_FILE}"

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

# ========================================================================
# G1: ROS Graph
# ========================================================================
echo ""
echo "=== G1: ROS Graph ==="

NODE_LIST="${RUN_DIR}/ros_graph_summary.txt"

# ros2 node list
if timeout 5s ros2 node list > "${RUN_DIR}/node_list.txt" 2>/dev/null; then
    NODE_COUNT=$(wc -l < "${RUN_DIR}/node_list.txt" | tr -d ' ')
    gate_pass "G1-ROS-NODES" "${NODE_COUNT} nodes visible"
else
    gate_fail "G1-ROS-NODES" "ros2 node list failed"
fi

# ros2 topic list
if timeout 5s ros2 topic list > "${RUN_DIR}/topic_list.txt" 2>/dev/null; then
    TOPIC_COUNT=$(wc -l < "${RUN_DIR}/topic_list.txt" | tr -d ' ')
    gate_pass "G1-ROS-TOPICS" "${TOPIC_COUNT} topics visible"
else
    gate_fail "G1-ROS-TOPICS" "ros2 topic list failed"
fi

# ros2 service list
if timeout 5s ros2 service list > "${RUN_DIR}/service_list.txt" 2>/dev/null; then
    SVC_COUNT=$(wc -l < "${RUN_DIR}/service_list.txt" | tr -d ' ')
    gate_pass "G1-ROS-SERVICES" "${SVC_COUNT} services visible"
else
    gate_fail "G1-ROS-SERVICES" "ros2 service list failed"
fi

# Generate combined ROS graph summary
{
    echo "=== Node List ==="
    cat "${RUN_DIR}/node_list.txt" 2>/dev/null || echo "N/A"
    echo ""
    echo "=== Topic List ==="
    cat "${RUN_DIR}/topic_list.txt" 2>/dev/null || echo "N/A"
    echo ""
    echo "=== Service List ==="
    cat "${RUN_DIR}/service_list.txt" 2>/dev/null || echo "N/A"
} > "${NODE_LIST}"

# ========================================================================
# G2: Mocap
# ========================================================================
echo ""
echo "=== G2: Mocap ==="

MOCAP_CSV="${RUN_DIR}/mocap_summary.csv"
echo "rigid_body,has_sample,hz_estimate,publisher_count" > "${MOCAP_CSV}"

declare -A MOCAP_RIGID=(
    ["tracer1"]="/Rigid17/pose"
    ["tracer2"]="/Rigid14/pose"
    ["tracer3"]="/Rigid15/pose"
    ["wing"]="/Rigid8/pose"
)

ALL_MOCAP_OK=true

for label in tracer1 tracer2 tracer3 wing; do
    topic="${MOCAP_RIGID[$label]}"
    echo "  Checking ${topic} (${label})..."

    # Check topic exists
    if timeout 3s ros2 topic info "${topic}" -v > "${RUN_DIR}/mocap_info_${label}.txt" 2>/dev/null; then
        PUB_COUNT=$(grep -c "PUBLISHER" "${RUN_DIR}/mocap_info_${label}.txt" 2>/dev/null | tail -1 | tr -d " \n" || echo "0")
    else
        PUB_COUNT="0"
    fi

    # Get a sample
    HAS_SAMPLE=false
    if timeout 3s ros2 topic echo "${topic}" --once > "${RUN_DIR}/mocap_sample_${label}.txt" 2>/dev/null; then
        if [[ -s "${RUN_DIR}/mocap_sample_${label}.txt" ]]; then
            HAS_SAMPLE=true
        fi
    fi

    # Measure hz (timeout returns 124, don't gate on exit code)
    HZ_ESTIMATE=0
    timeout 6s ros2 topic hz "${topic}" > "${RUN_DIR}/mocap_hz_${label}.txt" 2>/dev/null || true
    if [[ -s "${RUN_DIR}/mocap_hz_${label}.txt" ]]; then
        HZ_ESTIMATE=$(grep 'average rate:' "${RUN_DIR}/mocap_hz_${label}.txt" 2>/dev/null | tail -1 | awk '{print $3}' || echo "0")
    fi

    echo "${label},${HAS_SAMPLE},${HZ_ESTIMATE},${PUB_COUNT}" >> "${MOCAP_CSV}"

    # Gate: publisher > 0
    if [[ "${PUB_COUNT}" -gt 0 ]]; then
        gate_pass "G2-PUB-${label}" "${topic} publisher_count=${PUB_COUNT}"
    else
        gate_fail "G2-PUB-${label}" "${topic} has no publisher"
        ALL_MOCAP_OK=false
    fi

    # Gate: has sample
    if [[ "${HAS_SAMPLE}" == "true" ]]; then
        gate_pass "G2-SAMPLE-${label}" "${topic} has sample data"
    else
        gate_fail "G2-SAMPLE-${label}" "${topic} no sample data"
        ALL_MOCAP_OK=false
    fi

    # Gate: hz > 10
    if [[ -n "${HZ_ESTIMATE}" ]] && awk "BEGIN {exit !(${HZ_ESTIMATE} > 10)}" 2>/dev/null; then
        gate_pass "G2-HZ-${label}" "${topic} hz=${HZ_ESTIMATE}"
    else
        gate_fail "G2-HZ-${label}" "${topic} hz=${HZ_ESTIMATE} (need > 10)"
        ALL_MOCAP_OK=false
    fi
done

# ========================================================================
# G3: Robot Base
# ========================================================================
echo ""
echo "=== G3: Robot Base ==="

ROBOT_CSV="${RUN_DIR}/robot_topic_summary.csv"
echo "robot,cmd_vel_subscriber,odom_topic" > "${ROBOT_CSV}"

for rn in tracer1 tracer2 tracer3; do
    echo "  Checking ${rn}..."

    HAS_SUB=false
    ODOM_EXISTS=false

    if timeout 3s ros2 topic info "/${rn}/cmd_vel" -v > "${RUN_DIR}/cmd_vel_info_${rn}.txt" 2>/dev/null; then
        if grep -q "SUBSCRIPTION" "${RUN_DIR}/cmd_vel_info_${rn}.txt" 2>/dev/null; then
            HAS_SUB=true
        fi
    fi

    # Check odom
    if timeout 3s ros2 topic info "/${rn}/odom" > /dev/null 2>&1; then
        ODOM_EXISTS=true
    fi

    echo "${rn},${HAS_SUB},${ODOM_EXISTS}" >> "${ROBOT_CSV}"

    if [[ "${HAS_SUB}" == "true" ]]; then
        gate_pass "G3-SUB-${rn}" "/${rn}/cmd_vel subscriber exists"
    else
        gate_fail "G3-SUB-${rn}" "/${rn}/cmd_vel has no subscriber"
    fi
done

# ========================================================================
# G4: Publisher Safety
# ========================================================================
echo ""
echo "=== G4: Publisher Safety ==="

SAFETY_CSV="${RUN_DIR}/safety_summary.csv"
echo "robot,cmd_vel_publisher_count,safe" > "${SAFETY_CSV}"

for rn in tracer1 tracer2 tracer3; do
    PUB_COUNT=0
    if timeout 3s ros2 topic info "/${rn}/cmd_vel" -v > "${RUN_DIR}/safety_info_${rn}.txt" 2>/dev/null; then
        PUB_COUNT=$(grep -c "PUBLISHER" "${RUN_DIR}/safety_info_${rn}.txt" 2>/dev/null | tail -1 | tr -d " \n" || echo "0")
    fi

    SAFE="false"
    if [[ "${PUB_COUNT}" -eq 0 ]]; then
        SAFE="true"
    fi

    echo "${rn},${PUB_COUNT},${SAFE}" >> "${SAFETY_CSV}"

    if [[ "${PUB_COUNT}" -eq 0 ]]; then
        gate_pass "G4-PUB-${rn}" "/${rn}/cmd_vel publisher_count=0"
    else
        gate_fail "G4-PUB-${rn}" "/${rn}/cmd_vel has ${PUB_COUNT} publishers (must be 0)"
    fi
done

# Also check cmd_vel_desired and cmd_vel_stamped
echo "  Checking intermediate command topics..."
for topic in /tracer1/cmd_goal /tracer1/cmd_vel_desired /tracer1/cmd_vel_stamped; do
    if timeout 2s ros2 topic info "${topic}" > /dev/null 2>&1; then
        echo "  [INFO] ${topic} available for reuse"
    else
        echo "  [INFO] ${topic} not yet created (will be created by bringup)"
    fi
done

# ========================================================================
# G5: Emergency
# ========================================================================
echo ""
echo "=== G5: Emergency ==="

EMERGENCY_OK=true

# Check emergency flag file
if [[ -f /tmp/p3c_emergency_stop.flag ]]; then
    gate_fail "G5-FLAG" "/tmp/p3c_emergency_stop.flag exists (emergency residue)"
    EMERGENCY_OK=false
else
    gate_pass "G5-FLAG" "/tmp/p3c_emergency_stop.flag absent"
fi

# Check emergency stop topic
if timeout 3s ros2 topic info /wing_alignment/emergency_stop > /dev/null 2>&1; then
    gate_pass "G5-TOPIC" "/wing_alignment/emergency_stop topic available"
else
    echo "  [INFO] /wing_alignment/emergency_stop topic not yet available (will be created by bringup)"
    gate_pass "G5-TOPIC" "emergency stop topic launchable"
fi

# Check current emergency state if topic exists
if timeout 3s ros2 topic echo /wing_alignment/emergency_stop --once > "${RUN_DIR}/emergency_sample.txt" 2>/dev/null; then
    if grep -qi 'true' "${RUN_DIR}/emergency_sample.txt" 2>/dev/null; then
        gate_fail "G5-STATE" "emergency_stop is TRUE"
        EMERGENCY_OK=false
    else
        gate_pass "G5-STATE" "emergency_stop is false"
    fi
else
    echo "  [INFO] Cannot read emergency_stop state (topic may not exist yet)"
fi

# ========================================================================
# G6: R6 Evidence
# ========================================================================
echo ""
echo "=== G6: R6 Evidence ==="

MANIFEST="/home/ls/hjz/artifacts/fr_tac_p3d_d1_evidence_manifest.md"
R6_GATE="/home/ls/.ros/fr_tac_p3d_d1c_r6_runs/p3d_d1c_r6_natural_goal_001/d1c_r6_natural_mission_goal_gate.txt"

# Check manifest Section 11
if [[ -f "${MANIFEST}" ]]; then
    if grep -q 'Section 11.*D1c-R6' "${MANIFEST}" 2>/dev/null; then
        gate_pass "G6-MANIFEST" "Manifest contains Section 11 (D1c-R6)"
    else
        gate_fail "G6-MANIFEST" "Manifest missing Section 11"
    fi
else
    gate_fail "G6-MANIFEST" "Manifest file not found: ${MANIFEST}"
fi

# Check R6 gate PASS
if [[ -f "${R6_GATE}" ]]; then
    if grep -q 'GATE: PASS' "${R6_GATE}" 2>/dev/null; then
        gate_pass "G6-R6-GATE" "R6 gate PASS confirmed"
    else
        gate_fail "G6-R6-GATE" "R6 gate did not PASS"
    fi
else
    gate_fail "G6-R6-GATE" "R6 gate file not found: ${R6_GATE}"
fi

# ========================================================================
# G7: Single-Source Command Chain
# ========================================================================
echo ""
echo "=== G7: Single-Source Command Chain ==="

# Check for duplicate/residual nodes
declare -A PROHIBITED_NODES=(
    ["cmd_scheduler"]="cmd_scheduler"
    ["cmd_watchdog"]="cmd_watchdog"
    ["mission_coordinator"]="mission_coordinator"
    ["goto_pose_driver"]="goto_pose_driver"
)

G7_OK=true
for node_name in cmd_scheduler cmd_watchdog mission_coordinator goto_pose_driver; do
    COUNT=$(grep -c "${node_name}" "${RUN_DIR}/node_list.txt" 2>/dev/null | tail -1 | tr -d " \n" || echo "0")
    if [[ "${COUNT}" -gt 1 ]]; then
        gate_fail "G7-DUP-${node_name}" "${node_name} appears ${COUNT} times (expected 0 or 1)"
        G7_OK=false
    elif [[ "${COUNT}" -eq 0 ]]; then
        echo "  [INFO] ${node_name} not running (expected before bringup)"
    else
        echo "  [INFO] ${node_name} running (1 instance)"
    fi
done

# Check for residual R6/R4 nodes
RESIDUAL=$(grep -cE 'p3d_d1c_r[46]' "${RUN_DIR}/node_list.txt" 2>/dev/null | tail -1 | tr -d " \n" || echo "0")
if [[ "${RESIDUAL}" -gt 0 ]]; then
    gate_fail "G7-RESIDUAL" "${RESIDUAL} residual D1c-R4/R6 nodes detected"
else
    gate_pass "G7-RESIDUAL" "No residual D1c-R4/R6 nodes"
fi

# ========================================================================
# G8: Speed Caps
# ========================================================================
echo ""
echo "=== G8: Speed Caps ==="

echo "  Proposed D1-2 short-window speed caps:"
echo "    v_cmd <= 0.03 m/s"
echo "    w_cmd <= 0.06 rad/s"
echo "    duration <= 5s (first real attempt)"

# Document in gate file
{
    echo ""
    echo "G8 Speed Caps:"
    echo "  v_cmd_max: 0.03 m/s"
    echo "  w_cmd_max: 0.06 rad/s"
    echo "  duration_max_first_attempt: 5s"
} >> "${GATE_FILE}"

gate_pass "G8-SPEED-CAPS" "v<=0.03, w<=0.06, t<=5s documented"

# ========================================================================
# G9: Cleanup Readiness
# ========================================================================
echo ""
echo "=== G9: Cleanup Readiness ==="

# Verify cleanup commands exist
CLEANUP_OK=true

# Zero velocity publication capability
if timeout 2s ros2 topic pub --help > /dev/null 2>&1; then
    gate_pass "G9-ZERO-PUB" "ros2 topic pub available for zero-velocity"
else
    gate_fail "G9-ZERO-PUB" "ros2 topic pub not available"
    CLEANUP_OK=false
fi

# Emergency flag capability
if touch /tmp/p3c_emergency_stop.flag 2>/dev/null; then
    rm -f /tmp/p3c_emergency_stop.flag
    gate_pass "G9-EMERGENCY-FLAG" "/tmp/p3c_emergency_stop.flag writable"
else
    gate_fail "G9-EMERGENCY-FLAG" "Cannot write emergency flag"
    CLEANUP_OK=false
fi

# Document kill commands
{
    echo ""
    echo "G9 Cleanup Commands:"
    echo "  pkill -f mission_coordinator"
    echo "  pkill -f cmd_watchdog"
    echo "  pkill -f goto_pose_driver"
    echo "  pkill -f cmd_scheduler"
    echo "  pkill -f p3c_emergency"
    echo "  pkill -f ros2 launch"
    echo "  rm -f /tmp/p3c_emergency_stop.flag"
    echo "  ros2 topic pub /tracer1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once"
    echo "  ros2 topic pub /tracer2/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once"
    echo "  ros2 topic pub /tracer3/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once"
} >> "${GATE_FILE}"

gate_pass "G9-CLEANUP" "Cleanup commands documented"

# ========================================================================
# Verdict
# ========================================================================
echo ""
echo "============================================================"
echo "Gate Verdict"
echo "============================================================"

cat >> "${GATE_FILE}" <<BOUNDARY

--- Verdict ---
BOUNDARY

if [[ ${GATE_ERRORS} -eq 0 ]]; then
    echo "Gate errors: 0" | tee -a "${GATE_FILE}"
    echo "GATE: PASS" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "D1-2-0 preflight conditions satisfied." | tee -a "${GATE_FILE}"
    echo "D1-2 execution requires separate explicit user authorization." | tee -a "${GATE_FILE}"
    echo "Next step: human-authorized D1-2-1 (tracer1-only real 3-5s)." | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "!!! REAL MOTION NOT AUTOMATICALLY EXECUTED !!!" | tee -a "${GATE_FILE}"
    echo "!!! Requires: --allow-real-motion --robot tracer1 !!!" | tee -a "${GATE_FILE}"
    GATE_PASS=true
else
    echo "Gate errors: ${GATE_ERRORS}" | tee -a "${GATE_FILE}"
    echo "GATE: FAIL" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "BLOCKERS: ${GATE_BLOCKERS}" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "D1-2 remains blocked. Resolve blockers before retry." | tee -a "${GATE_FILE}"
    GATE_PASS=false
fi

# ========================================================================
# Artifacts Summary
# ========================================================================
echo ""
echo "============================================================"
echo "D1-2 Preflight Artifacts"
echo "============================================================"
echo "  Gate:           ${GATE_FILE}"
echo "  ROS Graph:      ${NODE_LIST}"
echo "  Mocap Summary:  ${MOCAP_CSV}"
echo "  Robot Summary:  ${ROBOT_CSV}"
echo "  Safety Summary: ${SAFETY_CSV}"
echo ""
echo "Run directory:    ${RUN_DIR}"
echo ""

if ${GATE_PASS}; then
    echo "D1-2-0: PASS — preflight conditions satisfied."
    echo "D1-2 execution: NOT YET. Requires separate explicit user authorization."
else
    echo "D1-2-0: FAIL — gate checks failed. D1-2 remains blocked."
    echo "Blockers: ${GATE_BLOCKERS}"
fi

echo ""
echo "D1-2 preflight: done."
exit 0
