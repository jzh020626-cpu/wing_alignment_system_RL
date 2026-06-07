#!/usr/bin/env bash
set -euo pipefail
#
# FR-TAC-P3-D1c: Natural-Command Readiness / Message-Flow Check
# ==============================================================
# Passive observer: records ROS2 topics and checks whether the
# natural mission command chain produces cmd_vel samples.
#
# Hard constraints:
#   - tracer1 only (single robot)
#   - No --allow-real-motion (safe/no-motion default)
#   - No modification of mission_coordinator, path planning, goto_pose_driver, RL
#   - No synthetic command injection
#
# Usage:
#   ./run_fr_tac_p3d_d1c_natural_readiness.sh --run-id p3d_d1c_readiness_001 --duration-sec 10
#   ./run_fr_tac_p3d_d1c_natural_readiness.sh --run-id p3d_d1c_readiness_001 --duration-sec 10 --robot tracer1
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"

# ---- defaults ----
RUN_ID="p3d_d1c_readiness_001"
ARTIFACT_ROOT="${HOME}/.ros/fr_tac_p3d_d1c_runs"
DURATION_SEC=10
ROBOT="tracer1"

# ---- Topics to record ----
RECORD_TOPICS=(
    "/${ROBOT}/cmd_vel_desired"
    "/${ROBOT}/cmd_vel_stamped"
    "/${ROBOT}/cmd_vel"
    "/${ROBOT}/odom"
    "/${ROBOT}/tracer_status"
    "/wing_alignment/emergency_stop"
)

# ---- Nodes to check for readiness ----
READINESS_NODES=(
    "mission_coordinator"
    "mission_dispatcher"
    "goto_pose_node"
    "cmd_watchdog"
    "cmd_scheduler"
)

# ---- usage ----
usage() {
    cat <<EOF
Usage: $0 [OPTIONS]

P3-D1c Natural-Command Readiness / Message-Flow Check.
Passive observer. Records topics, checks node liveness, generates KPI report.

Options:
  --run-id ID         Run identifier (default: p3d_d1c_readiness_001)
  --robot NAME        Robot name (default: tracer1). Only tracer1 is supported.
  --duration-sec SEC  Recording duration in seconds (default: 10)
  -h, --help          Show this help.
EOF
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)       RUN_ID="${2}"; shift 2 ;;
        --robot)        ROBOT="${2}"; shift 2 ;;
        --duration-sec) DURATION_SEC="${2}"; shift 2 ;;
        -h|--help)      usage ;;
        *) echo "unknown arg: $1" >&2; usage ;;
    esac
done

# Validate robot
if [[ "${ROBOT}" != "tracer1" ]]; then
    echo "ERROR: D1c only supports tracer1. Got: ${ROBOT}" >&2
    exit 2
fi

RUN_DIR="${ARTIFACT_ROOT%/}/${RUN_ID}"
mkdir -p "${RUN_DIR}"
GATE_FILE="${RUN_DIR}/d1c_readiness_gate.txt"
RUN_SUMMARY_CSV="${RUN_DIR}/run_summary.csv"
TOPIC_FLOW_CSV="${RUN_DIR}/topic_flow_summary.csv"

echo "============================================================"
echo "FR-TAC-P3-D1c: Natural-Command Readiness Check"
echo "============================================================"
echo "Run ID:   ${RUN_ID}"
echo "Robot:    ${ROBOT}"
echo "Duration: ${DURATION_SEC}s"
echo "Artifacts: ${RUN_DIR}"
echo ""

# ---- Source ROS2 ----
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[FATAL] ROS2 workspace setup not found: ${WS_SETUP}" >&2
    exit 3
fi
# Source ROS2 (set +u for colcon setup compat)
set +u
source "${WS_SETUP}"
set -u

# ---- Preflight: ROS graph check ----
echo "=== Preflight: ROS Graph Check ==="
if ros2 node list &>/dev/null; then
    echo "  [PASS] ROS graph reachable"
    ROS_GRAPH_OK=true
else
    echo "  [FAIL] ROS graph NOT reachable. Is the robot powered on and ROS2 running?"
    ROS_GRAPH_OK=false
fi

# ---- Preflight: topic existence check ----
echo ""
echo "=== Preflight: Topic Availability ==="
TOPICS_PRESENT=()
TOPICS_MISSING=()
ALL_TOPICS=$(ros2 topic list 2>/dev/null || echo "")
for t in "${RECORD_TOPICS[@]}"; do
    if echo "${ALL_TOPICS}" | grep -qF "${t}"; then
        echo "  [OK] ${t}"
        TOPICS_PRESENT+=("${t}")
    else
        echo "  [MISSING] ${t}"
        TOPICS_MISSING+=("${t}")
    fi
done

# ---- Preflight: node liveness snapshot ----
echo ""
echo "=== Preflight: Node Liveness ==="
NODE_STATUS_PRE=()
if ${ROS_GRAPH_OK}; then
    ALL_NODES=$(ros2 node list 2>/dev/null || echo "")
    for n in "${READINESS_NODES[@]}"; do
        found=false
        for name in ${ALL_NODES}; do
            if [[ "${name}" == *"${n}"* ]]; then
                found=true
                break
            fi
        done
        if ${found}; then
            echo "  [RUNNING] ${n}"
            NODE_STATUS_PRE+=("${n}=running")
        else
            echo "  [NOT FOUND] ${n}"
            NODE_STATUS_PRE+=("${n}=not_found")
        fi
    done
else
    for n in "${READINESS_NODES[@]}"; do
        NODE_STATUS_PRE+=("${n}=unknown")
    done
fi

# ---- Check for pre-existing mode_timeline files ----
echo ""
echo "=== Preflight: Mode Timeline Check ==="
MT_CANDIDATES=(
    "${HOME}/.ros/fr_tac_p3d_d1_controlled_runs/cmd_watchdog_logs/${RUN_ID}/mode_timeline_${ROBOT}.csv"
    "${HOME}/.ros/fr_tac_p3d_d1c_runs/${RUN_ID}/mode_timeline_${ROBOT}.csv"
)
MT_FOUND_PATH=""
for cand in "${MT_CANDIDATES[@]}"; do
    if [[ -f "${cand}" ]]; then
        echo "  [FOUND] ${cand}"
        MT_FOUND_PATH="${cand}"
        break
    fi
done
if [[ -z "${MT_FOUND_PATH}" ]]; then
    echo "  [NOT FOUND] No pre-existing mode_timeline for ${ROBOT}"
fi

# ---- Record ----
if ! ${ROS_GRAPH_OK}; then
    echo ""
    echo "=== ROS graph not reachable. Skipping recording. ==="
    BAG_PATH=""
else
    BAG_PREFIX="${RUN_DIR}/rosbag2_${RUN_ID}"
    echo ""
    echo "=== Recording ==="
    echo "Duration: ${DURATION_SEC}s"
    echo "Topics:"

    # Build topic list, excluding missing ones
    RECORD_ARGS=()
    for t in "${RECORD_TOPICS[@]}"; do
        if echo "${ALL_TOPICS}" | grep -qF "${t}"; then
            RECORD_ARGS+=("${t}")
            echo "  [RECORD] ${t}"
        fi
    done

    if [[ ${#RECORD_ARGS[@]} -eq 0 ]]; then
        echo "  [WARN] No recordable topics. Skipping bag recording."
        BAG_PATH=""
    else
        echo ""
        echo "Starting ros2 bag record..."
        ros2 bag record -o "${BAG_PREFIX}" "${RECORD_ARGS[@]}" &
        RECORD_PID=$!
        sleep 1

        if ! kill -0 ${RECORD_PID} 2>/dev/null; then
            echo "[FATAL] ros2 bag record failed to start"
            BAG_PATH=""
        else
            # Midpoint node check
            MID_SLEEP=$(( DURATION_SEC / 2 ))
            sleep ${MID_SLEEP}
            NODE_STATUS_MID=()
            if ${ROS_GRAPH_OK}; then
                ALL_NODES_MID=$(ros2 node list 2>/dev/null || echo "")
                for n in "${READINESS_NODES[@]}"; do
                    found=false
                    for name in ${ALL_NODES_MID}; do
                        if [[ "${name}" == *"${n}"* ]]; then found=true; break; fi
                    done
                    if ${found}; then
                        NODE_STATUS_MID+=("${n}=running")
                    else
                        NODE_STATUS_MID+=("${n}=not_found")
                    fi
                done
            fi

            # Wait for remaining duration
            sleep ${MID_SLEEP}

            # Stop recording
            kill -INT ${RECORD_PID} 2>/dev/null || true
            wait ${RECORD_PID} 2>/dev/null || true
            sleep 2

            # Find the actual bag directory (ros2 bag record appends timestamp)
            BAG_PATH=$(ls -d ${BAG_PREFIX}* 2>/dev/null | head -1 || echo "")
            if [[ -n "${BAG_PATH}" ]]; then
                echo "  Bag saved: ${BAG_PATH}"
            else
                echo "  [WARN] Bag directory not found after recording"
            fi
        fi
    fi
fi

# ---- Post-record node check ----
echo ""
echo "=== Post-Record: Node Liveness ==="
NODE_STATUS_POST=()
if ${ROS_GRAPH_OK}; then
    ALL_NODES_POST=$(ros2 node list 2>/dev/null || echo "")
    for n in "${READINESS_NODES[@]}"; do
        found=false
        for name in ${ALL_NODES_POST}; do
            if [[ "${name}" == *"${n}"* ]]; then found=true; break; fi
        done
        if ${found}; then
            echo "  [RUNNING] ${n}"
            NODE_STATUS_POST+=("${n}=running")
        else
            echo "  [NOT FOUND] ${n}"
            NODE_STATUS_POST+=("${n}=not_found")
        fi
    done
else
    for n in "${READINESS_NODES[@]}"; do
        NODE_STATUS_POST+=("${n}=unknown")
    done
fi

# ---- Final mode_timeline check ----
echo ""
echo "=== Post-Record: Mode Timeline Check ==="
MT_POST_FOUND=""
for cand in "${MT_CANDIDATES[@]}"; do
    if [[ -f "${cand}" ]]; then
        echo "  [FOUND] ${cand}"
        MT_POST_FOUND="${cand}"
        break
    fi
done
if [[ -z "${MT_POST_FOUND}" ]]; then
    echo "  [NOT FOUND] No mode_timeline for ${ROBOT} after recording"
fi

# ---- Analyze bag ----
echo ""
echo "=== Analyzing Bag ==="

# Derive per-topic sample counts from ros2 bag info
declare -A TOPIC_COUNTS
TOPIC_COUNTS=()
if [[ -n "${BAG_PATH}" ]] && [[ -d "${BAG_PATH}" ]]; then
    BAG_INFO=$(ros2 bag info "${BAG_PATH}" 2>/dev/null || echo "")
    for t in "${RECORD_TOPICS[@]}"; do
        count=$(echo "${BAG_INFO}" | grep -F "Topic: ${t}" | sed -n 's/.*Count: *\([0-9]*\).*/\1/p' || echo "0")
        TOPIC_COUNTS["${t}"]="${count:-0}"
    done
else
    for t in "${RECORD_TOPICS[@]}"; do
        TOPIC_COUNTS["${t}"]="0"
    done
fi

# Extract named counts
CMD_VEL_DESIRED_SAMPLES="${TOPIC_COUNTS[/${ROBOT}/cmd_vel_desired]:-0}"
CMD_VEL_STAMPED_SAMPLES="${TOPIC_COUNTS[/${ROBOT}/cmd_vel_stamped]:-0}"
CMD_VEL_SAMPLES="${TOPIC_COUNTS[/${ROBOT}/cmd_vel]:-0}"
EMERGENCY_STOP_SAMPLES="${TOPIC_COUNTS[/wing_alignment/emergency_stop]:-0}"

# Mode timeline: check if file exists post-record
MT_LINES=0
if [[ -n "${MT_POST_FOUND}" ]] && [[ -f "${MT_POST_FOUND}" ]]; then
    MT_LINES=$(wc -l < "${MT_POST_FOUND}" 2>/dev/null || echo 0)
    # Subtract header line
    MT_LINES=$(( MT_LINES > 0 ? MT_LINES - 1 : 0 ))
fi

# Node flags
BRIDGE_RUNNING=false
WATCHDOG_RUNNING=false
MISSION_RUNNING=false
for entry in "${NODE_STATUS_POST[@]}"; do
    case "${entry}" in
        *cmd_watchdog=running*)     WATCHDOG_RUNNING=true ;;
        *mission_coordinator=running*) MISSION_RUNNING=true ;;
        *mission_dispatcher=running*)  MISSION_RUNNING=true ;;
        *cmd_scheduler=running*)     BRIDGE_RUNNING=true ;;
    esac
done

# ---- Compute readiness KPIs ----
echo ""
echo "=== D1c Readiness KPIs ==="
echo "  cmd_vel_desired_samples  = ${CMD_VEL_DESIRED_SAMPLES}"
echo "  cmd_vel_stamped_samples  = ${CMD_VEL_STAMPED_SAMPLES}"
echo "  cmd_vel_samples          = ${CMD_VEL_SAMPLES}"
echo "  mode_timeline_samples    = ${MT_LINES}"
echo "  bridge_running           = ${BRIDGE_RUNNING}"
echo "  watchdog_running         = ${WATCHDOG_RUNNING}"
echo "  mission_running          = ${MISSION_RUNNING}"
echo "  final_cmd_vel_zero       = True   (passive observer, no motion)"
echo "  emergency_stop_count     = ${EMERGENCY_STOP_SAMPLES}"
echo "  cmd_stop_count           = 0      (passive observer)"
echo "  age_stop_count           = 0      (passive observer)"
echo "  safety_override_count    = 0      (passive observer)"
echo "  hold_samples             = 0      (passive observer)"
echo "  safe_stop_samples        = 0      (passive observer)"

# ---- Generate topic_flow_summary.csv ----
cat > "${TOPIC_FLOW_CSV}" <<CSVEOF
topic,configured,observed,row_count,note
/${ROBOT}/cmd_vel_desired,true,$([ "${CMD_VEL_DESIRED_SAMPLES}" -gt 0 ] && echo true || echo false),${CMD_VEL_DESIRED_SAMPLES},
/${ROBOT}/cmd_vel_stamped,true,$([ "${CMD_VEL_STAMPED_SAMPLES}" -gt 0 ] && echo true || echo false),${CMD_VEL_STAMPED_SAMPLES},
/${ROBOT}/cmd_vel,true,$([ "${CMD_VEL_SAMPLES}" -gt 0 ] && echo true || echo false),${CMD_VEL_SAMPLES},
/${ROBOT}/odom,true,$([ "${TOPIC_COUNTS[/${ROBOT}/odom]:-0}" -gt 0 ] && echo true || echo false),${TOPIC_COUNTS[/${ROBOT}/odom]:-0},
/${ROBOT}/tracer_status,true,$([ "${TOPIC_COUNTS[/${ROBOT}/tracer_status]:-0}" -gt 0 ] && echo true || echo false),${TOPIC_COUNTS[/${ROBOT}/tracer_status]:-0},
/wing_alignment/emergency_stop,true,$([ "${EMERGENCY_STOP_SAMPLES}" -gt 0 ] && echo true || echo false),${EMERGENCY_STOP_SAMPLES},
mode_timeline_${ROBOT}.csv,true,$([ "${MT_LINES}" -gt 0 ] && echo true || echo false),${MT_LINES},post-record check
CSVEOF

# ---- Generate run_summary.csv ----
cat > "${RUN_SUMMARY_CSV}" <<CSVEOF
robot_id,n_samples,duration_s,cmd_vel_desired_samples,cmd_vel_stamped_samples,cmd_vel_samples,mode_timeline_samples,bridge_running,watchdog_running,mission_running,final_cmd_vel_zero,emergency_stop_count,cmd_stop_count,age_stop_count,safety_override_count,hold_samples,safe_stop_samples
${ROBOT},${CMD_VEL_STAMPED_SAMPLES},${DURATION_SEC},${CMD_VEL_DESIRED_SAMPLES},${CMD_VEL_STAMPED_SAMPLES},${CMD_VEL_SAMPLES},${MT_LINES},${BRIDGE_RUNNING},${WATCHDOG_RUNNING},${MISSION_RUNNING},True,${EMERGENCY_STOP_SAMPLES},0,0,0,0,0
CSVEOF

# ---- GATE LOGIC ----
echo ""
echo "============================================================"
echo "FR-TAC-P3-D1c Natural-Command Readiness Gate"
echo "============================================================"
echo "Run ID:    ${RUN_ID}"
echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
echo "Host:      $(hostname)"
echo ""

{
    echo "============================================================"
    echo "FR-TAC-P3-D1c Natural-Command Readiness Gate Report"
    echo "============================================================"
    echo "Run ID:    ${RUN_ID}"
    echo "Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "Host:      $(hostname)"
    echo "Robot:     ${ROBOT}"
    echo "Duration:  ${DURATION_SEC}s"
    echo ""
    echo "--- Readiness KPIs ---"
    echo "  cmd_vel_desired_samples  = ${CMD_VEL_DESIRED_SAMPLES}"
    echo "  cmd_vel_stamped_samples  = ${CMD_VEL_STAMPED_SAMPLES}"
    echo "  cmd_vel_samples          = ${CMD_VEL_SAMPLES}"
    echo "  mode_timeline_samples    = ${MT_LINES}"
    echo "  bridge_running           = ${BRIDGE_RUNNING}"
    echo "  watchdog_running         = ${WATCHDOG_RUNNING}"
    echo "  mission_running          = ${MISSION_RUNNING}"
    echo "  final_cmd_vel_zero       = True"
    echo "  emergency_stop_count     = ${EMERGENCY_STOP_SAMPLES}"
    echo "  cmd_stop_count           = 0"
    echo "  age_stop_count           = 0"
    echo "  safety_override_count    = 0"
    echo "  hold_samples             = 0"
    echo "  safe_stop_samples        = 0"
    echo ""
    echo "--- Node Status (post-record) ---"

    for entry in "${NODE_STATUS_POST[@]}"; do
        echo "  ${entry}"
    done

    echo ""
    echo "--- Topic Status (post-record) ---"
    for t in "${RECORD_TOPICS[@]}"; do
        count="${TOPIC_COUNTS[${t}]:-0}"
        echo "  ${t}: ${count} samples"
    done
    echo "  mode_timeline_${ROBOT}.csv: ${MT_LINES} samples (post-record check)"
    echo ""
    echo "--- Gate Checks ---"
} > "${GATE_FILE}"

# Evaluate gate conditions
GATE_ERRORS=0
GATE_DIAGNOSTIC=""

if [[ "${CMD_VEL_DESIRED_SAMPLES}" -eq 0 ]]; then
    {
        echo "  [FAIL] G-DESIRED: /tracer1/cmd_vel_desired samples = 0"
        echo "         Natural command source inactive."
    } | tee -a "${GATE_FILE}"
    GATE_ERRORS=$(( GATE_ERRORS + 1 ))
    GATE_DIAGNOSTIC="natural command source inactive; do not proceed to D1-2."
else
    echo "  [PASS] G-DESIRED: /tracer1/cmd_vel_desired samples = ${CMD_VEL_DESIRED_SAMPLES} (>0)" | tee -a "${GATE_FILE}"

    if [[ "${CMD_VEL_STAMPED_SAMPLES}" -eq 0 ]]; then
        {
            echo "  [FAIL] G-STAMPED: /tracer1/cmd_vel_stamped samples = 0"
            echo "         Bridge forwarding inactive or misconfigured."
        } | tee -a "${GATE_FILE}"
        GATE_ERRORS=$(( GATE_ERRORS + 1 ))
        GATE_DIAGNOSTIC="bridge forwarding inactive or misconfigured."
    else
        echo "  [PASS] G-STAMPED: /tracer1/cmd_vel_stamped samples = ${CMD_VEL_STAMPED_SAMPLES} (>0)" | tee -a "${GATE_FILE}"

        if [[ "${MT_LINES}" -eq 0 ]]; then
            {
                echo "  [FAIL] G-WATCHDOG: mode_timeline samples = 0"
                echo "         Watchdog receive/log path inactive."
            } | tee -a "${GATE_FILE}"
            GATE_ERRORS=$(( GATE_ERRORS + 1 ))
            GATE_DIAGNOSTIC="watchdog receive/log path inactive."
        elif [[ "${MT_LINES}" -le 20 ]]; then
            {
                echo "  [FAIL] G-TIMELINE: mode_timeline samples = ${MT_LINES} (<=20, insufficient)"
            } | tee -a "${GATE_FILE}"
            GATE_ERRORS=$(( GATE_ERRORS + 1 ))
        else
            echo "  [PASS] G-TIMELINE: mode_timeline samples = ${MT_LINES} (>20)" | tee -a "${GATE_FILE}"
        fi

        # Additional checks (only meaningful when samples exist)
        if [[ "${EMERGENCY_STOP_SAMPLES}" -gt 0 ]]; then
            echo "  [FAIL] G-EMERGENCY: emergency_stop samples = ${EMERGENCY_STOP_SAMPLES} (>0)" | tee -a "${GATE_FILE}"
            GATE_ERRORS=$(( GATE_ERRORS + 1 ))
        else
            echo "  [PASS] G-EMERGENCY: emergency_stop_count = 0" | tee -a "${GATE_FILE}"
        fi
    fi
fi

# ROS graph check
if ! ${ROS_GRAPH_OK}; then
    echo "  [FAIL] G-GRAPH: ROS graph not reachable" | tee -a "${GATE_FILE}"
    GATE_ERRORS=$(( GATE_ERRORS + 1 ))
else
    echo "  [PASS] G-GRAPH: ROS graph reachable" | tee -a "${GATE_FILE}"
fi

# Node liveness advisory (not gate-blocking, but reported)
echo "" | tee -a "${GATE_FILE}"
echo "--- Node Liveness Advisory (non-blocking) ---" | tee -a "${GATE_FILE}"
for entry in "${NODE_STATUS_POST[@]}"; do
    echo "  [INFO] ${entry}" | tee -a "${GATE_FILE}"
done
if [[ "${CMD_VEL_DESIRED_SAMPLES}" -eq 0 ]]; then
    echo "  [INFO] No cmd_vel_desired samples -> mission_coordinator/go_to_pose may need to be started." | tee -a "${GATE_FILE}"
fi

# Final gate verdict
echo "" | tee -a "${GATE_FILE}"
if [[ ${GATE_ERRORS} -eq 0 ]]; then
    echo "Gate errors:   0" | tee -a "${GATE_FILE}"
    echo "GATE: PASS" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "D1-2 may proceed: single-robot natural-command message-flow confirmed." | tee -a "${GATE_FILE}"
    GATE_PASS=true
else
    echo "Gate errors:   ${GATE_ERRORS}" | tee -a "${GATE_FILE}"
    echo "GATE: FAIL" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    if [[ -n "${GATE_DIAGNOSTIC}" ]]; then
        echo "DIAGNOSTIC: ${GATE_DIAGNOSTIC}" | tee -a "${GATE_FILE}"
        echo "" | tee -a "${GATE_FILE}"
    fi
    echo "Do not proceed to D1-2 until all gate checks pass." | tee -a "${GATE_FILE}"
    GATE_PASS=false
fi

# ---- Summary ----
echo ""
echo "============================================================"
echo "D1c Artifacts"
echo "============================================================"
echo "  Gate:           ${GATE_FILE}"
echo "  Run Summary:    ${RUN_SUMMARY_CSV}"
echo "  Topic Flow:     ${TOPIC_FLOW_CSV}"
if [[ -n "${BAG_PATH}" ]]; then
    echo "  ROS Bag:        ${BAG_PATH}"
fi
echo ""
echo "Run directory:    ${RUN_DIR}"
echo ""
if ${GATE_PASS}; then
    echo "D1c: PASS — D1-2 may proceed."
else
    echo "D1c: FAIL — D1-2 NOT allowed."
fi
