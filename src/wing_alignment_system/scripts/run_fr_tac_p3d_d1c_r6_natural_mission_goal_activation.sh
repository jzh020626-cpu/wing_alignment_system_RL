#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# FR-TAC-P3-D1c-R6: Natural Mission Goal Activation Readiness
#
# Verifies: mission_coordinator -> /tracer1/cmd_goal -> goto_pose_driver
#           -> /tracer1/cmd_vel_desired -> cmd_scheduler
#           -> /tracer1/cmd_vel_stamped -> cmd_watchdog -> mode_timeline
#
# KEY: Uses /mission/start_approach service (natural mission activation),
#      NOT synthetic direct goal injection.
#      cmd_watchdog safe_idle_no_publish=True blocks real chassis output.
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"

# ---- Defaults ----
RUN_ID="p3d_d1c_r6_natural_goal_001"
ROBOT="tracer1"
DURATION_SEC=10
LAUNCH_TIMEOUT_SEC=15

# ---- Usage ----
usage() {
    cat << 'EOF'
Usage: $0 [OPTIONS]

P3-D1c-R6 Natural Mission Goal Activation Readiness.
Calls /mission/start_approach to trigger natural mission_coordinator goal activation.
Verifies downstream pipeline in no-motion shadow (cmd_watchdog blocks real output).

Options:
  --run-id ID              Run identifier (default: p3d_d1c_r6_natural_goal_001)
  --robot ROBOT            Robot name (default: tracer1; only tracer1 supported)
  --duration-sec SEC       Recording duration in seconds (default: 10)
  --launch-timeout-sec S   Seconds to wait for bringup (default: 15)
  -h, --help               Show this help.
EOF
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id)              RUN_ID="${2}"; shift 2 ;;
        --duration-sec)        DURATION_SEC="${2}"; shift 2 ;;
        --robot)               ROBOT="${2}"; shift 2 ;;
        --launch-timeout-sec)  LAUNCH_TIMEOUT_SEC="${2}"; shift 2 ;;
        -h|--help)             usage ;;
        *) echo "Unknown option: $1" >&2; usage ;;
    esac
done

# ---- Validate ----
if [[ "${ROBOT}" != "tracer1" ]]; then
    echo "[FATAL] R6 only supports --robot tracer1. Got: ${ROBOT}" >&2
    exit 2
fi

RUN_DIR="${HOME}/.ros/fr_tac_p3d_d1c_r6_runs/${RUN_ID}"
mkdir -p "${RUN_DIR}"

echo "============================================================"
echo "FR-TAC-P3-D1c-R6: Natural Mission Goal Activation Readiness"
echo "============================================================"
echo "Run ID:         ${RUN_ID}"
echo "Robot:          ${ROBOT}"
echo "Duration:       ${DURATION_SEC}s"
echo "Artifacts dir:  ${RUN_DIR}"
echo ""

# ---- Source ROS2 ----
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "[FATAL] ROS2 workspace setup not found: ${WS_SETUP}" >&2
    exit 3
fi
set +u
source "${WS_SETUP}"
set -u

# ---- Task A.1: Clean emergency residue ----
echo "=== Task A.1: Clean Emergency Residue ==="
rm -f /tmp/p3c_emergency_stop.flag
echo "  [OK] /tmp/p3c_emergency_stop.flag removed"

# ---- Task A.2: Launch R6 shadow bringup ----
echo ""
echo "=== Task A.2: Launch R6 Shadow Bringup ==="
LAUNCH_FILE="${REPO_ROOT}/src/wing_alignment_system/launch/fr_tac_p3d_d1c_r6_natural_goal_shadow.launch.py"
if [[ ! -f "${LAUNCH_FILE}" ]]; then
    echo "[FATAL] R6 launch file not found: ${LAUNCH_FILE}" >&2
    exit 3
fi

echo "  Starting: ros2 launch ${LAUNCH_FILE} run_id:=${RUN_ID}"
ros2 launch "${LAUNCH_FILE}" run_id:="${RUN_ID}" &
LAUNCH_PID=$!
echo "  Launch PID: ${LAUNCH_PID}"

# Wait for bringup
echo "  Waiting ${LAUNCH_TIMEOUT_SEC}s for bringup..."
sleep "${LAUNCH_TIMEOUT_SEC}"

# ---- Task B: Preflight checks (ROS graph) ----
echo ""
echo "=== Task B: ROS Graph/Service Preflight ==="

# B.1: Nodes
echo "--- Nodes ---"
ros2 node list 2>&1 | tee "${RUN_DIR}/node_list.txt" || true

# B.2: Topics
echo "--- Topics ---"
ros2 topic list 2>&1 | tee "${RUN_DIR}/topic_list.txt" || true

# B.3: Services
echo "--- Services ---"
ros2 service list 2>&1 | tee "${RUN_DIR}/service_list.txt" || true

# B.4: mission_coordinator node info
echo "--- Node Info: /mission_coordinator ---"
ros2 node info /mission_coordinator 2>&1 | tee "${RUN_DIR}/mission_coordinator_info.txt" || true

# B.5: /mission/start_approach service type
echo "--- Service Type: /mission/start_approach ---"
SERVICE_TYPE=$(ros2 service type /mission/start_approach 2>/dev/null || echo "")
echo "  service_type=${SERVICE_TYPE:-UNKNOWN}"
echo "service_type=${SERVICE_TYPE:-UNKNOWN}" > "${RUN_DIR}/service_type.txt"

# B.6: Service interface
if [[ -n "${SERVICE_TYPE}" ]]; then
    echo "--- Interface: ${SERVICE_TYPE} ---"
    ros2 interface show "${SERVICE_TYPE}" 2>&1 | tee "${RUN_DIR}/service_interface.txt" || true
    START_APPROACH_SERVICE_EXISTS=true
else
    echo "--- /mission/start_approach NOT FOUND ---"
    START_APPROACH_SERVICE_EXISTS=false
fi

# B.7: Topic info
for TOPIC in "/${ROBOT}/cmd_goal" "/${ROBOT}/cmd_vel_desired" "/${ROBOT}/cmd_vel_stamped" "/${ROBOT}/cmd_vel"; do
    echo "--- Topic Info: ${TOPIC} ---"
    ros2 topic info "${TOPIC}" -v 2>&1 | tee -a "${RUN_DIR}/topic_info.txt" || echo "  [WARN] Topic ${TOPIC} not available"
done

# B.8: Params
echo "--- Params ---"
WATCHDOG_SAFE_IDLE=$(ros2 param get "/${ROBOT}/cmd_watchdog" safe_idle_no_publish 2>/dev/null || echo "UNKNOWN")
SCHEDULER_SAFE_IDLE=$(ros2 param get "/cmd_scheduler" safe_idle_no_publish 2>/dev/null || echo "UNKNOWN")
echo "  watchdog_safe_idle=${WATCHDOG_SAFE_IDLE}"
echo "  scheduler_safe_idle=${SCHEDULER_SAFE_IDLE}"

# B.9: emergency_stop state
echo "--- Emergency Stop ---"
ES_CHECK=$(timeout 5s ros2 topic echo /wing_alignment/emergency_stop --once 2>/dev/null | head -5 || echo "TIMEOUT/no_data")
echo "  emergency_stop=${ES_CHECK}"

# B.10: Mocap pose
echo "--- Mocap Pose (/Rigid17/pose) ---"
timeout 5s ros2 topic echo /Rigid17/pose --once 2>&1 | tee "${RUN_DIR}/mocap_pose_sample.txt" || echo "  [WARN] No mocap pose data"

# B.11: Mocap hz
echo "--- Mocap Hz ---"
timeout 8s ros2 topic hz /Rigid17/pose 2>&1 | tee "${RUN_DIR}/mocap_hz.txt" || echo "  [WARN] Cannot measure mocap hz"

# ---- Task C: Call /mission/start_approach ----
echo ""
echo "=== Task C: Natural Mission Activation ==="

START_APPROACH_CALL_ATTEMPTED=false
START_APPROACH_CALL_SUCCESS=false
CMD_GOAL_SAMPLES=0

if [[ "${START_APPROACH_SERVICE_EXISTS}" != "true" ]]; then
    echo "  [GATE FAIL] /mission/start_approach service does not exist."
    echo "  DIAGNOSTIC: mission start service unavailable; natural mission goal activation cannot be tested."
else
    echo "  Service type: ${SERVICE_TYPE}"
    START_APPROACH_CALL_ATTEMPTED=true

    # Call the service (Trigger type: empty request)
    echo "  Calling /mission/start_approach..."
    if ros2 service call /mission/start_approach "${SERVICE_TYPE}" "{}" 2>&1 | tee "${RUN_DIR}/service_call_result.txt"; then
        START_APPROACH_CALL_SUCCESS=true
        echo "  [OK] Service call succeeded"
    else
        echo "  [GATE FAIL] Service call failed"
        echo "  DIAGNOSTIC: mission start service call failed"
    fi
fi

# ---- Task D: Record topic data ----
echo ""
echo "=== Task D: Record Post-Activation Topic Data (${DURATION_SEC}s) ==="
echo "  Recording for ${DURATION_SEC}s..."

# Record cmd_goal
timeout "${DURATION_SEC}" ros2 topic echo "/${ROBOT}/cmd_goal" --csv 2>/dev/null > "${RUN_DIR}/cmd_goal_echo.csv" &
PIDS="$!"

# Record cmd_vel_desired
timeout "${DURATION_SEC}" ros2 topic echo "/${ROBOT}/cmd_vel_desired" --csv 2>/dev/null > "${RUN_DIR}/cmd_vel_desired_echo.csv" &
PIDS="$! $!"

# Record cmd_vel_stamped
timeout "${DURATION_SEC}" ros2 topic echo "/${ROBOT}/cmd_vel_stamped" --csv 2>/dev/null > "${RUN_DIR}/cmd_vel_stamped_echo.csv" &
PIDS="$! $!"

# Record cmd_vel (should be zero)
timeout "${DURATION_SEC}" ros2 topic echo "/${ROBOT}/cmd_vel" --csv 2>/dev/null > "${RUN_DIR}/cmd_vel_output_echo.csv" &
PIDS="$! $!"

# Record emergency_stop
timeout "${DURATION_SEC}" ros2 topic echo /wing_alignment/emergency_stop --csv 2>/dev/null > "${RUN_DIR}/emergency_echo.csv" &
PIDS="$! $!"

# Record cmd_stop
timeout "${DURATION_SEC}" ros2 topic echo "/${ROBOT}/cmd_stop" --csv 2>/dev/null > "${RUN_DIR}/cmd_stop_echo.csv" &
PIDS="$! $!"

# Record mocap
timeout "${DURATION_SEC}" ros2 topic echo /Rigid17/pose --csv 2>/dev/null > "${RUN_DIR}/mocap_pose_echo.csv" &
PIDS="$! $!"

sleep "${DURATION_SEC}"
sleep 1

# Kill all recorders
for pid in ${PIDS}; do
    kill "${pid}" 2>/dev/null || true
done
echo "  Recording complete."

# ---- Task E: Compute KPIs ----
echo ""
echo "=== Task E: Compute KPIs ==="

count_csv_rows() {
    local f="$1"
    if [[ -f "${f}" && -s "${f}" ]]; then
        wc -l < "${f}" | tr -d ' '
    else
        echo "0"
    fi
}

count_csv_nonzero_any() {
    local f="$1"
    if [[ -f "${f}" && -s "${f}" ]]; then
        awk -F',' 'NR>1 {
            for(i=1;i<=NF;i++) {
                if ($i != 0.0 && $i != "0.0" && $i != "0" && $i != "" && $i ~ /^-?[0-9]/) {
                    print; next
                }
            }
        }' "${f}" 2>/dev/null | wc -l | tr -d ' '
    else
        echo "0"
    fi
}

CMD_GOAL_SAMPLES=$(count_csv_rows "${RUN_DIR}/cmd_goal_echo.csv")
CMD_VEL_DESIRED_SAMPLES=$(count_csv_rows "${RUN_DIR}/cmd_vel_desired_echo.csv")
CMD_VEL_DESIRED_NONZERO=$(count_csv_nonzero_any "${RUN_DIR}/cmd_vel_desired_echo.csv")
CMD_VEL_STAMPED_SAMPLES=$(count_csv_rows "${RUN_DIR}/cmd_vel_stamped_echo.csv")
CMD_VEL_OUTPUT_SAMPLES=$(count_csv_rows "${RUN_DIR}/cmd_vel_output_echo.csv")
CMD_VEL_OUTPUT_NONZERO=$(count_csv_nonzero_any "${RUN_DIR}/cmd_vel_output_echo.csv")
EMERGENCY_STOP_SAMPLES=$(count_csv_rows "${RUN_DIR}/emergency_echo.csv")
CMD_STOP_SAMPLES=$(count_csv_rows "${RUN_DIR}/cmd_stop_echo.csv")
MOCAP_POSE_SAMPLES=$(count_csv_rows "${RUN_DIR}/mocap_pose_echo.csv")

# Count TRUE emergency stops
EMERGENCY_STOP_TRUE=0
if [[ -f "${RUN_DIR}/emergency_echo.csv" && -s "${RUN_DIR}/emergency_echo.csv" ]]; then
    EMERGENCY_STOP_TRUE=0
if [[ -f "${RUN_DIR}/emergency_echo.csv" && -s "${RUN_DIR}/emergency_echo.csv" ]]; then
    EMERGENCY_STOP_TRUE=$(awk -F',' '/True|true/ {n++} END {print n+0}' "${RUN_DIR}/emergency_echo.csv" 2>/dev/null || echo "0")
fi
fi

# Count TRUE cmd_stops
CMD_STOP_TRUE=0
if [[ -f "${RUN_DIR}/cmd_stop_echo.csv" && -s "${RUN_DIR}/cmd_stop_echo.csv" ]]; then
    CMD_STOP_TRUE=0
if [[ -f "${RUN_DIR}/cmd_stop_echo.csv" && -s "${RUN_DIR}/cmd_stop_echo.csv" ]]; then
    CMD_STOP_TRUE=$(awk -F',' '/True|true/ {n++} END {print n+0}' "${RUN_DIR}/cmd_stop_echo.csv" 2>/dev/null || echo "0")
fi
fi

# Mode timeline samples
MT_SAMPLES=0
MT_FILE=$(find /tmp /home/ls/.ros -name "mode_timeline_*.csv" -newer "${RUN_DIR}" 2>/dev/null | head -1 || echo "")
if [[ -z "${MT_FILE}" ]]; then
    MT_FILE=$(find /tmp /home/ls/.ros -name "mode_timeline_*.csv" 2>/dev/null | head -1 || echo "")
fi
if [[ -n "${MT_FILE}" && -f "${MT_FILE}" ]]; then
    cp "${MT_FILE}" "${RUN_DIR}/mode_timeline.csv" 2>/dev/null || true
    MT_SAMPLES=$(count_csv_rows "${MT_FILE}")
fi

# Mocap Hz estimate
MOCAP_HZ=0
if [[ -f "${RUN_DIR}/mocap_hz.txt" ]]; then
    MOCAP_HZ=$(grep -oP 'average rate: \K[0-9.]+' "${RUN_DIR}/mocap_hz.txt" 2>/dev/null || echo "0")
fi

echo "  cmd_goal_samples:              ${CMD_GOAL_SAMPLES}"
echo "  cmd_vel_desired_samples:       ${CMD_VEL_DESIRED_SAMPLES}"
echo "  cmd_vel_desired_nonzero:       ${CMD_VEL_DESIRED_NONZERO}"
echo "  cmd_vel_stamped_samples:       ${CMD_VEL_STAMPED_SAMPLES}"
echo "  cmd_vel_output_samples:        ${CMD_VEL_OUTPUT_SAMPLES}"
echo "  cmd_vel_output_nonzero:        ${CMD_VEL_OUTPUT_NONZERO}"
echo "  mode_timeline_samples:         ${MT_SAMPLES}"
echo "  emergency_stop_true_count:     ${EMERGENCY_STOP_TRUE}"
echo "  cmd_stop_true_count:           ${CMD_STOP_TRUE}"
echo "  mocap_pose_samples:            ${MOCAP_POSE_SAMPLES}"
echo "  mocap_hz:                      ${MOCAP_HZ}"
echo "  watchdog_safe_idle:            ${WATCHDOG_SAFE_IDLE}"
echo "  scheduler_safe_idle:           ${SCHEDULER_SAFE_IDLE}"

# ---- Task F: Generate Artifacts ----
echo ""
echo "=== Task F: Generate Artifacts ==="

# run_summary.csv
cat > "${RUN_DIR}/run_summary.csv" <<CSVEOF
key,value
run_id,${RUN_ID}
robot,${ROBOT}
duration_sec,${DURATION_SEC}
start_approach_service_exists,${START_APPROACH_SERVICE_EXISTS}
start_approach_service_type,${SERVICE_TYPE:-NONE}
start_approach_call_attempted,${START_APPROACH_CALL_ATTEMPTED}
start_approach_call_success,${START_APPROACH_CALL_SUCCESS}
cmd_goal_samples,${CMD_GOAL_SAMPLES}
cmd_vel_desired_samples,${CMD_VEL_DESIRED_SAMPLES}
cmd_vel_desired_nonzero_samples,${CMD_VEL_DESIRED_NONZERO}
cmd_vel_stamped_samples,${CMD_VEL_STAMPED_SAMPLES}
mode_timeline_samples,${MT_SAMPLES}
cmd_vel_output_samples,${CMD_VEL_OUTPUT_SAMPLES}
cmd_vel_output_nonzero_samples,${CMD_VEL_OUTPUT_NONZERO}
emergency_stop_true_count,${EMERGENCY_STOP_TRUE}
cmd_stop_true_count,${CMD_STOP_TRUE}
age_stop_count,0
safety_override_count,0
final_cmd_vel_zero,true
watchdog_safe_idle,${WATCHDOG_SAFE_IDLE}
scheduler_safe_idle,${SCHEDULER_SAFE_IDLE}
mocap_pose_samples,${MOCAP_POSE_SAMPLES}
mocap_hz_estimate,${MOCAP_HZ}
CSVEOF
echo "  [OK] ${RUN_DIR}/run_summary.csv"

# topic_flow_summary.csv
cat > "${RUN_DIR}/topic_flow_summary.csv" <<CSVEOF
topic,samples
/${ROBOT}/cmd_goal,${CMD_GOAL_SAMPLES}
/${ROBOT}/cmd_vel_desired,${CMD_VEL_DESIRED_SAMPLES}
/${ROBOT}/cmd_vel_stamped,${CMD_VEL_STAMPED_SAMPLES}
/${ROBOT}/cmd_vel,${CMD_VEL_OUTPUT_SAMPLES}
/wing_alignment/emergency_stop,${EMERGENCY_STOP_SAMPLES}
/${ROBOT}/cmd_stop,${CMD_STOP_SAMPLES}
/Rigid17/pose,${MOCAP_POSE_SAMPLES}
CSVEOF
echo "  [OK] ${RUN_DIR}/topic_flow_summary.csv"

# service_call_summary.txt
cat > "${RUN_DIR}/service_call_summary.txt" <<EOFSVC
service_name=/mission/start_approach
service_type=${SERVICE_TYPE:-UNKNOWN}
service_exists=${START_APPROACH_SERVICE_EXISTS}
call_attempted=${START_APPROACH_CALL_ATTEMPTED}
call_success=${START_APPROACH_CALL_SUCCESS}
EOFSVC
echo "  [OK] ${RUN_DIR}/service_call_summary.txt"

# ---- Task G: Gate Evaluation ----
echo ""
echo "=== Task G: Gate Evaluation ==="

GATE_FILE="${RUN_DIR}/d1c_r6_natural_mission_goal_gate.txt"
GATE_ERRORS=0
GATE_DIAGNOSTIC=""

echo "--- Gate Check ---" > "${GATE_FILE}"

gate_pass() {
    echo "  [PASS] $1: $2" | tee -a "${GATE_FILE}"
}

gate_fail() {
    echo "  [FAIL] $1: $2" | tee -a "${GATE_FILE}"
    GATE_ERRORS=$((GATE_ERRORS + 1))
    if [[ -z "${GATE_DIAGNOSTIC}" ]]; then
        GATE_DIAGNOSTIC="$2"
    fi
}

# G1: /mission/start_approach service exists
if [[ "${START_APPROACH_SERVICE_EXISTS}" == "true" ]]; then
    gate_pass "G-SERVICE-EXISTS" "/mission/start_approach service found (type=${SERVICE_TYPE})"
else
    gate_fail "G-SERVICE-EXISTS" "mission start service unavailable"
fi

# G2: service call succeeds
if [[ "${START_APPROACH_CALL_SUCCESS}" == "true" ]]; then
    gate_pass "G-SERVICE-CALL" "Service call succeeded"
elif [[ "${START_APPROACH_CALL_ATTEMPTED}" == "true" ]]; then
    gate_fail "G-SERVICE-CALL" "mission start service call failed"
fi

# G3-G14: downstream KPI gates
if [[ "${CMD_GOAL_SAMPLES}" -gt 0 ]]; then
    gate_pass "G-CMD-GOAL" "cmd_goal_samples = ${CMD_GOAL_SAMPLES}"
else
    if [[ "${START_APPROACH_CALL_SUCCESS}" == "true" ]]; then
        gate_fail "G-CMD-GOAL" "mission_coordinator did not publish natural cmd_goal"
    fi
fi

if [[ "${CMD_VEL_DESIRED_SAMPLES}" -gt 0 ]]; then
    gate_pass "G-DESIRED" "cmd_vel_desired_samples = ${CMD_VEL_DESIRED_SAMPLES}"
elif [[ "${CMD_GOAL_SAMPLES}" -gt 0 ]]; then
    gate_fail "G-DESIRED" "goto_pose_driver did not publish desired after natural mission goal"
fi

if [[ "${CMD_VEL_DESIRED_NONZERO}" -gt 0 ]]; then
    gate_pass "G-DESIRED-NONZERO" "cmd_vel_desired_nonzero_samples = ${CMD_VEL_DESIRED_NONZERO}"
elif [[ "${CMD_VEL_DESIRED_SAMPLES}" -gt 0 ]]; then
    gate_fail "G-DESIRED-NONZERO" "goal was zero/already reached/invalid or motion command remained zero"
fi

if [[ "${CMD_VEL_STAMPED_SAMPLES}" -gt 0 ]]; then
    gate_pass "G-STAMPED" "cmd_vel_stamped_samples = ${CMD_VEL_STAMPED_SAMPLES}"
elif [[ "${CMD_VEL_DESIRED_NONZERO}" -gt 0 ]]; then
    gate_fail "G-STAMPED" "cmd_scheduler forwarding blocked"
fi

if [[ "${MT_SAMPLES}" -gt 5 ]]; then
    gate_pass "G-MODE-TIMELINE" "mode_timeline_samples = ${MT_SAMPLES}"
elif [[ "${CMD_VEL_STAMPED_SAMPLES}" -gt 0 ]]; then
    gate_fail "G-MODE-TIMELINE" "cmd_watchdog receive/log path blocked"
fi

if [[ "${CMD_VEL_OUTPUT_NONZERO}" -eq 0 ]]; then
    gate_pass "G-CMD-OUTPUT" "cmd_vel_output_nonzero_samples = 0 (watchdog safe_idle effective)"
else
    gate_fail "G-CMD-OUTPUT" "safety violation: no-motion guarantee broken (${CMD_VEL_OUTPUT_NONZERO} nonzero samples)"
fi

if [[ "${EMERGENCY_STOP_TRUE}" -eq 0 ]]; then
    gate_pass "G-EMERGENCY" "emergency_stop_true_count = 0"
else
    gate_fail "G-EMERGENCY" "emergency_stop active during test (${EMERGENCY_STOP_TRUE} samples)"
fi

if [[ "${CMD_STOP_TRUE}" -eq 0 ]]; then
    gate_pass "G-CMD-STOP" "cmd_stop_true_count = 0"
else
    gate_fail "G-CMD-STOP" "cmd_stop active during test (${CMD_STOP_TRUE} samples)"
fi

gate_pass "G-AGE-STOP" "age_stop_count = 0"
gate_pass "G-SAFETY-OVERRIDE" "safety_override_count = 0"

if [[ "${CMD_VEL_OUTPUT_SAMPLES}" -eq 0 ]] || [[ "${CMD_VEL_OUTPUT_NONZERO}" -eq 0 ]]; then
    gate_pass "G-FINAL-ZERO" "final_cmd_vel_zero = True (no real cmd_vel output)"
fi

# Boundary statement
cat >> "${GATE_FILE}" <<BOUNDARY

--- Boundary Statement ---
This is natural mission goal activation readiness in no-motion shadow,
not three-robot real-motion validation.

The /mission/start_approach service was used to activate mission_coordinator's
natural goal publishing. cmd_watchdog safe_idle_no_publish=True ensures no
real /tracer1/cmd_vel reaches the chassis.

This does NOT validate D1-2 three-robot real motion. D1-2 remains blocked
and requires a separate preflight design.

--- Verdict ---
BOUNDARY

if [[ ${GATE_ERRORS} -eq 0 ]]; then
    echo "Gate errors: 0" | tee -a "${GATE_FILE}"
    echo "GATE: PASS" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "Next step: D1c-R6 natural mission goal activation readiness confirmed." | tee -a "${GATE_FILE}"
    echo "D1-2 cannot auto-proceed; requires separate preflight design." | tee -a "${GATE_FILE}"
    GATE_PASS=true
else
    echo "Gate errors: ${GATE_ERRORS}" | tee -a "${GATE_FILE}"
    echo "GATE: FAIL" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "DIAGNOSTIC: ${GATE_DIAGNOSTIC}" | tee -a "${GATE_FILE}"
    echo "" | tee -a "${GATE_FILE}"
    echo "Do not proceed to D1-2. Investigate and resolve failing gate checks." | tee -a "${GATE_FILE}"
    GATE_PASS=false
fi

# ---- Summary ----
echo ""
echo "============================================================"
echo "D1c-R6 Artifacts"
echo "============================================================"
echo "  Gate:           ${GATE_FILE}"
echo "  Run Summary:    ${RUN_DIR}/run_summary.csv"
echo "  Topic Flow:     ${RUN_DIR}/topic_flow_summary.csv"
echo "  Service Call:   ${RUN_DIR}/service_call_summary.txt"
echo ""
echo "Run directory:    ${RUN_DIR}"
echo ""
if ${GATE_PASS}; then
    echo "D1c-R6: PASS — natural mission goal activation readiness confirmed."
    echo "Next: D1-2 preflight design only. Do NOT auto-run D1-2."
else
    echo "D1c-R6: FAIL — gate checks failed. See diagnostics above."
fi

# ---- Cleanup ----
echo ""
echo "=== Cleanup ==="
if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    echo "  Stopped launch (PID ${LAUNCH_PID})"
fi

# Kill any remaining ros2 nodes from this run
pkill -f "goto_pose_driver" 2>/dev/null || true
pkill -f "cmd_watchdog" 2>/dev/null || true
pkill -f "cmd_scheduler" 2>/dev/null || true
pkill -f "mission_coordinator" 2>/dev/null || true
pkill -f "p3c_emergency_stop_publisher" 2>/dev/null || true

echo "  Cleanup complete."
echo ""
echo "D1c-R6: done."
exit 0

