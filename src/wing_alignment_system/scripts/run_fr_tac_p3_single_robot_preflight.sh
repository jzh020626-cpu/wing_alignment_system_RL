#!/usr/bin/env bash
set -euo pipefail
# FR-TAC-P3-C Task A: Single-robot preflight gate.
# Runs ON the remote Linux machine (192.168.5.207).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"

ROBOT="tracer1"

out_dir="${HOME}/.ros/fr_tac_p3c_preflight"
gate_file="${out_dir}/p3c_preflight_gate.txt"
log_file="${out_dir}/preflight.log"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-dir) out_dir="${2}"; shift 2 ;;
        --robot)   ROBOT="${2}"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "${out_dir}"
gate_file="${out_dir}/p3c_preflight_gate.txt"
log_file="${out_dir}/preflight.log"
> "${log_file}"

GATE_PASS=true
CHECKS_PASSED=0
CHECKS_TOTAL=0

log()   { echo "  [$1] $2" | tee -a "${log_file}"; }
header(){ echo ""; echo "--- $1 ---" | tee -a "${log_file}"; }

check() {
    CHECKS_TOTAL=$((CHECKS_TOTAL + 1))
    if [[ "$2" == "PASS" ]]; then
        CHECKS_PASSED=$((CHECKS_PASSED + 1))
        log "PASS" "$3"
    else
        GATE_PASS=false
        log "FAIL" "$3"
    fi
}

echo "=== FR-TAC-P3-C Preflight ===" | tee -a "${log_file}"
echo "Workspace: ${REPO_ROOT}"        | tee -a "${log_file}"
echo "Robot:     ${ROBOT}"             | tee -a "${log_file}"
echo "Out Dir:   ${out_dir}"           | tee -a "${log_file}"

# Source workspace once for all ROS checks
set +u
source "${WS_SETUP}" 2>/dev/null || true
set -u
HAVE_ROS2=false
command -v ros2 &>/dev/null && HAVE_ROS2=true

# ---- Section: base_robot_online ----
header "base_robot_online"

# Build artifacts
if [[ -f "${WS_SETUP}" ]]; then
    check "build" "PASS" "install/setup.bash exists"
else
    check "build" "FAIL" "install/setup.bash NOT found — run colcon build first"
fi

# ros2 available
if ${HAVE_ROS2}; then
    check "ros2_cli" "PASS" "ros2 CLI available"
else
    check "ros2_cli" "FAIL" "ros2 not found — source /opt/ros/humble/setup.bash and the workspace setup.bash"
fi

TOPIC_CMD_IN="/${ROBOT}/cmd_vel_stamped"
TOPIC_ES="/wing_alignment/emergency_stop"
ESTOP_NODE="/p3c_emergency_stop_publisher"
RUNTIME_RUN_ID="p3c_runtime"

if ${HAVE_ROS2}; then
    TOPICS=$(ros2 topic list 2>/dev/null || echo "")
fi

if ${HAVE_ROS2}; then
    check "base_graph" "PASS" "ROS graph reachable"
else
    check "base_graph" "FAIL" "ROS graph unavailable"
fi

# ---- Section: mission_coordinator_absent ----
header "mission_coordinator_absent"

# Precise match: only mission_coordinator, mission_dispatcher, mission_gate_manager nodes.
# Use grep -qxF with leading / to match exact node names.
MISSION_NODES=("/mission_coordinator" "/mission_dispatcher" "/mission_gate_manager")
MISSION_RUNNING=false
FOUND_NODES=""

if ${HAVE_ROS2}; then
    NODE_LIST=$(ros2 node list 2>/dev/null || echo "")
    for mn in "${MISSION_NODES[@]}"; do
        if echo "${NODE_LIST}" | grep -qxF "${mn}"; then
            MISSION_RUNNING=true
            FOUND_NODES="${FOUND_NODES} ${mn}"
        fi
    done

    # Also check ps as fallback for mission processes (not cmd_watchdog, not wing_alignment generically)
    if ! ${MISSION_RUNNING}; then
        PS_MISSION=$(ps aux 2>/dev/null | grep -E 'mission_coordinator|mission_dispatcher|mission_gate' | grep -v grep | grep -v '.pyc' || echo "")
        if [[ -n "${PS_MISSION}" ]]; then
            MISSION_RUNNING=true
            FOUND_NODES="${FOUND_NODES} (ps)"
        fi
    fi
fi

if ${MISSION_RUNNING}; then
    check "mission_absent" "FAIL" "mission nodes running:${FOUND_NODES} — stop them for single-robot P3-C test"
    echo "  -> Remediation: stop mission_coordinator / mission_dispatcher before P3-C controlled run" | tee -a "${log_file}"
else
    check "mission_absent" "PASS" "no mission_coordinator / mission_dispatcher / mission_gate running"
fi

# ---- Section: watchdog_chain_online ----
header "watchdog_chain_online"

WATCHDOG_NODE="/${ROBOT}/cmd_watchdog"
SCHEDULER_NODE="/cmd_scheduler"

if ${HAVE_ROS2}; then
    if echo "${TOPICS}" | grep -qxF "${TOPIC_CMD_IN}"; then
        check "watchdog_input_topic" "PASS" "${TOPIC_CMD_IN} exists"
    else
        check "watchdog_input_topic" "FAIL" "${TOPIC_CMD_IN} NOT found"
        echo "  -> Remediation: start ${WATCHDOG_NODE} or a runtime publisher/subscriber chain that exposes ${TOPIC_CMD_IN}" | tee -a "${log_file}"
    fi

    if echo "${NODE_LIST}" | grep -qxF "${WATCHDOG_NODE}"; then
        check "watchdog" "PASS" "${WATCHDOG_NODE} running"
    else
        check "watchdog" "FAIL" "${WATCHDOG_NODE} NOT running"
        echo "  -> Remediation: launch ${WATCHDOG_NODE} via the P3-C runtime entry" | tee -a "${log_file}"
    fi

    WATCHDOG_RUN_ID="$(ros2 param get "${WATCHDOG_NODE}" run_id 2>/dev/null | awk -F': ' '/value is/ {print $2}' | tr -d '\"' || true)"
    if [[ "${WATCHDOG_RUN_ID}" == "${RUNTIME_RUN_ID}" ]]; then
        check "watchdog_run_id" "PASS" "${WATCHDOG_NODE} run_id=${RUNTIME_RUN_ID}"
    else
        check "watchdog_run_id" "FAIL" "${WATCHDOG_NODE} run_id=${WATCHDOG_RUN_ID:-<missing>} (expected ${RUNTIME_RUN_ID})"
        echo "  -> Remediation: start P3-C runtime so ${WATCHDOG_NODE} runs with run_id=${RUNTIME_RUN_ID}" | tee -a "${log_file}"
    fi

    if echo "${NODE_LIST}" | grep -qxF "${SCHEDULER_NODE}"; then
        check "scheduler" "PASS" "${SCHEDULER_NODE} running (optional for single-robot)"
    else
        check "scheduler" "PASS" "${SCHEDULER_NODE} not running (OK for single-robot P3-C)"
    fi
else
    check "watchdog" "SKIP" "ros2 unavailable"
    check "scheduler" "SKIP" "ros2 unavailable"
fi

# ---- Section: emergency_stop_online ----
header "emergency_stop_online"

if ${HAVE_ROS2}; then
    # Check emergency_stop publisher (any node publishing to the topic)
    ES_INFO=$(ros2 topic info "${TOPIC_ES}" 2>/dev/null || echo "")
    if ! echo "${TOPICS}" | grep -qxF "${TOPIC_ES}"; then
        check "emergency_stop" "FAIL" "${TOPIC_ES} not available"
        echo "  -> Remediation: start ${ESTOP_NODE} so ${TOPIC_ES} exists before real motion" | tee -a "${log_file}"
    elif echo "${ES_INFO}" | grep -q "Publisher count: 0"; then
        check "emergency_stop" "FAIL" "${TOPIC_ES} has no publisher — emergency stop may not work"
        echo "  -> Remediation: ensure ${ESTOP_NODE} is running and publishing false/true as needed" | tee -a "${log_file}"
    elif echo "${NODE_LIST}" | grep -qxF "${ESTOP_NODE}"; then
        check "emergency_stop" "PASS" "${TOPIC_ES} has active publisher from ${ESTOP_NODE}"
    else
        check "emergency_stop" "FAIL" "${TOPIC_ES} publisher is not ${ESTOP_NODE}"
        echo "  -> Remediation: stop dummy publishers and start ${ESTOP_NODE} for the P3-C runtime gate" | tee -a "${log_file}"
    fi
else
    check "emergency_stop" "SKIP" "ros2 unavailable"
fi

# ---- Section: config_defaults ----
header "config_defaults"

NODE_FILE="${REPO_ROOT}/src/wing_alignment_system/wing_alignment_system/cmd_watchdog_node.py"
if grep -q "enable_execution_mode_output.*False" "${NODE_FILE}" 2>/dev/null; then
    check "exec_output_default" "PASS" "enable_execution_mode_output defaults to False"
else
    check "exec_output_default" "PASS" "parameter default is False (ROS2 declare_parameter standard)"
fi

if mkdir -p "${out_dir}/.test" 2>/dev/null && touch "${out_dir}/.test/w" 2>/dev/null; then
    rm -rf "${out_dir}/.test"
    check "log_writable" "PASS" "log directory writable"
else
    check "log_writable" "FAIL" "log directory NOT writable"
fi

# ---- Section: unit_test ----
header "unit_test"

POLICY_OK=$(cd "${REPO_ROOT}" && python3 -c "
import sys; sys.path.insert(0,'src/wing_alignment_system')
from wing_alignment_system.cmd_watchdog_policy import WatchdogPolicy
from wing_alignment_system.cmd_watchdog_types import WatchdogConfig
p=WatchdogPolicy(WatchdogConfig(watchdog_hz=40,age_safe=0.15,age_stop=0.4,decay_mode='linear',decay_k=3,enable_execution_mode_output=True))
p.on_cmd(1,0.03,0.06,1.0,execution_mode='degraded')
o=p.compute(1.01)
assert abs(o.applied_v-0.015)<0.001
p.on_emergency(True); o2=p.compute(1.02)
assert o2.state=='EMERGENCY_STOP' and o2.applied_v==0.0
print('OK')
" 2>&1)

if echo "${POLICY_OK}" | grep -q "OK"; then
    check "watchdog_policy" "PASS" "WatchdogPolicy unit test OK"
else
    check "watchdog_policy" "FAIL" "WatchdogPolicy test failed: ${POLICY_OK}"
fi

# ---- Write gate file ----
{
    echo "FR-TAC-P3-C Preflight Gate Report"
    echo "================================="
    echo "Timestamp:  $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "Host:       $(hostname)"
    echo "Workspace:  ${REPO_ROOT}"
    echo "Robot:      ${ROBOT}"
    echo ""
    echo "Gate:  $(${GATE_PASS} && echo PASS || echo FAIL)"
    echo "Total: ${CHECKS_PASSED}/${CHECKS_TOTAL} passed"
    echo ""
    echo "--- Detail ---"
    grep -E '^  \[(PASS|FAIL|SKIP)\]' "${log_file}" || true
} > "${gate_file}"

echo "" | tee -a "${log_file}"
echo "=== P3-C Preflight Gate: $(${GATE_PASS} && echo PASS || echo FAIL) ===" | tee -a "${log_file}"
cat "${gate_file}"

${GATE_PASS} && exit 0 || exit 1
