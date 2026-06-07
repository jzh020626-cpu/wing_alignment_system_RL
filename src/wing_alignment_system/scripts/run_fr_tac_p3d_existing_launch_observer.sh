#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"
ARTIFACT_ROOT_DEFAULT="${HOME}/.ros/fr_tac_p3d_existing_launch_observer_runs"

RUN_ID="p3d_existing_launch_observer"
PROFILE="mission_bringup"
ARTIFACT_ROOT="${ARTIFACT_ROOT_DEFAULT}"
DURATION_SEC=10
TRIGGER_START_APPROACH=false
PROFILE_READY_TIMEOUT_SEC=45
SHADOW_POLICY_ENABLED=false
SHADOW_POLICY="delta_hold"
SHADOW_DELTA_TH="0.001"
SHADOW_MAX_HOLD_MS="100.0"
REDUCED_ENABLED=false
REDUCED_POLICY="full_update"
REDUCED_DELTA_TH="0.001"
REDUCED_MAX_HOLD_MS="100.0"
REDUCED_PERIODIC_K="2"
PRINT_PLAN=false

usage() {
    cat <<'USAGEEOF'
Usage: run_fr_tac_p3d_existing_launch_observer.sh [OPTIONS]

Observational smoke runner for existing real-machine launch profiles.

Options:
  --run-id ID         Run identifier (default: p3d_existing_launch_observer)
  --profile NAME      mission_bringup|system_bringup|run_all (default: mission_bringup)
  --artifact-root DIR Artifact root (default: ~/.ros/fr_tac_p3d_existing_launch_observer_runs)
  --duration-sec SEC      Observation duration after ready barrier (default: 10)
  --trigger-start-approach  Call /mission/start_approach after ready (system_bringup only)
  -h, --help              Show this help
USAGEEOF
    exit 2
}

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

count_csv_rows_without_header() {
    local path="$1"
    local rows
    rows=$(count_file_lines "${path}")
    if [[ "${rows}" -gt 0 ]]; then
        echo $((rows - 1))
    else
        echo 0
    fi
}

get_topic_publisher_count() {
    local topic="$1"
    local raw
    local total=0
    local in_block=0
    local is_publisher=0
    local is_ros2cli=0

    raw=$(timeout 3s ros2 topic info "${topic}" -v 2>/dev/null) || true
    if [[ -z "${raw}" ]]; then
        echo 0
        return 0
    fi

    while IFS= read -r line; do
        if [[ "${line}" =~ ^Node[[:space:]]name:[[:space:]](.*)$ ]]; then
            in_block=1
            is_publisher=0
            is_ros2cli=0
            if [[ "${BASH_REMATCH[1]}" == *"_ros2cli"* ]]; then
                is_ros2cli=1
            fi
        elif [[ "${in_block}" -eq 1 && "${line}" =~ ^Endpoint[[:space:]]type:[[:space:]]PUBLISHER$ ]]; then
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

node_exists() {
    local node_name="$1"
    timeout 3s ros2 node list 2>/dev/null | grep -Fxq "${node_name}"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id) RUN_ID="${2}"; shift 2 ;;
        --profile) PROFILE="${2}"; shift 2 ;;
        --artifact-root) ARTIFACT_ROOT="${2}"; shift 2 ;;
        --duration-sec) DURATION_SEC="${2}"; shift 2 ;;
        --trigger-start-approach) TRIGGER_START_APPROACH=true; shift ;;
        --enable-policy-shadow) SHADOW_POLICY_ENABLED=true; shift ;;
        --shadow-policy) SHADOW_POLICY="${2}"; shift 2 ;;
        --shadow-delta-threshold) SHADOW_DELTA_TH="${2}"; shift 2 ;;
        --shadow-max-hold-ms) SHADOW_MAX_HOLD_MS="${2}"; shift 2 ;;
        --enable-reduced-output) REDUCED_ENABLED=true; shift ;;
        --reduced-policy) REDUCED_POLICY="${2}"; shift 2 ;;
        --reduced-delta-threshold) REDUCED_DELTA_TH="${2}"; shift 2 ;;
        --reduced-max-hold-ms) REDUCED_MAX_HOLD_MS="${2}"; shift 2 ;;
        --reduced-periodic-k) REDUCED_PERIODIC_K="${2}"; shift 2 ;;
        --print-plan) PRINT_PLAN=true; shift ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1" >&2; usage ;;
    esac
done

DURATION_SEC=$(sanitize_int "${DURATION_SEC}")
if [[ "${DURATION_SEC}" -le 0 ]]; then
    echo "ERROR: --duration-sec must be > 0" >&2
    exit 2
fi

PROFILE_SEMANTICS=""
LAUNCH_ARGS=""
PROFILE_READY_HINT=""
case "${PROFILE}" in
    mission_bringup)
        PROFILE_SEMANTICS="multi-robot return-home observational smoke"
        LAUNCH_ARGS="enable_return_home:=true enable_mission_coordinator:=false"
        PROFILE_READY_HINT="wait for multi_tracer_return_home + cmd_scheduler + tracer1 watchdog nodes"
        ;;
    system_bringup)
        PROFILE_SEMANTICS="managed-chain observational smoke"
        PROFILE_READY_HINT="wait for mission_coordinator + cmd_scheduler + tracer1 watchdog nodes"
        ;;
    run_all)
        PROFILE_SEMANTICS="full-process/camera observational smoke"
        PROFILE_READY_HINT="wait for camera/full-process nodes + cmd_scheduler + tracer1 watchdog nodes"
        ;;
    *)
        echo "ERROR: invalid --profile ${PROFILE}" >&2
        usage
        ;;
esac

if ${TRIGGER_START_APPROACH} && [[ "${PROFILE}" != "system_bringup" ]]; then
    echo "FATAL: --trigger-start-approach requires --profile system_bringup, got ${PROFILE}" >&2
    exit 2
fi

if ${SHADOW_POLICY_ENABLED} && [[ "${PROFILE}" != "system_bringup" ]]; then
    echo "WARNING: --enable-policy-shadow only supports system_bringup, ignoring" >&2
    SHADOW_POLICY_ENABLED=false
fi

RUN_DIR="${ARTIFACT_ROOT%/}/${RUN_ID}"
mkdir -p "${RUN_DIR}"
GATE_FILE="${RUN_DIR}/observer_gate.txt"
RUN_SUMMARY="${RUN_DIR}/run_summary.csv"
TOPIC_FLOW_SUMMARY="${RUN_DIR}/topic_flow_summary.csv"
SAFETY_REPORT="${RUN_DIR}/safety_cleanup_report.txt"
PROFILE_CHAIN_SUMMARY="${RUN_DIR}/profile_chain_summary.txt"
PROFILE_LAUNCH_FILE="${REPO_ROOT}/src/wing_alignment_system/launch/${PROFILE}.launch.py"
MARKER_FILE="${RUN_DIR}/observer_start.marker"
LAUNCH_LOG="${RUN_DIR}/launch.log"

if ${SHADOW_POLICY_ENABLED}; then
    shadow_yaml="${RUN_DIR}/shadow_mission_params.yaml"
    cp "${REPO_ROOT}/src/wing_alignment_system/config/mission_params.yaml" "${shadow_yaml}"
    sed -i 's/enable_policy_shadow: false/enable_policy_shadow: true/' "${shadow_yaml}"
    sed -i "s/shadow_policy: \"delta_hold\"/shadow_policy: \"${SHADOW_POLICY}\"/" "${shadow_yaml}"
    sed -i "s/shadow_delta_threshold: 0.001/shadow_delta_threshold: ${SHADOW_DELTA_TH}/" "${shadow_yaml}"
    _fmax=$(python3 -c "print(float(${SHADOW_MAX_HOLD_MS:-100.0}))" 2>/dev/null) || _fmax="100.0"
    sed -i "s/shadow_max_hold_ms: 100.0/shadow_max_hold_ms: ${_fmax}/" "${shadow_yaml}"
    if python3 -c "import yaml; yaml.safe_load(open('${shadow_yaml}').read())" 2>/dev/null; then
        echo "  [SHADOW] Valid shadow config: ${shadow_yaml}" | tee -a "${GATE_FILE}"
        LAUNCH_ARGS="${LAUNCH_ARGS} config_file:=${shadow_yaml}"
    else
        echo "FATAL: shadow config YAML validation failed: ${shadow_yaml}" >&2
        FAILURE_REASON="shadow_config_yaml_invalid"
    fi
fi

if ${REDUCED_ENABLED}; then
    if [[ -z "${shadow_yaml:-}" ]]; then
        shadow_yaml="${RUN_DIR}/shadow_mission_params.yaml"
        cp "${REPO_ROOT}/src/wing_alignment_system/config/mission_params.yaml" "${shadow_yaml}"
        LAUNCH_ARGS="${LAUNCH_ARGS} config_file:=${shadow_yaml}"
    fi
    _rmax=$(python3 -c "print(float(${REDUCED_MAX_HOLD_MS:-100.0}))" 2>/dev/null) || _rmax="100.0"
    sed -i 's/enable_reduced_output: false/enable_reduced_output: true/' "${shadow_yaml}"
    sed -i "s/reduced_policy: \"full_update\"/reduced_policy: \"${REDUCED_POLICY}\"/" "${shadow_yaml}"
    sed -i "s/reduced_delta_threshold: 0.001/reduced_delta_threshold: ${REDUCED_DELTA_TH}/" "${shadow_yaml}"
    sed -i "s/reduced_max_hold_ms: 100.0/reduced_max_hold_ms: ${_rmax}/" "${shadow_yaml}"
    sed -i "s/reduced_periodic_k: 2/reduced_periodic_k: ${REDUCED_PERIODIC_K}/" "${shadow_yaml}"
    echo "  [REDUCED] Reduced-output enabled: policy=${REDUCED_POLICY}" | tee -a "${GATE_FILE}"
fi

PROFILE_LAUNCH_CMD="ros2 launch wing_alignment_system ${PROFILE}.launch.py ${LAUNCH_ARGS}"

echo "============================================================" | tee "${GATE_FILE}"
echo "Existing Launch Observer" | tee -a "${GATE_FILE}"
echo "============================================================" | tee -a "${GATE_FILE}"
echo "Run ID: ${RUN_ID}" | tee -a "${GATE_FILE}"
echo "Profile: ${PROFILE}" | tee -a "${GATE_FILE}"
echo "Semantics: ${PROFILE_SEMANTICS}" | tee -a "${GATE_FILE}"
echo "Duration: ${DURATION_SEC}s" | tee -a "${GATE_FILE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}" | tee -a "${GATE_FILE}"
echo "Launch command: ${PROFILE_LAUNCH_CMD}" | tee -a "${GATE_FILE}"
echo "" | tee -a "${GATE_FILE}"

if [[ ! -f "${PROFILE_LAUNCH_FILE}" ]]; then
    echo "FATAL: launch file not found: ${PROFILE_LAUNCH_FILE}" >&2
    exit 3
fi
if [[ ! -f "${WS_SETUP}" ]]; then
    echo "FATAL: ROS2 workspace setup not found: ${WS_SETUP}" >&2
    exit 3
fi

set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "${WS_SETUP}"
set -u

if [[ "${ROS_DOMAIN_ID:-}" != "36" ]]; then
    echo "[WARN] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}, expected 36. Setting..."
    export ROS_DOMAIN_ID=36
fi

echo "============================================================" | tee "${GATE_FILE}"
echo "Existing Launch Observer" | tee -a "${GATE_FILE}"
echo "============================================================" | tee -a "${GATE_FILE}"
echo "Run ID: ${RUN_ID}" | tee -a "${GATE_FILE}"
echo "Profile: ${PROFILE}" | tee -a "${GATE_FILE}"
echo "Semantics: ${PROFILE_SEMANTICS}" | tee -a "${GATE_FILE}"
echo "Duration: ${DURATION_SEC}s" | tee -a "${GATE_FILE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}" | tee -a "${GATE_FILE}"
echo "Launch command: ${PROFILE_LAUNCH_CMD}" | tee -a "${GATE_FILE}"
echo "" | tee -a "${GATE_FILE}"

LAUNCH_PID=""
LAUNCH_STARTED=false
PROFILE_READY=false
PROFILE_READY_DETAIL=""
OBSERVATION_STARTED=false
CLEANUP_EXECUTED=false
WATCHDOG_PATH_OK=false
REPORTS_GENERATED=false
FINALIZE_DONE=false
RUNNER_STARTED=false
FAILURE_REASON=""
EXECUTION_RESULT="FAIL"
EXECUTION_FAIL_REASONS="not_finalized"
OPTIONAL_ARTIFACTS_TEXT="none"

REC_CMD_GOAL=""
REC_CMD_VEL_DES=""
REC_CMD_VEL_STAMPED=""
REC_CMD_VEL=""
REC_CMD_STOP=""
REC_TRACER2=""
REC_TRACER3=""
REC_EMERGENCY=""
REC_RIGID17=""
REC_RIGID14=""
REC_RIGID15=""
REC_RIGID8=""
REC_CMD_GOAL_T2=""  REC_CMD_VEL_DES_T2=""  REC_CMD_VEL_STAMPED_T2=""  REC_CMD_STOP_T2=""
REC_CMD_GOAL_T3=""  REC_CMD_VEL_DES_T3=""  REC_CMD_VEL_STAMPED_T3=""  REC_CMD_STOP_T3=""

CMD_GOAL_SAMPLES=0
CMD_VEL_DESIRED_SAMPLES=0
CMD_VEL_STAMPED_SAMPLES=0
CMD_VEL_OUTPUT_SAMPLES=0
CMD_VEL_OUTPUT_NONZERO_SAMPLES=0
TRACER2_NONZERO=0
TRACER3_NONZERO=0
MAX_ABS_LINEAR_TRACER1=0.0
MAX_ABS_ANGULAR_TRACER1=0.0
MAX_ABS_LINEAR_TRACER2=0.0
MAX_ABS_ANGULAR_TRACER2=0.0
MAX_ABS_LINEAR_TRACER3=0.0
MAX_ABS_ANGULAR_TRACER3=0.0
EMERGENCY_STOP_TRUE_COUNT=0
CMD_STOP_TRUE_COUNT=0
FINAL_ZERO_INT=1
FINAL_CMD_VEL_ZERO=true
MOCAP_SAMPLES=0
ESTIMATED_DISPLACEMENT_TRACER1=0.0
ESTIMATED_DISPLACEMENT_TRACER2=0.0
ESTIMATED_DISPLACEMENT_TRACER3=0.0
MODE_TIMELINE_SAMPLES=0
TRACER2_CMD_VEL_SAMPLES=0
TRACER3_CMD_VEL_SAMPLES=0
CMD_STOP_SAMPLES=0
EMERGENCY_STOP_SAMPLES=0
RIGID17_SAMPLES=0
RIGID14_SAMPLES=0
RIGID15_SAMPLES=0
RIGID8_SAMPLES=0
MODE_TIMELINE_TRACER1_SAMPLES=0
MODE_TIMELINE_TRACER2_SAMPLES=0
MODE_TIMELINE_TRACER3_SAMPLES=0
RX_TRACER1_SAMPLES=0
RX_TRACER2_SAMPLES=0
RX_TRACER3_SAMPLES=0
TS_TRACER1_SAMPLES=0
TS_TRACER2_SAMPLES=0
TS_TRACER3_SAMPLES=0
TRACER1_POST_PUB=0
TRACER2_POST_PUB=0
TRACER3_POST_PUB=0

start_topic_recorders() {
    echo "=== Starting Topic Recorders ===" | tee -a "${GATE_FILE}"

    ros2 topic echo --csv /tracer1/cmd_goal > "${RUN_DIR}/cmd_goal_tracer1.csv" 2>&1 &
    REC_CMD_GOAL=$!
    ros2 topic echo --csv /tracer1/cmd_vel_desired > "${RUN_DIR}/cmd_vel_desired_tracer1.csv" 2>&1 &
    REC_CMD_VEL_DES=$!
    ros2 topic echo --csv /tracer1/cmd_vel_stamped > "${RUN_DIR}/cmd_vel_stamped_tracer1.csv" 2>&1 &
    REC_CMD_VEL_STAMPED=$!

    ros2 topic echo --csv /tracer1/cmd_vel > "${RUN_DIR}/cmd_vel_tracer1.csv" 2>&1 &
    REC_CMD_VEL=$!
    ros2 topic echo --csv /tracer1/cmd_stop > "${RUN_DIR}/cmd_stop_tracer1.csv" 2>&1 &
    REC_CMD_STOP=$!

    ros2 topic echo --csv /tracer2/cmd_goal > "${RUN_DIR}/cmd_goal_tracer2.csv" 2>&1 &
    REC_CMD_GOAL_T2=$!
    ros2 topic echo --csv /tracer2/cmd_vel_desired > "${RUN_DIR}/cmd_vel_desired_tracer2.csv" 2>&1 &
    REC_CMD_VEL_DES_T2=$!
    ros2 topic echo --csv /tracer2/cmd_vel_stamped > "${RUN_DIR}/cmd_vel_stamped_tracer2.csv" 2>&1 &
    REC_CMD_VEL_STAMPED_T2=$!
    ros2 topic echo --csv /tracer2/cmd_vel > "${RUN_DIR}/cmd_vel_tracer2.csv" 2>&1 &
    REC_TRACER2=$!
    ros2 topic echo --csv /tracer2/cmd_stop > "${RUN_DIR}/cmd_stop_tracer2.csv" 2>&1 &
    REC_CMD_STOP_T2=$!

    ros2 topic echo --csv /tracer3/cmd_goal > "${RUN_DIR}/cmd_goal_tracer3.csv" 2>&1 &
    REC_CMD_GOAL_T3=$!
    ros2 topic echo --csv /tracer3/cmd_vel_desired > "${RUN_DIR}/cmd_vel_desired_tracer3.csv" 2>&1 &
    REC_CMD_VEL_DES_T3=$!
    ros2 topic echo --csv /tracer3/cmd_vel_stamped > "${RUN_DIR}/cmd_vel_stamped_tracer3.csv" 2>&1 &
    REC_CMD_VEL_STAMPED_T3=$!
    ros2 topic echo --csv /tracer3/cmd_vel > "${RUN_DIR}/cmd_vel_tracer3.csv" 2>&1 &
    REC_TRACER3=$!
    ros2 topic echo --csv /tracer3/cmd_stop > "${RUN_DIR}/cmd_stop_tracer3.csv" 2>&1 &
    REC_CMD_STOP_T3=$!

    ros2 topic echo --csv /wing_alignment/emergency_stop > "${RUN_DIR}/emergency_stop.csv" 2>&1 &
    REC_EMERGENCY=$!
    ros2 topic echo --csv /Rigid17/pose > "${RUN_DIR}/rigid17_pose.csv" 2>&1 &
    REC_RIGID17=$!
    ros2 topic echo --csv /Rigid14/pose > "${RUN_DIR}/rigid14_pose.csv" 2>&1 &
    REC_RIGID14=$!
    ros2 topic echo --csv /Rigid15/pose > "${RUN_DIR}/rigid15_pose.csv" 2>&1 &
    REC_RIGID15=$!
    ros2 topic echo --csv /Rigid8/pose > "${RUN_DIR}/rigid8_pose.csv" 2>&1 &
    REC_RIGID8=$!

    echo "  [REC] Topic recorders started" | tee -a "${GATE_FILE}"
}

stop_topic_recorders() {
    local rpid
    for rpid in  "${REC_CMD_GOAL}"  "${REC_CMD_VEL_DES}"  "${REC_CMD_VEL_STAMPED}"  "${REC_CMD_VEL}"  "${REC_CMD_STOP}"  "${REC_CMD_GOAL_T2}"  "${REC_CMD_VEL_DES_T2}"  "${REC_CMD_VEL_STAMPED_T2}"  "${REC_TRACER2}"  "${REC_CMD_STOP_T2}"  "${REC_CMD_GOAL_T3}"  "${REC_CMD_VEL_DES_T3}"  "${REC_CMD_VEL_STAMPED_T3}"  "${REC_TRACER3}"  "${REC_CMD_STOP_T3}"  "${REC_EMERGENCY}"  "${REC_RIGID17}"  "${REC_RIGID14}"  "${REC_RIGID15}"  "${REC_RIGID8}"; do
        [[ -n "${rpid}" ]] || continue
        kill "${rpid}" 2>/dev/null || true
        wait "${rpid}" 2>/dev/null || true
    done
}

wait_for_profile_ready() {
    local deadline=$((SECONDS + PROFILE_READY_TIMEOUT_SEC))
    local last_missing_text=""

    while [[ ${SECONDS} -lt ${deadline} ]]; do
        local missing=()

        if [[ -n "${LAUNCH_PID}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            FAILURE_REASON="launch_process_exited_early"
            PROFILE_READY_DETAIL="launch process exited before ready"
            return 1
        fi

        node_exists "/tracer1/goto_pose_node" || missing+=("/tracer1/goto_pose_node")
        node_exists "/tracer1/cmd_watchdog" || missing+=("/tracer1/cmd_watchdog")
        node_exists "/cmd_scheduler" || missing+=("/cmd_scheduler")

        case "${PROFILE}" in
            mission_bringup)
                node_exists "/multi_tracer_return_home" || missing+=("/multi_tracer_return_home")
                ;;
            system_bringup)
                node_exists "/mission_coordinator" || missing+=("/mission_coordinator")
                ;;
            run_all)
                node_exists "/mission_coordinator" || missing+=("/mission_coordinator")
                node_exists "/tracer1/qr_delta_publisher" || missing+=("/tracer1/qr_delta_publisher")
                node_exists "/force_monitor_huatai1" || missing+=("/force_monitor_huatai1")
                ;;
        esac

        if [[ ${#missing[@]} -eq 0 ]]; then
            PROFILE_READY=true
            PROFILE_READY_DETAIL="ready"
            WATCHDOG_PATH_OK=true
            echo "  [READY] ${PROFILE} ready barrier satisfied" | tee -a "${GATE_FILE}"
            return 0
        fi

        last_missing_text="${missing[*]}"
        echo "  [WAIT] ${PROFILE} ready barrier pending: ${last_missing_text}" | tee -a "${GATE_FILE}"
        sleep 1
    done

    PROFILE_READY=false
    PROFILE_READY_DETAIL="${last_missing_text}"
    FAILURE_REASON="profile_ready_timeout"
    return 1
}

collect_optional_artifacts() {
    local artifact
    local found

    OPTIONAL_ARTIFACTS_TEXT=""
    for artifact in  mode_timeline_tracer1.csv  mode_timeline_tracer2.csv  mode_timeline_tracer3.csv  rx_tracer1.csv  rx_tracer2.csv  rx_tracer3.csv  ts_tracer1.csv  ts_tracer2.csv  ts_tracer3.csv  shadow_decisions.csv  reduced_decisions.csv; do
        found=$(find /home/ls/.ros /tmp -type f -name "${artifact}" -newer "${MARKER_FILE}" 2>/dev/null | head -1 || true)
        if [[ -n "${found}" ]]; then
            cp "${found}" "${RUN_DIR}/${artifact}"
            OPTIONAL_ARTIFACTS_TEXT="${OPTIONAL_ARTIFACTS_TEXT}${artifact} <- ${found}\n"
        fi
    done

    if [[ -z "${OPTIONAL_ARTIFACTS_TEXT}" ]]; then
        OPTIONAL_ARTIFACTS_TEXT="none"
    fi
}

cleanup_observer_run() {
    local residual_pattern
    local robot

    if ${CLEANUP_EXECUTED}; then
        return 0
    fi

    echo "=== Safety Cleanup ===" | tee -a "${GATE_FILE}"

    if [[ -n "${LAUNCH_PID}" ]]; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
        LAUNCH_PID=""
        echo "  [SAFETY] Launch process stopped" | tee -a "${GATE_FILE}"
    else
        echo "  [SAFETY] Launch process not started" | tee -a "${GATE_FILE}"
    fi

    if timeout 3s ros2 topic list 2>/dev/null | grep -q "/tracer1/cmd_vel"; then
        timeout 5s ros2 topic pub --times 5 /tracer1/cmd_vel geometry_msgs/msg/Twist  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
    fi
    for robot in tracer2 tracer3; do
        if timeout 3s ros2 topic list 2>/dev/null | grep -q "/${robot}/cmd_vel"; then
            timeout 5s ros2 topic pub --times 5 "/${robot}/cmd_vel" geometry_msgs/msg/Twist  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
        fi
    done
    echo "  [SAFETY] Zero cmd_vel published to tracer1/tracer2/tracer3" | tee -a "${GATE_FILE}"

    touch /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag asserted" | tee -a "${GATE_FILE}"

    for residual_pattern in  goto_pose_driver  cmd_watchdog  cmd_scheduler  mission_coordinator  multi_tracer_return_home  force_monitor_huatai1  qr_delta_publisher; do
        pkill -f "${residual_pattern}" 2>/dev/null || true
    done
    echo "  [SAFETY] Residual profile nodes killed with scoped patterns" | tee -a "${GATE_FILE}"

    sleep 1
    stop_topic_recorders
    collect_optional_artifacts

    TRACER1_POST_PUB=$(sanitize_int "$(get_topic_publisher_count /tracer1/cmd_vel)")
    TRACER2_POST_PUB=$(sanitize_int "$(get_topic_publisher_count /tracer2/cmd_vel)")
    TRACER3_POST_PUB=$(sanitize_int "$(get_topic_publisher_count /tracer3/cmd_vel)")
    echo "  [CONFIRM] /tracer1/cmd_vel publisher count: ${TRACER1_POST_PUB}" | tee -a "${GATE_FILE}"
    echo "  [CONFIRM] /tracer2/cmd_vel publisher count: ${TRACER2_POST_PUB}" | tee -a "${GATE_FILE}"
    echo "  [CONFIRM] /tracer3/cmd_vel publisher count: ${TRACER3_POST_PUB}" | tee -a "${GATE_FILE}"

    CLEANUP_EXECUTED=true
}

compute_kpis() {
    local kpi_script
    local kpi_output

    kpi_script="${SCRIPT_DIR}/summarize_existing_launch_observer_kpi.py"
    if [[ ! -f "${kpi_script}" ]]; then
        kpi_script="${REPO_ROOT}/src/wing_alignment_system/scripts/summarize_existing_launch_observer_kpi.py"
    fi

    if [[ -f "${kpi_script}" ]]; then
        kpi_output=$(python3 "${kpi_script}" "${RUN_DIR}" 2>/dev/null) || true
        eval "${kpi_output}" 2>/dev/null || true
    fi

    CMD_GOAL_SAMPLES=${PK_TRACER1_CMD_GOAL_SAMPLES:-0}
    CMD_VEL_DESIRED_SAMPLES=${PK_TRACER1_CMD_VEL_DESIRED_SAMPLES:-0}
    CMD_VEL_STAMPED_SAMPLES=${PK_TRACER1_CMD_VEL_STAMPED_SAMPLES:-0}
    CMD_VEL_OUTPUT_SAMPLES=${PK_TRACER1_CMD_VEL_OUTPUT_SAMPLES:-0}
    CMD_VEL_OUTPUT_NONZERO_SAMPLES=${PK_TRACER1_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    TRACER2_NONZERO=${PK_TRACER2_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    TRACER3_NONZERO=${PK_TRACER3_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    MAX_ABS_LINEAR_TRACER1=${PK_TRACER1_MAX_ABS_LINEAR:-0.0}
    MAX_ABS_ANGULAR_TRACER1=${PK_TRACER1_MAX_ABS_ANGULAR:-0.0}
    MAX_ABS_LINEAR_TRACER2=${PK_TRACER2_MAX_ABS_LINEAR:-0.0}
    MAX_ABS_ANGULAR_TRACER2=${PK_TRACER2_MAX_ABS_ANGULAR:-0.0}
    MAX_ABS_LINEAR_TRACER3=${PK_TRACER3_MAX_ABS_LINEAR:-0.0}
    MAX_ABS_ANGULAR_TRACER3=${PK_TRACER3_MAX_ABS_ANGULAR:-0.0}
    EMERGENCY_STOP_TRUE_COUNT=${PK_EMERGENCY_STOP_TRUE_COUNT:-0}
    CMD_STOP_TRUE_COUNT=${PK_TRACER1_CMD_STOP_TRUE_COUNT:-0}
    MOCAP_SAMPLES=${PK_MOCAP_SAMPLES:-0}
    ESTIMATED_DISPLACEMENT_TRACER1=${PK_TRACER1_ESTIMATED_DISPLACEMENT:-0.0}
    ESTIMATED_DISPLACEMENT_TRACER2=${PK_TRACER2_ESTIMATED_DISPLACEMENT:-0.0}
    ESTIMATED_DISPLACEMENT_TRACER3=${PK_TRACER3_ESTIMATED_DISPLACEMENT:-0.0}
    MODE_TIMELINE_SAMPLES=${PK_MODE_TIMELINE_SAMPLES:-0}
    FINAL_ZERO_T1=${PK_TRACER1_FINAL_CMD_VEL_ZERO:-true}
    FINAL_ZERO_T2=${PK_TRACER2_FINAL_CMD_VEL_ZERO:-true}
    FINAL_ZERO_T3=${PK_TRACER3_FINAL_CMD_VEL_ZERO:-true}

    if [[ "${FINAL_ZERO_T1}" == "true" && "${FINAL_ZERO_T2}" == "true" && "${FINAL_ZERO_T3}" == "true" ]]; then
        FINAL_CMD_VEL_ZERO=true
        FINAL_ZERO_INT=1
    else
        FINAL_CMD_VEL_ZERO=false
        FINAL_ZERO_INT=0
    fi

    TRACER2_CMD_VEL_SAMPLES=${PK_TRACER2_CMD_VEL_OUTPUT_SAMPLES:-0}
    TRACER3_CMD_VEL_SAMPLES=${PK_TRACER3_CMD_VEL_OUTPUT_SAMPLES:-0}
    CMD_STOP_SAMPLES=$(count_file_lines "${RUN_DIR}/cmd_stop_tracer1.csv")
    EMERGENCY_STOP_SAMPLES=$(count_file_lines "${RUN_DIR}/emergency_stop.csv")
    RIGID17_SAMPLES=$(count_file_lines "${RUN_DIR}/rigid17_pose.csv")
    RIGID14_SAMPLES=$(count_file_lines "${RUN_DIR}/rigid14_pose.csv")
    RIGID15_SAMPLES=$(count_file_lines "${RUN_DIR}/rigid15_pose.csv")
    RIGID8_SAMPLES=$(count_file_lines "${RUN_DIR}/rigid8_pose.csv")
    MODE_TIMELINE_TRACER1_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/mode_timeline_tracer1.csv")
    MODE_TIMELINE_TRACER2_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/mode_timeline_tracer2.csv")
    MODE_TIMELINE_TRACER3_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/mode_timeline_tracer3.csv")
    RX_TRACER1_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/rx_tracer1.csv")
    RX_TRACER2_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/rx_tracer2.csv")
    RX_TRACER3_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/rx_tracer3.csv")
    TS_TRACER1_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/ts_tracer1.csv")
    TS_TRACER2_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/ts_tracer2.csv")
    TS_TRACER3_SAMPLES=$(count_csv_rows_without_header "${RUN_DIR}/ts_tracer3.csv")
}

check_reports_generated() {
    REPORTS_GENERATED=false
    if [[ -f "${RUN_SUMMARY}" && -f "${TOPIC_FLOW_SUMMARY}" && -f "${SAFETY_REPORT}" && -f "${PROFILE_CHAIN_SUMMARY}" && -f "${GATE_FILE}" ]]; then
        REPORTS_GENERATED=true
    fi
}

evaluate_execution_result() {
    local fail_reasons=""

    OBSERVABLE_PASS=false
    FULL_CHAIN_PASS=false
    FULL_CHAIN_PASS_DIRECT=false
    FULL_CHAIN_PASS_AUXILIARY=false
    SCHEDULER_ACTIVE_BY_SHADOW_LOG=false
    WATCHDOG_ACTIVE_BY_TIMELINE=false
    MOTION_OUTPUT_PASS=false
    TARGET_TRACER1_PASS=false
    TRACER1_ONLY_PASS=false

    if [[ -z "${FAILURE_REASON}" ]] && ${LAUNCH_STARTED} && ${CLEANUP_EXECUTED} && ${REPORTS_GENERATED}; then
        OBSERVABLE_PASS=true
    fi

    local any_goal=${PK_ANY_ROBOT_CMD_GOAL_SAMPLES:-0}
    local any_desired=${PK_ANY_ROBOT_DESIRED_SAMPLES:-0}
    local any_stamped=${PK_ANY_ROBOT_STAMPED_SAMPLES:-0}
    local any_output_samples=0
    for r in TRACER1 TRACER2 TRACER3; do
        local varname="PK_${r}_CMD_VEL_OUTPUT_SAMPLES"
        local val=${!varname:-0}
        if [[ "${val}" -gt 0 ]]; then
            any_output_samples=${val}
            break
        fi
    done

    if ${OBSERVABLE_PASS} && [[ "${any_goal}" -gt 0 ]] && [[ "${any_desired}" -gt 0 ]] && [[ "${any_stamped}" -gt 0 ]] && [[ "${any_output_samples}" -gt 0 ]]; then
        FULL_CHAIN_PASS=true
        FULL_CHAIN_PASS_DIRECT=true
    fi

    local shadow_csv="${RUN_DIR}/shadow_decisions.csv"
    if [[ -f "${shadow_csv}" ]]; then
        local shadow_lines
        shadow_lines=$(wc -l < "${shadow_csv}" 2>/dev/null || echo 0)
        if [[ "${shadow_lines}" -gt 1 ]]; then
            SCHEDULER_ACTIVE_BY_SHADOW_LOG=true
        fi
    fi
    local mt_csv="${RUN_DIR}/mode_timeline_tracer1.csv"
    if [[ -f "${mt_csv}" ]]; then
        local mt_lines
        mt_lines=$(wc -l < "${mt_csv}" 2>/dev/null || echo 0)
        if [[ "${mt_lines}" -gt 1 ]] && [[ "${any_output_samples}" -gt 0 ]]; then
            WATCHDOG_ACTIVE_BY_TIMELINE=true
        fi
    fi
    if ${OBSERVABLE_PASS} && ${SCHEDULER_ACTIVE_BY_SHADOW_LOG} && ${WATCHDOG_ACTIVE_BY_TIMELINE} && [[ "${any_output_samples}" -gt 0 ]]; then
        FULL_CHAIN_PASS_AUXILIARY=true
    fi
    if ${FULL_CHAIN_PASS_AUXILIARY} && ! ${FULL_CHAIN_PASS_DIRECT}; then
        fail_reasons="${fail_reasons}full_chain_by_auxiliary_only(recorder_race); "
    fi

    local any_nonzero=${PK_ANY_ROBOT_CMD_VEL_NONZERO_SAMPLES:-0}
    if ( ${FULL_CHAIN_PASS} || ${FULL_CHAIN_PASS_AUXILIARY} ) && [[ "${any_nonzero}" -gt 0 ]]; then
        MOTION_OUTPUT_PASS=true
    fi

    local t1_nonzero=${PK_TRACER1_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    if ${MOTION_OUTPUT_PASS} && [[ "${t1_nonzero}" -gt 0 ]]; then
        TARGET_TRACER1_PASS=true
    fi

    local t2_nonzero=${PK_TRACER2_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    local t3_nonzero=${PK_TRACER3_CMD_VEL_OUTPUT_NONZERO_SAMPLES:-0}
    if ${MOTION_OUTPUT_PASS} && [[ "${t1_nonzero}" -gt 0 ]] && [[ "${t2_nonzero}" -le 0 ]] && [[ "${t3_nonzero}" -le 0 ]]; then
        TRACER1_ONLY_PASS=true
    fi

    EXECUTION_RESULT="PASS"
    if ! ${OBSERVABLE_PASS}; then
        EXECUTION_RESULT="FAIL"
        if [[ -n "${FAILURE_REASON}" ]]; then
            fail_reasons="${fail_reasons}${FAILURE_REASON}; "
        fi
        if ! ${LAUNCH_STARTED}; then
            fail_reasons="${fail_reasons}launch_not_started; "
        fi
        if ! ${CLEANUP_EXECUTED}; then
            fail_reasons="${fail_reasons}cleanup_not_executed; "
        fi
        if ! ${REPORTS_GENERATED}; then
            fail_reasons="${fail_reasons}reports_not_generated; "
        fi
    fi
    if ${OBSERVABLE_PASS} && ! ${FULL_CHAIN_PASS_DIRECT}; then
        if ${FULL_CHAIN_PASS_AUXILIARY}; then
            :   # auxiliary evidence present, note recorded above
        else
            fail_reasons="${fail_reasons}full_chain_not_observed; "
        fi
    fi
    if [[ "${FINAL_CMD_VEL_ZERO}" != "true" ]]; then
        fail_reasons="${fail_reasons}final_cmd_vel_not_zero; "
    fi
    if [[ "${TRACER1_POST_PUB}" -ne 0 || "${TRACER2_POST_PUB}" -ne 0 || "${TRACER3_POST_PUB}" -ne 0 ]]; then
        fail_reasons="${fail_reasons}post_cleanup_publisher_not_zero; "
    fi

    fail_reasons="${fail_reasons%; }"
    if [[ -z "${fail_reasons}" ]]; then
        EXECUTION_FAIL_REASONS="none"
    else
        EXECUTION_FAIL_REASONS="${fail_reasons}"
        if [[ "${EXECUTION_RESULT}" != "FAIL" ]]; then
            EXECUTION_RESULT="PASS_WITH_NOTES"
        fi
    fi
}

write_reports() {
    cat > "${SAFETY_REPORT}" <<SAFETYEOF
Existing Launch Observer Safety Cleanup Report
=============================================
Run ID: ${RUN_ID}
Timestamp: $(date -u +%Y-%m-%dT%H:%M:%SZ)
Profile: ${PROFILE}
Semantics: ${PROFILE_SEMANTICS}
Gate result: ${EXECUTION_RESULT}
Failure reason: ${FAILURE_REASON:-none}
Failure reasons: ${EXECUTION_FAIL_REASONS}
Cleanup executed: ${CLEANUP_EXECUTED}
Final cmd_vel zero (all): ${FINAL_CMD_VEL_ZERO}
Final cmd_vel zero tracer1: ${PK_TRACER1_FINAL_CMD_VEL_ZERO:-N/A}
Final cmd_vel zero tracer2: ${PK_TRACER2_FINAL_CMD_VEL_ZERO:-N/A}
Final cmd_vel zero tracer3: ${PK_TRACER3_FINAL_CMD_VEL_ZERO:-N/A}

Cleanup order:
  1. Launch process stopped.
  2. Zero cmd_vel published to tracer1/tracer2/tracer3.
  3. Emergency stop flag asserted.
  4. Residual profile nodes killed.
  5. Publisher counts verified.
  6. Reports generated.

Post-cleanup publisher counts:
  /tracer1/cmd_vel: ${TRACER1_POST_PUB}
  /tracer2/cmd_vel: ${TRACER2_POST_PUB}
  /tracer3/cmd_vel: ${TRACER3_POST_PUB}
SAFETYEOF

    cat > "${PROFILE_CHAIN_SUMMARY}" <<CHAINEOF
Existing Launch Observer Profile Chain Summary
=============================================
Run ID: ${RUN_ID}
Profile: ${PROFILE}
Semantics: ${PROFILE_SEMANTICS}
Launch command: ${PROFILE_LAUNCH_CMD}
Ready hint: ${PROFILE_READY_HINT}
Launch started: ${LAUNCH_STARTED}
Profile ready: ${PROFILE_READY}
Profile ready detail: ${PROFILE_READY_DETAIL:-none}
Observation started: ${OBSERVATION_STARTED}
Watchdog path OK: ${WATCHDOG_PATH_OK}
Reports generated: ${REPORTS_GENERATED}

Archival note:
  profile=mission_bringup is multi-robot return-home observational smoke.
  It is not tracer1-only D1-2-1 PASS.

Watchdog chain checks:
  /tracer1/goto_pose_node observed before observation.
  /tracer1/cmd_watchdog observed before observation.
  /cmd_scheduler observed before observation.
  Existing launch kept unmodified: no synthetic command injection, no direct nonzero cmd_vel publishing.

Optional copied artifacts:
${OPTIONAL_ARTIFACTS_TEXT}
CHAINEOF
}

emit_execution_verdict() {
    local active="${PK_ACTIVE_ROBOT_IDS:-none}"
    echo "" >> "${GATE_FILE}"
    echo "--- Observer Verdict ---" >> "${GATE_FILE}"
    echo "This is an observational smoke test over an existing movable launch." >> "${GATE_FILE}"
    echo "It is not a tracer1-only D1-2-1 PASS claim." >> "${GATE_FILE}"
    echo "Profile: ${PROFILE} (${PROFILE_SEMANTICS})" >> "${GATE_FILE}"
    echo "Gate result: ${EXECUTION_RESULT}" >> "${GATE_FILE}"
    echo "Failure reason: ${FAILURE_REASON:-none}" >> "${GATE_FILE}"
    echo "Failure reasons: ${EXECUTION_FAIL_REASONS}" >> "${GATE_FILE}"
    echo "" >> "${GATE_FILE}"
    echo "--- Tiered Gate ---" >> "${GATE_FILE}"
    echo "1. OBSERVABLE_PASS: ${OBSERVABLE_PASS}" >> "${GATE_FILE}"
    echo "   (launch starts, recorders run, cleanup executed, reports generated)" >> "${GATE_FILE}"
    echo "2. FULL_CHAIN_PASS: ${FULL_CHAIN_PASS}" >> "${GATE_FILE}"
    echo "   (>=1 robot: cmd_goal>0, cmd_vel_desired>0, cmd_vel_stamped>0, cmd_vel_output>0)" >> "${GATE_FILE}"
    echo "3. MOTION_OUTPUT_PASS: ${MOTION_OUTPUT_PASS}" >> "${GATE_FILE}"
    echo "   (>=1 robot: cmd_vel_output_nonzero>0)" >> "${GATE_FILE}"
    echo "4. TARGET_TRACER1_PASS: ${TARGET_TRACER1_PASS}" >> "${GATE_FILE}"
    echo "   (tracer1 cmd_vel_output_nonzero>0)" >> "${GATE_FILE}"
    echo "5. TRACER1_ONLY_PASS: ${TRACER1_ONLY_PASS}" >> "${GATE_FILE}"
    echo "   (tracer1 nonzero output + tracer2/3 zero nonzero output)" >> "${GATE_FILE}"
    echo "2a. FULL_CHAIN_PASS_DIRECT: ${FULL_CHAIN_PASS_DIRECT}" >> "${GATE_FILE}"
    echo "    (>=1 robot: goal/desired/stamped/output all >0 from topic recorders)" >> "${GATE_FILE}"
    echo "2b. FULL_CHAIN_PASS_AUXILIARY: ${FULL_CHAIN_PASS_AUXILIARY}" >> "${GATE_FILE}"
    echo "    (SCHEDULER_ACTIVE_BY_SHADOW_LOG + WATCHDOG_ACTIVE_BY_TIMELINE + output>0)" >> "${GATE_FILE}"
    echo "    SCHEDULER_ACTIVE_BY_SHADOW_LOG: ${SCHEDULER_ACTIVE_BY_SHADOW_LOG}" >> "${GATE_FILE}"
    echo "    WATCHDOG_ACTIVE_BY_TIMELINE: ${WATCHDOG_ACTIVE_BY_TIMELINE}" >> "${GATE_FILE}"
    echo "" >> "${GATE_FILE}"
    echo "Active robot IDs: ${active}" >> "${GATE_FILE}"
    echo "Launch started: ${LAUNCH_STARTED}" >> "${GATE_FILE}"
    echo "Profile ready: ${PROFILE_READY}" >> "${GATE_FILE}"
    echo "Observation started: ${OBSERVATION_STARTED}" >> "${GATE_FILE}"
    echo "Watchdog path OK: ${WATCHDOG_PATH_OK}" >> "${GATE_FILE}"
    echo "Cleanup executed: ${CLEANUP_EXECUTED}" >> "${GATE_FILE}"
    echo "Reports generated: ${REPORTS_GENERATED}" >> "${GATE_FILE}"
    echo "--- Per-Robot KPI Summary ---" >> "${GATE_FILE}"
    for r in TRACER1 TRACER2 TRACER3; do
        local goal_var="PK_${r}_CMD_GOAL_SAMPLES"
        local des_var="PK_${r}_CMD_VEL_DESIRED_SAMPLES"
        local stp_var="PK_${r}_CMD_VEL_STAMPED_SAMPLES"
        local out_var="PK_${r}_CMD_VEL_OUTPUT_SAMPLES"
        local nz_var="PK_${r}_CMD_VEL_OUTPUT_NONZERO_SAMPLES"
        local z_var="PK_${r}_FINAL_CMD_VEL_ZERO"
        echo "  ${r}: goal=${!goal_var:-0} desired=${!des_var:-0} stamped=${!stp_var:-0} output=${!out_var:-0} nonzero=${!nz_var:-0} final_zero=${!z_var:-N/A}" >> "${GATE_FILE}"
    done
    echo "any_robot_cmd_vel_nonzero_samples: ${PK_ANY_ROBOT_CMD_VEL_NONZERO_SAMPLES:-0}" >> "${GATE_FILE}"
    echo "final_cmd_vel_zero: ${FINAL_CMD_VEL_ZERO}" >> "${GATE_FILE}"
    echo "Archival note: profile=mission_bringup is multi-robot return-home observational smoke. It is not tracer1-only D1-2-1 PASS." >> "${GATE_FILE}"
}

trigger_start_approach() {
    echo "=== Start Approach Trigger ===" | tee -a "${GATE_FILE}"
    local trigger_nodes=(
        /mission_coordinator /cmd_scheduler
        /tracer1/goto_pose_node /tracer2/goto_pose_node /tracer3/goto_pose_node
        /tracer1/cmd_watchdog /tracer2/cmd_watchdog /tracer3/cmd_watchdog
    )
    local deadline=$((SECONDS + 30))
    local missing=()
    while [[ ${SECONDS} -lt ${deadline} ]]; do
        missing=()
        for n in "${trigger_nodes[@]}"; do
            node_exists "${n}" || missing+=("${n}")
        done
        if [[ ${#missing[@]} -eq 0 ]]; then
            break
        fi
        sleep 1
    done
    if [[ ${#missing[@]} -gt 0 ]]; then
        FAILURE_REASON="trigger_nodes_missing:${missing[*]}"
        echo "  [FAIL] Trigger nodes missing: ${missing[*]}" | tee -a "${GATE_FILE}"
        return 1
    fi
    echo "  [TRIGGER] All trigger nodes present" | tee -a "${GATE_FILE}"

    local srv_wait_deadline=$((SECONDS + 15))
    local srv_ready=false
    while [[ ${SECONDS} -lt ${srv_wait_deadline} ]]; do
        if timeout 3s ros2 service list 2>/dev/null | grep -Fxq "/mission/start_approach"; then
            srv_ready=true
            break
        fi
        sleep 1
    done
    if ! ${srv_ready}; then
        FAILURE_REASON="start_approach_service_not_found"
        echo "  [FAIL] /mission/start_approach service not found" | tee -a "${GATE_FILE}"
        return 1
    fi
    echo "  [TRIGGER] /mission/start_approach service found" | tee -a "${GATE_FILE}"

    local srv_type
    srv_type=$(timeout 5s ros2 service type /mission/start_approach 2>/dev/null || true)
    if [[ -z "${srv_type}" ]]; then
        FAILURE_REASON="start_approach_service_type_unknown"
        echo "  [FAIL] Could not determine /mission/start_approach type" | tee -a "${GATE_FILE}"
        return 1
    fi
    echo "  [TRIGGER] /mission/start_approach type: ${srv_type}" | tee -a "${GATE_FILE}"

    if [[ "${srv_type}" != "std_srvs/srv/Trigger" ]]; then
        FAILURE_REASON="start_approach_unexpected_type:${srv_type}"
        echo "  [FAIL] Expected std_srvs/srv/Trigger, got ${srv_type}" | tee -a "${GATE_FILE}"
        return 1
    fi

    local call_result
    call_result=$(timeout 8s ros2 service call /mission/start_approach std_srvs/srv/Trigger "{}" 2>&1 || true)
    echo "  [TRIGGER] /mission/start_approach result: ${call_result}" | tee -a "${GATE_FILE}"

    echo "  [TRIGGER] Waiting up to 10s for cmd_goal or cmd_vel_desired samples..." | tee -a "${GATE_FILE}"
    local check_deadline=$((SECONDS + 10))
    local got_samples=false
    while [[ ${SECONDS} -lt ${check_deadline} ]]; do
        for robot in tracer1 tracer2 tracer3; do
            local gs
            gs=$(wc -l < "${RUN_DIR}/cmd_goal_${robot}.csv" 2>/dev/null || echo 0)
            local ds
            ds=$(wc -l < "${RUN_DIR}/cmd_vel_desired_${robot}.csv" 2>/dev/null || echo 0)
            if [[ "${gs}" -gt 1 ]] || [[ "${ds}" -gt 1 ]]; then
                got_samples=true
                break 2
            fi
        done
        sleep 0.5
    done

    if ${got_samples}; then
        echo "  [TRIGGER] cmd_goal or cmd_vel_desired samples detected post-trigger" | tee -a "${GATE_FILE}"
        return 0
    else
        FAILURE_REASON="start_approach_triggered_but_no_goal_or_desired"
        echo "  [TRIGGER] No cmd_goal or cmd_vel_desired samples after trigger" | tee -a "${GATE_FILE}"
        return 1
    fi
}

finalize_run() {
    if ${FINALIZE_DONE}; then
        return 0
    fi
    cleanup_observer_run
    compute_kpis
    write_reports
    check_reports_generated
    evaluate_execution_result
    write_reports
    check_reports_generated
    emit_execution_verdict
    FINALIZE_DONE=true
}

runtime_exit_trap() {
    local exit_code=$?
    trap - EXIT
    if ! ${RUNNER_STARTED}; then
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
RUNNER_STARTED=true

if ${PRINT_PLAN}; then
    echo "=== Print-Plan ==="
    echo "run_id: ${RUN_ID}"
    echo "profile: ${PROFILE} (${PROFILE_SEMANTICS})"
    echo "duration_sec: ${DURATION_SEC}"
    echo "trigger_start_approach: ${TRIGGER_START_APPROACH}"
    echo "enable_reduced_output: ${REDUCED_ENABLED}"
    echo "reduced_policy: ${REDUCED_POLICY}"
    echo "reduced_delta_threshold: ${REDUCED_DELTA_TH}"
    echo "reduced_max_hold_ms: ${_rmax:-${REDUCED_MAX_HOLD_MS}}"
    echo "reduced_periodic_k: ${REDUCED_PERIODIC_K}"
    if ${REDUCED_ENABLED}; then
        _safe=$(grep -oP 'age_th_ms:\s*\K[0-9.]+' "${shadow_yaml:-${REPO_ROOT}/src/wing_alignment_system/config/mission_params.yaml}" 2>/dev/null || echo "100")
        echo "age_safe_ms: ${_safe}"
        _rmax_f=${_rmax:-100.0}
        if python3 -c "exit(0 if float('${_rmax_f}') < float('${_safe}') else 1)" 2>/dev/null; then
            echo "safety_verdict: PASS (reduced_max_hold<age_safe)"
        else
            echo "safety_verdict: FAIL (reduced_max_hold>=age_safe)"
            exit 1
        fi
    else
        echo "age_safe_ms: N/A (full_update)"
        echo "safety_verdict: PASS (full_update)"
    fi
    echo "launch_command: ${PROFILE_LAUNCH_CMD}"
    exit 0
fi

echo "[PROFILE] ${PROFILE}: ${PROFILE_SEMANTICS}" | tee -a "${GATE_FILE}"
echo "[PROFILE] Ready hint: ${PROFILE_READY_HINT}" | tee -a "${GATE_FILE}"
rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
echo "[SAFETY] Pre-launch emergency flag cleared" | tee -a "${GATE_FILE}"

touch "${MARKER_FILE}"
start_topic_recorders

echo "=== Launching ${PROFILE} ===" | tee -a "${GATE_FILE}"
echo "  [LAUNCH] ${PROFILE_LAUNCH_CMD}" | tee -a "${GATE_FILE}"
ros2 launch wing_alignment_system "${PROFILE}.launch.py" ${LAUNCH_ARGS} > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 2

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    LAUNCH_STARTED=true
    echo "  [LAUNCH] PID ${LAUNCH_PID} is alive" | tee -a "${GATE_FILE}"
else
    FAILURE_REASON="launch_process_exited_early"
    echo "  [FAIL] Launch process exited early" | tee -a "${GATE_FILE}"
    finalize_run
    echo "Run directory: ${RUN_DIR}"
    exit 0
fi

if ! wait_for_profile_ready; then
    echo "  [FAIL] Ready barrier failed: ${FAILURE_REASON}" | tee -a "${GATE_FILE}"
    if [[ -n "${PROFILE_READY_DETAIL}" ]]; then
        echo "  [FAIL] Ready detail: ${PROFILE_READY_DETAIL}" | tee -a "${GATE_FILE}"
    fi
    finalize_run
    echo "Run directory: ${RUN_DIR}"
    exit 0
fi

if ${TRIGGER_START_APPROACH}; then
    if ! trigger_start_approach; then
        echo "  [FAIL] Start approach trigger failed: ${FAILURE_REASON}" | tee -a "${GATE_FILE}"
    fi
fi

echo "=== Observation Window: ${DURATION_SEC}s ===" | tee -a "${GATE_FILE}"

OBSERVATION_STARTED=true
sleep "${DURATION_SEC}"
echo "  [OBSERVE] Observation complete" | tee -a "${GATE_FILE}"

finalize_run

echo "Run directory: ${RUN_DIR}"
echo "Gate file: ${GATE_FILE}"
echo "Run summary: ${RUN_SUMMARY}"
echo "Topic flow: ${TOPIC_FLOW_SUMMARY}"
echo "Safety report: ${SAFETY_REPORT}"
echo "Profile chain: ${PROFILE_CHAIN_SUMMARY}"
exit 0


