#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"
ARTIFACT_ROOT_DEFAULT="${HOME}/.ros/fr_tac_p3d_run_all_resource_audit"

RUN_ID="p3d_run_all_resource_audit"
ARTIFACT_ROOT="${ARTIFACT_ROOT_DEFAULT}"
DURATION_SEC=20
PROFILE_READY_TIMEOUT_SEC=45
PROFILE="run_all"

usage() {
    cat <<'USAGEEOF'
Usage: run_fr_tac_p3d_run_all_resource_audit.sh [OPTIONS]

Resource audit for run_all camera/full-process launch.
Records system resources, camera topic bandwidth, ROS node/topic topology.
No image recording. No motion. Duration <= 20s.

Options:
  --run-id ID         Run identifier (default: p3d_run_all_resource_audit)
  --duration-sec SEC  Observation duration (default: 20, max: 20)
  -h, --help          Show this help
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

node_exists() {
    local node_name="$1"
    timeout 3s ros2 node list 2>/dev/null | grep -Fxq "${node_name}"
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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-id) RUN_ID="${2}"; shift 2 ;;
        --duration-sec) DURATION_SEC="${2}"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1" >&2; usage ;;
    esac
done

DURATION_SEC=$(sanitize_int "${DURATION_SEC}")
if [[ "${DURATION_SEC}" -le 0 ]]; then
    echo "ERROR: --duration-sec must be > 0" >&2
    exit 2
fi
if [[ "${DURATION_SEC}" -gt 20 ]]; then
    echo "WARNING: --duration-sec capped at 20s" >&2
    DURATION_SEC=20
fi

LAUNCH_ARGS=""
LAUNCH_CMD="ros2 launch wing_alignment_system run_all.launch.py"

RUN_DIR="${ARTIFACT_ROOT%/}/${RUN_ID}"
mkdir -p "${RUN_DIR}"
GATE_FILE="${RUN_DIR}/run_all_resource_audit_gate.txt"
RESOURCE_SUMMARY="${RUN_DIR}/resource_summary.csv"
TOPIC_BW_SUMMARY="${RUN_DIR}/topic_bandwidth_summary.csv"
NODE_PROCESS_SUMMARY="${RUN_DIR}/node_process_summary.txt"
RECOMMENDATIONS="${RUN_DIR}/recommendations.md"
LAUNCH_LOG="${RUN_DIR}/launch.log"
SYS_SNAPSHOT="${RUN_DIR}/system_snapshot.txt"
TOPIC_LIST="${RUN_DIR}/topic_list.txt"
NODE_LIST="${RUN_DIR}/node_list.txt"

LAUNCH_PID=""
LAUNCH_STARTED=false
PROFILE_READY=false
CLEANUP_EXECUTED=false
RUNNER_STARTED=false
FAILURE_REASON=""

CAMERA_TOPIC_COUNT=0
HIGH_BW_COUNT=0
ROS_CLI_TIMEOUT_COUNT=0
MHZ_COUNT=0
HZ_TIMEOUT=0
BW_TIMEOUT=0

COLLECT_OK=false
ROS_RESPONSIVE=true

PROFILE_LAUNCH_FILE="${REPO_ROOT}/src/wing_alignment_system/launch/${PROFILE}.launch.py"

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
    export ROS_DOMAIN_ID=36
fi

echo "============================================================" | tee "${GATE_FILE}"
echo "run_all Resource Audit" | tee -a "${GATE_FILE}"
echo "============================================================" | tee -a "${GATE_FILE}"
echo "Run ID: ${RUN_ID}" | tee -a "${GATE_FILE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}" | tee -a "${GATE_FILE}"
echo "Launch command: ${LAUNCH_CMD}" | tee -a "${GATE_FILE}"
echo "" | tee -a "${GATE_FILE}"

wait_for_profile_ready() {
    local deadline=$((SECONDS + PROFILE_READY_TIMEOUT_SEC))
    local last_missing_text=""

    while [[ ${SECONDS} -lt ${deadline} ]]; do
        local missing=()

        if [[ -n "${LAUNCH_PID}" ]] && ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            FAILURE_REASON="launch_process_exited_early"
            return 1
        fi

        node_exists "/tracer1/goto_pose_node" || missing+=("/tracer1/goto_pose_node")
        node_exists "/tracer1/cmd_watchdog" || missing+=("/tracer1/cmd_watchdog")
        node_exists "/cmd_scheduler" || missing+=("/cmd_scheduler")
        node_exists "/mission_coordinator" || missing+=("/mission_coordinator")
        node_exists "/tracer1/qr_delta_publisher" || missing+=("/tracer1/qr_delta_publisher")
        node_exists "/force_monitor_huatai1" || missing+=("/force_monitor_huatai1")

        if [[ ${#missing[@]} -eq 0 ]]; then
            PROFILE_READY=true
            echo "  [READY] run_all ready barrier satisfied" | tee -a "${GATE_FILE}"
            return 0
        fi

        last_missing_text="${missing[*]}"
        echo "  [WAIT] ready barrier pending: ${last_missing_text}" | tee -a "${GATE_FILE}"
        sleep 1
    done

    FAILURE_REASON="profile_ready_timeout"
    return 1
}

collect_system_snapshot() {
    {
        echo "=== System Snapshot  $(date -u +%Y-%m-%dT%H:%M:%SZ) ==="
        echo ""
        echo "--- top -bn1 | head -20 ---"
        top -bn1 2>/dev/null | head -20 || echo "top failed"
        echo ""
        echo "--- free -h ---"
        free -h 2>/dev/null || echo "free failed"
        echo ""
        echo "--- df -h ---"
        df -h 2>/dev/null || echo "df failed"
        echo ""
        echo "--- Load Average ---"
        cat /proc/loadavg 2>/dev/null || echo "loadavg failed"
        echo ""
        echo "--- uptime ---"
        uptime 2>/dev/null || echo "uptime failed"
    } > "${RUN_DIR}/system_snapshot_$(date +%s).txt" 2>/dev/null
    cp "${RUN_DIR}/system_snapshot_$(date +%s).txt" "${SYS_SNAPSHOT}" 2>/dev/null || true
    echo "  [AUDIT] System snapshot collected" | tee -a "${GATE_FILE}"
}

collect_ros_topology() {
    timeout 8s ros2 node list > "${NODE_LIST}" 2>/dev/null || true
    timeout 8s ros2 topic list > "${TOPIC_LIST}" 2>/dev/null || true
    local node_count
    node_count=$(wc -l < "${NODE_LIST}" 2>/dev/null || echo 0)
    local topic_count
    topic_count=$(wc -l < "${TOPIC_LIST}" 2>/dev/null || echo 0)
    echo "  [AUDIT] ROS topology: ${node_count} nodes, ${topic_count} topics" | tee -a "${GATE_FILE}"

    COLLECT_OK=true
    if [[ "${node_count}" -le 0 && "${topic_count}" -le 0 ]]; then
        ROS_RESPONSIVE=false
        COLLECT_OK=false
        FAILURE_REASON="${FAILURE_REASON:+${FAILURE_REASON};}ros_cli_unresponsive"
    fi
}

audit_camera_topics() {
    local cand=""
    local found
    local topic

    echo "=== Camera / Vision Topic Audit $(date -u +%Y-%m-%dT%H:%M:%SZ) ===" > "${TOPIC_BW_SUMMARY}"
    echo "=== Node / Process Summary $(date -u +%Y-%m-%dT%H:%M:%SZ) ===" > "${NODE_PROCESS_SUMMARY}"

    cand=$(timeout 8s ros2 topic list 2>/dev/null | grep -Ei "image|camera|qr|force|delta|compressed" || true)

    if [[ -z "${cand}" ]]; then
        echo "No camera/vision-related topics found." | tee -a "${GATE_FILE}" "${TOPIC_BW_SUMMARY}"
        echo "topic,hz_result,hz_timeout_sec,bw_result,bw_timeout_sec" >> "${TOPIC_BW_SUMMARY}"
        return 0
    fi

    echo "topic,hz_result,hz_timeout_sec,bw_result,bw_timeout_sec" >> "${TOPIC_BW_SUMMARY}"

    echo "  [AUDIT] Scanning camera/vision topics..." | tee -a "${GATE_FILE}"
    while IFS= read -r topic; do
        [[ -z "${topic}" ]] && continue
        CAMERA_TOPIC_COUNT=$((CAMERA_TOPIC_COUNT + 1))
        local hz_result=""
        local bw_result=""
        local hz_timeout=0
        local bw_timeout=0

        local start_sec=${SECONDS}
        hz_result=$(timeout 5s ros2 topic hz "${topic}" 2>&1 || true)
        local hz_elapsed=$((SECONDS - start_sec))
        if [[ ${hz_elapsed} -ge 5 ]]; then
            hz_timeout=1
            HZ_TIMEOUT=$((HZ_TIMEOUT + 1))
            ROS_CLI_TIMEOUT_COUNT=$((ROS_CLI_TIMEOUT_COUNT + 1))
            echo "  [TIMEOUT] ros2 topic hz ${topic}" | tee -a "${GATE_FILE}"
            ROS_RESPONSIVE=false
        fi

        start_sec=${SECONDS}
        bw_result=$(timeout 5s ros2 topic bw "${topic}" 2>&1 || true)
        local bw_elapsed=$((SECONDS - start_sec))
        if [[ ${bw_elapsed} -ge 5 ]]; then
            bw_timeout=1
            BW_TIMEOUT=$((BW_TIMEOUT + 1))
            ROS_CLI_TIMEOUT_COUNT=$((ROS_CLI_TIMEOUT_COUNT + 1))
            echo "  [TIMEOUT] ros2 topic bw ${topic}" | tee -a "${GATE_FILE}"
            ROS_RESPONSIVE=false
        fi

        local hz_val
        hz_val=$(echo "${hz_result}" | grep -oP 'average rate:\s*\K[0-9.]+' | tail -1 || echo "N/A")
        local bw_val
        bw_val=$(echo "${bw_result}" | grep -oP 'average:\s*\K[0-9.]+(?=\s*[KMG]?B/s)' | tail -1 || echo "N/A")

        echo "${topic},${hz_val},${hz_timeout},${bw_val},${bw_timeout}" >> "${TOPIC_BW_SUMMARY}"

        if [[ "${bw_val}" != "N/A" ]]; then
            local bw_num
            bw_num=$(echo "${bw_val}" | grep -oP '^[0-9.]+' || echo 0)
            if [[ -n "${bw_num}" ]]; then
                local bw_int
                bw_int=$(printf '%.0f' "${bw_num}" 2>/dev/null || echo 0)
                if [[ "${bw_int}" -gt 50 ]]; then
                    HIGH_BW_COUNT=$((HIGH_BW_COUNT + 1))
                    echo "  [HIGH_BW] ${topic}: ${bw_val} B/s" | tee -a "${GATE_FILE}"
                fi
            fi
        fi

        local hz_num
        hz_num=$(echo "${hz_val}" | grep -oP '^[0-9.]+' || echo 0)
        if [[ "${hz_val}" != "N/A" && -n "${hz_num}" ]]; then
            local hz_int
            hz_int=$(printf '%.0f' "${hz_num}" 2>/dev/null || echo 0)
            if [[ "${hz_int}" -gt 29 ]]; then
                MHZ_COUNT=$((MHZ_COUNT + 1))
                echo "  [HIGH_HZ] ${topic}: ${hz_val} Hz" | tee -a "${GATE_FILE}"
            fi
        fi

    done <<< "${cand}"

    echo "  [AUDIT] Camera topics scanned: ${CAMERA_TOPIC_COUNT}, high-bw: ${HIGH_BW_COUNT}, hz-timeouts: ${HZ_TIMEOUT}, bw-timeouts: ${BW_TIMEOUT}" | tee -a "${GATE_FILE}"

    cat "${NODE_LIST}" >> "${NODE_PROCESS_SUMMARY}" 2>/dev/null || true
    echo "" >> "${NODE_PROCESS_SUMMARY}"
    echo "--- Top processes by CPU ---" >> "${NODE_PROCESS_SUMMARY}"
    ps aux --sort=-%cpu 2>/dev/null | head -15 >> "${NODE_PROCESS_SUMMARY}" || true
}

cleanup_audit() {
    if ${CLEANUP_EXECUTED}; then
        return 0
    fi

    echo "=== Safety Cleanup ===" | tee -a "${GATE_FILE}"

    if [[ -n "${LAUNCH_PID}" ]]; then
        kill "${LAUNCH_PID}" 2>/dev/null || true
        wait "${LAUNCH_PID}" 2>/dev/null || true
        LAUNCH_PID=""
        echo "  [SAFETY] Launch process stopped" | tee -a "${GATE_FILE}"
    fi

    for robot in tracer1 tracer2 tracer3; do
        if timeout 3s ros2 topic list 2>/dev/null | grep -q "/${robot}/cmd_vel"; then
            timeout 5s ros2 topic pub --times 5 "/${robot}/cmd_vel" geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null || true
        fi
    done
    echo "  [SAFETY] Zero cmd_vel published to tracer1/tracer2/tracer3" | tee -a "${GATE_FILE}"

    touch /tmp/p3c_emergency_stop.flag 2>/dev/null || true
    echo "  [SAFETY] Emergency stop flag asserted" | tee -a "${GATE_FILE}"

    for residual_pattern in goto_pose_driver cmd_watchdog cmd_scheduler mission_coordinator multi_tracer_return_home force_monitor_huatai1 qr_delta_publisher; do
        pkill -f "${residual_pattern}" 2>/dev/null || true
    done
    echo "  [SAFETY] Residual profile nodes killed" | tee -a "${GATE_FILE}"

    local t1_post t2_post t3_post
    t1_post=$(sanitize_int "$(get_topic_publisher_count /tracer1/cmd_vel)")
    t2_post=$(sanitize_int "$(get_topic_publisher_count /tracer2/cmd_vel)")
    t3_post=$(sanitize_int "$(get_topic_publisher_count /tracer3/cmd_vel)")
    echo "  [CONFIRM] /tracer1/cmd_vel pub: ${t1_post} /tracer2/cmd_vel pub: ${t2_post} /tracer3/cmd_vel pub: ${t3_post}" | tee -a "${GATE_FILE}"

    CLEANUP_EXECUTED=true
}

evaluate_resource_gate() {
    RESOURCE_PASS=false
    RESOURCE_WARN=false
    RESOURCE_FAIL=false

    if ! ${LAUNCH_STARTED}; then
        RESOURCE_FAIL=true
        FAILURE_REASON="${FAILURE_REASON:+${FAILURE_REASON};}launch_not_started"
        return
    fi

    if ! ${CLEANUP_EXECUTED}; then
        RESOURCE_FAIL=true
        FAILURE_REASON="${FAILURE_REASON:+${FAILURE_REASON};}cleanup_not_executed"
        return
    fi

    if [[ -n "${FAILURE_REASON}" ]]; then
        RESOURCE_FAIL=true
        return
    fi

    if ! ${ROS_RESPONSIVE}; then
        RESOURCE_FAIL=true
        return
    fi

    if [[ "${ROS_CLI_TIMEOUT_COUNT}" -gt 0 ]]; then
        RESOURCE_FAIL=true
        return
    fi

    local load
    load=$(cat /proc/loadavg 2>/dev/null | awk '{print $1}' || echo 0)
    local cpu_cores
    cpu_cores=$(nproc 2>/dev/null || echo 1)

    if [[ "${HIGH_BW_COUNT}" -gt 0 || "${MHZ_COUNT}" -gt 0 ]]; then
        RESOURCE_WARN=true
    fi

    if ${RESOURCE_WARN}; then
        RESOURCE_PASS=true
    else
        RESOURCE_PASS=true
    fi
}

write_reports() {
    local load
    load=$(cat /proc/loadavg 2>/dev/null | awk '{print $1}' || echo "N/A")
    local mem_avail
    mem_avail=$(free -h 2>/dev/null | awk '/^Mem:/{print $7}' || echo "N/A")
    local disk_avail
    disk_avail=$(df -h /home 2>/dev/null | awk 'NR==2{print $4}' || echo "N/A")

    cat > "${RESOURCE_SUMMARY}" <<RESEOF
metric,value
run_id,${RUN_ID}
profile,${PROFILE}
launch_started,${LAUNCH_STARTED}
profile_ready,${PROFILE_READY}
ros_responsive,${ROS_RESPONSIVE}
cleanup_executed,${CLEANUP_EXECUTED}
resource_gate,${RESOURCE_FAIL:-false}
camera_topic_count,${CAMERA_TOPIC_COUNT}
high_bw_topic_count,${HIGH_BW_COUNT}
high_hz_topic_count,${MHZ_COUNT}
hz_timeout_count,${HZ_TIMEOUT}
bw_timeout_count,${BW_TIMEOUT}
system_load,${load}
memory_available,${mem_avail}
disk_available,${disk_avail}
RESEOF

    local gate_label=""
    if ${RESOURCE_FAIL}; then
        gate_label="RESOURCE_FAIL"
    elif ${RESOURCE_WARN}; then
        gate_label="RESOURCE_WARN"
    elif ${RESOURCE_PASS}; then
        gate_label="RESOURCE_PASS"
    else
        gate_label="UNDETERMINED"
    fi

    echo "" >> "${GATE_FILE}"
    echo "--- Resource Gate Verdict ---" >> "${GATE_FILE}"
    echo "Gate: ${gate_label}" >> "${GATE_FILE}"
    echo "ROS responsive: ${ROS_RESPONSIVE}" >> "${GATE_FILE}"
    echo "Camera topics: ${CAMERA_TOPIC_COUNT}" >> "${GATE_FILE}"
    echo "High-bandwidth topics: ${HIGH_BW_COUNT}" >> "${GATE_FILE}"
    echo "High-frequency topics: ${MHZ_COUNT}" >> "${GATE_FILE}"
    echo "ROS CLI timeouts: ${ROS_CLI_TIMEOUT_COUNT}" >> "${GATE_FILE}"
    echo "System load: ${load}" >> "${GATE_FILE}"
    echo "Memory avail: ${mem_avail}" >> "${GATE_FILE}"
    echo "Cleanup executed: ${CLEANUP_EXECUTED}" >> "${GATE_FILE}"
    echo "Failure reason: ${FAILURE_REASON:-none}" >> "${GATE_FILE}"
}

write_recommendations() {
    cat > "${RECOMMENDATIONS}" <<RECEOF
# run_all Resource Audit Recommendations

Run ID: ${RUN_ID}
Date: $(date -u +%Y-%m-%dT%H:%M:%SZ)

## Gate Result

$(cat "${GATE_FILE}" | grep -E "^Gate:|^ROS responsive|^Camera topics|^High-bandwidth|^High-frequency|^ROS CLI timeouts|^System load|^Memory avail")

## Findings

- Camera/vision-related topic count: ${CAMERA_TOPIC_COUNT}
- High-bandwidth topics (>50 B/s): ${HIGH_BW_COUNT}
- High-frequency topics (>29 Hz): ${MHZ_COUNT}
- ROS CLI hz timeouts: ${HZ_TIMEOUT}
- ROS CLI bw timeouts: ${BW_TIMEOUT}
- ROS responsiveness after launch: ${ROS_RESPONSIVE}
- Cleanup success: ${CLEANUP_EXECUTED}

## Downgrade / Mitigation Recommendations

1. **降低 camera FPS**: 修改 camera launch/config 将 frame rate 从默认降至 15 Hz 或更低。
2. **降低 resolution**: 将 camera image resolution 降低至 VGA (640x480) 或更低。
3. **使用 compressed image**: 在 run_all 或 camera launch 中启用 compressed image transport (/tracer1/camera/image_raw/compressed)。
4. **observer 排除 image raw topic**: 审计/observer 脚本默认不录制 image_raw 话题。
5. **输出重定向到 log**: run_all 实机实验 stdout/stderr 输出到文件，不使用 screen/tmux console 直出。
6. **camera/QR 与控制分离**: 考虑 camera 和 QR processing 与 control pipeline 分进程或分机器部署。
7. **必要时用 system_bringup 作为主线**: 如果 camera-induced freeze 无法快速解决，先用 system_bringup 或 mission_bringup 作为论文主实机 profile，run_all 仅做最终短窗验证。

## Status

- D1-2-2 / D1-3 remain blocked until resource audit confirms a safe configuration.
- run_all cannot be declared the main real-machine profile without resource mitigation.
- system_bringup / mission_bringup remain viable short-term alternatives.
RECEOF
}

finalize_audit() {
    if ${CLEANUP_EXECUTED}; then
        return 0
    fi
    collect_system_snapshot
    cleanup_audit
    evaluate_resource_gate
    write_reports
    write_recommendations
}

runtime_exit_trap() {
    local exit_code=$?
    trap - EXIT
    if ! ${RUNNER_STARTED}; then
        return 0
    fi
    if ${CLEANUP_EXECUTED}; then
        return 0
    fi
    if [[ -z "${FAILURE_REASON}" ]]; then
        FAILURE_REASON="runner_exit_${exit_code}"
    fi
    finalize_audit
    return 0
}

trap runtime_exit_trap EXIT
RUNNER_STARTED=true

echo "[PROFILE] run_all: camera/full-process resource audit" | tee -a "${GATE_FILE}"
echo "[SAFETY] No image recording. No motion commands. Observer only." | tee -a "${GATE_FILE}"

rm -f /tmp/p3c_emergency_stop.flag 2>/dev/null || true
echo "[SAFETY] Pre-launch emergency flag cleared" | tee -a "${GATE_FILE}"

collect_system_snapshot

echo "=== Launching run_all ===" | tee -a "${GATE_FILE}"
echo "  [LAUNCH] ${LAUNCH_CMD}" | tee -a "${GATE_FILE}"
ros2 launch wing_alignment_system run_all.launch.py > "${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!
sleep 2

if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    LAUNCH_STARTED=true
    echo "  [LAUNCH] PID ${LAUNCH_PID} is alive" | tee -a "${GATE_FILE}"
else
    FAILURE_REASON="launch_process_exited_early"
    echo "  [FAIL] Launch process exited early" | tee -a "${GATE_FILE}"
    finalize_audit
    echo "Run directory: ${RUN_DIR}"
    exit 0
fi

if ! wait_for_profile_ready; then
    echo "  [FAIL] Ready barrier failed: ${FAILURE_REASON}" | tee -a "${GATE_FILE}"
    finalize_audit
    echo "Run directory: ${RUN_DIR}"
    exit 0
fi

echo "=== Audit Window: ${DURATION_SEC}s ===" | tee -a "${GATE_FILE}"
collect_ros_topology
audit_camera_topics
collect_system_snapshot

if [[ "${DURATION_SEC}" -gt 0 ]]; then
    sleep "${DURATION_SEC}"
fi

collect_system_snapshot
finalize_audit

echo "Run directory: ${RUN_DIR}"
echo "Gate file: ${GATE_FILE}"
echo "Resource summary: ${RESOURCE_SUMMARY}"
echo "Topic bandwidth: ${TOPIC_BW_SUMMARY}"
echo "Node/process: ${NODE_PROCESS_SUMMARY}"
echo "Recommendations: ${RECOMMENDATIONS}"
exit 0
