#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_SETUP="${REPO_ROOT}/install/setup.bash"

baseline="B3"
validation_mode="runtime_replay"
scenario_id="real-main"
run_id="fr_tac_p2_shadow"
duration_sec="30"
artifact_root="${HOME}/.ros/fr_tac_p2_shadow_runs"
mission_runtime_events_path=""
node_output="log"
replay_speed="5.0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --baseline)
            baseline="${2}"
            shift 2
            ;;
        --validation-mode)
            validation_mode="${2}"
            shift 2
            ;;
        --scenario-id)
            scenario_id="${2}"
            shift 2
            ;;
        --run-id)
            run_id="${2}"
            shift 2
            ;;
        --duration-sec)
            duration_sec="${2}"
            shift 2
            ;;
        --artifact-root)
            artifact_root="${2}"
            shift 2
            ;;
        --mission-runtime-events)
            mission_runtime_events_path="${2}"
            shift 2
            ;;
        --node-output)
            node_output="${2}"
            shift 2
            ;;
        --replay-speed)
            replay_speed="${2}"
            shift 2
            ;;
        *)
            echo "unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

case "${baseline}" in
    B0)
        method_id="periodic_full"
        enable_execution_mode="false"
        ;;
    B1)
        method_id="event_triggered"
        enable_execution_mode="false"
        ;;
    B2)
        method_id="freshness_aware_tx"
        enable_execution_mode="false"
        ;;
    B3)
        method_id="freshness_aware_tx"
        enable_execution_mode="true"
        ;;
    *)
        echo "baseline must be one of B0 B1 B2 B3" >&2
        exit 2
        ;;
esac

run_root="${artifact_root%/}/${run_id}"
mission_log_root="${run_root}/mission"
mission_run_dir="${mission_log_root}/${run_id}"
cmd_safety_log_root="${run_root}/cmd_safety"
wrapper_metadata_log_root="${run_root}/wrapper_metadata"

mkdir -p "${mission_run_dir}" "${cmd_safety_log_root}" "${wrapper_metadata_log_root}"

if [[ "${validation_mode}" == "runtime_replay" ]]; then
    if [[ -z "${mission_runtime_events_path}" ]]; then
        echo "--mission-runtime-events is required when --validation-mode runtime_replay" >&2
        exit 2
    fi
    cp "${mission_runtime_events_path}" "${mission_run_dir}/mission_runtime_events.csv"
fi

phase_source_mode="mission_runtime_tail"
if [[ "${validation_mode}" == "runtime_replay" ]]; then
    phase_source_mode="mission_runtime_replay"
fi

if [[ -f "${WORKSPACE_SETUP}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${WORKSPACE_SETUP}"
    set -u
fi

set +e
timeout "${duration_sec}s" ros2 launch freshness_real_robot_validation mission_aware_shadow_validation.launch.py \
    validation_mode:="${validation_mode}" \
    scenario_id:="${scenario_id}" \
    method_id:="${method_id}" \
    enable_execution_mode:="${enable_execution_mode}" \
    measurement_run_id:="${run_id}" \
    mission_log_root:="${mission_log_root}" \
    mission_runtime_events_path:="${mission_run_dir}/mission_runtime_events.csv" \
    phase_source_mode:="${phase_source_mode}" \
    replay_speed:="${replay_speed}" \
    cmd_safety_log_root:="${cmd_safety_log_root}" \
    wrapper_metadata_log_root:="${wrapper_metadata_log_root}" \
    node_output:="${node_output}"
launch_status=$?
set -e

if [[ ${launch_status} -ne 0 && ${launch_status} -ne 124 ]]; then
    exit "${launch_status}"
fi

python3 "${REPO_ROOT}/src/wing_alignment_system/scripts/summarize_run_kpi.py" "${run_root}"
