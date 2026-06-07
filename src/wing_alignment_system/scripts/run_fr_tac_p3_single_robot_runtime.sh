#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_SETUP="${REPO_ROOT}/install/setup.bash"

allow_real_motion=false
background=false
stop_runtime=false
run_id="p3c_runtime"
artifact_root="${HOME}/.ros/fr_tac_p3c_runtime"
pid_file="${artifact_root}/runtime.pid"
log_file="${artifact_root}/runtime.log"
emergency_stop_file="/tmp/p3c_emergency_stop.flag"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --allow-real-motion) allow_real_motion=true; shift ;;
        --background) background=true; shift ;;
        --stop) stop_runtime=true; shift ;;
        --run-id) run_id="${2}"; shift 2 ;;
        --artifact-root) artifact_root="${2}"; shift 2 ;;
        --emergency-stop-file) emergency_stop_file="${2}"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "${artifact_root}"
pid_file="${artifact_root}/runtime.pid"
log_file="${artifact_root}/runtime.log"

if [[ -f "${WS_SETUP}" ]]; then
    set +u
    source "${WS_SETUP}"
    set -u
fi

if ${stop_runtime}; then
    if [[ -f "${pid_file}" ]]; then
        kill "$(cat "${pid_file}")" 2>/dev/null || true
        rm -f "${pid_file}"
    fi
    exit 0
fi

safe_idle_no_publish=true
if ${allow_real_motion}; then
    safe_idle_no_publish=false
fi

cmd=(
    ros2 launch wing_alignment_system fr_tac_p3_single_robot_runtime.launch.py
    run_id:="${run_id}"
    log_dir:="${artifact_root}"
    safe_idle_no_publish:="${safe_idle_no_publish}"
    emergency_stop_file:="${emergency_stop_file}"
)

if ${background}; then
    nohup "${cmd[@]}" > "${log_file}" 2>&1 &
    echo $! > "${pid_file}"
    echo "${pid_file}"
    exit 0
fi

exec "${cmd[@]}"
