#!/usr/bin/env bash
set -euo pipefail
# FR-TAC-P3-C Task C: Single-robot controlled closed-loop runner.
# Runs ON the remote Linux machine (192.168.5.207).
# MUST pass --allow-real-motion to publish cmd_vel to tracer1.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CTRL_PY="${SCRIPT_DIR}/run_fr_tac_p3_single_robot_controlled.py"

# P3-C hard speed limits
MAX_LINEAR=0.05
MAX_ANGULAR=0.10
V_CMD=0.03
W_CMD=0.06

allow_real_motion=false
artifact_root="${HOME}/.ros/fr_tac_p3c_controlled_runs"
run_id="p3c_controlled"
cases="C1,C2,C3,C4,C5"

usage() {
    echo "Usage: $0 [--allow-real-motion] [--artifact-root DIR] [--run-id ID] [--cases C1,C2,...]"
    echo ""
    echo "  --allow-real-motion  REQUIRED to publish real cmd_vel to tracer1."
    echo "                       Without this flag, NO real motion is performed."
    echo "  --artifact-root DIR  Root for output artifacts (default: ~/.ros/fr_tac_p3c_controlled_runs)"
    echo "  --run-id ID          Run identifier (default: p3c_controlled)"
    echo "  --cases CASES        Comma-separated cases: C1,C2,C3,C4,C5 (default: all)"
    echo ""
    echo "Speed limits (hard): linear <= ${MAX_LINEAR} m/s, angular <= ${MAX_ANGULAR} rad/s"
    exit 2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --allow-real-motion)
            allow_real_motion=true
            shift
            ;;
        --artifact-root)
            artifact_root="${2}"; shift 2 ;;
        --run-id)
            run_id="${2}"; shift 2 ;;
        --cases)
            cases="${2}"; shift 2 ;;
        -h|--help)
            usage ;;
        *)
            echo "unknown arg: $1" >&2; usage ;;
    esac
done

# ---- Pre-flight checks ----
echo "============================================================"
echo "  FR-TAC-P3-C Controlled Runner"
echo "  Run ID:       ${run_id}"
echo "  Robot:        tracer1"
echo "  V_MAX:        ${MAX_LINEAR} m/s"
echo "  W_MAX:        ${MAX_ANGULAR} rad/s"
echo "  V_CMD:        ${V_CMD} m/s"
echo "  W_CMD:        ${W_CMD} rad/s"
echo "  Real Motion:  ${allow_real_motion}"
echo "  Cases:        ${cases}"
echo "============================================================"
echo ""

# Verify V_CMD and W_CMD are within limits
if python3 -c "exit(0 if ${V_CMD} <= ${MAX_LINEAR} and ${W_CMD} <= ${MAX_ANGULAR} else 1)"; then
    echo "[OK] Command speeds within limits."
else
    echo "[FAIL] V_CMD=${V_CMD} > MAX_LINEAR=${MAX_LINEAR} or W_CMD=${W_CMD} > MAX_ANGULAR=${MAX_ANGULAR}"
    exit 1
fi

# Real motion gate
if ! ${allow_real_motion}; then
    echo ""
    echo "================================================================"
    echo "  --allow-real-motion NOT set."
    echo "  Running in-memory WatchdogPolicy validation ONLY."
    echo "  No cmd_vel will be published to the robot."
    echo "  To publish real cmd_vel, re-run with --allow-real-motion."
    echo "================================================================"
    echo ""
fi

# ---- Source workspace and run ----
WS_SETUP="${REPO_ROOT}/install/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
    set +u
    source "${WS_SETUP}"
    set -u
fi

run_dir="${artifact_root%/}/${run_id}"
mkdir -p "${run_dir}"

cd "${REPO_ROOT}"
python3 "${CTRL_PY}" \
    --artifact-root "${artifact_root}" \
    --run-id "${run_id}" \
    --cases "${cases}" \
    $(${allow_real_motion} && echo "--allow-real-motion" || true)

rc=$?

echo ""
if [[ ${rc} -eq 0 ]]; then
    echo "=== P3-C Controlled: PASS ==="
else
    echo "=== P3-C Controlled: FAIL (rc=${rc}) ==="
fi

if [[ -f "${run_dir}/p3c_controlled_gate.txt" ]]; then
    cat "${run_dir}/p3c_controlled_gate.txt"
fi
exit ${rc}
