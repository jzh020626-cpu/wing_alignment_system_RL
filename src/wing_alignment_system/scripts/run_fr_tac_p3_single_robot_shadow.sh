#!/usr/bin/env bash
set -euo pipefail
# FR-TAC-P3-C Task B: Single-robot low-speed shadow runner.
# Pure Python WatchdogPolicy validation — no ROS, no real motion.
# Runs on the remote Linux machine.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SHADOW_PY="${SCRIPT_DIR}/run_fr_tac_p3_single_robot_shadow.py"

artifact_root="${HOME}/.ros/fr_tac_p3c_shadow_runs"
run_id="p3c_shadow"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --artifact-root) artifact_root="${2}"; shift 2 ;;
        --run-id)        run_id="${2}"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

run_dir="${artifact_root%/}/${run_id}"
mkdir -p "${run_dir}"

echo "============================================================"
echo "  FR-TAC-P3-C Shadow Runner"
echo "  Run ID:  ${run_id}"
echo "  Out Dir: ${run_dir}"
echo "  Mode:    pure-Python (no ROS, no real motion)"
echo "============================================================"
echo ""

cd "${REPO_ROOT}"
python3 "${SHADOW_PY}" "${run_dir}"
rc=$?

echo ""
if [[ ${rc} -eq 0 ]]; then
    echo "P3-C Shadow: PASS"
else
    echo "P3-C Shadow: FAIL (rc=${rc})"
fi
if [[ -f "${run_dir}/p3c_shadow_gate.txt" ]]; then
    cat "${run_dir}/p3c_shadow_gate.txt"
fi
exit ${rc}
