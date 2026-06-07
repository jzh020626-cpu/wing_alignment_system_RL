#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
P2_RUNNER="${SCRIPT_DIR}/run_fr_tac_p2_shadow_validation.sh"

artifact_root="${HOME}/.ros/fr_tac_p3_mission_safe_idle_runs"
duration_sec="60"
baselines=("B0" "B1" "B2" "B3")
mission_csv="/home/ls/hjz/analysis/fr_tac_p2/mission_runtime_events_replay.csv"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --artifact-root) artifact_root="${2}"; shift 2 ;;
        --duration-sec)  duration_sec="${2}"; shift 2 ;;
        --mission-csv)   mission_csv="${2}"; shift 2 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

declare -A baseline_results
all_passed=true

for baseline in "${baselines[@]}"; do
    run_id="p3b_${baseline}_safe_idle"
    echo ""
    echo "======================================================"
    echo "  P3-B: ${baseline} (runtime_replay / safe-idle equiv)"
    echo "  run_id: ${run_id}"
    echo "======================================================"

    set +e
    bash "${P2_RUNNER}"         --baseline "${baseline}"         --validation-mode runtime_replay         --scenario-id real-main         --run-id "${run_id}"         --duration-sec "${duration_sec}"         --artifact-root "${artifact_root}"         --mission-runtime-events "${mission_csv}"
    rc=$?
    set -e

    if [[ ${rc} -eq 0 ]]; then
        echo "  ${baseline}: PASS (rc=0)"
        baseline_results["${baseline}"]="PASS"
    else
        echo "  ${baseline}: FAIL (rc=${rc})"
        baseline_results["${baseline}"]="FAIL (rc=${rc})"
        all_passed=false
    fi
done

# Write gate file
gate_path="${artifact_root}/p3b_gate.txt"
mkdir -p "$(dirname "${gate_path}")"
{
    echo "FR-TAC-P3-B Gate Report"
    echo "========================"
    if ${all_passed}; then
        echo "Gate: PASS"
    else
        echo "Gate: FAIL"
    fi
    echo ""
    echo "Baseline Results:"
    for baseline in "${baselines[@]}"; do
        echo "  ${baseline}: ${baseline_results[${baseline}]}"
    done
    echo ""
    if ${all_passed}; then
        echo "All baselines completed successfully."
    else
        echo "One or more baselines failed."
    fi
    echo ""
    echo "Artifact root: ${artifact_root}"
} > "${gate_path}"

cat "${gate_path}"
exit 0
