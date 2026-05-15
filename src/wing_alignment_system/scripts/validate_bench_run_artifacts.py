#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validate P1-real bench/replay artifacts without inventing results."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

REQUIRED_MANIFEST_FIELDS = [
    "run_id",
    "profile",
    "baseline_mode",
    "evidence_class",
    "csv_paths",
]

EVIDENCE_CLASSES = {
    "synthetic_dry_run",
    "replay",
    "ros2_bench",
    "hardware_preliminary",
}


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _csv_header(path: Path) -> list[str]:
    with path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        return list(reader.fieldnames or [])


def _role_check(paths: list[str]) -> dict:
    files = []
    passed_any = False
    for value in paths:
        path = Path(value).expanduser()
        exists = path.exists()
        nonempty = exists and path.stat().st_size > 0
        header = _csv_header(path) if nonempty else []
        passed = bool(nonempty and header)
        passed_any = passed_any or passed
        files.append(
            {
                "path": str(path),
                "exists": exists,
                "nonempty": nonempty,
                "fields": header,
                "passed": passed,
            }
        )
    return {"passed": passed_any, "files": files}


def _first_existing(paths: list[str]) -> str:
    for value in paths:
        path = Path(value).expanduser()
        if path.exists() and path.stat().st_size > 0:
            return str(path)
    return ""


def _raw_delta_check(watchdog_check: dict) -> dict:
    required = {"delta_net_proxy_ms", "delta_exec_proxy_ms", "delta_eff_proxy_ms"}
    for item in watchdog_check.get("files", []):
        fields = set(item.get("fields", []))
        if required.issubset(fields):
            return {"passed": True, "fields": sorted(required), "path": item["path"]}
    return {"passed": False, "fields": [], "path": ""}


def _analyzer_check(run_dir: Path) -> dict:
    rows_path = run_dir / "effective_staleness_risk_rows.csv"
    summary_path = run_dir / "effective_freshness_summary.json"
    fields = []
    if rows_path.exists() and rows_path.stat().st_size > 0:
        fields = _csv_header(rows_path)
    summary = _read_json(summary_path) if summary_path.exists() else {}
    required = {"S_eff", "F_eff"}
    grouped = all(key in summary for key in ("by_baseline_mode", "by_profile", "by_task_phase"))
    return {
        "passed": bool(required.issubset(set(fields)) and grouped),
        "fields": fields,
        "summary_json": str(summary_path),
        "staleness_rows_csv": str(rows_path),
        "has_grouped_stats": grouped,
    }


def _manifest_check(manifest: dict) -> dict:
    missing = [key for key in REQUIRED_MANIFEST_FIELDS if key not in manifest]
    valid_evidence = manifest.get("evidence_class") in EVIDENCE_CLASSES
    return {
        "passed": not missing and valid_evidence,
        "missing": missing,
        "valid_evidence_class": valid_evidence,
    }


def _markdown(report: dict) -> str:
    lines = [
        "# Bench run artifact validation report",
        "",
        "This validation checks artifact presence and analyzer compatibility. It does not prove ROS2 bench execution quality, 5G performance, actuator freshness, physical residual ground truth, or safety.",
        "",
        f"- passed: {report['passed']}",
        f"- run_id: {report['manifest'].get('run_id', '')}",
        f"- profile: {report['manifest'].get('profile', '')}",
        f"- baseline_mode: {report['manifest'].get('baseline_mode', '')}",
        f"- evidence_class: {report['manifest'].get('evidence_class', '')}",
        f"- artifact_scope: {report.get('artifact_scope', '')}",
        f"- mission_missing_allowed: {report.get('mission_missing_allowed', False)}",
        f"- complete_mission_bench: {report.get('complete_mission_bench', False)}",
        "",
        "## Checks",
        "",
    ]
    for name, check in report["checks"].items():
        lines.append(f"- {name}: {check.get('passed')}")
    lines.append("")
    return "\n".join(lines)


def _mission_check_with_policy(mission_check: dict, allow_missing_mission: bool) -> dict:
    out = dict(mission_check)
    raw_passed = bool(mission_check.get("passed", False))
    missing = not raw_passed
    out["raw_passed"] = raw_passed
    out["mission_missing_allowed"] = bool(allow_missing_mission and missing)
    out["complete_mission_bench"] = raw_passed
    if allow_missing_mission and missing:
        out["passed"] = True
        out["scope"] = "scheduler_watchdog_only"
        out["note"] = (
            "mission CSV missing was explicitly allowed; this is not a complete mission bench artifact."
        )
    else:
        out["scope"] = "complete_mission_required"
        out["note"] = ""
    return out


def validate_manifest(manifest_path: str, out_dir: str = "", allow_missing_mission: bool = False) -> dict:
    manifest_file = Path(manifest_path).expanduser()
    manifest = _read_json(manifest_file)
    output_dir = Path(out_dir).expanduser() if out_dir else manifest_file.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_paths = manifest.get("csv_paths", {})
    scheduler_check = _role_check(csv_paths.get("scheduler", []))
    watchdog_check = _role_check(csv_paths.get("watchdog", []))
    mission_check = _role_check(csv_paths.get("mission", []))
    mission_policy_check = _mission_check_with_policy(mission_check, allow_missing_mission)

    run_dir = Path(manifest.get("analysis", {}).get("run_dir", manifest_file.parent)).expanduser()
    mission_csv = _first_existing(csv_paths.get("mission", []))
    if scheduler_check["passed"] and watchdog_check["passed"]:
        from analyze_effective_freshness import analyze

        analyze(run_dir=str(run_dir), mission_csv=mission_csv, out_dir=str(run_dir))

    checks = {
        "manifest": _manifest_check(manifest),
        "scheduler_csv": scheduler_check,
        "watchdog_csv": watchdog_check,
        "mission_csv": mission_policy_check,
        "delta_proxy_fields": _raw_delta_check(watchdog_check),
        "analyzer_outputs": _analyzer_check(run_dir),
    }
    passed = all(check.get("passed", False) for check in checks.values())
    complete_mission_bench = bool(mission_policy_check.get("complete_mission_bench", False))
    report = {
        "passed": passed,
        "manifest_path": str(manifest_file),
        "manifest": {
            "run_id": manifest.get("run_id", ""),
            "profile": manifest.get("profile", ""),
            "baseline_mode": manifest.get("baseline_mode", ""),
            "evidence_class": manifest.get("evidence_class", ""),
            "baseline_candidate_status": manifest.get("baseline_candidate_status", ""),
        },
        "checks": checks,
        "mission_missing_allowed": bool(allow_missing_mission),
        "complete_mission_bench": complete_mission_bench,
        "artifact_scope": "complete_mission_bench" if complete_mission_bench else "scheduler_watchdog_only",
        "boundary": "Proxy validation only; no 5G, safety, actuator freshness, or physical residual claim.",
    }
    (output_dir / "validation_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output_dir / "validation_report.md").write_text(_markdown(report), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate P1-real bench run artifacts.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--out-dir", default="")
    parser.add_argument(
        "--allow-missing-mission",
        action="store_true",
        help=(
            "Allow scheduler/watchdog-only artifact validation when mission CSV is absent. "
            "The report will mark mission_missing_allowed=true and complete_mission_bench=false."
        ),
    )
    args = parser.parse_args()
    report = validate_manifest(
        args.manifest,
        out_dir=args.out_dir,
        allow_missing_mission=args.allow_missing_mission,
    )
    print(json.dumps({
        "passed": report["passed"],
        "run_id": report["manifest"]["run_id"],
        "mission_missing_allowed": report["mission_missing_allowed"],
        "complete_mission_bench": report["complete_mission_bench"],
        "artifact_scope": report["artifact_scope"],
    }, indent=2))


if __name__ == "__main__":
    main()
