import csv
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "src/wing_alignment_system/scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import create_bench_run_manifest as manifest_script
import validate_bench_run_artifacts as validator


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _hardware_paths(root: Path) -> dict:
    return {
        "clock_sync_status.csv": root / "capture" / "clock_sync_status.csv",
        "network_ping_samples.csv": root / "capture" / "network_ping_samples.csv",
        "interface_counters.csv": root / "capture" / "interface_counters.csv",
        "recorder_health.csv": root / "capture" / "recorder_health.csv",
        "recorder_topic_status.csv": root / "capture" / "recorder_topic_status.csv",
    }


def _hardware_manifest(root: Path, run_id: str = "hw_run") -> Path:
    paths = _hardware_paths(root)
    manifest = {
        "run_id": run_id,
        "profile": "nominal",
        "baseline_mode": "current_safe_default",
        "evidence_class": "hardware_preliminary",
        "manifest_sha256": "fixture",
        "csv_paths": {
            "scheduler": [],
            "watchdog": [],
            "mission": [],
        },
        "capture_artifacts": [
            {"name": name, "path": str(path)}
            for name, path in paths.items()
        ],
        "derived_artifacts": [
            {"name": "command_residence_events.csv", "path": str(root / "derived" / "command_residence_events.csv")},
            {"name": "derivation_report.json", "path": str(root / "derived" / "derivation_report.json")},
        ],
        "gate_defaults": {
            "default_gate": "H0",
            "supported_gates": ["H0", "H1", "H2", "H3", "H4", "H5"],
        },
        "prereq_gate_results": {
            "H0": {"status": "unknown", "validation_report": "", "run_id": "", "manifest_sha256": ""},
            "H1": {"status": "unknown", "validation_report": "", "run_id": "", "manifest_sha256": ""},
            "H2": {"status": "unknown", "validation_report": "", "run_id": "", "manifest_sha256": ""},
            "H3": {"status": "unknown", "validation_report": "", "run_id": "", "manifest_sha256": ""},
        },
        "mocap_wing_topic": "/Rigid8/pose",
        "mocap_robot_topics": ["/Rigid17/pose", "/Rigid14/pose", "/Rigid15/pose"],
        "mocap_message_type": "geometry_msgs/msg/PoseStamped",
        "mocap_frame_id_policy": "configured_until_runtime_observed",
        "clock_sync_policy": {
            "true_one_way_delay_requires_sync_verified": True,
            "true_one_way_delay_requires_source_stamp_valid": True,
            "true_one_way_delay_requires_stamp_origin": "upstream_header",
            "fallback_stamp_origin_disallows_one_way_delay": True,
        },
        "network_monitor_config": {
            "peers": [{"host": "192.168.1.1", "peer_role": "gateway"}],
            "gateway_only_context": True,
            "requires_robot_or_control_peer_for_H4": True,
            "waiver": {"enabled": False, "reason": "", "approved_by": ""},
        },
        "operator_safety_precheck": {
            "operator_precheck_ack": False,
            "safety_observer_present": False,
            "emergency_stop_tested_pre_run": False,
            "workspace_clear": False,
            "degradation_scope_confirmed": False,
            "safety_channels_excluded_from_degradation": False,
        },
        "hardware_preliminary_boundary": {
            "h0_scope": "startup_only",
            "sensor_availability_proven": False,
            "task_execution_proven": False,
            "throughput_is_context_only": True,
            "recorder_callback_timing_is_proxy_only": True,
        },
    }
    manifest["manifest_sha256"] = manifest_script._canonical_manifest_sha256(manifest)
    path = root / "run_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _write_h0_files(root: Path, run_id: str = "hw_run", control_publishers_count: int = 0) -> None:
    paths = _hardware_paths(root)
    _write_csv(
        paths["clock_sync_status.csv"],
        [
            "run_id",
            "host_id",
            "host_role",
            "monitor_source",
            "sync_available",
            "sync_verified",
            "offset_ms",
            "reference_clock",
            "time_base",
            "one_way_delay_allowed",
            "t_wall",
        ],
        [
            {
                "run_id": run_id,
                "host_id": "host-a",
                "host_role": "control_host",
                "monitor_source": "chrony",
                "sync_available": "true",
                "sync_verified": "false",
                "offset_ms": "",
                "reference_clock": "",
                "time_base": "system_wall",
                "one_way_delay_allowed": "false",
                "t_wall": "1.0",
            }
        ],
    )
    _write_csv(
        paths["network_ping_samples.csv"],
        [
            "run_id",
            "host_id",
            "peer_host",
            "peer_role",
            "iface",
            "rtt_ms",
            "jitter_ms",
            "packet_loss_percent",
            "t_wall",
        ],
        [
            {
                "run_id": run_id,
                "host_id": "host-a",
                "peer_host": "192.168.1.1",
                "peer_role": "gateway",
                "iface": "wlan0",
                "rtt_ms": "2.1",
                "jitter_ms": "0.2",
                "packet_loss_percent": "0.0",
                "t_wall": "1.0",
            }
        ],
    )
    _write_csv(
        paths["interface_counters.csv"],
        [
            "run_id",
            "host_id",
            "iface",
            "rx_bytes",
            "tx_bytes",
            "rx_dropped",
            "tx_dropped",
            "rx_errors",
            "tx_errors",
            "t_wall",
        ],
        [
            {
                "run_id": run_id,
                "host_id": "host-a",
                "iface": "wlan0",
                "rx_bytes": "10",
                "tx_bytes": "20",
                "rx_dropped": "0",
                "tx_dropped": "0",
                "rx_errors": "0",
                "tx_errors": "0",
                "t_wall": "1.0",
            }
        ],
    )
    _write_csv(
        paths["recorder_health.csv"],
        [
            "run_id",
            "logger_name",
            "stream_name",
            "rows_written",
            "rows_dropped",
            "queue_depth",
            "max_queue_depth",
            "last_write_error",
            "t_wall",
            "t_ros",
            "user_defined_publishers_count",
            "control_publishers_count",
            "service_clients_count",
            "user_defined_services_count",
            "ros_infrastructure_endpoints",
            "configured_subscriptions_count",
        ],
        [
            {
                "run_id": run_id,
                "logger_name": "recorder",
                "stream_name": "startup",
                "rows_written": "1",
                "rows_dropped": "0",
                "queue_depth": "0",
                "max_queue_depth": "0",
                "last_write_error": "",
                "t_wall": "1.0",
                "t_ros": "1.0",
                "user_defined_publishers_count": "0",
                "control_publishers_count": str(control_publishers_count),
                "service_clients_count": "0",
                "user_defined_services_count": "0",
                "ros_infrastructure_endpoints": "[]",
                "configured_subscriptions_count": "0",
            }
        ],
    )
    _write_csv(
        paths["recorder_topic_status.csv"],
        ["run_id", "stream_name", "topic", "status", "t_wall"],
        [],
    )


def _legacy_manifest(root: Path, run_id: str = "bench_run") -> Path:
    run_dir = root / "cmd_safety_logs" / run_id
    mission_dir = root / "mission_bench_logs" / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    mission_dir.mkdir(parents=True, exist_ok=True)
    scheduler = run_dir / "events.csv"
    watchdog = run_dir / "ts_tracer1.csv"
    mission = mission_dir / "mission_runtime_events.csv"
    _write_csv(
        scheduler,
        [
            "run_id",
            "robot_id",
            "task_phase",
            "command_id",
            "command_type",
            "t_tx",
            "precision_mode",
            "scheduler_decision",
            "communication_profile_mode",
            "baseline_mode",
            "baseline_execution_environment",
        ],
        [
            {
                "run_id": run_id,
                "robot_id": "tracer1",
                "task_phase": "approach",
                "command_id": "1",
                "command_type": "twist",
                "t_tx": "1.0",
                "precision_mode": "0",
                "scheduler_decision": "transmit",
                "communication_profile_mode": "nominal",
                "baseline_mode": "current_safe_default",
                "baseline_execution_environment": "bench",
            }
        ],
    )
    _write_csv(
        watchdog,
        [
            "run_id",
            "robot_id",
            "command_id",
            "command_type",
            "t_source",
            "t_rx",
            "t_watchdog",
            "t",
            "age_ms",
            "delta_net_proxy_ms",
            "delta_exec_proxy_ms",
            "delta_eff_proxy_ms",
            "v",
            "w",
            "state",
            "watchdog_action",
            "stale_reason",
        ],
        [
            {
                "run_id": run_id,
                "robot_id": "tracer1",
                "command_id": "1",
                "command_type": "twist",
                "t_source": "1.0",
                "t_rx": "1.05",
                "t_watchdog": "1.07",
                "t": "1.07",
                "age_ms": "70.0",
                "delta_net_proxy_ms": "50.0",
                "delta_exec_proxy_ms": "20.0",
                "delta_eff_proxy_ms": "70.0",
                "v": "0.1",
                "w": "0.0",
                "state": "FRESH",
                "watchdog_action": "ALLOW",
                "stale_reason": "",
            }
        ],
    )
    _write_csv(
        mission,
        [
            "run_id",
            "timestamp",
            "mission_state",
            "task_phase",
            "precision_mode",
            "robot_id",
            "team_scope",
            "Delta_eff_proxy_ms",
            "S_eff",
            "F_eff",
            "base_authority_weight",
            "slide_authority_weight",
            "authority_policy_mode",
            "freeze_state",
            "watchdog_or_safe_state",
            "docking_residual_proxy",
            "slide_residual_proxy",
            "support_residual_proxy",
            "safe_abort_reason",
            "event_type",
            "event_note",
        ],
        [
            {
                "run_id": run_id,
                "timestamp": "1.1",
                "mission_state": "active",
                "task_phase": "approach",
                "precision_mode": "0",
                "robot_id": "tracer1",
                "team_scope": "team",
                "Delta_eff_proxy_ms": "70.0",
                "S_eff": "0.3",
                "F_eff": "0.7",
                "base_authority_weight": "1.0",
                "slide_authority_weight": "0.0",
                "authority_policy_mode": "precision_mode_proxy",
                "freeze_state": "off",
                "watchdog_or_safe_state": "FRESH",
                "docking_residual_proxy": "0.0",
                "slide_residual_proxy": "0.0",
                "support_residual_proxy": "0.0",
                "safe_abort_reason": "",
                "event_type": "SNAPSHOT",
                "event_note": "",
            }
        ],
    )
    manifest = {
        "run_id": run_id,
        "profile": "nominal",
        "baseline_mode": "current_safe_default",
        "evidence_class": "ros2_bench",
        "analysis": {"run_dir": str(run_dir), "analyzer": "scripts/analyze_effective_freshness.py"},
        "csv_paths": {
            "scheduler": [str(scheduler)],
            "watchdog": [str(watchdog)],
            "mission": [str(mission)],
        },
    }
    path = root / "legacy_manifest.json"
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def test_create_manifest_hardware_preliminary_includes_patch1_schema(tmp_path):
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path),
        profile="nominal",
        evidence_class="hardware_preliminary",
        ros2_available=False,
        tc_available=False,
    )

    assert manifest["evidence_class"] == "hardware_preliminary"
    assert "manifest_sha256" in manifest
    assert "capture_artifacts" in manifest
    assert "derived_artifacts" in manifest
    assert "gate_defaults" in manifest
    assert "prereq_gate_results" in manifest
    assert "clock_sync_policy" in manifest
    assert "network_monitor_config" in manifest
    assert "operator_safety_precheck" in manifest
    assert manifest["prereq_gate_results"]["H3"]["status"] == "unknown"


def test_create_manifest_hardware_preliminary_manifest_sha256_recomputes_cleanly(tmp_path):
    manifest = manifest_script.create_manifest(
        out_dir=str(tmp_path),
        profile="nominal",
        evidence_class="hardware_preliminary",
        ros2_available=False,
        tc_available=False,
    )
    manifest_path = tmp_path / "run_manifest.json"
    manifest_on_disk = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert manifest["manifest_sha256"] == manifest_on_disk["manifest_sha256"]
    assert manifest_script._canonical_manifest_sha256(manifest_on_disk) == manifest_on_disk["manifest_sha256"]


def test_h0_passes_with_monitor_and_health_files_only(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "passed_with_warnings"
    assert report["failures"] == []
    assert report["artifact_summary"]["clock_sync_status.csv"]["row_count"] == 1


def test_h0_fails_when_clock_sync_status_csv_is_missing(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    _hardware_paths(tmp_path)["clock_sync_status.csv"].unlink()

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "failed"
    assert any("clock_sync_status.csv" in failure for failure in report["failures"])


def test_h0_fails_when_recorder_health_reports_control_publisher(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path, control_publishers_count=1)

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "failed"
    assert any("control_publishers_count" in failure for failure in report["failures"])


def test_h0_fails_on_run_id_mismatch(tmp_path):
    manifest_path = _hardware_manifest(tmp_path, run_id="expected_run")
    _write_h0_files(tmp_path, run_id="wrong_run")

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "failed"
    assert any("inconsistent run_id" in failure for failure in report["failures"])


def test_h0_fails_on_non_monotonic_timestamp(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    clock_path = _hardware_paths(tmp_path)["clock_sync_status.csv"]
    _write_csv(
        clock_path,
        [
            "run_id",
            "host_id",
            "host_role",
            "monitor_source",
            "sync_available",
            "sync_verified",
            "offset_ms",
            "reference_clock",
            "time_base",
            "one_way_delay_allowed",
            "t_wall",
        ],
        [
            {
                "run_id": "hw_run",
                "host_id": "host-a",
                "host_role": "control_host",
                "monitor_source": "chrony",
                "sync_available": "true",
                "sync_verified": "false",
                "offset_ms": "",
                "reference_clock": "",
                "time_base": "system_wall",
                "one_way_delay_allowed": "false",
                "t_wall": "2.0",
            },
            {
                "run_id": "hw_run",
                "host_id": "host-a",
                "host_role": "control_host",
                "monitor_source": "chrony",
                "sync_available": "true",
                "sync_verified": "false",
                "offset_ms": "",
                "reference_clock": "",
                "time_base": "system_wall",
                "one_way_delay_allowed": "false",
                "t_wall": "1.0",
            },
        ],
    )

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "failed"
    assert any("non-monotonic" in failure for failure in report["failures"])


def test_h0_fails_when_recorder_endpoint_policy_fields_are_missing(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    recorder_health = _hardware_paths(tmp_path)["recorder_health.csv"]
    _write_csv(
        recorder_health,
        [
            "run_id",
            "logger_name",
            "stream_name",
            "rows_written",
            "rows_dropped",
            "queue_depth",
            "max_queue_depth",
            "last_write_error",
            "t_wall",
            "t_ros",
            "configured_subscriptions_count",
        ],
        [
            {
                "run_id": "hw_run",
                "logger_name": "recorder",
                "stream_name": "startup",
                "rows_written": "1",
                "rows_dropped": "0",
                "queue_depth": "0",
                "max_queue_depth": "0",
                "last_write_error": "",
                "t_wall": "1.0",
                "t_ros": "1.0",
                "configured_subscriptions_count": "0",
            }
        ],
    )

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["status"] == "failed"
    assert any("missing endpoint policy fields" in failure for failure in report["failures"])


def test_h4_fails_without_h3_prereq_pass(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)

    report = validator.validate_manifest(str(manifest_path), gate="H4")

    assert report["status"] == "failed"
    assert any("H3" in failure for failure in report["failures"])


def test_h4_fails_when_h3_prereq_object_is_missing(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["prereq_gate_results"].pop("H3", None)
    manifest["operator_safety_precheck"] = {
        "operator_precheck_ack": True,
        "safety_observer_present": True,
        "emergency_stop_tested_pre_run": True,
        "workspace_clear": True,
        "degradation_scope_confirmed": True,
        "safety_channels_excluded_from_degradation": True,
    }
    manifest["manifest_sha256"] = manifest_script._canonical_manifest_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    report = validator.validate_manifest(str(manifest_path), gate="H4")

    assert report["status"] == "failed"
    assert any("H3.status" in failure or "H3" in failure for failure in report["failures"])


def test_h4_fails_with_gateway_only_ping_without_waiver(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["prereq_gate_results"]["H3"] = {
        "status": "passed",
        "validation_report": "/tmp/h3_validation_report.json",
        "run_id": manifest["run_id"],
        "manifest_sha256": manifest["manifest_sha256"],
    }
    manifest["operator_safety_precheck"] = {
        "operator_precheck_ack": True,
        "safety_observer_present": True,
        "emergency_stop_tested_pre_run": True,
        "workspace_clear": True,
        "degradation_scope_confirmed": True,
        "safety_channels_excluded_from_degradation": True,
    }
    manifest["manifest_sha256"] = manifest_script._canonical_manifest_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    report = validator.validate_manifest(str(manifest_path), gate="H4")

    assert report["status"] == "failed"
    assert any("gateway-only" in failure for failure in report["failures"])


def test_h5_fails_when_operator_safety_precheck_is_false(tmp_path):
    manifest_path = _hardware_manifest(tmp_path)
    _write_h0_files(tmp_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["prereq_gate_results"]["H3"] = {
        "status": "passed",
        "validation_report": "/tmp/h3_validation_report.json",
        "run_id": manifest["run_id"],
        "manifest_sha256": manifest["manifest_sha256"],
    }
    manifest["manifest_sha256"] = manifest_script._canonical_manifest_sha256(manifest)
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    report = validator.validate_manifest(str(manifest_path), gate="H5")

    assert report["status"] == "failed"
    assert any("operator_safety_precheck" in failure for failure in report["failures"])


def test_non_hardware_validation_behavior_still_passes(tmp_path):
    manifest_path = _legacy_manifest(tmp_path)

    report = validator.validate_manifest(str(manifest_path), gate="H0")

    assert report["passed"] is True
    assert "checks" in report
    assert "gate" not in report
