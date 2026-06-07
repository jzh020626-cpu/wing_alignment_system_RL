from __future__ import annotations

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_safe_idle_launch_references_hardware_preliminary_and_sidecars():
    text = (PACKAGE_ROOT / "launch" / "safe_idle_validation.launch.py").read_text(encoding="utf-8")

    assert "hardware_preliminary_safe_idle.launch.py" in text
    assert "phase_source_bridge" in text
    assert "shadow_policy_sidecar" in text
    assert "cmd_channel_wrapper" in text
    assert "observe_only_traffic_source" in text
    assert "operator_id" in text
    assert "measurement_run_id" in text
    assert "mission_runtime_events_path" in text
    assert "phase_source_mode" in text
    assert "fallback_policy" in text
    assert "\"run_id\": measurement_run_id" in text
    assert "\"mission_log_root\": mission_log_root" in text
    assert "start_observe_only_synthetic_source" in text
    assert "wrapper_metadata_log_root" in text
    assert "\"metadata_log_root\": wrapper_metadata_log_root" in text


def test_controlled_motion_launch_references_system_bringup_and_tx_sidecar():
    text = (PACKAGE_ROOT / "launch" / "controlled_motion_validation.launch.py").read_text(encoding="utf-8")

    assert "system_bringup.launch.py" in text
    assert "tx_policy_sidecar" in text
    assert "/fr_validation/{robot}/cmd_vel_stamped_tx" in text
    assert "wrapper_mode" in text
    assert "mission_log_dir" in text


def test_mission_aware_shadow_launch_references_safe_idle_replay_and_tx_chain():
    text = (PACKAGE_ROOT / "launch" / "mission_aware_shadow_validation.launch.py").read_text(encoding="utf-8")

    assert "hardware_preliminary_safe_idle.launch.py" in text
    assert "validation_mode" in text
    assert "runtime_replay" in text
    assert "phase_source_mode" in text
    assert "replay_speed" in text
    assert "enable_execution_mode" in text
    assert "enable_execution_mode_output" in text
    assert "SetRemap" in text
    assert "tx_policy_sidecar" in text
    assert "cmd_channel_wrapper" in text
    assert "observe_only_traffic_source" in text
    assert "enable_comm_proxy" in text


def test_phase_source_config_locks_runtime_priority_and_pose_mapping():
    config = yaml.safe_load((PACKAGE_ROOT / "config" / "real_system_phase_source.yaml").read_text(encoding="utf-8"))

    assert config["source_priority"] == ["native_topic", "mission_runtime_tail", "geometry_heuristic"]
    assert config["pose_topics"]["tracer1"] == "/Rigid17/pose"
    assert config["pose_topics"]["tracer2"] == "/Rigid14/pose"
    assert config["pose_topics"]["tracer3"] == "/Rigid15/pose"
    assert config["wing_pose_topic"] == "/Rigid8/pose"


def test_comm_scenarios_config_contains_three_real_profiles():
    config = yaml.safe_load((PACKAGE_ROOT / "config" / "real_comm_scenarios.yaml").read_text(encoding="utf-8"))

    assert set(config["scenarios"]) == {"real-nominal", "real-main", "real-boundary-onset"}
    assert config["scenarios"]["real-main"]["wrapper_mode"] == "active"
    assert config["scenarios"]["real-boundary-onset"]["duplicate_on_critical"] is True


def test_setup_registers_observe_only_traffic_source_entry_point():
    text = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "observe_only_traffic_source = freshness_real_robot_validation.observe_only_traffic_source:main" in text
