from __future__ import annotations

import csv
import time
from copy import deepcopy
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from freshness_real_robot_validation.communication_execution_proxy import build_comm_proxy_snapshot
from freshness_real_robot_validation.config_loader import load_yaml_config
from freshness_real_robot_validation.json_topics import payload_from_string_message, string_message_from_payload
from freshness_real_robot_validation.phase_contract import build_phase_status_payload
from freshness_real_robot_validation.phase_source_runtime import (
    build_phase_source_status,
    format_phase_source_status,
    resolve_phase_source_order,
    resolve_runtime_events_path,
)


class PhaseSourceBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("phase_source_bridge")
        config_path = str(self.declare_parameter("config_path", "").value)
        config = load_yaml_config(config_path)
        source_priority = config.get("source_priority", ["native_topic", "mission_runtime_tail", "geometry_heuristic"])

        self.run_id = str(self.declare_parameter("run_id", "real_robot_validation").value)
        self.output_topic = str(self.declare_parameter("output_topic", "/fr_validation/derived_phase_status").value)
        self.native_phase_topic = str(self.declare_parameter("native_phase_topic", "").value)
        self.mission_log_dir = str(self.declare_parameter("mission_log_dir", "").value)
        self.mission_log_root = str(self.declare_parameter("mission_log_root", self.mission_log_dir).value)
        self.mission_runtime_csv = str(self.declare_parameter("mission_runtime_csv", "").value)
        self.mission_runtime_events_path = str(self.declare_parameter("mission_runtime_events_path", "").value)
        self.enable_comm_proxy = bool(self.declare_parameter("enable_comm_proxy", False).value)
        self.scenario_id = str(self.declare_parameter("scenario_id", "real-nominal").value)
        self.comm_scenarios_config = str(self.declare_parameter("comm_scenarios_config", "").value)
        self.phase_source_mode = str(
            self.declare_parameter("phase_source_mode", config.get("default_phase_source_mode", "auto")).value
        )
        self.fallback_policy = str(
            self.declare_parameter("fallback_policy", config.get("default_fallback_policy", "allow_priority_fallback")).value
        )
        self.poll_period_sec = float(self.declare_parameter("poll_period_sec", 0.2).value)
        self.replay_speed = max(0.1, float(self.declare_parameter("replay_speed", 1.0).value))
        self.replay_loop = bool(self.declare_parameter("replay_loop", False).value)
        self.status_log_period_sec = max(0.5, float(self.declare_parameter("status_log_period_sec", 2.0).value))
        self.runtime_tail_stale_after_sec = max(
            self.poll_period_sec,
            float(self.declare_parameter("runtime_tail_stale_after_sec", 2.0).value),
        )
        self.pose_topics = dict(config.get("pose_topics", {}))
        self.wing_pose_topic = str(config.get("wing_pose_topic", "/Rigid8/pose"))
        self._comm_scenario_cfg = self._load_comm_scenario_cfg()
        self.source_priority = resolve_phase_source_order(
            config_priority=source_priority,
            phase_source_mode=self.phase_source_mode,
            fallback_policy=self.fallback_policy,
            mission_runtime_events_path=self.mission_runtime_events_path,
        )

        self.publisher = self.create_publisher(String, self.output_topic, 10)
        self._latest_native_payload: dict = {}
        self._last_valid_task_progress = 0.0
        self._robot_poses: dict[str, PoseStamped] = {}
        self._wing_pose: PoseStamped | None = None
        self._publish_count = 0
        self._phase_valid_count = 0
        self._task_progress_valid_count = 0
        self._last_status_log_monotonic = 0.0
        self._last_status_line = ""
        self._runtime_replay_path: Path | None = None
        self._runtime_replay_rows: list[dict] = []
        self._runtime_replay_index = 0
        self._runtime_replay_started_monotonic = time.monotonic()

        if self.native_phase_topic:
            self.create_subscription(String, self.native_phase_topic, self._native_phase_cb, 10)

        for robot_id, topic in self.pose_topics.items():
            self.create_subscription(PoseStamped, str(topic), self._mk_pose_cb(str(robot_id)), 10)
        self.create_subscription(PoseStamped, self.wing_pose_topic, self._wing_pose_cb, 10)

        self.create_timer(max(self.poll_period_sec, 0.05), self._publish_current_status)

    def _mk_pose_cb(self, robot_id: str):
        def cb(msg: PoseStamped) -> None:
            self._robot_poses[robot_id] = msg

        return cb

    def _wing_pose_cb(self, msg: PoseStamped) -> None:
        self._wing_pose = msg

    def _native_phase_cb(self, msg: String) -> None:
        payload = payload_from_string_message(msg)
        if payload:
            self._latest_native_payload = payload

    def _publish_current_status(self) -> None:
        selected_source_for_log = self.source_priority[0] if self.source_priority else "unavailable"
        for source_mode in self.source_priority:
            payload = self._payload_for_source(source_mode)
            if payload:
                selected_source_for_log = source_mode
                self._publish_count += 1
                if str(payload.get("task_phase", "")).strip():
                    self._phase_valid_count += 1
                try:
                    float(payload.get("task_progress", 0.0))
                    self._task_progress_valid_count += 1
                except (TypeError, ValueError):
                    pass
                self._last_valid_task_progress = float(payload.get("task_progress", self._last_valid_task_progress))
                self.publisher.publish(string_message_from_payload(payload))
                self._maybe_log_status(selected_source_for_log)
                return
        self._maybe_log_status(selected_source_for_log)

    def _payload_for_source(self, source_mode: str) -> dict | None:
        if source_mode == "native_topic":
            return self._augment_comm_proxy(self._native_payload())
        if source_mode == "mission_runtime_tail":
            return self._augment_comm_proxy(self._mission_runtime_payload())
        if source_mode == "mission_runtime_replay":
            return self._augment_comm_proxy(self._mission_runtime_replay_payload())
        if source_mode == "geometry_heuristic":
            return self._augment_comm_proxy(self._geometry_payload())
        return None

    def _augment_comm_proxy(self, payload: dict | None) -> dict | None:
        if payload is None or not self.enable_comm_proxy:
            return payload
        proxy_snapshot = build_comm_proxy_snapshot(
            task_phase=str(payload.get("task_phase", "standby")),
            scenario_id=self.scenario_id,
            scenario_cfg=self._comm_scenario_cfg,
        )
        enriched = dict(payload)
        enriched.update(proxy_snapshot)
        return enriched

    def _native_payload(self) -> dict | None:
        if not self._latest_native_payload:
            return None
        payload = deepcopy(self._latest_native_payload)
        mission_state = str(payload.get("mission_state", payload.get("task_phase", "STANDBY")))
        phase_progress_proxy = payload.get("phase_progress_proxy", payload.get("task_progress", 0.0))
        return build_phase_status_payload(
            mission_state=mission_state,
            source_mode="native_topic",
            run_id=self.run_id,
            last_valid_task_progress=self._last_valid_task_progress,
            confidence=float(payload.get("confidence", 1.0)),
            phase_progress_proxy=float(phase_progress_proxy),
            native_phase=str(payload.get("task_phase", "")),
        )

    def _mission_runtime_payload(self) -> dict | None:
        runtime_path = self._resolve_runtime_csv()
        if runtime_path is None or not runtime_path.exists():
            return None
        rows = list(csv.DictReader(runtime_path.read_text(encoding="utf-8").splitlines()))
        if not rows:
            return None
        row = rows[-1]
        mission_state = str(row.get("mission_state", "") or "").strip().upper()
        if not mission_state:
            return None
        return build_phase_status_payload(
            mission_state=mission_state,
            source_mode="mission_runtime_tail",
            run_id=self.run_id,
            last_valid_task_progress=self._last_valid_task_progress,
            confidence=0.95,
        )

    def _geometry_payload(self) -> dict | None:
        if self._wing_pose is None or len(self._robot_poses) < 3:
            return None
        wing = self._wing_pose.pose.position
        distances = []
        for pose in self._robot_poses.values():
            dx = float(pose.pose.position.x) - float(wing.x)
            dy = float(pose.pose.position.y) - float(wing.y)
            dz = float(pose.pose.position.z) - float(wing.z)
            distances.append((dx * dx + dy * dy + dz * dz) ** 0.5)
        avg_distance = sum(distances) / float(len(distances))

        if avg_distance > 1.50:
            mission_state = "WAIT_WING"
            phase_progress_proxy = 0.20
        elif avg_distance > 0.90:
            mission_state = "SYNC_SLIDE_ALIGN"
            phase_progress_proxy = 0.50
        elif avg_distance > 0.35:
            mission_state = "SYNC_RECENTER"
            phase_progress_proxy = 0.60
        else:
            mission_state = "TRANSPORT_PRECHECK"
            phase_progress_proxy = 0.30

        return build_phase_status_payload(
            mission_state=mission_state,
            source_mode="geometry_heuristic",
            run_id=self.run_id,
            last_valid_task_progress=self._last_valid_task_progress,
            confidence=0.40,
            phase_progress_proxy=phase_progress_proxy,
        )

    def _mission_runtime_replay_payload(self) -> dict | None:
        runtime_path = self._resolve_runtime_csv()
        if runtime_path is None or not runtime_path.exists():
            return None
        if runtime_path != self._runtime_replay_path or not self._runtime_replay_rows:
            self._runtime_replay_path = runtime_path
            self._runtime_replay_rows = self._load_runtime_replay_rows(runtime_path)
            self._runtime_replay_index = 0
            self._runtime_replay_started_monotonic = time.monotonic()
        if not self._runtime_replay_rows:
            return None

        elapsed_sec = max(0.0, (time.monotonic() - self._runtime_replay_started_monotonic) * self.replay_speed)
        max_offset_sec = float(self._runtime_replay_rows[-1]["offset_sec"])
        if self.replay_loop and max_offset_sec > 0.0:
            elapsed_sec = elapsed_sec % max_offset_sec
            if elapsed_sec < float(self._runtime_replay_rows[self._runtime_replay_index]["offset_sec"]):
                self._runtime_replay_index = 0

        while (
            self._runtime_replay_index + 1 < len(self._runtime_replay_rows)
            and float(self._runtime_replay_rows[self._runtime_replay_index + 1]["offset_sec"]) <= elapsed_sec
        ):
            self._runtime_replay_index += 1

        row = self._runtime_replay_rows[self._runtime_replay_index]
        return build_phase_status_payload(
            mission_state=str(row["mission_state"]),
            source_mode="mission_runtime_replay",
            run_id=self.run_id,
            last_valid_task_progress=self._last_valid_task_progress,
            confidence=0.95,
        )

    def _resolve_runtime_csv(self) -> Path | None:
        return resolve_runtime_events_path(
            mission_runtime_events_path=self.mission_runtime_events_path,
            mission_runtime_csv=self.mission_runtime_csv,
            mission_log_root=self.mission_log_root or self.mission_log_dir,
            run_id=self.run_id,
        )

    def _load_comm_scenario_cfg(self) -> dict:
        if not self.enable_comm_proxy or not str(self.comm_scenarios_config or "").strip():
            return {}
        config = load_yaml_config(self.comm_scenarios_config)
        return dict(config.get("scenarios", {}).get(self.scenario_id, {}))

    @staticmethod
    def _load_runtime_replay_rows(runtime_path: Path) -> list[dict]:
        rows = list(csv.DictReader(runtime_path.read_text(encoding="utf-8").splitlines()))
        replay_rows: list[dict] = []
        first_timestamp: float | None = None
        last_key: tuple[str, str] | None = None
        for row in rows:
            mission_state = str(row.get("mission_state", "") or "").strip().upper()
            task_phase = str(row.get("task_phase", "") or "").strip().lower()
            if not mission_state:
                continue
            timestamp = row.get("timestamp", "")
            try:
                timestamp_value = float(timestamp)
            except (TypeError, ValueError):
                timestamp_value = 0.0
            if first_timestamp is None:
                first_timestamp = timestamp_value
            key = (mission_state, task_phase)
            if key == last_key:
                continue
            replay_rows.append(
                {
                    "mission_state": mission_state,
                    "task_phase": task_phase,
                    "offset_sec": max(0.0, timestamp_value - float(first_timestamp or 0.0)),
                }
            )
            last_key = key
        return replay_rows

    def _maybe_log_status(self, selected_source_for_log: str) -> None:
        now = time.monotonic()
        if now - self._last_status_log_monotonic < self.status_log_period_sec:
            return

        runtime_path = self._resolve_runtime_csv()
        file_exists = bool(runtime_path and runtime_path.exists())
        last_modified_ns = runtime_path.stat().st_mtime_ns if file_exists and runtime_path is not None else None
        tail_update_status = "missing"
        if file_exists and runtime_path is not None and last_modified_ns is not None:
            age_sec = max(0.0, time.time() - (last_modified_ns / 1_000_000_000))
            tail_update_status = "fresh" if age_sec <= self.runtime_tail_stale_after_sec else "stale"

        status = build_phase_source_status(
            selected_phase_source=selected_source_for_log,
            runtime_path=runtime_path,
            file_exists=file_exists,
            last_modified_ns=last_modified_ns,
            tail_update_status=tail_update_status,
            phase_valid_count=self._phase_valid_count,
            task_progress_valid_count=self._task_progress_valid_count,
            publish_count=self._publish_count,
        )
        line = format_phase_source_status(status)
        if line != self._last_status_line:
            self.get_logger().info(line)
            self._last_status_line = line
        self._last_status_log_monotonic = now


def main() -> None:
    rclpy.init(args=None)
    node = PhaseSourceBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
