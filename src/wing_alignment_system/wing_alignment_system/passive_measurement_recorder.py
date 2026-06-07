#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from std_msgs.msg import Bool, Float32MultiArray
try:
    from base_interfaces_demo.msg import MotorCommand, MotorStatus
except Exception:  # pragma: no cover - test env may not have generated Python msg bindings
    class MotorStatus:  # type: ignore[no-redef]
        pass

    class MotorCommand:  # type: ignore[no-redef]
        pass


DEFAULT_ROBOTS = ["tracer1", "tracer2", "tracer3"]
DEFAULT_SLIDES = ["huatai1", "huatai2", "huatai3"]
DEFAULT_WING_MOCAP_TOPIC = "/Rigid8/pose"
DEFAULT_ROBOT_MOCAP_TOPICS = ["/Rigid17/pose", "/Rigid14/pose", "/Rigid15/pose"]

RCLError = _rclpy.RCLError

COMMON_FIELDS = [
    "run_id",
    "stream_name",
    "topic",
    "robot_id",
    "slide_id",
    "classification",
    "t_receive_wall",
    "t_receive_ros",
    "msg_source_stamp",
    "source_stamp_valid",
    "stamp_origin",
    "source_time_base",
    "frame_id",
]


class _NonBlockingCsvWriter:
    def __init__(self, path: str, fieldnames: list[str], max_queue: int = 4096):
        self.path = path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._fieldnames = list(fieldnames)
        self._queue: queue.Queue[dict] = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._rows_written = 0
        self._rows_dropped = 0
        self._max_queue_depth = 0
        self._last_write_error = ""
        self._lock = threading.Lock()
        self._closed = False
        self._fp = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._fp, fieldnames=self._fieldnames, extrasaction="ignore")
        self._writer.writeheader()
        self._fp.flush()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def log(self, row: dict) -> None:
        with self._lock:
            if self._closed:
                return
            try:
                self._queue.put_nowait(row)
                depth = self._queue.qsize()
                if depth > self._max_queue_depth:
                    self._max_queue_depth = depth
            except queue.Full:
                self._rows_dropped += 1

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "rows_written": self._rows_written,
                "rows_dropped": self._rows_dropped,
                "queue_depth": self._queue.qsize(),
                "max_queue_depth": self._max_queue_depth,
                "last_write_error": self._last_write_error,
            }

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._stop.set()
        self._thread.join(timeout=2.0)
        while True:
            try:
                self._write_row(self._queue.get_nowait())
            except queue.Empty:
                break
        try:
            self._fp.flush()
        except ValueError:
            return
        try:
            self._fp.close()
        except ValueError:
            pass

    def _write_row(self, row: dict) -> None:
        try:
            self._writer.writerow(row)
            with self._lock:
                self._rows_written += 1
        except Exception as exc:  # pragma: no cover - defensive
            with self._lock:
                self._last_write_error = str(exc)

    def _run(self) -> None:
        last_flush = time.time()
        while not self._stop.is_set():
            try:
                row = self._queue.get(timeout=0.2)
                self._write_row(row)
            except queue.Empty:
                pass
            if time.time() - last_flush >= 0.5:
                self._fp.flush()
                last_flush = time.time()


@dataclass
class _TopicSpec:
    topic: str
    message_type: str
    message_cls: object
    stream_name: str
    artifact: str
    classification: str
    qos: QoSProfile
    qos_note: str
    required_gate: str
    robot_id: str = ""
    slide_id: str = ""


def build_endpoint_policy_status(configured_subscriptions_count: int) -> dict:
    return {
        "control_publishers_count": 0,
        "user_defined_publishers_count": 0,
        "service_clients_count": 0,
        "user_defined_services_count": 0,
        "ros_infrastructure_endpoints": "[]",
        "configured_subscriptions_count": int(configured_subscriptions_count),
    }


def _parse_csv_arg(value: str, default: list[str]) -> list[str]:
    items = [part.strip() for part in str(value).split(",") if part.strip()]
    return items or list(default)


def _extract_first(text: str, pattern: str, default: str = "") -> str:
    match = re.search(pattern, text, flags=re.MULTILINE)
    return match.group(1).strip() if match else default


def _extract_list(text: str, pattern: str, default: list[str]) -> list[str]:
    match = re.search(pattern, text, flags=re.MULTILINE)
    if not match:
        return list(default)
    raw = match.group(1).strip()
    if not raw:
        return list(default)
    items = [part.strip().strip("'\"") for part in raw.split(",") if part.strip()]
    return items or list(default)


def _load_config_defaults(config_file: str) -> dict:
    defaults = {
        "robots": list(DEFAULT_ROBOTS),
        "slides": list(DEFAULT_SLIDES),
        "wing_mocap_topic": DEFAULT_WING_MOCAP_TOPIC,
        "robot_mocap_topics": list(DEFAULT_ROBOT_MOCAP_TOPICS),
    }
    if not config_file:
        return defaults
    path = Path(config_file).expanduser()
    if not path.exists():
        return defaults
    text = path.read_text(encoding="utf-8")
    robots = _extract_list(text, r"^\s*robots:\s*\[(.*?)\]\s*$", defaults["robots"])
    slides = [robot.replace("tracer", "huatai") if "tracer" in robot else robot for robot in robots]
    defaults["robots"] = robots
    defaults["slides"] = slides or list(DEFAULT_SLIDES)
    defaults["wing_mocap_topic"] = _extract_first(
        text,
        r'^\s*wing_mocap_topic:\s*"([^"]+)"\s*$',
        defaults["wing_mocap_topic"],
    )
    defaults["robot_mocap_topics"] = _extract_list(
        text,
        r"^\s*robot_mocap_topics:\s*\[(.*?)\]\s*$",
        defaults["robot_mocap_topics"],
    )
    if len(defaults["robot_mocap_topics"]) != len(defaults["robots"]):
        defaults["robot_mocap_topics"] = list(DEFAULT_ROBOT_MOCAP_TOPICS[: len(defaults["robots"])])
    return defaults


def _stamp_to_sec(stamp) -> float:
    sec = float(getattr(stamp, "sec", 0.0) or 0.0)
    nanosec = float(getattr(stamp, "nanosec", 0.0) or 0.0)
    value = sec + nanosec * 1e-9
    return value if value > 0.0 else 0.0


def _fmt_sec(value: float) -> str:
    return f"{float(value):.6f}" if float(value) > 0.0 else ""


def _make_qos_profiles() -> tuple[QoSProfile, QoSProfile, QoSProfile]:
    qos_be = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    qos_rel = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    qos_emg = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    return qos_be, qos_rel, qos_emg


def build_topic_specs(robots: list[str], slides: list[str], config_file: str = "") -> list[_TopicSpec]:
    qos_be, qos_rel, qos_emg = _make_qos_profiles()
    specs: list[_TopicSpec] = []
    config = _load_config_defaults(config_file)
    specs.append(
        _TopicSpec(
            topic=config["wing_mocap_topic"],
            message_type="geometry_msgs/msg/PoseStamped",
            message_cls=PoseStamped,
            stream_name="wing_mocap",
            artifact="raw_qr_samples.csv",
            classification="feedback",
            qos=qos_be,
            qos_note="best_effort",
            required_gate="H1",
        )
    )
    for robot, mocap_topic in zip(robots, config["robot_mocap_topics"]):
        specs.append(
            _TopicSpec(
                topic=mocap_topic,
                message_type="geometry_msgs/msg/PoseStamped",
                message_cls=PoseStamped,
                stream_name="robot_mocap",
                artifact="raw_qr_samples.csv",
                classification="feedback",
                qos=qos_be,
                qos_note="best_effort",
                required_gate="H1",
                robot_id=robot,
            )
        )
        specs.extend(
            [
                _TopicSpec(f"/{robot}/object_position", "geometry_msgs/msg/PoseStamped", PoseStamped, "raw_qr", "raw_qr_samples.csv", "feedback", qos_be, "best_effort", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/wing_alignment/delta", "geometry_msgs/msg/Vector3Stamped", Vector3Stamped, "qr_delta", "delta_samples.csv", "feedback", qos_be, "best_effort", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_goal", "geometry_msgs/msg/Twist", Twist, "cmd_goal", "chassis_command_samples.csv", "feedback", qos_rel, "reliable_transient_local_source", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_vel_desired", "geometry_msgs/msg/Twist", Twist, "cmd_vel_desired", "chassis_command_samples.csv", "feedback", qos_rel, "reliable_volatile", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_vel_stamped", "geometry_msgs/msg/TwistStamped", TwistStamped, "cmd_vel_stamped", "chassis_command_samples.csv", "feedback", qos_be, "best_effort", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_vel", "geometry_msgs/msg/Twist", Twist, "cmd_vel", "chassis_command_samples.csv", "feedback", qos_rel, "reliable_volatile", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_stop", "std_msgs/msg/Bool", Bool, "cmd_stop", "safety_events.csv", "feedback", qos_rel, "reliable_volatile", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/cmd_resume", "std_msgs/msg/Bool", Bool, "cmd_resume", "safety_events.csv", "feedback", qos_rel, "reliable_volatile", "H1", robot_id=robot),
                _TopicSpec(f"/{robot}/precision_mode", "std_msgs/msg/Bool", Bool, "precision_mode", "safety_events.csv", "feedback", qos_rel, "reliable_volatile", "H1", robot_id=robot),
            ]
        )
    specs.append(
        _TopicSpec(
            "/wing_alignment/emergency_stop",
            "std_msgs/msg/Bool",
            Bool,
            "emergency_stop",
            "safety_events.csv",
            "feedback",
            qos_emg,
            "reliable_transient_local",
            "H1",
        )
    )
    for slide in slides:
        specs.extend(
            [
                _TopicSpec(f"/{slide}_force_filtered", "std_msgs/msg/Float32MultiArray", Float32MultiArray, "force_filtered", "force_samples.csv", "measured", qos_be, "best_effort", "H1", slide_id=slide),
                _TopicSpec(f"/{slide}_pos_spe_p", "base_interfaces_demo/msg/MotorStatus", MotorStatus, "slide_status", "slide_status_samples.csv", "feedback", qos_rel, "reliable_volatile", "H1", slide_id=slide),
                _TopicSpec(f"/{slide}/force_contact", "std_msgs/msg/Bool", Bool, "force_contact", "safety_events.csv", "feedback", qos_emg, "reliable_transient_local", "H1", slide_id=slide),
                _TopicSpec(f"/{slide}_pos_spe_pd", "base_interfaces_demo/msg/MotorCommand", MotorCommand, "slide_pos_spe_pd", "slide_command_samples.csv", "feedback", qos_rel, "reliable_volatile", "H1", slide_id=slide),
                _TopicSpec(f"/{slide}_compensation_ref", "base_interfaces_demo/msg/MotorCommand", MotorCommand, "slide_compensation_ref", "slide_command_samples.csv", "feedback", qos_rel, "reliable_volatile", "H1", slide_id=slide),
            ]
        )
    return specs


def _create_writers(out_dir: str) -> dict[str, _NonBlockingCsvWriter]:
    return {
        "recorder_health.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "recorder_health.csv"),
            [
                "run_id",
                "logger_name",
                "stream_name",
                "rows_written",
                "rows_dropped",
                "queue_depth",
                "max_queue_depth",
                "last_write_error",
                "control_publishers_count",
                "user_defined_publishers_count",
                "service_clients_count",
                "user_defined_services_count",
                "ros_infrastructure_endpoints",
                "configured_subscriptions_count",
                "t_wall",
                "t_ros",
            ],
        ),
        "recorder_topic_status.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "recorder_topic_status.csv"),
            [
                "run_id",
                "topic",
                "message_type",
                "configured",
                "observed",
                "row_count",
                "last_observed_wall",
                "last_observed_ros",
                "source_stamp_observed",
                "qos_note",
                "classification",
                "required_gate",
            ],
        ),
        "raw_qr_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "raw_qr_samples.csv"),
            COMMON_FIELDS + ["x", "y", "z", "qx", "qy", "qz", "qw"],
        ),
        "delta_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "delta_samples.csv"),
            COMMON_FIELDS + ["x", "y", "z"],
        ),
        "force_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "force_samples.csv"),
            COMMON_FIELDS + ["data_json"],
        ),
        "slide_status_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "slide_status_samples.csv"),
            COMMON_FIELDS + ["x", "y", "z", "vx", "vy", "vz", "reached_target", "gx", "gy", "gz", "grx", "gry", "grz"],
        ),
        "safety_events.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "safety_events.csv"),
            COMMON_FIELDS + ["bool_value"],
        ),
        "chassis_command_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "chassis_command_samples.csv"),
            COMMON_FIELDS + ["command_kind", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"],
        ),
        "slide_command_samples.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "slide_command_samples.csv"),
            COMMON_FIELDS + ["command_type", "x", "y", "z", "time", "is_relative", "vx", "vy", "vz", "can_id"],
        ),
        "recorder_callback_timing.csv": _NonBlockingCsvWriter(
            os.path.join(out_dir, "recorder_callback_timing.csv"),
            [
                "run_id",
                "callback_owner",
                "stream_name",
                "topic",
                "t_callback_start_wall",
                "t_callback_end_wall",
                "duration_ms",
                "classification",
                "interpretation_note",
            ],
        ),
    }


def write_minimal_h0_artifacts(run_id: str, out_dir: str, robots: list[str], slides: list[str], config_file: str = "") -> dict[str, str]:
    out_dir = os.path.expanduser(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    specs = build_topic_specs(robots, slides, config_file=config_file)
    writers = _create_writers(out_dir)
    now_wall = _fmt_sec(time.time())
    now_ros = now_wall
    endpoint_status = build_endpoint_policy_status(len(specs))
    for spec in specs:
        writers["recorder_topic_status.csv"].log(
            {
                "run_id": run_id,
                "topic": spec.topic,
                "message_type": spec.message_type,
                "configured": "true",
                "observed": "false",
                "row_count": 0,
                "last_observed_wall": "",
                "last_observed_ros": "",
                "source_stamp_observed": "false",
                "qos_note": spec.qos_note,
                "classification": spec.classification,
                "required_gate": spec.required_gate,
            }
        )
    for name, artifact_writer in writers.items():
        stats = artifact_writer.snapshot()
        writers["recorder_health.csv"].log(
            {
                "run_id": run_id,
                "logger_name": "passive_measurement_recorder",
                "stream_name": name,
                "rows_written": stats["rows_written"],
                "rows_dropped": stats["rows_dropped"],
                "queue_depth": stats["queue_depth"],
                "max_queue_depth": stats["max_queue_depth"],
                "last_write_error": stats["last_write_error"],
                "t_wall": now_wall,
                "t_ros": now_ros,
                **endpoint_status,
            }
        )
    for writer in writers.values():
        writer.close()
    return {name: writer.path for name, writer in writers.items()}


def _is_normal_shutdown_rcl_error(exc: BaseException) -> bool:
    text = str(exc).lower()
    return (
        "rcl_shutdown already called" in text
        or "destroyable because destruction was requested" in text
        or "cannot shutdown a handle that is not initialized" in text
        or "context is invalid" in text
        or "invalid handle" in text
        or ("wait set" in text and "context" in text and "not valid" in text)
    )


def _safe_rclpy_shutdown() -> None:
    try:
        ok_fn = getattr(rclpy, "ok", None)
        if callable(ok_fn):
            if ok_fn():
                rclpy.shutdown()
        else:
            rclpy.shutdown()
    except (RCLError, RuntimeError, ValueError) as exc:
        if not _is_normal_shutdown_rcl_error(exc):
            raise


def shutdown_passive_measurement_recorder(node, executor=None) -> None:
    if getattr(node, "_shutdown_completed", False):
        return
    setattr(node, "_shutdown_completed", True)
    try:
        node.close()
    finally:
        if executor is not None:
            try:
                executor.remove_node(node)
            except (AttributeError, RuntimeError, ValueError):
                pass
        try:
            node.destroy_node()
        except (AttributeError, RuntimeError, ValueError):
            pass
        _safe_rclpy_shutdown()


def run_passive_measurement_recorder(node, executor, spin_timeout_sec: float = 0.1) -> None:
    deadline = None
    if node.duration_sec > 0.0:
        deadline = time.monotonic() + node.duration_sec
    while True:
        try:
            if not rclpy.ok():
                break
        except RCLError as exc:
            if _is_normal_shutdown_rcl_error(exc):
                break
            raise
        if deadline is not None and time.monotonic() >= deadline:
            break
        try:
            executor.spin_once(timeout_sec=spin_timeout_sec)
        except ExternalShutdownException:
            break
        except RCLError as exc:
            if _is_normal_shutdown_rcl_error(exc):
                break
            raise


class PassiveMeasurementRecorder(Node):
    def __init__(self, run_id: str, out_dir: str, robots: list[str], slides: list[str], config_file: str = "", duration_sec: float = 0.0):
        super().__init__(
            "passive_measurement_recorder",
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.run_id = run_id
        self.out_dir = os.path.expanduser(out_dir)
        self.robots = list(robots)
        self.slides = list(slides)
        self.config_file = config_file
        self.duration_sec = max(0.0, float(duration_sec))
        os.makedirs(self.out_dir, exist_ok=True)
        self._writers = _create_writers(self.out_dir)
        self._close_lock = threading.Lock()
        self._closed = False

        self._topic_status: dict[str, dict] = {}
        self._subscriptions = []
        self._specs = build_topic_specs(self.robots, self.slides, config_file=self.config_file)
        for spec in self._specs:
            self._topic_status[spec.topic] = {
                "message_type": spec.message_type,
                "configured": True,
                "observed": False,
                "row_count": 0,
                "last_observed_wall": "",
                "last_observed_ros": "",
                "source_stamp_observed": False,
                "qos_note": spec.qos_note,
                "classification": spec.classification,
                "required_gate": spec.required_gate,
            }
            self._subscriptions.append(
                self.create_subscription(
                    spec.message_cls,
                    spec.topic,
                    self._make_callback(spec),
                    spec.qos,
                )
            )

        self.create_timer(2.0, self._emit_health_rows)
        self.create_timer(2.0, self._emit_topic_status_rows)
        self._emit_topic_status_rows()
        self._emit_health_rows()

    def _now_ros(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _base_row(self, spec: _TopicSpec, source_stamp: float = 0.0, frame_id: str = "", source_valid: bool = False, stamp_origin: str = "none", source_time_base: str = "unknown") -> dict:
        now_wall = time.time()
        now_ros = self._now_ros()
        msg_source_stamp = source_stamp if source_stamp > 0.0 else (now_ros if stamp_origin == "local_receive_fallback" else 0.0)
        return {
            "run_id": self.run_id,
            "stream_name": spec.stream_name,
            "topic": spec.topic,
            "robot_id": spec.robot_id,
            "slide_id": spec.slide_id,
            "classification": spec.classification,
            "t_receive_wall": _fmt_sec(now_wall),
            "t_receive_ros": _fmt_sec(now_ros),
            "msg_source_stamp": _fmt_sec(msg_source_stamp),
            "source_stamp_valid": "true" if source_valid else "false",
            "stamp_origin": stamp_origin,
            "source_time_base": source_time_base,
            "frame_id": frame_id,
        }

    def _callback_timing(self, spec: _TopicSpec, start_wall: float, end_wall: float) -> None:
        self._writers["recorder_callback_timing.csv"].log(
            {
                "run_id": self.run_id,
                "callback_owner": "passive_recorder",
                "stream_name": spec.stream_name,
                "topic": spec.topic,
                "t_callback_start_wall": _fmt_sec(start_wall),
                "t_callback_end_wall": _fmt_sec(end_wall),
                "duration_ms": f"{(end_wall - start_wall) * 1e3:.6f}",
                "classification": "proxy",
                "interpretation_note": "not_controller_callback_timing",
            }
        )

    def _mark_observed(self, spec: _TopicSpec, source_stamp_observed: bool) -> None:
        status = self._topic_status[spec.topic]
        status["observed"] = True
        status["row_count"] += 1
        status["last_observed_wall"] = _fmt_sec(time.time())
        status["last_observed_ros"] = _fmt_sec(self._now_ros())
        status["source_stamp_observed"] = source_stamp_observed

    def _make_callback(self, spec: _TopicSpec):
        def cb(msg):
            start_wall = time.time()
            source_stamp = 0.0
            source_valid = False
            stamp_origin = "none"
            source_time_base = "unknown"
            frame_id = ""
            header = getattr(msg, "header", None)
            if header is not None:
                source_stamp = _stamp_to_sec(header.stamp)
                frame_id = str(getattr(header, "frame_id", "") or "")
                if source_stamp > 0.0:
                    source_valid = True
                    stamp_origin = "upstream_header"
                    source_time_base = "ros_time"
                else:
                    source_valid = False
                    stamp_origin = "local_receive_fallback"
                    source_time_base = "ros_time"
            row = self._base_row(
                spec,
                source_stamp=source_stamp,
                frame_id=frame_id,
                source_valid=source_valid,
                stamp_origin=stamp_origin,
                source_time_base=source_time_base,
            )
            if isinstance(msg, PoseStamped):
                row.update(
                    {
                        "x": float(msg.pose.position.x),
                        "y": float(msg.pose.position.y),
                        "z": float(msg.pose.position.z),
                        "qx": float(msg.pose.orientation.x),
                        "qy": float(msg.pose.orientation.y),
                        "qz": float(msg.pose.orientation.z),
                        "qw": float(msg.pose.orientation.w),
                    }
                )
            elif isinstance(msg, Vector3Stamped):
                row.update({"x": float(msg.vector.x), "y": float(msg.vector.y), "z": float(msg.vector.z)})
            elif isinstance(msg, Float32MultiArray):
                row.update({"data_json": json.dumps(list(msg.data), ensure_ascii=True)})
            elif isinstance(msg, MotorStatus):
                row.update(
                    {
                        "x": float(msg.x),
                        "y": float(msg.y),
                        "z": float(msg.z),
                        "vx": float(msg.vx),
                        "vy": float(msg.vy),
                        "vz": float(msg.vz),
                        "reached_target": "true" if bool(msg.reached_target) else "false",
                        "gx": float(msg.gx),
                        "gy": float(msg.gy),
                        "gz": float(msg.gz),
                        "grx": float(msg.grx),
                        "gry": float(msg.gry),
                        "grz": float(msg.grz),
                    }
                )
            elif isinstance(msg, Bool):
                row.update({"bool_value": "true" if bool(msg.data) else "false"})
            elif isinstance(msg, TwistStamped):
                row.update(
                    {
                        "command_kind": spec.stream_name,
                        "linear_x": float(msg.twist.linear.x),
                        "linear_y": float(msg.twist.linear.y),
                        "linear_z": float(msg.twist.linear.z),
                        "angular_x": float(msg.twist.angular.x),
                        "angular_y": float(msg.twist.angular.y),
                        "angular_z": float(msg.twist.angular.z),
                    }
                )
            elif isinstance(msg, Twist):
                row.update(
                    {
                        "command_kind": spec.stream_name,
                        "linear_x": float(msg.linear.x),
                        "linear_y": float(msg.linear.y),
                        "linear_z": float(msg.linear.z),
                        "angular_x": float(msg.angular.x),
                        "angular_y": float(msg.angular.y),
                        "angular_z": float(msg.angular.z),
                    }
                )
            elif isinstance(msg, MotorCommand):
                row.update(
                    {
                        "command_type": str(msg.command_type),
                        "x": float(msg.x),
                        "y": float(msg.y),
                        "z": float(msg.z),
                        "time": float(msg.time),
                        "is_relative": "true" if bool(msg.is_relative) else "false",
                        "vx": float(msg.vx),
                        "vy": float(msg.vy),
                        "vz": float(msg.vz),
                        "can_id": str(msg.can_id),
                    }
                )
            self._writers[spec.artifact].log(row)
            self._mark_observed(spec, source_valid)
            self._callback_timing(spec, start_wall, time.time())
        return cb

    def _emit_topic_status_rows(self) -> None:
        writer = self._writers["recorder_topic_status.csv"]
        for topic, state in self._topic_status.items():
            writer.log(
                {
                    "run_id": self.run_id,
                    "topic": topic,
                    "message_type": state["message_type"],
                    "configured": "true" if state["configured"] else "false",
                    "observed": "true" if state["observed"] else "false",
                    "row_count": int(state["row_count"]),
                    "last_observed_wall": state["last_observed_wall"],
                    "last_observed_ros": state["last_observed_ros"],
                    "source_stamp_observed": "true" if state["source_stamp_observed"] else "false",
                    "qos_note": state["qos_note"],
                    "classification": state["classification"],
                    "required_gate": state["required_gate"],
                }
            )

    def _emit_health_rows(self) -> None:
        now_wall = _fmt_sec(time.time())
        now_ros = _fmt_sec(self._now_ros())
        writer = self._writers["recorder_health.csv"]
        endpoint_status = build_endpoint_policy_status(len(self._specs))
        for name, artifact_writer in self._writers.items():
            stats = artifact_writer.snapshot()
            writer.log(
                {
                    "run_id": self.run_id,
                    "logger_name": "passive_measurement_recorder",
                    "stream_name": name,
                    "rows_written": stats["rows_written"],
                    "rows_dropped": stats["rows_dropped"],
                    "queue_depth": stats["queue_depth"],
                    "max_queue_depth": stats["max_queue_depth"],
                    "last_write_error": stats["last_write_error"],
                    "t_wall": now_wall,
                    "t_ros": now_ros,
                    **endpoint_status,
                }
            )

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
        self._emit_topic_status_rows()
        self._emit_health_rows()
        for writer in self._writers.values():
            writer.close()


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Passive real-machine measurement recorder.")
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--require-run-id", action="store_true")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--config-file", default="")
    parser.add_argument("--robots", default=",".join(DEFAULT_ROBOTS))
    parser.add_argument("--slides", default="")
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parsed = parser.parse_args(argv)
    parsed.run_id = str(parsed.run_id).strip()
    if parsed.require_run_id and not parsed.run_id:
        parser.error("--require-run-id requires a non-empty --run-id")
    return parsed


def main(args=None) -> None:
    argv = remove_ros_args(args=args if args is not None else sys.argv)[1:]
    parsed = _parse_args(argv)
    config_defaults = _load_config_defaults(parsed.config_file)
    robots = _parse_csv_arg(parsed.robots, config_defaults["robots"])
    default_slides = [robot.replace("tracer", "huatai") if "tracer" in robot else robot for robot in robots]
    slides = _parse_csv_arg(parsed.slides, default_slides or config_defaults["slides"])

    rclpy.init(args=args)
    node = PassiveMeasurementRecorder(
        run_id=parsed.run_id,
        out_dir=parsed.out_dir,
        robots=robots,
        slides=slides,
        config_file=parsed.config_file,
        duration_sec=parsed.duration_sec,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        run_passive_measurement_recorder(node, executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except RCLError as exc:
        if _is_normal_shutdown_rcl_error(exc):
            pass
        else:
            raise
    finally:
        shutdown_passive_measurement_recorder(node, executor=executor)


if __name__ == "__main__":
    main()
