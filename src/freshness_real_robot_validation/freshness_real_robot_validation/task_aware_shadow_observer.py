from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

try:
    import numpy as np
except Exception:  # pragma: no cover - runtime guard only
    np = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
except Exception:  # pragma: no cover - offline import guard only
    rclpy = None
    Node = object  # type: ignore[assignment]
    QoSProfile = object  # type: ignore[assignment]
    ReliabilityPolicy = object  # type: ignore[assignment]
    DurabilityPolicy = object  # type: ignore[assignment]
    HistoryPolicy = object  # type: ignore[assignment]

try:
    from rclpy.serialization import serialize_message
except Exception:  # pragma: no cover - fallback only
    serialize_message = None

try:
    from rosidl_runtime_py.utilities import get_message
except Exception:  # pragma: no cover - fallback only
    get_message = None

from freshness_real_robot_validation.json_topics import payload_from_string_message


CRITICAL_PHASES = {"narrow_passage", "final_alignment", "slide_align", "level_recenter"}
LOW_RISK_PHASES = {"approach", "transport", "cooperative_transport", "release_exit"}
HIGH_RISK_PHASES = {"narrow_passage", "final_alignment"}
TRANSMISSION_MODES = ["skip_update", "compact_update", "full_update", "urgent_refresh"]
TX_RANK = {mode: idx for idx, mode in enumerate(TRANSMISSION_MODES)}
PAYLOAD_BYTES = {
    "skip_update": 0,
    "compact_update": 128,
    "full_update": 256,
    "urgent_refresh": 320,
}
CAPTURE_FIELDS = [
    "topic",
    "receive_time",
    "source_timestamp",
    "parse_ok",
    "qos_profile_used",
    "payload_summary",
    "nonempty",
    "robot_id",
    "phase",
    "task_progress",
    "peer_id",
    "msg_size_bytes",
    "inter_arrival_ms",
    "AoI_ms",
    "Effective_Freshness",
    "warning_flags",
]
STATE_FIELDS = [
    "state_row_id",
    "receive_time",
    "robot_id",
    "phase",
    "task_progress",
    "app_channel_topic",
    "app_channel_nonempty",
    "message_age",
    "inter_arrival",
    "AoI",
    "EffectiveFreshness",
    "CN_SRE_candidate",
    "pose_context_available",
    "status_context_available",
    "communication_observation_available",
    "missing_field_list",
    "state_schema_valid",
    "aoi_source",
    "effective_freshness_source",
    "cn_sre_source",
    "episode_id",
    "AoI_ms",
    "EffectiveFreshness_num",
    "CN_SRE_margin",
    "CN_SRE_step_or_component",
    "stale_indicator",
    "risk_score",
    "phase_progress",
    "execution_mode",
    "DGWS_execution_mode",
    "current_transmission_mode",
    "transmission_mode",
    "bytes_this_step",
    "background_agvs",
    "bottlenecks",
    "severity",
    "policy_name",
    "source_role",
    "exposure_split",
    "fallback_reason",
    "invalid_state",
]
ACTION_FIELDS = [
    "state_row_id",
    "timestamp",
    "phase",
    "task_progress",
    "freshness_fields",
    "policy_loaded",
    "action_candidate",
    "action_confidence",
    "fallback_reason",
    "not_applied_to_system",
    "used_fallback",
    "unsupported_context",
    "current_transmission_mode",
    "dgws_reference_action",
    "blocked_unsafe_downgrade",
    "policy_binding_mode",
]


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat()


def _format_float(value: float | None, digits: int = 6) -> str:
    if value is None:
        return ""
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return ""
    return f"{float(value):.{digits}f}"


def _robot_from_topic(topic: str) -> str:
    if "/tracer1/" in topic:
        return "tracer1"
    if "/tracer2/" in topic:
        return "tracer2"
    if "/tracer3/" in topic:
        return "tracer3"
    return "fleet"


def _peer_from_topic(topic: str) -> str:
    if topic.endswith("/cmd_vel_stamped"):
        return "mission_stack"
    if topic.endswith("/cmd_goal") or topic.endswith("/cmd_resume") or topic.endswith("/precision_mode"):
        return "mission_coordinator"
    if topic.endswith("/object_position"):
        return "perception_stack"
    if topic == "/fr_validation/derived_phase_status":
        return "mission_runtime_tail"
    return "unknown"


def _serialize_size(msg: Any) -> int:
    if serialize_message is not None:
        try:
            return len(serialize_message(msg))
        except Exception:
            pass
    text = repr(msg)
    return len(text.encode("utf-8"))


def _stamp_to_ns(stamp: Any) -> int | None:
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def _message_stamp_ns(msg: Any) -> int | None:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return _stamp_to_ns(getattr(header, "stamp", None))


def _resolve_message_class(msg_type: str):
    if get_message is not None:
        return get_message(msg_type)
    package_name, _, tail = msg_type.partition("/msg/")
    module = __import__(f"{package_name}.msg", fromlist=[tail])
    return getattr(module, tail)


def _normalize_qos_token(value: Any, default: str) -> str:
    if value is None:
        return default
    return str(value).strip().upper()


def _qos_profile_from_dict(payload: dict[str, Any]) -> QoSProfile:
    history_name = _normalize_qos_token(payload.get("history"), "KEEP_LAST")
    reliability_name = _normalize_qos_token(payload.get("reliability"), "RELIABLE")
    durability_name = _normalize_qos_token(payload.get("durability"), "VOLATILE")
    depth = max(int(payload.get("depth", 10)), 1)
    history = HistoryPolicy.KEEP_LAST if history_name != "KEEP_ALL" else HistoryPolicy.KEEP_ALL
    reliability = ReliabilityPolicy.BEST_EFFORT if reliability_name == "BEST_EFFORT" else ReliabilityPolicy.RELIABLE
    durability = DurabilityPolicy.TRANSIENT_LOCAL if durability_name == "TRANSIENT_LOCAL" else DurabilityPolicy.VOLATILE
    return QoSProfile(history=history, depth=depth, reliability=reliability, durability=durability)


def _qos_summary(payload: dict[str, Any]) -> str:
    return ";".join(
        [
            f"reliability={_normalize_qos_token(payload.get('reliability'), 'RELIABLE')}",
            f"durability={_normalize_qos_token(payload.get('durability'), 'VOLATILE')}",
            f"history={_normalize_qos_token(payload.get('history'), 'KEEP_LAST')}",
            f"depth={max(int(payload.get('depth', 10)), 1)}",
        ]
    )


def _load_yaml(path: str | Path) -> dict[str, Any]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise ValueError(f"config at {path} must be a mapping")
    return payload


def _phase_progress_from_task_progress(phase: str, task_progress: float) -> float:
    phase = str(phase)
    if phase == "standby":
        return 0.0
    if phase == "abort":
        return max(0.0, min(1.0, float(task_progress)))
    return max(0.0, min(1.0, float(task_progress)))


def _dgws_reference_mode(task_phase: str, effective_freshness: float, aoi_ms: float, stale_indicator: float) -> str:
    if stale_indicator >= 1.0 or effective_freshness < 0.40 or aoi_ms >= 350.0:
        return "urgent_refresh"
    if task_phase in CRITICAL_PHASES:
        return "full_update"
    return "full_update"


def _bucket_key(phase: str, aoi_bin: str, freshness_bin: str, margin_bin: str, stale_flag: int, execution_mode: str) -> str:
    return "|".join([str(phase), str(aoi_bin), str(freshness_bin), str(margin_bin), str(int(stale_flag)), str(execution_mode)])


class RepairedPolicyEngine:
    def __init__(self, policy_path: str | Path):
        self.policy_path = str(Path(policy_path).resolve())
        self.policy = json.loads(Path(policy_path).read_text(encoding="utf-8"))
        self.policy_loaded = bool(self.policy)
        self.binding_mode = "repaired_policy_candidate_json"

    def _label_from_edges(self, value: float, edges: list[float], labels: list[str]) -> str:
        if np is None:
            idx = 0
            for boundary in edges[1:-1]:
                if value > float(boundary):
                    idx += 1
            idx = max(0, min(idx, len(labels) - 1))
            return str(labels[idx])
        boundaries = list(edges[1:-1])
        idx = int(np.digitize([float(value)], boundaries, right=True)[0])
        idx = max(0, min(idx, len(labels) - 1))
        return str(labels[idx])

    def _row_to_feature_vector(self, record: dict[str, Any], candidate_action: str, spec: dict[str, Any]) -> list[float]:
        numeric_stats = dict(spec["numeric_stats"])
        values: list[float] = [1.0]
        for name in ["AoI_ms", "EffectiveFreshness", "CN_SRE_margin", "background_agvs", "bottlenecks"]:
            mean_value, std_value = numeric_stats[name]
            raw = float(record.get(name, 0.0) or 0.0)
            denom = max(float(std_value), 1e-6)
            values.append((raw - float(mean_value)) / denom)
        values.append(1.0 if str(record.get("severity", "")) == "severe" else 0.0)
        values.append(1.0 if float(record.get("stale_indicator", 0.0) or 0.0) >= 0.5 else 0.0)
        for phase in list(spec["phase_categories"])[1:]:
            values.append(1.0 if str(record.get("phase", "")) == str(phase) else 0.0)
        for execution_mode in list(spec["execution_categories"])[1:]:
            values.append(1.0 if str(record.get("DGWS_execution_mode", "")) == str(execution_mode) else 0.0)
        for action in list(spec["action_categories"])[1:]:
            values.append(1.0 if str(candidate_action) == str(action) else 0.0)
        return values

    def _predict_regression(self, model: dict[str, Any] | None, record: dict[str, Any], candidate_action: str) -> float | None:
        if not model or np is None:
            return None
        vector = np.asarray(self._row_to_feature_vector(record, candidate_action, dict(model["feature_spec"])), dtype=float)
        coef = np.asarray(model["coefficients"], dtype=float)
        return float(np.dot(vector, coef))

    def _select_hierarchical_stat(self, record: dict[str, Any], candidate_action: str) -> dict[str, Any]:
        thresholds = dict(self.policy["thresholds"])
        lookup = dict(self.policy["stats_lookup"])
        level_order = [
            ("level1", "level1_bucket", int(thresholds["level1"]), 1.0),
            ("level2", "level2_bucket", int(thresholds["level2"]), 0.75),
            ("level3", "level3_bucket", int(thresholds["level3"]), 0.55),
        ]
        collected: list[dict[str, Any]] = []
        for level_name, bucket_key_name, threshold, base_weight in level_order:
            bucket = str(record.get(bucket_key_name, ""))
            action_map = dict(lookup[level_name].get(bucket, {}))
            stats = dict(action_map.get(str(candidate_action), {}))
            if not stats:
                continue
            sample_count = int(stats["sample_count"])
            confidence = base_weight * min(1.0, sample_count / max(float(threshold), 1.0))
            collected.append(
                {
                    "level": level_name,
                    "sample_count": sample_count,
                    "confidence": confidence,
                    "mean_bytes": float(stats["mean_bytes"]),
                    "mean_cn_sre": float(stats["mean_cn_sre"]),
                    "mean_progress_delta": float(stats["mean_progress_delta"]),
                    "mean_success": float(stats["mean_success"]),
                }
            )
        global_stats = dict(self.policy["global_lookup"].get(str(candidate_action), {}))
        if global_stats:
            collected.append(
                {
                    "level": "global",
                    "sample_count": int(global_stats["sample_count"]),
                    "confidence": 0.25 * min(1.0, int(global_stats["sample_count"]) / max(float(thresholds["level1"]), 1.0)),
                    "mean_bytes": float(global_stats["mean_bytes"]),
                    "mean_cn_sre": float(global_stats["mean_cn_sre"]),
                    "mean_progress_delta": float(global_stats["mean_progress_delta"]),
                    "mean_success": float(global_stats["mean_success"]),
                }
            )
        if not collected:
            return {
                "support_level": "none",
                "support_confidence": 0.0,
                "sample_count": 0,
                "mean_bytes": float("nan"),
                "mean_cn_sre": float("nan"),
                "mean_progress_delta": float("nan"),
                "mean_success": float("nan"),
                "regression_blend_weight": 1.0,
            }
        top_level = str(collected[0]["level"])
        support_confidence = min(1.0, sum(float(item["confidence"]) for item in collected))
        weight_sum = sum(max(float(item["confidence"]), 1e-6) for item in collected)
        weights = [max(float(item["confidence"]), 1e-6) / weight_sum for item in collected]
        empirical = {
            "support_level": top_level,
            "support_confidence": support_confidence,
            "sample_count": max(int(item["sample_count"]) for item in collected),
            "mean_bytes": sum(w * float(item["mean_bytes"]) for w, item in zip(weights, collected)),
            "mean_cn_sre": sum(w * float(item["mean_cn_sre"]) for w, item in zip(weights, collected)),
            "mean_progress_delta": sum(w * float(item["mean_progress_delta"]) for w, item in zip(weights, collected)),
            "mean_success": sum(w * float(item["mean_success"]) for w, item in zip(weights, collected)),
        }
        return empirical

    def _guard_unsafe_downgrade(self, record: dict[str, Any], proposed_action: str, fallback_action: str) -> tuple[str, bool]:
        phase = str(record.get("phase", ""))
        stale_indicator = float(record.get("stale_indicator", 0.0) or 0.0)
        cn_sre_margin = float(record.get("CN_SRE_margin", 1.0) or 1.0)
        current_action = str(record.get("current_transmission_mode", fallback_action))
        if phase in HIGH_RISK_PHASES and (stale_indicator >= 0.5 or cn_sre_margin <= 0.05):
            if TX_RANK.get(str(proposed_action), -1) < 2:
                return str(fallback_action), True
            if TX_RANK.get(str(proposed_action), -1) < TX_RANK.get(current_action, -1):
                return str(fallback_action), True
        return str(proposed_action), False

    def _score_counterfactual(self, record: dict[str, Any], candidate_action: str) -> dict[str, Any]:
        empirical = self._select_hierarchical_stat(record, candidate_action)
        current_cn = float(record.get("CN_SRE_step_or_component", 0.0) or 0.0)
        current_action = str(record.get("current_transmission_mode", candidate_action))
        safe_action, unsafe_flag = self._guard_unsafe_downgrade(record, proposed_action=str(candidate_action), fallback_action=current_action)
        bytes_reg = self._predict_regression(self.policy.get("bytes_regression"), record, candidate_action)
        cn_reg = self._predict_regression(self.policy.get("cn_sre_regression"), record, candidate_action)
        regression_blend_weight = 0.0
        predicted_bytes = float(empirical["mean_bytes"]) if math.isfinite(float(empirical["mean_bytes"])) else float(record.get("bytes_this_step", 0.0) or 0.0)
        predicted_cn = float(empirical["mean_cn_sre"]) if math.isfinite(float(empirical["mean_cn_sre"])) else current_cn
        if bytes_reg is not None and cn_reg is not None and float(empirical["support_confidence"]) < 0.90:
            regression_blend_weight = max(0.0, 1.0 - float(empirical["support_confidence"]))
            predicted_bytes = (1.0 - regression_blend_weight) * predicted_bytes + regression_blend_weight * float(bytes_reg)
            predicted_cn = (1.0 - regression_blend_weight) * predicted_cn + regression_blend_weight * float(cn_reg)
        return {
            "candidate_action": str(candidate_action),
            "expected_bytes": float(predicted_bytes),
            "expected_cn_sre": float(predicted_cn),
            "expected_cn_sre_increment": float(predicted_cn - current_cn),
            "expected_progress_delta": float(empirical["mean_progress_delta"]) if math.isfinite(float(empirical["mean_progress_delta"])) else 0.0,
            "expected_success": float(empirical["mean_success"]) if math.isfinite(float(empirical["mean_success"])) else 0.0,
            "unsafe_downgrade_flag": bool(unsafe_flag or str(safe_action) != str(candidate_action)),
            "support_confidence": float(empirical["support_confidence"]),
            "support_level": str(empirical["support_level"]),
            "sample_count": int(empirical["sample_count"]),
            "regression_blend_weight": float(regression_blend_weight),
        }

    def _is_high_value_state(self, record: dict[str, Any]) -> bool:
        current_action = str(record.get("current_transmission_mode", ""))
        if current_action not in {"urgent_refresh", "full_update"}:
            return False
        phase = str(record.get("phase", ""))
        if phase == "final_alignment":
            return False
        if phase in HIGH_RISK_PHASES and float(record.get("stale_indicator", 0.0) or 0.0) >= 0.5:
            return False
        if float(record.get("CN_SRE_margin", 0.0) or 0.0) <= 0.0:
            return False
        freshness_bin = str(record.get("EffectiveFreshness_bin", ""))
        aoi_bin = str(record.get("AoI_bin", ""))
        return freshness_bin == "fresh_high" or aoi_bin in {"aoi_low", "aoi_mid"}

    def _downgrade_candidates(self, current_action: str) -> list[str]:
        if current_action == "urgent_refresh":
            return ["full_update", "compact_update", "skip_update"]
        if current_action == "full_update":
            return ["compact_update", "skip_update"]
        if current_action == "compact_update":
            return ["skip_update"]
        return []

    def decide(self, record: dict[str, Any]) -> dict[str, Any]:
        current_action = str(record["current_transmission_mode"])
        output = {
            "candidate_transmission_mode": current_action,
            "used_fallback": False,
            "unsupported_context": False,
            "unsafe_downgrade_blocked": False,
            "support_level_used": "none",
            "support_confidence": 1.0,
            "policy_reason": "no_correction_not_high_value",
            "high_value_state": False,
            "predicted_candidate_bytes": float(record.get("bytes_this_step", 0.0) or 0.0),
            "predicted_candidate_cn_sre": float(record.get("CN_SRE_step_or_component", 0.0) or 0.0),
            "predicted_candidate_cn_sre_increment": 0.0,
            "regression_blend_weight": 0.0,
        }
        if not self._is_high_value_state(record):
            if str(record.get("phase", "")) in HIGH_RISK_PHASES and float(record.get("stale_indicator", 0.0) or 0.0) >= 0.5:
                output["unsafe_downgrade_blocked"] = True
                output["policy_reason"] = "blocked_unsafe_high_risk_state"
            return output
        output["high_value_state"] = True
        current_progress = float(record.get("progress_delta", 0.0) or 0.0)
        current_success = float(record.get("success", 0.0) or 0.0)
        cn_tolerance = min(max(float(record.get("CN_SRE_margin", 0.0) or 0.0), 0.0), 0.02)
        had_any_support = False
        for candidate_action in self._downgrade_candidates(current_action):
            score = self._score_counterfactual(record, candidate_action)
            if score["support_level"] != "none":
                had_any_support = True
            if float(score["support_confidence"]) < 0.02:
                continue
            if bool(score["unsafe_downgrade_flag"]):
                output["unsafe_downgrade_blocked"] = True
                continue
            if float(score["expected_cn_sre_increment"]) > cn_tolerance + 1e-9:
                continue
            if float(score["expected_progress_delta"]) < current_progress - 0.010:
                continue
            if float(score["expected_success"]) < current_success - 0.02:
                continue
            output.update(
                {
                    "candidate_transmission_mode": str(candidate_action),
                    "used_fallback": False,
                    "unsupported_context": False,
                    "support_level_used": str(score["support_level"]),
                    "support_confidence": float(score["support_confidence"]),
                    "policy_reason": "hierarchical_support_aware_high_value_correction",
                    "predicted_candidate_bytes": float(score["expected_bytes"]),
                    "predicted_candidate_cn_sre": float(score["expected_cn_sre"]),
                    "predicted_candidate_cn_sre_increment": float(score["expected_cn_sre_increment"]),
                    "regression_blend_weight": float(score["regression_blend_weight"]),
                }
            )
            return output
        output["used_fallback"] = not had_any_support
        output["unsupported_context"] = not had_any_support
        output["support_level_used"] = "dgws_fallback" if not had_any_support else "no_safe_correction"
        output["support_confidence"] = 0.0 if not had_any_support else 1.0
        output["policy_reason"] = "fallback_to_dgws_no_support" if not had_any_support else "no_safe_correction_under_cn_sre_guard"
        return output

    def build_state_policy_record(self, state_row: dict[str, Any]) -> dict[str, Any]:
        binning = dict(self.policy["binning"])
        phase = str(state_row.get("phase", "standby"))
        aoi_ms = float(state_row.get("AoI_ms", 0.0) or 0.0)
        effective_freshness = float(state_row.get("EffectiveFreshness_num", 0.0) or 0.0)
        cn_sre_margin = float(state_row.get("CN_SRE_margin", 0.0) or 0.0)
        stale_indicator = float(state_row.get("stale_indicator", 0.0) or 0.0)
        exec_mode = str(state_row.get("DGWS_execution_mode", "normal_execute") or "normal_execute")
        enriched = dict(state_row)
        enriched["AoI_bin"] = self._label_from_edges(aoi_ms, list(binning["AoI_ms_edges"]), list(binning["AoI_ms_labels"]))
        enriched["EffectiveFreshness_bin"] = self._label_from_edges(
            effective_freshness,
            list(binning["EffectiveFreshness_edges"]),
            list(binning["EffectiveFreshness_labels"]),
        )
        enriched["CN_SRE_margin_bin"] = self._label_from_edges(
            cn_sre_margin,
            list(binning["CN_SRE_margin_edges"]),
            list(binning["CN_SRE_margin_labels"]),
        )
        enriched["level1_bucket"] = _bucket_key(
            phase,
            enriched["AoI_bin"],
            enriched["EffectiveFreshness_bin"],
            enriched["CN_SRE_margin_bin"],
            int(stale_indicator >= 0.5),
            exec_mode,
        )
        enriched["level2_bucket"] = "|".join([phase, enriched["EffectiveFreshness_bin"], str(int(stale_indicator >= 0.5)), exec_mode])
        enriched["level3_bucket"] = "|".join([phase, str(int(stale_indicator >= 0.5))])
        return enriched


@dataclass
class TopicSpec:
    topic_name: str
    msg_type: str
    qos: dict[str, Any]


class CaptureWriter:
    def __init__(self, path: str | Path, fields: list[str]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fp, fieldnames=fields)
        self._writer.writeheader()

    def write_row(self, row: dict[str, Any]) -> None:
        self._writer.writerow({field: row.get(field, "") for field in self._writer.fieldnames or []})

    def close(self) -> None:
        self._fp.flush()
        self._fp.close()


class TaskAwareShadowObserverNode(Node):
    def __init__(self, config: dict[str, Any], engine: RepairedPolicyEngine, *, shutdown_after_sec: float | None = None):
        super().__init__(str(config["ros"]["node_name"]))
        self.config = config
        self.engine = engine
        self.shutdown_after_sec = shutdown_after_sec
        self.started_wall = time.time()
        self.subscriber_ready_at: str | None = None
        self.capture_writer = CaptureWriter(config["output"]["capture_csv_path"], CAPTURE_FIELDS)
        self.state_writer = CaptureWriter(config["output"]["state_rows_csv_path"], STATE_FIELDS)
        self.action_writer = CaptureWriter(config["output"]["action_csv_path"], ACTION_FIELDS)
        self.capture_rows = 0
        self.state_rows = 0
        self.action_rows = 0
        self.invalid_state_count = 0
        self.fallback_count = 0
        self.phase_valid_rows = 0
        self.task_valid_rows = 0
        self.ef_non_null_rows = 0
        self.cn_non_null_rows = 0
        self.missing_field_count = 0
        self.action_counter: Counter[str] = Counter()
        self.callback_latency_ms: list[float] = []
        self.inference_latency_ms: list[float] = []
        self.latest_phase: dict[str, Any] | None = None
        self.latest_observations: dict[str, dict[str, Any]] = {}
        self.previous_receive_ns: dict[str, int] = {}
        self._finalized = False

        self.topic_specs: list[TopicSpec] = []
        for topic_cfg in list(config.get("topics", [])):
            if not bool(topic_cfg.get("enabled", True)):
                continue
            self.topic_specs.append(
                TopicSpec(
                    topic_name=str(topic_cfg["topic_name"]),
                    msg_type=str(topic_cfg["msg_type"]),
                    qos=dict(topic_cfg["qos"]),
                )
            )
        for topic_spec in self.topic_specs:
            message_class = _resolve_message_class(topic_spec.msg_type)
            self.create_subscription(message_class, topic_spec.topic_name, self._build_callback(topic_spec), _qos_profile_from_dict(topic_spec.qos))
        self.subscriber_ready_at = _now_iso()
        self.get_logger().info(f"subscriber_ready timestamp={self.subscriber_ready_at} subscriptions={len(self.topic_specs)}")
        if self.shutdown_after_sec is not None:
            self.create_timer(0.5, self._check_shutdown)

    def _check_shutdown(self) -> None:
        if self.shutdown_after_sec is None or self._finalized:
            return
        if (time.time() - self.started_wall) >= float(self.shutdown_after_sec):
            self.get_logger().info("Shadow observer duration reached; shutting down.")
            self.finalize()
            if rclpy.ok():
                rclpy.shutdown()
            os._exit(0)

    def _build_callback(self, topic_spec: TopicSpec):
        def _callback(msg: Any) -> None:
            started = time.perf_counter()
            self._handle_message(topic_spec, msg)
            self.callback_latency_ms.append((time.perf_counter() - started) * 1000.0)

        return _callback

    def _extract_capture_fields(self, topic_spec: TopicSpec, msg: Any, receive_wall_ns: int) -> dict[str, Any]:
        topic = topic_spec.topic_name
        warning_flags: list[str] = []
        source_timestamp_ns: int | None = None
        parse_ok = True
        phase = ""
        task_progress: float | None = None
        payload_summary = ""
        msg_size = _serialize_size(msg)
        if topic == "/fr_validation/derived_phase_status":
            payload = payload_from_string_message(msg)
            parse_ok = bool(payload)
            if not payload:
                warning_flags.append("phase_payload_parse_failed")
            phase = str(payload.get("task_phase", "")) if payload else ""
            if payload and payload.get("task_progress") is not None:
                task_progress = float(payload["task_progress"])
            source_timestamp_ns = int(payload.get("sender_timestamp")) if payload and payload.get("sender_timestamp") is not None else None
            payload_summary = f"phase={phase};task_progress={task_progress if task_progress is not None else ''};source={payload.get('source_mode','') if payload else ''}"
        else:
            source_timestamp_ns = _message_stamp_ns(msg)
            if topic.endswith("/precision_mode"):
                payload_summary = f"precision_mode={getattr(msg, 'data', '')}"
            elif topic.endswith("/cmd_resume"):
                payload_summary = f"cmd_resume={getattr(msg, 'data', '')}"
            elif topic.endswith("/cmd_goal"):
                payload_summary = f"cmd_goal_bytes={msg_size}"
            elif topic.endswith("/cmd_vel_stamped"):
                payload_summary = f"cmd_vel_stamped_bytes={msg_size}"
            elif topic.endswith("/object_position"):
                payload_summary = f"object_position_bytes={msg_size}"
            else:
                payload_summary = f"msg_size_bytes={msg_size}"

        previous_ns = self.previous_receive_ns.get(topic)
        inter_arrival_ms = None if previous_ns is None else max((receive_wall_ns - previous_ns) / 1_000_000.0, 0.0)
        self.previous_receive_ns[topic] = receive_wall_ns

        aoi_ms = None
        ef = None
        if source_timestamp_ns is not None:
            aoi_ms = max((receive_wall_ns - int(source_timestamp_ns)) / 1_000_000.0, 0.0)
            ef = math.exp(-float(aoi_ms) / 1000.0)

        if source_timestamp_ns is None and topic.endswith("/cmd_vel_stamped"):
            warning_flags.append("sender_timestamp_missing")
        if not topic.endswith("/cmd_vel_stamped") and source_timestamp_ns is None and topic != "/fr_validation/derived_phase_status":
            warning_flags.append("no_freshness_computation")

        row = {
            "topic": topic,
            "receive_time": datetime.fromtimestamp(receive_wall_ns / 1_000_000_000.0, tz=timezone.utc).astimezone().isoformat(),
            "source_timestamp": str(source_timestamp_ns or ""),
            "parse_ok": bool(parse_ok),
            "qos_profile_used": _qos_summary(topic_spec.qos),
            "payload_summary": payload_summary,
            "nonempty": bool(msg_size > 0),
            "robot_id": _robot_from_topic(topic),
            "phase": phase,
            "task_progress": "" if task_progress is None else _format_float(task_progress),
            "peer_id": _peer_from_topic(topic),
            "msg_size_bytes": int(msg_size),
            "inter_arrival_ms": _format_float(inter_arrival_ms),
            "AoI_ms": _format_float(aoi_ms),
            "Effective_Freshness": _format_float(ef),
            "warning_flags": "|".join(sorted(set(warning_flags))),
        }
        self.latest_observations[topic] = {
            "topic": topic,
            "receive_wall_ns": receive_wall_ns,
            "source_timestamp_ns": source_timestamp_ns,
            "inter_arrival_ms": inter_arrival_ms,
            "aoi_ms": aoi_ms,
            "effective_freshness": ef,
            "msg_size_bytes": msg_size,
            "phase": phase,
            "task_progress": task_progress,
            "warning_flags": list(warning_flags),
            "row": row,
        }
        return row

    def _pick_observation(self) -> dict[str, Any] | None:
        priorities = [
            "/tracer1/cmd_vel_stamped",
            "/tracer2/cmd_vel_stamped",
            "/tracer3/cmd_vel_stamped",
            "/tracer1/cmd_goal",
            "/tracer2/cmd_goal",
            "/tracer3/cmd_goal",
            "/tracer1/cmd_resume",
            "/tracer2/cmd_resume",
            "/tracer3/cmd_resume",
            "/tracer1/precision_mode",
            "/tracer2/precision_mode",
            "/tracer3/precision_mode",
        ]
        candidates = [self.latest_observations[name] for name in priorities if name in self.latest_observations]
        if not candidates:
            return None
        return max(candidates, key=lambda item: int(item["receive_wall_ns"]))

    def _build_state_row(self, receive_wall_ns: int) -> dict[str, Any]:
        phase_payload = dict(self.latest_phase or {})
        phase = str(phase_payload.get("task_phase", ""))
        phase_present = bool(phase)
        task_progress_raw = phase_payload.get("task_progress")
        task_present = task_progress_raw is not None
        task_progress = float(task_progress_raw) if task_present else float("nan")
        obs = self._pick_observation()
        missing_fields: list[str] = []
        fallback_reason = ""
        app_topic = "not_live"
        app_nonempty = "not_live"
        message_age_ms = float("nan")
        inter_arrival_ms = float("nan")
        aoi_ms = float("nan")
        effective_freshness = float("nan")
        cn_sre_candidate = float("nan")
        aoi_source = "missing"
        ef_source = "missing"
        cn_source = "missing"
        robot_id = "fleet"
        bytes_this_step = 0.0
        communication_available = obs is not None
        pose_context_available = False
        status_context_available = False

        if not phase_present:
            missing_fields.append("phase_missing")
        if not task_present:
            missing_fields.append("task_progress_missing")
        if obs is None:
            missing_fields.append("app_channel_not_live")
            fallback_reason = "missing_app_channel"
        else:
            app_topic = str(obs["topic"])
            app_nonempty = "yes" if bool(obs["msg_size_bytes"] > 0) else "no"
            robot_id = _robot_from_topic(app_topic)
            bytes_this_step = float(obs["msg_size_bytes"])
            message_age_ms = max((receive_wall_ns - int(obs["receive_wall_ns"])) / 1_000_000.0, 0.0)
            if obs["inter_arrival_ms"] is not None:
                inter_arrival_ms = float(obs["inter_arrival_ms"])
            if obs["aoi_ms"] is not None:
                aoi_ms = float(obs["aoi_ms"])
                aoi_source = "app_channel"
            else:
                missing_fields.append("AoI_missing")
            if obs["effective_freshness"] is not None:
                effective_freshness = float(obs["effective_freshness"])
                ef_source = "app_channel"
            elif not math.isnan(aoi_ms):
                effective_freshness = math.exp(-float(aoi_ms) / 1000.0)
                ef_source = "exp_from_aoi"
            else:
                missing_fields.append("EffectiveFreshness_missing")
            if not math.isnan(effective_freshness):
                cn_sre_candidate = max(0.0, min(1.0, 1.0 - float(effective_freshness)))
                cn_source = "one_minus_effective_freshness"
            else:
                missing_fields.append("CN_SRE_missing")
            if obs["source_timestamp_ns"] is None:
                missing_fields.append("no_freshness_computation")
                fallback_reason = fallback_reason or "no_freshness_computation"

        state_valid = phase_present and task_present and obs is not None and not math.isnan(aoi_ms) and not math.isnan(effective_freshness) and not math.isnan(cn_sre_candidate)
        if not state_valid and not fallback_reason:
            fallback_reason = "invalid_or_degraded_state"

        current_tx_mode = ""
        dgws_execution_mode = ""
        execution_mode = ""
        stale_indicator = float("nan")
        risk_score = float("nan")
        cn_sre_margin = float("nan")
        if state_valid:
            stale_indicator = 1.0 if float(cn_sre_candidate) >= 0.5 else 0.0
            risk_score = float(cn_sre_candidate)
            cn_sre_margin = float(cn_sre_candidate)
            current_tx_mode = _dgws_reference_mode(phase, float(effective_freshness), float(aoi_ms), float(stale_indicator))
            dgws_execution_mode = "normal_execute"
            execution_mode = "normal_execute"

        if not pose_context_available:
            missing_fields.append("pose_context_missing")
        if not status_context_available:
            missing_fields.append("status_context_missing")

        state_row = {
            "state_row_id": self.state_rows,
            "receive_time": datetime.fromtimestamp(receive_wall_ns / 1_000_000_000.0, tz=timezone.utc).astimezone().isoformat(),
            "robot_id": robot_id,
            "phase": phase if phase_present else "",
            "task_progress": "" if not task_present else _format_float(task_progress),
            "app_channel_topic": app_topic,
            "app_channel_nonempty": app_nonempty,
            "message_age": "" if math.isnan(message_age_ms) else _format_float(message_age_ms),
            "inter_arrival": "" if math.isnan(inter_arrival_ms) else _format_float(inter_arrival_ms),
            "AoI": "" if math.isnan(aoi_ms) else _format_float(aoi_ms),
            "EffectiveFreshness": "" if math.isnan(effective_freshness) else _format_float(effective_freshness),
            "CN_SRE_candidate": "" if math.isnan(cn_sre_candidate) else _format_float(cn_sre_candidate),
            "pose_context_available": pose_context_available,
            "status_context_available": status_context_available,
            "communication_observation_available": communication_available,
            "missing_field_list": "|".join(sorted(set(missing_fields))),
            "state_schema_valid": "yes" if state_valid else "no",
            "aoi_source": aoi_source,
            "effective_freshness_source": ef_source,
            "cn_sre_source": cn_source,
            "episode_id": f"tj2b_live_{self.state_rows:06d}",
            "AoI_ms": "" if math.isnan(aoi_ms) else _format_float(aoi_ms),
            "EffectiveFreshness_num": "" if math.isnan(effective_freshness) else _format_float(effective_freshness),
            "CN_SRE_margin": "" if math.isnan(cn_sre_margin) else _format_float(cn_sre_margin),
            "CN_SRE_step_or_component": "" if math.isnan(cn_sre_candidate) else _format_float(cn_sre_candidate),
            "stale_indicator": "" if math.isnan(stale_indicator) else _format_float(stale_indicator),
            "risk_score": "" if math.isnan(risk_score) else _format_float(risk_score),
            "phase_progress": "" if not task_present else _format_float(_phase_progress_from_task_progress(phase, task_progress)),
            "execution_mode": execution_mode,
            "DGWS_execution_mode": dgws_execution_mode,
            "current_transmission_mode": current_tx_mode,
            "transmission_mode": current_tx_mode,
            "bytes_this_step": _format_float(bytes_this_step),
            "background_agvs": _format_float(float(self.config["shadow_context"]["background_agvs"])),
            "bottlenecks": _format_float(float(self.config["shadow_context"]["bottlenecks"])),
            "severity": str(self.config["shadow_context"]["severity"]),
            "policy_name": "FR-TPO-live-shadow",
            "source_role": "record_only_shadow",
            "exposure_split": "live_shadow",
            "fallback_reason": fallback_reason,
            "invalid_state": (not state_valid),
        }
        self.missing_field_count += 1 if state_row["missing_field_list"] else 0
        if state_valid:
            self.phase_valid_rows += 1
            self.task_valid_rows += 1
            self.ef_non_null_rows += 1
            self.cn_non_null_rows += 1
        else:
            self.invalid_state_count += 1
        return state_row

    def _build_action_row(self, state_row: dict[str, Any]) -> dict[str, Any]:
        started = time.perf_counter()
        state_valid = str(state_row["state_schema_valid"]) == "yes"
        if not state_valid:
            decision = {
                "candidate_transmission_mode": "skip_update",
                "support_confidence": "",
                "policy_reason": str(state_row["fallback_reason"] or "invalid_or_degraded_state"),
                "used_fallback": True,
                "unsupported_context": True,
                "unsafe_downgrade_blocked": False,
            }
        else:
            record = self.engine.build_state_policy_record(
                {
                    "phase": state_row["phase"],
                    "task_progress": float(state_row["task_progress"]),
                    "AoI_ms": float(state_row["AoI_ms"]),
                    "EffectiveFreshness": float(state_row["EffectiveFreshness_num"]),
                    "CN_SRE_margin": float(state_row["CN_SRE_margin"]),
                    "CN_SRE_step_or_component": float(state_row["CN_SRE_step_or_component"]),
                    "stale_indicator": float(state_row["stale_indicator"]),
                    "risk_score": float(state_row["risk_score"]),
                    "DGWS_execution_mode": state_row["DGWS_execution_mode"],
                    "execution_mode": state_row["execution_mode"],
                    "current_transmission_mode": state_row["current_transmission_mode"],
                    "bytes_this_step": float(state_row["bytes_this_step"]),
                    "background_agvs": float(state_row["background_agvs"]),
                    "severity": state_row["severity"],
                    "bottlenecks": float(state_row["bottlenecks"]),
                    "progress_delta": float(self.config["shadow_context"].get("progress_delta_proxy", 0.0)),
                    "success": float(self.config["shadow_context"].get("success_proxy", 0.0)),
                }
            )
            decision = self.engine.decide(record)
        self.inference_latency_ms.append((time.perf_counter() - started) * 1000.0)
        self.action_counter[str(decision["candidate_transmission_mode"])] += 1
        if bool(decision.get("used_fallback", False)) or str(decision.get("policy_reason", "")).startswith("fallback"):
            self.fallback_count += 1
        action_row = {
            "state_row_id": state_row["state_row_id"],
            "timestamp": state_row["receive_time"],
            "phase": state_row["phase"],
            "task_progress": state_row["task_progress"],
            "freshness_fields": f"AoI_ms={state_row['AoI_ms']};EffectiveFreshness={state_row['EffectiveFreshness_num']};CN_SRE_candidate={state_row['CN_SRE_step_or_component']}",
            "policy_loaded": "yes" if self.engine.policy_loaded else "no",
            "action_candidate": decision["candidate_transmission_mode"],
            "action_confidence": "" if decision.get("support_confidence", "") == "" else _format_float(float(decision["support_confidence"])),
            "fallback_reason": decision.get("policy_reason", state_row["fallback_reason"]),
            "not_applied_to_system": True,
            "used_fallback": bool(decision.get("used_fallback", False)),
            "unsupported_context": bool(decision.get("unsupported_context", False)),
            "current_transmission_mode": state_row["current_transmission_mode"],
            "dgws_reference_action": state_row["current_transmission_mode"],
            "blocked_unsafe_downgrade": bool(decision.get("unsafe_downgrade_blocked", False)),
            "policy_binding_mode": self.engine.binding_mode,
        }
        return action_row

    def _handle_message(self, topic_spec: TopicSpec, msg: Any) -> None:
        receive_wall_ns = time.time_ns()
        row = self._extract_capture_fields(topic_spec, msg, receive_wall_ns)
        self.capture_writer.write_row(row)
        self.capture_rows += 1
        if topic_spec.topic_name == "/fr_validation/derived_phase_status":
            payload = payload_from_string_message(msg)
            self.latest_phase = payload if payload else None
            state_row = self._build_state_row(receive_wall_ns)
            self.state_writer.write_row(state_row)
            self.state_rows += 1
            action_row = self._build_action_row(state_row)
            self.action_writer.write_row(action_row)
            self.action_rows += 1

    def finalize(self) -> dict[str, Any]:
        if self._finalized:
            return {}
        self._finalized = True
        self.capture_writer.close()
        self.state_writer.close()
        self.action_writer.close()
        summary = {
            "subscriber_ready_at": self.subscriber_ready_at,
            "capture_rows": int(self.capture_rows),
            "state_rows": int(self.state_rows),
            "action_rows": int(self.action_rows),
            "action_distribution": dict(self.action_counter),
            "fallback_count": int(self.fallback_count),
            "invalid_state_count": int(self.invalid_state_count),
            "missing_field_count": int(self.missing_field_count),
            "phase_valid_rate": float(self.phase_valid_rows / self.state_rows) if self.state_rows else 0.0,
            "task_progress_valid_rate": float(self.task_valid_rows / self.state_rows) if self.state_rows else 0.0,
            "effective_freshness_non_null_rate": float(self.ef_non_null_rows / self.state_rows) if self.state_rows else 0.0,
            "cn_sre_non_null_rate": float(self.cn_non_null_rows / self.state_rows) if self.state_rows else 0.0,
            "callback_latency_ms": self._latency_stats(self.callback_latency_ms),
            "inference_latency_ms": self._latency_stats(self.inference_latency_ms),
            "policy_loaded": bool(self.engine.policy_loaded),
            "policy_binding_mode": self.engine.binding_mode,
            "all_actions_not_applied_to_system": True,
        }
        Path(self.config["output"]["summary_json_path"]).write_text(json.dumps(summary, indent=2), encoding="utf-8")
        return summary

    @staticmethod
    def _latency_stats(values: list[float]) -> dict[str, float]:
        if not values:
            return {"mean": 0.0, "p50": 0.0, "p95": 0.0, "max": 0.0}
        ordered = sorted(float(v) for v in values)
        def _quantile(q: float) -> float:
            idx = min(max(int(round((len(ordered) - 1) * q)), 0), len(ordered) - 1)
            return float(ordered[idx])
        return {
            "mean": float(sum(ordered) / len(ordered)),
            "p50": _quantile(0.50),
            "p95": _quantile(0.95),
            "max": float(ordered[-1]),
        }


def run_offline_replay(config: dict[str, Any], engine: RepairedPolicyEngine, state_rows_path: str | Path, output_csv_path: str | Path) -> dict[str, Any]:
    frame = list(csv.DictReader(Path(state_rows_path).open("r", encoding="utf-8")))
    writer = CaptureWriter(output_csv_path, ACTION_FIELDS)
    counter: Counter[str] = Counter()
    fallback_count = 0
    for row in frame:
        state_valid = str(row.get("state_schema_valid", "no")) == "yes"
        current_tx_mode = str(row.get("current_transmission_mode", "") or "")
        if state_valid and current_tx_mode in {"", "skip_update"}:
            current_tx_mode = _dgws_reference_mode(
                str(row["phase"]),
                float(row["EffectiveFreshness_num"]),
                float(row["AoI_ms"]),
                float(row["stale_indicator"]),
            )
        if not state_valid:
            decision = {
                "candidate_transmission_mode": "skip_update",
                "support_confidence": "",
                "policy_reason": str(row.get("fallback_reason", "") or "invalid_or_degraded_state"),
                "used_fallback": True,
                "unsupported_context": True,
                "unsafe_downgrade_blocked": False,
            }
        else:
            record = engine.build_state_policy_record(
                {
                    "phase": row["phase"],
                    "task_progress": float(row["task_progress"]),
                    "AoI_ms": float(row["AoI_ms"]),
                    "EffectiveFreshness": float(row["EffectiveFreshness_num"]),
                    "CN_SRE_margin": float(row["CN_SRE_margin"]),
                    "CN_SRE_step_or_component": float(row["CN_SRE_step_or_component"]),
                    "stale_indicator": float(row["stale_indicator"]),
                    "risk_score": float(row["risk_score"]),
                    "DGWS_execution_mode": row.get("DGWS_execution_mode", "normal_execute") or "normal_execute",
                    "execution_mode": row.get("execution_mode", "normal_execute") or "normal_execute",
                    "current_transmission_mode": current_tx_mode or "full_update",
                    "bytes_this_step": float(row.get("bytes_this_step", 0.0) or 0.0),
                    "background_agvs": float(row.get("background_agvs", config["shadow_context"]["background_agvs"]) or 0.0),
                    "severity": row.get("severity", config["shadow_context"]["severity"]),
                    "bottlenecks": float(row.get("bottlenecks", config["shadow_context"]["bottlenecks"]) or 0.0),
                    "progress_delta": float(config["shadow_context"].get("progress_delta_proxy", 0.0)),
                    "success": float(config["shadow_context"].get("success_proxy", 0.0)),
                }
            )
            decision = engine.decide(record)
        counter[str(decision["candidate_transmission_mode"])] += 1
        if bool(decision.get("used_fallback", False)) or str(decision.get("policy_reason", "")).startswith("fallback"):
            fallback_count += 1
        writer.write_row(
            {
                "state_row_id": row["state_row_id"],
                "timestamp": row["receive_time"],
                "phase": row["phase"],
                "task_progress": row["task_progress"],
                "freshness_fields": f"AoI_ms={row['AoI_ms']};EffectiveFreshness={row['EffectiveFreshness_num']};CN_SRE_candidate={row['CN_SRE_step_or_component']}",
                "policy_loaded": "yes" if engine.policy_loaded else "no",
                "action_candidate": decision["candidate_transmission_mode"],
                "action_confidence": "" if decision.get("support_confidence", "") == "" else _format_float(float(decision["support_confidence"])),
                "fallback_reason": decision.get("policy_reason", row.get("fallback_reason", "")),
                "not_applied_to_system": True,
                "used_fallback": bool(decision.get("used_fallback", False)),
                "unsupported_context": bool(decision.get("unsupported_context", False)),
                "current_transmission_mode": current_tx_mode,
                "dgws_reference_action": current_tx_mode,
                "blocked_unsafe_downgrade": bool(decision.get("unsafe_downgrade_blocked", False)),
                "policy_binding_mode": engine.binding_mode,
            }
        )
    writer.close()
    return {
        "rows": len(frame),
        "action_distribution": dict(counter),
        "fallback_count": int(fallback_count),
        "policy_loaded": bool(engine.policy_loaded),
    }


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--offline-state-rows", default=None)
    parser.add_argument("--offline-output-csv", default=None)
    parser.add_argument("--shutdown-after-sec", type=float, default=None)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_argument_parser().parse_args(argv)
    config = _load_yaml(args.config)
    engine = RepairedPolicyEngine(config["policy_path"])
    if args.offline_state_rows:
        output_csv = args.offline_output_csv or config["output"]["action_csv_path"]
        result = run_offline_replay(config, engine, args.offline_state_rows, output_csv)
        print(json.dumps(result, indent=2))
        return

    if rclpy is None:
        raise RuntimeError("rclpy is required for live shadow mode")

    os.environ.setdefault("ROS_DOMAIN_ID", str(config["ros"]["domain_id"]))
    rclpy.init(args=[])
    node = TaskAwareShadowObserverNode(config, engine, shutdown_after_sec=args.shutdown_after_sec)

    def _handle_signal(_signum, _frame):
        node.finalize()
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)
    try:
        rclpy.spin(node)
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
