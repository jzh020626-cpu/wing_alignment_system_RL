from __future__ import annotations

import random
from dataclasses import dataclass


CRITICAL_PHASES = {"narrow_passage", "final_alignment", "slide_align", "level_recenter"}


@dataclass(frozen=True)
class MessageEnvelope:
    robot_id: str
    seq_id: int
    sender_timestamp: int
    payload_bytes: int
    task_phase: str
    task_progress: float
    scenario_id: str
    method_id: str
    transmission_mode: str = "full_update"
    execution_mode: str = "normal"
    aoi_ms: float | None = None
    effective_freshness: float | None = None


@dataclass(frozen=True)
class ForwardPlan:
    forward_count: int
    retry_count: int
    delivery_expected: bool
    delay_ms: float
    metadata: dict


class ChannelWrapperEngine:
    def __init__(
        self,
        *,
        scenario_id: str,
        wrapper_mode: str,
        delay_ms_mean: float,
        jitter_ms: float,
        loss_rate: float,
        burst_loss_rate: float,
        duplicate_on_critical: bool,
        rng: random.Random | None = None,
    ) -> None:
        self.scenario_id = str(scenario_id)
        self.wrapper_mode = str(wrapper_mode)
        self.delay_ms_mean = max(0.0, float(delay_ms_mean))
        self.jitter_ms = max(0.0, float(jitter_ms))
        self.loss_rate = max(0.0, min(1.0, float(loss_rate)))
        self.burst_loss_rate = max(0.0, min(1.0, float(burst_loss_rate)))
        self.duplicate_on_critical = bool(duplicate_on_critical)
        self.rng = rng or random.Random()

    def build_plan(self, envelope: MessageEnvelope) -> ForwardPlan:
        if self.wrapper_mode == "observe":
            return ForwardPlan(
                forward_count=1,
                retry_count=0,
                delivery_expected=True,
                delay_ms=0.0,
                metadata=self._metadata(envelope, retry_count=0, delivery_expected=True),
            )

        drop_draw = self.rng.random()
        drop_threshold = max(self.loss_rate, self.burst_loss_rate)
        delivery_expected = drop_draw >= drop_threshold
        forward_count = 0 if not delivery_expected else 1
        retry_count = 1 if (not delivery_expected or self.duplicate_on_critical) else 0

        if delivery_expected and self.duplicate_on_critical and str(envelope.task_phase) in CRITICAL_PHASES:
            forward_count = 2

        jitter_offset = self.rng.uniform(-self.jitter_ms, self.jitter_ms) if self.jitter_ms > 0.0 else 0.0
        delay_ms = max(0.0, self.delay_ms_mean + jitter_offset)
        return ForwardPlan(
            forward_count=forward_count,
            retry_count=retry_count,
            delivery_expected=delivery_expected,
            delay_ms=delay_ms,
            metadata=self._metadata(envelope, retry_count=retry_count, delivery_expected=delivery_expected),
        )

    def _metadata(self, envelope: MessageEnvelope, *, retry_count: int, delivery_expected: bool) -> dict:
        return {
            "seq_id": int(envelope.seq_id),
            "sender_timestamp": int(envelope.sender_timestamp),
            "robot_id": str(envelope.robot_id),
            "task_phase": str(envelope.task_phase),
            "task_progress": float(envelope.task_progress),
            "transmission_mode": str(envelope.transmission_mode),
            "execution_mode": str(envelope.execution_mode),
            "aoi_ms": "" if envelope.aoi_ms is None else float(envelope.aoi_ms),
            "effective_freshness": "" if envelope.effective_freshness is None else float(envelope.effective_freshness),
            "retry_count": int(retry_count),
            "payload_bytes": int(envelope.payload_bytes),
            "source_mode": "wrapped_cmd_vel_stamped",
            "scenario_id": str(self.scenario_id),
            "method_id": str(envelope.method_id),
            "delivery_expected": bool(delivery_expected),
            "wrapper_mode": str(self.wrapper_mode),
        }
