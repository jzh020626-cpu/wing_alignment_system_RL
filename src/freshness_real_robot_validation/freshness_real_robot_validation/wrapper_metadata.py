from __future__ import annotations

import csv
from pathlib import Path


NA = "n/a"
TRACE_FIELDS = [
    "seq_id",
    "seq_monotonic",
    "payload_type",
    "payload_bytes",
    "source_send_timestamp",
    "source_clock_type",
    "wrapper_receive_timestamp",
    "receiver_clock_type",
    "receiver_node_time_ns",
    "wrapper_emit_timestamp",
    "inter_arrival_ms",
    "receiver_side_aoi_proxy_ms",
    "true_one_way_delay_ms",
    "delay_semantics",
    "deadline_met",
    "retry_count",
    "transmission_mode",
    "execution_mode",
    "aoi_ms",
    "effective_freshness",
    "phase",
    "task_progress",
    "scenario_id",
    "method_id",
    "robot_id",
    "source_mode",
    "delivery_expected",
    "wrapper_mode",
]


def _ns_to_ms(delta_ns: int | None) -> float | str:
    if delta_ns is None:
        return NA
    return round(max(0, int(delta_ns)) / 1_000_000.0, 3)


class WrapperMetadataTracker:
    def __init__(self, *, receiver_clock_type: str) -> None:
        self.receiver_clock_type = str(receiver_clock_type)
        self._last_receive_timestamp_ns: int | None = None
        self._last_seq_id: int | None = None

    def build_row(
        self,
        *,
        seq_id: int,
        payload_type: str,
        payload_bytes: int,
        source_send_timestamp_ns: int | None,
        source_clock_type: str,
        wrapper_receive_timestamp_ns: int,
        transmission_mode: str,
        phase: str,
        task_progress: float,
        retry_count: int,
        execution_mode: str,
        aoi_ms: float | None,
        effective_freshness: float | None,
        deadline_met: str | bool,
        scenario_id: str,
        method_id: str,
        robot_id: str,
        source_mode: str,
        delivery_expected: bool,
        wrapper_mode: str,
        wrapper_emit_timestamp_ns: int | None,
        allow_true_one_way_delay: bool,
    ) -> dict:
        receive_ns = int(wrapper_receive_timestamp_ns)
        inter_arrival_ns = None
        if self._last_receive_timestamp_ns is not None:
            inter_arrival_ns = receive_ns - self._last_receive_timestamp_ns

        seq_monotonic = self._last_seq_id is None or int(seq_id) > int(self._last_seq_id)
        inter_arrival_ms = _ns_to_ms(inter_arrival_ns)

        if source_send_timestamp_ns is not None:
            receiver_side_aoi_proxy_ms = _ns_to_ms(receive_ns - int(source_send_timestamp_ns))
        else:
            receiver_side_aoi_proxy_ms = inter_arrival_ms

        true_one_way_delay_ms: float | str = NA
        if (
            allow_true_one_way_delay
            and source_send_timestamp_ns is not None
            and str(source_clock_type) == str(self.receiver_clock_type)
        ):
            true_one_way_delay_ms = _ns_to_ms(receive_ns - int(source_send_timestamp_ns))

        row = {
            "seq_id": int(seq_id),
            "seq_monotonic": bool(seq_monotonic),
            "payload_type": str(payload_type),
            "payload_bytes": int(payload_bytes),
            "source_send_timestamp": int(source_send_timestamp_ns) if source_send_timestamp_ns is not None else NA,
            "source_clock_type": str(source_clock_type or NA),
            "wrapper_receive_timestamp": receive_ns,
            "receiver_clock_type": self.receiver_clock_type,
            "receiver_node_time_ns": receive_ns,
            "wrapper_emit_timestamp": int(wrapper_emit_timestamp_ns) if wrapper_emit_timestamp_ns is not None else NA,
            "inter_arrival_ms": inter_arrival_ms,
            "receiver_side_aoi_proxy_ms": receiver_side_aoi_proxy_ms,
            "true_one_way_delay_ms": true_one_way_delay_ms,
            "delay_semantics": "receiver_side_proxy_only" if true_one_way_delay_ms == NA else "true_one_way_delay",
            "deadline_met": deadline_met,
            "retry_count": int(retry_count),
            "transmission_mode": str(transmission_mode),
            "execution_mode": str(execution_mode),
            "aoi_ms": NA if aoi_ms is None else round(float(aoi_ms), 3),
            "effective_freshness": NA if effective_freshness is None else round(float(effective_freshness), 6),
            "phase": str(phase),
            "task_progress": float(task_progress),
            "scenario_id": str(scenario_id),
            "method_id": str(method_id),
            "robot_id": str(robot_id),
            "source_mode": str(source_mode),
            "delivery_expected": bool(delivery_expected),
            "wrapper_mode": str(wrapper_mode),
        }

        self._last_receive_timestamp_ns = receive_ns
        self._last_seq_id = int(seq_id)
        return row


class WrapperMetadataCsvWriter:
    def __init__(self, csv_path: Path) -> None:
        self.csv_path = Path(csv_path)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not self.csv_path.exists() or self.csv_path.stat().st_size == 0
        self._file = self.csv_path.open("a", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=TRACE_FIELDS)
        if new_file:
            self._writer.writeheader()
            self._file.flush()

    def write_row(self, row: dict) -> None:
        serializable = {field: row.get(field, NA) for field in TRACE_FIELDS}
        self._writer.writerow(serializable)
        self._file.flush()

    def close(self) -> None:
        self._file.close()
