from __future__ import annotations

import csv
from pathlib import Path
from typing import Any


CSV_FIELDS = [
    "timestamp_wall",
    "timestamp_ros",
    "robot_id",
    "peer_id",
    "topic_name",
    "seq_id",
    "msg_size_bytes",
    "sender_timestamp",
    "receiver_timestamp",
    "one_way_delay_ms",
    "receiver_side_latency_proxy_ms",
    "inter_arrival_ms",
    "packet_loss_flag",
    "estimated_bandwidth_kbps",
    "AoI_ms",
    "Effective_Freshness",
    "phase",
    "task_progress",
    "control_mode",
    "emergency_stop",
    "fallback_flag",
    "done_reason",
    "time_sync_mode",
    "warning_flags",
]


class CalibrationCsvWriter:
    def __init__(self, path: str | Path, *, flush_every_n_rows: int = 10):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.flush_every_n_rows = max(int(flush_every_n_rows), 1)
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._rows_written = 0

    @staticmethod
    def _normalize_value(value: Any) -> Any:
        return "n/a" if value is None else value

    def write_row(self, row: dict[str, Any]) -> None:
        normalized = {field: self._normalize_value(row.get(field)) for field in CSV_FIELDS}
        self._writer.writerow(normalized)
        self._rows_written += 1
        if self._rows_written % self.flush_every_n_rows == 0:
            self._file.flush()

    def flush(self) -> None:
        self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()
