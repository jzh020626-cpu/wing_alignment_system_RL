from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


@dataclass
class TopicState:
    topic_name: str
    robot_id: str
    peer_id: str
    bandwidth_window_sec: float = 5.0
    last_seq_id: int | None = None
    last_receive_time_ns: int | None = None
    last_fresh_update_time_ns: int | None = None
    received_count: int = 0
    missing_count: int = 0
    total_bytes: int = 0
    loss_events: int = 0
    inter_arrival_history_ms: list[float] = field(default_factory=list)
    _bandwidth_window: deque[tuple[int, int]] = field(default_factory=deque)

    def _update_bandwidth(self, receive_time_ns: int, msg_size_bytes: int) -> float:
        self._bandwidth_window.append((receive_time_ns, int(msg_size_bytes)))
        min_ns = receive_time_ns - int(self.bandwidth_window_sec * 1_000_000_000)
        while self._bandwidth_window and self._bandwidth_window[0][0] < min_ns:
            self._bandwidth_window.popleft()
        if len(self._bandwidth_window) <= 1:
            return 0.0
        first_time = self._bandwidth_window[0][0]
        elapsed_sec = max((receive_time_ns - first_time) / 1_000_000_000.0, 1e-9)
        total_bits = sum(size for _stamp, size in self._bandwidth_window) * 8.0
        return total_bits / elapsed_sec / 1000.0

    def update(
        self,
        *,
        seq_id: int,
        receive_time_ns: int,
        msg_size_bytes: int,
        sender_timestamp_ns: int | None,
    ) -> dict[str, float | bool | int | None]:
        packet_loss_flag = False
        if self.last_seq_id is not None and seq_id > self.last_seq_id + 1:
            self.missing_count += seq_id - self.last_seq_id - 1
            self.loss_events += 1
            packet_loss_flag = True
        inter_arrival_ms = None
        if self.last_receive_time_ns is not None:
            inter_arrival_ms = (receive_time_ns - self.last_receive_time_ns) / 1_000_000.0
            self.inter_arrival_history_ms.append(inter_arrival_ms)
        self.last_receive_time_ns = int(receive_time_ns)
        self.last_seq_id = int(seq_id)
        self.received_count += 1
        self.total_bytes += int(msg_size_bytes)
        if sender_timestamp_ns is not None:
            self.last_fresh_update_time_ns = int(sender_timestamp_ns)
            aoi_ms = max((receive_time_ns - sender_timestamp_ns) / 1_000_000.0, 0.0)
        elif self.last_fresh_update_time_ns is not None:
            aoi_ms = max((receive_time_ns - self.last_fresh_update_time_ns) / 1_000_000.0, 0.0)
        else:
            aoi_ms = None
        bandwidth = self._update_bandwidth(int(receive_time_ns), int(msg_size_bytes))
        return {
            "packet_loss_flag": packet_loss_flag,
            "inter_arrival_ms": inter_arrival_ms,
            "estimated_bandwidth_kbps": bandwidth,
            "AoI_ms": aoi_ms,
        }

    def summary(self) -> dict[str, int | float | str | None]:
        return {
            "topic_name": self.topic_name,
            "robot_id": self.robot_id,
            "peer_id": self.peer_id,
            "received_count": self.received_count,
            "missing_count": self.missing_count,
            "loss_events": self.loss_events,
            "total_bytes": self.total_bytes,
            "last_seq_id": self.last_seq_id,
            "mean_inter_arrival_ms": (
                sum(self.inter_arrival_history_ms) / len(self.inter_arrival_history_ms)
                if self.inter_arrival_history_ms
                else None
            ),
        }
