from __future__ import annotations

from freshness_real_robot_validation.frame_id_codec import encode_validation_frame_id


def build_synthetic_frame_spec(
    *,
    seq_id: int,
    robot_name: str,
    task_phase: str,
    task_progress: float,
    payload_bytes: int,
    method_id: str = "SYNTHETIC_OBSERVE_ONLY",
    transmission_mode: str = "synthetic_heartbeat",
) -> dict:
    return {
        "seq_id": int(seq_id),
        "robot_id": str(robot_name),
        "task_phase": str(task_phase),
        "task_progress": float(task_progress),
        "payload_bytes": int(payload_bytes),
        "method_id": str(method_id),
        "transmission_mode": str(transmission_mode),
        "source_mode": "synthetic_observe_only",
        "frame_id": encode_validation_frame_id(
            seq_id=int(seq_id),
            transmission_mode=str(transmission_mode),
            payload_bytes=int(payload_bytes),
            method_id=str(method_id),
            task_phase=str(task_phase),
            task_progress=float(task_progress),
        ),
    }
