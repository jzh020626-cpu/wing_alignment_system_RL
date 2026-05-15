# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Optional, Tuple, List


@dataclass
class RobotState:
    seq_tx: int = 0
    last_des_v: float = 0.0
    last_des_w: float = 0.0
    voi: float = 0.0
    stream_started: bool = False

    last_ack_seq: int = 0
    t_last_ack_rx: float = 0.0
    unacked_streak: int = 0

    t_last_tx: float = 0.0
    next_periodic_due: float = 0.0

    dup_pending: bool = False
    dup_due: float = 0.0
    dup_seq: int = 0

    goal_xy: Optional[Tuple[float, float]] = None
    pose_xy: Optional[Tuple[float, float]] = None
    eps: float = 0.0
    precision_mode: bool = False


@dataclass
class SchedulerConfig:
    robots: List[str]
    tick_hz: float
    v_max: float
    w_max: float
    base_period: float
    jitter: float
    t_min: float
    t_max: float
    age_th: float
    prec_tmax_scale: float
    prec_age_scale: float
    voi_th: float
    voi_high_th: float
    dup_delay: float
    enable_eps: bool
    eps_th: float
    eps_high_th: float
    t0: float = 0.0


@dataclass
class TxDecision:
    seq: int
    v: float
    w: float
    reason: str
    age_est: float
    is_duplicate: bool = False
