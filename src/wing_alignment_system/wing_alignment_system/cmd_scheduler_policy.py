# -*- coding: utf-8 -*-

import random
from typing import Dict, List, Tuple

from wing_alignment_system.cmd_scheduler_types import RobotState, SchedulerConfig, TxDecision


class SchedulerPolicy:
    _IDLE_CMD_EPS = 1e-4

    def __init__(self, cfg: SchedulerConfig):
        self.cfg = cfg
        self.st: Dict[str, RobotState] = {rn: RobotState() for rn in cfg.robots}
        self._rng = random.Random()

        for rn in cfg.robots:
            self.st[rn].next_periodic_due = cfg.t0 + self._jittered(cfg.base_period)

    def _jittered(self, base: float) -> float:
        if self.cfg.jitter <= 1e-9:
            return base
        return max(0.0, base + self._rng.uniform(-self.cfg.jitter, self.cfg.jitter))

    def on_desired(self, rn: str, v: float, w: float):
        s = self.st[rn]
        dv = (float(v) - s.last_des_v) / self.cfg.v_max
        dw = (float(w) - s.last_des_w) / self.cfg.w_max
        s.last_des_v = float(v)
        s.last_des_w = float(w)
        s.voi = (dv * dv + dw * dw) ** 0.5
        if abs(s.last_des_v) > self._IDLE_CMD_EPS or abs(s.last_des_w) > self._IDLE_CMD_EPS:
            s.stream_started = True

    def on_ack(self, rn: str, seq: int, now: float):
        s = self.st[rn]
        if int(seq) > s.last_ack_seq:
            s.last_ack_seq = int(seq)
            s.t_last_ack_rx = float(now)
            s.unacked_streak = 0

    def on_precision(self, rn: str, enabled: bool):
        self.st[rn].precision_mode = bool(enabled)

    def on_goal(self, rn: str, x: float, y: float):
        self.st[rn].goal_xy = (float(x), float(y))

    def on_pose(self, rn: str, x: float, y: float):
        self.st[rn].pose_xy = (float(x), float(y))

    def tick(self, now: float) -> List[Tuple[str, TxDecision]]:
        out: List[Tuple[str, TxDecision]] = []

        for rn in self.cfg.robots:
            s = self.st[rn]

            if self.cfg.enable_eps and s.goal_xy and s.pose_xy:
                dx = s.goal_xy[0] - s.pose_xy[0]
                dy = s.goal_xy[1] - s.pose_xy[1]
                s.eps = (dx * dx + dy * dy) ** 0.5
            else:
                s.eps = 0.0

            age_est = (now - s.t_last_ack_rx) if (s.t_last_ack_rx > 0.0) else 1e9
            backoff_multiplier = min(4.0, 1.0 + 0.5 * s.unacked_streak)

            tmax_eff = self.cfg.t_max * (self.cfg.prec_tmax_scale if s.precision_mode else 1.0) * backoff_multiplier
            age_th_eff = self.cfg.age_th * (self.cfg.prec_age_scale if s.precision_mode else 1.0)

            if s.dup_pending and now >= s.dup_due:
                out.append((
                    rn,
                    TxDecision(
                        seq=s.dup_seq,
                        v=s.last_des_v,
                        w=s.last_des_w,
                        reason="dup",
                        age_est=age_est,
                        is_duplicate=True,
                    )
                ))
                s.dup_pending = False

            periodic_due = (self.cfg.base_period > 1e-6 and now >= s.next_periodic_due)
            if periodic_due:
                s.next_periodic_due = now + self._jittered(self.cfg.base_period * backoff_multiplier)

            t_min_eff = self.cfg.t_min * (1.0 if s.unacked_streak < 5 else 2.0)
            if (now - s.t_last_tx) < t_min_eff:
                continue

            idle_before_start = (
                (not s.stream_started) and
                abs(s.last_des_v) <= self._IDLE_CMD_EPS and
                abs(s.last_des_w) <= self._IDLE_CMD_EPS and
                s.voi <= self.cfg.voi_th and
                s.eps <= self.cfg.eps_th
            )
            if idle_before_start:
                continue

            reason = None
            critical = False

            if self.cfg.enable_eps and (s.eps > self.cfg.eps_th):
                reason = "event_eps"
                critical = s.eps > self.cfg.eps_high_th
            elif s.voi > self.cfg.voi_th:
                reason = "event_voi"
                critical = s.voi > self.cfg.voi_high_th
            elif age_est > age_th_eff:
                reason = "event_age"
                critical = age_est > (1.5 * age_th_eff)
            elif age_est > tmax_eff or periodic_due:
                reason = "periodic"
                critical = False

            if not reason:
                continue

            s.seq_tx += 1
            s.unacked_streak += 1
            s.t_last_tx = now
            s.stream_started = True

            out.append((
                rn,
                TxDecision(
                    seq=s.seq_tx,
                    v=s.last_des_v,
                    w=s.last_des_w,
                    reason=reason,
                    age_est=age_est,
                    is_duplicate=False,
                )
            ))

            if critical and self.cfg.dup_delay > 1e-6:
                s.dup_pending = True
                s.dup_due = now + self.cfg.dup_delay
                s.dup_seq = s.seq_tx

        return out
