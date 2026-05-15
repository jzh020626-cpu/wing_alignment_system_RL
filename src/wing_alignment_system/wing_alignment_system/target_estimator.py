# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


def _clamp01(x: float) -> float:
    return 0.0 if x <= 0.0 else (1.0 if x >= 1.0 else x)


@dataclass
class TargetStatus:
    tracking: bool
    soft_lost: bool
    hard_lost: bool
    confidence: float
    stable_count: int
    age_sec: float
    last_seen_ago: float


@dataclass
class TargetProposal:
    goal_x: float
    goal_y: float
    dx: float
    dy: float
    dz: float
    confidence: float


class TargetEstimator:
    def __init__(
        self,
        *,
        arm_delay_sec: float = 0.6,
        max_age_sec: float = 0.35,
        frame_gap_max_sec: float = 0.25,
        abs_max_m: float = 2.0,
        stable_eps: float = 0.005,
        stable_frames: int = 3,
        min_confidence: float = 0.55,
        jump_sigma: float = 0.02,
        lost_timeout_sec: float = 0.25,
        freeze_timeout_sec: float = 1.20,
        ema_alpha: float = 0.45,
        gain: float = 1.0,
        step_cap_m: float = 0.02,
        delta_in_world: bool = False,
        error_sign: float = 1.0,
        dx_sign: float = 1.0,
        dy_sign: float = 1.0,
        duplicate_eps: float = 0.0,
        max_jump_suppress: float = 0.0,
        jump_decay_alpha: float = 0.3,
    ):
        self.arm_delay_sec = float(arm_delay_sec)
        self.max_age_sec = float(max_age_sec)
        self.frame_gap_max_sec = float(frame_gap_max_sec)
        self.abs_max_m = float(abs_max_m)
        self.stable_eps = float(stable_eps)
        self.stable_frames = max(1, int(stable_frames))
        self.min_confidence = float(min_confidence)
        self.jump_sigma = float(jump_sigma)
        self.lost_timeout_sec = float(lost_timeout_sec)
        self.freeze_timeout_sec = float(freeze_timeout_sec)
        self.ema_alpha = _clamp01(float(ema_alpha))
        self.gain = float(gain)
        self.step_cap_m = float(step_cap_m)
        self.delta_in_world = bool(delta_in_world)
        self.error_sign = float(error_sign)
        self.dx_sign = float(dx_sign)
        self.dy_sign = float(dy_sign)
        self.duplicate_eps = max(0.0, float(duplicate_eps))
        self.max_jump_suppress = max(0.0, float(max_jump_suppress))
        self.jump_decay_alpha = _clamp01(float(jump_decay_alpha))
        self.reset()

    def reset(self):
        self._armed = False
        self._armed_since_wall = 0.0
        self._last_seen_wall = 0.0
        self._last_msg_t_ros = None
        self._last_age_sec = 1e9
        self._last_raw = None
        self._last_stable = None
        self._stable_count = 0
        self._confidence = 0.0
        self._goal_ema = None
        self._duplicate_count = 0
        self._jump_suppressed_count = 0
        self._last_jump_magnitude = 0.0

    def clear_goal_ema(self):
        self._goal_ema = None

    def arm(self, now_wall: float):
        self._armed = True
        self._armed_since_wall = float(now_wall)
        self._last_msg_t_ros = None
        self._last_age_sec = 1e9
        self._last_raw = None
        self._last_stable = None
        self._stable_count = 0
        self._confidence = 0.0
        self._goal_ema = None
        self._last_seen_wall = 0.0
        self._duplicate_count = 0
        self._jump_suppressed_count = 0
        self._last_jump_magnitude = 0.0

    def observe_delta(self, dx: float, dy: float, dz: float, msg_stamp_ros_sec: float, now_ros_sec: float, now_wall: float) -> bool:
        if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
            return False
        if (abs(dx) > self.abs_max_m) or (abs(dy) > self.abs_max_m) or (abs(dz) > self.abs_max_m):
            return False

        current = (float(dx), float(dy), float(dz))

        if not self._armed:
            return True
        self._last_seen_wall = float(now_wall)
        if (now_wall - self._armed_since_wall) < self.arm_delay_sec:
            return True
        age = float(now_ros_sec - msg_stamp_ros_sec)
        self._last_age_sec = age
        if age > self.max_age_sec:
            return True
        if self._last_msg_t_ros is not None and (msg_stamp_ros_sec - self._last_msg_t_ros) > self.frame_gap_max_sec:
            self._stable_count = 0
            self._last_raw = None
            self._last_stable = None
            self._confidence = 0.0

        self._last_msg_t_ros = float(msg_stamp_ros_sec)
        if self._last_raw is None:
            self._stable_count = 1
        else:
            lx, ly, lz = self._last_raw
            if self.duplicate_eps > 0.0:
                raw_jump = math.sqrt((dx - lx) ** 2 + (dy - ly) ** 2 + (dz - lz) ** 2)
                if raw_jump <= self.duplicate_eps:
                    self._duplicate_count += 1
            ok = (abs(dx - lx) <= self.stable_eps and abs(dy - ly) <= self.stable_eps and abs(dz - lz) <= self.stable_eps)
            self._stable_count = (self._stable_count + 1) if ok else 1
        self._last_raw = current
        if self._stable_count >= self.stable_frames:
            conf_age = _clamp01(1.0 - max(0.0, age) / max(1e-6, self.max_age_sec))
            conf_stable = _clamp01(self._stable_count / float(self.stable_frames))
            if self._last_stable is None:
                conf_jump = 1.0
                self._last_jump_magnitude = 0.0
            else:
                sx, sy, sz = self._last_stable
                jump = math.sqrt((dx - sx) ** 2 + (dy - sy) ** 2 + (dz - sz) ** 2)
                self._last_jump_magnitude = float(jump)
                jump_for_conf = float(jump)
                if self.max_jump_suppress > 0.0 and jump_for_conf > self.max_jump_suppress:
                    jump_for_conf = self.max_jump_suppress
                    self._jump_suppressed_count += 1
                sigma = max(1e-6, self.jump_sigma)
                conf_jump = math.exp(-jump_for_conf / sigma)
            new_confidence = float(conf_age * conf_stable * conf_jump)
            if self._confidence > 0.0:
                alpha = self.jump_decay_alpha
                self._confidence = float((1.0 - alpha) * self._confidence + alpha * new_confidence)
            else:
                self._confidence = new_confidence
            self._last_stable = current
        return True

    def status(self, now_wall: float) -> TargetStatus:
        if (not self._armed) or (self._last_seen_wall <= 0.0):
            last_seen_ago = 1e9
            soft_lost = False
            hard_lost = False
        else:
            last_seen_ago = float(now_wall - self._last_seen_wall)
            soft_lost = last_seen_ago > self.lost_timeout_sec
            hard_lost = last_seen_ago > self.freeze_timeout_sec
        tracking = self._armed and (not hard_lost) and (not soft_lost) and (self._last_stable is not None) and (self._confidence >= self.min_confidence) and (self._last_age_sec <= self.max_age_sec)
        return TargetStatus(tracking=tracking, soft_lost=soft_lost, hard_lost=hard_lost, confidence=float(self._confidence), stable_count=int(self._stable_count), age_sec=float(self._last_age_sec), last_seen_ago=float(last_seen_ago))

    def get_stable_delta(self):
        if self._last_stable is None:
            return None
        dx, dy, dz = self._last_stable
        return float(dx), float(dy), float(dz)

    def get_diagnostics(self) -> Dict[str, float]:
        return {
            'duplicate_count': int(self._duplicate_count),
            'jump_suppressed_count': int(self._jump_suppressed_count),
            'last_jump_magnitude': float(self._last_jump_magnitude),
            'confidence': float(self._confidence),
            'stable_count': int(self._stable_count),
        }
