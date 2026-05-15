# -*- coding: utf-8 -*-

import math

from wing_alignment_system.cmd_watchdog_types import WatchdogState, WatchdogConfig, WatchdogOutput


class WatchdogPolicy:
    def __init__(self, cfg: WatchdogConfig):
        self.cfg = cfg
        self.st = WatchdogState()

    def on_cmd(self, seq: int, v: float, w: float, now: float) -> bool:
        if int(seq) < self.st.last_seq:
            return False
        self.st.last_seq = int(seq)
        self.st.t_last_cmd_rx = float(now)
        self.st.last_v = float(v)
        self.st.last_w = float(w)
        return True

    def on_stop(self, asserted: bool):
        if asserted:
            self.st.stop_latched = True

    def on_resume(self, asserted: bool):
        if asserted and (not self.st.emergency_latched):
            self.st.stop_latched = False

    def on_emergency(self, asserted: bool):
        self.st.emergency_latched = bool(asserted)

    def compute(self, now: float) -> WatchdogOutput:
        age = now - self.st.t_last_cmd_rx if self.st.t_last_cmd_rx > 0.0 else 1e9
        clock_jump_reset = False
        if age < -0.1:
            self.st.t_last_cmd_rx = now
            age = 0.0
            clock_jump_reset = True
        applied_v = 0.0
        applied_w = 0.0
        state = 'STOP'
        if self.st.emergency_latched:
            state = 'EMERGENCY_STOP'
        elif self.st.stop_latched:
            state = 'CMD_STOP'
        else:
            if age > self.cfg.age_stop:
                state = 'AGE_STOP'
            elif age > self.cfg.age_safe:
                x = min(1.0, max(0.0, (age - self.cfg.age_safe) / max(1e-6, self.cfg.age_stop - self.cfg.age_safe)))
                scale = math.exp(-self.cfg.decay_k * x) if self.cfg.decay_mode == 'exp' else (1.0 - x)
                if scale < 0.05:
                    scale = 0.0
                applied_v = scale * self.st.last_v
                applied_w = scale * self.st.last_w
                state = 'DECAY'
            else:
                applied_v = self.st.last_v
                applied_w = self.st.last_w
                state = 'NORMAL'
        return WatchdogOutput(applied_v=float(applied_v), applied_w=float(applied_w), state=state, age=float(age), clock_jump_reset=clock_jump_reset)
