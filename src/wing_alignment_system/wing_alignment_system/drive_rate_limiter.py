# -*- coding: utf-8 -*-

from wing_alignment_system.drive_math import clamp
from wing_alignment_system.drive_types import DriveCommand


class RateLimiter:
    def __init__(self, dv_max: float, dw_max: float):
        self.dv_max = max(1e-6, float(dv_max))
        self.dw_max = max(1e-6, float(dw_max))
        self._last_v = 0.0
        self._last_w = 0.0

    def reset(self):
        self._last_v = 0.0
        self._last_w = 0.0

    def step(self, target_v: float, target_w: float, dt: float) -> DriveCommand:
        dt = clamp(dt, 0.005, 0.05)

        safe_v = clamp(
            float(target_v),
            self._last_v - self.dv_max * dt,
            self._last_v + self.dv_max * dt,
        )
        safe_w = clamp(
            float(target_w),
            self._last_w - self.dw_max * dt,
            self._last_w + self.dw_max * dt,
        )

        if abs(safe_v) < 1e-4:
            safe_v = 0.0
        if abs(safe_w) < 1e-4:
            safe_w = 0.0

        self._last_v = safe_v
        self._last_w = safe_w
        return DriveCommand(v=safe_v, w=safe_w)
