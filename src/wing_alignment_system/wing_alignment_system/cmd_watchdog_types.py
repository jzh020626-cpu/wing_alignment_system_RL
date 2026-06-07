# -*- coding: utf-8 -*-

from dataclasses import dataclass


@dataclass
class WatchdogState:
    last_seq: int = 0
    t_last_cmd_rx: float = 0.0
    last_v: float = 0.0
    last_w: float = 0.0
    execution_mode: str = "normal"
    stop_latched: bool = False
    emergency_latched: bool = False


@dataclass
class WatchdogConfig:
    watchdog_hz: float
    age_safe: float
    age_stop: float
    decay_mode: str
    decay_k: float
    enable_execution_mode_output: bool = False
    degraded_linear_scale: float = 0.5
    degraded_angular_scale: float = 0.25


@dataclass
class WatchdogOutput:
    applied_v: float
    applied_w: float
    state: str
    age: float
    clock_jump_reset: bool = False
    output_scale: float = 1.0
    stop_reason: str = ""
