# -*- coding: utf-8 -*-

import time
import math

from base_interfaces_demo.msg import MotorCommand
from std_msgs.msg import Bool

from wing_alignment_system.mission_geometry import _now_sec


class MissionSlideManagerMixin:
    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------
    def _diag_ok(self, ctx, key: str, period_sec: float) -> bool:
        now = _now_sec(self)
        last = getattr(ctx, key, 0.0)
        if (now - last) >= period_sec:
            setattr(ctx, key, now)
            return True
        return False

    def _diag_slide_status(self, rn: str, ctx, st, prefix: str):
        if not self._diag_ok(ctx, "_diag_slide_status_epoch", 1.0):
            return
        self.get_logger().warn(
            f"[SLIDE_DIAG][{rn}] {prefix} "
            f"tracking={st.tracking} soft_lost={st.soft_lost} hard_lost={st.hard_lost} "
            f"conf={st.confidence:.3f} stable_count={st.stable_count} "
            f"age_sec={st.age_sec:.3f} last_seen_ago={st.last_seen_ago:.3f} "
            f"delta_latest={ctx.delta_latest} last_delta_stamp={ctx.last_delta_stamp}"
        )

    def _diag_slide_cmd(self, rn: str, ctx, dx: float, dy: float, dz: float, vx: float, vy: float, vz: float, mode: str = "align"):
        if not self._diag_ok(ctx, f"_diag_slide_cmd_epoch_{mode}", 0.5):
            return
        self.get_logger().warn(
            f"[SLIDE_CMD][{rn}][{mode}] delta=({dx:.4f},{dy:.4f},{dz:.4f}) m "
            f"-> speed_cmd=({vx:.2f},{vy:.2f},{vz:.2f}) mm/s "
            f"fine_active={ctx.fine_active} ready={ctx.ready_to_lift}"
        )

    def _diag_direct_pos_cmd(self, rn: str, phase: str, x_mm: float, y_mm: float, z_mm: float, move_time: float):
        self.get_logger().warn(
            f"[SLIDE_DIRECT][{rn}] phase={phase} "
            f"relative_pos_cmd=({x_mm:.1f},{y_mm:.1f},{z_mm:.1f}) mm "
            f"time={move_time:.2f}s"
        )

    def _diag_ready_check(self, rn: str, ctx, dx: float, dy: float, dz: float, xy_ok: bool, z_ok: bool):
        if not self._diag_ok(ctx, "_diag_ready_epoch", 0.5):
            return
        self.get_logger().warn(
            f"[SLIDE_READY_CHECK][{rn}] delta=({dx:.4f},{dy:.4f},{dz:.4f}) "
            f"xy_ok={xy_ok} z_ok={z_ok} xy_stable_count={ctx.xy_stable_count}/{self.fine_xy_stable_frames}"
        )

    def _diag_transport_center_hold(
        self,
        rn: str,
        ctx,
        target_src: str,
        target_x_mm: float,
        target_y_mm: float,
        current_x_mm: float,
        current_y_mm: float,
        err_x_mm: float,
        err_y_mm: float,
        vx_hold: float,
        vy_hold: float,
        force_scale: float
    ):
        if not self._diag_ok(ctx, "_diag_transport_center_hold_epoch", 0.5):
            return
        self.get_logger().warn(
            f"[SLIDE_CENTER_HOLD][{rn}] src={target_src} "
            f"target=({target_x_mm:.2f},{target_y_mm:.2f}) mm "
            f"current=({current_x_mm:.2f},{current_y_mm:.2f}) mm "
            f"err=({err_x_mm:.2f},{err_y_mm:.2f}) mm "
            f"hold_cmd=({vx_hold:.2f},{vy_hold:.2f}) mm/s "
            f"force_scale={force_scale:.3f}"
        )

    def _diag_transport_blend(
        self,
        rn: str,
        ctx,
        dx: float,
        dy: float,
        dz: float,
        vx_delta: float,
        vy_delta: float,
        vz_delta: float,
        vx_ff: float,
        vy_ff: float,
        vx_hold: float,
        vy_hold: float,
        vx_final: float,
        vy_final: float,
        vz_final: float
    ):
        if not self._diag_ok(ctx, "_diag_transport_blend_epoch", 0.5):
            return

        te = ctx.te
        vision_diag = ""
        if te is not None and hasattr(te, 'get_diagnostics'):
            diag = te.get_diagnostics()
            dup = diag.get('duplicate_count', 0)
            jump_sup = diag.get('jump_suppressed_count', 0)
            last_jump = diag.get('last_jump_magnitude', 0.0)
            if dup > 0 or jump_sup > 0:
                vision_diag = f" | vision: dup={dup} jump_sup={jump_sup} last_jump={last_jump:.3f}m"

        mode_tag = 'transport+recenter' if bool(getattr(self, 'slide_transport_recenter_enable', False)) else 'transport'
        self.get_logger().warn(
            f"[SLIDE_CMD][{rn}][{mode_tag}] "
            f"delta=({dx:.4f},{dy:.4f},{dz:.4f}) m "
            f"delta_cmd=({vx_delta:.2f},{vy_delta:.2f},{vz_delta:.2f}) mm/s "
            f"ff_cmd=({vx_ff:.2f},{vy_ff:.2f},0.00) mm/s "
            f"recenter_cmd=({vx_hold:.2f},{vy_hold:.2f},0.00) mm/s "
            f"final_cmd=({vx_final:.2f},{vy_final:.2f},{vz_final:.2f}) mm/s "
            f"transporting={ctx.transporting}"
            f"{vision_diag}"
        )

    def _diag_transport_ff(self, rn: str, ctx, vx_body: float, vy_body: float, wz: float, vx_ff: float, vy_ff: float):
        if not self._diag_ok(ctx, "_diag_transport_ff_epoch", 0.5):
            return
        self.get_logger().warn(
            f"[SLIDE_FF][{rn}] chassis_body_twist=({vx_body:.3f},{vy_body:.3f},{wz:.3f}) "
            f"-> ff_cmd=({vx_ff:.2f},{vy_ff:.2f},0.00) mm/s"
        )

    def _diag_axis_mask(self, rn: str, kind: str, raw_xyz, masked_xyz):
        if raw_xyz == masked_xyz:
            return
        self.get_logger().warn(
            f"[SLIDE_MASK][{rn}][{kind}] raw={raw_xyz} -> masked={masked_xyz}"
        )

    def _slide_comp_zero_due(self, ctx, now: float) -> bool:
        return ctx.comp_active and ((now - ctx.last_comp_pub_epoch) >= self.slide_comp_cmd_period_sec)

    # ------------------------------------------------------------------
    # alignment strategy
    # ------------------------------------------------------------------
    def _slide_align_mode(self) -> str:
        # 上架/对位阶段强制只走 direct position，避免在该阶段进入连续速度补偿。
        mode = str(getattr(self, 'slide_align_mode', 'direct_only')).strip().lower()
        if mode != 'direct_only':
            if not getattr(self, '_slide_align_mode_forced_logged', False):
                self.get_logger().warn(
                    f'[SLIDE_ALIGN_MODE] requested={mode} but alignment stage is forced to direct_only'
                )
                self._slide_align_mode_forced_logged = True
        return 'direct_only'

    def _reset_direct_align_state(self, ctx):
        ctx.direct_align_phase = "idle"
        ctx.direct_align_retry = 0
        ctx.direct_align_epoch = 0.0
        ctx.direct_align_xy_time = 0.0
        ctx.direct_align_z_time = 0.0
        ctx.direct_align_pending_z_mm = 0.0
        ctx.direct_align_ready_dz_m = 0.0
        ctx.direct_align_started_at = 0.0
        ctx.direct_align_xy_start_pos = None
        ctx.direct_align_xy_cmd_x_mm = 0.0
        ctx.direct_align_xy_cmd_y_mm = 0.0
        ctx.direct_align_z_cmd_mm = 0.0
        ctx.direct_align_positive_z_attempted = False
        ctx.direct_align_small_residual_count = 0
        ctx.direct_align_z_done = False
        ctx.direct_align_force_monitor_latched = False
        ctx.direct_align_force_contact_latched = False
        ctx.direct_align_force_contact_count = 0
        ctx.direct_align_force_last_eval_stamp = 0.0
        ctx.direct_align_post_contact_hold_start = 0.0
        ctx.direct_align_post_contact_z_ref_mm = None
        ctx.direct_align_xy_ack_drop_seen = False

    # ------------------------------------------------------------------
    # capability map
    # ------------------------------------------------------------------
    def _slide_actor_key(self, rn: str) -> str:
        s = str(rn)
        if s.startswith('tracer'):
            suffix = s[len('tracer'):]
            if suffix.isdigit():
                return f'huatai{suffix}'
        return s

    def _default_slide_axis_enable_map(self):
        # 默认三轴全开；如需限轴，请通过 slide_axis_enable_map 覆盖。
        return {
            'huatai1': {'x': True,  'y': True, 'z': True},
            'huatai2': {'x': True,  'y': True, 'z': True},
            'huatai3': {'x': True,  'y': True, 'z': True},
            'tracer1': {'x': True,  'y': True, 'z': True},
            'tracer2': {'x': True,  'y': True, 'z': True},
            'tracer3': {'x': True,  'y': True, 'z': True},
        }

    def _slide_axis_enable_map(self):
        cfg = getattr(self, 'slide_axis_enable_map', None)
        default_cfg = self._default_slide_axis_enable_map()

        if not isinstance(cfg, dict):
            return default_cfg

        merged = dict(default_cfg)
        for k, v in cfg.items():
            if not isinstance(v, dict):
                continue
            base = merged.get(k, {'x': False, 'y': False, 'z': False})
            merged[k] = {
                'x': bool(v.get('x', base.get('x', False))),
                'y': bool(v.get('y', base.get('y', False))),
                'z': bool(v.get('z', base.get('z', False))),
            }
        return merged

    def _slide_axis_enabled(self, rn: str, axis: str) -> bool:
        cfg = self._slide_axis_enable_map()

        key_robot = str(rn)
        key_slide = self._slide_actor_key(rn)

        if key_slide in cfg:
            return bool(cfg[key_slide].get(axis, True))
        if key_robot in cfg:
            return bool(cfg[key_robot].get(axis, True))

        return True

    def _mask_delta_for_robot(self, rn: str, dx: float, dy: float, dz: float):
        mdx = float(dx) if self._slide_axis_enabled(rn, 'x') else 0.0
        mdy = float(dy) if self._slide_axis_enabled(rn, 'y') else 0.0
        mdz = float(dz) if self._slide_axis_enabled(rn, 'z') else 0.0
        self._diag_axis_mask(rn, "delta", (dx, dy, dz), (mdx, mdy, mdz))
        return mdx, mdy, mdz

    def _mask_speed_for_robot(self, rn: str, vx: float, vy: float, vz: float):
        mvx = float(vx) if self._slide_axis_enabled(rn, 'x') else 0.0
        mvy = float(vy) if self._slide_axis_enabled(rn, 'y') else 0.0
        mvz = float(vz) if self._slide_axis_enabled(rn, 'z') else 0.0
        self._diag_axis_mask(rn, "speed", (vx, vy, vz), (mvx, mvy, mvz))
        return mvx, mvy, mvz

    def _mask_rel_position_for_robot(self, rn: str, x_mm: float, y_mm: float, z_mm: float):
        mx = float(x_mm) if self._slide_axis_enabled(rn, 'x') else 0.0
        my = float(y_mm) if self._slide_axis_enabled(rn, 'y') else 0.0
        mz = float(z_mm) if self._slide_axis_enabled(rn, 'z') else 0.0
        self._diag_axis_mask(rn, "rel_pos", (x_mm, y_mm, z_mm), (mx, my, mz))
        return mx, my, mz

    def _mask_abs_position_for_robot(self, rn: str, x_mm: float, y_mm: float, z_mm: float):
        ctx = self.rt[rn]
        cur = getattr(ctx, 'slide_pos', None)

        cx = float(cur[0]) if cur is not None else float(x_mm)
        cy = float(cur[1]) if cur is not None else float(y_mm)
        cz = float(cur[2]) if cur is not None else float(z_mm)

        mx = float(x_mm) if self._slide_axis_enabled(rn, 'x') else cx
        my = float(y_mm) if self._slide_axis_enabled(rn, 'y') else cy
        mz = float(z_mm) if self._slide_axis_enabled(rn, 'z') else cz

        self._diag_axis_mask(rn, "abs_pos", (x_mm, y_mm, z_mm), (mx, my, mz))
        return mx, my, mz

    def _axis_ok_or_disabled(self, rn: str, axis: str, err_abs: float, tol: float) -> bool:
        if not self._slide_axis_enabled(rn, axis):
            return True
        return abs(err_abs) <= tol

    # ------------------------------------------------------------------
    # common mapping helpers
    # ------------------------------------------------------------------
    def _norm_sign(self, v: float) -> float:
        v = float(v)
        if abs(v) < 1e-9:
            return 1.0
        return 1.0 if v > 0.0 else -1.0

    def _delta_to_slide_rel_mm(self, dx: float, dy: float, dz: float):
        sx = self._norm_sign(self.slide_comp_x_sign)
        sy = self._norm_sign(self.slide_comp_y_sign)
        sz = self._norm_sign(self.slide_comp_z_sign)
        return (
            sx * dx * 1000.0,
            sy * dy * 1000.0,
            sz * dz * 1000.0,
        )

    # ------------------------------------------------------------------
    # slide command helpers
    # ------------------------------------------------------------------
    def _clamp(self, v: float, lim: float) -> float:
        return max(-abs(lim), min(abs(lim), v))

    def _apply_min_comp_speed(self, v: float, vmin: float) -> float:
        if abs(v) < 1e-9:
            return 0.0
        if abs(v) < abs(vmin):
            return math.copysign(abs(vmin), v)
        return v

    def _slew_limit(self, prev: float, target: float, amax: float, dt: float) -> float:
        step = abs(amax) * max(dt, 1e-3)
        if target > prev + step:
            return prev + step
        if target < prev - step:
            return prev - step
        return target

    def send_slide_position(self, rn: str, x_mm: float, y_mm: float, z_mm: float, time_sec: float, is_relative: bool = False):
        if is_relative:
            x_mm, y_mm, z_mm = self._mask_rel_position_for_robot(rn, x_mm, y_mm, z_mm)
        else:
            x_mm, y_mm, z_mm = self._mask_abs_position_for_robot(rn, x_mm, y_mm, z_mm)

        msg = MotorCommand()
        msg.command_type = 'position'
        msg.x = float(x_mm)
        msg.y = float(y_mm)
        msg.z = float(z_mm)
        msg.time = float(time_sec)
        msg.is_relative = bool(is_relative)
        self.slide_cmd_pub[rn].publish(msg)

    def stop_slide_comp(self, rn: str):
        if not self.slide_comp_enable:
            return

        msg = MotorCommand()
        msg.command_type = 'stop'
        self.slide_comp_pub[rn].publish(msg)

        ctx = self.rt[rn]
        ctx.comp_active = False
        ctx.comp_last_vx = 0.0
        ctx.comp_last_vy = 0.0
        ctx.comp_last_vz = 0.0
        ctx.last_comp_pub_epoch = _now_sec(self)

    def stop_slide_position(self, rn: str):
        msg = MotorCommand()
        msg.command_type = 'stop'
        self.slide_cmd_pub[rn].publish(msg)

    def stop_all_slide_comp(self):
        for rn in self.robots:
            self.stop_slide_comp(rn)

    def _pub_slide_speed(self, rn: str, vx: float, vy: float, vz: float, force: bool = False):
        if not self.slide_comp_enable:
            return

        vx, vy, vz = self._mask_speed_for_robot(rn, vx, vy, vz)

        now = _now_sec(self)
        ctx = self.rt[rn]

        if (not force) and (now - ctx.last_comp_pub_epoch) < self.slide_comp_cmd_period_sec:
            return

        vx = self._apply_min_comp_speed(vx, self.slide_comp_vx_min_mmps) if abs(vx) > 1e-9 else 0.0
        vy = self._apply_min_comp_speed(vy, self.slide_comp_vy_min_mmps) if abs(vy) > 1e-9 else 0.0
        vz = self._apply_min_comp_speed(vz, self.slide_comp_vz_min_mmps) if abs(vz) > 1e-9 else 0.0

        dt = max(self.slide_comp_cmd_period_sec, now - ctx.last_comp_pub_epoch) if ctx.last_comp_pub_epoch > 0.0 else self.slide_comp_cmd_period_sec

        vx = self._slew_limit(ctx.comp_last_vx, vx, self.slide_comp_ax_limit_mmps2, dt)
        vy = self._slew_limit(ctx.comp_last_vy, vy, self.slide_comp_ay_limit_mmps2, dt)
        vz = self._slew_limit(ctx.comp_last_vz, vz, self.slide_comp_az_limit_mmps2, dt)

        msg = MotorCommand()
        msg.command_type = 'speed'
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        self.slide_comp_pub[rn].publish(msg)

        ctx.comp_active = not (abs(vx) < 1e-9 and abs(vy) < 1e-9 and abs(vz) < 1e-9)
        ctx.comp_last_vx = vx
        ctx.comp_last_vy = vy
        ctx.comp_last_vz = vz
        ctx.last_comp_pub_epoch = now

    def _slide_speed_from_delta(self, dx: float, dy: float, dz: float):
        if abs(dx) < self.slide_comp_dx_deadband_m:
            dx = 0.0
        if abs(dy) < self.slide_comp_dy_deadband_m:
            dy = 0.0
        if abs(dz) < self.slide_comp_dz_deadband_m:
            dz = 0.0

        sx = self._norm_sign(self.slide_comp_x_sign)
        sy = self._norm_sign(self.slide_comp_y_sign)
        sz = self._norm_sign(self.slide_comp_z_sign)

        vx = sx * self.slide_comp_vx_gain_mmps_per_m * dx
        vy = sy * self.slide_comp_vy_gain_mmps_per_m * dy
        vz = sz * self.slide_comp_vz_gain_mmps_per_m * dz

        vx = self._clamp(vx, self.slide_comp_vx_limit_mmps)
        vy = self._clamp(vy, self.slide_comp_vy_limit_mmps)
        vz = self._clamp(vz, self.slide_comp_vz_limit_mmps)
        return vx, vy, vz

    def _get_stable_delta(self, rn: str):
        ctx = self.rt[rn]
        te = ctx.te
        if te is None:
            return None

        st = te.status(time.time())
        if not st.tracking:
            return None

        if hasattr(te, 'get_stable_delta'):
            out = te.get_stable_delta()
            if out is not None:
                dx, dy, dz = out
                return float(dx), float(dy), float(dz)

        last_stable = getattr(te, "_last_stable", None)
        if last_stable is None:
            return ctx.delta_latest

        dx, dy, dz = last_stable
        return float(dx), float(dy), float(dz)

    # ------------------------------------------------------------------
    # transport-specific compliant tracking
    # ------------------------------------------------------------------
    def _transport_xy_force_scale(self, ctx) -> float:
        enabled = bool(
            getattr(
                self,
                'slide_transport_force_yield_enable',
                getattr(self, 'transport_xy_force_yield_enable', True)
            )
        )
        if not enabled:
            return 1.0

        ff = getattr(ctx, 'force_f', None)
        if ff is None:
            return 1.0

        fx = float(ff[0])
        fy = float(ff[1])
        fxy = math.hypot(fx, fy)

        lo = float(
            getattr(
                self,
                'slide_transport_force_yield_deadband_n',
                getattr(self, 'transport_xy_force_yield_deadband_n', 0.0)
            )
        )
        hi = float(
            getattr(
                self,
                'slide_transport_force_yield_full_n',
                getattr(self, 'transport_xy_force_yield_full_n', max(lo + 1.0, 1.0))
            )
        )
        s_min = float(
            getattr(
                self,
                'slide_transport_force_yield_min_scale',
                getattr(self, 'transport_xy_force_yield_min_scale', 0.2)
            )
        )

        if fxy <= lo:
            return 1.0
        if fxy >= hi:
            return s_min

        ratio = (fxy - lo) / max(1e-6, hi - lo)
        return 1.0 - ratio * (1.0 - s_min)

    def _slide_transport_speed_from_delta(self, dx: float, dy: float, dz: float, ctx):
        if abs(dx) < self.slide_transport_dx_deadband_m:
            dx = 0.0
        if abs(dy) < self.slide_transport_dy_deadband_m:
            dy = 0.0

        if abs(dz) < self.slide_transport_dz_deadband_m:
            dz = 0.0
        if abs(dz) < self.slide_transport_dz_hold_tol_m:
            dz = 0.0

        sx = self._norm_sign(self.slide_comp_x_sign)
        sy = self._norm_sign(self.slide_comp_y_sign)
        sz = self._norm_sign(self.slide_comp_z_sign)

        force_scale = self._transport_xy_force_scale(ctx)

        vx = sx * self.slide_transport_vx_gain_mmps_per_m * dx * force_scale
        vy = sy * self.slide_transport_vy_gain_mmps_per_m * dy * force_scale
        vz = sz * self.slide_transport_vz_gain_mmps_per_m * dz

        vx = self._clamp(vx, self.slide_transport_vx_limit_mmps)
        vy = self._clamp(vy, self.slide_transport_vy_limit_mmps)
        vz = self._clamp(vz, self.slide_transport_vz_limit_mmps)
        return vx, vy, vz

    def _resolve_transport_center_target_xy_mm(self, rn: str, ctx):
        targets = getattr(self, 'slide_transport_recenter_targets', None)
        if isinstance(targets, dict) and (rn in targets):
            tgt = targets[rn]
            if tgt is not None and len(tgt) >= 2:
                return float(tgt[0]), float(tgt[1]), 'explicit_target'

        targets2 = getattr(self, 'slide_transport_recenter_targets', None)
        key = self._slide_actor_key(rn)
        if isinstance(targets2, dict) and (key in targets2):
            tgt = targets2[key]
            if tgt is not None and len(tgt) >= 2:
                return float(tgt[0]), float(tgt[1]), 'explicit_target'

        center_ref = getattr(ctx, 'transport_center_ref', None)
        if center_ref is not None and len(center_ref) >= 2:
            return float(center_ref[0]), float(center_ref[1]), 'transport_center_ref'

        return None

    def _slide_transport_center_hold_speed(self, rn: str, ctx):
        enable = bool(getattr(self, 'slide_transport_recenter_enable', False))
        if not enable:
            return 0.0, 0.0

        if ctx.slide_pos is None:
            return 0.0, 0.0

        resolved = self._resolve_transport_center_target_xy_mm(rn, ctx)
        if resolved is None:
            return 0.0, 0.0

        target_x_mm, target_y_mm, target_src = resolved

        current_x_mm = float(ctx.slide_pos[0])
        current_y_mm = float(ctx.slide_pos[1])

        err_x_mm = target_x_mm - current_x_mm
        err_y_mm = target_y_mm - current_y_mm

        if not self._slide_axis_enabled(rn, 'x'):
            err_x_mm = 0.0
        if not self._slide_axis_enabled(rn, 'y'):
            err_y_mm = 0.0

        deadband_mm = float(getattr(self, 'slide_transport_recenter_xy_deadband_mm', 1.0))
        gain_mmps_per_mm = float(getattr(self, 'slide_transport_recenter_gain_mmps_per_mm', 0.5))
        limit_mmps = float(getattr(self, 'slide_transport_recenter_vxy_limit_mmps', 5.0))

        if abs(err_x_mm) <= deadband_mm:
            vx_hold = 0.0
        else:
            vx_hold = err_x_mm * gain_mmps_per_mm
            vx_hold = self._clamp(vx_hold, limit_mmps)

        if abs(err_y_mm) <= deadband_mm:
            vy_hold = 0.0
        else:
            vy_hold = err_y_mm * gain_mmps_per_mm
            vy_hold = self._clamp(vy_hold, limit_mmps)

        force_scale = self._transport_xy_force_scale(ctx)
        vx_hold *= force_scale
        vy_hold *= force_scale

        self._diag_transport_center_hold(
            rn=rn,
            ctx=ctx,
            target_src=target_src,
            target_x_mm=target_x_mm,
            target_y_mm=target_y_mm,
            current_x_mm=current_x_mm,
            current_y_mm=current_y_mm,
            err_x_mm=err_x_mm,
            err_y_mm=err_y_mm,
            vx_hold=vx_hold,
            vy_hold=vy_hold,
            force_scale=force_scale
        )

        return vx_hold, vy_hold

    def _slide_transport_feedforward_speed(self, rn: str, ctx):
        if not bool(getattr(self, 'transport_chassis_fusion_enable', False)):
            return 0.0, 0.0, 0.0
        if not bool(getattr(self, 'transport_ff_enable', False)):
            return 0.0, 0.0, 0.0

        twist = self._transport_chassis_twist_body(rn)
        if twist is None:
            return 0.0, 0.0, 0.0

        vx_body, vy_body, wz = twist

        vx_ff = -float(getattr(self, 'transport_ff_vx_gain_mmps_per_mps', 220.0)) * vx_body
        vy_ff = -float(getattr(self, 'transport_ff_vy_gain_mmps_per_mps', 220.0)) * vy_body

        if bool(getattr(self, 'transport_ff_yaw_enable', True)):
            lever = getattr(ctx, 'transport_center_ref', None)
            if lever is None and ctx.slide_pos is not None:
                lever = ctx.slide_pos
            if lever is not None:
                lever_x_m = float(lever[0]) / 1000.0
                lever_y_m = float(lever[1]) / 1000.0
                yaw_scale = float(getattr(self, 'transport_ff_yaw_gain_scale', 0.25))
                vx_ff += yaw_scale * (wz * lever_y_m * 1000.0)
                vy_ff += yaw_scale * (-wz * lever_x_m * 1000.0)

        vx_ff = self._clamp(vx_ff, self.slide_transport_vx_limit_mmps)
        vy_ff = self._clamp(vy_ff, self.slide_transport_vy_limit_mmps)
        self._diag_transport_ff(rn, ctx, vx_body, vy_body, wz, vx_ff, vy_ff)
        return vx_ff, vy_ff, 0.0

    # ------------------------------------------------------------------
    # direct position fallback / preferred alignment mode
    # ------------------------------------------------------------------
    def _direct_align_enable(self) -> bool:
        return bool(getattr(self, "slide_direct_align_enable", True))

    def _direct_align_speed_mmps(self) -> float:
        return float(getattr(self, "slide_direct_align_speed_mmps", 20.0))

    def _direct_align_min_time_sec(self) -> float:
        return float(getattr(self, "slide_direct_align_min_time_sec", 0.10))

    def _direct_align_settle_margin_sec(self) -> float:
        return float(getattr(self, "slide_direct_align_settle_margin_sec", 0.25))

    def _direct_align_fresh_timeout_sec(self) -> float:
        return float(getattr(self, "slide_direct_align_fresh_timeout_sec", 0.50))

    def _direct_align_trigger_wait_sec(self) -> float:
        return float(getattr(self, "slide_direct_align_trigger_wait_sec", 0.80))

    def _direct_align_max_retry(self) -> int:
        return int(getattr(self, "slide_direct_align_max_retry", 3))

    def _direct_align_pos_deadband_mm(self) -> float:
        return float(getattr(self, "slide_direct_align_pos_deadband_mm", 1.0))

    def _direct_align_xy_move_confirm_mm(self) -> float:
        return float(getattr(self, "slide_direct_align_xy_move_confirm_mm", 0.8))

    def _direct_align_xy_move_timeout_margin_sec(self) -> float:
        return float(getattr(self, "slide_direct_align_xy_move_timeout_margin_sec", 2.0))

    def _direct_align_ack_drop_timeout_sec(self) -> float:
        return max(0.20, float(getattr(self, "slide_direct_align_ack_drop_timeout_sec", 0.80)))

    def _direct_align_contact_seek_mm(self) -> float:
        return max(0.0, float(getattr(self, "slide_direct_align_contact_seek_mm", 140.0)))

    def _slide_pos_fresh(self, ctx) -> bool:
        stamp = float(getattr(ctx, "slide_pos_stamp", 0.0))
        if stamp <= 0.0:
            return False
        timeout = float(getattr(self, "slide_status_fresh_timeout_sec", 0.5))
        return (_now_sec(self) - stamp) <= timeout

    def _direct_align_retry_or_fail(self, rn: str, ctx, reason: str) -> bool:
        if ctx.direct_align_retry < self._direct_align_max_retry():
            ctx.direct_align_retry += 1
            ctx.direct_align_phase = "idle"
            ctx.direct_align_epoch = _now_sec(self)
            # Retry immediately on next cycle; no need to wait trigger window again.
            ctx.direct_align_started_at = ctx.direct_align_epoch - self._direct_align_trigger_wait_sec()
            ctx.direct_align_xy_start_pos = tuple(ctx.slide_pos) if getattr(ctx, "slide_pos", None) is not None else None
            self.get_logger().warn(
                f"[SLIDE_DIRECT_RETRY][{rn}] reason={reason} retry={ctx.direct_align_retry}/{self._direct_align_max_retry()}"
            )
            return True

        self._fail_robot(rn, reason)
        return False

    def _direct_ctx_init(self, ctx):
        if not hasattr(ctx, "direct_align_phase"):
            ctx.direct_align_phase = "idle"
        if not hasattr(ctx, "direct_align_retry"):
            ctx.direct_align_retry = 0
        if not hasattr(ctx, "direct_align_epoch"):
            ctx.direct_align_epoch = 0.0
        if not hasattr(ctx, "direct_align_xy_time"):
            ctx.direct_align_xy_time = 0.0
        if not hasattr(ctx, "direct_align_z_time"):
            ctx.direct_align_z_time = 0.0
        if not hasattr(ctx, "direct_align_pending_z_mm"):
            ctx.direct_align_pending_z_mm = 0.0
        if not hasattr(ctx, "direct_align_ready_dz_m"):
            ctx.direct_align_ready_dz_m = 0.0
        if not hasattr(ctx, "direct_align_started_at"):
            ctx.direct_align_started_at = 0.0
        if not hasattr(ctx, "direct_align_xy_start_pos"):
            ctx.direct_align_xy_start_pos = None
        if not hasattr(ctx, "direct_align_xy_cmd_x_mm"):
            ctx.direct_align_xy_cmd_x_mm = 0.0
        if not hasattr(ctx, "direct_align_xy_cmd_y_mm"):
            ctx.direct_align_xy_cmd_y_mm = 0.0
        if not hasattr(ctx, "direct_align_z_cmd_mm"):
            ctx.direct_align_z_cmd_mm = 0.0
        if not hasattr(ctx, "direct_align_positive_z_attempted"):
            ctx.direct_align_positive_z_attempted = False
        if not hasattr(ctx, "direct_align_small_residual_count"):
            ctx.direct_align_small_residual_count = 0
        if not hasattr(ctx, "direct_align_z_done"):
            ctx.direct_align_z_done = False
        if not hasattr(ctx, "direct_align_force_monitor_latched"):
            ctx.direct_align_force_monitor_latched = False
        if not hasattr(ctx, "direct_align_force_contact_latched"):
            ctx.direct_align_force_contact_latched = False
        if not hasattr(ctx, "direct_align_force_contact_count"):
            ctx.direct_align_force_contact_count = 0
        if not hasattr(ctx, "direct_align_force_last_eval_stamp"):
            ctx.direct_align_force_last_eval_stamp = 0.0
        if not hasattr(ctx, "direct_align_post_contact_hold_start"):
            ctx.direct_align_post_contact_hold_start = 0.0
        if not hasattr(ctx, "direct_align_post_contact_z_ref_mm"):
            ctx.direct_align_post_contact_z_ref_mm = None
        if not hasattr(ctx, "direct_align_xy_ack_drop_seen"):
            ctx.direct_align_xy_ack_drop_seen = False

    def _direct_align_xy_moved(self, rn: str, ctx) -> bool:
        start = getattr(ctx, "direct_align_xy_start_pos", None)
        cur = getattr(ctx, "slide_pos", None)
        if start is None or cur is None:
            return False

        confirm_mm = self._direct_align_xy_move_confirm_mm()
        sx, sy, _ = start
        cx, cy, _ = cur
        ex = abs(float(cx) - float(sx))
        ey = abs(float(cy) - float(sy))

        need_x = abs(float(getattr(ctx, "direct_align_xy_cmd_x_mm", 0.0))) >= self._direct_align_pos_deadband_mm()
        need_y = abs(float(getattr(ctx, "direct_align_xy_cmd_y_mm", 0.0))) >= self._direct_align_pos_deadband_mm()

        x_ok = (not need_x) or (ex >= confirm_mm)
        y_ok = (not need_y) or (ey >= confirm_mm)
        return x_ok and y_ok

    def _direct_align_contact_force_sign(self) -> float:
        return self._norm_sign(getattr(self, "slide_direct_align_contact_force_sign", 1.0))

    def _direct_align_contact_frames(self) -> int:
        return max(1, int(getattr(self, "slide_direct_align_contact_frames", 3)))

    def _direct_align_post_contact_hold_short_sec(self) -> float:
        return max(0.05, float(getattr(self, "slide_direct_align_post_contact_hold_sec", 0.50)))

    def _direct_align_force_fresh_timeout_sec(self) -> float:
        return float(getattr(self, "load_level_force_fresh_timeout_sec", 0.40))

    def _direct_align_contact_takeover_grace_sec(self) -> float:
        return max(self._direct_align_force_fresh_timeout_sec(), 0.25)

    def _direct_align_passive_z_wait_timeout_sec(self) -> float:
        return max(
            self._direct_align_force_fresh_timeout_sec(),
            self._direct_align_settle_margin_sec(),
            0.40,
        )

    def _direct_align_contact_authority_active(self, ctx) -> bool:
        return bool(
            getattr(ctx, "direct_align_force_monitor_latched", False)
            or getattr(ctx, "direct_align_force_contact_latched", False)
            or getattr(ctx, "contact_confirmed", False)
        )

    def _direct_align_positive_z_blocked(self, ctx, z_mm: float) -> bool:
        return (
            float(z_mm) > self._direct_align_pos_deadband_mm()
            and self._direct_align_contact_authority_active(ctx)
        )

    def _direct_align_repeat_positive_z_blocked(self, ctx, z_mm: float) -> bool:
        return (
            float(z_mm) > self._direct_align_pos_deadband_mm()
            and bool(getattr(ctx, "direct_align_positive_z_attempted", False))
            and (not self._direct_align_contact_authority_active(ctx))
        )

    def _direct_align_contact_takeover_grace_active(self, ctx, now: float) -> bool:
        if not bool(getattr(self, "slide_direct_align_contact_enable", False)):
            return False
        if bool(getattr(ctx, "direct_align_z_done", False)):
            return False
        return self._direct_align_contact_authority_active(ctx)

    def _direct_align_z_cmd_sign(self, ctx) -> float:
        z_cmd_mm = float(getattr(ctx, "direct_align_z_cmd_mm", 0.0))
        if abs(z_cmd_mm) < self._direct_align_pos_deadband_mm():
            return 1.0
        return 1.0 if z_cmd_mm > 0.0 else -1.0

    def _direct_align_force_sample_ok(self, rn: str, ctx):
        ff = getattr(ctx, "force_f", None)
        fz = float(ff[2]) if ff is not None else 0.0
        if ff is None:
            return False, "force_missing", fz, 0.0, 0.0
        if not self._force_msg_fresh(rn, self._direct_align_force_fresh_timeout_sec()):
            return False, "force_stale", fz, 0.0, 0.0

        z_cmd_mm = float(getattr(ctx, "direct_align_z_cmd_mm", 0.0))
        if abs(z_cmd_mm) < self._direct_align_pos_deadband_mm():
            return False, "z_cmd_inactive", fz, 0.0, 0.0

        cmd_sign = self._direct_align_z_cmd_sign(ctx)
        contact_force_th = float(getattr(self, "load_level_contact_force_n", 18.0))
        if (
            bool(getattr(ctx, "direct_align_force_monitor_latched", False))
            or bool(getattr(ctx, "direct_align_force_contact_latched", False))
            or bool(getattr(ctx, "contact_confirmed", False))
        ):
            signed_force = abs(fz)
            if signed_force < contact_force_th:
                return False, "latched_force_below_threshold", fz, signed_force, cmd_sign
            return True, "latched_force_ok", fz, signed_force, cmd_sign

        signed_force = cmd_sign * self._direct_align_contact_force_sign() * fz
        if signed_force <= 0.0:
            return False, "wrong_force_direction", fz, signed_force, cmd_sign
        if signed_force < contact_force_th:
            return False, "force_below_threshold", fz, signed_force, cmd_sign
        return True, "force_ok", fz, signed_force, cmd_sign

    def _freeze_positive_pending_z_mm(self, rn: str, ctx):
        pending_z_mm = float(getattr(ctx, "direct_align_pending_z_mm", 0.0))
        if pending_z_mm <= self._direct_align_pos_deadband_mm():
            return
        ctx.direct_align_pending_z_mm = 0.0
        if self._diag_ok(ctx, "_diag_slide_positive_z_frozen_epoch", 0.5):
            self.get_logger().warn(
                f"[Z_STOP_BY_FORCE][{rn}] freeze positive pending_z_mm "
                f"from {pending_z_mm:.1f}mm to 0.0mm after contact latch"
            )

    def _enter_direct_align_post_contact_hold(self, rn: str, ctx, source: str, fz: float, signed_force: float):
        now = _now_sec(self)
        newly_latched = not bool(getattr(ctx, "direct_align_force_contact_latched", False))
        if "force_monitor" in str(source):
            ctx.direct_align_force_monitor_latched = True
            ctx.direct_align_force_contact_count = max(
                int(getattr(ctx, "direct_align_force_contact_count", 0)),
                self._direct_align_contact_frames(),
            )
        ctx.direct_align_force_contact_latched = True
        ctx.contact_confirmed = True
        if float(getattr(ctx, "force_contact_epoch", 0.0)) <= 0.0:
            ctx.force_contact_epoch = now
        if float(getattr(ctx, "direct_align_post_contact_hold_start", 0.0)) <= 0.0:
            ctx.direct_align_post_contact_hold_start = now
        if getattr(ctx, "direct_align_post_contact_z_ref_mm", None) is None and getattr(ctx, "slide_pos", None) is not None:
            ctx.direct_align_post_contact_z_ref_mm = float(ctx.slide_pos[2])

        self._freeze_positive_pending_z_mm(rn, ctx)
        self.stop_slide_position(rn)
        self.stop_slide_comp(rn)
        self._latch_post_contact_state(rn, ctx, source=source)
        ctx.direct_align_phase = "post_contact_hold"
        ctx.direct_align_epoch = now

        if newly_latched:
            self.get_logger().warn(
                f"[FORCE_CONTACT_LATCHED][{rn}] source={source} "
                f"frames={int(getattr(ctx, 'direct_align_force_contact_count', 0))}/{self._direct_align_contact_frames()} "
                f"fz={fz:.2f} signed_fz={signed_force:.2f} "
                f"z_cmd_mm={float(getattr(ctx, 'direct_align_z_cmd_mm', 0.0)):.1f}"
            )
            self.get_logger().warn(
                f"[Z_STOP_BY_FORCE][{rn}] phase=post_contact_hold "
                f"stop positive Z after directional force latch"
            )

    def _handle_force_monitor_contact_event(self, rn: str, ctx) -> bool:
        phase = str(getattr(ctx, "direct_align_phase", ""))
        z_cmd_mm = float(getattr(ctx, "direct_align_z_cmd_mm", 0.0))
        pending_z_mm = float(getattr(ctx, "direct_align_pending_z_mm", 0.0))
        z_relevant = (
            phase in ("z_sent", "post_z_wait_contact_or_small_residual", "done", "post_contact_hold", "xy_barrier", "ready_barrier")
            or abs(z_cmd_mm) >= self._direct_align_pos_deadband_mm()
            or pending_z_mm > self._direct_align_pos_deadband_mm()
        )
        if not z_relevant:
            return False

        ff = getattr(ctx, "force_f", None)
        fz = float(ff[2]) if ff is not None else 0.0
        self._enter_direct_align_post_contact_hold(
            rn,
            ctx,
            source="force_monitor_topic",
            fz=fz,
            signed_force=abs(fz),
        )
        if self._diag_ok(ctx, "_diag_force_monitor_takeover_epoch", 0.5):
            self.get_logger().warn(
                f"[FORCE_CONTACT_TOPIC][{rn}] takeover phase={phase} "
                f"z_cmd_mm={z_cmd_mm:.1f} pending_z_mm={pending_z_mm:.1f}"
            )
        return True

    def _update_direct_align_force_contact_latch(self, rn: str, ctx) -> bool:
        if not bool(getattr(self, "slide_direct_align_contact_enable", False)):
            return False

        if bool(getattr(ctx, "direct_align_force_monitor_latched", False)):
            if str(getattr(ctx, "direct_align_phase", "")) != "post_contact_hold":
                ff = getattr(ctx, "force_f", None)
                fz = float(ff[2]) if ff is not None else 0.0
                self._enter_direct_align_post_contact_hold(
                    rn,
                    ctx,
                    source="force_monitor_topic",
                    fz=fz,
                    signed_force=abs(fz),
                )
            return True

        if bool(getattr(ctx, "direct_align_force_contact_latched", False)):
            phase = str(getattr(ctx, "direct_align_phase", ""))
            if phase != "post_contact_hold":
                ff = getattr(ctx, "force_f", None)
                fz = float(ff[2]) if ff is not None else 0.0
                signed_force = abs(fz)
                self._enter_direct_align_post_contact_hold(
                    rn,
                    ctx,
                    source="force_sensor_latched",
                    fz=fz,
                    signed_force=signed_force,
                )
            else:
                self._latch_post_contact_state(rn, ctx, source="force_sensor_latched")
            return True

        sample_stamp = float(getattr(ctx, "force_stamp", 0.0))
        if sample_stamp <= 0.0:
            return False

        sample_ok, reason, fz, signed_force, _ = self._direct_align_force_sample_ok(rn, ctx)
        last_eval_stamp = float(getattr(ctx, "direct_align_force_last_eval_stamp", 0.0))
        if sample_stamp > last_eval_stamp:
            ctx.direct_align_force_last_eval_stamp = sample_stamp
            if sample_ok:
                ctx.direct_align_force_contact_count = int(getattr(ctx, "direct_align_force_contact_count", 0)) + 1
            else:
                ctx.direct_align_force_contact_count = 0

        if (not sample_ok) and self._diag_ok(ctx, "_diag_slide_force_contact_wait_epoch", 0.5):
            self.get_logger().warn(
                f"[FORCE_CONTACT_WAIT][{rn}] reason={reason} "
                f"fz={fz:.2f} signed_fz={signed_force:.2f} "
                f"count={int(getattr(ctx, 'direct_align_force_contact_count', 0))}/{self._direct_align_contact_frames()}"
            )

        if int(getattr(ctx, "direct_align_force_contact_count", 0)) < self._direct_align_contact_frames():
            return False

        self._enter_direct_align_post_contact_hold(rn, ctx, source="force_sensor", fz=fz, signed_force=signed_force)
        return True

    def _direct_align_post_contact_hold_ready(self, rn: str, ctx) -> bool:
        if not bool(getattr(ctx, "direct_align_force_contact_latched", False)):
            return False

        now = _now_sec(self)
        elapsed = now - float(getattr(ctx, "direct_align_post_contact_hold_start", 0.0))
        hold_sec = self._direct_align_post_contact_hold_short_sec()
        force_ok, force_reason, fz, signed_force, cmd_sign = self._direct_align_force_sample_ok(rn, ctx)

        z_ref_mm = getattr(ctx, "direct_align_post_contact_z_ref_mm", None)
        pos_fresh = self._slide_pos_fresh(ctx)
        worsen_mm = 0.0
        pos_ok = False
        pos_reason = "slide_pos_missing"
        if z_ref_mm is not None and getattr(ctx, "slide_pos", None) is not None and pos_fresh:
            worsen_mm = cmd_sign * (float(ctx.slide_pos[2]) - float(z_ref_mm))
            pos_ok = worsen_mm <= self._direct_align_pos_deadband_mm()
            pos_reason = "pos_ok" if pos_ok else "z_worsened_after_contact"
        elif not pos_fresh:
            pos_reason = "slide_pos_stale"

        if self._diag_ok(ctx, "_diag_post_contact_hold_wait_epoch", 0.5):
            self.get_logger().warn(
                f"[POST_CONTACT_HOLD][{rn}] elapsed={elapsed:.2f}/{hold_sec:.2f}s "
                f"force_ok={int(bool(force_ok))} force_reason={force_reason} "
                f"pos_ok={int(bool(pos_ok))} pos_reason={pos_reason} "
                f"worsen_mm={worsen_mm:.2f} fz={fz:.2f} signed_fz={signed_force:.2f}"
            )

        if elapsed < hold_sec:
            return False
        if ctx.faulted:
            return False
        if not force_ok:
            return False
        if not pos_ok:
            return False

        ctx.direct_align_z_done = True
        self.get_logger().warn(
            f"[POST_CONTACT_HOLD][{rn}] stable -> z_done=1 "
            f"elapsed={elapsed:.2f}s worsen_mm={worsen_mm:.2f}"
        )
        return True

    def _direct_align_contact_ok(self, rn: str, ctx) -> bool:
        return bool(self._update_direct_align_force_contact_latch(rn, ctx))

    def _direct_align_exit_passive_z_by_force_contact(self, rn: str, ctx, now: float) -> bool:
        if not self._direct_align_contact_authority_active(ctx):
            return False

        if str(getattr(ctx, "direct_align_phase", "")) != "post_contact_hold":
            ff = getattr(ctx, "force_f", None)
            fz = float(ff[2]) if ff is not None else 0.0
            signed_force = abs(fz)
            self._enter_direct_align_post_contact_hold(
                rn,
                ctx,
                source="passive_z_force_contact",
                fz=fz,
                signed_force=signed_force,
            )

        ctx.direct_align_epoch = now
        if self._diag_ok(ctx, "_diag_passive_z_exit_force_epoch", 0.5):
            self.get_logger().warn(
                f"[PASSIVE_Z_SETTLE][{rn}] exit by force_contact "
                f"next_phase={getattr(ctx, 'direct_align_phase', '')}"
            )
        return True

    def _direct_align_passive_z_small_residual_tol(self, ctx):
        base_tol_m = float(self.fine_z_tol_m)
        z_cmd_abs_m = abs(float(getattr(ctx, "direct_align_z_cmd_mm", 0.0))) / 1000.0
        cmd_scaled_tol_m = min(
            z_cmd_abs_m,
            min(
                0.020,
                max(0.015, 0.12 * z_cmd_abs_m),
            ),
        )
        tol_m = max(base_tol_m, cmd_scaled_tol_m)
        source = (
            f"max(fine_z_tol_m={base_tol_m:.4f},"
            f"passive_cmd_scaled={cmd_scaled_tol_m:.4f},"
            f"z_cmd_abs_m={z_cmd_abs_m:.4f})"
        )
        return tol_m, source

    def _direct_align_passive_z_small_residual_ready(self, rn: str, ctx):
        fresh = self._direct_align_fresh_delta(rn, ctx)
        if fresh is None:
            ctx.direct_align_small_residual_count = 0
            return False, None, 'delta_stale'

        dz = float(fresh[2])
        tol_m, tol_source = self._direct_align_passive_z_small_residual_tol(ctx)
        z_ok = self._axis_ok_or_disabled(rn, 'z', dz, tol_m)
        prev_count = int(getattr(ctx, 'direct_align_small_residual_count', 0))
        if z_ok:
            ctx.direct_align_small_residual_count = prev_count + 1
        else:
            ctx.direct_align_small_residual_count = 0
        cur_count = int(getattr(ctx, 'direct_align_small_residual_count', 0))
        req_count = max(1, int(self.fine_xy_stable_frames))

        if self._diag_ok(ctx, "_diag_passive_z_wait_epoch", 0.5):
            self.get_logger().warn(
                f"[PASSIVE_Z_SETTLE][{rn}] dz={dz:.4f} "
                f"z_ok={int(bool(z_ok))} "
                f"tol={tol_m:.4f} tol_source={tol_source} "
                f"stable={prev_count}->{cur_count}/{req_count}"
            )

        ready = cur_count >= req_count
        return bool(ready), dz, ('small_residual_ready' if ready else 'small_residual_wait')

    def _direct_align_fresh_delta(self, rn: str, ctx):
        stable = self._get_stable_delta(rn)
        if stable is not None:
            dx, dy, dz = stable
            return float(dx), float(dy), float(dz)

        if ctx.delta_latest is None or ctx.last_delta_stamp is None:
            return None

        age = _now_sec(self) - float(ctx.last_delta_stamp)
        if age > self._direct_align_fresh_timeout_sec():
            return None

        dx, dy, dz = ctx.delta_latest
        return float(dx), float(dy), float(dz)

    def _direct_align_compute_time(self, x_mm: float, y_mm: float, z_mm: float) -> float:
        vmax = max(1.0, self._direct_align_speed_mmps())
        max_dist = max(abs(x_mm), abs(y_mm), abs(z_mm))
        move_time = max_dist / vmax
        return max(self._direct_align_min_time_sec(), move_time)

    def _post_contact_hold_sec(self) -> float:
        return float(getattr(self, "slide_post_contact_hold_sec", 5.0))

    def _recent_align_delta(self, rn: str, ctx, max_age_sec: float):
        stable = self._get_stable_delta(rn)
        if stable is not None:
            return stable

        if ctx.delta_latest is None or ctx.last_delta_stamp is None:
            return None

        age = _now_sec(self) - float(ctx.last_delta_stamp)
        if age > max_age_sec:
            return None

        dx, dy, dz = ctx.delta_latest
        return float(dx), float(dy), float(dz)

    def _align_xy_ok_from_delta(self, rn: str, delta_xyz) -> bool:
        dx, dy, _ = delta_xyz
        x_ok = self._axis_ok_or_disabled(rn, 'x', dx, self.fine_xy_tol_m)
        y_ok = self._axis_ok_or_disabled(rn, 'y', dy, self.fine_xy_tol_m)
        return x_ok and y_ok

    def _latch_post_contact_state(self, rn: str, ctx, source: str):
        now = _now_sec(self)
        newly_latched = not bool(getattr(ctx, "post_contact_latched", False))
        if newly_latched:
            ctx.post_contact_latched = True
            ctx.post_contact_epoch = now

        delta = self._recent_align_delta(rn, ctx, self._post_contact_hold_sec())
        if delta is not None:
            prev = getattr(ctx, "post_contact_delta", None)
            if prev is None or self._align_xy_ok_from_delta(rn, delta):
                ctx.post_contact_delta = delta

        if newly_latched and self._diag_ok(ctx, "_diag_post_contact_epoch", 0.5):
            delta = getattr(ctx, "post_contact_delta", None)
            delta_str = f"({delta[0]:.4f},{delta[1]:.4f},{delta[2]:.4f})" if delta is not None else "None"
            self.get_logger().warn(
                f"[SLIDE_POST_CONTACT][{rn}] source={source} hold={self._post_contact_hold_sec():.2f}s "
                f"snapshot_delta={delta_str}"
            )

    def _post_contact_ready_delta(self, rn: str, ctx):
        if not bool(getattr(ctx, "post_contact_latched", False)):
            return None

        epoch = float(getattr(ctx, "post_contact_epoch", 0.0))
        if epoch <= 0.0:
            return None
        if (_now_sec(self) - epoch) > self._post_contact_hold_sec():
            return None

        delta = self._recent_align_delta(rn, ctx, self._post_contact_hold_sec())
        if delta is not None and self._align_xy_ok_from_delta(rn, delta):
            ctx.post_contact_delta = delta
            return delta

        delta = getattr(ctx, "post_contact_delta", None)
        if delta is None:
            return None
        if self._align_xy_ok_from_delta(rn, delta):
            return tuple(float(v) for v in delta)
        return None

    def _complete_post_contact_ready(self, rn: str, ctx, reason: str) -> bool:
        delta = self._post_contact_ready_delta(rn, ctx)
        if delta is None:
            return False

        dx, dy, dz = delta
        self.stop_slide_position(rn)
        self.stop_slide_comp(rn)
        if self._diag_ok(ctx, "_diag_post_contact_ready_epoch", 0.5):
            age = _now_sec(self) - float(getattr(ctx, "post_contact_epoch", 0.0))
            self.get_logger().warn(
                f"[SLIDE_POST_CONTACT_READY][{rn}] reason={reason} "
                f"delta=({dx:.4f},{dy:.4f},{dz:.4f}) age={age:.2f}s"
            )
        if self.state == 'SYNC_SLIDE_ALIGN':
            self._sync_slide_enter_ready_barrier(rn, ctx, dz, reason=f'post_contact:{reason}')
            if self._sync_slide_wait_for_ready_barrier(rn, source='post_contact_ready'):
                self._mark_ready_to_lift(rn, dz)
            return True
        self._mark_ready_to_lift(rn, dz)
        return True

    def _start_direct_align_z(self, rn: str, ctx, z_mm: float, now: float) -> bool:
        if self._direct_align_positive_z_blocked(ctx, z_mm):
            self._freeze_positive_pending_z_mm(rn, ctx)
            if self._diag_ok(ctx, "_diag_slide_block_positive_z_epoch", 0.5):
                self.get_logger().warn(
                    f"[Z_STOP_BY_FORCE][{rn}] skip positive Z reissue "
                    f"requested_z_mm={float(z_mm):.1f} phase={getattr(ctx, 'direct_align_phase', '')}"
                )
            ctx.direct_align_phase = "post_contact_hold"
            ctx.direct_align_epoch = now
            return True

        z_time = self._direct_align_compute_time(0.0, 0.0, z_mm)

        ctx.slide_reached = False
        ctx.direct_align_z_cmd_mm = float(z_mm)
        if float(z_mm) > self._direct_align_pos_deadband_mm():
            ctx.direct_align_positive_z_attempted = True
        ctx.direct_align_small_residual_count = 0
        ctx.direct_align_z_done = False
        ctx.direct_align_force_monitor_latched = False
        ctx.direct_align_force_contact_latched = False
        ctx.direct_align_force_contact_count = 0
        ctx.direct_align_force_last_eval_stamp = 0.0
        ctx.direct_align_post_contact_hold_start = 0.0
        ctx.direct_align_post_contact_z_ref_mm = None
        self.send_slide_position(
            rn,
            0.0,
            0.0,
            z_mm,
            z_time,
            is_relative=True
        )
        self._diag_direct_pos_cmd(rn, "z", 0.0, 0.0, z_mm, z_time)

        ctx.direct_align_phase = "z_sent"
        ctx.direct_align_epoch = now
        ctx.direct_align_z_time = z_time
        ctx.direct_align_xy_cmd_x_mm = 0.0
        ctx.direct_align_xy_cmd_y_mm = 0.0
        return True

    def _remaining_contact_seek_mm(self, ctx) -> float:
        if ctx.slide_pos is None:
            return 0.0
        z_max = float(getattr(self, "load_level_z_plane_max_mm", 180.0))
        cur_z = float(ctx.slide_pos[2])
        return max(0.0, z_max - cur_z)

    def _direct_align_z_only_contact_step(self, rn: str) -> bool:
        ctx = self.rt[rn]
        self._direct_ctx_init(ctx)

        if not self._direct_align_enable():
            return False

        now = _now_sec(self)

        need_slide_pos = (ctx.direct_align_phase == "idle")
        if need_slide_pos and (ctx.slide_pos is None or (not self._slide_pos_fresh(ctx))):
            if self._diag_ok(ctx, "_diag_slide_no_pos_epoch", 1.0):
                self.get_logger().warn(
                    f"[SLIDE_Z_ONLY_WAIT][{rn}] slide_pos not ready; delay z-only contact seek"
                )
            return False

        if ctx.direct_align_phase == "idle":
            if ctx.direct_align_started_at <= 0.0:
                ctx.direct_align_started_at = now

            if (now - ctx.direct_align_started_at) < self._direct_align_trigger_wait_sec():
                return True

            remaining_mm = self._remaining_contact_seek_mm(ctx)
            seek_mm = min(self._direct_align_contact_seek_mm(), remaining_mm)
            if seek_mm < self._direct_align_pos_deadband_mm():
                self._fail_robot(rn, "DIRECT_ALIGN_NO_CONTACT_BEFORE_Z_LIMIT")
                return True

            return self._start_direct_align_z(rn, ctx, seek_mm, now)

        if ctx.direct_align_phase == "z_sent":
            elapsed_z = now - ctx.direct_align_epoch
            min_guard = min(1.0, max(0.15, 0.3 * max(0.0, ctx.direct_align_z_time)))
            if elapsed_z < min_guard:
                return True

            if not self._slide_pos_fresh(ctx):
                if self._diag_ok(ctx, "_diag_slide_z_status_stale_epoch", 1.0):
                    self.get_logger().warn(
                        f"[SLIDE_Z_ONLY_INFLIGHT][{rn}] slide_pos stale during z seek; "
                        f"keep waiting for contact or command timeout"
                    )

            if self._direct_align_contact_ok(rn, ctx):
                return True

            expected_finish = ctx.direct_align_z_time + self._direct_align_settle_margin_sec()
            if bool(getattr(ctx, "slide_reached", False)) and elapsed_z < expected_finish:
                if self._diag_ok(ctx, "_diag_slide_z_ack_early_epoch", 1.0):
                    self.get_logger().warn(
                        f"[SLIDE_Z_ONLY_ACK_EARLY][{rn}] reached_target=True at {elapsed_z:.2f}s "
                        f"before expected_finish={expected_finish:.2f}s; ignore early ack and keep waiting for contact"
                    )

            # In z-only loading, some slide drivers raise reached_target early while the
            # actuator is still moving. Waiting for the commanded duration is safer than
            # trusting the early ack, otherwise we can repeatedly resend Z and abort
            # before the physical contact actually happens.
            z_done = elapsed_z >= expected_finish
            if not z_done:
                return True

            ctx.direct_align_phase = "post_z_wait_contact_or_small_residual"
            ctx.direct_align_epoch = now
            ctx.direct_align_small_residual_count = 0
            if self._diag_ok(ctx, "_diag_passive_z_enter_epoch", 0.5):
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] entered passive Z settle window "
                    f"z_cmd_mm={float(getattr(ctx, 'direct_align_z_cmd_mm', 0.0)):.1f} "
                    f"timeout={self._direct_align_passive_z_wait_timeout_sec():.2f}s"
                )
            return True

        if ctx.direct_align_phase == "post_z_wait_contact_or_small_residual":
            if self._direct_align_contact_ok(rn, ctx):
                return self._direct_align_exit_passive_z_by_force_contact(rn, ctx, now)
            if self._direct_align_contact_authority_active(ctx):
                return self._direct_align_exit_passive_z_by_force_contact(rn, ctx, now)

            ready, dz, _ = self._direct_align_passive_z_small_residual_ready(rn, ctx)
            if ready:
                self.stop_slide_position(rn)
                self.stop_slide_comp(rn)
                ctx.direct_align_z_done = True
                ctx.direct_align_phase = "ready_barrier"
                ctx.direct_align_epoch = now
                self._sync_slide_enter_ready_barrier(rn, ctx, float(dz or 0.0), reason='z_only_small_residual')
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] exited by small residual dz={float(dz or 0.0):.4f}"
                )
                return True

            if (now - float(getattr(ctx, 'direct_align_epoch', 0.0))) >= self._direct_align_passive_z_wait_timeout_sec():
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] exited by timeout -> DIRECT_ALIGN_NOT_CONVERGED"
                )
                self._fail_robot(rn, "DIRECT_ALIGN_NOT_CONVERGED")
            return True

        if ctx.direct_align_phase == "post_contact_hold":
            if not self._direct_align_post_contact_hold_ready(rn, ctx):
                return True
            self.stop_slide_position(rn)
            self.stop_slide_comp(rn)
            ctx.direct_align_phase = "ready_barrier"
            ctx.direct_align_epoch = now
            self._sync_slide_enter_ready_barrier(rn, ctx, 0.0, reason='z_only_post_contact_hold')
            return True

        if ctx.direct_align_phase == "ready_barrier":
            dz = float(getattr(ctx, 'direct_align_ready_dz_m', 0.0))
            if self._sync_slide_wait_for_ready_barrier(rn, source="z_only_ready_barrier"):
                self._reset_direct_align_state(ctx)
                if self._diag_ok(ctx, "_diag_slide_z_only_ready_epoch", 0.5):
                    self.get_logger().warn(
                        f"[SLIDE_Z_ONLY_READY][{rn}] post-contact hold stable; alignment vision disabled, accept z-only loading"
                    )
                self._mark_ready_to_lift(rn, dz)
            return True

        return True

    def _direct_align_try_start(self, rn: str, ctx) -> bool:
        if ctx.slide_pos is None or (not self._slide_pos_fresh(ctx)):
            if self._diag_ok(ctx, "_diag_slide_no_pos_epoch", 1.0):
                self.get_logger().warn(
                    f"[SLIDE_DIRECT_WAIT][{rn}] slide_pos not ready; delay XY start"
                )
            return False

        fresh = self._direct_align_fresh_delta(rn, ctx)
        if fresh is None:
            return False

        dx, dy, dz = self._mask_delta_for_robot(rn, *fresh)
        x_mm, y_mm, z_mm = self._delta_to_slide_rel_mm(dx, dy, dz)
        x_ok = self._axis_ok_or_disabled(rn, 'x', dx, self.fine_xy_tol_m)
        y_ok = self._axis_ok_or_disabled(rn, 'y', dy, self.fine_xy_tol_m)
        z_ok = self._axis_ok_or_disabled(rn, 'z', dz, self.fine_z_tol_m)
        xy_ok = x_ok and y_ok
        z_ok = z_ok or bool(getattr(ctx, "direct_align_z_done", False))

        if xy_ok and z_ok:
            self.stop_slide_comp(rn)
            self._sync_slide_enter_ready_barrier(rn, ctx, dz, reason='xy_and_z_already_ok')
            if self._sync_slide_wait_for_ready_barrier(rn, source='xy_and_z_already_ok'):
                self._mark_ready_to_lift(rn, dz)
            return True

        if xy_ok:
            if self._direct_align_repeat_positive_z_blocked(ctx, z_mm):
                if self._diag_ok(ctx, "_diag_slide_repeat_positive_z_block_epoch", 0.5):
                    self.get_logger().warn(
                        f"[SLIDE_DIRECT_Z_REISSUE_BLOCKED][{rn}] "
                        f"requested_z_mm={float(z_mm):.1f} "
                        f"previous_z_cmd_mm={float(getattr(ctx, 'direct_align_z_cmd_mm', 0.0)):.1f} "
                        f"latched=0 retry={int(getattr(ctx, 'direct_align_retry', 0))}"
                    )
                ctx.direct_align_phase = "done"
                ctx.direct_align_epoch = _now_sec(self)
                return True
            if self._direct_align_positive_z_blocked(ctx, z_mm):
                self._freeze_positive_pending_z_mm(rn, ctx)
                if self._diag_ok(ctx, "_diag_slide_xy_block_positive_z_epoch", 0.5):
                    self.get_logger().warn(
                        f"[Z_STOP_BY_FORCE][{rn}] block xy_already_ok -> phase=z "
                        f"requested_z_mm={float(z_mm):.1f} after contact latch"
                    )
                ctx.direct_align_phase = "post_contact_hold"
                ctx.direct_align_epoch = _now_sec(self)
                return True
        ctx.direct_align_pending_z_mm = z_mm
        ctx.direct_align_started_at = _now_sec(self)
        self._sync_slide_enter_xy_barrier(rn, ctx, z_mm, reason="xy_already_ok")
        if self._sync_slide_wait_for_xy_barrier(rn, source="xy_already_ok"):
            return self._start_direct_align_z(rn, ctx, z_mm, _now_sec(self))
            return True

        if (
            abs(x_mm) < self._direct_align_pos_deadband_mm() and
            abs(y_mm) < self._direct_align_pos_deadband_mm() and
            abs(z_mm) < self._direct_align_pos_deadband_mm()
        ):
            return False

        xy_time = self._direct_align_compute_time(x_mm, y_mm, 0.0)

        # Do NOT publish compensation-stop here.
        # reason: compensation and position commands are on different topics, and ROS delivery order
        # across topics is not guaranteed. A late compensation stop may cancel the just-sent XY position
        # command, which manifests as "only Z moves while XY stays still".
        # 防止沿用上一条任务残留的 reached=True 导致“秒完成”误判
        ctx.slide_reached = False
        ctx.direct_align_xy_ack_drop_seen = False
        self.send_slide_position(rn, x_mm, y_mm, 0.0, xy_time, is_relative=True)
        self._diag_direct_pos_cmd(rn, "xy", x_mm, y_mm, 0.0, xy_time)

        ctx.direct_align_phase = "xy_sent"
        ctx.direct_align_epoch = _now_sec(self)
        ctx.direct_align_xy_time = xy_time
        ctx.direct_align_pending_z_mm = z_mm
        ctx.direct_align_started_at = _now_sec(self)
        ctx.direct_align_xy_start_pos = tuple(ctx.slide_pos)
        ctx.direct_align_xy_cmd_x_mm = float(x_mm)
        ctx.direct_align_xy_cmd_y_mm = float(y_mm)
        self._sync_slide_trace_xy_cmd_sent(rn, ctx, x_mm, y_mm, xy_time)

        return True

    def _direct_align_step(self, rn: str) -> bool:
        ctx = self.rt[rn]
        self._direct_ctx_init(ctx)

        if not self._direct_align_enable():
            return False

        now = _now_sec(self)

        if ctx.direct_align_phase == "done":
            fresh = self._direct_align_fresh_delta(rn, ctx)
            if fresh is None:
                if self._complete_post_contact_ready(rn, ctx, 'stale_delta_after_contact'):
                    return True
                return True

            dx, dy, dz = fresh
            x_ok = self._axis_ok_or_disabled(rn, 'x', dx, self.fine_xy_tol_m)
            y_ok = self._axis_ok_or_disabled(rn, 'y', dy, self.fine_xy_tol_m)
            z_ok = self._axis_ok_or_disabled(rn, 'z', dz, self.fine_z_tol_m)
            if bool(getattr(ctx, "direct_align_z_done", False)) and (not z_ok):
                if self._diag_ok(ctx, "_diag_slide_contact_ready_epoch", 1.0):
                    self.get_logger().warn(
                        f"[SLIDE_DIRECT_CONTACT_READY][{rn}] post-contact hold complete; accept Z without QR convergence"
                    )
            z_ok = z_ok or bool(getattr(ctx, "direct_align_z_done", False))
            xy_ok = x_ok and y_ok

            if xy_ok and z_ok:
                self.stop_slide_comp(rn)
                self._reset_direct_align_state(ctx)
                self._sync_slide_enter_ready_barrier(rn, ctx, dz, reason='direct_align_done')
                if self._sync_slide_wait_for_ready_barrier(rn, source='direct_align_done'):
                    self._mark_ready_to_lift(rn, dz)
                return True

            if self._direct_align_contact_takeover_grace_active(ctx, now):
                if self._diag_ok(ctx, "_diag_direct_align_contact_grace_epoch", 0.5):
                    self.get_logger().warn(
                        f"[DIRECT_ALIGN_CONTACT_GRACE][{rn}] hold non-converged "
                        f"phase=done z_cmd_mm={float(getattr(ctx, 'direct_align_z_cmd_mm', 0.0)):.1f} "
                        f"latched={int(self._direct_align_contact_authority_active(ctx))}"
                    )
                return True

            if ctx.direct_align_retry < self._direct_align_max_retry():
                if (now - ctx.direct_align_epoch) >= self._direct_align_settle_margin_sec():
                    ctx.direct_align_retry += 1
                    ctx.direct_align_phase = "idle"
            else:
                self._fail_robot(rn, "DIRECT_ALIGN_NOT_CONVERGED")
            return True

        if ctx.direct_align_phase == "idle":
            if ctx.direct_align_started_at <= 0.0:
                ctx.direct_align_started_at = now

            if (now - ctx.direct_align_started_at) < self._direct_align_trigger_wait_sec():
                return False

            started = self._direct_align_try_start(rn, ctx)
            return started

        if ctx.direct_align_phase == "xy_sent":
            elapsed_xy = now - ctx.direct_align_epoch
            min_guard = min(1.0, max(0.15, 0.3 * max(0.0, ctx.direct_align_xy_time)))
            if elapsed_xy < min_guard:
                return True

            reached_flag = bool(getattr(ctx, "slide_reached", False))
            pos_fresh = self._slide_pos_fresh(ctx)
            xy_reached_time = (elapsed_xy >= (ctx.direct_align_xy_time + self._direct_align_settle_margin_sec()))
            xy_reached = reached_flag or xy_reached_time
            xy_moved = self._direct_align_xy_moved(rn, ctx) if pos_fresh else None
            need_x = abs(float(getattr(ctx, "direct_align_xy_cmd_x_mm", 0.0))) >= self._direct_align_pos_deadband_mm()
            need_y = abs(float(getattr(ctx, "direct_align_xy_cmd_y_mm", 0.0))) >= self._direct_align_pos_deadband_mm()
            if not reached_flag:
                ctx.direct_align_xy_ack_drop_seen = True
            ack_drop_seen = bool(getattr(ctx, "direct_align_xy_ack_drop_seen", False))
            # In XY stage, do not trust reached-ack alone for non-trivial XY motions.
            if xy_moved is None:
                xy_done = False
            else:
                xy_done = xy_reached and xy_moved
            if (not pos_fresh) and (not self._diag_ok(ctx, "_diag_slide_pos_stale_epoch", 0.5)):
                pass
            elif not pos_fresh:
                self.get_logger().warn(
                    f"[SLIDE_DIRECT_XY_NO_POS][{rn}] slide_pos stale or missing; will wait and fail if no feedback"
                )
            elif reached_flag and (not xy_moved) and (need_x or need_y):
                if self._diag_ok(ctx, "_diag_slide_xy_ack_early_epoch", 0.5):
                    self.get_logger().warn(
                        f"[SLIDE_DIRECT_XY_ACK_EARLY][{rn}] reached=True but movement_confirm=False; wait for movement/timeout"
                    )
            if (
                reached_flag
                and (not ack_drop_seen)
                and (not xy_moved)
                and (need_x or need_y)
                and pos_fresh
                and elapsed_xy >= self._direct_align_ack_drop_timeout_sec()
            ):
                if self._diag_ok(ctx, "_diag_slide_xy_cmd_not_accepted_epoch", 0.5):
                    self.get_logger().warn(
                        f"[SLIDE_DIRECT_XY_CMD_NOT_ACCEPTED][{rn}] "
                        f"reached_target stayed high for {elapsed_xy:.2f}s with no XY motion; retry early"
                    )
                if not self._direct_align_retry_or_fail(rn, ctx, "DIRECT_ALIGN_XY_CMD_NOT_ACCEPTED"):
                    return True
                return True
            if rn == 'tracer1' and (not xy_done) and self._diag_ok(ctx, "_diag_tracer1_xy_blocked_epoch", 0.5):
                status = self._sync_slide_xy_phase_status(rn)
                self.get_logger().warn(
                    f"[SLIDE_TRACE][tracer1] movement_confirm_blocked phase=xy_sent "
                    f"reason={status['confirm_reason']} "
                    f"slide_pos_fresh={int(bool(status['pos_fresh']))} "
                    f"reached_target={int(bool(reached_flag))} "
                    f"dxy=({float(status['move_dx_mm']):.1f},{float(status['move_dy_mm']):.1f}) "
                    f"cmd_xy=({float(getattr(ctx, 'direct_align_xy_cmd_x_mm', 0.0)):.1f},"
                    f"{float(getattr(ctx, 'direct_align_xy_cmd_y_mm', 0.0)):.1f})"
                )
            if not xy_done:
                hard_timeout = (
                    ctx.direct_align_xy_time +
                    self._direct_align_settle_margin_sec() +
                    self._direct_align_xy_move_timeout_margin_sec()
                )
                if elapsed_xy >= hard_timeout:
                    if (not pos_fresh):
                        self._fail_robot(rn, "DIRECT_ALIGN_NO_SLIDE_FEEDBACK")
                        return True
                    if not self._direct_align_retry_or_fail(rn, ctx, "DIRECT_ALIGN_XY_NO_EFFECTIVE_MOTION"):
                        return True
                return True

            z_mm = float(ctx.direct_align_pending_z_mm)
            self._sync_slide_enter_xy_barrier(rn, ctx, z_mm, reason="xy_movement_confirmed")
            if self._sync_slide_wait_for_xy_barrier(rn, source="xy_movement_confirmed"):
                return self._start_direct_align_z(rn, ctx, z_mm, now)
            return True

        if ctx.direct_align_phase == "xy_barrier":
            z_mm = float(ctx.direct_align_pending_z_mm)
            if self._sync_slide_wait_for_xy_barrier(rn, source="xy_barrier"):
                return self._start_direct_align_z(rn, ctx, z_mm, now)
            return True

        if ctx.direct_align_phase == "z_sent":
            elapsed_z = now - ctx.direct_align_epoch
            min_guard = min(1.0, max(0.15, 0.3 * max(0.0, ctx.direct_align_z_time)))
            if elapsed_z < min_guard:
                return True

            if self._direct_align_contact_ok(rn, ctx):
                return True

            z_done = bool(getattr(ctx, "slide_reached", False)) or \
                     (elapsed_z >= (ctx.direct_align_z_time + self._direct_align_settle_margin_sec()))
            if not z_done:
                return True

            ctx.direct_align_phase = "post_z_wait_contact_or_small_residual"
            ctx.direct_align_epoch = now
            ctx.direct_align_small_residual_count = 0
            if self._diag_ok(ctx, "_diag_passive_z_enter_epoch", 0.5):
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] entered passive Z settle window "
                    f"z_cmd_mm={float(getattr(ctx, 'direct_align_z_cmd_mm', 0.0)):.1f} "
                    f"timeout={self._direct_align_passive_z_wait_timeout_sec():.2f}s"
                )
            return True

        if ctx.direct_align_phase == "post_z_wait_contact_or_small_residual":
            if self._direct_align_contact_ok(rn, ctx):
                return self._direct_align_exit_passive_z_by_force_contact(rn, ctx, now)
            if self._direct_align_contact_authority_active(ctx):
                return self._direct_align_exit_passive_z_by_force_contact(rn, ctx, now)

            ready, dz, _ = self._direct_align_passive_z_small_residual_ready(rn, ctx)
            if ready:
                ctx.direct_align_z_done = True
                ctx.direct_align_phase = "done"
                ctx.direct_align_epoch = now
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] exited by small residual dz={float(dz or 0.0):.4f}"
                )
                return True

            if (now - float(getattr(ctx, 'direct_align_epoch', 0.0))) >= self._direct_align_passive_z_wait_timeout_sec():
                self.get_logger().warn(
                    f"[PASSIVE_Z_SETTLE][{rn}] exited by timeout -> DIRECT_ALIGN_NOT_CONVERGED"
                )
                self._fail_robot(rn, "DIRECT_ALIGN_NOT_CONVERGED")
                return True
            return True

        if ctx.direct_align_phase == "post_contact_hold":
            if not self._direct_align_post_contact_hold_ready(rn, ctx):
                return True
            ctx.direct_align_phase = "done"
            ctx.direct_align_epoch = now
            return True

        if ctx.direct_align_phase == "ready_barrier":
            dz = float(getattr(ctx, 'direct_align_ready_dz_m', 0.0))
            if self._sync_slide_wait_for_ready_barrier(rn, source="ready_barrier"):
                self._mark_ready_to_lift(rn, dz)
            return True

        return False

    # ------------------------------------------------------------------
    # recenter helpers
    # ------------------------------------------------------------------
    def _capture_virtual_center_ref(self, rn: str) -> bool:
        ctx = self.rt[rn]
        if ctx.slide_pos is None:
            return False
        ctx.transport_center_ref = ctx.slide_pos
        ctx.recenter_target = ctx.slide_pos
        ctx.recenter_done = True
        return True

    def _start_recenter_one(self, rn: str):
        ctx = self.rt[rn]

        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)
        self.stop_slide_comp(rn)

        if self.slide_recenter_mode == 'virtual':
            ok = self._capture_virtual_center_ref(rn)
            if not ok:
                self._fail_robot(rn, 'RECENTER_VIRTUAL_NO_SLIDE_STATUS')
                return
            self._set_local_state(
                rn,
                'RECENTERED',
                f'virtual recenter: freeze current slide pose as transport reference {ctx.transport_center_ref}'
            )
            return

        if self.slide_recenter_mode not in ('physical', 'physical_xy'):
            self._fail_robot(rn, f'INVALID_SLIDE_RECENTER_MODE:{self.slide_recenter_mode}')
            return

        cur = ctx.slide_pos
        cx = float(cur[0]) if cur is not None else float(self.slide_center_x_mm)
        cy = float(cur[1]) if cur is not None else float(self.slide_center_y_mm)
        cz = float(cur[2]) if cur is not None else float(self.slide_center_z_mm)

        resolved_xy = self._resolve_transport_center_target_xy_mm(rn, ctx)
        if resolved_xy is not None:
            txc, tyc, _ = resolved_xy
        else:
            txc = float(self.slide_center_x_mm)
            tyc = float(self.slide_center_y_mm)

        tx = float(txc) if self._slide_axis_enabled(rn, 'x') else cx
        ty = float(tyc) if self._slide_axis_enabled(rn, 'y') else cy
        if self.slide_recenter_mode == 'physical_xy':
            tz = cz
        else:
            tz = float(self.slide_center_z_mm) if self._slide_axis_enabled(rn, 'z') else cz

        ctx.recenter_target = (tx, ty, tz)
        ctx.transport_center_ref = ctx.recenter_target
        ctx.recenter_done = False

        self.send_slide_position(
            rn,
            tx,
            ty,
            tz,
            self.slide_recenter_time_sec,
            is_relative=False
        )

        self._set_local_state(
            rn,
            'RECENTERING',
            f'physical recenter target=({tx:.1f},{ty:.1f},{tz:.1f}) mm'
        )

    def _start_recenter_all(self):
        for rn in self.robots:
            self._start_recenter_one(rn)

    def _check_recenter_done_one(self, rn: str) -> bool:
        ctx = self.rt[rn]

        if self.slide_recenter_mode == 'virtual':
            return bool(ctx.recenter_done)

        if self.slide_recenter_mode not in ('physical', 'physical_xy'):
            return False

        if ctx.slide_pos is None or ctx.slide_vel is None or ctx.recenter_target is None:
            return False

        x, y, z = ctx.slide_pos
        vx, vy, vz = ctx.slide_vel
        tx, ty, tz = ctx.recenter_target

        ex = abs(x - tx)
        ey = abs(y - ty)
        ez = abs(z - tz)
        xy_only = (self.slide_recenter_mode == 'physical_xy')

        pos_ok = (
            (not self._slide_axis_enabled(rn, 'x') or ex <= self.slide_recenter_tol_mm) and
            (not self._slide_axis_enabled(rn, 'y') or ey <= self.slide_recenter_tol_mm) and
            (xy_only or (not self._slide_axis_enabled(rn, 'z') or ez <= self.slide_recenter_tol_mm))
        )
        vel_ok = (
            (not self._slide_axis_enabled(rn, 'x') or abs(vx) <= 1.0) and
            (not self._slide_axis_enabled(rn, 'y') or abs(vy) <= 1.0) and
            (xy_only or (not self._slide_axis_enabled(rn, 'z') or abs(vz) <= 1.0))
        )

        done = pos_ok and vel_ok and ctx.slide_reached
        if done and not ctx.recenter_done:
            ctx.recenter_done = True
            self._set_local_state(
                rn,
                'RECENTERED',
                f'physical recenter done: pos=({x:.2f},{y:.2f},{z:.2f}) mm'
            )
        return done

    def _all_recenter_done(self) -> bool:
        return all(self._check_recenter_done_one(rn) for rn in self.robots)

    # ------------------------------------------------------------------
    # loaded Z-leveling
    # ------------------------------------------------------------------
    def _start_level_z_one(self, rn: str, z_plane_mm: float):
        ctx = self.rt[rn]

        if ctx.slide_pos is None:
            self._fail_robot(rn, 'LEVEL_Z_NO_SLIDE_STATUS')
            return

        x_mm, y_mm, z_mm = ctx.slide_pos
        target_z = float(max(z_mm, z_plane_mm)) if self.load_level_only_raise_z else float(z_plane_mm)

        z_max = getattr(self, 'load_level_z_plane_max_mm', 180.0)
        z_min = getattr(self, 'load_level_z_plane_min_mm', 80.0)

        if target_z > z_max:
            self.get_logger().error(
                f'[LEVEL_Z][{rn}] target_z={target_z:.2f}mm exceeds max={z_max:.2f}mm, clamping'
            )
            target_z = z_max
        if target_z < z_min:
            self.get_logger().warn(
                f'[LEVEL_Z][{rn}] target_z={target_z:.2f}mm below min={z_min:.2f}mm, clamping'
            )
            target_z = z_min

        if not self._slide_axis_enabled(rn, 'z'):
            ctx.level_active = False
            ctx.level_done = True
            ctx.level_target_z_mm = z_mm
            self._set_local_state(rn, 'LEVEL_Z_DONE', f'z axis disabled by capability mask; keep z={z_mm:.2f}mm')
            return

        ctx.level_active = True
        ctx.level_done = False
        ctx.level_target_z_mm = target_z

        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)
        self.stop_slide_comp(rn)

        self.send_slide_position(
            rn,
            float(x_mm),
            float(y_mm),
            float(target_z),
            self.load_level_time_sec,
            is_relative=False
        )

        self._set_local_state(rn, 'LEVEL_Z', f'loaded z-plane target={target_z:.2f}mm (clamped to [{z_min:.1f}, {z_max:.1f}])')

    def _start_level_z_all(self):
        z_plane_mm = self._build_loaded_z_plane_mm()
        if z_plane_mm is None:
            for rn in self.robots:
                if not self.rt[rn].faulted:
                    self._fail_robot(rn, 'LEVEL_Z_NO_VALID_PLANE')
            return

        self.loaded_z_plane_mm = float(z_plane_mm)
        self.get_logger().warn(f'[LEVEL_Z] contact-gated z_plane_mm={self.loaded_z_plane_mm:.2f}')

        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted or (not ctx.ready_to_lift):
                continue
            self._start_level_z_one(rn, self.loaded_z_plane_mm)

    def _check_level_z_done_one(self, rn: str) -> bool:
        ctx = self.rt[rn]
        if ctx.faulted:
            return False
        if not getattr(ctx, 'level_active', False):
            return bool(getattr(ctx, 'level_done', False))
        if ctx.slide_pos is None or ctx.slide_vel is None:
            return False
        if getattr(ctx, 'level_target_z_mm', None) is None:
            return False

        z_mm = float(ctx.slide_pos[2])
        vz_mmps = float(ctx.slide_vel[2])
        z_err = abs(z_mm - float(ctx.level_target_z_mm))

        done = (z_err <= self.load_level_z_tol_mm) and (abs(vz_mmps) <= self.load_level_vel_tol_mmps)
        if done and not getattr(ctx, 'level_done', False):
            ctx.level_done = True
            ctx.level_active = False
            self._set_local_state(rn, 'LEVEL_Z_DONE', f'loaded z-plane done z={z_mm:.2f}mm')
        return done

    def _all_level_z_done(self) -> bool:
        ok_list = []
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted or (not ctx.ready_to_lift):
                continue
            ok_list.append(self._check_level_z_done_one(rn))
        return (len(ok_list) > 0) and all(ok_list)

    # ------------------------------------------------------------------
    # slide alignment / transport compensation
    # ------------------------------------------------------------------
    def _slide_align_step(self, rn: str):
        ctx = self.rt[rn]
        self._direct_ctx_init(ctx)

        align_mode = self._slide_align_mode()

        if not self._alignment_vision_enabled():
            self.stop_pub[rn].publish(Bool(data=True))
            self.precision_on(rn, False)

            handled = self._direct_align_z_only_contact_step(rn)
            if handled:
                return

            if self.slide_comp_hold_zero_on_lost:
                self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
            return

        te = ctx.te
        if te is None:
            if align_mode in ('direct_only', 'direct_then_speed'):
                handled = self._direct_align_step(rn)
                if handled:
                    return
            return

        st = te.status(time.time())

        _diag_key = f'_slide_align_diag_{rn}'
        _now = _now_sec(self)
        _last_diag = getattr(self, _diag_key, 0.0)
        if (_now - _last_diag) > 3.0:
            setattr(self, _diag_key, _now)
            d = ctx.delta_latest
            d_str = f'({d[0]:.4f},{d[1]:.4f},{d[2]:.4f})' if d else 'None'
            d_age = f'{_now - ctx.last_delta_stamp:.2f}s' if ctx.last_delta_stamp else 'N/A'
            self.get_logger().warn(
                f'[SLIDE_ALIGN_DIAG][{rn}] mode={align_mode} te_tracking={st.tracking} '
                f'conf={st.confidence:.2f} stable={st.stable_count} '
                f'soft_lost={st.soft_lost} hard_lost={st.hard_lost} '
                f'last_seen_ago={st.last_seen_ago:.2f}s age={st.age_sec:.2f}s '
                f'delta={d_str} delta_age={d_age} '
                f'phase={ctx.direct_align_phase} xy_stable={ctx.xy_stable_count}'
            )

        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)

        if st.hard_lost:
            if self._complete_post_contact_ready(rn, ctx, 'hard_lost_after_contact'):
                return
            self._diag_slide_status(rn, ctx, st, "hard_lost -> WAIT_QR")
            self.stop_slide_comp(rn)
            ctx.fine_active = False
            ctx.xy_stable_count = 0
            ctx.sync_wait_qr = True
            ctx.sync_wait_qr_epoch = _now_sec(self)
            ctx.dwell_start = 0.0
            ctx.dwell_locked = False
            self._reset_raw_qr_tracking(ctx)
            self._reset_direct_align_state(ctx)
            self._set_local_state(rn, 'WAIT_QR', 'hard visual loss during slide alignment; hold base, wait for visual reacquire')
            emit_event = getattr(self, '_bench_emit_event', None)
            if callable(emit_event):
                emit_event(
                    event_type='DEGRADED_HOLD_ENTER',
                    robot_id=rn,
                    phase=self.state.lower(),
                    reason='hard_lost_during_slide_align',
                    note='visual loss detected; entering degraded hold (WAIT_QR)',
                )
            return

        if align_mode in ('direct_only', 'direct_then_speed'):
            handled = self._direct_align_step(rn)
            if handled:
                return

            if align_mode == 'direct_only':
                if self.slide_comp_hold_zero_on_lost:
                    self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
                return

        if not st.tracking:
            self._diag_slide_status(rn, ctx, st, "not_tracking")
            ctx.xy_stable_count = 0
            if self.slide_comp_hold_zero_on_lost:
                self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
            return

        stable = self._get_stable_delta(rn)
        if stable is None:
            return

        dx_raw, dy_raw, dz_raw = stable
        dx, dy, dz = self._mask_delta_for_robot(rn, dx_raw, dy_raw, dz_raw)

        x_ok = self._axis_ok_or_disabled(rn, 'x', dx_raw, self.fine_xy_tol_m)
        y_ok = self._axis_ok_or_disabled(rn, 'y', dy_raw, self.fine_xy_tol_m)
        z_ok = self._axis_ok_or_disabled(rn, 'z', dz_raw, self.fine_z_tol_m)
        xy_ok = x_ok and y_ok

        self._diag_ready_check(rn, ctx, dx_raw, dy_raw, dz_raw, xy_ok, z_ok)

        if xy_ok:
            ctx.xy_stable_count += 1
        else:
            ctx.xy_stable_count = 0

        if ctx.xy_stable_count >= self.fine_xy_stable_frames:
            self.stop_slide_comp(rn)
            self._reset_direct_align_state(ctx)
            if z_ok:
                self._sync_slide_enter_ready_barrier(rn, ctx, dz_raw, reason='speed_align_done')
                if self._sync_slide_wait_for_ready_barrier(rn, source='speed_align_done'):
                    self._mark_ready_to_lift(rn, dz_raw)
            else:
                _, _, vz = self._slide_speed_from_delta(0.0, 0.0, dz)
                self._diag_slide_cmd(rn, ctx, 0.0, 0.0, dz, 0.0, 0.0, vz, mode="align-z")
                self._pub_slide_speed(rn, 0.0, 0.0, vz, force=True)
            return

        if self.slide_comp_alignment_enable and align_mode in ('speed_only', 'direct_then_speed'):
            vx, vy, vz = self._slide_speed_from_delta(dx, dy, dz)
            self._diag_slide_cmd(rn, ctx, dx, dy, dz, vx, vy, vz, mode="align")
            self._pub_slide_speed(rn, vx, vy, vz)

    def _slide_transport_step(self, rn: str):
        if not self.slide_comp_enable or not self.slide_comp_transport_enable:
            return

        ctx = self.rt[rn]
        vx_ff, vy_ff, vz_ff = self._slide_transport_feedforward_speed(rn, ctx)
        vx_hold, vy_hold = self._slide_transport_center_hold_speed(rn, ctx)
        te = ctx.te
        if te is None:
            vx = self._clamp(vx_ff + vx_hold, self.slide_transport_vx_limit_mmps)
            vy = self._clamp(vy_ff + vy_hold, self.slide_transport_vy_limit_mmps)
            vz = self._clamp(vz_ff, self.slide_transport_vz_limit_mmps)
            if abs(vx) > 1e-9 or abs(vy) > 1e-9 or abs(vz) > 1e-9:
                self._pub_slide_speed(rn, vx, vy, vz)
            return

        st = te.status(time.time())

        if st.hard_lost:
            self._diag_slide_status(rn, ctx, st, "transport hard_lost")
            if self.slide_comp_abort_transport_on_hard_lost and abs(vx_ff) < 1e-9 and abs(vy_ff) < 1e-9:
                self.stop_slide_comp(rn)
                self._fail_robot(rn, 'TRANSPORT_VISUAL_HARD_LOST')
                return
            vx = self._clamp(vx_ff + vx_hold, self.slide_transport_vx_limit_mmps)
            vy = self._clamp(vy_ff + vy_hold, self.slide_transport_vy_limit_mmps)
            self._pub_slide_speed(rn, vx, vy, 0.0)
            return

        if not st.tracking:
            self._diag_slide_status(rn, ctx, st, "transport not_tracking")
            vx = self._clamp(vx_ff + vx_hold, self.slide_transport_vx_limit_mmps)
            vy = self._clamp(vy_ff + vy_hold, self.slide_transport_vy_limit_mmps)
            if abs(vx) > 1e-9 or abs(vy) > 1e-9:
                self._pub_slide_speed(rn, vx, vy, 0.0)
            elif self.slide_comp_hold_zero_on_lost:
                self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
            return

        stable = self._get_stable_delta(rn)
        if stable is None:
            vx = self._clamp(vx_ff + vx_hold, self.slide_transport_vx_limit_mmps)
            vy = self._clamp(vy_ff + vy_hold, self.slide_transport_vy_limit_mmps)
            if abs(vx) > 1e-9 or abs(vy) > 1e-9:
                self._pub_slide_speed(rn, vx, vy, 0.0)
            return

        dx_raw, dy_raw, dz_raw = stable
        dx, dy, dz = self._mask_delta_for_robot(rn, dx_raw, dy_raw, dz_raw)

        vx_delta, vy_delta, vz_delta = self._slide_transport_speed_from_delta(dx, dy, dz, ctx)

        vx = self._clamp(vx_delta + vx_ff + vx_hold, self.slide_transport_vx_limit_mmps)
        vy = self._clamp(vy_delta + vy_ff + vy_hold, self.slide_transport_vy_limit_mmps)
        vz = self._clamp(vz_delta + vz_ff, self.slide_transport_vz_limit_mmps)

        self._diag_transport_blend(
            rn=rn,
            ctx=ctx,
            dx=dx_raw, dy=dy_raw, dz=dz_raw,
            vx_delta=vx_delta, vy_delta=vy_delta, vz_delta=vz_delta,
            vx_ff=vx_ff, vy_ff=vy_ff,
            vx_hold=vx_hold, vy_hold=vy_hold,
            vx_final=vx, vy_final=vy, vz_final=vz
        )

        self._pub_slide_speed(rn, vx, vy, vz)

    def slide_rt_loop(self):
        if self.emergency or self.state in ('ABORT', 'DONE', 'STANDBY'):
            self.stop_all_slide_comp()
            return

        if self.state in ('ALL_READY_HOLD', 'SYNC_RECENTER', 'SYNC_LIFT', 'SYNC_LEVEL_Z', 'LOAD_STABLE_HOLD'):
            self.stop_all_slide_comp()
            return

        now = _now_sec(self)

        for rn in self.robots:
            ctx = self.rt[rn]

            if ctx.faulted or ctx.finished or self.state in ('SYNC_LEVEL_Z', 'LOAD_STABLE_HOLD'):
                self.stop_slide_comp(rn)
                continue

            if self.state == 'SYNC_SLIDE_ALIGN':
                if ctx.fine_active:
                    self._slide_align_step(rn)
                else:
                    if self._slide_comp_zero_due(ctx, now):
                        self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
                continue

            if self.state == 'SYNC_TRANSPORT' and ctx.transporting:
                self._slide_transport_step(rn)
                continue

            if self._slide_comp_zero_due(ctx, now):
                self._pub_slide_speed(rn, 0.0, 0.0, 0.0)
