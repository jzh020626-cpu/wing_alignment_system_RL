# -*- coding: utf-8 -*-

import math
import time
from collections import deque

from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from wing_alignment_system.mission_geometry import _now_sec, wrap_angle_rad


class MissionStateHelpersMixin:
    def _fmt(self, tmpl: str, rn: str) -> str:
        return (tmpl or '').replace('{robot}', rn)

    def _alignment_vision_enabled(self) -> bool:
        return bool(getattr(self, 'alignment_vision_enable', True))

    # ------------------------------------------------------------------
    # logging / state helpers
    # ------------------------------------------------------------------
    def _set_global_state(self, new_state: str, reason: str = ''):
        if self.state == new_state:
            return
        
        if not self._workflow_allows_state(new_state):
            wf = getattr(self, 'workflow', 'full')
            self.get_logger().error(
                f'[WORKFLOW={wf}] State transition BLOCKED: {self.state} -> {new_state} not allowed in workflow={wf}'
            )
            return
        
        old = self.state
        self.state = new_state
        suffix = f' | {reason}' if reason else ''
        self.get_logger().warn(f'[MISSION] {old} -> {new_state}{suffix}')
        emit_event = getattr(self, '_bench_emit_event', None)
        if callable(emit_event) and new_state in {
            'PHASE1_DONE_HOLD',
            'ALL_READY_HOLD',
            'SYNC_LEVEL_Z',
            'SYNC_RECENTER',
            'LOAD_STABLE_HOLD',
            'TRANSPORT_PRECHECK',
            'TRANSPORT_SETTLE',
        }:
            emit_event(
                event_type='FREEZE_ON',
                robot_id='fleet',
                phase=new_state.lower(),
                reason=reason or f'{old}->{new_state}',
                note='global hold/freeze state transition',
                state_from=old,
                state_to=new_state,
            )
        emit_outcome = getattr(self, '_bench_emit_outcome', None)
        if callable(emit_outcome) and new_state in {'ABORT', 'DONE'}:
            emit_outcome(reason=reason, state_from=old, state_to=new_state)

    def _set_local_state(self, rn: str, new_state: str, reason: str = ''):
        ctx = self.rt[rn]
        if ctx.local_state == new_state:
            return
        old = ctx.local_state
        ctx.local_state = new_state
        suffix = f' | {reason}' if reason else ''
        self.get_logger().info(f'[STATE][{rn}] {old} -> {new_state}{suffix}')
        emit_event = getattr(self, '_bench_emit_event', None)
        if callable(emit_event) and new_state in {'WAIT_QR', 'TRANSPORT_WAIT_SETTLE'}:
            emit_event(
                event_type='FREEZE_ON',
                robot_id=rn,
                phase=getattr(self, 'state', '').lower(),
                reason=reason or f'{old}->{new_state}',
                note='local chassis hold/freeze state transition',
                state_from=old,
                state_to=new_state,
            )

    def _clear_ready_flags(self, ctx):
        ctx.confirmed = False
        ctx.align_done = False
        ctx.align_done_epoch = 0.0
        ctx.ready_to_lift = False
        ctx.ready_epoch = 0.0
        ctx.lifting = False
        ctx.finished = False
        ctx.transport_target = None
        ctx.contact_confirmed = False
        self._reset_transport_state(ctx)
        self._clear_post_contact_state(ctx)

    def _clear_post_contact_state(self, ctx):
        ctx.force_contact_epoch = 0.0
        ctx.post_contact_latched = False
        ctx.post_contact_epoch = 0.0
        ctx.post_contact_delta = None

    def _reset_transport_state(self, ctx, clear_center_ref: bool = False):
        ctx.transporting = False
        ctx.transport_dispatched = False
        ctx.transport_arrived = False
        ctx.transport_settled = False
        ctx.transport_failed = False
        ctx.transport_start_stamp = 0.0
        ctx.transport_arrive_stamp = 0.0
        ctx.transport_settle_stamp = 0.0
        ctx.formation_error_m = 0.0
        ctx.formation_error_yaw_deg = 0.0
        ctx.group_stop_reason = ''
        if clear_center_ref:
            ctx.transport_center_ref = None
            ctx.loaded_ref_captured = False

    def _reset_raw_qr_tracking(self, ctx):
        ctx.raw_qr_seen_stamp = 0.0
        ctx.raw_qr_hit_count = 0
        ctx.raw_qr_last_hit_stamp = 0.0
        ctx.raw_qr_pose = None

    def _reset_direct_align_state(self, ctx):
        ctx.direct_align_phase = 'idle'
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
        ctx.contact_confirmed = False

    def _reset_level_and_load_state(self, ctx):
        ctx.level_active = False
        ctx.level_done = False
        ctx.level_target_z_mm = None
        ctx.loaded_ref_captured = False
        ctx.transport_center_ref = None
        ctx.contact_confirmed = False
        self._clear_post_contact_state(ctx)

    def _reset_runtime_for_new_mission_leg(self, rn: str, clear_alignment: bool = True):
        ctx = self.rt[rn]
        if clear_alignment:
            self._clear_ready_flags(ctx)

        ctx.faulted = False
        ctx.fault_reason = ''
        ctx.segs = None
        ctx.seg_i = 0
        ctx.dwell_start = 0.0
        ctx.dwell_locked = False
        ctx.fine_active = False
        ctx.fine_goal_inflight = False
        ctx.fine_settle_until = 0.0
        ctx.xy_stable_count = 0
        ctx.gate_stopped = False
        ctx.gate_hold_until = 0.0
        ctx.slow_mode = False
        ctx.delta_armed_since = 0.0
        ctx.delta_latest = None
        ctx.last_delta_stamp = None
        ctx.qr_zero_pending = False
        ctx.qr_zero_done = False
        ctx.qr_zero_req_epoch = 0.0
        ctx.comp_active = False
        ctx.comp_last_vx = 0.0
        ctx.comp_last_vy = 0.0
        ctx.comp_last_vz = 0.0
        ctx.last_comp_pub_epoch = 0.0
        ctx.recenter_done = False
        ctx.recenter_target = None
        ctx.first_qr_locked = False
        ctx.first_qr_lock_epoch = 0.0
        ctx.sync_wait_qr = False
        ctx.sync_wait_qr_epoch = 0.0

        self._reset_raw_qr_tracking(ctx)
        self._reset_direct_align_state(ctx)
        self._reset_level_and_load_state(ctx)
        self._reset_transport_state(ctx, clear_center_ref=True)

        if ctx.te is not None:
            ctx.te.reset()

    def _mark_ready_to_lift(self, rn: str, dz: float):
        ctx = self.rt[rn]
        now = _now_sec(self)

        ctx.confirmed = True
        ctx.align_done = True
        ctx.align_done_epoch = now
        ctx.ready_to_lift = True
        ctx.ready_epoch = now
        ctx.lifting = False
        ctx.finished = False
        ctx.fine_active = False
        ctx.fine_goal_inflight = False
        ctx.dwell_start = 0.0
        ctx.dwell_locked = False
        ctx.gate_stopped = False
        ctx.sync_wait_qr = False
        ctx.sync_wait_qr_epoch = 0.0

        self._reset_direct_align_state(ctx)
        self._reset_transport_state(ctx)
        self._clear_post_contact_state(ctx)

        self.stop_slide_comp(rn)
        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)

        self._set_local_state(rn, 'READY_TO_LIFT', f'alignment confirmed, dz={dz:.4f}m')
        self.get_logger().info(f'[READY] {rn} alignment confirmed; waiting for loaded leveling/transport barrier.')

    def _fail_robot(self, rn: str, reason: str):
        ctx = self.rt[rn]
        if ctx.faulted:
            return

        self._clear_ready_flags(ctx)
        ctx.faulted = True
        ctx.fault_reason = reason
        ctx.transport_failed = True
        ctx.group_stop_reason = reason
        ctx.segs = None
        ctx.seg_i = 0
        ctx.fine_active = False
        ctx.fine_goal_inflight = False
        ctx.dwell_start = 0.0
        ctx.dwell_locked = False
        ctx.gate_stopped = True
        ctx.gate_hold_until = _now_sec(self) + self.gate_hold_sec
        ctx.sync_wait_qr = False
        ctx.sync_wait_qr_epoch = 0.0

        self._reset_direct_align_state(ctx)

        self.stop_slide_comp(rn)
        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)

        self._set_local_state(rn, 'ERROR', reason)
        self.get_logger().error(f'[FAULT] {rn}: {reason}')

    # ------------------------------------------------------------------
    # diagnostics / freshness helpers
    # ------------------------------------------------------------------
    def _mocap_fresh(self):
        now = _now_sec(self)
        bad = []

        if self.wing_pose_stamp <= 0.0 or (now - self.wing_pose_stamp) > self.mocap_timeout_sec:
            bad.append('wing')

        for rn in self.robots:
            ts = self.robot_pose_stamp.get(rn, 0.0)
            if ts <= 0.0 or (now - ts) > self.mocap_timeout_sec:
                bad.append(rn)

        return (len(bad) == 0), bad

    def _all_first_qr_locked(self) -> bool:
        return all((not self.rt[rn].faulted) and self.rt[rn].first_qr_locked for rn in self.robots)

    # ------------------------------------------------------------------
    # raw QR gating
    # ------------------------------------------------------------------
    def _raw_qr_pose_reasonable_xyz(self, x: float, y: float, z: float) -> bool:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return False
        if x < self.raw_qr_min_x_m or x > self.raw_qr_max_x_m:
            return False
        if abs(y) > self.raw_qr_max_abs_y_m:
            return False
        if z < self.raw_qr_min_z_m or z > self.raw_qr_max_z_m:
            return False
        return True

    def _raw_qr_pose_reasonable(self, rn: str) -> bool:
        pose = self.rt[rn].raw_qr_pose
        if pose is None:
            return False
        return self._raw_qr_pose_reasonable_xyz(pose[0], pose[1], pose[2])

    def _raw_qr_gate_ok(self, rn: str) -> bool:
        ctx = self.rt[rn]

        if self.state not in ('RUN_ALIGNMENT', 'SYNC_SLIDE_ALIGN'):
            return False
        if ctx.goal_kind != 'FINAL':
            return False
        if not ctx.entered:
            return False

        if ctx.dwell_start <= 0.0 and (not getattr(ctx, 'sync_wait_qr', False)):
            return False

        dwell_t0 = ctx.sync_wait_qr_epoch if getattr(ctx, 'sync_wait_qr', False) else ctx.dwell_start
        if dwell_t0 <= 0.0:
            return False

        now = _now_sec(self)
        elapsed = now - dwell_t0

        if elapsed < self.raw_qr_arm_delay_sec:
            return False

        if ctx.final_target is None:
            return False
        if rn not in self.robot_xy:
            return False

        x, y = self.robot_xy[rn]
        xt, yt = ctx.final_target
        dist_to_final = math.hypot(x - xt, y - yt)

        pose_ok = self._raw_qr_pose_reasonable(rn)
        hit_age = (now - ctx.raw_qr_last_hit_stamp) if ctx.raw_qr_last_hit_stamp > 0 else 999.0
        seen_age = (now - ctx.raw_qr_seen_stamp) if ctx.raw_qr_seen_stamp > 0 else 999.0
        effective_seen_timeout = max(
            float(getattr(self, 'raw_qr_seen_timeout_sec', 0.0)),
            float(getattr(self, 'raw_qr_hit_timeout_sec', 0.0)),
            0.1,
        )

        # periodic diagnostic during WAIT_QR (every ~2s at 50Hz tick)
        if not ctx.first_qr_locked and elapsed > 2.0:
            diag_key = f'_raw_qr_diag_{rn}'
            last_diag = getattr(self, diag_key, 0.0)
            if (now - last_diag) > 2.0:
                setattr(self, diag_key, now)
                pose_str = f'({ctx.raw_qr_pose[0]:.4f},{ctx.raw_qr_pose[1]:.4f},{ctx.raw_qr_pose[2]:.4f})' if ctx.raw_qr_pose else 'None'
                self.get_logger().warn(
                    f'[RAW_QR_DIAG][{rn}] elapsed={elapsed:.1f}s '
                    f'dist_final={dist_to_final:.3f}(<{self.raw_qr_accept_radius_m}) '
                    f'pose={pose_str} pose_ok={pose_ok} '
                    f'hits={ctx.raw_qr_hit_count}/{self.raw_qr_min_hits} '
                    f'hit_age={hit_age:.3f}s(<{self.raw_qr_hit_timeout_sec}) '
                    f'seen_age={seen_age:.3f}s(<={effective_seen_timeout:.3f})'
                )

        if dist_to_final > self.raw_qr_accept_radius_m:
            return False

        if not pose_ok:
            return False

        if ctx.raw_qr_hit_count < self.raw_qr_min_hits:
            return False

        if hit_age > self.raw_qr_hit_timeout_sec:
            return False

        if seen_age > effective_seen_timeout:
            return False

        return True

    def _raw_qr_fresh(self, rn: str) -> bool:
        return self._raw_qr_gate_ok(rn)

    def _wait_qr_timeout_reason(self, rn: str, prefix: str) -> str:
        ctx = self.rt[rn]
        now = _now_sec(self)
        pose = getattr(ctx, 'raw_qr_pose', None)
        pose_str = (
            f'({float(pose[0]):.4f},{float(pose[1]):.4f},{float(pose[2]):.4f})'
            if pose is not None else
            'None'
        )
        seen_age = (now - float(getattr(ctx, 'raw_qr_seen_stamp', 0.0))) if float(getattr(ctx, 'raw_qr_seen_stamp', 0.0)) > 0.0 else 999.0
        hit_age = (now - float(getattr(ctx, 'raw_qr_last_hit_stamp', 0.0))) if float(getattr(ctx, 'raw_qr_last_hit_stamp', 0.0)) > 0.0 else 999.0
        last_reject = str(getattr(ctx, 'raw_qr_last_reject_reason', 'none') or 'none')
        receive_stamp = float(getattr(ctx, 'raw_qr_receive_stamp', 0.0) or 0.0)
        receive_seen = int(receive_stamp > 0.0)

        dist_to_final = -1.0
        if ctx.final_target is not None and rn in self.robot_xy:
            rx, ry = self.robot_xy[rn]
            xt, yt = ctx.final_target
            dist_to_final = math.hypot(rx - xt, ry - yt)

        return (
            f'{prefix} '
            f'pose={pose_str} hits={int(getattr(ctx, "raw_qr_hit_count", 0))}/{self.raw_qr_min_hits} '
            f'seen_age={seen_age:.3f}s hit_age={hit_age:.3f}s '
            f'last_reject={last_reject} receive_seen={receive_seen} '
            f'dist_final={dist_to_final:.3f}m'
        )

    def _start_sync_slide_align(self):
        vision_enabled = self._alignment_vision_enabled()
        sync_start_epoch = _now_sec(self)
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue

            self.stop_pub[rn].publish(Bool(data=True))
            self.precision_on(rn, False)
            self.stop_slide_comp(rn)

            ctx.fine_active = True
            ctx.xy_stable_count = 0
            ctx.dwell_start = 0.0
            ctx.dwell_locked = False
            ctx.sync_wait_qr = False
            ctx.sync_wait_qr_epoch = 0.0

            ctx.delta_armed_since = 0.0
            ctx.delta_latest = None
            ctx.last_delta_stamp = None
            ctx.qr_zero_pending = False
            ctx.qr_zero_done = False
            ctx.qr_zero_req_epoch = 0.0

            self._reset_direct_align_state(ctx)
            self._clear_post_contact_state(ctx)
            ctx.direct_align_started_at = sync_start_epoch

            if ctx.te is not None:
                ctx.te.reset()

            if vision_enabled:
                self.request_qr_reset_tracking(rn)
                if self.qr_zero_enable:
                    self.request_qr_zero(rn)
                else:
                    self.arm_delta(rn)
            else:
                self.stop_slide_position(rn)

            self._set_local_state(
                rn,
                'SLIDE_ALIGNING',
                'all robots parked at coarse final goal; start synchronized z-only contact alignment'
                if not vision_enabled else
                'all robots parked at coarse final goal and stable QR sight confirmed; start synchronized slide alignment'
            )

        if self.robots:
            self.get_logger().warn(
                f'[SLIDE_SYNC_START] epoch={sync_start_epoch:.3f} '
                f'mode={"z_only_contact" if not vision_enabled else "vision_align"} '
                f'robots={[rn for rn in self.robots if not self.rt[rn].faulted]}'
            )

    def _restart_sync_slide_align_one(self, rn: str, reason: str):
        ctx = self.rt[rn]
        vision_enabled = self._alignment_vision_enabled()

        self.stop_pub[rn].publish(Bool(data=True))
        self.precision_on(rn, False)
        self.stop_slide_comp(rn)

        ctx.fine_active = True
        ctx.xy_stable_count = 0
        ctx.dwell_start = 0.0
        ctx.dwell_locked = False
        ctx.sync_wait_qr = False
        ctx.sync_wait_qr_epoch = 0.0

        ctx.delta_armed_since = 0.0
        ctx.delta_latest = None
        ctx.last_delta_stamp = None
        ctx.qr_zero_pending = False
        ctx.qr_zero_done = False
        ctx.qr_zero_req_epoch = 0.0

        self._reset_direct_align_state(ctx)

        if ctx.te is not None:
            ctx.te.reset()

        if vision_enabled:
            self.request_qr_reset_tracking(rn)
            if self.qr_zero_enable:
                self.request_qr_zero(rn)
            else:
                self.arm_delta(rn)
        else:
            self.stop_slide_position(rn)

        self._set_local_state(rn, 'SLIDE_ALIGNING', reason)

    def _next_micro_offset(self, ctx):
        if not self.micro_list:
            return None

        best = None
        best_norm = -1.0
        n = len(self.micro_list)

        for _ in range(n):
            dxm, dym = self.micro_list[ctx.micro_i % n]
            ctx.micro_i += 1
            norm = math.hypot(dxm, dym)

            if norm > best_norm:
                best = (dxm, dym)
                best_norm = norm

            if norm >= self.micro_min_move_m:
                return (dxm, dym)

        return best

    # ------------------------------------------------------------------
    # QR zero / barrier helpers
    # ------------------------------------------------------------------
    def request_qr_zero(self, rn: str) -> bool:
        if not self.qr_zero_enable:
            return False

        ctx = self.rt[rn]
        now = _now_sec(self)
        if ctx.qr_zero_done:
            return True

        if ctx.qr_zero_req_epoch > 0.0 and (now - ctx.qr_zero_req_epoch) < self.qr_zero_retry_sec:
            return False

        cli = self.qr_zero_cli.get(rn)
        if cli is None or not cli.service_is_ready():
            ctx.qr_zero_req_epoch = now
            return False

        fut = cli.call_async(Trigger.Request())
        ctx.qr_zero_req_epoch = now
        ctx.qr_zero_pending = True

        def _done_cb(fut_obj):
            c = self.rt.get(rn)
            if c is None:
                return

            c.qr_zero_pending = False

            try:
                res = fut_obj.result()
                ok = bool(getattr(res, 'success', True))
                msg = str(getattr(res, 'message', ''))
            except Exception as e:
                ok = False
                msg = f'exception: {e}'

            c.qr_zero_done = ok

            self.get_logger().warn(
                f"[QR_ZERO][{rn}] request_done success={ok} msg={msg}"
            )

            if ok:
                self.arm_delta(rn)

        fut.add_done_callback(_done_cb)
        return False

    def request_qr_reset_tracking(self, rn: str):
        """Call qr_delta/reset_tracking service to clear stale tracking state.
        Fire-and-forget: we don't block on the result."""
        cli = getattr(self, 'qr_reset_cli', {}).get(rn)
        if cli is None or not cli.service_is_ready():
            self.get_logger().warn(
                f"[QR_RESET][{rn}] reset_tracking service not ready, skipping"
            )
            return

        fut = cli.call_async(Trigger.Request())

        def _done_cb(fut_obj):
            try:
                res = fut_obj.result()
                ok = bool(getattr(res, 'success', True))
                msg = str(getattr(res, 'message', ''))
            except Exception as e:
                ok = False
                msg = f'exception: {e}'
            self.get_logger().warn(
                f"[QR_RESET][{rn}] reset_tracking done success={ok} msg={msg}"
            )

        fut.add_done_callback(_done_cb)
        self.get_logger().info(
            f"[QR_RESET][{rn}] reset_tracking requested (fire-and-forget)"
        )

    # ------------------------------------------------------------------
    # force / loaded-plane / stability helpers
    # ------------------------------------------------------------------
    def _force_msg_fresh(self, rn: str, timeout_sec: float) -> bool:
        ctx = self.rt[rn]
        ts = getattr(ctx, 'force_stamp', 0.0)
        if ts <= 0.0:
            return False
        return (_now_sec(self) - ts) <= timeout_sec

    def _force_contact_ok(self, rn: str) -> bool:
        ctx = self.rt[rn]
        ff = getattr(ctx, 'force_f', None)
        if ff is None:
            return False
        
        fresh = self._force_msg_fresh(rn, self.load_level_force_fresh_timeout_sec)
        if not fresh:
            return False
        
        fz = float(ff[2])
        contact = abs(fz) >= self.load_level_contact_force_n
        
        return contact

    def _build_loaded_z_plane_mm(self):
        z_contact = []
        z_all = []
        contact_status = []

        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted or (not ctx.ready_to_lift):
                continue
            if ctx.slide_pos is None:
                continue

            z_mm = float(ctx.slide_pos[2])
            z_all.append(z_mm)
            
            is_contact = self._force_contact_ok(rn)
            if is_contact:
                z_contact.append(z_mm)
                contact_status.append(f'{rn}:z={z_mm:.1f}mm(contact)')
            else:
                contact_status.append(f'{rn}:z={z_mm:.1f}mm(no_contact)')

        if z_contact:
            z_plane = max(z_contact)
            self.get_logger().warn(
                f'[Z_PLANE] contact-gated: {contact_status} -> z_plane={z_plane:.2f}mm from {len(z_contact)}/{len(z_all)} contacted slides'
            )
            return z_plane
        
        if z_all:
            z_plane = max(z_all)
            self.get_logger().warn(
                f'[Z_PLANE] fallback (no contact detected): {contact_status} -> z_plane={z_plane:.2f}mm from all {len(z_all)} slides'
            )
            return z_plane
        
        self.get_logger().error('[Z_PLANE] no valid slide positions available')
        return None

    def _force_slope_abs_nps(self, rn: str) -> float:
        ctx = self.rt[rn]
        hist = getattr(ctx, 'force_hist', None)
        if hist is None or len(hist) < 2:
            return 1e9

        t0, f0 = hist[0]
        t1, f1 = hist[-1]
        dt = max(1e-6, float(t1 - t0))
        return abs((float(f1) - float(f0)) / dt)

    def _transport_chassis_twist_body(self, rn: str):
        if not bool(getattr(self, 'transport_chassis_fusion_enable', False)):
            return None

        ctx = self.rt[rn]
        now = _now_sec(self)
        timeout = float(getattr(self, 'transport_chassis_fresh_timeout_sec', 0.4))

        vx_samples = []
        vy_samples = []
        wz_samples = []

        # Odom branch is disabled for the current setup.
        # if bool(getattr(self, 'transport_ff_use_odom', True)):
        #     odom = getattr(ctx, 'odom_twist_body', None)
        #     if odom is not None and (now - float(getattr(ctx, 'odom_stamp', 0.0))) <= timeout:
        #         vx_samples.append(float(odom[0]))
        #         vy_samples.append(float(odom[1]))
        #         wz_samples.append(float(odom[2]))

        if bool(getattr(self, 'transport_ff_use_mocap', True)):
            mocap_body = getattr(ctx, 'mocap_twist_body', None)
            if mocap_body is not None and (now - float(getattr(ctx, 'mocap_twist_stamp', 0.0))) <= timeout:
                vx_samples.append(float(mocap_body[0]))
                vy_samples.append(float(mocap_body[1]))
            mocap_wz = getattr(ctx, 'mocap_wz', None)
            if mocap_wz is not None and (now - float(getattr(ctx, 'mocap_twist_stamp', 0.0))) <= timeout:
                wz_samples.append(float(mocap_wz))

        # IMU branch is disabled for the current setup.
        # if bool(getattr(self, 'transport_ff_use_imu', True)):
        #     imu_wz = getattr(ctx, 'imu_wz', None)
        #     if imu_wz is not None and (now - float(getattr(ctx, 'imu_stamp', 0.0))) <= timeout:
        #         wz_samples.append(float(imu_wz))

        if not vx_samples or not vy_samples:
            return None

        vx_body = sum(vx_samples) / len(vx_samples)
        vy_body = sum(vy_samples) / len(vy_samples)
        wz = (sum(wz_samples) / len(wz_samples)) if wz_samples else 0.0
        return (vx_body, vy_body, wz)

    def _transport_chassis_stable_ok_one(self, rn: str) -> bool:
        twist = self._transport_chassis_twist_body(rn)
        if twist is None:
            return False

        vx_body, vy_body, wz = twist
        v_body = math.hypot(vx_body, vy_body)
        return (
            v_body <= float(getattr(self, 'transport_chassis_stable_body_vel_tol_mps', 0.015))
            and abs(wz) <= float(getattr(self, 'transport_chassis_stable_yaw_rate_tol_rps', 0.08))
        )

    def _load_stable_ok_one(self, rn: str, require_ready_flag: bool = True) -> bool:
        ctx = self.rt[rn]
        if ctx.faulted:
            ctx.load_stable_vel_ok = False
            ctx.load_stable_force_fresh_ok = False
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False
        if require_ready_flag and (not ctx.ready_to_lift):
            ctx.load_stable_vel_ok = False
            ctx.load_stable_force_fresh_ok = False
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False

        if ctx.slide_pos is None or ctx.slide_vel is None:
            ctx.load_stable_vel_ok = False
            ctx.load_stable_force_fresh_ok = False
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False

        if not self._force_msg_fresh(rn, self.load_stable_force_fresh_timeout_sec):
            ctx.load_stable_force_fresh_ok = False
            ctx.load_stable_vel_ok = True
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False
        ctx.load_stable_force_fresh_ok = True

        vx, vy, vz = ctx.slide_vel
        vxy = math.hypot(vx, vy)
        if vxy > self.load_stable_slide_vel_tol_mmps:
            ctx.load_stable_vel_ok = False
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False
        if abs(vz) > self.load_stable_slide_vel_tol_mmps:
            ctx.load_stable_vel_ok = False
            ctx.load_stable_delta_fresh_ok = False
            ctx.load_stable_force_slope_ok = False
            ctx.load_stable_residual_ok = False
            return False
        ctx.load_stable_vel_ok = True

        delta_ok = False
        if ctx.delta_latest is not None and ctx.last_delta_stamp is not None:
            if (_now_sec(self) - ctx.last_delta_stamp) <= self.load_stable_delta_fresh_timeout_sec:
                ctx.load_stable_delta_fresh_ok = True
                dx, dy, dz = ctx.delta_latest
                if (
                    abs(dx) <= self.load_stable_xy_tol_m and
                    abs(dy) <= self.load_stable_xy_tol_m and
                    abs(dz) <= self.load_stable_z_tol_m
                ):
                    delta_ok = True
            else:
                ctx.load_stable_delta_fresh_ok = False
        else:
            ctx.load_stable_delta_fresh_ok = False

        if (not delta_ok) and (not self._transport_chassis_stable_ok_one(rn)):
            ctx.load_stable_residual_ok = False
            ctx.load_stable_force_slope_ok = False
            return False
        ctx.load_stable_residual_ok = True

        slope = self._force_slope_abs_nps(rn)
        if slope > self.load_stable_force_slope_tol_nps:
            ctx.load_stable_force_slope_ok = False
            return False
        ctx.load_stable_force_slope_ok = True

        return True

    def _all_load_stable(self, require_ready_flag: bool = True) -> bool:
        return all(
            self._load_stable_ok_one(rn, require_ready_flag=require_ready_flag)
            for rn in self.robots if (not self.rt[rn].faulted)
        )

    def _capture_transport_center_ref(self):
        missing = []
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue
            if ctx.slide_pos is None:
                missing.append(rn)
                continue
            ctx.transport_center_ref = (
                float(ctx.slide_pos[0]),
                float(ctx.slide_pos[1]),
                float(ctx.slide_pos[2]),
            )
            ctx.loaded_ref_captured = True
        return (len(missing) == 0), missing

    def _capture_loaded_transport_refs(self):
        ok, _ = self._capture_transport_center_ref()
        return ok

    # ------------------------------------------------------------------
    # entry / speed profile / finish conditions
    # ------------------------------------------------------------------
    def _near_wing(self, rn: str) -> bool:
        if self.wing_x is None or self.wing_y is None:
            return False
        p = self.robot_xy.get(rn)
        if p is None:
            return False
        return math.hypot(p[0] - self.wing_x, p[1] - self.wing_y) <= self.gate_near_wing_r

    def _robot_stage(self, rn: str) -> str:
        ctx = self.rt[rn]
        if ctx.faulted:
            return 'ERROR'
        if ctx.finished:
            return 'DONE'
        if ctx.transporting:
            return 'TRANSPORT'
        if getattr(ctx, 'level_active', False):
            return 'LEVEL_Z'
        if ctx.recenter_done:
            return 'RECENTERED'
        if ctx.ready_to_lift:
            return 'READY'
        if ctx.fine_active:
            return 'SLIDE_ALIGNING'
        if getattr(ctx, 'sync_wait_qr', False):
            return 'WAIT_QR'
        if ctx.first_qr_locked and not ctx.ready_to_lift:
            return 'WAIT_ALL_QR_LOCK'
        if ctx.dwell_start > 0.0:
            return 'WAIT_QR'
        if ctx.staged and not ctx.entered:
            return 'STAGED'
        if ctx.goal_kind == 'APPROACH_X' and ctx.segs is not None:
            return 'NAV_SYNC_X'
        if ctx.goal_kind == 'APPROACH_Y' and ctx.segs is not None:
            return 'NAV_SYNC_Y'
        if ctx.goal_kind == 'STAGING' and ctx.segs is not None:
            return 'NAV_STAGING'
        if ctx.goal_kind == 'FINAL' and ctx.segs is not None:
            return 'NAV_FINAL'
        return 'COARSE'

    def _entry_hold(self, rn: str, ctx) -> bool:
        return self.entry_enable and ctx.staged and (not ctx.entered) and (self.entry_owner is not None) and (rn != self.entry_owner)

    def _leader_robot(self):
        if self.entry_owner is not None:
            return self.entry_owner
        return self.dispatch_order[0] if self.dispatch_order else None

    def update_entry_owner(self):
        if not self.entry_enable:
            self.entry_owner = None
            return

        if self.entry_owner is not None:
            owner_ctx = self.rt[self.entry_owner]
            if owner_ctx.confirmed or owner_ctx.first_qr_locked:
                self.entry_owner = None

        if self.entry_owner is None:
            for rn in self.dispatch_order:
                ctx = self.rt[rn]
                if ctx.faulted or ctx.finished:
                    continue
                if ctx.confirmed or ctx.first_qr_locked:
                    continue
                self.entry_owner = rn
                break

    def _update_speed_profile(self, rn: str):
        if not self.approach_slow_enable:
            return

        ctx = self.rt[rn]
        if (
            ctx.faulted or ctx.finished or ctx.ready_to_lift or ctx.fine_active or
            ctx.first_qr_locked or (ctx.dwell_start > 0.0) or ctx.gate_stopped or self.emergency
        ):
            return
        if ctx.goal_kind not in ('FINAL', 'TRANSPORT'):
            return

        target = ctx.transport_target if ctx.goal_kind == 'TRANSPORT' else ctx.final_target
        if target is None or rn not in self.robot_xy:
            return

        x, y = self.robot_xy[rn]
        xt, yt = target
        d = math.hypot(x - xt, y - yt)

        if (not ctx.slow_mode) and (d <= self.approach_slow_r):
            ctx.slow_mode = True
            self.precision_on(rn, True)
        elif ctx.slow_mode and (d >= self.approach_slow_r + self.approach_slow_hyst):
            ctx.slow_mode = False
            self.precision_on(rn, False)

    def _all_ready_to_lift(self) -> bool:
        return all((not self.rt[rn].faulted) and self.rt[rn].ready_to_lift for rn in self.robots)

    def _transport_target_pose_error(self, rn: str):
        ctx = self.rt[rn]
        if ctx.transport_target is None or rn not in self.robot_xy:
            return None

        x, y = self.robot_xy[rn]
        xt, yt = ctx.transport_target
        pos_err = math.hypot(x - xt, y - yt)

        yaw_err_deg = 0.0
        if ctx.locked_yaw is not None and rn in self.robot_yaw:
            yaw_err_deg = abs(math.degrees(wrap_angle_rad(self.robot_yaw[rn] - ctx.locked_yaw)))

        return pos_err, yaw_err_deg

    def _transport_finish_pose_ok_one(self, rn: str) -> bool:
        err = self._transport_target_pose_error(rn)
        if err is None:
            return False
        pos_err, yaw_err_deg = err
        return (
            pos_err <= float(getattr(self, 'transport_finish_pos_tol_m', 0.08))
            and yaw_err_deg <= float(getattr(self, 'transport_finish_yaw_tol_deg', 6.0))
        )

    def _transport_pose_stable_ok_one(self, rn: str) -> bool:
        ctx = self.rt[rn]
        now = _now_sec(self)
        vel_tol_mps = float(getattr(self, 'transport_settle_pose_vel_tol_mps', 0.03))
        yaw_rate_tol_rps = float(getattr(self, 'transport_settle_pose_yaw_rate_tol_rps', 0.15))

        if (
            ctx.mocap_twist_world is not None and
            ctx.mocap_twist_stamp > 0.0 and
            (now - ctx.mocap_twist_stamp) <= self.mocap_timeout_sec
        ):
            vx, vy = ctx.mocap_twist_world
            wz = float(ctx.mocap_wz) if ctx.mocap_wz is not None else 0.0
            return math.hypot(vx, vy) <= vel_tol_mps and abs(wz) <= yaw_rate_tol_rps

        if rn not in self.robot_xy:
            return False

        pose_stamp = float(self.robot_pose_stamp.get(rn, 0.0))
        if pose_stamp <= 0.0:
            return False

        cur_xy = self.robot_xy[rn]
        last_xy = ctx.chassis_check_last_xy
        last_stamp = float(ctx.chassis_check_last_stamp)

        ctx.chassis_check_last_xy = (float(cur_xy[0]), float(cur_xy[1]))
        ctx.chassis_check_last_stamp = pose_stamp

        if last_xy is None or last_stamp <= 0.0 or pose_stamp <= last_stamp:
            return False

        dt = pose_stamp - last_stamp
        if dt < 0.10:
            return False

        dist = math.hypot(cur_xy[0] - last_xy[0], cur_xy[1] - last_xy[1])
        return (dist / max(dt, 1e-6)) <= vel_tol_mps

    def _transport_settle_ok_one(self, rn: str) -> bool:
        ctx = self.rt[rn]
        if ctx.faulted or (not ctx.transport_arrived):
            return False

        if not self._transport_finish_pose_ok_one(rn):
            return False

        return True

    def _all_transport_arrived(self) -> bool:
        arrived = [
            self.rt[rn].transport_arrived or self.rt[rn].transport_settled or self.rt[rn].finished
            for rn in self.robots
            if not self.rt[rn].faulted
        ]
        return (len(arrived) > 0) and all(arrived)

    def _all_transport_settled(self) -> bool:
        settled = [
            self.rt[rn].transport_settled or self.rt[rn].finished
            for rn in self.robots
            if not self.rt[rn].faulted
        ]
        return (len(settled) > 0) and all(settled)

    def _all_transport_finished(self) -> bool:
        return all((not self.rt[rn].faulted) and self.rt[rn].finished for rn in self.robots)

    def _has_any_fault(self) -> bool:
        return any(self.rt[rn].faulted for rn in self.robots)

    # ------------------------------------------------------------------
    # workflow preflight checks and gating
    # ------------------------------------------------------------------
    def _check_workflow_preflight(self) -> bool:
        """检查当前 workflow 的前置条件是否满足"""
        wf = getattr(self, 'workflow', 'full')
        skip = getattr(self, 'skip_preflight', False)
        
        if skip:
            if not getattr(self, '_skip_preflight_logged', False):
                self.get_logger().warn(f'[WORKFLOW={wf}] skip_preflight=True, bypassing all preflight checks')
                self._skip_preflight_logged = True
            return True
        
        if wf == 'full':
            return True
        
        if wf == 'approach':
            return True
        
        if wf == 'lift':
            ok, missing = self._can_start_lift_workflow()
            if not ok:
                now = _now_sec(self)
                if now - getattr(self, 'preflight_last_log', 0.0) >= 3.0:
                    self.get_logger().warn(f'[WORKFLOW=lift] Preflight check FAILED: {missing}')
                    self.get_logger().warn('[WORKFLOW=lift] Waiting for preflight conditions...')
                    self.get_logger().warn('[WORKFLOW=lift] TIP: Use skip_preflight:=true to bypass checks for testing')
                    self.preflight_last_log = now
            return ok
        
        if wf == 'transport':
            ok, missing = self._can_start_transport_workflow()
            if not ok:
                now = _now_sec(self)
                if now - getattr(self, 'preflight_last_log', 0.0) >= 3.0:
                    self.get_logger().warn(f'[WORKFLOW=transport] Preflight check FAILED: {missing}')
                    self.get_logger().warn('[WORKFLOW=transport] Waiting for preflight conditions...')
                    self.get_logger().warn('[WORKFLOW=transport] TIP: Use skip_preflight:=true to bypass checks for testing')
                    self.preflight_last_log = now
            return ok
        
        self.get_logger().error(f'[WORKFLOW] Unknown workflow: {wf}')
        return False

    def _can_start_lift_workflow(self):
        """检查是否可以启动 lift workflow
        Returns: (bool, str) - (是否满足, 缺失项描述)
        """
        missing = []
        
        qr_locked = [rn for rn in self.robots if not self.rt[rn].first_qr_locked]
        if qr_locked:
            missing.append(f'QR not locked: {qr_locked}')
        
        no_slide = [rn for rn in self.robots if self.rt[rn].slide_pos is None]
        if no_slide:
            missing.append(f'slide offline: {no_slide}')
        
        no_force = []
        for rn in self.robots:
            if not self._force_msg_fresh(rn, 1.0):
                no_force.append(rn)
        if no_force:
            missing.append(f'force sensor stale: {no_force}')
        
        moving = []
        for rn in self.robots:
            ctx = self.rt[rn]
            is_moving = False

            if abs(ctx.last_cmd_v) > 0.05 or abs(ctx.last_cmd_w) > 0.05:
                is_moving = True

            if rn in self.robot_xy and rn in self.robot_pose_stamp:
                now_stamp = self.robot_pose_stamp[rn]
                if ctx.chassis_check_last_xy is None:
                    ctx.chassis_check_last_xy = self.robot_xy[rn]
                    ctx.chassis_check_last_stamp = now_stamp
                else:
                    dt = now_stamp - ctx.chassis_check_last_stamp
                    if dt > 0.1:
                        x0, y0 = ctx.chassis_check_last_xy
                        x1, y1 = self.robot_xy[rn]
                        dist = math.hypot(x1 - x0, y1 - y0)
                        vel = dist / dt
                        if vel > 0.05:
                            is_moving = True

                        ctx.chassis_check_last_xy = (x1, y1)
                        ctx.chassis_check_last_stamp = now_stamp

            if is_moving:
                moving.append(rn)
        
        if moving:
            missing.append(f'chassis not stopped: {moving}')
        
        if missing:
            return False, '; '.join(missing)
        return True, ''

    def _transport_precheck_ok(self):
        """检查是否可以安全进入 transport 闭环。"""
        missing = []

        now = _now_sec(self)
        wing_ok = (
            self._wing_frozen and
            self.wing_x is not None and
            self.wing_y is not None and
            self.wing_yaw is not None
        ) or (
            self.wing_pose_stamp > 0.0 and
            (now - self.wing_pose_stamp) <= self.mocap_timeout_sec
        )
        if not wing_ok:
            missing.append('wing mocap stale/missing')

        robot_bad = [
            rn for rn in self.robots
            if self.robot_pose_stamp.get(rn, 0.0) <= 0.0 or
            (now - self.robot_pose_stamp.get(rn, 0.0)) > self.mocap_timeout_sec
        ]
        if robot_bad:
            missing.append(f'robot mocap stale/missing: {robot_bad}')

        no_slide = [rn for rn in self.robots if self.rt[rn].slide_pos is None]
        if no_slide:
            missing.append(f'slide offline: {no_slide}')
        
        no_force = []
        low_force = []
        for rn in self.robots:
            if not self._force_msg_fresh(rn, 1.0):
                no_force.append(rn)
            else:
                ctx = self.rt[rn]
                ff = getattr(ctx, 'force_f', None)
                if ff is not None:
                    fz = abs(float(ff[2]))
                    if fz < self.load_level_contact_force_n:
                        low_force.append(f'{rn}(fz={fz:.1f}N)')
        
        if no_force:
            missing.append(f'force sensor stale: {no_force}')
        if low_force:
            missing.append(f'force below load threshold ({self.load_level_contact_force_n}N): {low_force}')
        
        z_out_of_range = []
        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.slide_pos is not None:
                z_mm = float(ctx.slide_pos[2])
                z_min = getattr(self, 'load_level_z_plane_min_mm', 80.0)
                z_max = getattr(self, 'load_level_z_plane_max_mm', 180.0)
                if z_mm < z_min or z_mm > z_max:
                    z_out_of_range.append(f'{rn}(z={z_mm:.1f}mm)')
        
        if z_out_of_range:
            missing.append(f'Z not in lifted range [{z_min:.0f}, {z_max:.0f}]mm: {z_out_of_range}')
        
        if not self._all_load_stable(require_ready_flag=False):
            missing.append('load not stable')
        
        if self.emergency:
            missing.append('emergency stop active')
        
        if self._has_any_fault():
            faulted = [rn for rn in self.robots if self.rt[rn].faulted]
            missing.append(f'robot faulted: {faulted}')

        ref_ok, ref_missing = self._capture_transport_center_ref()
        if not ref_ok:
            missing.append(f'transport_center_ref capture failed: {ref_missing}')
        
        if missing:
            return False, '; '.join(missing)
        return True, ''

    def _can_start_transport_workflow(self):
        """检查是否可以启动 transport workflow（物理态驱动）"""
        return self._transport_precheck_ok()

    def _transport_consistency_ok(self):
        """transport 运行中一致性检查。

        保守策略：只要关键检查失败，就交由上层整体停车/中止。
        """
        missing = []
        now = _now_sec(self)
        wing_ok = (
            self._wing_frozen and
            self.wing_x is not None and
            self.wing_y is not None and
            self.wing_yaw is not None
        ) or (
            self.wing_pose_stamp > 0.0 and
            (now - self.wing_pose_stamp) <= self.mocap_timeout_sec
        )
        if (not wing_ok) and bool(getattr(self, 'transport_abort_on_wing_pose_lost', True)):
            missing.append('wing mocap stale/missing during transport')

        robot_bad = [
            rn for rn in self.robots
            if (not self.rt[rn].faulted) and (
                self.robot_pose_stamp.get(rn, 0.0) <= 0.0 or
                (now - self.robot_pose_stamp.get(rn, 0.0)) > self.mocap_timeout_sec
            )
        ]
        if robot_bad:
            missing.append(f'robot mocap stale/missing during transport: {robot_bad}')

        current_pts = []
        target_pts = []
        max_yaw_err_deg = 0.0
        formation_error_m = 0.0

        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue
            if ctx.transport_target is None:
                missing.append(f'{rn} transport_target missing')
                continue
            if rn not in self.robot_xy:
                missing.append(f'{rn} mocap pose missing')
                continue

            current_pts.append(self.robot_xy[rn])
            target_pts.append(ctx.transport_target)

            err = self._transport_target_pose_error(rn)
            if err is not None:
                _, yaw_err_deg = err
                max_yaw_err_deg = max(max_yaw_err_deg, yaw_err_deg)

        if missing:
            reason = '; '.join(missing)
            for rn in self.robots:
                self.rt[rn].group_stop_reason = reason
            return False, reason

        cx = sum(p[0] for p in current_pts) / len(current_pts)
        cy = sum(p[1] for p in current_pts) / len(current_pts)
        tx = sum(p[0] for p in target_pts) / len(target_pts)
        ty = sum(p[1] for p in target_pts) / len(target_pts)
        center_error_m = math.hypot(cx - tx, cy - ty)
        for cur_pt, tgt_pt in zip(current_pts, target_pts):
            cur_rel_x = cur_pt[0] - cx
            cur_rel_y = cur_pt[1] - cy
            tgt_rel_x = tgt_pt[0] - tx
            tgt_rel_y = tgt_pt[1] - ty
            formation_error_m = max(
                formation_error_m,
                math.hypot(cur_rel_x - tgt_rel_x, cur_rel_y - tgt_rel_y)
            )

        combined_error_m = max(center_error_m, formation_error_m)

        for rn in self.robots:
            ctx = self.rt[rn]
            if ctx.faulted:
                continue
            ctx.formation_error_m = combined_error_m
            ctx.formation_error_yaw_deg = max_yaw_err_deg
            ctx.group_stop_reason = ''

        if combined_error_m > float(getattr(self, 'transport_max_center_error_m', 0.20)):
            reason = (
                f'transport center/formation error too large: center={center_error_m:.3f}m '
                f'formation={formation_error_m:.3f}m '
                f'(>{float(getattr(self, "transport_max_center_error_m", 0.20)):.3f}m)'
            )
            for rn in self.robots:
                self.rt[rn].group_stop_reason = reason
            return False, reason

        return True, ''

    def _workflow_allows_state(self, target_state: str) -> bool:
        """检查当前 workflow 是否允许进入目标状态"""
        wf = getattr(self, 'workflow', 'full')
        
        if wf == 'full':
            return True
        
        common_allowed = ['ABORT', 'DONE', 'STANDBY']
        if target_state in common_allowed:
            return True
        
        if wf == 'approach':
            allowed = [
                'WAIT_WING',
                'SYNC_APPROACH_X',
                'SYNC_APPROACH_Y',
                'WAIT_ENTRY_RELEASE',
                'RUN_ALIGNMENT',
                'PHASE1_DONE_HOLD',
            ]
            return target_state in allowed
        
        if wf == 'lift':
            allowed = ['SYNC_SLIDE_ALIGN', 'ALL_READY_HOLD', 'SYNC_LEVEL_Z', 'SYNC_RECENTER', 'LOAD_STABLE_HOLD']
            return target_state in allowed
        
        if wf == 'transport':
            allowed = ['TRANSPORT_PRECHECK', 'SYNC_TRANSPORT', 'TRANSPORT_SETTLE', 'DONE']
            return target_state in allowed
        
        return False
