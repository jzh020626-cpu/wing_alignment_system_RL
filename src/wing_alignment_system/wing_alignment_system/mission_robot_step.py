# -*- coding: utf-8 -*-

from std_msgs.msg import Bool
from wing_alignment_system.mission_geometry import _now_sec


class MissionRobotStepMixin:
    def step_robot(self, rn: str):
        ctx = self.rt[rn]

        if self.emergency or self.state in ('ABORT', 'DONE'):
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            return

        if ctx.faulted or ctx.finished:
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            return

        if self.state in ('RUN_ALIGNMENT', 'SYNC_SLIDE_ALIGN', 'ALL_READY_HOLD', 'SYNC_RECENTER', 'SYNC_LIFT', 'SYNC_LEVEL_Z', 'LOAD_STABLE_HOLD') and ctx.ready_to_lift:
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            return

        if self.state == 'TRANSPORT_PRECHECK':
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            return

        if self.state == 'TRANSPORT_SETTLE':
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)

            if not ctx.transport_arrived:
                return

            if not self._transport_settle_ok_one(rn):
                ctx.transport_settled = False
                ctx.transport_settle_stamp = 0.0
                self._set_local_state(rn, 'TRANSPORT_WAIT_SETTLE', 'transport arrived; waiting stable settle confirmation')
                return

            if ctx.transport_settle_stamp <= 0.0:
                ctx.transport_settle_stamp = _now_sec(self)
                self._set_local_state(rn, 'TRANSPORT_WAIT_SETTLE', 'transport pose and chassis stable; starting settle timer')
                return

            if (_now_sec(self) - ctx.transport_settle_stamp) >= self.transport_settle_sec:
                if not ctx.transport_settled:
                    ctx.transport_settled = True
                    ctx.transporting = False
                    ctx.finished = True
                    self._set_local_state(rn, 'FINISHED', 'transport settled and target maintained')
            return

        if self.state == 'SYNC_TRANSPORT':
            if (not ctx.transport_dispatched) and (not ctx.transport_arrived):
                self.dispatch_transport_one(rn)
                return
            if ctx.transport_arrived:
                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)
                return

        if self.state == 'SYNC_SLIDE_ALIGN' and ctx.sync_wait_qr:
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)

            if not self._alignment_vision_enabled():
                self._restart_sync_slide_align_one(
                    rn,
                    'alignment vision disabled; resume z-only contact alignment'
                )
                return

            if self._raw_qr_fresh(rn):
                self._restart_sync_slide_align_one(
                    rn,
                    'stable raw QR reacquired during synchronized slide alignment; resume fine alignment'
                )
                emit_event = getattr(self, '_bench_emit_event', None)
                if callable(emit_event):
                    emit_event(
                        event_type='REACQUIRE_SUCCESS',
                        robot_id=rn,
                        phase=self.state.lower(),
                        reason='raw_qr_reacquired',
                        note='stable raw QR reacquired; resuming slide alignment',
                    )
                return

            if ctx.sync_wait_qr_epoch > 0.0 and (_now_sec(self) - ctx.sync_wait_qr_epoch) >= self.wait_qr_fail_timeout_sec:
                emit_event = getattr(self, '_bench_emit_event', None)
                if callable(emit_event):
                    emit_event(
                        event_type='DEGRADED_HOLD_TIMEOUT',
                        robot_id=rn,
                        phase=self.state.lower(),
                        reason='wait_qr_timeout',
                        note=self._wait_qr_timeout_reason(rn, 'SYNC_WAIT_QR_TIMEOUT_NO_STABLE_RAW_QR'),
                    )
                self._fail_robot(rn, self._wait_qr_timeout_reason(rn, 'SYNC_WAIT_QR_TIMEOUT_NO_STABLE_RAW_QR'))
                return

            self._set_local_state(rn, 'WAIT_QR', 'slide alignment visual lost; hold base and wait stable raw QR reacquire')
            return

        if self.state == 'RUN_ALIGNMENT' and ctx.first_qr_locked and (not ctx.ready_to_lift):
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            self._set_local_state(
                rn,
                'WAIT_ALL_QR_LOCK',
                'coarse final parked and alignment vision bypassed; base frozen until all robots park'
                if not self._alignment_vision_enabled() else
                'coarse final parked and QR confirmed; base frozen until all robots lock'
            )
            return

        if self.entry_enable and ctx.staged and not ctx.entered:
            if self._entry_hold(rn, ctx):
                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)
                self._set_local_state(rn, 'WAIT_ENTRY', 'waiting for entry owner permission')
                return
            if self.entry_owner == rn and not ctx.entered:
                ctx.entered = True
                self.dispatch_to_final_one(rn, tag='FINAL')
                return

        self._update_speed_profile(rn)

        if ctx.gate_stopped:
            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            return

        if ctx.fine_active:
            self.stop_pub[rn].publish(Bool(data=True))
            self.precision_on(rn, False)
            return

        self._update_final_precision(rn)

        if ctx.segs is not None and self.reached_ok(rn):
            if ctx.seg_i + 1 < len(ctx.segs):
                ctx.seg_i += 1
                tag = 'TRANSPORT' if ctx.goal_kind == 'TRANSPORT' else 'PATH'
                self._send_current_segment(rn, tag=tag)
                return

            ctx.segs = None
            ctx.seg_i = 0

            if ctx.goal_kind == 'STAGING':
                ctx.staged = True
                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)
                self._set_local_state(rn, 'WAIT_ENTRY', 'staging target reached')
                return

            if ctx.goal_kind in ('APPROACH_X', 'APPROACH_Y'):
                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)
                if ctx.goal_kind == 'APPROACH_X':
                    self._set_local_state(rn, 'SYNC_APPROACH_X_HOLD', 'cooperative x leg reached; waiting team barrier')
                else:
                    self._set_local_state(rn, 'SYNC_APPROACH_Y_HOLD', 'cooperative y leg reached; waiting team barrier')
                return

            if ctx.goal_kind == 'TRANSPORT':
                ctx.transporting = False
                ctx.transport_arrived = True
                ctx.transport_arrive_stamp = _now_sec(self)
                ctx.transport_settle_stamp = 0.0
                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)
                self._set_local_state(rn, 'TRANSPORT_ARRIVED', 'transport target reached; waiting group settle barrier')
                self.get_logger().info(f'[TRANSPORT_ARRIVED] {rn} transport target reached; waiting settle barrier')
                return

            if not self._alignment_vision_enabled():
                ctx.dwell_start = 0.0
                ctx.dwell_locked = False
                ctx.sync_wait_qr = False
                ctx.sync_wait_qr_epoch = 0.0
                self._reset_raw_qr_tracking(ctx)
                if ctx.te is not None:
                    ctx.te.reset()

                ctx.first_qr_locked = True
                ctx.first_qr_lock_epoch = _now_sec(self)

                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)

                if self.entry_owner == rn:
                    self.entry_owner = None

                self._set_local_state(
                    rn,
                    'WAIT_ALL_QR_LOCK',
                    'coarse final goal reached; alignment vision disabled, park here and release next robot'
                )
                self.get_logger().warn(
                    f'[COARSE_PARKED] {rn} coarse final goal reached with alignment_vision_enable=False -> freeze base here.'
                )
                return

            ctx.dwell_start = _now_sec(self)
            ctx.dwell_locked = True

            ctx.qr_zero_pending = False
            ctx.qr_zero_done = False
            ctx.qr_zero_req_epoch = 0.0
            ctx.delta_armed_since = 0.0
            ctx.delta_latest = None
            ctx.last_delta_stamp = None
            ctx.sync_wait_qr = False
            ctx.sync_wait_qr_epoch = 0.0
            self._reset_raw_qr_tracking(ctx)

            if ctx.te is not None:
                ctx.te.reset()

            self.request_qr_reset_tracking(rn)

            self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)
            self._set_local_state(rn, 'WAIT_QR', 'coarse final goal reached, hold base and wait stable raw QR')
            return

        if ctx.dwell_start > 0.0:
            if ctx.dwell_locked:
                self.stop_pub[rn].publish(Bool(data=True))
            self.stop_slide_comp(rn)
            self.precision_on(rn, False)

            elapsed = _now_sec(self) - ctx.dwell_start

            if self._raw_qr_fresh(rn) and (not ctx.first_qr_locked):
                ctx.first_qr_locked = True
                ctx.first_qr_lock_epoch = _now_sec(self)

                self.stop_pub[rn].publish(Bool(data=True))
                self.stop_slide_comp(rn)
                self.precision_on(rn, False)

                if self.entry_owner == rn:
                    self.entry_owner = None

                self._set_local_state(
                    rn,
                    'WAIT_ALL_QR_LOCK',
                    'stable raw QR confirmed after coarse final stop; park here and release next robot'
                )
                self.get_logger().warn(
                    f'[QR_PARKED] {rn} stable raw QR confirmed after coarse final stop -> freeze base here.'
                )
                return

            if self.state == 'RUN_ALIGNMENT':
                if not self.base_micro_search_enable:
                    if elapsed >= self.wait_qr_fail_timeout_sec:
                        self._fail_robot(rn, self._wait_qr_timeout_reason(rn, 'WAIT_QR_TIMEOUT_NO_STABLE_RAW_QR'))
                        return

                    self._set_local_state(rn, 'WAIT_QR', 'holding at coarse final goal; waiting for stable raw QR')
                    return

                ctx.dwell_start = 0.0
                ctx.dwell_locked = False
                ctx.micro_attempts += 1
                if ctx.micro_attempts > self.micro_max_attempts:
                    self._fail_robot(rn, 'MICRO_ATTEMPTS_EXCEEDED')
                    return

                candidate = self._next_micro_offset(ctx)
                if candidate is None:
                    self._fail_robot(rn, 'MICRO_OFFSETS_INVALID')
                    return

                dxm, dym = candidate
                self.resume_one(rn)
                self.dispatch_to_final_one(rn, float(dxm), float(dym), tag=f'MICRO#{ctx.micro_attempts}')
                return

            self._set_local_state(rn, 'WAIT_QR', 'synchronized stage: hold base only')
            return
