# -*- coding: utf-8 -*-

import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from wing_alignment_system.mission_geometry import _now_sec, wrap_angle_rad


class MissionDispatcherMixin:
    def _final_precision_enabled_for(self, rn: str) -> bool:
        if not bool(getattr(self, 'final_precision_enable', True)):
            return False
        robots = getattr(self, 'final_precision_robots', ['tracer1', 'tracer2', 'tracer3'])
        if isinstance(robots, str):
            items = [item.strip() for item in robots.split(',')]
        else:
            items = [str(item).strip() for item in robots]
        items = [item for item in items if item]
        return ('all' in items) or (str(rn) in items)

    def _final_precision_window_m(self) -> float:
        configured = float(getattr(self, 'final_precision_window_m', 0.35))
        return max(configured, float(getattr(self, 'raw_qr_accept_radius_m', 0.18)))

    def _clear_final_precision(self, rn: str, disable_precision: bool = False):
        ctx = self.rt[rn]
        setattr(ctx, '_final_precision_pending', False)
        setattr(ctx, '_final_precision_active', False)
        setattr(ctx, '_final_precision_window_m', 0.0)
        if disable_precision:
            self.precision_on(rn, False)

    def _clear_tracer2_final_precision(self, rn: str, disable_precision: bool = False):
        self._clear_final_precision(rn, disable_precision=disable_precision)

    def _update_final_precision(self, rn: str):
        ctx = self.rt[rn]
        pending = bool(getattr(ctx, '_final_precision_pending', False))
        active = bool(getattr(ctx, '_final_precision_active', False))

        if (
            (not pending)
            or (not self._final_precision_enabled_for(rn))
            or ctx.goal_kind != 'FINAL'
            or ctx.segs is None
            or ctx.seg_i != (len(ctx.segs) - 1)
            or ctx.final_target is None
            or rn not in self.robot_xy
        ):
            if pending or active:
                self._clear_final_precision(rn, disable_precision=active)
            return

        if active:
            return

        xt, yt = ctx.final_target
        rx, ry = self.robot_xy[rn]
        dist_to_final = math.hypot(float(rx) - float(xt), float(ry) - float(yt))
        threshold_m = float(getattr(ctx, '_final_precision_window_m', 0.0))
        if dist_to_final > threshold_m:
            return

        setattr(ctx, '_final_precision_active', True)
        self.precision_on(rn, True)
        self.get_logger().warn(
            f'[FINAL_TIGHTEN] {rn} final precision activated '
            f'dist_to_final={dist_to_final:.3f} threshold={threshold_m:.3f} '
            f'final=({float(xt):.3f},{float(yt):.3f})'
        )

    def _update_tracer2_final_precision(self, rn: str):
        self._update_final_precision(rn)

    def _normalize_path_mode(self, mode: str, default: str = 'x_first') -> str:
        mode_norm = str(mode or '').lower().strip()
        if mode_norm in ('x_first', 'y_first'):
            return mode_norm
        return str(default or 'x_first').lower().strip()

    def _path_mode_for_goal_kind(self, goal_kind: str) -> str:
        kind = str(goal_kind or '').upper()
        if kind == 'STAGING':
            return self.staging_path_mode
        if kind == 'TRANSPORT':
            return self.transport_path_mode
        return self.final_path_mode

    def _path_mode_for_dispatch(self, rn: str, goal_kind: str, tag: str = '') -> str:
        kind = str(goal_kind or '').upper()
        return self._path_mode_for_goal_kind(kind)

    def predict_target_world(self, rn: str, micro_dx: float, micro_dy: float):
        off = self.tool_offsets[rn]
        c, s = math.cos(self.wing_yaw), math.sin(self.wing_yaw)
        dx, dy = c * off.x - s * off.y, s * off.x + c * off.y
        xw, yw = self.wing_x + dx + micro_dx, self.wing_y + dy + micro_dy
        yaw_rad = wrap_angle_rad(self.wing_yaw + math.radians(off.yaw_deg))
        return xw, yw, yaw_rad

    def predict_staging_world(self, rn: str):
        sx, sy = self.staging_offsets[rn]
        c, s = math.cos(self.wing_yaw), math.sin(self.wing_yaw)
        dx, dy = c * sx - s * sy, s * sx + c * sy
        return (self.wing_x + dx, self.wing_y + dy)

    def predict_transport_target_world(self, rn: str):
        off = self.tool_offsets[rn]
        wing_tx = self.wing_x + self.transport_goal_dx_m
        wing_ty = self.wing_y + self.transport_goal_dy_m
        wing_tyaw = wrap_angle_rad(self.wing_yaw + math.radians(self.transport_goal_dyaw_deg))
        c, s = math.cos(wing_tyaw), math.sin(wing_tyaw)
        dx, dy = c * off.x - s * off.y, s * off.x + c * off.y
        xw, yw = wing_tx + dx, wing_ty + dy
        yaw_rad = wrap_angle_rad(wing_tyaw + math.radians(off.yaw_deg))
        return xw, yw, yaw_rad

    def stop_all(self):
        for rn in self.robots:
            self.stop_pub[rn].publish(Bool(data=True))
            self.precision_on(rn, False)

    def resume_one(self, rn: str):
        self.resume_pub[rn].publish(Bool(data=True))

    def precision_on(self, rn: str, on: bool):
        self.precision_pub[rn].publish(Bool(data=bool(on)))

    def arm_delta(self, rn: str):
        ctx = self.rt[rn]
        ctx.delta_armed_since = _now_sec(self)
        ctx.delta_latest = None
        ctx.last_delta_stamp = None
        if ctx.te is not None:
            ctx.te.arm(__import__('time').time())

    def reached_ok(self, rn: str) -> bool:
        ctx = self.rt[rn]
        return ctx.reached and (_now_sec(self) - ctx.last_goal_epoch) >= self.reach_min_delay_sec

    def send_goal(self, rn: str, x: float, y: float, yaw_deg: float, profile_code: float = 0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(profile_code)
        msg.angular.z = float(yaw_deg) if self.use_goal_yaw else 0.0
        self.goal_pub[rn].publish(msg)

        ctx = self.rt[rn]
        ctx.reached = False
        ctx.last_goal_epoch = _now_sec(self)

    def _build_L_from(self, x0: float, y0: float, xt: float, yt: float, path_mode: str):
        seg1 = (xt, y0) if path_mode == 'x_first' else (x0, yt)
        segs = []
        if math.hypot(seg1[0] - x0, seg1[1] - y0) >= self.path_min_seg_m:
            segs.append(seg1)
        segs.append((xt, yt))
        if len(segs) == 2 and math.hypot(segs[1][0] - segs[0][0], segs[1][1] - segs[0][1]) < self.path_min_seg_m:
            return [segs[1]]
        return segs

    def build_L_segments(self, rn: str, xt: float, yt: float, goal_kind: str = 'FINAL', path_mode: str = None):
        if rn not in self.robot_xy:
            return [(xt, yt)]
        x0, y0 = self.robot_xy[rn]
        use_mode = self._normalize_path_mode(path_mode, default=self._path_mode_for_goal_kind(goal_kind))
        return self._build_L_from(x0, y0, xt, yt, use_mode)

    def _send_current_segment(self, rn: str, tag: str = 'PATH'):
        ctx = self.rt[rn]
        if ctx.segs is None or ctx.seg_i >= len(ctx.segs):
            return

        gx, gy = ctx.segs[ctx.seg_i]
        is_last_seg = (ctx.seg_i == len(ctx.segs) - 1)
        defer_final_precision = (
            self._final_precision_enabled_for(rn)
            and ctx.goal_kind == 'FINAL'
            and is_last_seg
        )

        if ctx.seg_i == 0:
            px, py = self.robot_xy.get(rn, (gx, gy))
        else:
            px, py = ctx.segs[ctx.seg_i - 1]

        dx, dy = gx - px, gy - py
        if math.hypot(dx, dy) < 0.02:
            yaw_rad = self.robot_yaw.get(rn, 0.0)
        else:
            yaw_rad = math.atan2(dy, dx)

        yaw_deg = math.degrees(yaw_rad)
        if ctx.locked_yaw is not None and is_last_seg and ctx.goal_kind in ('FINAL', 'TRANSPORT'):
            yaw_deg = math.degrees(ctx.locked_yaw)

        if ctx.goal_kind in ('STAGING', 'APPROACH_X', 'APPROACH_Y'):
            profile_code = getattr(self, 'staging_profile_code', 1.0)
        elif ctx.goal_kind == 'TRANSPORT':
            profile_code = getattr(self, 'transport_profile_code', 2.0)
        else:
            profile_code = 0.0

        if defer_final_precision:
            threshold_m = self._final_precision_window_m()
            setattr(ctx, '_final_precision_pending', True)
            setattr(ctx, '_final_precision_active', False)
            setattr(ctx, '_final_precision_window_m', threshold_m)
            self.precision_on(rn, False)
            self.get_logger().warn(
                f'[FINAL_TIGHTEN] {rn} final precision deferred until terminal window | '
                f'start_dist_to_final={math.hypot(float(gx) - float(px), float(gy) - float(py)):.3f} '
                f'threshold={threshold_m:.3f} '
                f'waypoint=({px:.3f},{py:.3f}) final=({gx:.3f},{gy:.3f})'
            )
        else:
            self._clear_final_precision(rn, disable_precision=True)

        self.send_goal(rn, gx, gy, yaw_deg, profile_code=profile_code)

    def dispatch_to_staging_one(self, rn: str):
        ctx = self.rt[rn]
        self._reset_runtime_for_new_mission_leg(rn, clear_alignment=True)
        ctx.micro_i = 0
        ctx.micro_attempts = 0
        ctx.locked_yaw = None

        xt, yt, _ = self.predict_target_world(rn, 0.0, 0.0)
        ctx.final_target = (xt, yt)
        xs, ys = self.predict_staging_world(rn)
        ctx.staging_target = (xs, ys)
        ctx.goal_kind = 'STAGING'
        ctx.staged = False
        ctx.entered = False

        ctx.segs = self.build_L_segments(rn, xs, ys, goal_kind='STAGING')
        ctx.seg_i = 0

        self._set_local_state(rn, 'NAV_TO_STAGING', 'initial staging dispatch')
        self.precision_on(rn, False)
        self.resume_one(rn)
        self._send_current_segment(rn, tag='STAGING')

    def dispatch_to_final_one(self, rn: str, micro_dx: float = 0.0, micro_dy: float = 0.0, tag: str = 'FINAL'):
        ctx = self.rt[rn]
        self._reset_runtime_for_new_mission_leg(rn, clear_alignment=True)

        xt, yt, yaw_rad = self.predict_target_world(rn, micro_dx, micro_dy)
        ctx.final_target = (xt, yt)
        ctx.goal_kind = 'FINAL'
        ctx.locked_yaw = yaw_rad

        is_micro = str(tag).upper().startswith('MICRO')
        if is_micro:
            ctx.segs = [(xt, yt)]
        else:
            path_mode = self._path_mode_for_dispatch(rn, 'FINAL', tag=tag)
            ctx.segs = self.build_L_segments(rn, xt, yt, goal_kind='FINAL', path_mode=path_mode)
            if path_mode == 'y_first':
                if len(ctx.segs) >= 2:
                    wx, wy = ctx.segs[0]
                    fx, fy = ctx.segs[-1]
                    self.get_logger().warn(
                        f'[PATH_OVERRIDE] {rn} final approach uses y_first | '
                        f'waypoint=({wx:.3f},{wy:.3f}) final=({fx:.3f},{fy:.3f})'
                    )
                else:
                    fx, fy = ctx.segs[-1]
                    self.get_logger().warn(
                        f'[PATH_OVERRIDE] {rn} final approach uses y_first | '
                        f'waypoint=(direct) final=({fx:.3f},{fy:.3f})'
                    )

        ctx.seg_i = 0
        ctx.xy_stable_count = 0

        self._set_local_state(rn, 'NAV_TO_FINAL', f'dispatch tag={tag}')
        self.precision_on(rn, False)
        self.resume_one(rn)
        self._send_current_segment(rn, tag=tag)

    def dispatch_transport_one(self, rn: str, tag: str = 'TRANSPORT', force_refresh: bool = False):
        ctx = self.rt[rn]
        if ctx.faulted or ctx.finished:
            return False

        if ctx.transport_arrived and force_refresh:
            return False

        if ctx.transport_center_ref is None or (not ctx.loaded_ref_captured):
            self.get_logger().warn(
                f'[TRANSPORT][{rn}] skip dispatch: transport_center_ref not captured in precheck'
            )
            return False

        now = _now_sec(self)
        xt, yt, yaw_rad = self.predict_transport_target_world(rn)
        ctx.transport_target = (xt, yt)
        ctx.goal_kind = 'TRANSPORT'
        ctx.segs = self.build_L_segments(rn, xt, yt, goal_kind='TRANSPORT')
        ctx.seg_i = 0
        ctx.locked_yaw = yaw_rad
        ctx.transport_dispatched = True
        ctx.transporting = True

        if not force_refresh:
            ctx.dwell_start = 0.0
            ctx.dwell_locked = False
            ctx.fine_active = False
            ctx.ready_to_lift = False
            ctx.lifting = False
            ctx.transport_arrived = False
            ctx.transport_settled = False
            ctx.transport_failed = False
            ctx.transport_arrive_stamp = 0.0
            ctx.transport_settle_stamp = 0.0
            ctx.transport_start_stamp = now
            ctx.finished = False
            ctx.formation_error_m = 0.0
            ctx.formation_error_yaw_deg = 0.0
            ctx.group_stop_reason = ''

            self._set_local_state(
                rn,
                'TRANSPORTING',
                f'cooperative transport dispatched; slide_ref={ctx.transport_center_ref}'
            )

        self.precision_on(rn, False)
        self.resume_one(rn)
        self._send_current_segment(rn, tag=tag)
        return True

    def dispatch_transport_all(self, force_refresh: bool = False):
        dispatched = False
        for rn in self.robots:
            dispatched = self.dispatch_transport_one(rn, force_refresh=force_refresh) or dispatched
        return dispatched
