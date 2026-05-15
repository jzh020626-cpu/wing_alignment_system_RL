# -*- coding: utf-8 -*-

import math
from std_msgs.msg import Bool


class MissionGateManagerMixin:
    def apply_collision_gate(self):
        if not self.gate_enable or self.emergency or self.state != 'RUN_ALIGNMENT':
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        pos = {rn: self.robot_xy.get(rn) for rn in self.robots}
        valid = [rn for rn in self.robots if pos.get(rn) is not None and (not self.rt[rn].faulted) and (not self.rt[rn].finished) and (not self.rt[rn].confirmed)]
        if len(valid) < 2:
            return

        leader = self._leader_robot()
        for rn in valid:
            ctx = self.rt[rn]
            if not ctx.gate_stopped or now < ctx.gate_hold_until:
                continue
            if ctx.dwell_start > 0.0 or ctx.fine_active or ctx.first_qr_locked or (ctx.staged and not ctx.entered):
                continue
            xr, yr = pos[rn]
            near = self._near_wing(rn) or (self._robot_stage(rn) in ('WAIT_QR', 'SLIDE_ALIGNING', 'STAGED', 'WAIT_ALL_QR_LOCK'))
            dresume = self.gate_dresume_near if near else self.gate_dresume_far
            ok_far = True
            for other in valid:
                if other == rn:
                    continue
                xo, yo = pos[other]
                if math.hypot(xr - xo, yr - yo) < dresume:
                    ok_far = False
                    break
            if ok_far:
                ctx.gate_stopped = False
                self.resume_one(rn)

        not_stopped = [rn for rn in valid if not self.rt[rn].gate_stopped]
        if self.gate_keep_one_moving and len(not_stopped) <= 1:
            return

        pairs = []
        for i in range(len(valid)):
            for j in range(i + 1, len(valid)):
                a, b = valid[i], valid[j]
                xa, ya = pos[a]
                xb, yb = pos[b]
                d = math.hypot(xa - xb, ya - yb)
                pairs.append((d, a, b))
        pairs.sort(key=lambda x: x[0])

        for d, a, b in pairs:
            ca, cb = self.rt[a], self.rt[b]
            if ca.gate_stopped or cb.gate_stopped:
                continue
            stage_a, stage_b = self._robot_stage(a), self._robot_stage(b)
            near_pair = self._near_wing(a) or self._near_wing(b) or (stage_a in ('WAIT_QR', 'SLIDE_ALIGNING', 'STAGED', 'WAIT_ALL_QR_LOCK')) or (stage_b in ('WAIT_QR', 'SLIDE_ALIGNING', 'STAGED', 'WAIT_ALL_QR_LOCK'))
            dmin = self.gate_dmin_near if near_pair else self.gate_dmin_far
            if d >= dmin:
                break

            def immobile(rname: str) -> bool:
                c = self.rt[rname]
                return c.staged or c.fine_active or c.first_qr_locked or c.dwell_start > 0.0 or c.ready_to_lift or c.faulted or c.finished

            imm_a = immobile(a)
            imm_b = immobile(b)
            if imm_a and not imm_b:
                to_stop = b
            elif imm_b and not imm_a:
                to_stop = a
            else:
                to_stop = b if self.dispatch_order.index(b) > self.dispatch_order.index(a) else a

            if self.gate_keep_one_moving:
                not_stopped_now = [r for r in valid if not self.rt[r].gate_stopped]
                if len(not_stopped_now) <= 1:
                    return
                if len(not_stopped_now) == 2 and to_stop in not_stopped_now:
                    other = not_stopped_now[0] if not_stopped_now[1] == to_stop else not_stopped_now[1]
                    if other != leader and to_stop == leader:
                        to_stop = other

            self.rt[to_stop].gate_stopped = True
            self.rt[to_stop].gate_hold_until = now + self.gate_hold_sec
            self.stop_pub[to_stop].publish(Bool(data=True))
            self.stop_slide_comp(to_stop)
            self.get_logger().warn(f'[COLLISION_GATE] {to_stop} is stopped to preserve safe spacing (pair distance: {d:.2f} m)')
            return
