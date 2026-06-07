from types import SimpleNamespace

import pytest

from wing_alignment_system.mission_dispatcher import MissionDispatcherMixin


class _Logger:
    def warn(self, msg):
        pass


class _Dispatcher(MissionDispatcherMixin):
    def __init__(self, robot):
        self.rt = {
            robot: SimpleNamespace(
                goal_kind="FINAL",
                segs=[(0.0, 0.0), (1.0, 0.0)],
                seg_i=1,
                final_target=(1.0, 0.0),
                locked_yaw=0.0,
                reached=False,
                last_goal_epoch=0.0,
            )
        }
        self.robot_xy = {robot: (0.0, 0.0)}
        self.robot_yaw = {robot: 0.0}
        self.final_precision_enable = True
        self.final_precision_robots = ["tracer1", "tracer2", "tracer3"]
        self.final_precision_window_m = 0.35
        self.raw_qr_accept_radius_m = 0.30
        self.use_goal_yaw = True
        self.sent_goals = []
        self.precision_events = []

    def get_logger(self):
        return _Logger()

    def precision_on(self, rn, on):
        self.precision_events.append((rn, bool(on)))

    def send_goal(self, rn, x, y, yaw_deg, profile_code=0.0):
        self.sent_goals.append((rn, float(x), float(y), float(yaw_deg), float(profile_code)))


@pytest.mark.parametrize("robot", ["tracer1", "tracer2", "tracer3"])
def test_final_precision_deferred_and_activated_for_all_robots(robot):
    node = _Dispatcher(robot)

    node._send_current_segment(robot, tag="FINAL")

    ctx = node.rt[robot]
    assert getattr(ctx, "_final_precision_pending") is True
    assert getattr(ctx, "_final_precision_active") is False
    assert node.precision_events[-1] == (robot, False)

    node.robot_xy[robot] = (0.80, 0.0)
    node._update_final_precision(robot)

    assert getattr(ctx, "_final_precision_active") is True
    assert node.precision_events[-1] == (robot, True)
