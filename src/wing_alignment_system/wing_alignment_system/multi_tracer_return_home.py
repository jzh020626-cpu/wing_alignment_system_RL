#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Bool, String
except ImportError:  # pragma: no cover - enables pure-logic unit tests without ROS installed
    rclpy = None
    Node = object  # type: ignore[misc,assignment]
    PoseStamped = Twist = Bool = String = None  # type: ignore[assignment]
    QoSProfile = ReliabilityPolicy = HistoryPolicy = DurabilityPolicy = None  # type: ignore[assignment]

from wing_alignment_system.mission_geometry import extract_mocap_yaw_rad


GO_HOME = "GO_HOME"
AVOIDING = "AVOIDING"
WAIT_ENTRY = "WAIT_ENTRY"
RESUME_HOME = "RESUME_HOME"
DONE = "DONE"
PAUSED_SAFE = "PAUSED_SAFE"

DEFAULT_ROBOT_NAMES: Tuple[str, str, str] = ("tracer1", "tracer2", "tracer3")
DEFAULT_MOCAP_TOPICS: Tuple[str, str, str] = ("/Rigid17/pose", "/Rigid14/pose", "/Rigid15/pose")
DEFAULT_PRIORITY_ORDER: Tuple[str, str, str] = ("tracer1", "tracer2", "tracer3")
DEFAULT_HOME_GOALS_MM: Dict[str, Tuple[float, float, float]] = {
    "tracer1": (14540.0, 3320.0, 180.0),
    "tracer2": (14540.0, 2320.0, 180.0),
    "tracer3": (14540.0, 1320.0, 180.0),
}


@dataclass(frozen=True)
class GoalCommand:
    x: float
    y: float
    yaw_deg: float
    profile_code: float = 0.0


@dataclass(frozen=True)
class PoseState:
    x: float
    y: float
    yaw: float
    stamp_sec: float


@dataclass
class RobotRuntime:
    name: str
    home_goal: GoalCommand
    pose: Optional[PoseState] = None
    mode: str = GO_HOME
    active_goal: Optional[GoalCommand] = None
    avoidance_goal: Optional[GoalCommand] = None
    avoidance_hold_until_sec: float = 0.0
    goal_reached: bool = False
    fault_text: str = ""
    stop_requested: bool = False
    last_goal_sent: Optional[GoalCommand] = None
    last_goal_sent_sec: float = float("-inf")

    def __post_init__(self) -> None:
        if self.active_goal is None:
            self.active_goal = self.home_goal


@dataclass(frozen=True)
class PlannerConfig:
    robot_names: Tuple[str, ...] = DEFAULT_ROBOT_NAMES
    robot_radius_m: float = 0.9
    pair_clearance_m: float = 1.8
    avoidance_lateral_offset_m: float = 1.15
    decision_hold_sec: float = 1.0
    pose_timeout_sec: float = 0.3
    distance_tie_tol_m: float = 0.05
    staging_profile_code: float = 1.0
    home_profile_code: float = 0.0
    waypoint_reached_tol_m: float = 0.10
    done_reached_tol_m: float = 0.12
    queue_x_backoff_m: float = 1.0
    owner_near_home_radius_m: float = 0.0
    priority_order: Tuple[str, ...] = DEFAULT_PRIORITY_ORDER
    home_goals: Dict[str, GoalCommand] = field(default_factory=lambda: build_default_home_goals())


def wrap_angle(a: float) -> float:
    if math.isnan(a) or math.isinf(a):
        return 0.0
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def map_mocap_point_to_world(
    x_mm: float,
    z_mm: float,
    swap_xz: bool = False,
    negate_x: bool = False,
    negate_z: bool = True,
) -> Tuple[float, float]:
    xw = float(x_mm) * 0.001
    yw = float(z_mm) * 0.001
    if swap_xz:
        xw, yw = yw, xw
    if negate_x:
        xw = -xw
    if negate_z:
        yw = -yw
    return round(xw, 6), round(yw, 6)


def build_default_home_goals() -> Dict[str, GoalCommand]:
    goals: Dict[str, GoalCommand] = {}
    for name, (x_mm, y_mm, yaw_deg) in DEFAULT_HOME_GOALS_MM.items():
        xw, yw = map_mocap_point_to_world(x_mm, y_mm)
        goals[name] = GoalCommand(x=xw, y=yw, yaw_deg=float(yaw_deg), profile_code=0.0)
    return goals


def distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(float(ax) - float(bx), float(ay) - float(by))


def distance_to_goal(pose: PoseState, goal: GoalCommand) -> float:
    return distance_xy(pose.x, pose.y, goal.x, goal.y)


def startup_gate_update(
    runtimes: Dict[str, RobotRuntime],
    now_sec: float,
    pose_timeout_sec: float,
    ready_since_sec: Optional[float],
) -> Optional[float]:
    for runtime in runtimes.values():
        if runtime.pose is None:
            return None
        if float(now_sec) - float(runtime.pose.stamp_sec) > float(pose_timeout_sec):
            return None
    return float(now_sec) if ready_since_sec is None else float(ready_since_sec)


def startup_gate_open(ready_since_sec: Optional[float], now_sec: float, hold_sec: float) -> bool:
    if ready_since_sec is None:
        return False
    return float(now_sec) - float(ready_since_sec) >= float(hold_sec)


def goal_changed(previous: Optional[GoalCommand], current: Optional[GoalCommand], pos_tol_m: float, yaw_tol_deg: float) -> bool:
    if previous is None or current is None:
        return True
    return (
        distance_xy(previous.x, previous.y, current.x, current.y) > float(pos_tol_m)
        or abs(wrap_angle(math.radians(current.yaw_deg - previous.yaw_deg))) > math.radians(float(yaw_tol_deg))
        or abs(float(previous.profile_code) - float(current.profile_code)) > 1e-6
    )


def heading_error_to_goal(pose: PoseState, goal_xy: Tuple[float, float]) -> float:
    goal_heading = math.atan2(float(goal_xy[1]) - pose.y, float(goal_xy[0]) - pose.x)
    return abs(wrap_angle(goal_heading - pose.yaw))


def _dot(ax: float, ay: float, bx: float, by: float) -> float:
    return float(ax) * float(bx) + float(ay) * float(by)


def _cross(ax: float, ay: float, bx: float, by: float) -> float:
    return float(ax) * float(by) - float(ay) * float(bx)


def _segment_direction(start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[float, float]:
    dx = float(goal[0]) - float(start[0])
    dy = float(goal[1]) - float(start[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-9:
        return 1.0, 0.0
    return dx / norm, dy / norm


def _closest_point_on_segment(
    point: Tuple[float, float],
    seg_start: Tuple[float, float],
    seg_end: Tuple[float, float],
) -> Tuple[Tuple[float, float], float]:
    sx, sy = float(seg_start[0]), float(seg_start[1])
    ex, ey = float(seg_end[0]), float(seg_end[1])
    px, py = float(point[0]), float(point[1])
    dx = ex - sx
    dy = ey - sy
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return (sx, sy), 0.0
    t = _dot(px - sx, py - sy, dx, dy) / denom
    t = max(0.0, min(1.0, t))
    return (sx + t * dx, sy + t * dy), t


def _orientation(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    return _cross(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]), float(c[0]) - float(a[0]), float(c[1]) - float(a[1]))


def _on_segment(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> bool:
    return (
        min(float(a[0]), float(c[0])) - 1e-9 <= float(b[0]) <= max(float(a[0]), float(c[0])) + 1e-9
        and min(float(a[1]), float(c[1])) - 1e-9 <= float(b[1]) <= max(float(a[1]), float(c[1])) + 1e-9
    )


def _segments_intersect(
    a0: Tuple[float, float],
    a1: Tuple[float, float],
    b0: Tuple[float, float],
    b1: Tuple[float, float],
) -> bool:
    o1 = _orientation(a0, a1, b0)
    o2 = _orientation(a0, a1, b1)
    o3 = _orientation(b0, b1, a0)
    o4 = _orientation(b0, b1, a1)

    if (o1 * o2 < 0.0) and (o3 * o4 < 0.0):
        return True

    return (
        (abs(o1) <= 1e-9 and _on_segment(a0, b0, a1))
        or (abs(o2) <= 1e-9 and _on_segment(a0, b1, a1))
        or (abs(o3) <= 1e-9 and _on_segment(b0, a0, b1))
        or (abs(o4) <= 1e-9 and _on_segment(b0, a1, b1))
    )


def _segment_intersection_point(
    a0: Tuple[float, float],
    a1: Tuple[float, float],
    b0: Tuple[float, float],
    b1: Tuple[float, float],
) -> Optional[Tuple[float, float]]:
    x1, y1 = float(a0[0]), float(a0[1])
    x2, y2 = float(a1[0]), float(a1[1])
    x3, y3 = float(b0[0]), float(b0[1])
    x4, y4 = float(b1[0]), float(b1[1])
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) <= 1e-12:
        return None
    det1 = x1 * y2 - y1 * x2
    det2 = x3 * y4 - y3 * x4
    px = (det1 * (x3 - x4) - (x1 - x2) * det2) / denom
    py = (det1 * (y3 - y4) - (y1 - y2) * det2) / denom
    return px, py


def segment_closest_points(
    a0: Tuple[float, float],
    a1: Tuple[float, float],
    b0: Tuple[float, float],
    b1: Tuple[float, float],
) -> Tuple[float, Tuple[float, float], Tuple[float, float]]:
    if _segments_intersect(a0, a1, b0, b1):
        intersection = _segment_intersection_point(a0, a1, b0, b1)
        if intersection is None:
            intersection = ((float(a0[0]) + float(a1[0]) + float(b0[0]) + float(b1[0])) * 0.25, (float(a0[1]) + float(a1[1]) + float(b0[1]) + float(b1[1])) * 0.25)
        return 0.0, intersection, intersection

    candidates = []
    cp, _ = _closest_point_on_segment(a0, b0, b1)
    candidates.append((distance_xy(a0[0], a0[1], cp[0], cp[1]), a0, cp))
    cp, _ = _closest_point_on_segment(a1, b0, b1)
    candidates.append((distance_xy(a1[0], a1[1], cp[0], cp[1]), a1, cp))
    cp, _ = _closest_point_on_segment(b0, a0, a1)
    candidates.append((distance_xy(b0[0], b0[1], cp[0], cp[1]), cp, b0))
    cp, _ = _closest_point_on_segment(b1, a0, a1)
    candidates.append((distance_xy(b1[0], b1[1], cp[0], cp[1]), cp, b1))
    return min(candidates, key=lambda item: item[0])


def path_conflict(
    start_a: Tuple[float, float],
    goal_a: Tuple[float, float],
    start_b: Tuple[float, float],
    goal_b: Tuple[float, float],
    clearance_m: float,
) -> bool:
    dir_a = _segment_direction(start_a, goal_a)
    dir_b = _segment_direction(start_b, goal_b)
    alignment = _dot(dir_a[0], dir_a[1], dir_b[0], dir_b[1])
    nearly_parallel = abs(_cross(dir_a[0], dir_a[1], dir_b[0], dir_b[1])) <= 0.15 and alignment > 0.85

    min_dist, _, _ = segment_closest_points(start_a, goal_a, start_b, goal_b)
    if min_dist >= float(clearance_m):
        return False
    if _segments_intersect(start_a, goal_a, start_b, goal_b):
        return True
    if nearly_parallel:
        return False
    return True


def choose_yield_robot(
    robot_a: str,
    robot_b: str,
    runtimes: Dict[str, RobotRuntime],
    distance_tie_tol_m: float,
    priority_order: Sequence[str] = DEFAULT_PRIORITY_ORDER,
) -> str:
    runtime_a = runtimes[robot_a]
    runtime_b = runtimes[robot_b]
    if runtime_a.pose is None:
        return robot_a
    if runtime_b.pose is None:
        return robot_b

    dist_a = distance_to_goal(runtime_a.pose, runtime_a.home_goal)
    dist_b = distance_to_goal(runtime_b.pose, runtime_b.home_goal)
    if abs(dist_a - dist_b) > float(distance_tie_tol_m):
        return robot_a if dist_a > dist_b else robot_b

    err_a = heading_error_to_goal(runtime_a.pose, (runtime_a.home_goal.x, runtime_a.home_goal.y))
    err_b = heading_error_to_goal(runtime_b.pose, (runtime_b.home_goal.x, runtime_b.home_goal.y))
    if abs(err_a - err_b) > math.radians(1.0):
        return robot_a if err_a > err_b else robot_b

    priority_index = {name: idx for idx, name in enumerate(priority_order)}
    return robot_a if priority_index.get(robot_a, 999) > priority_index.get(robot_b, 999) else robot_b


def select_avoidance_goal(
    yielder: RobotRuntime,
    yielder_target: Tuple[float, float],
    keeper: RobotRuntime,
    keeper_target: Tuple[float, float],
    clearance_m: float,
    lateral_offset_m: float,
    staging_profile_code: float,
) -> GoalCommand:
    if yielder.pose is None or keeper.pose is None:
        return GoalCommand(
            x=float(yielder.home_goal.x),
            y=float(yielder.home_goal.y),
            yaw_deg=float(yielder.home_goal.yaw_deg),
            profile_code=float(staging_profile_code),
        )

    _, closest_on_yielder, _ = segment_closest_points(
        (yielder.pose.x, yielder.pose.y),
        yielder_target,
        (keeper.pose.x, keeper.pose.y),
        keeper_target,
    )
    dir_x, dir_y = _segment_direction((yielder.pose.x, yielder.pose.y), yielder_target)
    normal_x, normal_y = -dir_y, dir_x
    candidates = [
        (closest_on_yielder[0] + normal_x * float(lateral_offset_m), closest_on_yielder[1] + normal_y * float(lateral_offset_m)),
        (closest_on_yielder[0] - normal_x * float(lateral_offset_m), closest_on_yielder[1] - normal_y * float(lateral_offset_m)),
    ]

    best_candidate = candidates[0]
    best_score = None
    for candidate in candidates:
        keeper_path_point, _ = _closest_point_on_segment(candidate, (keeper.pose.x, keeper.pose.y), keeper_target)
        separation = distance_xy(candidate[0], candidate[1], keeper_path_point[0], keeper_path_point[1])
        cost_to_finish = distance_xy(yielder.pose.x, yielder.pose.y, candidate[0], candidate[1]) + distance_xy(candidate[0], candidate[1], yielder.home_goal.x, yielder.home_goal.y)
        score = (separation, -cost_to_finish)
        if best_score is None or score > best_score:
            best_score = score
            best_candidate = candidate

    yaw_deg = math.degrees(math.atan2(best_candidate[1] - yielder.pose.y, best_candidate[0] - yielder.pose.x))
    return GoalCommand(
        x=float(best_candidate[0]),
        y=float(best_candidate[1]),
        yaw_deg=float(yaw_deg),
        profile_code=float(staging_profile_code),
    )


def update_robot_mode(runtime: RobotRuntime, now_sec: float, waypoint_reached_tol_m: float) -> None:
    if runtime.pose is None or runtime.mode == DONE:
        return
    if runtime.mode == AVOIDING and runtime.avoidance_goal is not None:
        dist_to_waypoint = distance_xy(runtime.pose.x, runtime.pose.y, runtime.avoidance_goal.x, runtime.avoidance_goal.y)
        if dist_to_waypoint <= float(waypoint_reached_tol_m) and float(now_sec) >= float(runtime.avoidance_hold_until_sec):
            runtime.mode = RESUME_HOME
            runtime.avoidance_goal = None
            runtime.active_goal = runtime.home_goal


class MultiTracerReturnHomePlanner:
    def __init__(self, config: Optional[PlannerConfig] = None) -> None:
        self.config = config or PlannerConfig()
        self.global_state = GO_HOME
        self.robots: Dict[str, RobotRuntime] = {
            name: RobotRuntime(name=name, home_goal=self.config.home_goals[name])
            for name in self.config.robot_names
        }
        self.entry_owner: Optional[str] = None

    def update_pose(self, robot_name: str, x: float, y: float, yaw: float, stamp_sec: float) -> None:
        runtime = self.robots[robot_name]
        runtime.pose = PoseState(x=float(x), y=float(y), yaw=float(yaw), stamp_sec=float(stamp_sec))

    def update_goal_reached(self, robot_name: str, reached: bool) -> None:
        self.robots[robot_name].goal_reached = bool(reached)

    def update_fault(self, robot_name: str, fault_text: str) -> None:
        self.robots[robot_name].fault_text = str(fault_text or "").strip()

    def evaluate(self, now_sec: float) -> None:
        for runtime in self.robots.values():
            runtime.stop_requested = False

        if self._should_pause_for_safety(now_sec):
            self.global_state = PAUSED_SAFE
            for runtime in self.robots.values():
                runtime.stop_requested = True
            return

        for runtime in self.robots.values():
            update_robot_mode(runtime, now_sec, self.config.waypoint_reached_tol_m)
            self._mark_done_if_home_reached(runtime)
            if runtime.mode != DONE:
                if runtime.mode == AVOIDING and runtime.avoidance_goal is not None:
                    runtime.active_goal = runtime.avoidance_goal
                else:
                    runtime.active_goal = runtime.home_goal
                    if runtime.mode != RESUME_HOME:
                        runtime.mode = GO_HOME

        had_entry_owner = self.entry_owner is not None
        self._update_entry_owner(require_near_home=not had_entry_owner)
        if self.entry_owner is not None:
            self._apply_entry_queue()
        else:
            assigned_yielders = set()
            active_names = [name for name in self.config.robot_names if self.robots[name].mode != DONE]
            for idx, robot_a in enumerate(active_names):
                for robot_b in active_names[idx + 1:]:
                    runtime_a = self.robots[robot_a]
                    runtime_b = self.robots[robot_b]
                    if runtime_a.pose is None or runtime_b.pose is None or runtime_a.active_goal is None or runtime_b.active_goal is None:
                        continue
                    if not path_conflict(
                        (runtime_a.pose.x, runtime_a.pose.y),
                        (runtime_a.active_goal.x, runtime_a.active_goal.y),
                        (runtime_b.pose.x, runtime_b.pose.y),
                        (runtime_b.active_goal.x, runtime_b.active_goal.y),
                        self.config.pair_clearance_m,
                    ):
                        continue

                    yielder_name = choose_yield_robot(
                        robot_a,
                        robot_b,
                        self.robots,
                        self.config.distance_tie_tol_m,
                        self.config.priority_order,
                    )
                    if yielder_name in assigned_yielders:
                        continue
                    keeper_name = robot_b if yielder_name == robot_a else robot_a
                    yielder = self.robots[yielder_name]
                    keeper = self.robots[keeper_name]
                    if yielder.pose is None or keeper.pose is None or keeper.active_goal is None:
                        continue
                    if yielder.mode == AVOIDING and yielder.avoidance_goal is not None:
                        assigned_yielders.add(yielder_name)
                        continue

                    yielder_target = (yielder.active_goal.x, yielder.active_goal.y) if yielder.active_goal is not None else (yielder.home_goal.x, yielder.home_goal.y)
                    keeper_target = (keeper.active_goal.x, keeper.active_goal.y)
                    yielder.avoidance_goal = select_avoidance_goal(
                        yielder=yielder,
                        yielder_target=yielder_target,
                        keeper=keeper,
                        keeper_target=keeper_target,
                        clearance_m=self.config.pair_clearance_m,
                        lateral_offset_m=self.config.avoidance_lateral_offset_m,
                        staging_profile_code=self.config.staging_profile_code,
                    )
                    yielder.mode = AVOIDING
                    yielder.active_goal = yielder.avoidance_goal
                    yielder.avoidance_hold_until_sec = float(now_sec) + float(self.config.decision_hold_sec)
                    assigned_yielders.add(yielder_name)

        if all(runtime.mode == DONE for runtime in self.robots.values()):
            self.global_state = DONE
        elif any(runtime.mode == AVOIDING for runtime in self.robots.values()):
            self.global_state = AVOIDING
        else:
            self.global_state = GO_HOME

    def _mark_done_if_home_reached(self, runtime: RobotRuntime) -> None:
        if runtime.pose is None or not runtime.goal_reached:
            return
        if distance_to_goal(runtime.pose, runtime.home_goal) <= float(self.config.done_reached_tol_m):
            runtime.mode = DONE
            runtime.active_goal = runtime.home_goal
            runtime.avoidance_goal = None

    def _update_entry_owner(self, require_near_home: bool) -> None:
        if float(self.config.owner_near_home_radius_m) <= 0.0:
            self.entry_owner = None
            return

        if self.entry_owner is not None:
            owner = self.robots.get(self.entry_owner)
            if owner is None or owner.mode == DONE:
                self.entry_owner = None

        if self.entry_owner is not None:
            return

        for robot_name in self.config.priority_order:
            runtime = self.robots.get(robot_name)
            if runtime is None or runtime.mode == DONE or runtime.pose is None:
                continue
            if require_near_home and distance_to_goal(runtime.pose, runtime.home_goal) > float(self.config.owner_near_home_radius_m):
                continue
            self.entry_owner = robot_name
            return

    def _apply_entry_queue(self) -> None:
        for robot_name in self.config.robot_names:
            runtime = self.robots[robot_name]
            if runtime.mode == DONE:
                continue
            runtime.avoidance_goal = None
            if robot_name == self.entry_owner:
                runtime.mode = GO_HOME
                runtime.active_goal = runtime.home_goal
                continue
            runtime.mode = WAIT_ENTRY
            runtime.active_goal = GoalCommand(
                x=float(runtime.home_goal.x) - float(self.config.queue_x_backoff_m),
                y=float(runtime.home_goal.y),
                yaw_deg=float(runtime.home_goal.yaw_deg),
                profile_code=float(self.config.staging_profile_code),
            )

    def _should_pause_for_safety(self, now_sec: float) -> bool:
        for runtime in self.robots.values():
            if runtime.fault_text:
                return True
            if runtime.pose is None:
                return True
            if float(now_sec) - float(runtime.pose.stamp_sec) > float(self.config.pose_timeout_sec):
                return True
        return False


class MultiTracerReturnHomeNode(Node):
    def __init__(self) -> None:
        super().__init__("multi_tracer_return_home")

        robot_names = tuple(str(x) for x in self.declare_parameter("robot_names", list(DEFAULT_ROBOT_NAMES)).value)
        robot_mocap_topics = tuple(str(x) for x in self.declare_parameter("robot_mocap_topics", list(DEFAULT_MOCAP_TOPICS)).value)
        goal_xs = [float(x) for x in self.declare_parameter("home_goal_x_m", [14.54, 14.54, 14.54]).value]
        goal_ys = [float(y) for y in self.declare_parameter("home_goal_y_m", [-3.32, -2.32, -1.32]).value]
        goal_yaws = [float(yaw) for yaw in self.declare_parameter("home_goal_yaw_deg", [180.0, 180.0, 180.0]).value]
        if not (len(robot_names) == len(robot_mocap_topics) == len(goal_xs) == len(goal_ys) == len(goal_yaws)):
            raise ValueError("robot_names, robot_mocap_topics, home_goal_x_m, home_goal_y_m, home_goal_yaw_deg must share the same length")

        self.declare_parameter("robot_radius_m", 0.9)
        self.declare_parameter("pair_clearance_m", 1.8)
        self.declare_parameter("avoidance_lateral_offset_m", 1.15)
        self.declare_parameter("goal_republish_sec", 1.0)
        self.declare_parameter("decision_hold_sec", 1.0)
        self.declare_parameter("pose_timeout_sec", 0.3)
        self.declare_parameter("distance_tie_tol_m", 0.05)
        self.declare_parameter("planner_hz", 10.0)
        self.declare_parameter("staging_profile_code", 1.0)
        self.declare_parameter("queue_x_backoff_m", 1.0)
        self.declare_parameter("owner_near_home_radius_m", 0.0)
        self.declare_parameter("goal_change_tol_m", 0.02)

        self.swap_xz = bool(self.declare_parameter("swap_xz", False).value)
        self.negate_x = bool(self.declare_parameter("negate_x", False).value)
        self.negate_z = bool(self.declare_parameter("negate_z", True).value)
        self.mocap_yaw_mode = str(self.declare_parameter("mocap_yaw_mode", "legacy_deg_y").value).strip().lower()
        self.flip_heading_sign = bool(self.declare_parameter("flip_heading_sign", False).value)
        self.heading_deg_bias = float(self.declare_parameter("heading_deg_bias", 0.0).value)

        self.goal_republish_sec = float(self.get_parameter("goal_republish_sec").value)
        self.goal_change_tol_m = float(self.get_parameter("goal_change_tol_m").value)
        self.planner_hz = max(1.0, float(self.get_parameter("planner_hz").value))

        home_goals = {
            name: GoalCommand(x=goal_xs[idx], y=goal_ys[idx], yaw_deg=goal_yaws[idx], profile_code=0.0)
            for idx, name in enumerate(robot_names)
        }
        config = PlannerConfig(
            robot_names=robot_names,
            robot_radius_m=float(self.get_parameter("robot_radius_m").value),
            pair_clearance_m=float(self.get_parameter("pair_clearance_m").value),
            avoidance_lateral_offset_m=float(self.get_parameter("avoidance_lateral_offset_m").value),
            decision_hold_sec=float(self.get_parameter("decision_hold_sec").value),
            pose_timeout_sec=float(self.get_parameter("pose_timeout_sec").value),
            distance_tie_tol_m=float(self.get_parameter("distance_tie_tol_m").value),
            staging_profile_code=float(self.get_parameter("staging_profile_code").value),
            queue_x_backoff_m=float(self.get_parameter("queue_x_backoff_m").value),
            owner_near_home_radius_m=float(self.get_parameter("owner_near_home_radius_m").value),
            priority_order=robot_names,
            home_goals=home_goals,
        )
        self.planner = MultiTracerReturnHomePlanner(config)
        self._was_paused_safe = False

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_local = QoSProfile(depth=10)

        self.goal_publishers = {}
        self.stop_publishers = {}
        self.resume_publishers = {}
        for idx, robot_name in enumerate(robot_names):
            mocap_topic = robot_mocap_topics[idx]
            self.create_subscription(PoseStamped, mocap_topic, self._make_pose_cb(robot_name), qos_pose)
            self.create_subscription(Bool, f"/{robot_name}/goal_reached", self._make_goal_reached_cb(robot_name), qos_local)
            self.create_subscription(String, f"/{robot_name}/driver_fault", self._make_fault_cb(robot_name), qos_reliable)
            self.goal_publishers[robot_name] = self.create_publisher(Twist, f"/{robot_name}/cmd_goal", qos_reliable)
            self.stop_publishers[robot_name] = self.create_publisher(Bool, f"/{robot_name}/cmd_stop", qos_local)
            self.resume_publishers[robot_name] = self.create_publisher(Bool, f"/{robot_name}/cmd_resume", qos_local)

        self.timer = self.create_timer(1.0 / self.planner_hz, self._tick)

    def _make_pose_cb(self, robot_name: str):
        def cb(msg: PoseStamped) -> None:
            xw, yw = map_mocap_point_to_world(
                msg.pose.position.x,
                msg.pose.position.z,
                swap_xz=self.swap_xz,
                negate_x=self.negate_x,
                negate_z=self.negate_z,
            )
            yaw = extract_mocap_yaw_rad(
                ox=float(msg.pose.orientation.x),
                oy=float(msg.pose.orientation.y),
                oz=float(msg.pose.orientation.z),
                ow=float(msg.pose.orientation.w),
                mode=self.mocap_yaw_mode,
                flip_heading_sign=self.flip_heading_sign,
                heading_deg_bias=self.heading_deg_bias,
            )
            stamp_sec = float(self.get_clock().now().nanoseconds) * 1e-9
            self.planner.update_pose(robot_name, xw, yw, yaw, stamp_sec)

        return cb

    def _make_goal_reached_cb(self, robot_name: str):
        def cb(msg: Bool) -> None:
            self.planner.update_goal_reached(robot_name, bool(msg.data))

        return cb

    def _make_fault_cb(self, robot_name: str):
        def cb(msg: String) -> None:
            self.planner.update_fault(robot_name, str(msg.data))

        return cb

    def _tick(self) -> None:
        now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        self.planner.evaluate(now_sec)

        if self._was_paused_safe and self.planner.global_state != PAUSED_SAFE:
            for robot_name in self.planner.config.robot_names:
                self.resume_publishers[robot_name].publish(Bool(data=True))
                self.planner.robots[robot_name].last_goal_sent = None

        for robot_name, runtime in self.planner.robots.items():
            if runtime.stop_requested:
                self.stop_publishers[robot_name].publish(Bool(data=True))
                continue
            if runtime.mode == DONE or runtime.active_goal is None:
                continue
            if self._should_publish_goal(runtime, now_sec):
                self.goal_publishers[robot_name].publish(self._build_goal_msg(runtime.active_goal))
                runtime.last_goal_sent = runtime.active_goal
                runtime.last_goal_sent_sec = now_sec

        self._was_paused_safe = self.planner.global_state == PAUSED_SAFE

    def _should_publish_goal(self, runtime: RobotRuntime, now_sec: float) -> bool:
        if runtime.last_goal_sent is None:
            return True
        goal = runtime.active_goal
        last = runtime.last_goal_sent
        return goal_changed(last, goal, pos_tol_m=self.goal_change_tol_m, yaw_tol_deg=1.0)

    @staticmethod
    def _build_goal_msg(goal: GoalCommand) -> Twist:
        msg = Twist()
        msg.linear.x = float(goal.x)
        msg.linear.y = float(goal.y)
        msg.linear.z = float(goal.profile_code)
        msg.angular.z = float(goal.yaw_deg)
        return msg


def main(args: Optional[Iterable[str]] = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run multi_tracer_return_home")
    rclpy.init(args=args)
    node = MultiTracerReturnHomeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = [
    "AVOIDING",
    "DONE",
    "GO_HOME",
    "PAUSED_SAFE",
    "RESUME_HOME",
    "WAIT_ENTRY",
    "GoalCommand",
    "PlannerConfig",
    "PoseState",
    "RobotRuntime",
    "MultiTracerReturnHomePlanner",
    "build_default_home_goals",
    "choose_yield_robot",
    "goal_changed",
    "map_mocap_point_to_world",
    "path_conflict",
    "select_avoidance_goal",
    "startup_gate_open",
    "startup_gate_update",
    "update_robot_mode",
]
