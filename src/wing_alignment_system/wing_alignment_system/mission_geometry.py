# -*- coding: utf-8 -*-

import math
from typing import List, Tuple


def wrap_angle_rad(a: float) -> float:
    if math.isnan(a) or math.isinf(a):
        return 0.0
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _now_sec(node) -> float:
    return float(node.get_clock().now().nanoseconds) * 1e-9


def map_mocap_xy(mx_m: float, mz_m: float, swap_xz: bool, negate_x: bool, negate_z: bool) -> Tuple[float, float]:
    xw, yw = mx_m, mz_m
    if swap_xz:
        xw, yw = mz_m, mx_m
    if negate_x:
        xw = -xw
    if negate_z:
        yw = -yw
    return xw, yw


def extract_mocap_yaw_rad(
    ox: float,
    oy: float,
    oz: float,
    ow: float,
    mode: str = 'legacy_deg_y',
    flip_heading_sign: bool = False,
    heading_deg_bias: float = 0.0,
) -> float:
    mode_norm = str(mode or 'legacy_deg_y').strip().lower()

    def _from_legacy_deg_y() -> float:
        yaw_deg = float(oy)
        if flip_heading_sign:
            yaw_deg = -yaw_deg
        yaw_deg += float(heading_deg_bias)
        return wrap_angle_rad(math.radians(yaw_deg))

    def _from_quaternion() -> float:
        qn = math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow)
        if qn <= 1e-9:
            return 0.0
        qx = ox / qn
        qy = oy / qn
        qz = oz / qn
        qw = ow / qn
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return wrap_angle_rad(math.atan2(siny_cosp, cosy_cosp))

    if mode_norm == 'legacy_deg_y':
        return _from_legacy_deg_y()

    if mode_norm == 'quaternion':
        return _from_quaternion()

    qn = math.sqrt(ox * ox + oy * oy + oz * oz + ow * ow)
    looks_like_quat = (
        qn > 1e-6 and
        abs(qn - 1.0) < 0.2 and
        (abs(ow) > 1e-3 or abs(ox) > 1e-3 or abs(oz) > 1e-3)
    )
    if looks_like_quat:
        return _from_quaternion()

    return _from_legacy_deg_y()


def _unique_pts(pts: List[Tuple[float, float]], eps: float = 1e-9) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for x, y in pts:
        if not any(abs(x - a) <= eps and abs(y - b) <= eps for a, b in out):
            out.append((x, y))
    return out


def micro_offsets(mode: str, radii: List[float]) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for r in radii:
        if abs(r) < 1e-12:
            pts.append((0.0, 0.0))
            continue
        pts.extend([(r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)])
    return _unique_pts(pts)
