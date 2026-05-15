#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
import threading
from typing import List, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from base_interfaces_demo.msg import MotorCommand, MotorStatus
from geometry_msgs.msg import PoseStamped


class MathUtils:
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


class HuataiControlNode(Node):
    def __init__(self) -> None:
        super().__init__('huatai_control_node')

        # --- 状态标志 ---
        self.poses_initialized = False
        self.motors_initialized = False

        # --- 性能与安全配置 ---
        self.POSITION_TOLERANCE = 1.0
        self.ROTATION_TOLERANCE = 0.1
        self.MAX_TRANS_SPEED = 10.0  # mm/s
        self.MAX_ROT_SPEED = 0.5     # deg/s
        self.X_MIN, self.X_MAX = 1.0, 275.0
        self.Y_MIN, self.Y_MAX = 1.0, 275.0
        self.Z_MIN, self.Z_MAX = 1.0, 195.0

        # --- 发布器 ---
        self.publisher1 = self.create_publisher(MotorCommand, '/huatai1_pos_spe_pd', 10)
        self.publisher2 = self.create_publisher(MotorCommand, '/huatai2_pos_spe_pd', 10)
        self.publisher3 = self.create_publisher(MotorCommand, '/huatai3_pos_spe_pd', 10)
        self.motor_publishers = [self.publisher1, self.publisher2, self.publisher3]

        # --- 位姿与滑台状态 ---
        self.current_car_poses: List[np.ndarray] = [np.eye(4) for _ in range(3)]
        self.obj_pose_curr: np.ndarray = np.eye(4)

        self.motor_zeros: List[np.ndarray] = [
            np.array([125.00, 135.00, 1.00], dtype=float),
            np.array([126.01, 140.99, 1.00], dtype=float),
            np.array([126.01, 133.99, 1.00], dtype=float),
        ]

        self.current_local_pts: List[np.ndarray] = [np.zeros(3, dtype=float) for _ in range(3)]
        self.grab_points_in_obj: List[np.ndarray] = [np.zeros(3, dtype=float) for _ in range(3)]
        self.initial_tips_local: List[np.ndarray] = [np.zeros(3, dtype=float) for _ in range(3)]

        self.motor_init_flags = [False, False, False]
        self.received_poses = [False, False, False, False]
        self.theta_unwrapped = [0.0, 0.0, 0.0, 0.0]
        self.prev_raw_theta: List[Optional[float]] = [None, None, None, None]

        self.preset_commands: List[List[float]] = []
        self.pose_subs = []
        self.motor_subs = []

        self.state_lock = threading.Lock()

        self.init_motor_subscriptions()
        self.init_pose_subscriptions()
        self.init_preset_commands()

        self.get_logger().info('=== 协同调姿控制节点启动 (闭环纠偏版 Python) ===')

        # 用 daemon=True，避免 stdin 阻塞导致节点退出时卡死
        self.input_thread = threading.Thread(target=self.read_user_input, daemon=True)
        self.input_thread.start()

    # ------------------------------------------------------------------
    # 运动学辅助
    # ------------------------------------------------------------------
    @staticmethod
    def make_affine(rotation: Optional[np.ndarray] = None,
                    translation: Optional[np.ndarray] = None) -> np.ndarray:
        t = np.eye(4, dtype=float)
        if rotation is not None:
            t[:3, :3] = rotation
        if translation is not None:
            t[:3, 3] = translation
        return t

    @staticmethod
    def affine_inverse(t: np.ndarray) -> np.ndarray:
        inv = np.eye(4, dtype=float)
        r = t[:3, :3]
        p = t[:3, 3]
        inv[:3, :3] = r.T
        inv[:3, 3] = -r.T @ p
        return inv

    @staticmethod
    def transform_point(t: np.ndarray, p: np.ndarray) -> np.ndarray:
        return t[:3, :3] @ p + t[:3, 3]

    @staticmethod
    def euler_to_matrix(r: float, p: float, y: float) -> np.ndarray:
        """等价于 C++ Eigen: Rz(y) * Ry(p) * Rx(r)。输入单位为 rad。"""
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)

        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ], dtype=float)
        ry = np.array([
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ], dtype=float)
        rz = np.array([
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        return rz @ ry @ rx

    @staticmethod
    def get_euler_stable(rotation: np.ndarray) -> np.ndarray:
        """返回 roll, pitch, yaw，单位为 deg。"""
        value = float(np.clip(rotation[2, 0], -1.0, 1.0))
        pitch = -math.asin(value)

        if math.cos(pitch) > 0.001:
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = 0.0
            yaw = math.atan2(-rotation[0, 1], rotation[1, 1])

        return np.array([
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        ], dtype=float)

    def calculate_move_time(self, start: np.ndarray, end: np.ndarray) -> float:
        d_p = float(np.linalg.norm(end[:3, 3] - start[:3, 3]))

        s_e = self.get_euler_stable(start[:3, :3])
        e_e = self.get_euler_stable(end[:3, :3])
        d_r = max(
            abs(MathUtils.normalize_angle(math.radians(e_e[0] - s_e[0]))),
            abs(MathUtils.normalize_angle(math.radians(e_e[1] - s_e[1]))),
            abs(MathUtils.normalize_angle(math.radians(e_e[2] - s_e[2]))),
        )
        d_r = math.degrees(d_r)

        return max(d_p / self.MAX_TRANS_SPEED, d_r / self.MAX_ROT_SPEED, 2.0)

    def process_pose(self, msg: PoseStamped, idx: int) -> np.ndarray:
        """
        保留 C++ 原始坐标转换逻辑：
        - orientation.x/y/z 被当作欧拉角角度值，而不是标准四元数。
        - position 映射为 [x, -z, y]。
        """
        rx_rad = math.radians(msg.pose.orientation.x)
        ry_rad = -math.radians(msg.pose.orientation.z)
        rz_rad = math.radians(msg.pose.orientation.y)

        if self.prev_raw_theta[idx] is None:
            self.theta_unwrapped[idx] = rz_rad
        else:
            self.theta_unwrapped[idx] += MathUtils.normalize_angle(
                rz_rad - self.prev_raw_theta[idx]
            )
        self.prev_raw_theta[idx] = rz_rad

        translation = np.array([
            msg.pose.position.x,
            -msg.pose.position.z,
            msg.pose.position.y,
        ], dtype=float)
        rotation = self.euler_to_matrix(rx_rad, ry_rad, self.theta_unwrapped[idx])
        return self.make_affine(rotation, translation)

    # ------------------------------------------------------------------
    # 预设指令
    # ------------------------------------------------------------------
    def init_preset_commands(self) -> None:
        # 格式：dx, dy, dz, rx, ry, rz, min_time, mode
        # mode = 0：相对当前位姿；mode = 1：绝对位姿
        self.preset_commands = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 80.0, 0.0, 0.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, -4.0, 0.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, -4.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, -4.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 5.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 5.0, 0.0],
            [50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0],
            [0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0],
        ]

    def execute_preset_commands(self) -> None:
        self.get_logger().info(
            f'>>> 开始执行预设指令序列 (共 {len(self.preset_commands)} 条) <<<'
        )
        for i, cmd in enumerate(self.preset_commands):
            if not rclpy.ok():
                break
            self.get_logger().info(
                f'正在执行第 {i + 1}/{len(self.preset_commands)} 条指令...'
            )
            self.execute_tiaozi(
                cmd[0], cmd[1], cmd[2],
                cmd[3], cmd[4], cmd[5],
                cmd[6], bool(cmd[7] != 0.0),
            )
            time.sleep(0.5)
        self.get_logger().info('>>> 预设指令序列执行完毕 <<<')

    # ------------------------------------------------------------------
    # 核心调姿函数
    # ------------------------------------------------------------------
    def execute_tiaozi(self,
                       dx: float, dy: float, dz: float,
                       rx: float, ry: float, rz: float,
                       time_min: float,
                       is_absolute: bool) -> None:
        if not self.poses_initialized or not self.motors_initialized:
            self.get_logger().warn('位姿或滑台状态尚未初始化，忽略本次指令。')
            return

        try:
            with self.state_lock:
                obj_snapshot = self.obj_pose_curr.copy()

            if is_absolute:
                target_translation = np.array([dx, dy, dz], dtype=float)
                target_rotation = self.euler_to_matrix(
                    math.radians(rx), math.radians(ry), math.radians(rz)
                )
            else:
                target_translation = obj_snapshot[:3, 3] + np.array([dx, dy, dz], dtype=float)
                delta_rotation = self.euler_to_matrix(
                    math.radians(rx), math.radians(ry), math.radians(rz)
                )
                target_rotation = delta_rotation @ obj_snapshot[:3, :3]

            t_target = self.make_affine(target_rotation, target_translation)

            accuracy_met = False
            retries = 0

            while rclpy.ok():
                with self.state_lock:
                    obj_now = self.obj_pose_curr.copy()
                    car_poses = [pose.copy() for pose in self.current_car_poses]
                    grab_points = [p.copy() for p in self.grab_points_in_obj]
                    initial_tips = [p.copy() for p in self.initial_tips_local]

                move_time = max(self.calculate_move_time(obj_now, t_target), float(time_min))

                target_m: List[np.ndarray] = []
                for i in range(3):
                    world_tip = self.transform_point(t_target, grab_points[i])
                    p_tip_l = self.transform_point(self.affine_inverse(car_poses[i]), world_tip)
                    motor_target = (p_tip_l - initial_tips[i]) + self.motor_zeros[i]
                    target_m.append(motor_target)

                    if (
                        motor_target[0] < self.X_MIN - 0.1 or motor_target[0] > self.X_MAX + 0.1 or
                        motor_target[1] < self.Y_MIN - 0.1 or motor_target[1] > self.Y_MAX + 0.1 or
                        motor_target[2] < self.Z_MIN - 0.1 or motor_target[2] > self.Z_MAX + 0.1
                    ):
                        self.get_logger().warn(
                            '❌ 行程拦截：滑台 %d 无法运动到 [%.1f, %.1f, %.1f]' %
                            (i + 1, motor_target[0], motor_target[1], motor_target[2])
                        )
                        return

                self.get_logger().info('------------------------------------------------------')
                self.get_logger().info(f'>>> 预计算滑台期望目标读数 (第{retries + 1}次纠偏) <<<')
                for i in range(3):
                    self.get_logger().info(
                        '滑台 %d [自身坐标系]: X:%.2f, Y:%.2f, Z:%.2f' %
                        (i + 1, target_m[i][0], target_m[i][1], target_m[i][2])
                    )
                self.get_logger().info('------------------------------------------------------')
                self.get_logger().info(
                    '指令已发送 (%s模式)，规划耗时 %.2fs...' %
                    ('绝对' if is_absolute else '相对', move_time)
                )

                for i in range(3):
                    msg = MotorCommand()
                    msg.command_type = 'position'
                    msg.x = float(target_m[i][0])
                    msg.y = float(target_m[i][1])
                    msg.z = float(target_m[i][2])
                    msg.time = float(move_time)
                    msg.is_relative = False
                    self.motor_publishers[i].publish(msg)

                self.wait_for_arrival(target_m, move_time + 1.5)

                with self.state_lock:
                    obj_after = self.obj_pose_curr.copy()

                p_e = float(np.linalg.norm(obj_after[:3, 3] - t_target[:3, 3]))
                c_e = self.get_euler_stable(obj_after[:3, :3])
                t_e = self.get_euler_stable(t_target[:3, :3])
                r_e = max(
                    abs(MathUtils.normalize_angle(math.radians(c_e[0] - t_e[0]))),
                    abs(MathUtils.normalize_angle(math.radians(c_e[1] - t_e[1]))),
                    abs(MathUtils.normalize_angle(math.radians(c_e[2] - t_e[2]))),
                )
                r_e = math.degrees(r_e)

                if p_e <= self.POSITION_TOLERANCE and r_e <= self.ROTATION_TOLERANCE:
                    accuracy_met = True
                elif (not is_absolute) or (retries + 1 >= 100):
                    break

                retries += 1
                if (not is_absolute) or accuracy_met:
                    break

            with self.state_lock:
                obj_final = self.obj_pose_curr.copy()
            p = obj_final[:3, 3]
            e = self.get_euler_stable(obj_final[:3, :3])

            self.get_logger().info('------------------------------------------------------')
            self.get_logger().info('>>> 运动完成：物体(Rigid8)当前实际位姿 <<<')
            self.get_logger().info('位置 (Pos): X: %.2f, Y: %.2f, Z: %.2f (mm)' % (p[0], p[1], p[2]))
            self.get_logger().info('姿态 (Deg): Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°' % (e[0], e[1], e[2]))
            self.get_logger().info('------------------------------------------------------')

        except Exception as exc:
            self.get_logger().error(f'异常: {exc}')

    # ------------------------------------------------------------------
    # 初始化检查与回调
    # ------------------------------------------------------------------
    def check_ready_internal(self) -> None:
        """调用该函数时，外层已经持有 self.state_lock。"""
        if all(self.received_poses):
            if (not self.motors_initialized) or self.poses_initialized:
                return

            for i in range(3):
                p_init_w = np.array([
                    self.current_car_poses[i][0, 3],
                    self.current_car_poses[i][1, 3],
                    self.obj_pose_curr[2, 3],
                ], dtype=float)
                self.grab_points_in_obj[i] = self.transform_point(
                    self.affine_inverse(self.obj_pose_curr), p_init_w
                )
                self.initial_tips_local[i] = self.transform_point(
                    self.affine_inverse(self.current_car_poses[i]), p_init_w
                )

            self.poses_initialized = True
            p = self.obj_pose_curr[:3, 3]
            e = self.get_euler_stable(self.obj_pose_curr[:3, :3])

            self.get_logger().info('======================================================')
            self.get_logger().info('===          物体协同控制初始化完成: 抓取点已锁定      ===')
            self.get_logger().info('初始锁定位置: X: %.2f, Y: %.2f, Z: %.2f' % (p[0], p[1], p[2]))
            self.get_logger().info('初始锁定姿态: R: %.2f, P: %.2f, Y: %.2f' % (e[0], e[1], e[2]))
            self.get_logger().info('======================================================')

    def handle_pose_stamped(self, rigid_id: int, msg: PoseStamped) -> None:
        if msg.pose.position.x <= -180.0:
            name = 'Car1' if rigid_id == 0 else 'Car2' if rigid_id == 1 else 'Car3' if rigid_id == 2 else 'Obj'
            self.get_logger().error(
                f'>>> 测量致命错误 <<< 刚体 [{name}] 信号丢失 (-181)。程序强制退出！'
            )
            rclpy.shutdown()
            os._exit(1)

        with self.state_lock:
            if rigid_id < 3:
                self.current_car_poses[rigid_id] = self.process_pose(msg, rigid_id)
            else:
                self.obj_pose_curr = self.process_pose(msg, rigid_id)

            self.received_poses[rigid_id] = True
            self.check_ready_internal()

    def handle_motor_status(self, motor_id: int, msg: MotorStatus) -> None:
        with self.state_lock:
            self.current_local_pts[motor_id] = np.array([msg.x, msg.y, msg.z], dtype=float)

            if not self.motor_init_flags[motor_id]:
                self.motor_init_flags[motor_id] = True
                if all(self.motor_init_flags):
                    self.motors_initialized = True
                    self.check_ready_internal()

    def wait_for_arrival(self, target: List[np.ndarray], timeout: float) -> bool:
        start = time.monotonic()
        last_e = 999.0
        stable_count = 0

        while rclpy.ok():
            with self.state_lock:
                max_error = 0.0
                for i in range(3):
                    err = float(np.linalg.norm(self.current_local_pts[i] - target[i]))
                    max_error = max(max_error, err)

            if max_error <= self.POSITION_TOLERANCE:
                return True

            if abs(last_e - max_error) < 0.005:
                stable_count += 1
            else:
                stable_count = 0
            last_e = max_error

            if stable_count >= 12:
                return True

            if time.monotonic() - start > timeout:
                return False

            time.sleep(0.1)

        return False

    # ------------------------------------------------------------------
    # ROS 通信初始化
    # ------------------------------------------------------------------
    def init_motor_subscriptions(self) -> None:
        for i in range(3):
            topic = f'huatai{i + 1}_pos_spe_p'
            sub = self.create_subscription(
                MotorStatus,
                topic,
                lambda msg, idx=i: self.handle_motor_status(idx, msg),
                10,
            )
            self.motor_subs.append(sub)

    def init_pose_subscriptions(self) -> None:
        topics = ['Rigid17/pose', 'Rigid14/pose', 'Rigid15/pose', 'Rigid8/pose']
        for i, topic in enumerate(topics):
            sub = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, idx=i: self.handle_pose_stamped(idx, msg),
                10,
            )
            self.pose_subs.append(sub)

    # ------------------------------------------------------------------
    # 终端输入线程
    # ------------------------------------------------------------------
    def read_user_input(self) -> None:
        while rclpy.ok():
            if not self.poses_initialized:
                time.sleep(0.5)
                continue

            try:
                user_input = input('\n输入目标(dx dy dz rx ry rz min_t mode) 或 [p]执行预存 或 [q]退出: ').strip()
            except EOFError:
                return
            except KeyboardInterrupt:
                rclpy.shutdown()
                return

            if user_input == 'q':
                rclpy.shutdown()
                return

            if user_input == 'p':
                self.execute_preset_commands()
                continue

            try:
                values = [float(x) for x in user_input.split()]
            except ValueError:
                self.get_logger().warn('输入格式错误，请输入 8 个数字，或输入 p/q。')
                continue

            if len(values) == 8:
                self.execute_tiaozi(
                    values[0], values[1], values[2],
                    values[3], values[4], values[5],
                    values[6], bool(values[7] != 0.0),
                )
            else:
                self.get_logger().warn('输入参数数量错误，应为 8 个：dx dy dz rx ry rz min_t mode。')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HuataiControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
