#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
P3-D0 Shadow Cmd Bridge: Forwards cmd_vel_desired -> cmd_vel_stamped
without publishing real chassis commands. Used only in P3-D0 shadow validation.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TwistStamped


class P3DShadowCmdBridge(Node):
    def __init__(self):
        super().__init__("p3d_shadow_cmd_bridge")
        robots_raw = str(self.declare_parameter("robots", "tracer1,tracer2,tracer3").value)
        self.robots = [r.strip() for r in robots_raw.split(",") if r.strip()]
        qos_in = QoSProfile(depth=10)
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._seq = {rn: 0 for rn in self.robots}
        self._pubs = {}
        for rn in self.robots:
            self._pubs[rn] = self.create_publisher(TwistStamped, f"/{rn}/cmd_vel_stamped", qos_out)
            self.create_subscription(Twist, f"/{rn}/cmd_vel_desired", self._mk_desired_cb(rn), qos_in)

    def _mk_desired_cb(self, rn: str):
        def cb(msg: Twist):
            self._seq[rn] += 1
            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.header.frame_id = (
                f"seq={self._seq[rn]}"
                f"|tx=shadow_bridge"
                f"|exec=normal"
                f"|aoi=0.0"
                f"|eff=1.0"
            )
            ts.twist.linear.x = float(msg.linear.x)
            ts.twist.linear.y = float(msg.linear.y)
            ts.twist.linear.z = float(msg.linear.z)
            ts.twist.angular.x = float(msg.angular.x)
            ts.twist.angular.y = float(msg.angular.y)
            ts.twist.angular.z = float(msg.angular.z)
            self._pubs[rn].publish(ts)
        return cb


def main():
    rclpy.init(args=None)
    node = P3DShadowCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

