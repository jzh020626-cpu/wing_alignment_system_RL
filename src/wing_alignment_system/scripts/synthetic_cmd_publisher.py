#!/usr/bin/env python3
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import TwistStamped

class SyntheticPublisher(Node):
    def __init__(self, robot, v, w, rate, nonzero_sec):
        super().__init__(f"synthetic_cmd_{robot}")
        self.pub = self.create_publisher(TwistStamped, f"/{robot}/cmd_vel_stamped", 10)
        self.v = float(v)
        self.w = float(w)
        self.rate_hz = float(rate)
        self.nonzero_sec = float(nonzero_sec)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

    def tick(self):
        if not rclpy.ok():
            return
        try:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed < self.nonzero_sec:
                vx = self.v
                wz = self.w
            else:
                vx = 0.0
                wz = 0.0
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "seq=999|tx=full|exec=degraded|aoi=0.0|eff=1.0|phase=synthetic_degraded"
            msg.twist.linear.x = vx
            msg.twist.angular.z = wz
            self.pub.publish(msg)
        except RCLError:
            pass
        except Exception:
            rclpy.shutdown()

def main():
    rclpy.init(args=sys.argv)
    robot = sys.argv[1] if len(sys.argv) > 1 else "tracer1"
    v = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03
    w = float(sys.argv[3]) if len(sys.argv) > 3 else 0.06
    rate = float(sys.argv[4]) if len(sys.argv) > 4 else 100
    nonzero_sec = float(sys.argv[5]) if len(sys.argv) > 5 else 24.0
    node = SyntheticPublisher(robot, v, w, rate, nonzero_sec)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
