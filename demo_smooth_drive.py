#!/usr/bin/env python3
"""Publish smooth cmd_vel ramps — forward, spin, accel/decel."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS


class SmoothDriveDemo(Node):
    def __init__(self):
        super().__init__('smooth_drive_demo')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.t0 = time.monotonic()
        self.create_timer(0.05, self._tick)  # 20 Hz
        self.get_logger().info('smooth drive demo running')

    def _tick(self):
        t = time.monotonic() - self.t0
        msg = Twist()

        # 0-4s: smooth forward ramp
        # 4-8s: smooth spin
        # 8-12s: forward then smooth stop
        # 12-16s: reverse arc
        # repeat
        phase = t % 16.0

        if phase < 4.0:
            v = self._ramp(phase, 0.0, 4.0, 0.0, 0.35)
            msg.linear.x = v
        elif phase < 8.0:
            w = self._ramp(phase - 4.0, 0.0, 4.0, 0.0, 1.2)
            msg.angular.z = w
        elif phase < 12.0:
            local = phase - 8.0
            if local < 2.0:
                msg.linear.x = self._ramp(local, 0.0, 2.0, 0.0, 0.3)
            else:
                msg.linear.x = self._ramp(local - 2.0, 0.0, 2.0, 0.3, 0.0)
        else:
            local = phase - 12.0
            msg.linear.x = -0.25
            msg.angular.z = 0.5 * math.sin(local * 1.5)

        self.pub.publish(msg)

    @staticmethod
    def _ramp(t, t0, t1, v0, v1):
        if t <= t0:
            return v0
        if t >= t1:
            return v1
        a = (t - t0) / (t1 - t0)
        return v0 + (v1 - v0) * (3 * a * a - 2 * a * a * a)  # smoothstep


def main():
    rclpy.init()
    node = SmoothDriveDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
