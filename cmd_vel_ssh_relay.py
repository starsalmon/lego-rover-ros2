#!/usr/bin/env python3
"""Pi-side: read linear/angular from stdin, publish /cmd_vel at 20 Hz."""
from __future__ import annotations

import os
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS
from rover_twist import set_twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_ssh_relay')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.linear = 0.0
        self.angular = 0.0
        self._lock = threading.Lock()
        self.create_timer(0.05, self._tick)
        time.sleep(1.0)  # DDS discovery before first cmd
        if os.environ.get('RELAY_DEBUG'):
            self.get_logger().info('relay ready')

    def set_cmd(self, linear: float, angular: float) -> None:
        with self._lock:
            self.linear = linear
            self.angular = angular

    def _tick(self) -> None:
        with self._lock:
            lin, ang = self.linear, self.angular
        msg = Twist()
        set_twist(msg, lin, ang)
        self.pub.publish(msg)
        if os.environ.get('RELAY_DEBUG') and (abs(lin) > 0.01 or abs(ang) > 0.01):
            self.get_logger().info(f'pub {lin:.2f} {ang:.2f}')


def _stdin_loop(node: CmdVelRelay) -> None:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            node.set_cmd(float(parts[0]), float(parts[1]))
        except ValueError:
            continue


def main() -> int:
    rclpy.init()
    node = CmdVelRelay()
    threading.Thread(target=_stdin_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_cmd(0.0, 0.0)
        for _ in range(8):
            node._tick()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
