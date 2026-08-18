#!/usr/bin/env python3
"""Publish constant forward /cmd_vel — heading-hold tune (no teleop)."""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS
from rover_twist import set_twist


class StraightDrive(Node):
    def __init__(self, speed: float, duration: float):
        super().__init__('straight_drive_tune')
        self.speed = speed
        self.deadline = time.monotonic() + duration
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.create_timer(0.05, self._tick)

    def _tick(self) -> None:
        if time.monotonic() >= self.deadline:
            raise SystemExit(0)
        msg = Twist()
        set_twist(msg, self.speed, 0.0)
        self.pub.publish(msg)


def main() -> int:
    ap = argparse.ArgumentParser(description='Drive straight for heading-hold tune')
    ap.add_argument('-d', '--duration', type=float, default=20.0)
    ap.add_argument('-s', '--speed', type=float, default=0.40)
    args = ap.parse_args()

    rclpy.init()
    node = StraightDrive(args.speed, args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        stop = Twist()
        for _ in range(10):
            node.pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
