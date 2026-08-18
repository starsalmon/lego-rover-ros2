#!/usr/bin/env python3
"""Wait until /cmd_vel has a subscriber visible from this machine (ESP via Pi agent)."""
import sys
import time

import rclpy
from rclpy.node import Node


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    node = Node('mac_link_test')
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.get_subscriptions_info_by_topic('/cmd_vel'):
                print('cmd_vel subscriber visible on ROS graph')
                return 0
            time.sleep(0.5)
        print('ERROR: no /cmd_vel subscriber seen from Mac', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
