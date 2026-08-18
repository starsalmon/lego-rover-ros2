#!/usr/bin/env python3
"""Showcase demo: straight forward 5s, straight back 5s, smooth ramps."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS


class ShowcaseDemo(Node):
    CYCLE = 13.0
    SPEED = 0.60
    RAMP = 0.5
    PAUSE = 0.5

    def __init__(self):
        super().__init__('showcase_demo')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.t0 = time.monotonic()
        self._last_label = ''
        self.create_timer(0.05, self._tick)
        self.get_logger().info('showcase: 5s forward, 5s back (loop)')

    def _label(self, phase):
        if phase < 6.0:
            return 'FORWARD'
        if phase < 6.5:
            return 'pause'
        if phase < 12.5:
            return 'REVERSE'
        return 'stop'

    def _tick(self):
        phase = (time.monotonic() - self.t0) % self.CYCLE
        label = self._label(phase)
        if label != self._last_label:
            self._last_label = label
            self.get_logger().info(label)

        msg = Twist()

        t_fwd_start = 0.0
        t_fwd_cruise = t_fwd_start + self.RAMP
        t_fwd_end = t_fwd_cruise + 5.0
        t_fwd_stop = t_fwd_end + self.RAMP

        t_pause_end = t_fwd_stop + self.PAUSE

        t_rev_start = t_pause_end
        t_rev_cruise = t_rev_start + self.RAMP
        t_rev_end = t_rev_cruise + 5.0
        t_rev_stop = t_rev_end + self.RAMP

        if phase < t_fwd_cruise:
            msg.linear.x = self._lerp(phase, t_fwd_start, t_fwd_cruise, 0.0, self.SPEED)
        elif phase < t_fwd_end:
            msg.linear.x = self.SPEED
        elif phase < t_fwd_stop:
            msg.linear.x = self._lerp(phase, t_fwd_end, t_fwd_stop, self.SPEED, 0.0)
        elif phase < t_rev_cruise:
            msg.linear.x = self._lerp(phase, t_rev_start, t_rev_cruise, 0.0, -self.SPEED)
        elif phase < t_rev_end:
            msg.linear.x = -self.SPEED
        elif phase < t_rev_stop:
            msg.linear.x = self._lerp(phase, t_rev_end, t_rev_stop, -self.SPEED, 0.0)

        self.pub.publish(msg)

    @staticmethod
    def _lerp(t, t0, t1, v0, v1):
        if t <= t0:
            return v0
        if t >= t1:
            return v1
        a = (t - t0) / (t1 - t0)
        s = 3 * a * a - 2 * a * a * a
        return v0 + (v1 - v0) * s


def main():
    rclpy.init()
    node = ShowcaseDemo()
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
