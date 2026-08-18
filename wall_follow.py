#!/usr/bin/env python3
"""Wall-follow mode — aux IR locked to one side, front IR backs off head-on."""
from __future__ import annotations

import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_ir import close as ir_close, read_aux_value, read_front, set_aux_led
from rover_qos import CMD_VEL_QOS
from rover_radar import wall_servo_angle
from rover_ring_ipc import emit as ring_emit
from rover_servo import set_angle
from rover_twist import logical_from_wire_angular, set_twist

MIN_LIN_FOR_STEER = 0.11


def _max_linear() -> float:
    return max(0.08, min(1.0, float(os.environ.get('ROVER_WALL_LINEAR', '0.28'))))


def _wall_steer() -> float:
    return max(0.04, min(0.18, float(os.environ.get('ROVER_WALL_STEER', '0.08'))))


def _reverse_linear() -> float:
    raw = os.environ.get('ROVER_WALL_REVERSE_POWER', '').strip()
    if raw:
        return -max(0.08, min(1.0, float(raw)))
    frac = float(os.environ.get('ROVER_AUTO_REVERSE_FRAC', '0.72'))
    cruise = max(0.08, min(1.0, float(os.environ.get('ROVER_AUTO_MAX_LINEAR', '0.30'))))
    return -cruise * max(0.25, min(1.0, frac))


def _wall_side() -> str:
    return os.environ.get('ROVER_WALL_SIDE', 'right').strip().lower()


def _wall_side_label() -> str:
    return 'right' if _wall_side() in ('right', 'r') else 'left'


def _wire_steer(close: bool) -> float:
    """Steer on /cmd_vel wire: + = right, − = left on this rover."""
    s = _wall_steer()
    right = _wall_side() in ('right', 'r')
    if right:
        return -s if close else s
    return s if close else -s


def _arc_not_spin(lin: float, ang: float) -> tuple[float, float]:
    if abs(ang) <= 0.02:
        return lin, ang
    if abs(lin) >= MIN_LIN_FOR_STEER:
        return lin, ang
    creep = MIN_LIN_FOR_STEER if lin >= 0 else -MIN_LIN_FOR_STEER
    if abs(lin) < 0.02:
        creep = MIN_LIN_FOR_STEER
    return creep, ang


class WallFollow(Node):
    TICK = 0.05

    def __init__(self) -> None:
        super().__init__('wall_follow')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        side = _wall_side_label()
        angle = wall_servo_angle(side)
        set_angle(angle, settle_s=0.35)
        set_aux_led(True)
        time.sleep(0.05)
        ring_emit('auto')
        self.get_logger().info(
            f'wall follow: {side} wall  servo={angle:.0f}°  '
            f'speed={_max_linear():.0%}  steer={_wall_steer():.2f}'
        )
        self.create_timer(self.TICK, self._tick)

    def _tick(self) -> None:
        lin = _max_linear()
        wall_close = read_aux_value(settle_ms=0.008)
        ang = logical_from_wire_angular(_wire_steer(wall_close))

        if read_front():
            lin = _reverse_linear()
            # Back away while steering off the front obstacle.
            ang = logical_from_wire_angular(_wire_steer(True))

        lin, ang = _arc_not_spin(lin, ang)
        msg = Twist()
        set_twist(msg, lin, ang)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if type(exc).__name__ != 'ExternalShutdownException':
            raise
    finally:
        try:
            if rclpy.ok():
                stop = Twist()
                for _ in range(8):
                    node.pub.publish(stop)
                    time.sleep(0.05)
        except Exception:
            pass
        set_aux_led(False)
        ir_close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
