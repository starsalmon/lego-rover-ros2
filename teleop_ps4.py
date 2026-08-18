#!/usr/bin/env python3
"""PS4 / gamepad teleop -> /cmd_vel with RELIABLE QoS (matches ESP32 micro-ROS).

Linux: /dev/input/js0. macOS: pygame (USB or Bluetooth PS4).
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from joystick_device import JoystickDevice, open_joystick
from rover_qos import CMD_VEL_QOS

DEFAULTS = {
    'axis_linear': 1,
    'axis_angular': 0,
    'scale_linear': 0.45,
    'scale_linear_turbo': 0.7,
    'scale_angular': 1.0,
    'scale_angular_turbo': 1.5,
    'enable_button': 4,        # L1
    'enable_turbo_button': 5,  # R1
    'require_enable_button': True,
    'invert_linear': True,
    'invert_angular': False,
    'deadzone': 0.15,
    'publish_hz': 20.0,
}


def _load_yaml(path: Path) -> dict:
    try:
        import yaml
    except ImportError:
        return {}
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    block = data.get('/**') or data.get('teleop_ps4') or {}
    return block.get('ros__parameters') or block.get('parameters') or {}


def _deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


class TeleopPs4(Node):
    def __init__(self, cfg: dict):
        super().__init__('teleop_ps4')
        self.cfg = {**DEFAULTS, **cfg}
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self._js: JoystickDevice | None = None
        hz = float(self.cfg['publish_hz'])
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f"teleop ready — hold L1 (btn {self.cfg['enable_button']}) + left stick"
        )

    def open_device(self) -> bool:
        opened = open_joystick()
        if not opened:
            self.get_logger().error(
                'No joystick — pair PS4 to Mac (BT) or check /dev/input/js0 (Pi)'
            )
            return False
        self._js, desc = opened
        self.get_logger().info(f'Using joystick {desc}')
        return True

    def _publish(self) -> None:
        if self._js is None:
            return
        self._js.poll()

        c = self.cfg
        msg = Twist()

        enabled = True
        if c['require_enable_button']:
            enabled = self._js.buttons.get(int(c['enable_button']), 0) > 0

        turbo = self._js.buttons.get(int(c['enable_turbo_button']), 0) > 0
        lin_scale = c['scale_linear_turbo'] if turbo else c['scale_linear']
        ang_scale = c['scale_angular_turbo'] if turbo else c['scale_angular']

        if enabled:
            lin = self._js.axes.get(int(c['axis_linear']), 0.0)
            ang = self._js.axes.get(int(c['axis_angular']), 0.0)
            dz = float(c['deadzone'])
            lin = _deadzone(lin, dz)
            ang = _deadzone(ang, dz)
            if c['invert_linear']:
                lin = -lin
            if c['invert_angular']:
                ang = -ang
            msg.linear.x = float(lin * lin_scale)
            msg.angular.z = float(ang * ang_scale)

        self.pub.publish(msg)


def main() -> int:
    cfg_path = Path(
        os.environ.get(
            'TELEOP_CONFIG',
            Path(__file__).resolve().parent / 'teleop_ps4.yaml',
        )
    )
    cfg = _load_yaml(cfg_path)

    rclpy.init()
    node = TeleopPs4(cfg)
    if not node.open_device():
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        time.sleep(0.1)
        if node._js is not None:
            node._js.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
