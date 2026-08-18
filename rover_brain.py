#!/usr/bin/env python3
"""LEGO rover explore brain — dockerhost, WiFi micro-ROS.

Tap **Go** on the rover to start/stop. Uses bench-tested explore_controller
(port of autonomous_explore.py): cruise/wander, escape on bump/stall only.
Motor ramp + heading hold stay on the ESP.

Modes (ROVER_MODE env):
  explore / wander — default (same controller)
  hallway          — VL53 wall-follow
"""
from __future__ import annotations

import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, UInt8, UInt32

from explore_controller import ExploreController
from hallway_follow import HallwayConfig, HallwayFollow

BTN_ESTOP_LONG = 3


class RoverBrain(Node):
    TICK = 0.05
    HEARTBEAT_PERIOD = 0.5

    def __init__(self) -> None:
        self._mode = os.environ.get('ROVER_MODE', 'explore').strip().lower()
        super().__init__('rover_brain')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.session_pub = self.create_publisher(Bool, '/rover/session', 10)
        self.heartbeat_pub = self.create_publisher(UInt32, '/rover/heartbeat', 10)

        self._explore: ExploreController | None = None
        self._hallway: HallwayFollow | None = None
        if self._mode == 'hallway':
            self._hallway = HallwayFollow(HallwayConfig.from_env())
            self.get_logger().info('rover_brain: hallway — Go starts wall-follow')
        else:
            self._explore = ExploreController(log=self.get_logger().info)
            self.get_logger().info(
                'rover_brain: explore (autonomous_explore) — Go to start/stop; '
                'ESP ramps + heading hold'
            )

        self._heartbeat_n = 0
        self._session_active = False
        self._last_session_pub: bool | None = None
        self._pan_deg = float(os.environ.get('PAN_CENTER', '90'))

        self.create_subscription(Bool, '/rover/button', self._on_button, 10)
        self.create_subscription(UInt8, '/rover/button_event', self._on_button_event, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(Bool, '/rover/bump', self._on_bump, 10)
        self.create_subscription(Bool, '/rover/stall', self._on_stall, 10)
        if self._hallway is not None:
            self.create_subscription(Range, '/rover/sonar/range', self._on_range, 10)
            self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
            self.create_subscription(Range, '/rover/tof/left', self._on_tof_left, 10)
            self.create_subscription(Range, '/rover/tof/right', self._on_tof_right, 10)

        self.create_timer(self.TICK, self._tick)
        self.create_timer(self.HEARTBEAT_PERIOD, self._send_heartbeat)

    def _publish_stop(self, count: int = 8) -> None:
        stop = Twist()
        for _ in range(count):
            self.pub.publish(stop)

    def _set_session(self, active: bool, reason: str) -> None:
        if active == self._session_active:
            return
        self._session_active = active
        if active:
            if self._explore is not None:
                self._explore.reset()
            if self._hallway is not None:
                self._hallway = HallwayFollow(HallwayConfig.from_env())
            self.get_logger().info(f'session START ({reason})')
        else:
            self.get_logger().info(f'session STOP ({reason})')
            self._publish_stop()
        self._publish_session()

    def _publish_session(self, *, force: bool = False) -> None:
        if not force and self._session_active == self._last_session_pub:
            return
        msg = Bool()
        msg.data = self._session_active
        self.session_pub.publish(msg)
        self._last_session_pub = self._session_active

    def _on_button(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._session_active:
            self._set_session(False, 'Go tap')
        else:
            self._set_session(True, 'Go tap')

    def _on_button_event(self, msg: UInt8) -> None:
        if int(msg.data) == BTN_ESTOP_LONG:
            self._set_session(False, 'E-stop long')

    def _on_range(self, msg: Range) -> None:
        if self._hallway is not None:
            self._hallway.update_forward(float(msg.range), self._pan_deg)

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)

    def _on_imu(self, msg: Imu) -> None:
        if self._explore is not None and self._session_active:
            self._explore.on_imu(float(msg.angular_velocity.z))

    def _on_bump(self, msg: Bool) -> None:
        if msg.data and self._explore is not None and self._session_active:
            self._explore.on_bump()

    def _on_stall(self, msg: Bool) -> None:
        if msg.data and self._explore is not None and self._session_active:
            self._explore.on_stall()

    def _on_tof_left(self, msg: Range) -> None:
        if self._hallway is not None:
            self._hallway.update_left(float(msg.range))

    def _on_tof_right(self, msg: Range) -> None:
        if self._hallway is not None:
            self._hallway.update_right(float(msg.range))

    def _tick(self) -> None:
        msg = Twist()
        if self._session_active:
            if self._hallway is not None:
                self._hallway.fill_twist(msg)
            elif self._explore is not None:
                lin, ang = self._explore.tick()
                msg.linear.x = lin
                msg.angular.z = ang
        self.pub.publish(msg)

    def _send_heartbeat(self) -> None:
        self._heartbeat_n = (self._heartbeat_n + 1) & 0x7FFFFFFF
        driving_bit = 0
        driving = False
        if self._session_active:
            if self._hallway is not None:
                driving = self._hallway._last_driving
            elif self._explore is not None:
                driving = self._explore._last_driving
        if driving:
            driving_bit = 0x80000000
        hb = UInt32()
        hb.data = self._heartbeat_n | driving_bit
        self.heartbeat_pub.publish(hb)
        self._publish_session(force=self._session_active)


def main() -> None:
    rclpy.init()
    node = RoverBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._set_session(False, 'shutdown')
        if node._explore is not None:
            node._explore.shutdown()
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
