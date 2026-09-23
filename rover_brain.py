#!/usr/bin/env python3
"""LEGO rover explore brain — dockerhost, WiFi micro-ROS.

Tap **Go** on the rover to start/stop. Explore vs wall-follow is selected on the
ESP mode menu (`/rover/drive_mode`). `ROVER_MODE` env is only the startup default
if no drive_mode message has arrived yet.
"""
from __future__ import annotations

import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, UInt8, UInt32

from explore_controller import ExploreController
from hallway_follow import HallwayConfig, HallwayFollow
from rover_qos import CMD_VEL_QOS
from cmd_vel_slew import CmdVelSlew

BTN_ESTOP_LONG = 3
DRIVE_EXPLORE = 0
DRIVE_WALL = 1


class RoverBrain(Node):
    TICK = 0.05
    HEARTBEAT_PERIOD = 0.5

    def __init__(self) -> None:
        env_mode = os.environ.get('ROVER_MODE', 'explore').strip().lower()
        default_drive = DRIVE_WALL if env_mode == 'hallway' else DRIVE_EXPLORE
        super().__init__('rover_brain')

        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.session_pub = self.create_publisher(Bool, '/rover/session', 10)
        self.heartbeat_pub = self.create_publisher(UInt32, '/rover/heartbeat', 10)
        self.sonar_cal_pub = self.create_publisher(Bool, '/rover/sonar/cal_sweep', 10)

        self._explore = ExploreController(log=self.get_logger().info)
        self._hallway: HallwayFollow | None = None
        self._drive_mode = default_drive
        self._drive_mode_from_esp = False

        self._heartbeat_n = 0
        self._session_active = False
        self._last_session_pub: bool | None = None
        self._pan_deg = float(os.environ.get('PAN_CENTER', '90'))
        self._cal_last: bool | None = None
        self._sonar_avoid_active = False
        self._sonar_avoid_since: float | None = None
        self._brake_scan_until = 0.0
        self._brake_scan_cooldown_until = 0.0
        self._slew = CmdVelSlew.from_env()
        self._slew_t = time.monotonic()
        self._front_c = float('nan')
        self._front_l = float('nan')
        self._front_r = float('nan')

        self._allow_drive = True
        self.create_subscription(Bool, '/fleet/allow_drive', self._on_allow_drive, 10)

        self.create_subscription(Bool, '/rover/button', self._on_button, 10)
        self.create_subscription(UInt8, '/rover/button_event', self._on_button_event, 10)
        self.create_subscription(UInt8, '/rover/drive_mode', self._on_drive_mode, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(Bool, '/rover/bump', self._on_bump, 10)
        self.create_subscription(Bool, '/rover/stall', self._on_stall, 10)
        self.create_subscription(Range, '/rover/sonar/range', self._on_range, 10)
        self.create_subscription(Float32, '/rover/sonar/pan_deg', self._on_pan, 10)
        self.create_subscription(Bool, '/rover/sonar/avoid_active', self._on_sonar_avoid, 10)
        self.create_subscription(Range, '/rover/tof/left', self._on_tof_left, 10)
        self.create_subscription(Range, '/rover/tof/right', self._on_tof_right, 10)
        self.create_subscription(Range, '/rover/tof/front', self._on_tof_front, 10)
        self.create_subscription(Range, '/rover/tof/front_left', self._on_tof_front_left, 10)
        self.create_subscription(Range, '/rover/tof/front_right', self._on_tof_front_right, 10)
        self.create_subscription(UInt8, '/rover/ir/hits', self._on_ir_hits, 10)

        self.create_timer(self.TICK, self._tick)
        self.create_timer(self.HEARTBEAT_PERIOD, self._send_heartbeat)
        self._log_drive_mode('startup default' if env_mode == 'hallway' else 'startup')

    def _wall_follow(self) -> bool:
        return self._drive_mode == DRIVE_WALL

    def _ensure_hallway(self) -> HallwayFollow:
        if self._hallway is None:
            self._hallway = HallwayFollow(HallwayConfig.from_env())
        return self._hallway

    def _log_drive_mode(self, reason: str) -> None:
        label = 'wall follow' if self._wall_follow() else 'explore'
        self.get_logger().info(f'rover_brain: {label} ({reason})')

    def _set_drive_mode(self, mode: int, *, reason: str) -> None:
        mode = DRIVE_WALL if mode == DRIVE_WALL else DRIVE_EXPLORE
        if mode == self._drive_mode:
            return
        if self._session_active:
            self._set_session(False, f'{reason} (mode change)')
        self._drive_mode = mode
        if self._wall_follow():
            self._hallway = HallwayFollow(HallwayConfig.from_env())
        self._log_drive_mode(reason)

    def _on_drive_mode(self, msg: UInt8) -> None:
        self._drive_mode_from_esp = True
        self._set_drive_mode(int(msg.data), reason='ESP menu')

    def _publish_stop(self, count: int = 8) -> None:
        stop = Twist()
        for _ in range(count):
            self.pub.publish(stop)

    def _set_session(self, active: bool, reason: str) -> None:
        if active == self._session_active:
            return
        self._session_active = active
        if active:
            if self._wall_follow():
                self._hallway = HallwayFollow(HallwayConfig.from_env())
            else:
                self._explore.reset()
            self._slew.reset()
            self._slew_t = time.monotonic()
            self.get_logger().info(f'session START ({reason}) — {"wall" if self._wall_follow() else "explore"}')
        else:
            self.get_logger().info(f'session STOP ({reason})')
            try:
                self._publish_stop()
            except Exception:
                pass
        self._publish_session()

    def _publish_session(self, *, force: bool = False) -> None:
        if not force and self._session_active == self._last_session_pub:
            return
        msg = Bool()
        msg.data = self._session_active
        self.session_pub.publish(msg)
        self._last_session_pub = self._session_active

    def _on_allow_drive(self, msg: Bool) -> None:
        allow = bool(msg.data)
        if allow == self._allow_drive:
            return
        self._allow_drive = allow
        if not allow and self._session_active:
            self._set_session(False, 'HA stop all')
        self.get_logger().info(f'allow_drive={allow}')

    def _on_button(self, msg: Bool) -> None:
        if not msg.data:
            return
        if not self._allow_drive:
            self.get_logger().info('Go ignored — wall panel halt (Allow drive is off)')
            return
        if self._session_active:
            self._set_session(False, 'Go tap')
        else:
            self._set_session(True, 'Go tap')

    def _on_button_event(self, msg: UInt8) -> None:
        if int(msg.data) == BTN_ESTOP_LONG:
            self._set_session(False, 'E-stop long')

    def _on_range(self, msg: Range) -> None:
        rng = float(msg.range)
        if self._wall_follow():
            return
        if self._session_active:
            self._explore.on_sonar_range(rng)

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        if not self._wall_follow() and self._session_active:
            self._explore.on_pan(self._pan_deg)

    def _on_sonar_avoid(self, msg: Bool) -> None:
        active = bool(msg.data)
        now = time.monotonic()
        if active and not self._sonar_avoid_active:
            self._sonar_avoid_since = now
        if not active:
            self._sonar_avoid_since = None
        self._sonar_avoid_active = active
        if not self._wall_follow() and self._session_active:
            self._explore.on_sonar_avoid(active)

    def _on_imu(self, msg: Imu) -> None:
        if not self._wall_follow() and self._session_active:
            self._explore.on_imu(float(msg.angular_velocity.z))

    def _on_bump(self, msg: Bool) -> None:
        if msg.data and not self._wall_follow() and self._session_active:
            self._explore.on_bump()

    def _on_stall(self, msg: Bool) -> None:
        if msg.data and not self._wall_follow() and self._session_active:
            self._explore.on_stall()

    def _on_tof_left(self, msg: Range) -> None:
        rng = float(msg.range)
        if self._wall_follow():
            self._ensure_hallway().update_left(rng)
        elif self._session_active:
            self._explore.on_tof_left(rng)

    def _on_tof_right(self, msg: Range) -> None:
        rng = float(msg.range)
        if self._wall_follow():
            self._ensure_hallway().update_right(rng)
        elif self._session_active:
            self._explore.on_tof_right(rng)

    def _push_front_tof(self) -> None:
        if self._wall_follow():
            vals = [
                v for v in (self._front_c, self._front_l, self._front_r)
                if math.isfinite(v) and v > 0.02
            ]
            if vals:
                self._ensure_hallway().update_front(min(vals))
            return
        if self._session_active:
            self._explore.on_tof_front(self._front_c, self._front_l, self._front_r)

    def _on_tof_front(self, msg: Range) -> None:
        self._front_c = float(msg.range)
        self._push_front_tof()

    def _on_tof_front_left(self, msg: Range) -> None:
        self._front_l = float(msg.range)
        self._push_front_tof()

    def _on_tof_front_right(self, msg: Range) -> None:
        self._front_r = float(msg.range)
        self._push_front_tof()

    def _on_ir_hits(self, msg: UInt8) -> None:
        bits = int(msg.data)
        if self._wall_follow():
            self._ensure_hallway().update_ir_hits(bits)
        elif self._session_active:
            self._explore.on_ir_hits(bits)

    def _tick(self) -> None:
        msg = Twist()
        if self._session_active:
            if self._wall_follow():
                self._ensure_hallway().fill_twist(msg)
            else:
                lin, ang = self._explore.tick()
                msg.linear.x = lin
                msg.angular.z = ang
        now = time.monotonic()
        dt = now - self._slew_t
        self._slew_t = now
        if dt <= 0.0 or dt > 0.2:
            dt = self.TICK
        lin, ang = self._slew.step(
            float(msg.linear.x),
            float(msg.angular.z),
            dt,
            hard_stop=not self._session_active,
        )
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)
        self._publish_cal_sweep()

    def _publish_cal_sweep(self) -> None:
        now = time.monotonic()
        want = False
        if self._session_active:
            # Primary: explore controller requests scan windows.
            if not self._wall_follow():
                want = self._explore.wants_cal_sweep(now)

            # Rear sonar avoid must not trigger a sit-and-scan. The tail is
            # not a pathfinder; L8 peels the nose.
        if want == self._cal_last:
            return
        self._cal_last = want
        m = Bool()
        m.data = bool(want)
        self.sonar_cal_pub.publish(m)

    def _send_heartbeat(self) -> None:
        self._heartbeat_n = (self._heartbeat_n + 1) & 0x7FFFFFFF
        driving_bit = 0
        driving = False
        if self._session_active:
            if self._wall_follow():
                driving = self._ensure_hallway()._last_driving
            else:
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
        node._explore.shutdown()
        try:
            node._publish_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
