#!/usr/bin/env python3
"""Pi-side: UDP cmd_vel relay (port 9999) — Mac teleop sends 'linear angular' packets."""
from __future__ import annotations

import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS
from rover_twist import set_twist

PORT = int(os.environ.get('ROVER_CMD_UDP_PORT', '9999'))
# Stop if no packet (teleop died / disconnected).
STALE_SEC = float(os.environ.get('ROVER_CMD_STALE_SEC', '0.25'))


class CmdVelUdpRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_relay')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self.linear = 0.0
        self.angular = 0.0
        self._lock = threading.Lock()
        self._last_rx = time.monotonic()
        self.create_timer(0.025, self._tick)  # 40 Hz keepalive + stale watchdog
        time.sleep(0.2)

    def _publish(self, lin: float, ang: float) -> None:
        msg = Twist()
        set_twist(msg, lin, ang)
        self.pub.publish(msg)

    def set_cmd(self, linear: float, angular: float) -> None:
        with self._lock:
            self.linear = linear
            self.angular = angular
            self._last_rx = time.monotonic()
            lin, ang = self.linear, self.angular
        self._publish(lin, ang)

    def _tick(self) -> None:
        stale = time.monotonic() - self._last_rx > STALE_SEC
        with self._lock:
            if stale:
                self.linear = 0.0
                self.angular = 0.0
            lin, ang = self.linear, self.angular
        self._publish(lin, ang)
        if os.environ.get('RELAY_DEBUG') and (abs(lin) > 0.01 or abs(ang) > 0.01):
            try:
                with open('/tmp/cmd_vel_udp_last.txt', 'w') as f:
                    f.write(f'{lin:.4f} {ang:.4f}\n')
            except OSError:
                pass


def _udp_loop(node: CmdVelUdpRelay, stop: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', PORT))
    sock.settimeout(0.5)
    if os.environ.get('RELAY_DEBUG'):
        print(f'UDP relay on :{PORT}', flush=True)
    while not stop.is_set():
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        text = data.decode('ascii', errors='ignore').strip()
        parts = text.split()
        if len(parts) < 2:
            continue
        try:
            lin, ang = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        node.set_cmd(lin, ang)
        if os.environ.get('RELAY_DEBUG') and (abs(lin) > 0.01 or abs(ang) > 0.01):
            print(f'from {addr[0]}: {lin:.2f} {ang:.2f}', flush=True)


def main() -> int:
    rclpy.init()
    node = CmdVelUdpRelay()
    stop = threading.Event()
    threading.Thread(target=_udp_loop, args=(node, stop), daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        node.set_cmd(0.0, 0.0)
        for _ in range(8):
            node._tick()
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
