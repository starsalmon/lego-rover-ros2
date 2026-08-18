#!/usr/bin/env python3
"""Step through /cmd_vel patterns — find forward / spin / steer on new hardware."""
from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_qos import CMD_VEL_QOS
from rover_twist import set_twist, twist_swap_enabled

# (label, linear.x on wire, angular.z on wire, what you should see if ESP reads fields correctly)
WIRE_TESTS: list[tuple[str, float, float, str]] = [
    (
        'A',
        +0.25,
        0.0,
        'Straight motion on linear.x (sign depends on motor wiring)',
    ),
    (
        'B',
        0.0,
        +0.25,
        'Spin / pivot on angular.z only (not straight)',
    ),
    (
        'C',
        -0.25,
        0.0,
        'Straight REVERSE',
    ),
    (
        'D',
        0.0,
        -0.25,
        'Spin / pivot on angular.z only',
    ),
    (
        'E',
        +0.25,
        +0.15,
        'Forward + gentle LEFT arc (not spin in place)',
    ),
    (
        'F',
        +0.25,
        -0.15,
        'Forward + gentle RIGHT arc',
    ),
    (
        'G',
        0.0,
        +0.35,
        'Spin / pivot LEFT (wheels opposite)',
    ),
    (
        'H',
        0.0,
        -0.35,
        'Spin / pivot RIGHT',
    ),
]


class DriveTest(Node):
    def __init__(self, pub_hz: float) -> None:
        super().__init__('test_drive_dirs')
        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_VEL_QOS)
        self._hz = pub_hz
        self._lin = 0.0
        self._ang = 0.0
        self.create_timer(1.0 / pub_hz, self._tick)

    def set_wire(self, lin: float, ang: float) -> None:
        self._lin = lin
        self._ang = ang

    def set_logical(self, lin: float, ang: float) -> None:
        msg = Twist()
        set_twist(msg, lin, ang)
        self._lin = float(msg.linear.x)
        self._ang = float(msg.angular.z)

    def _tick(self) -> None:
        msg = Twist()
        msg.linear.x = self._lin
        msg.angular.z = self._ang
        self.pub.publish(msg)

    def stop(self) -> None:
        self.set_wire(0.0, 0.0)
        for _ in range(12):
            self._tick()
            time.sleep(1.0 / self._hz)


def _countdown(secs: float) -> None:
    for i in range(int(secs), 0, -1):
        print(f'  …{i}', flush=True)
        time.sleep(1.0)


def _run_burst(node: DriveTest, secs: float, hz: float) -> None:
    end = time.monotonic() + secs
    while time.monotonic() < end:
        node._tick()
        time.sleep(1.0 / hz)


def main() -> int:
    ap = argparse.ArgumentParser(description='Motor direction / cmd_vel diagnostic')
    ap.add_argument('-t', '--duration', type=float, default=2.5, help='Seconds per test')
    ap.add_argument('--prep', type=float, default=2.0, help='Countdown before each move')
    ap.add_argument('--hz', type=float, default=20.0, help='Publish rate')
    ap.add_argument(
        '--only',
        choices=[x[0] for x in WIRE_TESTS] + ['logical'],
        help='Run one test: A–H (wire) or logical (set_twist forward)',
    )
    ap.add_argument(
        '--logical',
        action='store_true',
        help='After wire tests, run logical forward via set_twist (+0.25, 0)',
    )
    args = ap.parse_args()
    run_logical = args.logical or args.only == 'logical'

    lin_sign = os.environ.get('ROVER_LINEAR_SIGN', '1')
    ang_sign = os.environ.get('ROVER_ANGULAR_SIGN', '1')
    print('=== Drive direction test ===')
    print(f'  ROVER_LINEAR_SIGN={lin_sign}  ROVER_ANGULAR_SIGN={ang_sign}  TWIST_SWAP={int(twist_swap_enabled())}')
    print('  Lift rover or clear floor. Ctrl+C to abort.')
    print('  ESP firmware: CMD_VEL_CROSS_FIX=0 (canonical linear.x / angular.z).')
    print()

    rclpy.init()
    node = DriveTest(args.hz)
    tests = [t for t in WIRE_TESTS if args.only is None or t[0] == args.only]

    try:
        for label, lin, ang, expect in tests:
            print(f'--- Test {label}: wire linear.x={lin:+.2f}  angular.z={ang:+.2f} ---')
            print(f'  Expect: {expect}')
            _countdown(args.prep)
            print('  >>> GO <<<', flush=True)
            node.set_wire(lin, ang)
            _run_burst(node, args.duration, args.hz)
            node.stop()
            time.sleep(0.4)
            print()

        if run_logical and (args.only is None or args.only == 'logical'):
            print('--- Logical forward: set_twist(+0.25, 0) ---')
            print('  Expect: your normal “forward” command from autonomous / tune scripts')
            _countdown(args.prep)
            print('  >>> GO <<<', flush=True)
            node.set_logical(0.25, 0.0)
            _run_burst(node, args.duration, args.hz)
            node.stop()
            print(f'  Published wire: linear.x={node._lin:+.3f}  angular.z={node._ang:+.3f}')
            print()

        print('Done. After CMD_VEL_CROSS_FIX=0 on ESP:')
        print('  • Logical forward at end should drive straight (not spin)')
        print('  • A/C = linear motion; B/D/G/H = spin only')
        print('  • Forward but curves  → MOTOR_INVERT_L or MOTOR_INVERT_R on ESP')
        print('  • Steer backwards     → ROVER_ANGULAR_SIGN=-1 on Pi')
        print('  • Fix in software only (no rewire):')
        print('      Pi:  ROVER_LINEAR_SIGN / ROVER_ANGULAR_SIGN in go_test_dirs.sh')
        print('      ESP: MOTOR_INVERT_L/R, MOTOR_SWAP_LR, CMD_VEL_CROSS_FIX in platformio.ini → re-flash')
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
