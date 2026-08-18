#!/usr/bin/env python3
"""Pi peripheral daemon — speaker + WS2812 ring only. No ROS, no micro-ROS agent.

ESP sends framed UART commands (115200 8N1). Install:
  bash ~/lego-rover-ros2/install_pi_peripheral.sh
"""
from __future__ import annotations

import os
import sys
import time

import serial

DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, DIR)

from rover_ring import set_mode as ring_set_mode
from rover_ring import sonar_frame
from rover_speaker import play as speaker_play

SYNC0 = 0xA5
SYNC1 = 0x5A

MELODY = {
    1: 'ready',
    2: 'autonomous_start',
    3: 'autonomous_stop',
    4: 'bump',
    5: 'stall',
    6: 'front_ir',
    7: 'button',
    8: 'menu',
}

RING = {
    0: 'standby',
    1: 'ready',
    2: 'auto',
    3: 'escape',
    4: 'bump',
    5: 'stall',
}


def _checksum(cmd: int, arg: int) -> int:
    return cmd ^ arg ^ 0xC3


def _checksum_ext(cmd: int, length: int, payload: bytes) -> int:
    c = cmd ^ length ^ 0xC3
    for b in payload:
        c ^= b
    return c & 0xFF


def _open_serial() -> serial.Serial:
    dev = os.environ.get('ROVER_PI_SERIAL', '/dev/ttyAMA0')
    baud = int(os.environ.get('ROVER_PI_PERIPH_BAUD', '115200'))
    return serial.Serial(dev, baud, timeout=0.05)


def _handle(cmd: int, arg: int) -> None:
    if cmd == 0x10:
        name = MELODY.get(arg)
        if name:
            speaker_play(name)
    elif cmd == 0x11:
        mode = RING.get(arg)
        if mode:
            ring_set_mode(mode)


def _handle_sonar(payload: bytes) -> None:
    n = int(os.environ.get('ROVER_RING_COUNT', '8'))
    if len(payload) < 2:
        return
    dist = list(payload[:-1])
    sweep = int(payload[-1])
    if len(dist) < n:
        dist.extend([255] * (n - len(dist)))
    sonar_frame(dist[:n], sweep % max(1, n))


def run() -> None:
    ring_set_mode('standby')
    ser = _open_serial()
    print(f'pi_peripheral: listening on {ser.port} @ {ser.baudrate}', flush=True)
    buf = bytearray()
    while True:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
        while len(buf) >= 5:
            if buf[0] != SYNC0 or buf[1] != SYNC1:
                del buf[0]
                continue
            cmd = buf[2]
            if cmd == 0x12:
                length = buf[3]
                need = 5 + length
                if len(buf) < need:
                    break
                payload = bytes(buf[4 : 4 + length])
                csum = buf[4 + length]
                if csum != _checksum_ext(cmd, length, payload):
                    del buf[0]
                    continue
                del buf[:need]
                try:
                    _handle_sonar(payload)
                except Exception as exc:
                    print(f'pi_peripheral: sonar error: {exc}', flush=True)
                continue
            arg, csum = buf[3], buf[4]
            if csum != _checksum(cmd, arg):
                del buf[0]
                continue
            del buf[:5]
            try:
                _handle(cmd, arg)
            except Exception as exc:
                print(f'pi_peripheral: handle error: {exc}', flush=True)
        if not chunk:
            time.sleep(0.01)


def main() -> None:
    try:
        run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
