#!/usr/bin/env python3
"""Client for WS2812 ring — talks to rover_ring_daemon (root systemd service).

Default: GPIO 18, 8 pixels. Disable: ROVER_RING_GPIO=0
Test: python3 rover_ring.py test  (daemon must be running)
"""
from __future__ import annotations

import os
import sys
import time

_CTL = os.environ.get('ROVER_RING_CTL', '/tmp/rover_ring_ctl')


def _gpio() -> int:
    return int(os.environ.get('ROVER_RING_GPIO', '18'))


def _enabled() -> bool:
    return _gpio() > 0


def _send(line: str) -> None:
    if not _enabled():
        return
    try:
        with open(_CTL, 'w', encoding='utf-8') as f:
            f.write(line.strip() + '\n')
    except OSError:
        pass


def set_mode(name: str) -> None:
    """Persistent animation: off | standby | ready | auto | escape."""
    _send(f'mode {name}')


def flash(name: str, seconds: float = 0.7) -> None:
    """Brief overlay: bump | stall | start | stop."""
    _send(f'flash {name} {seconds}')


def radar_frame(hits: list[bool], sweep_idx: int) -> None:
    """Live rear IR radar: red = hit, cyan = sweep beam."""
    bits = ''.join('1' if h else '0' for h in hits)
    _send(f'radar {bits} {sweep_idx}')


def sonar_frame(dist_cm: list[int], sweep_idx: int) -> None:
    """Front sonar: per-LED distance in cm (255 = no reading), cyan = beam."""
    vals = []
    for d in dist_cm:
        vals.append(f'{max(0, min(255, int(d))):03d}')
    _send(f'sonar {"".join(vals)} {sweep_idx}')


def shutdown() -> None:
    set_mode('off')


def main() -> int:
    if len(sys.argv) < 2:
        print('Usage: rover_ring.py test|standby|ready|auto', file=sys.stderr)
        return 1
    cmd = sys.argv[1]
    if cmd == 'test':
        set_mode('auto')
        time.sleep(4)
        flash('bump', 0.8)
        time.sleep(1)
        set_mode('standby')
        time.sleep(2)
        return 0
    set_mode(cmd)
    if cmd not in ('standby', 'ready', 'auto', 'off'):
        return 1
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown()
        return 0


if __name__ == '__main__':
    raise SystemExit(main())
