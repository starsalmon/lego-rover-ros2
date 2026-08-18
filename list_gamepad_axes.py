#!/usr/bin/env python3
"""Print PS4 axis values — wiggle sticks / squeeze triggers to see axis numbers."""
from __future__ import annotations

import sys
import time

from joystick_device import open_joystick

HELP = """
Move ONLY the left stick left/right — note which axis changes (should be steer).
Squeeze L2 / R2 — note which axes go from -1 toward +1 (should be drive).
Ctrl+C to quit.
"""


def main() -> int:
    opened = open_joystick()
    if not opened:
        print('No gamepad found.', file=sys.stderr)
        return 1
    js, desc = opened
    print(f'Gamepad: {desc}')
    print(HELP)
    try:
        while True:
            js.poll()
            n = max(6, max(js.axes.keys(), default=0) + 1)
            parts = [f'{i}:{js.axes.get(i, 0):+.2f}' for i in range(n)]
            print('  ' + '  '.join(parts), flush=True)
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    js.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
