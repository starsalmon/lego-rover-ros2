#!/usr/bin/env python3
"""Sonar radar demo — uses smooth sweep from rover_radar."""
from __future__ import annotations

import argparse
import os
import sys
import time

from rover_ir import close as ir_close, set_aux_led
from rover_ring import set_mode as ring_mode, shutdown as ring_off
from rover_radar import (
    clearest_gap_led,
    iter_sweep_angles,
    scan_hits,
)
from rover_scan_map import led_to_bearing, led_to_servo
from rover_servo import close as servo_close, info
from rover_speaker import play_blocking


def main() -> int:
    ap = argparse.ArgumentParser(description='Sonar radar demo (servo + aux IR + ring)')
    ap.add_argument('--smooth', action='store_true', help='Smooth continuous sweep (slower)')
    ap.add_argument('--stepped', action='store_true', help='Classic step-and-settle (default)')
    ap.add_argument('--hold', type=float, default=4.0, help='Hold final map (s)')
    ap.add_argument('--loops', type=int, default=1, help='Repeat scan (0 = forever)')
    args = ap.parse_args()

    lo = float(os.environ.get('ROVER_SERVO_MIN_ANGLE', '0'))
    hi = float(os.environ.get('ROVER_SERVO_MAX_ANGLE', '180'))
    n_led = int(os.environ.get('ROVER_RING_COUNT', '8'))
    smooth = args.smooth and not args.stepped
    n_steps = len(iter_sweep_angles(lo, hi, smooth=smooth))

    print(info(), flush=True)
    mode = 'smooth' if smooth else 'stepped'
    print(f'Radar demo  {lo:.0f}°→{hi:.0f}°  {mode}  steps≈{n_steps}  LEDs={n_led}', flush=True)

    ring_mode('radar')
    loops = 0

    try:
        while args.loops == 0 or loops < args.loops:
            loops += 1
            if args.loops != 1:
                print(f'\n--- scan {loops} ---', flush=True)

            hits = scan_hits(update_ring=True, ping=True, smooth=smooth)
            gap_led = clearest_gap_led(hits)
            gap_deg = led_to_servo(gap_led, lo, hi, n_led)
            gap_bearing = led_to_bearing(gap_led, n_led)
            print(
                f'\nMap: {_bar(hits)}  clearest ~{gap_bearing:.0f}° '
                f'(servo {gap_deg:.0f}°, LED {gap_led})',
                flush=True,
            )
            play_blocking('ready')
            time.sleep(args.hold)

        return 0

    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return 0
    finally:
        set_aux_led(False)
        ir_close()
        servo_close()
        ring_off()


def _bar(hits: list[bool]) -> str:
    return ''.join('#' if h else '.' for h in hits)


if __name__ == '__main__':
    sys.exit(main())
