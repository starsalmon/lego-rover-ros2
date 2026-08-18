#!/usr/bin/env python3
"""Sweep aux servo + optional IR pulse read at each angle."""
from __future__ import annotations

import argparse
import os
import sys
import time

from rover_ir import close as ir_close, read_front, sample_aux_pulse, set_aux_led
from rover_servo import center, close as servo_close, info, set_angle


def main() -> int:
    ap = argparse.ArgumentParser(description='Test aux head servo sweep')
    ap.add_argument(
        '--sweep',
        action='store_true',
        help='Sweep min→max→center (default if no other mode)',
    )
    ap.add_argument('--scan', action='store_true', help='Sweep + pulsed aux IR at each step')
    ap.add_argument('--steps', type=int, default=9, help='Steps across arc (scan/sweep)')
    ap.add_argument('--center', action='store_true', help='Move to center only')
    ap.add_argument('--min', type=float, default=None, dest='angle_min')
    ap.add_argument('--max', type=float, default=None, dest='angle_max')
    ap.add_argument(
        '--delta',
        action='store_true',
        help='Aux detect only when LED pulse changes reading (sync ranging)',
    )
    ap.add_argument('--on-ms', type=float, default=150.0, help='Aux LED on time during --scan')
    ap.add_argument('--settle', type=float, default=0.4, help='Seconds to wait after each move')
    args = ap.parse_args()

    lo = args.angle_min if args.angle_min is not None else float(os.environ.get('ROVER_SERVO_MIN_ANGLE', '0'))
    hi = args.angle_max if args.angle_max is not None else float(os.environ.get('ROVER_SERVO_MAX_ANGLE', '180'))

    print(info(), flush=True)

    try:
        if args.center:
            print('Center…', flush=True)
            center()
            return 0

        if not args.scan:
            args.sweep = True

        if args.sweep and not args.scan:
            print('Sweep (no IR) — Ctrl+C to stop early', flush=True)
            for deg in _sweep_angles(lo, hi, args.steps):
                print(f'  angle {deg:.0f}°', flush=True)
                set_angle(deg, settle_s=args.settle)
            center()
            return 0

        if args.scan:
            print(
                f'Scan: servo {lo:.0f}°→{hi:.0f}°  steps={args.steps}  '
                f'pulse={args.on_ms:.0f}ms  delta={int(args.delta)}',
                flush=True,
            )
            print('  angle | front | aux_off aux_on delta', flush=True)
            hits: list[tuple[float, bool, bool, bool, int]] = []
            for deg in _scan_angles(lo, hi, args.steps):
                set_angle(deg, settle_s=args.settle)
                front = read_front()
                off_hit, on_hit = sample_aux_pulse(on_ms=args.on_ms / 1000.0)
                delta = int(on_hit) - int(off_hit)
                aux = (on_hit and not off_hit) if args.delta else on_hit
                hits.append((deg, front, off_hit, on_hit, delta))
                print(
                    f'  {deg:5.0f}°  front={int(front)}  '
                    f'aux {int(off_hit)}→{int(on_hit)}  Δ{delta:+d}  '
                    f'{"HIT" if aux else "—"}',
                    flush=True,
                )
            center()
            if args.delta:
                detect_angles = [d for d, _, _, _, delta in hits if delta > 0]
            else:
                detect_angles = [d for d, _, _, on_hit, _ in hits if on_hit]
            if detect_angles:
                print(f'Aux detect at: {", ".join(f"{d:.0f}°" for d in detect_angles)}', flush=True)
            else:
                print('No aux detect — point head at a wall/object and retry', flush=True)
            return 0

        ap.print_help()
        return 1

    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return 0
    finally:
        set_aux_led(False)
        ir_close()
        servo_close()


def _scan_angles(lo: float, hi: float, steps: int) -> list[float]:
    if steps < 2:
        return [lo, hi]
    step = (hi - lo) / (steps - 1)
    return [lo + i * step for i in range(steps)]


def _sweep_angles(lo: float, hi: float, steps: int) -> list[float]:
    forward = _scan_angles(lo, hi, steps)
    return forward + forward[-2:0:-1]


if __name__ == '__main__':
    sys.exit(main())
