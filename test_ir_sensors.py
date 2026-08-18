#!/usr/bin/env python3
"""Bench test — Front + Aux IR (pulsed emitters on GPIO 6 / 26)."""
from __future__ import annotations

import argparse
import os
import sys
import time

import rover_ir
from rover_ir import (
    close,
    front_pulse_state,
    pins,
    read_both,
    read_gpio_raw,
    sample_aux_pulse,
    sample_front_pulse,
    set_aux_led,
    use_esp_ir,
)

try:
    from rover_esp_ir import ensure_bridge_started, ir_source
except ImportError:
    def ensure_bridge_started() -> None:
        return

    def ir_source() -> str:
        return 'pi'


def main() -> int:
    ap = argparse.ArgumentParser(description='Test IR proximity sensors')
    ap.add_argument('-w', '--watch', action='store_true', help='Stream state (Ctrl+C to stop)')
    ap.add_argument('-t', '--seconds', type=float, default=15.0, help='Watch duration')
    ap.add_argument('--pulse', action='store_true', help='Print aux pulse off/on/delta samples')
    ap.add_argument('--pulse-front', action='store_true', help='Print front pulse off/on/delta samples')
    ap.add_argument('--led-on', action='store_true', help='Hold aux LED on while testing')
    ap.add_argument('--on-ms', type=float, default=100.0, help='Aux LED on time during --pulse')
    ap.add_argument('--raw', action='store_true', help='Print 0/1 not DETECT/clear')
    ap.add_argument(
        '--pull',
        choices=('down', 'up', 'float'),
        default=None,
        help='GPIO input pull (default float)',
    )
    ap.add_argument('--lgpio', action='store_true', help='Also print direct lgpio pin reads')
    ap.add_argument('--active-low', action='store_true', help='Treat OUT LOW as detect')
    args = ap.parse_args()

    if args.pull:
        os.environ['ROVER_IR_PULL'] = args.pull
    if args.active_low:
        os.environ['ROVER_IR_ACTIVE_LOW'] = '1'

    esp = use_esp_ir()
    if esp:
        ensure_bridge_started()

    front_pin, aux_pin, aux_led_pin, front_led_pin = pins()
    active = 'LOW' if os.environ.get('ROVER_IR_ACTIVE_LOW') == '1' else 'HIGH'
    print(f'IR source: {ir_source()} ({"ESP MCP via ROS" if esp else "Pi GPIO"})', flush=True)
    if esp:
        print('  front/aux IR on ESP MCP23008 — Pi GPIO pins below are unused', flush=True)
    print(
        f'IR test  front_in=GPIO{front_pin}  front_led=GPIO{front_led_pin}',
        flush=True,
    )
    print(
        f'         aux_in=GPIO{aux_pin}  aux_led=GPIO{aux_led_pin}',
        flush=True,
    )
    print(
        f'  pull={os.environ.get("ROVER_IR_PULL", "float")}  detect_when_OUT={active}',
        flush=True,
    )
    print(
        f'  front pulse: {os.environ.get("ROVER_IR_FRONT_PULSE", "1")} '
        f'off={os.environ.get("ROVER_IR_FRONT_OFF_MS", "50")}ms '
        f'on={os.environ.get("ROVER_IR_FRONT_ON_MS", "10")}ms (background)',
        flush=True,
    )
    print('', flush=True)

    def fmt(front: bool, aux: bool, *, f_raw: int | None = None, a_raw: int | None = None) -> str:
        off_f, on_f, delta_f = front_pulse_state()
        extra = f'  front off={int(off_f)} on={int(on_f)} delta={int(delta_f)}'
        if args.lgpio and f_raw is not None and a_raw is not None:
            base = f'gz front={int(front)} aux={int(aux)}  lgpio f={f_raw} a={a_raw}{extra}'
            if args.raw:
                return base
            return base + (
                f'  ({ "DETECT" if front else "clear" } / { "DETECT" if aux else "clear" })'
            )
        if args.raw:
            return f'front={int(front)} aux={int(aux)}{extra}'
        return (
            f'front={"DETECT" if front else "clear":6s}  '
            f'aux={"DETECT" if aux else "clear":6s}{extra}'
        )

    def sample(*, hold_led: bool) -> str:
        f, a = read_both(pulse_aux=not hold_led, aux_led_hold=hold_led)
        if args.lgpio and not esp:
            return fmt(f, a, f_raw=read_gpio_raw(front_pin), a_raw=read_gpio_raw(aux_pin))
        return fmt(f, a)

    try:
        if args.led_on:
            set_aux_led(True)
            print('Aux LED held ON', flush=True)
            time.sleep(0.05)

        if args.watch or (args.led_on and not args.pulse and not args.pulse_front):
            deadline = time.monotonic() + (args.seconds if args.watch else 1e9)
            print('Watching… (Ctrl+C to stop)', flush=True)
            while time.monotonic() < deadline:
                print(f'  {sample(hold_led=args.led_on)}', flush=True)
                time.sleep(0.08)
            return 0

        if args.pulse_front:
            print('Front pulse samples (hand in beam during ON):', flush=True)
            for i in range(12):
                off_f, on_f = sample_front_pulse()
                print(
                    f'  [{i + 1:2d}]  front off={int(off_f)} on={int(on_f)}  '
                    f'delta={int(on_f) - int(off_f):+d}',
                    flush=True,
                )
                time.sleep(0.12)
            return 0

        if args.pulse:
            print(f'Aux pulse ({args.on_ms:.0f} ms on):', flush=True)
            for i in range(10):
                off_a, on_a = sample_aux_pulse(on_ms=args.on_ms / 1000.0)
                print(
                    f'  [{i + 1:2d}]  aux off={int(off_a)} on={int(on_a)}  '
                    f'delta={int(on_a) - int(off_a):+d}',
                    flush=True,
                )
                time.sleep(0.25)
            return 0

        print(sample(hold_led=False))
        print('Try:  -w  |  --pulse-front  |  --pulse  |  --lgpio', flush=True)
        return 0

    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return 0
    finally:
        set_aux_led(False)
        close()


if __name__ == '__main__':
    sys.exit(main())
