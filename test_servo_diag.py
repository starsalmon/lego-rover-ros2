#!/usr/bin/env python3
"""PCA9685 + servo hardware diagnostic — holds pulses, wiggle, channel scan."""
from __future__ import annotations

import argparse
import os
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser(description='PCA9685 servo diagnostic')
    ap.add_argument('--channel', type=int, default=None, help='PWM channel (default: env or 0)')
    ap.add_argument('--pulse-us', type=int, default=1500, help='Pulse width to hold (with --hold)')
    ap.add_argument('--hold', type=float, default=5.0, help='Seconds to hold each test')
    ap.add_argument('--oe-gpio', type=int, default=None, help='OE GPIO (omit if OE→GND)')
    ap.add_argument('--oe-invert', action='store_true', help='OE active HIGH (rare boards)')
    ap.add_argument('--scan-channels', action='store_true', help='Try channels 0-3')
    ap.add_argument('--all-channels', action='store_true', help='Try channels 0-15')
    ap.add_argument(
        '--wiggle',
        action='store_true',
        help='Rock min↔max on one channel (best bench test)',
    )
    ap.add_argument('--cycles', type=int, default=6, help='Wiggle cycles')
    args = ap.parse_args()

    if args.channel is not None:
        os.environ['ROVER_SERVO_CHANNEL'] = str(args.channel)
    if args.oe_gpio is not None:
        os.environ['ROVER_PCA9685_OE_GPIO'] = str(args.oe_gpio)
    elif 'ROVER_PCA9685_OE_GPIO' not in os.environ:
        os.environ['ROVER_PCA9685_OE_GPIO'] = '24'
    if args.oe_invert:
        os.environ['ROVER_PCA9685_OE_ACTIVE_LOW'] = '0'
    if args.wiggle:
        os.environ.setdefault('ROVER_SERVO_MIN_US', '1000')
        os.environ.setdefault('ROVER_SERVO_MAX_US', '2000')

    from rover_servo import _channel, _counts, _ensure, _set_channel, info
    import rover_servo

    bus, addr = _ensure()
    print(info(), flush=True)
    oe = rover_servo._oe
    print(f'OE GPIO: {oe!r}' if oe else 'OE: hardwired GND (expected)', flush=True)
    print(
        f'MODE1=0x{bus.read_byte_data(addr, 0):02x} '
        f'MODE2=0x{bus.read_byte_data(addr, 1):02x} '
        f'PRESCALE=0x{bus.read_byte_data(addr, 0xFE):02x}',
        flush=True,
    )
    print(
        '\nServo must be on a **PWM channel** triple (OUT0…OUT15), not the power-only header.\n'
        'Typical channel triple (check your silkscreen):  GND | V+ | SIG\n'
        'SG90 wires:  brown→GND  red→V+  orange/yellow→SIG\n'
        'Pi **GND** must also connect to PCA9685 **GND**.\n',
        flush=True,
    )

    if args.wiggle:
        ch = _channel()
        lo_us = int(os.environ.get('ROVER_SERVO_MIN_US', '1000'))
        hi_us = int(os.environ.get('ROVER_SERVO_MAX_US', '2000'))
        lo_c, hi_c = _counts(lo_us), _counts(hi_us)
        print(
            f'Wiggle OUT{ch}: {lo_us}µs ↔ {hi_us}µs  ({args.cycles} cycles, 1s each)',
            flush=True,
        )
        for i in range(args.cycles):
            for label, counts in (('MIN', lo_c), ('MAX', hi_c)):
                _set_channel(bus, addr, ch, counts)
                print(f'  cycle {i + 1}/{args.cycles}  {label}  counts={counts}', flush=True)
                time.sleep(1.0)
        return 0

    if args.all_channels:
        channels = list(range(16))
    elif args.scan_channels:
        channels = list(range(4))
    else:
        channels = [_channel()]

    pulse = _counts(args.pulse_us)
    for ch in channels:
        _set_channel(bus, addr, ch, pulse)
        reg = 0x06 + 4 * ch
        vals = [bus.read_byte_data(addr, reg + i) for i in range(4)]
        off = vals[2] | (vals[3] << 8)
        print(
            f'OUT{ch}: pulse_us={args.pulse_us} counts={pulse} regs={vals} OFF={off}',
            flush=True,
        )
        print(f'  holding {args.hold}s — if servo moves, note which OUT{n}', flush=True)
        time.sleep(args.hold)

    print(
        '\nNo movement? 1) Confirm orange wire on SIG pin of a channel triple\n'
        '2) Common GND: Pi GND ↔ PCA9685 GND ↔ servo brown\n'
        '3) Try rotating the 3-pin servo plug 180° on the channel header\n'
        '4) python3 test_servo_diag.py --wiggle --channel N for each N',
        flush=True,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
