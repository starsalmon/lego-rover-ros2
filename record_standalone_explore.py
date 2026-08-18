#!/usr/bin/env python3
"""Capture ESP standalone explore telemetry (UDP default; USB serial optional)."""
from __future__ import annotations

import argparse
import glob
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


def pick_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    matches = sorted(glob.glob('/dev/cu.usbmodem*'))
    if not matches:
        raise SystemExit('No /dev/cu.usbmodem* — use --udp or pass --port')
    return matches[-1]


def capture_udp(seconds: int, out: Path, host: str, send_go: bool) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 4243))
    sock.settimeout(0.2)
    if send_go:
        sock.sendto(b'!go', (host, 4244))
        print(f'Sent !go UDP → {host}:4244')
    t0 = time.monotonic()
    with out.open('w', encoding='utf-8') as f:
        while time.monotonic() - t0 < seconds:
            try:
                data, _addr = sock.recvfrom(4096)
            except TimeoutError:
                continue
            line = data.decode('utf-8', errors='replace').rstrip()
            if line:
                print(line)
                f.write(line + '\n')
                f.flush()


def capture_serial(seconds: int, out: Path, port: str, send_go: bool) -> None:
    if serial is None:
        raise SystemExit('pyserial required for --serial mode')
    with serial.Serial(port, 115200, timeout=0.05) as ser, out.open('w', encoding='utf-8') as f:
        time.sleep(0.3)
        ser.reset_input_buffer()
        if send_go:
            ser.write(b'!go\n')
            print('Sent !go (serial)')
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode('utf-8', errors='replace').rstrip()
            if line:
                print(line)
                f.write(line + '\n')
                f.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description='Record standalone rover telemetry')
    ap.add_argument('--udp', action='store_true', help='Capture WiFi UDP telem (default)')
    ap.add_argument('--serial', action='store_true', help='Capture USB serial (needs CDC on boot)')
    ap.add_argument('--host', default='rover-esp.local', help='Rover hostname for !go UDP')
    ap.add_argument('--port', help='Serial port (serial mode)')
    ap.add_argument('--seconds', type=int, default=90, help='Capture duration')
    ap.add_argument('--out', type=Path, help='Output log path')
    ap.add_argument('--no-go', action='store_true', help='Do not auto-start session')
    args = ap.parse_args()

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = args.out or Path(__file__).resolve().parent / 'captures' / f'standalone_explore_{stamp}.log'
    out.parent.mkdir(parents=True, exist_ok=True)

    use_serial = args.serial and not args.udp
    if not args.serial and not args.udp:
        use_serial = False

    print(f'Out:  {out}')
    print(f'Capturing {args.seconds}s — clear floor space around rover')

    if use_serial:
        port = pick_port(args.port)
        print(f'Port: {port}')
        capture_serial(args.seconds, out, port, not args.no_go)
    else:
        print(f'UDP listen :4243  rover {args.host}')
        capture_udp(args.seconds, out, args.host, not args.no_go)

    print(f'Done → {out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
