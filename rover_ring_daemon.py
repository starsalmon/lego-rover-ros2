#!/usr/bin/env python3
"""Root daemon for WS2812 ring — rpi_ws281x needs DMA (run as systemd system service)."""
from __future__ import annotations

import colorsys
import math
import os
import sys
import time

from rpi_ws281x import Color, PixelStrip

CTL_PATH = os.environ.get('ROVER_RING_CTL', '/tmp/rover_ring_ctl')


def _gpio() -> int:
    return int(os.environ.get('ROVER_RING_GPIO', '18'))


def _count() -> int:
    return int(os.environ.get('ROVER_RING_COUNT', '8'))


def _brightness() -> int:
    raw = int(os.environ.get('ROVER_RING_BRIGHTNESS', '24'))
    return max(8, min(255, raw))


def _hsv_color(h: float, s: float, v: float) -> tuple[int, int, int]:
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def _set_pixel(strip: PixelStrip, i: int, r: int, g: int, b: int) -> None:
    strip.setPixelColor(i % strip.numPixels(), Color(r, g, b))


def _render_frame(
    strip: PixelStrip,
    mode: str,
    phase: float,
    flash: str,
    flash_t: float,
    *,
    radar_hits: list[bool] | None = None,
    radar_sweep: int = 0,
    sonar_dist_cm: list[int] | None = None,
) -> None:
    n = strip.numPixels()
    now = time.monotonic()

    if flash and now < flash_t:
        if flash == 'bump':
            on = int(now * 14) % 2 == 0
            col = (220, 0, 40) if on else (0, 0, 0)
            for i in range(n):
                _set_pixel(strip, i, *col)
            return
        if flash == 'stall':
            t = (math.sin(now * 10.0) + 1.0) * 0.5
            r, g, b = _hsv_color(0.12, 1.0, 0.15 + 0.85 * t)
            for i in range(n):
                _set_pixel(strip, i, r, g, b)
            return
        if flash == 'start':
            for i in range(n):
                h = (i / n + phase * 0.4) % 1.0
                r, g, b = _hsv_color(h, 1.0, 0.9)
                _set_pixel(strip, i, r, g, b)
            return
        if flash == 'stop':
            for i in range(n):
                t = max(0.0, 1.0 - abs(i - n / 2) / (n / 2))
                _set_pixel(strip, i, int(200 * t), 0, int(80 * t))
            return

    if mode == 'off':
        for i in range(n):
            _set_pixel(strip, i, 0, 0, 0)
        return

    if mode == 'standby':
        for i in range(n):
            h = (i / n + phase * 0.08) % 1.0
            r, g, b = _hsv_color(h, 0.85, 0.35 + 0.25 * math.sin(phase + i * 0.5))
            _set_pixel(strip, i, r, g, b)
        return

    if mode == 'ready':
        for i in range(n):
            dist = (i - phase * 2.5) % n
            v = max(0.0, 1.0 - dist / (n * 0.45))
            r, g, b = _hsv_color(0.35, 0.9, v)
            _set_pixel(strip, i, r, g, b)
        return

    if mode == 'auto':
        for i in range(n):
            h1 = (i / n + phase * 0.15) % 1.0
            h2 = (i / n - phase * 0.11 + 0.5) % 1.0
            r1, g1, b1 = _hsv_color(h1, 1.0, 0.55)
            r2, g2, b2 = _hsv_color(h2, 1.0, 0.45)
            _set_pixel(strip, i, (r1 + r2) // 2, (g1 + g2) // 2, (b1 + b2) // 2)
        return

    if mode == 'escape':
        for i in range(n):
            h = (0.02 + i * 0.02 + phase * 0.25) % 1.0
            r, g, b = _hsv_color(h, 1.0, 0.8)
            _set_pixel(strip, i, r, g, b)
        return

    if mode == 'radar':
        hits = list(radar_hits or [False] * n)
        if len(hits) < n:
            hits.extend([False] * (n - len(hits)))
        hits = hits[:n]
        sweep = int(radar_sweep) % n
        for i in range(n):
            if hits[i]:
                _set_pixel(strip, i, 220, 30, 10)
            elif i == sweep:
                _set_pixel(strip, i, 40, 160, 255)
            else:
                _set_pixel(strip, i, 0, 12, 30)
        return

    if mode == 'sonar':
        dists = list(sonar_dist_cm or [255] * n)
        if len(dists) < n:
            dists.extend([255] * (n - len(dists)))
        dists = dists[:n]
        sweep = int(radar_sweep) % n
        for i in range(n):
            if i == sweep:
                _set_pixel(strip, i, 40, 200, 255)
                continue
            d = dists[i]
            if d >= 255:
                _set_pixel(strip, i, 0, 10, 24)
                continue
            # 0 cm = red, 120+ cm = blue/green
            t = max(0.0, min(1.0, d / 120.0))
            r = int(255 * (1.0 - t * 0.85))
            g = int(40 + 180 * t)
            b = int(60 + 195 * t)
            _set_pixel(strip, i, r, g, b)
        return

    for i in range(n):
        _set_pixel(strip, i, 0, 0, 0)


def _read_cmd() -> str | None:
    try:
        with open(CTL_PATH, encoding='utf-8') as f:
            line = f.read().strip()
    except OSError:
        return None
    return line or None


def _apply_cmd(line: str, state: dict) -> None:
    parts = line.split()
    if not parts:
        return
    if parts[0] == 'mode' and len(parts) >= 2:
        state['mode'] = parts[1]
    elif parts[0] == 'radar' and len(parts) >= 2:
        state['mode'] = 'radar'
        state['radar_hits'] = [c == '1' for c in parts[1]]
        if len(parts) >= 3:
            state['radar_sweep'] = int(parts[2])
    elif parts[0] == 'sonar' and len(parts) >= 2:
        state['mode'] = 'sonar'
        blob = parts[1]
        n = _count()
        dists: list[int] = []
        for i in range(0, len(blob), 3):
            dists.append(int(blob[i : i + 3]))
        if len(dists) < n:
            dists.extend([255] * (n - len(dists)))
        state['sonar_dist_cm'] = dists[:n]
        if len(parts) >= 3:
            state['radar_sweep'] = int(parts[2])
    elif parts[0] == 'flash' and len(parts) >= 2:
        state['flash'] = parts[1]
        dur = float(parts[2]) if len(parts) >= 3 else 0.7
        state['flash_until'] = time.monotonic() + dur


def main() -> int:
    gpio = _gpio()
    if gpio <= 0:
        print('ROVER_RING_GPIO=0 — ring disabled', file=sys.stderr)
        return 0

    strip = PixelStrip(
        _count(),
        gpio,
        freq_hz=800_000,
        dma=10,
        invert=False,
        brightness=_brightness(),
        channel=0,
    )
    strip.begin()
    print(f'rover-ring daemon on GPIO {gpio}, {_count()} LEDs', flush=True)

    state = {
        'mode': 'standby',
        'flash': '',
        'flash_until': 0.0,
        'phase': 0.0,
        'last_cmd': '',
        'radar_hits': [False] * _count(),
        'radar_sweep': 0,
        'sonar_dist_cm': [255] * _count(),
    }

    while True:
        cmd = _read_cmd()
        if cmd and cmd != state['last_cmd']:
            state['last_cmd'] = cmd
            _apply_cmd(cmd, state)

        flash = state['flash'] if time.monotonic() < state['flash_until'] else ''
        state['phase'] += 0.06
        _render_frame(
            strip,
            state['mode'],
            state['phase'],
            flash,
            state['flash_until'],
            radar_hits=state.get('radar_hits'),
            radar_sweep=int(state.get('radar_sweep', 0)),
            sonar_dist_cm=state.get('sonar_dist_cm'),
        )
        strip.show()
        time.sleep(0.04)


if __name__ == '__main__':
    raise SystemExit(main())
