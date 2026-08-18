#!/usr/bin/env python3
"""GPIO 17 start/stop button — press to start, press again to stop."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time

from gpiozero import Button

from rover_gpio import pin_factory
from rover_ring import flash as ring_flash
from rover_speaker import play as speaker_play

_button: Button | None = None


def _pin() -> int:
    return int(os.environ.get('ROVER_BUTTON_GPIO', '17'))


def close_button() -> None:
    global _button
    if _button is not None:
        try:
            _button.close()
        except Exception:
            pass
        _button = None


def _button_dev() -> Button:
    global _button
    if _button is not None:
        return _button
    kwargs: dict = {'pull_up': True, 'bounce_time': 0.04}
    factory = pin_factory()
    if factory is not None:
        kwargs['pin_factory'] = factory
    _button = Button(_pin(), **kwargs)
    return _button


def _drain_press(btn: Button) -> None:
    """Ignore the start press still held when autonomous begins."""
    if btn.is_pressed:
        btn.wait_for_release()
    time.sleep(0.12)


def arm_stop_button(stop_event: threading.Event) -> Button:
    """Open button GPIO — stop is polled via wait_stop_press() in a thread."""
    close_button()
    time.sleep(0.08)
    last_err: Exception | None = None
    for attempt in range(6):
        try:
            btn = _button_dev()
            break
        except Exception as exc:
            last_err = exc
            if 'busy' not in str(exc).lower():
                raise
            close_button()
            time.sleep(0.35)
    else:
        raise last_err or RuntimeError('GPIO busy — could not open button')

    _drain_press(btn)
    pin = _pin()
    print(f'Stop armed on GPIO {pin} (press again to stop).', flush=True)
    return btn


def wait_stop_press() -> None:
    """Block until stop button pressed (use from a dedicated thread)."""
    btn = _button_dev()
    btn.wait_for_press()


def poll_stop_pressed(*, debounce_s: float = 0.02) -> bool:
    """Non-blocking stop check — edge-friendly, no long hold required."""
    btn = _button_dev()
    if not btn.is_pressed:
        return False
    if debounce_s <= 0:
        return True
    time.sleep(debounce_s)
    return btn.is_pressed


def _physical_pin(bcm: int) -> int:
    """BCM → 40-pin header number (common pins only)."""
    return {17: 11, 18: 12, 27: 13, 24: 18}.get(bcm, bcm)


def wait_start() -> None:
    pin = _pin()
    phys = _physical_pin(pin)
    btn = _button_dev()
    print(
        f'Waiting for button on GPIO {pin} (physical pin {phys}, other leg to GND)...',
        flush=True,
    )
    btn.wait_for_press()
    print('Button pressed — starting.', flush=True)
    try:
        speaker_play('autonomous_start')
        ring_flash('start', 0.45)
    except Exception:
        pass
    close_button()


def watch_stop() -> None:
    """Legacy blocking stop — prefer rover_session.py."""
    pin = _pin()
    btn = _button_dev()
    rover_dir = os.environ.get('ROVER_DIR', os.path.expanduser('~/lego-rover-ros2'))
    _drain_press(btn)
    print(f'Autonomous running — press GPIO {pin} again to stop.', flush=True)
    btn.wait_for_press()
    print('Stop button pressed — stopping motors.', flush=True)
    speaker_play('autonomous_stop')
    ring_flash('stop', 0.6)
    subprocess.run(
        ['bash', '-lc', f'source {rover_dir}/rover_common.sh && rover_source_ros && rover_stop_motors'],
        check=False,
    )
    time.sleep(0.2)
    subprocess.run(['pkill', '-f', 'autonomous_explore.py'], check=False)
    subprocess.run(['pkill', '-f', 'demo_showcase.py'], check=False)


def main() -> None:
    parser = argparse.ArgumentParser(description='LEGO rover GPIO button')
    parser.add_argument('mode', choices=['start', 'watch'], help='start=wait to begin, watch=stop while running')
    args = parser.parse_args()

    if args.mode == 'start':
        wait_start()
    else:
        watch_stop()


if __name__ == '__main__':
    main()
