#!/usr/bin/env python3
"""Passive buzzer / piezo on Pi GPIO (PWM tones).

Uses the same approach as lego-roverquest: PWMOutputDevice + LGPIOFactory,
short blips at low duty — much cleaner than TonalBuzzer square waves.

Default: GPIO 17 (physical pin 11). Set ROVER_SPEAKER_GPIO=0 to disable.
"""
from __future__ import annotations

import os
import queue
import sys
import threading
import time
from typing import Sequence

# Equal temperament (A4 = 440 Hz).
NOTE_HZ: dict[str, int] = {
    'C4': 262,
    'D4': 294,
    'E4': 330,
    'F4': 349,
    'G4': 392,
    'A4': 440,
    'B4': 494,
    'C5': 523,
    'D5': 587,
    'E5': 659,
    'F5': 698,
    'G5': 784,
    'A5': 880,
    'B5': 988,
}

# (note_name, duration_seconds)
Melody = Sequence[tuple[str, float]]

MELODIES: dict[str, Melody] = {
    'ready': [('C5', 0.09), ('E5', 0.09), ('G5', 0.14)],
    'autonomous_start': [('C4', 0.06), ('E4', 0.06), ('G4', 0.08)],
    'autonomous_stop': [('G4', 0.06), ('E4', 0.06), ('C4', 0.08)],
    'bump': [('A4', 0.04), ('C5', 0.04), ('A4', 0.05)],
    'stall': [('A4', 0.06), ('A4', 0.06), ('F4', 0.08)],
    'front_ir': [('E5', 0.05), ('C5', 0.07)],
    'beacon_lock': [('G4', 0.05), ('C5', 0.05), ('E5', 0.09)],
    'ir_aux': [('G5', 0.03), ('E5', 0.04)],
    'ping': [('E5', 0.03)],
    'ping_near': [('G5', 0.025)],
    'ping_far': [('C5', 0.025)],
    'button': [('E5', 0.028)],
    'menu': [('C5', 0.025), ('G5', 0.03)],
    'menu_done': [('G5', 0.04), ('C5', 0.03)],
}

NOTE_GAP_S = 0.02
DEFAULT_VOL = 0.38
_pin_factory = None
_queue: queue.Queue[str | None] = queue.Queue()
_worker: threading.Thread | None = None
_gpio_busy_logged = 0.0


def _gpio() -> int:
    return int(os.environ.get('ROVER_SPEAKER_GPIO', '17'))


def _enabled() -> bool:
    return _gpio() > 0


def _vol() -> float:
    raw = float(os.environ.get('ROVER_SPEAKER_VOL', str(DEFAULT_VOL)))
    return max(0.05, min(1.0, raw))


def _pin_factory_dev():
    global _pin_factory
    if _pin_factory is not None:
        return _pin_factory
    try:
        from rover_gpio import pin_factory as _rover_pf

        _pin_factory = _rover_pf()
    except ImportError:
        _pin_factory = None
    if _pin_factory is None:
        try:
            from gpiozero.pins.lgpio import LGPIOFactory

            _pin_factory = LGPIOFactory()
        except ImportError:
            _pin_factory = None
    return _pin_factory


def _open_pwm():
    if not _enabled():
        return None
    try:
        from gpiozero import PWMOutputDevice
    except ImportError as exc:
        print(f'rover_speaker: gpiozero not installed: {exc}', file=sys.stderr)
        return None

    factory = _pin_factory_dev()
    kwargs = {'initial_value': 0.0}
    if factory is not None:
        kwargs['pin_factory'] = factory
    try:
        return PWMOutputDevice(_gpio(), **kwargs)
    except Exception as exc:
        global _gpio_busy_logged
        now = time.monotonic()
        if now - _gpio_busy_logged > 5.0:
            _gpio_busy_logged = now
            print(f'rover_speaker: cannot open GPIO {_gpio()}: {exc}', file=sys.stderr)
        return None


def _play_sync(melody: Melody) -> None:
    pwm = _open_pwm()
    if pwm is None:
        return
    vol = _vol()
    try:
        for note, dur in melody:
            hz = NOTE_HZ.get(note)
            if hz is None:
                continue
            pwm.frequency = int(hz)
            pwm.value = float(vol)
            time.sleep(dur)
            pwm.value = 0.0
            time.sleep(NOTE_GAP_S)
    finally:
        try:
            pwm.value = 0.0
            pwm.close()
        except Exception:
            pass


def _worker_loop() -> None:
    while True:
        name = _queue.get()
        try:
            if name is None:
                return
            melody = MELODIES.get(name)
            if melody:
                _play_sync(melody)
        finally:
            _queue.task_done()


def _ensure_worker() -> None:
    global _worker
    if _worker is not None and _worker.is_alive():
        return
    _worker = threading.Thread(target=_worker_loop, name='rover-speaker', daemon=True)
    _worker.start()


def play(name: str) -> None:
    """Queue a named melody (non-blocking)."""
    if not _enabled() or name not in MELODIES:
        return
    _ensure_worker()
    _queue.put(name)


def play_blocking(name: str) -> None:
    """Play immediately on this thread (for shell scripts)."""
    if not _enabled():
        print('rover_speaker: disabled (ROVER_SPEAKER_GPIO=0)', file=sys.stderr)
        return
    if name not in MELODIES:
        print(f'rover_speaker: unknown melody {name!r}', file=sys.stderr)
        return
    _play_sync(MELODIES[name])


def release() -> None:
    """No-op — PWM is opened and closed per melody."""


def blip(*, near: bool = False) -> None:
    """Quick sonar ping (non-blocking). near=True → higher pitch."""
    play('ping_near' if near else 'ping_far')


def main() -> int:
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} <{"|".join(MELODIES)}>', file=sys.stderr)
        return 1
    name = sys.argv[1]
    if name not in MELODIES:
        print(f'Unknown melody: {name}', file=sys.stderr)
        return 1
    play_blocking(name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
