#!/usr/bin/env python3
"""Run autonomous explore with a reliable stop button (parent owns GPIO or ESP topic)."""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from rover_esp_button import (
    clear_button_state,
    consume_press,
    consume_shutdown,
    set_session_active,
    use_esp_button,
)
from rover_button import arm_stop_button, close_button, poll_stop_pressed
from rover_ir import close as ir_close
from rover_ring import flash as ring_flash
from rover_ring import set_mode as ring_set_mode
from rover_ring import shutdown as ring_shutdown
from rover_ring_ipc import read_event as ring_read_event
from rover_speaker import play as speaker_play

DIR = Path(__file__).resolve().parent
_IPC = os.environ.get('ROVER_RING_IPC', '/tmp/rover_ring_evt')
_DRIVE_SCRIPT = os.environ.get('ROVER_DRIVE_SCRIPT', 'autonomous_explore.py')
_STOP_GRACE_SEC = float(os.environ.get('ROVER_STOP_GRACE_SEC', '1.5'))


def _stop_pressed() -> bool:
    if use_esp_button():
        if consume_shutdown():
            return True
        return consume_press()
    return poll_stop_pressed(debounce_s=0.015)


def _stop_motors() -> None:
    subprocess.run(
        ['bash', '-lc', f'source {DIR}/rover_common.sh && rover_source_ros && rover_stop_motors'],
        check=False,
    )


def _halt_autonomous(
    proc: subprocess.Popen,
    stop: threading.Event,
    sonar_proc: subprocess.Popen | None = None,
) -> None:
    """Stop session fast — ESP display updates immediately via SESSION_FILE."""
    if stop.is_set() and proc.poll() is not None:
        _stop_motors()
        return
    stop.set()
    set_session_active(False)
    speaker_play('autonomous_stop')

    if proc.poll() is None:
        try:
            proc.send_signal(signal.SIGTERM)
        except OSError:
            proc.kill()

    if sonar_proc is not None and sonar_proc.poll() is None:
        sonar_proc.terminate()

    deadline = time.monotonic() + 0.8
    while proc.poll() is None and time.monotonic() < deadline:
        _stop_motors()
        time.sleep(0.04)

    if proc.poll() is None:
        proc.kill()
        subprocess.run(['pkill', '-9', '-f', _DRIVE_SCRIPT], check=False)
        try:
            proc.wait(timeout=0.4)
        except subprocess.TimeoutExpired:
            pass

    if sonar_proc is not None and sonar_proc.poll() is None:
        try:
            sonar_proc.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            sonar_proc.kill()

    _stop_motors()
    ir_close()


def _ring_watcher(stop: threading.Event, last: list[str]) -> None:
    while not stop.is_set():
        ev = ring_read_event()
        if ev and ev != last[0]:
            last[0] = ev
            if ev in ('bump', 'stall', 'start', 'stop'):
                ring_flash(ev)
            elif ev == 'escape':
                ring_set_mode('escape')
            elif ev == 'auto':
                ring_set_mode('auto')
        time.sleep(0.06)


def _stop_watcher(
    stop: threading.Event,
    proc_holder: list[subprocess.Popen],
    grace_until: float,
    sonar_holder: list[subprocess.Popen | None],
) -> None:
    """Poll stop input in a thread — stops motors immediately on press."""
    while not stop.is_set():
        try:
            if time.monotonic() < grace_until:
                time.sleep(0.02)
                continue
            if _stop_pressed():
                print('Stop button — halting motors now.', flush=True)
                _halt_autonomous(proc_holder[0], stop, sonar_holder[0])
                return
        except Exception as exc:
            print(f'Stop button watcher ended: {exc}', flush=True)
            return
        time.sleep(0.008)


def main() -> int:
    try:
        Path(_IPC).unlink(missing_ok=True)
    except OSError:
        pass

    stop = threading.Event()
    clear_button_state()
    time.sleep(0.12)
    clear_button_state()
    set_session_active(True)
    if not use_esp_button():
        arm_stop_button(stop)

    grace_until = time.monotonic() + _STOP_GRACE_SEC
    ring_set_mode('auto')
    ipc_last = ['']
    ipc_thread = threading.Thread(target=_ring_watcher, args=(stop, ipc_last), daemon=True)
    ipc_thread.start()

    if use_esp_button():
        print('Autonomous running — press ESP Go button (GPIO14) to stop.', flush=True)
    else:
        print('Autonomous running — press GPIO button again to stop.', flush=True)

    proc = subprocess.Popen(
        [sys.executable, str(DIR / _DRIVE_SCRIPT)],
        cwd=str(DIR),
        env=os.environ.copy(),
    )
    sonar_proc = None
    record_proc = None
    if os.environ.get('ROVER_SONAR', '1').strip().lower() in ('1', 'yes', 'true'):
        sonar_proc = subprocess.Popen(
            [sys.executable, str(DIR / 'rover_sonar_ring.py')],
            cwd=str(DIR),
            env=os.environ.copy(),
        )
    if os.environ.get('ROVER_SONAR_RECORD', '0').strip().lower() in ('1', 'yes', 'true'):
        record_proc = subprocess.Popen(
            [sys.executable, str(DIR / 'rover_sonar_escape_recorder.py')],
            cwd=str(DIR),
            env=os.environ.copy(),
        )
    proc_holder: list[subprocess.Popen] = [proc]
    sonar_holder: list[subprocess.Popen | None] = [sonar_proc]
    stop_thread = threading.Thread(
        target=_stop_watcher, args=(stop, proc_holder, grace_until, sonar_holder), daemon=True,
    )
    stop_thread.start()

    rc = 0
    try:
        while not stop.is_set():
            if proc.poll() is not None:
                rc = proc.returncode or 0
                break
            try:
                if time.monotonic() < grace_until:
                    time.sleep(0.02)
                    continue
                if _stop_pressed():
                    print('Stop button — halting motors now.', flush=True)
                    _halt_autonomous(proc, stop, sonar_proc)
                    break
            except Exception:
                pass
            time.sleep(0.008)

        if stop.is_set():
            _halt_autonomous(proc, stop, sonar_proc)
            ring_flash('stop', 0.4)
            rc = 0
    finally:
        stop.set()
        set_session_active(False)
        if not use_esp_button():
            close_button()
        ipc_thread.join(timeout=0.2)
        stop_thread.join(timeout=0.2)
        if sonar_proc is not None and sonar_proc.poll() is None:
            sonar_proc.terminate()
            try:
                sonar_proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                sonar_proc.kill()
        if record_proc is not None and record_proc.poll() is None:
            record_proc.terminate()
            try:
                record_proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                record_proc.kill()
        _stop_motors()
        ir_close()
        ring_set_mode('standby')
        ring_shutdown()

    return rc


if __name__ == '__main__':
    raise SystemExit(main())
