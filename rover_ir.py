"""Pi GPIO IR — Front (22) + Aux (27), pulsed emitters on GPIO 6 / 26.

Set ROVER_IR_SOURCE=esp (default) when IR is on the ESP via MCP23008 expanders.
"""
from __future__ import annotations

import atexit
import os
import threading
import time
from typing import Literal

from gpiozero import DigitalInputDevice, DigitalOutputDevice

from rover_gpio import pin_factory

try:
    from rover_esp_ir import (
        ensure_bridge_started,
        front_pulse_state_cached,
        read_front_cached,
        request_aux_sample,
        request_front_sample,
        use_esp_ir,
    )
except ImportError:
    def use_esp_ir() -> bool:
        return False

    def ensure_bridge_started() -> None:
        return

    def read_front_cached() -> bool:
        return False

    def front_pulse_state_cached() -> tuple[bool, bool, bool]:
        return False, False, False

    def request_aux_sample(timeout_s: float = 0.65) -> tuple[bool, bool]:
        raise RuntimeError('rover_esp_ir not available')

    def request_front_sample(timeout_s: float = 0.35) -> tuple[bool, bool]:
        raise RuntimeError('rover_esp_ir not available')

Pull = Literal['down', 'up', 'float']

_front: DigitalInputDevice | None = None
_aux: DigitalInputDevice | None = None
_aux_led: DigitalOutputDevice | None = None
_front_led: DigitalOutputDevice | None = None

_front_pulse_thread: threading.Thread | None = None
_front_pulse_stop = threading.Event()
_front_pulse_lock = threading.Lock()
_front_pulsed_hit = False
_front_off_hit = False
_front_on_hit = False
_atexit_registered = False


def _register_atexit() -> None:
    global _atexit_registered
    if not _atexit_registered:
        atexit.register(close)
        _atexit_registered = True


def _pin(name: str, default: int) -> int:
    return int(os.environ.get(name, str(default)))


def _active_low() -> bool:
    return os.environ.get('ROVER_IR_ACTIVE_LOW', '0').strip().lower() in ('1', 'yes', 'true')


def _pull_mode() -> Pull:
    v = os.environ.get('ROVER_IR_PULL', 'float').strip().lower()
    if v in ('up', 'pull_up', '1'):
        return 'up'
    if v in ('down', 'pull_down'):
        return 'down'
    return 'float'


def _factory_kw() -> dict:
    f = pin_factory()
    return {'pin_factory': f} if f is not None else {}


def _input_kw(pull: Pull | None) -> dict:
    p = pull if pull is not None else _pull_mode()
    kw = _factory_kw()
    if p == 'up':
        kw['pull_up'] = True
    else:
        kw['pull_up'] = False
    return kw


def _front_led_gpio() -> int:
    return _pin('ROVER_IR_FRONT_LED_GPIO', 6)


def _front_pulse_enabled() -> bool:
    if _front_led_gpio() <= 0:
        return False
    return os.environ.get('ROVER_IR_FRONT_PULSE', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _front_require_delta() -> bool:
    return os.environ.get('ROVER_IR_FRONT_REQUIRE_DELTA', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _front_off_ms() -> float:
    return float(os.environ.get('ROVER_IR_FRONT_OFF_MS', '50')) / 1000.0


def _front_on_ms() -> float:
    return float(os.environ.get('ROVER_IR_FRONT_ON_MS', '10')) / 1000.0


def _aux_off_ms() -> float:
    return float(os.environ.get('ROVER_IR_AUX_OFF_MS', '10')) / 1000.0


def _aux_on_ms() -> float:
    return float(os.environ.get('ROVER_IR_AUX_ON_MS', '100')) / 1000.0


def pins() -> tuple[int, int, int, int]:
    return (
        _pin('ROVER_IR_FRONT_GPIO', 22),
        _pin('ROVER_IR_AUX_GPIO', 27),
        _pin('ROVER_IR_AUX_LED_GPIO', 26),
        _front_led_gpio(),
    )


def _ensure(
    *, start_front_pulse: bool = True,
) -> tuple[DigitalInputDevice, DigitalInputDevice, DigitalOutputDevice, DigitalOutputDevice | None]:
    global _front, _aux, _aux_led, _front_led
    if _front is None or _aux is None or _aux_led is None:
        front_pin, aux_pin, aux_led_pin, front_led_pin = pins()
        ikw = _input_kw(None)
        fkw = _factory_kw()
        _front = DigitalInputDevice(front_pin, **ikw)
        _aux = DigitalInputDevice(aux_pin, **ikw)
        _aux_led = DigitalOutputDevice(aux_led_pin, initial_value=False, **fkw)
        if front_led_pin > 0:
            _front_led = DigitalOutputDevice(front_led_pin, initial_value=False, **fkw)
        else:
            _front_led = None
    _register_atexit()
    if start_front_pulse and _front_pulse_enabled():
        _start_front_pulser()
    return _front, _aux, _aux_led, _front_led


def _stop_front_pulser() -> None:
    global _front_pulse_thread
    _front_pulse_stop.set()
    if _front_pulse_thread is not None and _front_pulse_thread.is_alive():
        _front_pulse_thread.join(timeout=0.5)
    _front_pulse_thread = None
    if _front_led is not None:
        try:
            _front_led.off()
        except Exception:
            pass


def _front_pulse_loop() -> None:
    global _front_pulsed_hit, _front_off_hit, _front_on_hit
    if _front is None or _front_led is None:
        return
    off_ms = _front_off_ms()
    on_ms = _front_on_ms()
    require_delta = _front_require_delta()
    while not _front_pulse_stop.is_set():
        _front_led.off()
        if _front_pulse_stop.wait(off_ms):
            break
        off_hit = _hit(_front.value)
        _front_led.on()
        if _front_pulse_stop.wait(on_ms):
            _front_led.off()
            break
        on_hit = _hit(_front.value)
        _front_led.off()
        hit = (on_hit and not off_hit) if require_delta else on_hit
        with _front_pulse_lock:
            _front_off_hit = off_hit
            _front_on_hit = on_hit
            _front_pulsed_hit = hit


def _start_front_pulser() -> None:
    global _front_pulse_thread
    if _front is None or _front_led is None:
        return
    if _front_pulse_thread is not None and _front_pulse_thread.is_alive():
        return
    _front_pulse_stop.clear()
    _front_pulse_thread = threading.Thread(
        target=_front_pulse_loop,
        name='rover-front-ir-pulse',
        daemon=True,
    )
    _front_pulse_thread.start()


def close() -> None:
    global _front, _aux, _aux_led, _front_led
    _stop_front_pulser()
    for dev in (_front_led, _aux_led, _aux, _front):
        if dev is not None:
            try:
                dev.close()
            except Exception:
                pass
    _front = _aux = _aux_led = _front_led = None


def _hit(raw: int) -> bool:
    high = bool(raw)
    return not high if _active_low() else high


def read_gpio_raw(pin: int) -> int:
    """Direct lgpio read (bypass gpiozero) — for diagnostics."""
    import lgpio

    chip = lgpio.gpiochip_open(0)
    try:
        pull = _pull_mode()
        flags = lgpio.SET_PULL_NONE
        if pull == 'up':
            flags = lgpio.SET_PULL_UP
        elif pull == 'down':
            flags = lgpio.SET_PULL_DOWN
        lgpio.gpio_claim_input(chip, pin, flags)
        return int(lgpio.gpio_read(chip, pin))
    finally:
        lgpio.gpiochip_close(chip)


def read_front() -> bool:
    """Front detect — uses background pulsed emitter + delta when enabled."""
    if use_esp_ir():
        ensure_bridge_started()
        return read_front_cached()
    if _front_pulse_enabled():
        _ensure()
        with _front_pulse_lock:
            return _front_pulsed_hit
    front, _, _, _ = _ensure()
    return _hit(front.value)


def front_pulse_state() -> tuple[bool, bool, bool]:
    """(off_hit, on_hit, delta_hit) from background front pulser."""
    if use_esp_ir():
        ensure_bridge_started()
        return front_pulse_state_cached()
    _ensure()
    with _front_pulse_lock:
        return _front_off_hit, _front_on_hit, _front_pulsed_hit


def read_aux_value(*, settle_ms: float = 0.010) -> bool:
    """Read aux OUT without changing LED state."""
    if use_esp_ir():
        return False
    _, aux, _, _ = _ensure()
    if settle_ms > 0:
        time.sleep(settle_ms)
    return _hit(aux.value)


def set_aux_led(on: bool) -> None:
    if use_esp_ir():
        return
    _, _, led, _ = _ensure()
    if on:
        led.on()
    else:
        led.off()


def set_front_led(on: bool) -> None:
    if use_esp_ir():
        return
    _, _, _, led = _ensure()
    if led is None:
        return
    if on:
        led.on()
    else:
        led.off()


def sample_front_pulse(
    *,
    off_ms: float | None = None,
    on_ms: float | None = None,
) -> tuple[bool, bool]:
    """Single front pulse sample (off_hit, on_hit) — blocks pulser briefly."""
    if use_esp_ir():
        ensure_bridge_started()
        timeout = 0.35
        if off_ms is not None:
            timeout += float(off_ms)
        if on_ms is not None:
            timeout += float(on_ms)
        return request_front_sample(timeout_s=timeout)
    was_running = _front_pulse_thread is not None and _front_pulse_thread.is_alive()
    if was_running:
        _stop_front_pulser()
    front, _, _, led = _ensure(start_front_pulse=False)
    if led is None:
        return False, False
    off_s = _front_off_ms() if off_ms is None else off_ms
    on_s = _front_on_ms() if on_ms is None else on_ms
    led.off()
    time.sleep(off_s)
    off_hit = _hit(front.value)
    led.on()
    time.sleep(on_s)
    on_hit = _hit(front.value)
    led.off()
    if _front_pulse_enabled():
        _start_front_pulser()
    return off_hit, on_hit


def read_aux_pulsed(
    *,
    off_ms: float | None = None,
    on_ms: float | None = None,
    require_delta: bool = False,
) -> bool:
    off_hit, on_hit = sample_aux_pulse(off_ms=off_ms, on_ms=on_ms)
    if require_delta:
        return on_hit and not off_hit
    return on_hit


def sample_aux_pulse(
    *,
    off_ms: float | None = None,
    on_ms: float | None = None,
) -> tuple[bool, bool]:
    """Pulse aux emitter; return (off_hit, on_hit)."""
    if use_esp_ir():
        ensure_bridge_started()
        timeout = 0.35
        if off_ms is not None:
            timeout += float(off_ms)
        if on_ms is not None:
            timeout += float(on_ms)
        return request_aux_sample(timeout_s=timeout)
    _, aux, led, _ = _ensure()
    off_s = _aux_off_ms() if off_ms is None else off_ms
    on_s = _aux_on_ms() if on_ms is None else on_ms
    led.off()
    time.sleep(off_s)
    off_hit = _hit(aux.value)
    led.on()
    time.sleep(on_s)
    on_hit = _hit(aux.value)
    led.off()
    return off_hit, on_hit


def read_both(*, pulse_aux: bool = True, aux_led_hold: bool = False) -> tuple[bool, bool]:
    front_hit = read_front()
    if aux_led_hold:
        set_aux_led(True)
        aux_hit = read_aux_value(settle_ms=0.010)
    elif pulse_aux:
        aux_hit = read_aux_pulsed(require_delta=True)
    else:
        set_aux_led(False)
        aux_hit = read_aux_value(settle_ms=0.005)
    return front_hit, aux_hit
