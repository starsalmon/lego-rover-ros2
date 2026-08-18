"""Aux head servo via PCA9685 I2C PWM (Pi) or ESP ROS topic (default)."""
from __future__ import annotations

import os
import time

try:
    from rover_esp_servo import publish_angle as esp_publish_angle
    from rover_esp_servo import use_esp_servo
except ImportError:
    def use_esp_servo() -> bool:
        return False

    def esp_publish_angle(_deg: float) -> None:
        raise RuntimeError('rover_esp_servo not available')

_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06

_bus = None
_addr: int | None = None
_oe: object | None = None


def _i2c_bus() -> int:
    return int(os.environ.get('ROVER_I2C_BUS', '1'))


def _pca_addr() -> int:
    return int(os.environ.get('ROVER_PCA9685_ADDR', '0x40'), 0)


def _channel() -> int:
    return int(os.environ.get('ROVER_SERVO_CHANNEL', '0'))


def _oe_gpio() -> int:
    return int(os.environ.get('ROVER_PCA9685_OE_GPIO', '24'))


def _oe_active_low() -> bool:
    return os.environ.get('ROVER_PCA9685_OE_ACTIVE_LOW', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _enabled() -> bool:
    return os.environ.get('ROVER_SERVO_DISABLE', '0').strip().lower() not in ('1', 'yes', 'true')


def _min_angle() -> float:
    return float(os.environ.get('ROVER_SERVO_MIN_ANGLE', '0'))


def _max_angle() -> float:
    return float(os.environ.get('ROVER_SERVO_MAX_ANGLE', '180'))


def _center_angle() -> float:
    return float(os.environ.get('ROVER_SERVO_CENTER_ANGLE', '90'))


def _pulse_us(deg: float) -> int:
    # 500–2500 µs ≈ full 180° on typical micro servos; 1000–2000 µs is only ~90°.
    lo = float(os.environ.get('ROVER_SERVO_MIN_US', '500'))
    hi = float(os.environ.get('ROVER_SERVO_MAX_US', '2500'))
    span = _max_angle() - _min_angle()
    if span <= 0:
        return int(lo)
    t = (deg - _min_angle()) / span
    t = max(0.0, min(1.0, t))
    return int(lo + (hi - lo) * t)


def _counts(pulse_us: int) -> int:
    # PCA9685 @ 50 Hz → 20 ms frame; 12-bit counter.
    return int(pulse_us * 4096 / 20000)


def _open_bus():
    try:
        from smbus2 import SMBus
    except ImportError:
        from smbus import SMBus  # apt: python3-smbus
    try:
        return SMBus(_i2c_bus())
    except PermissionError as e:
        raise PermissionError(
            f'{e} — run: bash ~/lego-rover-ros2/enable_i2c.sh then log out/in'
        ) from e


def _enable_outputs() -> None:
    """PCA9685 OE: active-LOW on most boards (LOW = PWM on). Tie OE→GND or use GPIO."""
    global _oe
    pin = _oe_gpio()
    if pin <= 0:
        return
    if _oe is None:
        from gpiozero import DigitalOutputDevice

        from rover_gpio import pin_factory

        kw = {'pin_factory': pin_factory()} if pin_factory() else {}
        _oe = DigitalOutputDevice(
            pin,
            active_high=not _oe_active_low(),
            initial_value=True,
            **kw,
        )
    else:
        _oe.on()


def _ensure():
    global _bus, _addr
    if _bus is not None and _addr is not None:
        return _bus, _addr
    _enable_outputs()
    _addr = _pca_addr()
    _bus = _open_bus()
    _init_pca(_bus, _addr)
    return _bus, _addr


def _init_pca(bus, addr: int) -> None:
    freq = 50
    prescale = int(25000000 / (4096 * freq) + 0.5) - 1
    bus.write_byte_data(addr, _MODE1, 0x00)  # wake, autoincrement off
    bus.write_byte_data(addr, _MODE2, 0x04)  # totem-pole outputs
    old = bus.read_byte_data(addr, _MODE1)
    bus.write_byte_data(addr, _MODE1, (old & 0x7F) | 0x10)  # sleep
    bus.write_byte_data(addr, _PRESCALE, prescale)
    bus.write_byte_data(addr, _MODE1, old & ~0x10)  # wake
    time.sleep(0.005)
    bus.write_byte_data(addr, _MODE1, (old & ~0x10) | 0x80)  # restart
    time.sleep(0.005)
    bus.write_byte_data(addr, _MODE1, (old & ~0x10) | 0x20)  # autoincrement on


def _set_channel(bus, addr: int, channel: int, pulse_counts: int) -> None:
    reg = _LED0_ON_L + 4 * channel
    lo = pulse_counts & 0xFF
    hi = (pulse_counts >> 8) & 0xFF
    bus.write_byte_data(addr, reg, 0)
    bus.write_byte_data(addr, reg + 1, 0)
    bus.write_byte_data(addr, reg + 2, lo)
    bus.write_byte_data(addr, reg + 3, hi)


def close() -> None:
    global _bus, _addr, _oe
    if _bus is not None:
        try:
            detach()
        finally:
            _bus.close()
    if _oe is not None:
        _oe.close()
        _oe = None
    _bus = None
    _addr = None


def detach() -> None:
    if not _enabled() or _bus is None or _addr is None:
        return
    _set_channel(_bus, _addr, _channel(), 0)


def set_angle(deg: float, *, settle_s: float = 0.25) -> None:
    if not _enabled():
        return
    lo, hi = _min_angle(), _max_angle()
    deg = max(lo, min(hi, float(deg)))
    if use_esp_servo():
        esp_publish_angle(deg)
        if settle_s > 0:
            time.sleep(settle_s)
        return
    bus, addr = _ensure()
    _set_channel(bus, addr, _channel(), _counts(_pulse_us(deg)))
    if settle_s > 0:
        time.sleep(settle_s)


def center(*, settle_s: float = 0.3) -> None:
    set_angle(_center_angle(), settle_s=settle_s)


def info() -> str:
    if use_esp_servo():
        return (
            f'ESP PCA9685 ch {_channel()}  '
            f'{_min_angle():.0f}°–{_max_angle():.0f}°  (ROVER_SERVO_SOURCE=esp)'
        )
    oe = _oe_gpio()
    oe_s = f'OE→GPIO{oe}' if oe > 0 else 'OE→GND (or set ROVER_PCA9685_OE_GPIO)'
    return (
        f'PCA9685 @ 0x{_pca_addr():02x} bus {_i2c_bus()} '
        f'ch {_channel()}  {_min_angle():.0f}°–{_max_angle():.0f}°  {oe_s}'
    )
