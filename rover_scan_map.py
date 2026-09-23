"""Map rear servo scan angles ↔ compass bearing ↔ LED ring index."""
from __future__ import annotations

import os


def _zero_bearing() -> float:
    """Compass bearing (° clockwise from forward) where LED 0 sits on the ring."""
    return float(os.environ.get('ROVER_RING_ZERO_BEARING', '270'))


def _clockwise() -> bool:
    return os.environ.get('ROVER_RING_CLOCKWISE', '1').strip().lower() not in ('0', 'no', 'false')


def _servo_bearing_at_min() -> float:
    """Where the aux head points at servo min angle (default: rover right = 90°)."""
    return float(os.environ.get('ROVER_SERVO_BEARING_MIN', '90'))


def _servo_bearing_at_max() -> float:
    """Where the aux head points at servo max angle (default: rover left = 270°)."""
    return float(os.environ.get('ROVER_SERVO_BEARING_MAX', '270'))


def servo_to_bearing(deg: float, lo: float, hi: float) -> float:
    """Servo angle → compass bearing (0°=forward, 90°=right, 180°=back, 270°=left)."""
    span = hi - lo
    if span <= 0:
        return _servo_bearing_at_min()
    t = (deg - lo) / span
    b0 = _servo_bearing_at_min()
    b1 = _servo_bearing_at_max()
    return (b0 + t * (b1 - b0)) % 360.0


def bearing_to_led(bearing: float, n: int) -> int:
    """Compass bearing → ring LED index (LED 0 at ROVER_RING_ZERO_BEARING)."""
    if n <= 0:
        return 0
    step = 360.0 / n
    delta = (bearing - _zero_bearing()) % 360.0
    if not _clockwise():
        delta = (-delta) % 360.0
    return int(round(delta / step)) % n


def led_to_bearing(led: int, n: int) -> float:
    """Ring LED index → compass bearing."""
    if n <= 0:
        return 0.0
    step = 360.0 / n
    if _clockwise():
        return (_zero_bearing() + led * step) % 360.0
    return (_zero_bearing() - led * step) % 360.0


def servo_to_led(deg: float, lo: float, hi: float, n: int) -> int:
    return bearing_to_led(servo_to_bearing(deg, lo, hi), n)


def led_to_servo(led: int, lo: float, hi: float, n: int) -> float:
    """Best servo angle to aim the aux head toward a ring LED direction."""
    bearing = led_to_bearing(led, n)
    b0 = _servo_bearing_at_min()
    b1 = _servo_bearing_at_max()
    span = b1 - b0
    if span < 0:
        span += 360.0
    if span <= 0:
        return (lo + hi) / 2
    # Map bearing into the rear arc [b0, b1] along the shorter path through back.
    if b0 <= b1:
        if b0 <= bearing <= b1:
            t = (bearing - b0) / span
        else:
            t = 0.5
    else:
        if bearing >= b0 or bearing <= b1:
            d = (bearing - b0) % 360.0
            t = d / span
        else:
            t = 0.5
    return lo + max(0.0, min(1.0, t)) * (hi - lo)


def forward_led(n: int) -> int:
    return bearing_to_led(0.0, n)


def front_pan_to_bearing(pan_deg: float, center: float = 90.0) -> float:
    """Legacy name. Pan now faces aft: 0=right, 90=back, 180=left."""
    return (center - pan_deg + 180.0) % 360.0


def front_pan_to_led(pan_deg: float, n: int, *, center: float = 90.0, mirror: bool = False) -> int:
    bearing = front_pan_to_bearing(pan_deg, center=center)
    if mirror:
        bearing = (360.0 - bearing) % 360.0
    return bearing_to_led(bearing, n)
