"""Rear IR radar — smooth servo sweep, escape scan, background cruise mapper."""
from __future__ import annotations

import os
import random
import threading
import time

from rover_ir import read_front, sample_aux_pulse, set_aux_led
from rover_ring import radar_frame
from rover_scan_map import forward_led, led_to_bearing, led_to_servo, servo_to_led
from rover_servo import center, set_angle
from rover_speaker import blip, play as speaker_play

_bg_instance: BackgroundRadar | None = None


def _enabled() -> bool:
    return os.environ.get('ROVER_RADAR_DISABLE', '0').strip().lower() not in ('1', 'yes', 'true')


def _steps() -> int:
    return int(os.environ.get('ROVER_RADAR_STEPS', '7'))


def _on_ms() -> float:
    return float(os.environ.get('ROVER_RADAR_ON_MS', '120'))


def _settle_s() -> float:
    return float(os.environ.get('ROVER_RADAR_SETTLE_S', '0.28'))


def _smooth_sweep() -> bool:
    return os.environ.get('ROVER_RADAR_SMOOTH', '0').strip().lower() in ('1', 'yes', 'true')


def _step_deg() -> float:
    return max(1.0, float(os.environ.get('ROVER_RADAR_STEP_DEG', '4')))


def _step_s() -> float:
    return max(0.02, float(os.environ.get('ROVER_RADAR_STEP_S', '0.045')))


def _servo_lo() -> float:
    return float(os.environ.get('ROVER_SERVO_MIN_ANGLE', '0'))


def _servo_hi() -> float:
    return float(os.environ.get('ROVER_SERVO_MAX_ANGLE', '180'))


def _n_led() -> int:
    return int(os.environ.get('ROVER_RING_COUNT', '8'))


def _cruise_radar_enabled() -> bool:
    if not _enabled():
        return False
    return os.environ.get('ROVER_CRUISE_RADAR', '1').strip().lower() not in ('0', 'no', 'false')


def cruise_radar_enabled() -> bool:
    return _cruise_radar_enabled()


def _cruise_radar_steps() -> int:
    return int(os.environ.get('ROVER_CRUISE_RADAR_STEPS', '5'))


def _cruise_radar_settle() -> float:
    return float(os.environ.get('ROVER_CRUISE_RADAR_SETTLE', '0.10'))


def _angles(lo: float, hi: float, steps: int) -> list[float]:
    if steps < 2:
        return [lo, hi]
    step = (hi - lo) / (steps - 1)
    return [lo + i * step for i in range(steps)]


def iter_sweep_angles(
    lo: float,
    hi: float,
    *,
    smooth: bool | None = None,
    steps: int | None = None,
) -> list[float]:
    """Return sweep angles — smooth small steps or classic step-and-settle poses."""
    if smooth is None:
        smooth = _smooth_sweep()
    if smooth:
        out: list[float] = []
        deg = lo
        while deg <= hi + 0.001:
            out.append(deg)
            deg += _step_deg()
        if out and out[-1] < hi - 0.5:
            out.append(hi)
        return out
    return _angles(lo, hi, steps if steps is not None else _steps())


def clearest_gap_led(hits: list[bool]) -> int:
    n = len(hits)
    best_start = 0
    best_len = 0
    i = 0
    while i < n:
        if hits[i]:
            i += 1
            continue
        j = i
        while j < n and not hits[j]:
            j += 1
        run = j - i
        if run > best_len:
            best_len = run
            best_start = i
        i = j if j > i else i + 1
    if best_len == 0:
        return n // 2
    return best_start + best_len // 2


def bearing_to_spin(bearing: float) -> tuple[float, float]:
    """Shortest body turn from forward to face compass bearing → (sign, degrees)."""
    rot = bearing % 360.0
    if rot > 180.0:
        rot -= 360.0
    sign = 1.0 if rot >= 0 else -1.0
    flip = float(os.environ.get('ROVER_ESCAPE_SPIN_SIGN', '-1'))
    return sign * flip, abs(rot)


def rear_arc_spin(hits: list[bool], lo: float, hi: float, n: int) -> tuple[float, float, float]:
    """Turn to face the clearest sector on rear aux sweep (servo right→back→left)."""
    gap_led = clearest_gap_led(hits)
    bearing = led_to_bearing(gap_led, n)
    sign, deg = bearing_to_spin(bearing)
    deg = max(50.0, min(abs(deg) * 0.85 + 35.0, 115.0))
    return sign, deg, bearing


def _sample_step(
    deg: float,
    lo: float,
    hi: float,
    n: int,
    hits: list[bool],
    *,
    ping: bool,
    update_ring: bool,
    smooth: bool,
) -> None:
    settle = 0.0 if smooth else _settle_s()
    set_angle(deg, settle_s=settle)
    led = servo_to_led(deg, lo, hi, n)
    if update_ring:
        radar_frame(hits, led)

    front_hit = read_front()
    off_hit, on_hit = sample_aux_pulse(on_ms=_on_ms() / 1000.0)
    aux_hit = int(on_hit) - int(off_hit) > 0
    if front_hit:
        hits[forward_led(n)] = True
    if aux_hit:
        hits[led] = True
    if ping:
        if aux_hit:
            blip(near=True)
        elif front_hit:
            blip(near=False)

    if update_ring:
        radar_frame(hits, led)

    if smooth:
        time.sleep(_step_s())


def scan_hits(
    *,
    update_ring: bool = True,
    ping: bool | None = None,
    smooth: bool | None = None,
) -> list[bool]:
    """Sweep aux head; return ring-sized obstacle map (True = blocked)."""
    lo, hi = _servo_lo(), _servo_hi()
    n = _n_led()
    hits = [False] * n
    if not _enabled():
        return hits

    if ping is None:
        ping = update_ring
    if smooth is None:
        smooth = _smooth_sweep()

    bg = background_instance()
    if bg is not None:
        bg.pause()

    try:
        for deg in iter_sweep_angles(lo, hi, smooth=smooth):
            _sample_step(
                deg, lo, hi, n, hits,
                ping=ping, update_ring=update_ring, smooth=smooth,
            )

        center(settle_s=0.15 if smooth else _settle_s())
        if update_ring:
            gap_led = clearest_gap_led(hits)
            radar_frame(hits, gap_led)
        set_aux_led(False)
        return hits
    finally:
        if bg is not None:
            bg.resume()


def cruise_steer_bias(hits: list[bool]) -> float:
    """Gentle steer from background map — away from blocked side sectors."""
    gain = float(os.environ.get('ROVER_CRUISE_RADAR_BIAS', '0.07'))
    if gain <= 0 or not hits:
        return 0.0
    n = len(hits)
    right_score = 0.0
    left_score = 0.0
    for i, blocked in enumerate(hits):
        if not blocked:
            continue
        bearing = led_to_bearing(i, n)
        if 45.0 <= bearing <= 135.0:
            right_score += 1.0
        elif 225.0 <= bearing <= 315.0:
            left_score += 1.0
    # Blocked right → turn left (+); blocked left → turn right (−).
    return gain * (right_score - left_score)


def wall_servo_angle(side: str | None = None) -> float:
    """Servo angle to aim aux IR along the side wall (hardware: 0°=right, 180°=left)."""
    side = (side or os.environ.get('ROVER_WALL_SIDE', 'right')).strip().lower()
    if side in ('left', 'l'):
        return float(os.environ.get('ROVER_WALL_SERVO_LEFT', '180'))
    return float(os.environ.get('ROVER_WALL_SERVO_RIGHT', '0'))


def pick_escape_spin(
    hits: list[bool] | None = None,
    *,
    spin_min: float = 45.0,
    spin_max: float = 140.0,
) -> tuple[float, float, float]:
    """IR map → (spin_dir, spin_deg, gap_bearing). Random fallback if no gap."""
    if not _enabled():
        sign = random.choice([-1.0, 1.0])
        deg = random.uniform(spin_min, spin_max)
        return sign, deg, -1.0

    lo, hi = _servo_lo(), _servo_hi()
    n = _n_led()
    if hits is None:
        hits = scan_hits(update_ring=True, ping=True)

    if hits and not all(hits):
        sign, deg, bearing = rear_arc_spin(hits, lo, hi, n)
        deg = max(spin_min, min(spin_max, deg))
        return sign, deg, bearing

    sign = random.choice([-1.0, 1.0])
    deg = random.uniform(spin_min, spin_max)
    return sign, deg, -1.0


def background_instance() -> BackgroundRadar | None:
    return _bg_instance


class BackgroundRadar:
    """Slow rear sweep while cruising — does not block the drive loop."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._hits = [False] * _n_led()
        self._sweep_idx = 0
        self._pause = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @classmethod
    def start(cls) -> BackgroundRadar:
        global _bg_instance
        if _bg_instance is not None:
            return _bg_instance
        _bg_instance = cls()
        _bg_instance._thread = threading.Thread(
            target=_bg_instance._loop,
            name='rover-cruise-radar',
            daemon=True,
        )
        _bg_instance._thread.start()
        return _bg_instance

    @classmethod
    def stop(cls) -> None:
        global _bg_instance
        if _bg_instance is None:
            return
        _bg_instance._stop.set()
        if _bg_instance._thread is not None:
            _bg_instance._thread.join(timeout=0.35)
        set_aux_led(False)
        try:
            center(settle_s=0.1)
        except Exception:
            pass
        _bg_instance = None

    def pause(self) -> None:
        self._pause.set()

    def resume(self) -> None:
        self._pause.clear()

    def get_hits(self) -> tuple[list[bool], int]:
        with self._lock:
            return list(self._hits), self._sweep_idx

    def steer_bias(self) -> float:
        hits, _ = self.get_hits()
        return cruise_steer_bias(hits)

    def _loop(self) -> None:
        lo, hi = _servo_lo(), _servo_hi()
        n = _n_led()
        settle = _cruise_radar_settle()
        angles = iter_sweep_angles(lo, hi, smooth=False, steps=_cruise_radar_steps())
        if not angles:
            return
        ping = os.environ.get('ROVER_CRUISE_RADAR_PING', '0').strip().lower() in (
            '1',
            'yes',
            'true',
        )
        show_ring = os.environ.get('ROVER_CRUISE_RADAR_RING', '0').strip().lower() in (
            '1',
            'yes',
            'true',
        )

        while not self._stop.is_set():
            if self._pause.is_set():
                self._stop.wait(0.08)
                continue

            hits = [False] * n
            for deg in angles:
                if self._stop.is_set() or self._pause.is_set():
                    break
                set_angle(deg, settle_s=settle)
                led = servo_to_led(deg, lo, hi, n)

                try:
                    front_hit = read_front()
                    off_hit, on_hit = sample_aux_pulse(on_ms=_on_ms() / 1000.0)
                except (TimeoutError, RuntimeError) as exc:
                    self.get_logger().warning(f'cruise radar sample skipped: {exc}')
                    front_hit = False
                    off_hit, on_hit = False, False
                aux_hit = int(on_hit) - int(off_hit) > 0
                if front_hit:
                    hits[forward_led(n)] = True
                if aux_hit:
                    hits[led] = True
                if ping and aux_hit:
                    speaker_play('ir_aux')

                with self._lock:
                    self._hits = list(hits)
                    self._sweep_idx = led
                if show_ring:
                    radar_frame(hits, led)

            if not self._pause.is_set():
                try:
                    center(settle_s=settle)
                except Exception:
                    pass
            self._stop.wait(0.35)
