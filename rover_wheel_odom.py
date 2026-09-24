"""Wheel tick odometry + stall detect (ESP /rover/wheel/*_ticks)."""
from __future__ import annotations

import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass


def _truthy(name: str, default: str = '1') -> bool:
    return os.environ.get(name, default).strip().lower() not in ('0', 'no', 'false')


def wheel_odom_enabled() -> bool:
    return _truthy('ROVER_WHEEL_ODOM', '1')


def wheel_stall_enabled() -> bool:
    return _truthy('ROVER_WHEEL_STALL', '1')


def _strips() -> int:
    return max(1, int(os.environ.get('ROVER_WHEEL_STRIPS', '6')))


def _diam_m() -> float:
    mm = float(os.environ.get('ROVER_WHEEL_DIAM_MM', '50'))
    return max(1.0, mm) / 1000.0


def _track_m() -> float:
    mm = float(os.environ.get('ROVER_WHEEL_TRACK_MM', '118'))
    return max(0.01, mm) / 1000.0


def _stall_ms() -> float:
    return max(0.15, float(os.environ.get('ROVER_WHEEL_STALL_MS', '400'))) / 1000.0


def _stall_grace_ms() -> float:
    return max(0.0, float(os.environ.get('ROVER_WHEEL_STALL_GRACE_MS', '350'))) / 1000.0


def _stall_cooldown_ms() -> float:
    return max(0.5, float(os.environ.get('ROVER_WHEEL_STALL_COOLDOWN_MS', '3000'))) / 1000.0


def _stall_min_lin() -> float:
    return max(0.02, float(os.environ.get('ROVER_WHEEL_STALL_MIN_LIN', '0.06')))


def _stall_min_ang() -> float:
    return max(0.01, float(os.environ.get('ROVER_WHEEL_STALL_MIN_ANG', '0.04')))


def _stall_min_rate() -> float:
    """Min combined wheel tick rate (ticks/s) expected while driving."""
    return max(0.2, float(os.environ.get('ROVER_WHEEL_STALL_MIN_RATE', '2.0')))


@dataclass(frozen=True)
class WheelOdomSnapshot:
    left_ticks: int
    right_ticks: int
    left_m: float
    right_m: float
    distance_m: float
    heading_rad: float
    left_rate: float
    right_rate: float
    ticks_ok: bool


@dataclass
class _TickSample:
    t: float
    left: int
    right: int


class WheelOdometry:
    """Track wheel ticks → metres, heading, and stall-while-driving."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._strips = _strips()
        self._m_per_tick = (math.pi * _diam_m()) / self._strips
        self._track_m = _track_m()
        self._history: deque[_TickSample] = deque(maxlen=32)
        self._left_ticks = 0
        self._right_ticks = 0
        self._left_m = 0.0
        self._right_m = 0.0
        self._heading_rad = 0.0
        self._last_left: int | None = None
        self._last_right: int | None = None
        self._ticks_ok = False
        self._drive_since: float | None = None
        self._last_stall_mono = 0.0
        self._ticks_per_cmd = 0.0
        self._cal_n = 0

    def config_summary(self) -> str:
        return (
            f'{self._strips} strips/rev, '
            f'{self._m_per_tick * 1000:.2f} mm/tick, '
            f'track {_track_m() * 1000:.0f} mm'
        )

    def on_left(self, ticks: int) -> None:
        with self._lock:
            self._ingest(left=ticks)

    def on_right(self, ticks: int) -> None:
        with self._lock:
            self._ingest(right=ticks)

    def _tick_delta(self, new: int, prev: int | None) -> int:
        if prev is None:
            return 0
        delta = new - prev
        if delta < 0:
            # ESP reset or uint32 wrap — treat as fresh baseline.
            return 0
        return delta

    def _ingest(self, *, left: int | None = None, right: int | None = None) -> None:
        now = time.monotonic()
        dl = dr = 0
        if left is not None:
            dl = self._tick_delta(left, self._last_left)
            self._left_ticks = left
            self._last_left = left
            if dl:
                self._left_m += dl * self._m_per_tick
        if right is not None:
            dr = self._tick_delta(right, self._last_right)
            self._right_ticks = right
            self._last_right = right
            if dr:
                self._right_m += dr * self._m_per_tick
        if self._last_left is not None and self._last_right is not None:
            self._ticks_ok = True
        if left is not None or right is not None:
            l = self._left_ticks if self._last_left is not None else 0
            r = self._right_ticks if self._last_right is not None else 0
            self._history.append(_TickSample(now, l, r))
            while self._history and now - self._history[0].t > 0.6:
                self._history.popleft()
        if dl or dr:
            if self._track_m > 0:
                self._heading_rad += (dr - dl) * self._m_per_tick / self._track_m

    def snapshot(self) -> WheelOdomSnapshot:
        with self._lock:
            l_rate, r_rate = self._rates_locked()
            return WheelOdomSnapshot(
                left_ticks=self._left_ticks,
                right_ticks=self._right_ticks,
                left_m=self._left_m,
                right_m=self._right_m,
                distance_m=0.5 * (self._left_m + self._right_m),
                heading_rad=self._heading_rad,
                left_rate=l_rate,
                right_rate=r_rate,
                ticks_ok=self._ticks_ok,
            )

    def _rates_locked(self) -> tuple[float, float]:
        if len(self._history) < 2:
            return 0.0, 0.0
        first = self._history[0]
        last = self._history[-1]
        dt = last.t - first.t
        if dt < 0.12:
            return 0.0, 0.0
        dl = last.left - first.left
        dr = last.right - first.right
        if dl < 0:
            dl = last.left
        if dr < 0:
            dr = last.right
        return dl / dt, dr / dt

    def check_stall(self, cmd_lin: float, cmd_ang: float, now: float) -> bool:
        """True once when wheels should move but tick rate stays near zero."""
        if not wheel_stall_enabled():
            return False
        if now < self._last_stall_mono + _stall_cooldown_ms():
            return False

        wants_motion = abs(cmd_lin) >= _stall_min_lin() or abs(cmd_ang) >= _stall_min_ang()
        if not wants_motion:
            self._drive_since = None
            return False

        if self._drive_since is None:
            self._drive_since = now
            return False

        if now - self._drive_since < _stall_grace_ms():
            return False

        snap = self.snapshot()
        if not snap.ticks_ok:
            return False

        combined_rate = abs(snap.left_rate) + abs(snap.right_rate)
        cmd_mag = max(abs(cmd_lin), abs(cmd_ang))
        if combined_rate >= 1.0 and cmd_mag >= _stall_min_lin():
            observed = combined_rate / cmd_mag
            if self._cal_n == 0:
                self._ticks_per_cmd = observed
            else:
                self._ticks_per_cmd = 0.85 * self._ticks_per_cmd + 0.15 * observed
            self._cal_n = min(255, self._cal_n + 1)

        if self._cal_n >= 4 and self._ticks_per_cmd > 0.5:
            expect = self._ticks_per_cmd * cmd_mag
            floor = max(0.6, 0.30 * expect)
        else:
            floor = _stall_min_rate()
        if combined_rate >= floor:
            return False

        if now - self._drive_since < _stall_ms():
            return False

        self._last_stall_mono = now
        self._drive_since = None
        return True

    def wheels_turning(self, min_rate: float = 1.5) -> bool:
        snap = self.snapshot()
        if not snap.ticks_ok:
            return False
        return (abs(snap.left_rate) + abs(snap.right_rate)) >= min_rate
