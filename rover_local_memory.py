"""Short-term heading memory — not SLAM.

IMU yaw drifts and cmd_vel is not metres. Remember which way we were
facing when we hit something, and refuse to drive that heading again
until the nose is actually open.
"""
from __future__ import annotations

import math
import time


N_BINS = 8
BIN_DEG = 360.0 / N_BINS
MARK_COOLDOWN_S = 0.6
DECAY_S = 45.0


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class LocalHeadingMemory:
    def __init__(self) -> None:
        self._hits = [0.0] * N_BINS
        self._leg = 0.0
        self._last_mark = 0.0
        self._last_decay = time.monotonic()

    def reset(self) -> None:
        self._hits = [0.0] * N_BINS
        self._leg = 0.0
        self._last_mark = 0.0
        self._last_decay = time.monotonic()

    def start_leg(self) -> None:
        self._leg = 0.0

    def note_progress(
        self, lin: float, dt: float, yaw_rad: float, *, front_open: bool = False,
        travel_m: float | None = None,
    ) -> None:
        self._decay()
        if lin > 0.05 and dt > 0.0:
            step = travel_m if travel_m is not None else lin * dt
            if step > 0.0:
                self._leg += step
            # Only forgive a heading if we actually drove into open space.
            if self._leg >= 0.50 and front_open:
                i = self._bin(math.degrees(yaw_rad))
                if self._hits[i] > 0.0:
                    self._hits[i] = 0.0
                self._leg = 0.0

    def mark_blocked(self, yaw_rad: float, *, reason: str) -> str | None:
        """Count a hit on the heading we were travelling — always, not only short legs."""
        now = time.monotonic()
        self._decay()
        if now - self._last_mark < MARK_COOLDOWN_S:
            self.start_leg()
            return None
        travelled = self._leg
        self._last_mark = now
        self.start_leg()
        i = self._bin(math.degrees(yaw_rad))
        self._hits[i] += 1.0
        heading = i * BIN_DEG
        return (
            f'memory: {heading:.0f}° blocked ({reason}, hit {self._hits[i]:.0f}, '
            f'travelled {travelled:.2f})'
        )

    def heading_blocked(self, yaw_rad: float, thresh: float = 1.0) -> bool:
        self._decay()
        return self._hits[self._bin(math.degrees(yaw_rad))] >= thresh

    def prefer_sign(self, yaw_rad: float, sign: float) -> tuple[float, str | None]:
        """Keep a left/right choice unless it aims into a hotter blocked sector."""
        if sign == 0.0:
            return self.suggest_sign(yaw_rad)
        yaw = math.degrees(yaw_rad)
        left_h = self._hits[self._bin(yaw + 50.0)]
        right_h = self._hits[self._bin(yaw - 50.0)]
        if sign > 0.0 and left_h >= 1.0 and left_h > right_h + 0.2:
            return -1.0, 'memory: skip left (blocked) → right'
        if sign < 0.0 and right_h >= 1.0 and right_h > left_h + 0.2:
            return 1.0, 'memory: skip right (blocked) → left'
        return sign, None

    def suggest_sign(self, yaw_rad: float) -> tuple[float, str | None]:
        """When sensors don't know, turn toward the less-tried side."""
        yaw = math.degrees(yaw_rad)
        left_h = self._hits[self._bin(yaw + 50.0)]
        right_h = self._hits[self._bin(yaw - 50.0)]
        if left_h < right_h - 0.2:
            return 1.0, 'memory: try left (right already blocked)'
        if right_h < left_h - 0.2:
            return -1.0, 'memory: try right (left already blocked)'
        return 0.0, None

    def cruise_bias(self, yaw_rad: float, max_steer: float) -> float:
        """If this heading already failed, turn. Do not drive straight at it again."""
        self._decay()
        yaw = math.degrees(yaw_rad)
        ahead = self._hits[self._bin(yaw)]
        if ahead < 1.0:
            return 0.0
        left_h = self._hits[self._bin(yaw + 50.0)]
        right_h = self._hits[self._bin(yaw - 50.0)]
        mag = max_steer * (0.70 if ahead < 2.0 else 1.0)
        if left_h <= right_h:
            return mag
        return -mag

    def _decay(self) -> None:
        now = time.monotonic()
        dt = now - self._last_decay
        if dt < DECAY_S:
            return
        steps = int(dt / DECAY_S)
        self._last_decay = now
        for _ in range(steps):
            self._hits = [max(0.0, h * 0.65) for h in self._hits]

    def _bin(self, deg: float) -> int:
        wrapped = (_wrap_deg(deg) + 360.0) % 360.0
        return int(wrapped / BIN_DEG) % N_BINS
