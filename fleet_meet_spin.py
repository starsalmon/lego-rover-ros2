"""Meet dance — two full 360° spins when a fleet peer IR ID is decoded."""
from __future__ import annotations

import math
import os
import time


class MeetSpinController:
    PEER_NONE = 255

    def __init__(self, fleet_id: int) -> None:
        self._fleet_id = fleet_id
        self._peer_stable = 0
        self._active = False
        self._cooldown_until = 0.0
        self._yaw_rad = 0.0
        self._start_yaw = 0.0
        self._last_peer = self.PEER_NONE
        self._meet_peer = self.PEER_NONE
        self._rate = float(os.environ.get('MEET_SPIN_RATE', '0.38'))
        self._turns = float(os.environ.get('MEET_TURNS', '2'))
        self._gyro_sign = float(os.environ.get('MEET_GYRO_SIGN', '1'))
        self._peer_need = max(2, int(os.environ.get('MEET_PEER_STABLE', '3')))

    @property
    def active(self) -> bool:
        return self._active

    def on_peer(self, peer_id: int) -> None:
        now = time.monotonic()
        if self._active or now < self._cooldown_until:
            return
        if peer_id == self._fleet_id:
            self._peer_stable = max(0, self._peer_stable - 1)
            return
        if peer_id == self.PEER_NONE:
            # ESP holds peer ~360 ms — decay slowly so brief gaps do not reset meet.
            self._peer_stable = max(0, self._peer_stable - 1)
            return
        if peer_id == self._last_peer:
            self._peer_stable = min(24, self._peer_stable + 1)
        else:
            self._last_peer = peer_id
            self._peer_stable = 1
        if self._peer_stable >= self._peer_need:
            self._begin(peer_id)

    def _begin(self, peer_id: int) -> None:
        self._active = True
        self._start_yaw = self._yaw_rad
        self._meet_peer = peer_id
        self._peer_stable = 0

    def on_imu_gz(self, gz: float, dt: float) -> None:
        if dt <= 0.0:
            return
        self._yaw_rad += self._gyro_sign * gz * dt

    def tick(self) -> tuple[float, float] | None:
        if not self._active:
            return None
        turned_deg = abs(self._yaw_rad - self._start_yaw) * 180.0 / math.pi
        if turned_deg >= 360.0 * self._turns:
            self._active = False
            self._cooldown_until = time.monotonic() + float(
                os.environ.get('MEET_COOLDOWN_SEC', '12')
            )
            return 0.0, 0.0
        return 0.0, self._rate

    def peer_label(self) -> str:
        return f'bot{self._meet_peer}' if self._meet_peer != self.PEER_NONE else '?'
