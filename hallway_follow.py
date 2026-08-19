"""Hallway wall-follow — keep a fixed lateral offset using side VL53 ToF."""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field

from geometry_msgs.msg import Twist


@dataclass
class HallwayConfig:
    target_m: float = 0.10
    cruise: float = 0.15
    kp: float = 4.0
    max_ang: float = 0.22
    stop_ahead_m: float = 0.35
    slow_ahead_m: float = 0.55
    min_valid_m: float = 0.04
    max_valid_m: float = 2.0
    stale_s: float = 0.6
    blind_cruise: float = 0.06
    turn_s: float = 7.0
    align_err_m: float = 0.12
    # auto | left | right — which wall to hug at target_m
    wall: str = 'auto'

    @classmethod
    def from_env(cls) -> HallwayConfig:
        return cls(
            target_m=float(os.environ.get('HALLWAY_TARGET_M', '0.10')),
            cruise=float(os.environ.get('HALLWAY_CRUISE', '0.15')),
            kp=float(os.environ.get('HALLWAY_KP', '4.0')),
            max_ang=float(os.environ.get('HALLWAY_MAX_ANG', '0.22')),
            stop_ahead_m=float(os.environ.get('HALLWAY_STOP_AHEAD_M', '0.35')),
            slow_ahead_m=float(os.environ.get('HALLWAY_SLOW_AHEAD_M', '0.55')),
            min_valid_m=float(os.environ.get('HALLWAY_MIN_VALID_M', '0.04')),
            max_valid_m=float(os.environ.get('HALLWAY_MAX_VALID_M', '2.0')),
            stale_s=float(os.environ.get('HALLWAY_STALE_S', '0.6')),
            blind_cruise=float(os.environ.get('HALLWAY_BLIND_CRUISE', '0.06')),
            turn_s=float(os.environ.get('HALLWAY_TURN_S', '7.0')),
            align_err_m=float(os.environ.get('HALLWAY_ALIGN_ERR_M', '0.12')),
            wall=os.environ.get('HALLWAY_WALL', 'auto').strip().lower(),
        )


@dataclass
class HallwayFollow:
    cfg: HallwayConfig
    _left_m: float = field(default_factory=lambda: math.nan)
    _right_m: float = field(default_factory=lambda: math.nan)
    _fwd_m: float = field(default_factory=lambda: math.nan)
    _left_ts: float = 0.0
    _right_ts: float = 0.0
    _fwd_ts: float = 0.0
    _pan_deg: float = 90.0
    _last_driving: bool = False
    _phase: str = 'cruise'
    _turn_until: float = 0.0
    _turn_ang: float = 0.0

    def update_left(self, meters: float) -> None:
        self._left_m = meters
        self._left_ts = time.monotonic()

    def update_right(self, meters: float) -> None:
        self._right_m = meters
        self._right_ts = time.monotonic()

    def update_forward(self, meters: float, pan_deg: float) -> None:
        if abs(pan_deg - 90.0) > 25.0:
            return
        self._fwd_m = meters
        self._fwd_ts = time.monotonic()
        self._pan_deg = pan_deg

    def _fresh(self, ts: float) -> bool:
        return (time.monotonic() - ts) <= self.cfg.stale_s

    def _valid_side(self, meters: float, ts: float) -> bool:
        if not self._fresh(ts) or not math.isfinite(meters):
            return False
        return self.cfg.min_valid_m <= meters <= self.cfg.max_valid_m

    def _pick_wall(self) -> tuple[str, float] | tuple[None, None]:
        wall = self.cfg.wall
        left_ok = self._valid_side(self._left_m, self._left_ts)
        right_ok = self._valid_side(self._right_m, self._right_ts)
        if wall == 'left' and left_ok:
            return 'left', self._left_m
        if wall == 'right' and right_ok:
            return 'right', self._right_m
        if wall == 'auto':
            picks: list[tuple[str, float]] = []
            if left_ok:
                picks.append(('left', self._left_m))
            if right_ok:
                picks.append(('right', self._right_m))
            if not picks:
                return None, None
            picks.sort(key=lambda item: item[1])
            return picks[0]
        return None, None

    def _fwd_blocked(self) -> bool:
        return self._fresh(self._fwd_ts) and math.isfinite(self._fwd_m) and (
            self._fwd_m <= self.cfg.stop_ahead_m
        )

    def _start_corner_turn(self, side: str, now: float) -> None:
        # Hugging left wall at a dead end → turn right (down the hall).
        self._phase = 'turn'
        self._turn_until = now + self.cfg.turn_s
        self._turn_ang = -self.cfg.max_ang if side == 'left' else self.cfg.max_ang

    def fill_twist(self, msg: Twist) -> None:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._last_driving = False
        now = time.monotonic()

        if self._phase == 'turn':
            if now < self._turn_until:
                msg.angular.z = self._turn_ang
                self._last_driving = True
                return
            self._phase = 'cruise'

        side, side_m = self._pick_wall()

        if self._fwd_blocked():
            if side is not None:
                self._start_corner_turn(side, now)
                msg.angular.z = self._turn_ang
                self._last_driving = True
                return
            msg.linear.x = -self.cfg.cruise * 0.65
            self._last_driving = True
            return

        if side is None or side_m is None:
            msg.linear.x = self.cfg.blind_cruise
            self._last_driving = True
            return

        err = side_m - self.cfg.target_m
        ang = self.cfg.kp * err
        if side == 'right':
            ang = -ang
        ang = max(-self.cfg.max_ang, min(self.cfg.max_ang, ang))

        lin = self.cfg.cruise
        if abs(err) > self.cfg.align_err_m:
            lin *= 0.35
        if self._fresh(self._fwd_ts) and math.isfinite(self._fwd_m):
            if self._fwd_m < self.cfg.slow_ahead_m:
                scale = max(
                    0.0,
                    (self._fwd_m - self.cfg.stop_ahead_m)
                    / max(0.05, self.cfg.slow_ahead_m - self.cfg.stop_ahead_m),
                )
                lin = self.cfg.cruise * scale

        msg.linear.x = lin
        msg.angular.z = ang
        self._last_driving = lin > 0.02 or abs(ang) > 0.02
