"""Hallway wall-follow — keep a fixed lateral offset using side VL53 ToF."""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field

from geometry_msgs.msg import Twist


@dataclass
class HallwayConfig:
    target_m: float = 0.28
    cruise: float = 0.15
    kp: float = 3.2
    max_ang: float = 0.22
    stop_ahead_m: float = 0.40
    slow_ahead_m: float = 0.80
    min_valid_m: float = 0.04
    max_valid_m: float = 2.0
    stale_s: float = 0.6
    blind_cruise: float = 0.06
    turn_s: float = 7.0
    align_err_m: float = 0.12
    center_kp: float = 2.2
    # auto | left | right — which wall to hug at target_m
    wall: str = 'auto'

    @classmethod
    def from_env(cls) -> HallwayConfig:
        return cls(
            target_m=float(os.environ.get('HALLWAY_TARGET_M', '0.28')),
            cruise=float(os.environ.get('HALLWAY_CRUISE', '0.15')),
            kp=float(os.environ.get('HALLWAY_KP', '3.2')),
            max_ang=float(os.environ.get('HALLWAY_MAX_ANG', '0.22')),
            stop_ahead_m=float(os.environ.get('HALLWAY_STOP_AHEAD_M', '0.40')),
            slow_ahead_m=float(os.environ.get('HALLWAY_SLOW_AHEAD_M', '0.80')),
            min_valid_m=float(os.environ.get('HALLWAY_MIN_VALID_M', '0.04')),
            max_valid_m=float(os.environ.get('HALLWAY_MAX_VALID_M', '2.0')),
            stale_s=float(os.environ.get('HALLWAY_STALE_S', '0.6')),
            blind_cruise=float(os.environ.get('HALLWAY_BLIND_CRUISE', '0.06')),
            turn_s=float(os.environ.get('HALLWAY_TURN_S', '7.0')),
            align_err_m=float(os.environ.get('HALLWAY_ALIGN_ERR_M', '0.12')),
            center_kp=float(os.environ.get('HALLWAY_CENTER_KP', '2.2')),
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
    _blind_turn_sign: float = 1.0
    _ir_fl: bool = False
    _ir_fr: bool = False
    _ir_rear: bool = False
    _ir_hits_at: float = 0.0
    _side_prev: tuple[float, float, float] | None = None
    _ang_filt: float = 0.0

    def update_left(self, meters: float) -> None:
        self._left_m = meters
        self._left_ts = time.monotonic()

    def update_right(self, meters: float) -> None:
        self._right_m = meters
        self._right_ts = time.monotonic()

    def update_front(self, meters: float) -> None:
        """Nose L8 — this is forward. Rear pan sonar is not."""
        if not math.isfinite(meters) or meters <= 0.0:
            return
        self._fwd_m = meters
        self._fwd_ts = time.monotonic()

    def update_forward(self, meters: float, pan_deg: float) -> None:
        # Legacy: pan 90° is aft. Ignore it as a path.
        del meters, pan_deg

    def update_ir_hits(self, bits: int) -> None:
        bits = int(bits) & 0xFF
        self._ir_fl = bool(bits & 0x01)
        self._ir_fr = bool(bits & 0x02)
        self._ir_rear = bool(bits & 0x04)
        self._ir_hits_at = time.monotonic()

    def _ir_fresh(self) -> bool:
        return (time.monotonic() - self._ir_hits_at) < 0.30

    def _front_ir(self) -> bool:
        return False

    def _rear_ir(self) -> bool:
        return self._ir_fresh() and self._ir_rear

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
        if self._front_ir():
            return True
        return self._fresh(self._fwd_ts) and math.isfinite(self._fwd_m) and (
            self._fwd_m <= self.cfg.stop_ahead_m
        )

    def _start_corner_turn(self, side: str, now: float) -> None:
        # +angular.z = left, − = right (same as ESP mix).
        # Hugging left wall at a dead end → turn right (down the hall).
        self._phase = 'turn'
        self._turn_until = now + self.cfg.turn_s
        self._turn_ang = -self.cfg.max_ang if side == 'left' else self.cfg.max_ang

    def fill_corridor_reaction(self, msg: Twist) -> bool:
        """Drive straight between two walls. Nudge away only if a hip is close.

        Do not equalize left/right ToF — yaw makes the outside sensor read
        longer and that used to steer harder into the wall.
        """
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        left_ok = self._valid_side(self._left_m, self._left_ts)
        right_ok = self._valid_side(self._right_m, self._right_ts)
        if not (left_ok and right_ok):
            return False

        comfort = 0.38
        deadband = 0.08
        max_ang = 0.04
        ang = 0.0
        if abs(self._left_m - self._right_m) < deadband:
            ang = 0.0
        elif self._left_m < comfort and (
            self._left_m < self._right_m - deadband
        ):
            ang -= max_ang * min(1.0, (comfort - self._left_m) / 0.20)
        elif self._right_m < comfort and (
            self._right_m < self._left_m - deadband
        ):
            ang += max_ang * min(1.0, (comfort - self._right_m) / 0.20)
        ang = max(-max_ang, min(max_ang, ang))
        if abs(ang) < 0.022:
            ang = 0.0

        lin = 0.14
        pinch = min(self._left_m, self._right_m)
        if pinch < 0.22:
            lin = 0.11

        msg.linear.x = lin
        msg.angular.z = ang
        self._last_driving = lin > 0.02 or abs(ang) > 0.02
        return True

    def fill_twist(self, msg: Twist) -> None:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self._last_driving = False
        now = time.monotonic()

        if self._phase == 'turn':
            if now < self._turn_until:
                # Arc forward lightly (avoid spin-in-place against a wall).
                msg.linear.x = 0.0 if self._front_ir() else self.cfg.blind_cruise
                msg.angular.z = self._turn_ang
                self._last_driving = True
                return
            self._phase = 'cruise'

        left_ok = self._valid_side(self._left_m, self._left_ts)
        right_ok = self._valid_side(self._right_m, self._right_ts)
        side, side_m = self._pick_wall()

        # Both walls: this is the centering that already worked on the bench.
        # Do not reverse here. A reverse in a hall hits the tail sonar, which
        # zeros it, and the ESP then drops the turn — the robot just sits.
        if left_ok and right_ok and self.cfg.wall in ('auto', 'center'):
            # Drive the hall. A small offset is not a turn. Only nudge off a hip
            # that is actually close, and keep forward much larger than the steer
            # so it does not circle.
            sl, sr = self._left_m, self._right_m
            ang = 0.0
            comfort = 0.40
            if sl < comfort or sr < comfort:
                if sl < sr:
                    ang = -0.045
                elif sr < sl:
                    ang = 0.045
                closer = min(sl, sr)
                if closer < 0.28:
                    ang = math.copysign(0.07, ang if ang != 0.0 else (1.0 if sr < sl else -1.0))
            lin = 0.14
            if min(sl, sr) < 0.24:
                lin = 0.12
            msg.linear.x = lin
            msg.angular.z = ang
            self._last_driving = True
            return

        if self._fwd_blocked():
            if side is not None:
                self._start_corner_turn(side, now)
                msg.linear.x = 0.0 if self._front_ir() else self.cfg.blind_cruise
                msg.angular.z = self._turn_ang
                self._last_driving = True
                return
            # Blind dead-end: back up while biasing a turn direction, so we don't
            # reverse perfectly straight into the same geometry.
            if self._rear_ir():
                msg.linear.x = 0.0
                msg.angular.z = self._blind_turn_sign * self.cfg.max_ang * 0.55
            else:
                msg.linear.x = -self.cfg.cruise * 0.65
                msg.angular.z = self._blind_turn_sign * self.cfg.max_ang * 0.55
            self._blind_turn_sign *= -1.0
            self._last_driving = True
            return

        if side is None or side_m is None:
            msg.linear.x = 0.12
            self._last_driving = True
            return

        err = side_m - self.cfg.target_m
        # Far from left wall → turn left (+) toward it. Far from right → turn right (−).
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
                # Avoid micro stop/start jitter — keep a tiny crawl until we hit stop_ahead.
                lin = max(self.cfg.blind_cruise, self.cfg.cruise * scale)

        msg.linear.x = lin
        msg.angular.z = ang
        self._last_driving = lin > 0.02 or abs(ang) > 0.02
