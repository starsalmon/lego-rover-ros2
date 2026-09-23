"""Shared proximity shaping for rover explore — sonar pan buckets + side ToF.

Chassis defaults (Cain bench, Aug 2026):
  pan sonar → **rear bumper** ~9 cm (bracket rotated 180°), side ToF → body side ~4 cm.
  Nose depth: VL53L8CX (not this sonar).

Motion signs (do not “fix” these independently):
  +angular.z = left, −angular.z = right  (ESP: l = v − turn, r = v + turn)
  Pan: 0° = right, 90° = **aft**, 180° = left
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field


def _f(name: str, default: float) -> float:
    raw = os.environ.get(name, '').strip()
    if not raw:
        return default
    return float(raw)


@dataclass
class ProximityConfig:
    sonar_to_bumper_m: float = 0.09
    tof_to_side_m: float = 0.04
    pan_center: float = 90.0
    pan_use_deg: float = 18.0
    stop_bumper_m: float = 0.15
    slow_bumper_m: float = 0.28
    spin_stop_bumper_m: float = 0.20
    side_stop_m: float = 0.12
    side_slow_m: float = 0.32
    # Peel before the body is already on the skirting.
    side_comfort_m: float = 0.45
    # Both side ToFs closer than this → treat as a corridor and center.
    side_corridor_max_m: float = 1.40
    # Nose L8 is a 4 m camera, not a bumper. Slow/turn long before ESP's 0.24 m brake.
    l8_slow_m: float = 0.85
    l8_turn_m: float = 0.55
    l8_stop_m: float = 0.34
    close_any_sonar_m: float = 0.20
    range_stale_s: float = 1.2
    tof_stale_s: float = 1.0
    side_steer_gain: float = 0.85
    tof_min_valid_m: float = 0.03
    tof_max_valid_m: float = 2.5
    sonar_faces_rear: bool = True

    @classmethod
    def from_env(cls) -> ProximityConfig:
        sonar_off = _f('ROVER_SONAR_TO_BUMPER_M', 0.09)
        tof_off = _f('ROVER_TOF_TO_SIDE_M', 0.04)
        return cls(
            sonar_to_bumper_m=sonar_off,
            tof_to_side_m=tof_off,
            pan_center=_f('PAN_CENTER', 90.0),
            pan_use_deg=_f('ROVER_PAN_USE_DEG', 18.0),
            stop_bumper_m=_f('ROVER_STOP_BUMPER_M', 0.15),
            slow_bumper_m=_f('ROVER_SLOW_BUMPER_M', 0.28),
            spin_stop_bumper_m=_f('ROVER_SPIN_STOP_BUMPER_M', 0.20),
            side_stop_m=_f('ROVER_SIDE_STOP_M', 0.12),
            side_slow_m=_f('ROVER_SIDE_SLOW_M', 0.32),
            side_comfort_m=_f('ROVER_SIDE_COMFORT_M', 0.45),
            side_corridor_max_m=_f('ROVER_SIDE_CORRIDOR_MAX_M', 1.40),
            l8_slow_m=_f('ROVER_L8_SLOW_M', 0.85),
            l8_turn_m=_f('ROVER_L8_TURN_M', 0.55),
            l8_stop_m=_f('ROVER_L8_STOP_M', 0.34),
            close_any_sonar_m=_f('ROVER_CLOSE_ANY_SONAR_M', 0.20),
            range_stale_s=_f('ROVER_RANGE_STALE_S', 1.2),
            tof_stale_s=_f('ROVER_TOF_STALE_S', 1.0),
            side_steer_gain=_f('ROVER_SIDE_STEER_GAIN', 0.85),
            tof_min_valid_m=_f('ROVER_TOF_MIN_VALID_M', 0.03),
            tof_max_valid_m=_f('ROVER_TOF_MAX_VALID_M', 2.5),
            sonar_faces_rear=os.environ.get('ROVER_SONAR_REAR', '1').strip().lower()
            not in ('0', 'no', 'false'),
        )

    def sonar_stop_m(self) -> float:
        return self.stop_bumper_m + self.sonar_to_bumper_m

    def sonar_slow_m(self) -> float:
        return self.slow_bumper_m + self.sonar_to_bumper_m

    def sonar_spin_stop_m(self) -> float:
        return self.spin_stop_bumper_m + self.sonar_to_bumper_m

    def tof_stop_m(self) -> float:
        return self.side_stop_m + self.tof_to_side_m

    def tof_slow_m(self) -> float:
        return self.side_slow_m + self.tof_to_side_m


@dataclass
class ProximityState:
    cfg: ProximityConfig = field(default_factory=ProximityConfig.from_env)
    _pan_deg: float = 90.0
    _range_m: float = field(default_factory=lambda: float('nan'))
    _last_range_ms: float = 0.0
    _pan_bucket: dict[str, tuple[float, float]] = field(default_factory=dict)
    _tof_l: float = field(default_factory=lambda: float('nan'))
    _tof_r: float = field(default_factory=lambda: float('nan'))
    _tof_l_ms: float = 0.0
    _tof_r_ms: float = 0.0
    _blocked_since: float | None = None
    _side_since: float | None = None
    _steer_hold_sign: float = 0.0
    _steer_hold_until: float = 0.0
    _front_m: float = field(default_factory=lambda: float('nan'))
    _front_l: float = field(default_factory=lambda: float('nan'))
    _front_r: float = field(default_factory=lambda: float('nan'))
    _front_ms: float = 0.0
    _tof_l_prev: tuple[float, float] | None = None
    _tof_r_prev: tuple[float, float] | None = None

    def update_pan(self, pan_deg: float) -> None:
        self._pan_deg = float(pan_deg)

    def update_range(self, range_m: float) -> None:
        now = time.monotonic()
        if math.isnan(range_m) or range_m <= 0.0:
            self._range_m = float('nan')
            self._last_range_ms = now
            return
        self._range_m = range_m
        self._last_range_ms = now
        bucket = self._pan_bucket_name(self._pan_deg)
        if bucket:
            self._pan_bucket[bucket] = (range_m, now)

    def update_tof_left(self, range_m: float) -> None:
        now = time.monotonic()
        if (
            math.isnan(range_m)
            or range_m <= 0.0
            or range_m < self.cfg.tof_min_valid_m
            or range_m > self.cfg.tof_max_valid_m
        ):
            self._tof_l = float('nan')
        else:
            if not math.isnan(self._tof_l):
                self._tof_l_prev = (self._tof_l_ms, self._tof_l)
            self._tof_l = range_m
        self._tof_l_ms = now

    def update_tof_right(self, range_m: float) -> None:
        now = time.monotonic()
        if (
            math.isnan(range_m)
            or range_m <= 0.0
            or range_m < self.cfg.tof_min_valid_m
            or range_m > self.cfg.tof_max_valid_m
        ):
            self._tof_r = float('nan')
        else:
            if not math.isnan(self._tof_r):
                self._tof_r_prev = (self._tof_r_ms, self._tof_r)
            self._tof_r = range_m
        self._tof_r_ms = now

    def update_front_tof(self, center_m: float, left_m: float = float('nan'),
                         right_m: float = float('nan')) -> None:
        now = time.monotonic()
        self._front_ms = now

        def _ok(v: float) -> float:
            if math.isnan(v) or v <= 0.0 or v > 4.0:
                return float('nan')
            return v

        self._front_m = _ok(center_m)
        if not math.isnan(left_m):
            self._front_l = _ok(left_m)
        if not math.isnan(right_m):
            self._front_r = _ok(right_m)

    def _front_fresh(self, max_age_s: float = 0.5) -> bool:
        return (time.monotonic() - self._front_ms) <= max_age_s

    def _front_half(self, side: str) -> float:
        if not self._front_fresh(0.6):
            return float('nan')
        v = self._front_l if side == 'left' else self._front_r
        if math.isnan(v) or v <= 0.02:
            return float('nan')
        return v

    def front_min_m(self) -> float:
        """Closest L8 zone (center or either half) — shallow walls live in a half."""
        if not self._front_fresh():
            return float('nan')
        vals = [v for v in (self._front_m, self._front_l, self._front_r)
                if not math.isnan(v) and v > 0.02]
        return min(vals) if vals else float('nan')

    def front_stop_m(self) -> float:
        return self.front_min_m()

    def front_open(self, min_m: float | None = None) -> bool:
        fwd = self.front_min_m()
        if math.isnan(fwd):
            return False
        return fwd > (min_m if min_m is not None else self.cfg.l8_slow_m)

    def _pan_bucket_name(self, pan_deg: float) -> str | None:
        err = pan_deg - self.cfg.pan_center
        if abs(err) <= self.cfg.pan_use_deg * 0.45:
            return 'center'
        # pan 0 = right, pan 180 = left
        if err < 0.0:
            return 'right'
        return 'left'

    def _bucket_range(self, name: str, max_age_s: float | None = None) -> float:
        sample = self._pan_bucket.get(name)
        if not sample:
            return float('nan')
        rng, ts = sample
        age_lim = max_age_s if max_age_s is not None else self.cfg.range_stale_s
        if time.monotonic() - ts > age_lim:
            return float('nan')
        return rng

    def forward_stop_m(self) -> float:
        """Conservative range for stop/trap — center cone + live reading only."""
        now = time.monotonic()
        vals: list[float] = []
        center = self._bucket_range('center', max_age_s=0.7)
        if not math.isnan(center):
            vals.append(center)
        if not math.isnan(self._range_m) and now - self._last_range_ms <= 0.45:
            err = abs(self._pan_deg - self.cfg.pan_center)
            if err <= self.cfg.pan_use_deg * 0.55:
                vals.append(self._range_m)
        if not vals:
            return float('nan')
        return min(vals)

    def effective_forward_m(self) -> float:
        now = time.monotonic()
        vals: list[float] = []
        if not math.isnan(self._range_m) and now - self._last_range_ms <= self.cfg.range_stale_s:
            vals.append(self._range_m)
        for name in ('center', 'left', 'right'):
            br = self._bucket_range(name, max_age_s=2.5)
            if not math.isnan(br):
                vals.append(br)
        if not vals:
            return float('nan')
        return min(vals)

    def is_forward_blocked(self) -> bool:
        fwd = self.front_min_m()
        if math.isnan(fwd):
            return False
        return fwd < self.cfg.l8_turn_m

    def is_nose_jammed(self) -> bool:
        fwd = self.front_min_m()
        if math.isnan(fwd):
            return False
        return fwd < self.cfg.l8_stop_m

    def wants_early_turn(self) -> bool:
        """Couch-range: L8 sees it — peel, do not reverse-jog then retry."""
        fwd = self.front_min_m()
        if math.isnan(fwd):
            return False
        return fwd < self.cfg.l8_turn_m

    def is_rear_blocked(self) -> bool:
        if not self.cfg.sonar_faces_rear:
            return False
        aft = self.forward_stop_m()
        if math.isnan(aft):
            return False
        return aft < self.cfg.sonar_stop_m()

    def is_side_occupied(self) -> bool:
        """Left or right ToF closer than slow — wall along the body, even if sonar is long."""
        tl = self._side_clear('left')
        tr = self._side_clear('right')
        if not math.isnan(tl) and tl < self.cfg.tof_slow_m():
            return True
        if not math.isnan(tr) and tr < self.cfg.tof_slow_m():
            return True
        return False

    def note_drive_intent(self, want_forward: bool) -> None:
        """Call each tick with cruise intent before apply()."""
        now = time.monotonic()
        if want_forward and self.is_forward_blocked():
            if self._blocked_since is None:
                self._blocked_since = now
        else:
            self._blocked_since = None
        if want_forward and self.is_side_occupied():
            if self._side_since is None:
                self._side_since = now
        else:
            self._side_since = None

    def persistently_blocked(self, now: float, min_s: float) -> bool:
        if self._blocked_since is None:
            return False
        return (now - self._blocked_since) >= min_s

    def persistently_side_pinched(self, now: float, min_s: float) -> bool:
        if self._side_since is None:
            return False
        return (now - self._side_since) >= min_s

    def clear_blocked(self) -> None:
        self._blocked_since = None
        self._side_since = None

    def open_steer(self, max_steer: float) -> float:
        now = time.monotonic()
        if now < self._steer_hold_until and self._steer_hold_sign != 0.0:
            return self._steer_hold_sign * max_steer
        steer = self._steer_toward_open(max_steer)
        if abs(steer) > 0.02:
            self._steer_hold_sign = 1.0 if steer > 0 else -1.0
            self._steer_hold_until = now + 1.2
        return steer

    def commit_steer_away(self, sign: float, hold_s: float = 6.0) -> None:
        if sign == 0.0:
            return
        self._steer_hold_sign = 1.0 if sign > 0 else -1.0
        self._steer_hold_until = time.monotonic() + hold_s

    def steer_commit_active(self, now: float) -> float:
        if now < self._steer_hold_until and self._steer_hold_sign != 0.0:
            return self._steer_hold_sign
        return 0.0

    def _side_clear(self, side: str) -> float:
        now = time.monotonic()
        if side == 'left':
            if math.isnan(self._tof_l) or now - self._tof_l_ms > self.cfg.tof_stale_s:
                return float('nan')
            return self._tof_l
        if math.isnan(self._tof_r) or now - self._tof_r_ms > self.cfg.tof_stale_s:
            return float('nan')
        return self._tof_r

    def _half_clear(self, side: str) -> float:
        """Clearance on a chassis side for *forward* steering.

        Rear pan buckets are not a path — they describe the tail.
        """
        vals: list[float] = []
        t = self._side_clear(side)
        if not math.isnan(t):
            vals.append(t)
        if self._front_fresh(0.6):
            v = self._front_l if side == 'left' else self._front_r
            if not math.isnan(v) and v > 0.02:
                vals.append(v)
        if not self.cfg.sonar_faces_rear:
            b = self._bucket_range(side, max_age_s=2.5)
            if not math.isnan(b):
                vals.append(b)
        return min(vals) if vals else float('nan')

    def _steer_toward_open(self, max_steer: float) -> float:
        tl = self._half_clear('left')
        tr = self._half_clear('right')
        score_l = 0.0
        score_r = 0.0
        if not math.isnan(tl):
            score_l += tl
        if not math.isnan(tr):
            score_r += tr
        if score_l <= 0.0 and score_r <= 0.0:
            return 0.0
        if score_r > score_l + 0.04:
            return -max_steer  # more space on the right → turn right
        if score_l > score_r + 0.04:
            return max_steer  # more space on the left → turn left
        # Tie-break: never random. Turn away from whichever side is closer.
        close = False
        fwd = self.front_min_m()
        if not math.isnan(fwd) and fwd < self.cfg.l8_slow_m:
            close = True
        if not math.isnan(tl) and tl < self.cfg.tof_slow_m():
            close = True
        if not math.isnan(tr) and tr < self.cfg.tof_slow_m():
            close = True
        if close:
            return self.turn_away_from_closest() * max_steer
        return 0.0

    def turn_away_from_closest(self) -> float:
        """+1 = turn left, −1 = turn right, away from the nearest side reading."""
        left_m = self._half_clear('left')
        right_m = self._half_clear('right')
        if math.isnan(left_m) and math.isnan(right_m):
            return 1.0
        if math.isnan(left_m):
            return 1.0  # only know the right — turn left
        if math.isnan(right_m):
            return -1.0
        if left_m < right_m - 0.03:
            return -1.0  # left closer → turn right
        if right_m < left_m - 0.03:
            return 1.0
        if self._steer_hold_sign != 0.0:
            return self._steer_hold_sign
        return 1.0

    def live_opening_sign(self) -> tuple[float, bool]:
        """Pick a peel side from nose L8 + side ToF. Rear sonar is not an opening.

        Returns (sign, confident). +1 = left, −1 = right.
        """
        tl = self._side_clear('left')
        tr = self._side_clear('right')
        cfg = self.cfg
        left_m = self._half_clear('left')
        right_m = self._half_clear('right')
        sign = 0.0
        confident = False
        if math.isnan(left_m) and math.isnan(right_m):
            return 0.0, False
        if math.isnan(left_m):
            if right_m <= cfg.tof_slow_m():
                return 1.0, True  # right pinched → peel left
            return 0.0, False
        if math.isnan(right_m):
            if left_m <= cfg.tof_slow_m():
                return -1.0, True
            return 0.0, False
        if right_m > left_m + 0.06:
            sign, confident = -1.0, True
        elif left_m > right_m + 0.06:
            sign, confident = 1.0, True
        else:
            left_pinched = not math.isnan(tl) and tl < cfg.tof_slow_m()
            right_pinched = not math.isnan(tr) and tr < cfg.tof_slow_m()
            if left_pinched and not right_pinched:
                sign, confident = -1.0, True
            elif right_pinched and not left_pinched:
                sign, confident = 1.0, True
            else:
                return 0.0, False
        if sign > 0 and not math.isnan(tl) and tl < cfg.tof_slow_m():
            sign = -1.0
        elif sign < 0 and not math.isnan(tr) and tr < cfg.tof_slow_m():
            sign = 1.0
        return sign, confident

    def opening_sign(self, best_deg: float, best_m: float) -> float:
        """Choose a recover/cruise turn from a pan sweep, else ToF."""
        if self.cfg.sonar_faces_rear:
            sign = self.turn_away_from_closest()
            tl = self._side_clear('left')
            tr = self._side_clear('right')
            if sign > 0 and not math.isnan(tl) and tl < self.cfg.tof_slow_m():
                return -1.0
            if sign < 0 and not math.isnan(tr) and tr < self.cfg.tof_slow_m():
                return 1.0
            return sign
        fwd = self.forward_stop_m()
        useful = best_m > 0.40 and (math.isnan(fwd) or best_m > fwd + 0.08)
        if useful:
            err = best_deg - self.cfg.pan_center
            if err > 8.0:
                sign = 1.0  # pan toward 180° = left opening
            elif err < -8.0:
                sign = -1.0  # pan toward 0° = right opening
            else:
                sign = self.turn_away_from_closest()
        else:
            sign = self.turn_away_from_closest()
        # Never "open" into a ToF-pinched side (scan along a wall looks like range).
        tl = self._side_clear('left')
        tr = self._side_clear('right')
        if sign > 0 and not math.isnan(tl) and tl < self.cfg.tof_slow_m():
            return -1.0
        if sign < 0 and not math.isnan(tr) and tr < self.cfg.tof_slow_m():
            return 1.0
        return sign

    def _steer_from_pan_bias(self, max_steer: float) -> float:
        """When glance buckets are empty, steer away from where pan is pointing."""
        err = self._pan_deg - self.cfg.pan_center
        if abs(err) < 6.0:
            return 0.0
        # Pan looking right (0°) → turn left (+) away from that side.
        return max_steer if err < 0.0 else -max_steer

    def cruise_steer_bias(self, max_steer: float) -> float:
        """Gentle steer-away while cruising — does not zero forward."""
        fwd = self.front_min_m()
        if math.isnan(fwd) or fwd > self.cfg.l8_slow_m:
            return 0.0
        steer = self.open_steer(max_steer * self.cfg.side_steer_gain)
        if abs(steer) < 0.02:
            if self.cfg.sonar_faces_rear:
                return 0.0
            # Pan wiggle crosses center often; if we use raw pan bias we can dither.
            # If we *must* fall back to pan, hold the chosen sign briefly.
            pan = self._steer_from_pan_bias(max_steer * 0.35)
            if abs(pan) >= 0.02:
                self.commit_steer_away(1.0 if pan > 0.0 else -1.0, hold_s=0.9)
            steer = pan
        return steer

    def _side_closing(self, side: str) -> float:
        """Positive = approaching that wall (m/s). 0 if unknown."""
        now = time.monotonic()
        if side == 'left':
            cur, prev = self._tof_l, self._tof_l_prev
        else:
            cur, prev = self._tof_r, self._tof_r_prev
        if prev is None or math.isnan(cur):
            return 0.0
        dt = now - prev[0]
        if dt < 0.08 or dt > 0.8:
            return 0.0
        return max(0.0, -(cur - prev[1]) / dt)

    def apply(self, lin: float, ang: float, *, max_steer: float) -> tuple[float, float]:
        """Slow and turn from L8 + side ToF before the ESP bumper brake."""
        cfg = self.cfg
        fwd = self.front_min_m()
        aft = self.forward_stop_m()
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        fl = self._front_half('left')
        fr = self._front_half('right')

        if lin > 0.02 and not math.isnan(fwd) and fwd < cfg.l8_stop_m:
            return -min(0.10, max(0.06, abs(lin) * 0.55)), 0.0

        if lin < -0.02 and self.cfg.sonar_faces_rear and not math.isnan(aft):
            bumper_clear = aft - cfg.sonar_to_bumper_m
            if bumper_clear < cfg.stop_bumper_m:
                return 0.0, ang

        # Side ToF: corridor-center when both walls visible; otherwise peel
        # from a closing wall (shallow angle included via L8 halves below).
        comfort = cfg.side_comfort_m
        stop_s = cfg.tof_stop_m()
        corridor = (
            not math.isnan(sl)
            and not math.isnan(sr)
            and sl < cfg.side_corridor_max_m
            and sr < cfg.side_corridor_max_m
        )
        steer = 0.0
        if lin > 0.02:
            if corridor:
                steer = -1.8 * (sr - sl)
            else:
                left_err = 0.0
                right_err = 0.0
                if not math.isnan(sl) and sl < comfort:
                    left_err = (comfort - sl) / max(0.04, comfort - stop_s)
                if not math.isnan(sr) and sr < comfort:
                    right_err = (comfort - sr) / max(0.04, comfort - stop_s)
                left_err += min(1.0, self._side_closing('left') / 0.20)
                right_err += min(1.0, self._side_closing('right') / 0.20)
                left_err = max(0.0, min(1.4, left_err))
                right_err = max(0.0, min(1.4, right_err))
                steer = (right_err - left_err) * max_steer

            if not math.isnan(fl) and not math.isnan(fr):
                closer = min(fl, fr)
                if closer < cfg.l8_slow_m and abs(fl - fr) > 0.08:
                    l8_w = (cfg.l8_slow_m - closer) / max(0.08, cfg.l8_slow_m - cfg.l8_stop_m)
                    l8_w = max(0.15, min(1.0, l8_w))
                    steer += (-1.0 if fl < fr else 1.0) * max_steer * l8_w
            elif not math.isnan(fl) and fl < cfg.l8_slow_m:
                steer -= max_steer * 0.55
            elif not math.isnan(fr) and fr < cfg.l8_slow_m:
                steer += max_steer * 0.55

            if abs(steer) >= 0.02:
                ang = max(-max_steer, min(max_steer, steer))
                if not corridor:
                    pinch = 0.0
                    if not math.isnan(sl):
                        pinch = max(pinch, max(0.0, comfort - sl))
                    if not math.isnan(sr):
                        pinch = max(pinch, max(0.0, comfort - sr))
                    if pinch > 0.12:
                        lin = min(lin, lin * 0.50)
                    elif pinch > 0.04:
                        lin = min(lin, lin * 0.75)

        if lin > 0.02 and not math.isnan(fwd) and fwd < cfg.l8_slow_m:
            scale = (fwd - cfg.l8_stop_m) / max(0.08, cfg.l8_slow_m - cfg.l8_stop_m)
            lin = min(lin, lin * max(0.30, min(1.0, scale)))
            if abs(steer) < 0.02 and fwd < cfg.l8_turn_m:
                ang = self.turn_away_from_closest() * max_steer

        if lin > 0.02:
            if ang > 0.02 and not math.isnan(sl) and sl < cfg.tof_slow_m():
                frac = (cfg.tof_slow_m() - sl) / max(0.05, cfg.tof_slow_m() - cfg.tof_stop_m())
                ang -= max_steer * cfg.side_steer_gain * max(0.15, min(1.0, frac))
                lin *= 0.90
            elif ang < -0.02 and not math.isnan(sr) and sr < cfg.tof_slow_m():
                frac = (cfg.tof_slow_m() - sr) / max(0.05, cfg.tof_slow_m() - cfg.tof_stop_m())
                ang += max_steer * cfg.side_steer_gain * max(0.15, min(1.0, frac))
                lin *= 0.90

        ang = max(-max_steer, min(max_steer, ang))
        return lin, ang
