"""Shared proximity shaping for rover explore — sonar pan buckets + side ToF.

Chassis defaults (Cain bench, Aug 2026):
  sonar → front bumper 9 cm, side ToF → body side ~4 cm.
"""
from __future__ import annotations

import math
import os
import random
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
    side_stop_m: float = 0.10
    side_slow_m: float = 0.18
    close_any_sonar_m: float = 0.20
    range_stale_s: float = 1.2
    tof_stale_s: float = 1.0
    side_steer_gain: float = 0.55
    tof_min_valid_m: float = 0.03
    tof_max_valid_m: float = 2.5

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
            side_stop_m=_f('ROVER_SIDE_STOP_M', 0.10),
            side_slow_m=_f('ROVER_SIDE_SLOW_M', 0.18),
            close_any_sonar_m=_f('ROVER_CLOSE_ANY_SONAR_M', 0.20),
            range_stale_s=_f('ROVER_RANGE_STALE_S', 1.2),
            tof_stale_s=_f('ROVER_TOF_STALE_S', 1.0),
            side_steer_gain=_f('ROVER_SIDE_STEER_GAIN', 0.55),
            tof_min_valid_m=_f('ROVER_TOF_MIN_VALID_M', 0.03),
            tof_max_valid_m=_f('ROVER_TOF_MAX_VALID_M', 2.5),
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
    _steer_hold_sign: float = 0.0
    _steer_hold_until: float = 0.0

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
            self._tof_r = range_m
        self._tof_r_ms = now

    def _pan_bucket_name(self, pan_deg: float) -> str | None:
        err = pan_deg - self.cfg.pan_center
        if abs(err) <= self.cfg.pan_use_deg * 0.45:
            return 'center'
        if err < 0.0:
            return 'left'
        return 'right'

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
            return self.effective_forward_m()
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
        fwd = self.forward_stop_m()
        if math.isnan(fwd):
            return False
        return fwd < self.cfg.sonar_stop_m()

    def note_drive_intent(self, want_forward: bool) -> None:
        """Call each tick with cruise intent before apply()."""
        now = time.monotonic()
        if want_forward and self.is_forward_blocked():
            if self._blocked_since is None:
                self._blocked_since = now
        else:
            self._blocked_since = None

    def persistently_blocked(self, now: float, min_s: float) -> bool:
        if self._blocked_since is None:
            return False
        return (now - self._blocked_since) >= min_s

    def clear_blocked(self) -> None:
        self._blocked_since = None

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

    def _steer_toward_open(self, max_steer: float) -> float:
        left = self._bucket_range('left', max_age_s=2.5)
        right = self._bucket_range('right', max_age_s=2.5)
        tl = self._side_clear('left')
        tr = self._side_clear('right')
        score_l = 0.0
        score_r = 0.0
        if not math.isnan(left):
            score_l += left
        if not math.isnan(tl):
            score_l += tl
        if not math.isnan(right):
            score_r += right
        if not math.isnan(tr):
            score_r += tr
        if score_l <= 0.0 and score_r <= 0.0:
            return 0.0
        if score_r > score_l + 0.04:
            return max_steer
        if score_l > score_r + 0.04:
            return -max_steer
        # Tie-break: if we're close to *something*, pick a side and hold.
        # This helps U-shaped "ping-pong" corners where both sides read similar.
        close = False
        fwd = self.forward_stop_m()
        if not math.isnan(fwd) and fwd < self.cfg.sonar_slow_m():
            close = True
        if not math.isnan(tl) and tl < self.cfg.tof_slow_m():
            close = True
        if not math.isnan(tr) and tr < self.cfg.tof_slow_m():
            close = True
        if close:
            if self._steer_hold_sign != 0.0 and time.monotonic() < self._steer_hold_until:
                return self._steer_hold_sign * max_steer
            return random.choice([-1.0, 1.0]) * max_steer
        return 0.0

    def _steer_from_pan_bias(self, max_steer: float) -> float:
        """When glance buckets are empty, steer away from where pan is pointing."""
        err = self._pan_deg - self.cfg.pan_center
        if abs(err) < 6.0:
            return 0.0
        # Pan right → obstacle on right → steer left (negative angular).
        return -max_steer if err > 0.0 else max_steer

    def cruise_steer_bias(self, max_steer: float) -> float:
        """Gentle steer-away while cruising — does not zero forward."""
        # Use a forward/center-biased clearance so stale side glances don't
        # constantly "think" we're about to hit something.
        fwd = self.forward_stop_m()
        if math.isnan(fwd) or fwd > self.cfg.sonar_slow_m():
            return 0.0
        steer = self.open_steer(max_steer * self.cfg.side_steer_gain)
        if abs(steer) < 0.02:
            # Pan wiggle crosses center often; if we use raw pan bias we can dither.
            # If we *must* fall back to pan, hold the chosen sign briefly.
            pan = self._steer_from_pan_bias(max_steer * 0.35)
            if abs(pan) >= 0.02:
                self.commit_steer_away(1.0 if pan > 0.0 else -1.0, hold_s=0.9)
            steer = pan
        return steer

    def apply(self, lin: float, ang: float, *, max_steer: float) -> tuple[float, float]:
        """Tier-1 shaping only — slow + steer. ESP hard-brakes forward at stop range."""
        cfg = self.cfg
        # Speed shaping should be based on the forward cone (live/center).
        # We still *steer* using left/right buckets + side ToF, but we shouldn't
        # slow down just because a stale side glance saw a nearby wall.
        fwd = self.forward_stop_m()
        tl = self._side_clear('left')
        tr = self._side_clear('right')
        # Fresh side glances help avoid angled bumper hits without using stale buckets.
        gl = self._bucket_range('left', max_age_s=0.7)
        gr = self._bucket_range('right', max_age_s=0.7)

        # If we're *very* close in front, do not spin-in-place into the object.
        # Reverse a touch (still smooth) and arc away using committed open steer.
        if lin > 0.02 and not math.isnan(fwd):
            bumper_clear = fwd - cfg.sonar_to_bumper_m
            if bumper_clear < cfg.stop_bumper_m:
                steer = self.open_steer(max_steer * cfg.side_steer_gain)
                if abs(steer) < 0.02:
                    steer = self._steer_from_pan_bias(max_steer * 0.35)
                ang = max(-max_steer, min(max_steer, ang + steer))
                return -min(0.10, max(0.06, lin * 0.55)), ang

            if fwd < cfg.sonar_slow_m():
                scale = (fwd - cfg.sonar_stop_m()) / max(
                    0.05, cfg.sonar_slow_m() - cfg.sonar_stop_m()
                )
                lin = min(lin, lin * max(0.35, min(1.0, scale)))
            steer_open = self.open_steer(max_steer * cfg.side_steer_gain * 0.5)
            if abs(steer_open) < 0.02:
                pan = self._steer_from_pan_bias(max_steer * 0.25)
                if abs(pan) >= 0.02:
                    self.commit_steer_away(1.0 if pan > 0.0 else -1.0, hold_s=0.9)
                steer_open = pan
            ang += steer_open

        side_steer = 0.0
        if lin > 0.02:
            # Side ToF: prevent "clipping the wall at an angle".
            # Apply strongly when we're turning *toward* the close side; otherwise only
            # intervene at the hard-stop distance.
            if not math.isnan(tl) and tl < cfg.tof_stop_m():
                # Left wall close → steer right (positive).
                side_steer += max_steer * cfg.side_steer_gain
                lin *= 0.55
            if not math.isnan(tr) and tr < cfg.tof_stop_m():
                # Right wall close → steer left (negative).
                side_steer -= max_steer * cfg.side_steer_gain
                lin *= 0.55

            # Sign convention here matches `_steer_from_pan_bias()`:
            #   positive ang = steer right, negative ang = steer left.
            if ang < -0.02 and not math.isnan(tl) and tl < cfg.tof_slow_m():
                frac = (cfg.tof_slow_m() - tl) / max(0.05, cfg.tof_slow_m() - cfg.tof_stop_m())
                side_steer += max_steer * cfg.side_steer_gain * max(0.15, min(1.0, frac))
                lin *= 0.85
            elif ang > 0.02 and not math.isnan(tr) and tr < cfg.tof_slow_m():
                frac = (cfg.tof_slow_m() - tr) / max(0.05, cfg.tof_slow_m() - cfg.tof_stop_m())
                side_steer -= max_steer * cfg.side_steer_gain * max(0.15, min(1.0, frac))
                lin *= 0.85

            # If we're turning into a side where the sonar glance is already close,
            # push away early (prevents "drive at wall on an angle → bump").
            if ang < -0.02 and not math.isnan(gl) and gl < cfg.sonar_slow_m():
                frac = (cfg.sonar_slow_m() - gl) / max(0.06, cfg.sonar_slow_m() - cfg.sonar_stop_m())
                side_steer += max_steer * 0.45 * max(0.15, min(1.0, frac))
                lin *= 0.92
            elif ang > 0.02 and not math.isnan(gr) and gr < cfg.sonar_slow_m():
                frac = (cfg.sonar_slow_m() - gr) / max(0.06, cfg.sonar_slow_m() - cfg.sonar_stop_m())
                side_steer -= max_steer * 0.45 * max(0.15, min(1.0, frac))
                lin *= 0.92

        ang = max(-max_steer, min(max_steer, ang + side_steer))
        return lin, ang
