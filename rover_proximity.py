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
    tof_min_valid_m: float = 0.10
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
            tof_min_valid_m=_f('ROVER_TOF_MIN_VALID_M', 0.10),
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
    _l8_cols: list[float] = field(default_factory=lambda: [float('nan')] * 8)
    _l8_cols_ms: float = 0.0
    _corr_filt: float = 0.0
    _corridor_seen: float = 0.0

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

    def update_l8_cols(self, cols: list[float]) -> None:
        now = time.monotonic()
        out = [float('nan')] * 8
        for i, raw in enumerate(cols[:8]):
            try:
                v = float(raw)
            except (TypeError, ValueError):
                continue
            if math.isfinite(v) and 0.02 < v < 4.0:
                out[i] = v
        self._l8_cols = out
        self._l8_cols_ms = now

    def _front_fresh(self, max_age_s: float = 0.5) -> bool:
        return (time.monotonic() - self._front_ms) <= max_age_s

    def _front_half(self, side: str) -> float:
        if self._l8_cols_ms > 0.0 and (time.monotonic() - self._l8_cols_ms) <= 0.6:
            idxs = (0, 1, 2) if side == 'left' else (5, 6, 7)
            vals = [self._l8_cols[i] for i in idxs if not math.isnan(self._l8_cols[i])]
            if vals:
                return min(vals)
        if not self._front_fresh(0.6):
            return float('nan')
        v = self._front_l if side == 'left' else self._front_r
        if math.isnan(v) or v <= 0.02:
            return float('nan')
        return v

    def front_center_m(self) -> float:
        """Inner 4×4 — used for stop/reverse. Halves are for steer only."""
        if not self._front_fresh():
            return float('nan')
        return self._front_m

    def front_min_m(self) -> float:
        """Closest *usable* L8 reading. Ignores floor-looking half cells."""
        if not self._front_fresh():
            return float('nan')
        vals = [v for v in (self._front_m, self._front_half('left'), self._front_half('right'))
                if not math.isnan(v) and v > 0.02]
        return min(vals) if vals else float('nan')

    def front_stop_m(self) -> float:
        return self.front_center_m()

    def front_open(self, min_m: float | None = None) -> bool:
        fwd = self.front_center_m()
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

    def along_wall(self) -> bool:
        """One or both body ToFs see a wall. Hallway, not open room."""
        for side in ('left', 'right'):
            v = self._side_clear(side)
            if not math.isnan(v) and v < 1.05:
                return True
        return False

    def in_corridor(self) -> bool:
        """Both hips see walls. Latch briefly so one VL53 dropout does not flap."""
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        both = (
            not math.isnan(sl)
            and not math.isnan(sr)
            and sl < self.cfg.side_corridor_max_m
            and sr < self.cfg.side_corridor_max_m
        )
        now = time.monotonic()
        if both:
            self._corridor_seen = now
            return True
        if self._corridor_seen > 0.0 and (now - self._corridor_seen) < 3.0:
            return True
        return False

    def wall_dead_end(self) -> bool:
        """Really boxed in: nose close and both hips tight."""
        fwd = self.front_center_m()
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        nose = not math.isnan(fwd) and fwd < 0.22
        hips = (
            not math.isnan(sl)
            and not math.isnan(sr)
            and sl < 0.38
            and sr < 0.38
        )
        return nose and hips

    def must_pivot(self) -> bool:
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        vals = [v for v in (sl, sr) if not math.isnan(v)]
        if not vals:
            return False
        return min(vals) < 0.28

    def repel_steer(self, max_steer: float) -> float:
        """Always away from the closer hip. Never toward a wall."""
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        if not math.isnan(sl) and not math.isnan(sr):
            return self.corridor_steer(max_steer)
        return self.turn_away_from_closest() * max_steer

    def repel_overlay(self, lin: float, ang: float, *, max_steer: float) -> tuple[float, float]:
        """Last word: do not drive into a wall. Pivot instead of creeping."""
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        vals = [v for v in (sl, sr) if not math.isnan(v)]
        if not vals:
            return lin, ang
        closer = min(vals)
        if closer < 0.28:
            # lin=0 makes ESP ignore turn. Reverse+nudge actually moves.
            return -0.10, self.turn_away_from_closest() * 0.05
        if self.in_corridor():
            return lin, ang
        if self.along_wall() and not math.isnan(sl) and not math.isnan(sr):
            ang = self.hip_repel(0.04)
        return lin, ang

    def side_too_close(self) -> bool:
        stop = self.cfg.tof_stop_m()
        for side in ('left', 'right'):
            v = self._side_clear(side)
            if not math.isnan(v) and v < stop:
                return True
        return False

    def hip_repel(self, max_steer: float = 0.05) -> float:
        """Steer away from a close or closing hip.

        ang=0 lets ESP heading-hold trim up to 0.15 — that walked it into the
        wall with cmd 0.14,0.00. |ang| must be > 0.04 to disarm heading-hold.
        """
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        comfort = 0.50
        ang = 0.0
        both = not math.isnan(sl) and not math.isnan(sr)
        # 2–6 cm of noise while both hips are comfortable is not a turn.
        if both and min(sl, sr) > 0.42 and abs(sl - sr) < 0.08:
            ang -= 0.03 * min(1.0, self._side_closing('left') / 0.12)
            ang += 0.03 * min(1.0, self._side_closing('right') / 0.12)
        else:
            if not math.isnan(sl):
                if sl < comfort:
                    ang -= max_steer * min(1.0, (comfort - sl) / 0.28)
                ang -= 0.03 * min(1.0, self._side_closing('left') / 0.12)
            if not math.isnan(sr):
                if sr < comfort:
                    ang += max_steer * min(1.0, (comfort - sr) / 0.28)
                ang += 0.03 * min(1.0, self._side_closing('right') / 0.12)
        ang = max(-max_steer, min(max_steer, ang))
        if abs(ang) < 0.012:
            return 0.0
        # ESP heading-hold stays on below 0.04 and will fight a weaker nudge.
        if abs(ang) < 0.045:
            return math.copysign(0.045, ang)
        return ang

    def corridor_drive(self, cruise: float) -> tuple[float, float]:
        """Hallway: hold the gap. Nudge off a closing hip so heading-hold cannot walk in."""
        lin = max(0.11, min(0.14, cruise))
        ang = self.hip_repel(0.05)
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        vals = [v for v in (sl, sr) if not math.isnan(v)]
        if vals and min(vals) < 0.22:
            lin = min(lin, 0.11)
        return lin, ang

    def corridor_steer(self, max_steer: float) -> float:
        return self.hip_repel(min(0.05, max_steer))

    def l8_col_min(self) -> float:
        if (time.monotonic() - self._l8_cols_ms) > 0.6:
            return float('nan')
        vals = [v for v in self._l8_cols if not math.isnan(v)]
        return min(vals) if vals else float('nan')

    def l8_opening_sign(self) -> float:
        """+1 left, −1 right, 0 unknown. Uses 8 columns, not inner 4×4."""
        if (time.monotonic() - self._l8_cols_ms) > 0.6:
            return 0.0
        left = [v for v in self._l8_cols[:4] if not math.isnan(v)]
        right = [v for v in self._l8_cols[4:] if not math.isnan(v)]
        if not left and not right:
            return 0.0
        if not left:
            return 1.0
        if not right:
            return -1.0
        lv, rv = min(left), min(right)
        if rv > lv + 0.12:
            return -1.0
        if lv > rv + 0.12:
            return 1.0
        return 0.0

    def path_clear_to_drive(self) -> bool:
        """Open space ahead — empty L8 is NOT open (no target ≠ a path)."""
        fwd = self.front_center_m()
        if math.isnan(fwd) or fwd < 0.90:
            return False
        col = self.l8_col_min()
        if not math.isnan(col) and col < 0.50:
            return False
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        hips = [v for v in (sl, sr) if not math.isnan(v)]
        if hips and min(hips) < 0.38:
            return False
        return True

    def l8_gap_steer(self, max_steer: float) -> float:
        """Steer toward the most open mid-row column. 0 = left, 7 = right."""
        if (time.monotonic() - self._l8_cols_ms) > 0.6:
            return 0.0
        best_i = -1
        best_v = -1.0
        worst_v = 99.0
        for i, v in enumerate(self._l8_cols):
            if math.isnan(v):
                continue
            if v > best_v:
                best_v = v
                best_i = i
            if v < worst_v:
                worst_v = v
        if best_i < 0 or best_v < 0.35:
            return 0.0
        if worst_v > 1.2 or (best_v - worst_v) < 0.18:
            return 0.0
        # col 0 is left (+ang), col 7 is right (−ang).
        return max(-max_steer, min(max_steer, ((3.5 - best_i) / 3.5) * max_steer))

    def is_forward_blocked(self) -> bool:
        fwd = self.front_center_m()
        if math.isnan(fwd):
            return False
        return fwd < self.cfg.l8_turn_m

    def is_nose_jammed(self) -> bool:
        """ESP brakes forward at 0.24 m. Must reverse — a zeroed lin also kills turn."""
        fwd = self.front_center_m()
        if not math.isnan(fwd) and fwd < 0.26:
            return True
        col = self.l8_col_min()
        if not math.isnan(col) and col < 0.24:
            return True
        return False

    def wants_early_turn(self) -> bool:
        """Wall entering the nose FOV — peel before the bumper."""
        if self.along_wall() or self.in_corridor():
            col = self.l8_col_min()
            if not math.isnan(col) and col < 0.40:
                return True
            return False
        fwd = self.front_center_m()
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
            v = self._front_half(side)
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
        """Peel away from the closer hip / closer L8 half. Never toward a wall."""
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        # +ang = left. Left hip closer → turn right (−). Right closer → turn left (+).
        if not math.isnan(sl) and not math.isnan(sr):
            if sl < 0.40 or sr < 0.40:
                if sl < sr - 0.04:
                    return -1.0, True
                if sr < sl - 0.04:
                    return 1.0, True
            if sl < 0.50 or sr < 0.50:
                if sl < sr:
                    return -1.0, True
                if sr < sl:
                    return 1.0, True
        elif not math.isnan(sl) and sl < 0.45:
            return -1.0, True
        elif not math.isnan(sr) and sr < 0.45:
            return 1.0, True
        l8 = self.l8_opening_sign()
        if l8 != 0.0:
            if l8 > 0 and not math.isnan(sl) and sl < 0.32:
                return -1.0, True
            if l8 < 0 and not math.isnan(sr) and sr < 0.32:
                return 1.0, True
            return l8, True
        return 0.0, False

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
        fwd = self.front_center_m()
        aft = self.forward_stop_m()
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        corridor = self.in_corridor()
        stop_s = cfg.tof_stop_m()
        comfort = cfg.side_comfort_m

        if lin > 0.02 and not math.isnan(fwd) and fwd < cfg.l8_stop_m:
            if corridor or self.along_wall():
                # Forward brake only — caller/recover must reverse; zero lin here
                # makes the ESP drop turn as well.
                lin = min(lin, 0.0)
            else:
                return -min(0.10, max(0.08, abs(lin) * 0.55)), 0.0

        if lin < -0.02 and self.cfg.sonar_faces_rear and not math.isnan(aft):
            bumper_clear = aft - cfg.sonar_to_bumper_m
            if bumper_clear < cfg.stop_bumper_m:
                return 0.0, ang

        if self.side_too_close() and lin > 0.02:
            ang = self.hip_repel(0.04) if corridor else self.turn_away_from_closest() * 0.05
            return -0.10, ang

        steer = 0.0
        if corridor:
            steer = self.hip_repel(0.04)
            pinch = min((v for v in (sl, sr) if not math.isnan(v)), default=1.0)
            if pinch < 0.22:
                lin = min(lin, 0.11) if lin > 0 else lin
        else:
            left_err = 0.0
            right_err = 0.0
            if lin > 0.02:
                if not math.isnan(sl) and sl < comfort:
                    left_err = (comfort - sl) / max(0.04, comfort - stop_s)
                if not math.isnan(sr) and sr < comfort:
                    right_err = (comfort - sr) / max(0.04, comfort - stop_s)
                left_err += min(1.0, self._side_closing('left') / 0.20)
                right_err += min(1.0, self._side_closing('right') / 0.20)
                left_err = max(0.0, min(1.4, left_err))
                right_err = max(0.0, min(1.4, right_err))
                steer = (right_err - left_err) * max_steer
                gap = self.l8_gap_steer(max_steer)
                if abs(gap) > abs(steer):
                    steer = 0.65 * steer + 0.35 * gap
                elif abs(steer) < 0.02:
                    steer = gap
                pinch = 0.0
                if not math.isnan(sl):
                    pinch = max(pinch, max(0.0, comfort - sl))
                if not math.isnan(sr):
                    pinch = max(pinch, max(0.0, comfort - sr))
                if pinch > 0.12:
                    lin = min(lin, lin * 0.50)
                elif pinch > 0.04:
                    lin = min(lin, lin * 0.75)

        if abs(steer) >= 0.02:
            ang = max(-max_steer, min(max_steer, steer))

        if lin > 0.02 and not math.isnan(fwd) and fwd < cfg.l8_slow_m and not corridor:
            scale = (fwd - cfg.l8_stop_m) / max(0.08, cfg.l8_slow_m - cfg.l8_stop_m)
            lin = min(lin, lin * max(0.30, min(1.0, scale)))
            if abs(steer) < 0.02 and fwd < cfg.l8_turn_m:
                ang = self.turn_away_from_closest() * max_steer

        ang = max(-max_steer, min(max_steer, ang))
        return lin, ang
