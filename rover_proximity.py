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
from collections import deque
from dataclasses import dataclass, field


# Explore picture. Open floor sits near 18 cm in the nose beam. A reading
# that steady is the floor, not a wall. A real obstacle is either closer
# than that baseline, or a forward-row range that is not in the floor band.
PICTURE_FLOOR_SEED_M = 0.18
PICTURE_FLOOR_LO_M = 0.15
PICTURE_FLOOR_HI_M = 0.24
PICTURE_FLOOR_DROP_M = 0.03
PICTURE_FRONT_FAR_M = 0.75
PICTURE_FRONT_CLOSE_M = 0.38
PICTURE_SIDE_NEAR_M = 0.55
PICTURE_SIDE_CLOSE_M = 0.30
PICTURE_SIDE_VETO_M = 0.20
PICTURE_COMMIT_S = 2.5
PICTURE_COMMIT_M = 0.70
PICTURE_FRONT_STALE_S = 3.2
# Side ToFs and the L8 both publish slower than 1 Hz. A 0.6 s window
# went stale between frames and the cruise command dropped to zero.
PICTURE_SIDE_STALE_S = 2.5
PICTURE_DWELL_S = 2.0
PICTURE_OPEN_M = 2.5
# Steer-around (2026-10-14): curve toward the gap instead of stop/reverse/rotate.
PICTURE_STEER_AROUND = os.environ.get('ROVER_PICTURE_STEER_AROUND', '1') != '0'
PICTURE_BOX_M = float(os.environ.get('ROVER_PICTURE_BOX_M', '0.20'))
PICTURE_BOX_EXIT_M = float(os.environ.get('ROVER_PICTURE_BOX_EXIT_M', '0.30'))
PICTURE_GAP_HYST_M = float(os.environ.get('ROVER_PICTURE_GAP_HYST_M', '0.15'))


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
    # Both side ToFs closer than this → treat as a corridor (not wide rooms).
    side_corridor_max_m: float = 1.15
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
            side_corridor_max_m=_f('ROVER_SIDE_CORRIDOR_MAX_M', 1.15),
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
    _l8_fwd_ema: float = field(default_factory=lambda: float('nan'))
    _l8_near_streak: int = 0
    # Firmware used to publish 3.5 m as "clear" — not a real range.
    _l8_open_m: float = 3.0
    _corr_filt: float = 0.0
    _corridor_seen: float = 0.0
    _corridor_latched: bool = False
    _corridor_exit_since: float = 0.0
    _corridor_ang_filt: float = 0.0
    _floor_m: float = PICTURE_FLOOR_SEED_M
    _floor_locked: bool = False
    _floor_hist: deque = field(default_factory=lambda: deque(maxlen=8))
    _front_hist: deque = field(default_factory=deque)
    _travel_hist: deque = field(default_factory=deque)
    _picture_rank: int = 0
    _picture_ease: int = 0
    _picture_rank_since: float = 0.0
    _commit_ang: float = 0.0
    _commit_until: float = 0.0
    _commit_travel: float = 0.0


    def floor_baseline_m(self) -> float:
        return self._floor_m

    def floor_locked(self) -> bool:
        return self._floor_locked

    def reset_picture(self) -> None:
        """Clear the short-term picture. Keep a floor baseline already learned."""
        self._picture_rank = 0
        self._steer_boxed = False
        self._steer_sign = 0.0
        self._steer_sign_since = 0.0
        self._picture_ease = 0
        self._picture_rank_since = 0.0
        self._commit_ang = 0.0
        self._commit_until = 0.0
        self._commit_travel = 0.0
        self._front_hist.clear()
        self._travel_hist.clear()

    def _note_floor(self, vals: list[float]) -> None:
        band = [v for v in vals if PICTURE_FLOOR_LO_M <= v <= PICTURE_FLOOR_HI_M]
        if len(band) < 3 or (max(band) - min(band)) > 0.02:
            return
        med = sorted(band)[len(band) // 2]
        self._floor_hist.append(med)
        if len(self._floor_hist) < 6:
            return
        if max(self._floor_hist) - min(self._floor_hist) > 0.02:
            return
        meds = sorted(self._floor_hist)
        self._floor_m = meds[len(meds) // 2]
        self._floor_locked = True

    def _is_floor(self, v: float) -> bool:
        """True when this return is the floor, not something in the way."""
        if math.isnan(v) or v < self._floor_m - PICTURE_FLOOR_DROP_M:
            return False
        if self._floor_locked:
            return abs(v - self._floor_m) <= 0.025
        # Before the baseline locks, only the ~18 cm floor band is the floor.
        # 0.22 m is already something you can bump, not open carpet.
        return 0.165 <= v <= 0.205

    def _picture_front_vals(self) -> list[float]:
        now = time.monotonic()
        vals: list[float] = []
        if self._l8_cols_ms > 0.0 and (now - self._l8_cols_ms) <= PICTURE_FRONT_STALE_S:
            vals.extend(v for v in self._l8_cols if not math.isnan(v))
        if self._front_ms > 0.0 and (now - self._front_ms) <= PICTURE_FRONT_STALE_S:
            for v in (self._front_m, self._front_l, self._front_r):
                if not math.isnan(v):
                    vals.append(v)
        return vals

    def _obstacle_front_m(self) -> float:
        real = [v for v in self._picture_front_vals() if not self._is_floor(v)]
        if not real:
            return float('nan')
        return min(real)

    def _front_picture_stale(self, now: float) -> bool:
        cols_ok = self._l8_cols_ms > 0.0 and (now - self._l8_cols_ms) <= PICTURE_FRONT_STALE_S
        front_ok = self._front_ms > 0.0 and (now - self._front_ms) <= PICTURE_FRONT_STALE_S
        return not (cols_ok or front_ok)

    def _side_span(self, side: str) -> float | None:
        """Metres of side clearance. Fresh no-return is open. None means stale."""
        now = time.monotonic()
        if side == 'left':
            ms, val = self._tof_l_ms, self._tof_l
        else:
            ms, val = self._tof_r_ms, self._tof_r
        if ms <= 0.0 or (now - ms) > PICTURE_SIDE_STALE_S:
            return None
        if math.isnan(val) or val <= 0.0:
            return PICTURE_OPEN_M
        return val

    def _half_threat(self, side: str) -> float:
        """Closest non-floor return on that half of the nose. Open if only floor."""
        now = time.monotonic()
        vals: list[float] = []
        if self._l8_cols_ms > 0.0 and (now - self._l8_cols_ms) <= PICTURE_FRONT_STALE_S:
            idxs = (0, 1, 2, 3) if side == 'left' else (4, 5, 6, 7)
            for i in idxs:
                v = self._l8_cols[i]
                if not math.isnan(v):
                    vals.append(v)
        if self._front_ms > 0.0 and (now - self._front_ms) <= PICTURE_FRONT_STALE_S:
            v = self._front_l if side == 'left' else self._front_r
            if not math.isnan(v):
                vals.append(v)
        if not vals:
            return float('nan')
        real = [v for v in vals if not self._is_floor(v)]
        if not real:
            return PICTURE_OPEN_M
        return min(real)

    def _front_zone(self) -> tuple[int, float]:
        """0 far, 1 near, 2 close. Sign is +left / -right, toward the open half."""
        vals = self._picture_front_vals()
        if not vals:
            return 2, 0.0
        if all(self._is_floor(v) for v in vals):
            rank = 0
        else:
            closest = min(v for v in vals if not self._is_floor(v))
            rank = self._front_band(closest)
        left = self._half_threat('left')
        right = self._half_threat('right')
        sign = 0.0
        if not math.isnan(left) and not math.isnan(right):
            if left + 0.08 < right:
                sign = -1.0
            elif right + 0.08 < left:
                sign = 1.0
        return rank, sign

    def _side_zone(self) -> tuple[int, float]:
        left = self._side_span('left')
        right = self._side_span('right')
        if left is None or right is None:
            return 2, 0.0
        closer = min(left, right)
        rank = self._side_band(closer)
        sign = 0.0
        if left + 0.05 < right:
            sign = -1.0
        elif right + 0.05 < left:
            sign = 1.0
        return rank, sign

    def _veto_sign(self, sign: float) -> float:
        """Never turn into a side that is already a wall. 0 if both are."""
        if sign == 0.0:
            return 0.0
        left = self._side_span('left')
        right = self._side_span('right')

        def blocked(span: float | None) -> bool:
            return span is not None and span < PICTURE_SIDE_VETO_M

        if sign > 0.0 and blocked(left):
            if right is not None and not blocked(right):
                return -1.0
            return 0.0
        if sign < 0.0 and blocked(right):
            if left is not None and not blocked(left):
                return 1.0
            return 0.0
        return 1.0 if sign > 0.0 else -1.0

    def _held_band(self) -> int:
        """Current band, ignoring a stale hold."""
        if self._picture_rank >= 3:
            return 0
        return self._picture_rank

    def _front_band(self, closest: float) -> int:
        """Hysteresis so a range near 0.38 m does not flip close and near."""
        held = self._held_band()
        if closest < self._floor_m - PICTURE_FLOOR_DROP_M:
            return 2
        # A wall you can still walk up to is not a stop. Reverse only inside
        # about a body length; cruise while the nose is clear past half a metre,
        # and keep going when it is clear past a metre.
        if held >= 2:
            close_at, far_at = 0.40, 0.70
        elif held == 1:
            close_at, far_at = 0.24, 0.65
        else:
            close_at, far_at = 0.28, 0.50
        if closest < close_at:
            return 2
        if closest < far_at:
            return 1
        return 0

    def _side_band(self, closer: float) -> int:
        held = self._held_band()
        # Hips a half-metre away are the room, not a wall. Only a body-width
        # gap changes the band.
        if held >= 2:
            close_at, near_at = 0.28, 0.40
        elif held == 1:
            close_at, near_at = 0.18, 0.45
        else:
            close_at, near_at = 0.20, 0.35
        if closer < close_at:
            return 2
        if closer < near_at:
            return 1
        return 0

    def _sticky_rank(self, rank: int, now: float) -> int:
        """Step up immediately. Keep a cautious band across one slow sensor frame.

        Leaving stale is immediate once a sample is back.
        """
        if self._picture_rank >= 3 and rank < 3:
            self._picture_rank = rank
            self._picture_rank_since = now
            return rank
        if rank >= self._picture_rank:
            if rank != self._picture_rank:
                self._picture_rank_since = now
            self._picture_rank = rank
            return rank
        if (now - self._picture_rank_since) < PICTURE_DWELL_S:
            return self._picture_rank
        self._picture_rank = rank
        self._picture_rank_since = now
        return rank

    def _note_picture_motion(self, now: float, travel_m: float, closest: float | None) -> None:
        if travel_m > 0.0:
            self._travel_hist.append((now, travel_m))
        self._front_hist.append((now, closest))
        while self._travel_hist and (now - self._travel_hist[0][0]) > 1.0:
            self._travel_hist.popleft()
        while self._front_hist and (now - self._front_hist[0][0]) > 1.0:
            self._front_hist.popleft()

    def _closing_rank(self, rank: int, now: float) -> int:
        travel = sum(d for t, d in self._travel_hist if (now - t) <= 0.8)
        if travel < 0.06 or len(self._front_hist) < 2:
            return rank
        old = self._front_hist[0][1]
        new = self._front_hist[-1][1]
        if old is None or new is None:
            return rank
        if old - new > 0.12 and new < 0.50:
            return min(2, rank + 1)
        return rank

    def _far_steer(self, max_steer: float) -> float:
        left = self._side_span('left')
        right = self._side_span('right')
        if left is None or right is None:
            return 0.0
        # Hips lead. Nose columns only break a near-tie, so a long centre
        # reading cannot hide a closer wall.
        diff = left - right
        lt = self._half_threat('left')
        rt = self._half_threat('right')
        if not math.isnan(lt) and not math.isnan(rt) and abs(diff) < 0.15:
            nose = min(lt, PICTURE_OPEN_M) - min(rt, PICTURE_OPEN_M)
            diff += max(-0.40, min(0.40, nose * 0.35))
        # Both hips open: go straight. Do not weave on a 10 cm difference.
        if min(left, right) > 0.45 and abs(diff) < 0.40:
            return 0.0
        if abs(diff) < 0.25:
            return 0.0
        cap = min(max_steer, 0.06)
        return max(-cap, min(cap, (diff / 0.80) * cap))

    def _commit_heading(self, ang: float, now: float, travel_m: float) -> float:
        """Hold a far heading long enough to leave a 1 m pocket."""
        if now < self._commit_until and self._commit_travel < PICTURE_COMMIT_M:
            self._commit_travel += max(0.0, travel_m)
            return self._commit_ang
        self._commit_ang = ang
        self._commit_until = now + PICTURE_COMMIT_S
        self._commit_travel = 0.0
        return ang

    def picture_front_blocked(self) -> bool:
        """Nose has a real obstacle, not the floor and not a stale beam."""
        now = time.monotonic()
        if self._front_picture_stale(now):
            return False
        rank, _sign = self._front_zone()
        return rank >= 2

    def cruise_picture(
        self, cruise: float, max_steer: float, travel_m: float,
    ) -> tuple[float, float, str]:
        """Far / near / close command from the nose and the hips.

        Far: cruise, steer toward the more open side.
        Near: slow down and curve away.
        Close: stop the ram and turn into the open side. A hip inside the
        veto distance forbids that turn.
        Stale front or side: do not roll forward as if the way were clear.
        """
        now = time.monotonic()
        if self._front_picture_stale(now) or self._side_span('left') is None or self._side_span('right') is None:
            self._sticky_rank(3, now)
            return 0.0, 0.0, 'stale'

        front_rank, front_sign = self._front_zone()
        side_rank, side_sign = self._side_zone()
        raw = max(front_rank, side_rank)
        closest = self._obstacle_front_m()
        self._note_picture_motion(now, max(0.0, travel_m), None if math.isnan(closest) else closest)
        raw = self._closing_rank(raw, now)
        rank = self._sticky_rank(raw, now)

        if side_rank >= front_rank and side_sign != 0.0:
            sign = side_sign
        elif front_sign != 0.0:
            sign = front_sign
        else:
            sign = side_sign
        if sign == 0.0 and raw >= 2:
            # Tied picture still has to leave. Peel toward the wider hip,
            # and if they match, peel right so the choice does not chatter.
            left = self._side_span('left')
            right = self._side_span('right')
            if left is None or right is None:
                sign = 0.0
            elif left > right + 0.02:
                sign = 1.0
            else:
                sign = -1.0
        sign = self._veto_sign(sign)

        if rank >= 3:
            return 0.0, 0.0, 'stale'
        if PICTURE_STEER_AROUND:
            return self._steer_around(cruise, max_steer, front_rank, side_rank, side_sign, sign, now)
        if rank >= 2:
            if sign == 0.0:
                sign = self._veto_sign(-1.0)
            if sign == 0.0:
                return 0.0, 0.0, 'close'
            # Nose in something: back off while turning. Hip only: creep-turn.
            # lin 0 makes this ESP drop the turn. A dwell keeps the same
            # command until the band is allowed to relax.
            nose = front_rank >= 2 or rank > side_rank
            lin = -0.08 if nose else 0.06
            ang = sign * min(max_steer, 0.16)
            return lin, ang, 'close'
        if rank == 1:
            # Still approach. A near wall is not a reason to creep-turn in place.
            lin = min(max(0.0, cruise), max(0.10, cruise * 0.75))
            ang = 0.0 if sign == 0.0 else sign * min(max_steer, 0.05)
            return lin, ang, 'near'
        lin = max(0.0, cruise)
        ang = self._commit_heading(self._far_steer(max_steer), now, travel_m)
        held = self._veto_sign(1.0 if ang > 0.02 else (-1.0 if ang < -0.02 else 0.0))
        if held == 0.0:
            ang = 0.0
        elif ang > 0.02 and held < 0.0 or ang < -0.02 and held > 0.0:
            ang = held * min(max_steer, 0.08)
        return lin, ang, 'far'

    def _steer_around(
        self, cruise: float, max_steer: float, front_rank: int, side_rank: int,
        side_sign: float, sign: float, now: float,
    ) -> tuple[float, float, str]:
        """Keep rolling and curve toward the open side; reverse only when boxed in.

        Turn rate grows as the nose closes from PICTURE_FRONT_FAR_M to the box
        distance. Gap side is held with hysteresis so it does not wiggle.
        """
        if not hasattr(self, '_steer_boxed'):
            self._steer_boxed = False
            self._steer_sign = 0.0
            self._steer_sign_since = 0.0
        lh = self._half_threat('left')
        rh = self._half_threat('right')
        sl = self._side_span('left')
        sr = self._side_span('right')
        lh = PICTURE_OPEN_M if math.isnan(lh) else lh
        rh = PICTURE_OPEN_M if math.isnan(rh) else rh
        sl = PICTURE_OPEN_M if sl is None else sl
        sr = PICTURE_OPEN_M if sr is None else sr
        closest = min(lh, rh)

        # Boxed in: both halves of the nose inside the box distance (hysteresis).
        if self._steer_boxed:
            if max(lh, rh) > PICTURE_BOX_EXIT_M:
                self._steer_boxed = False
        elif max(lh, rh) < PICTURE_BOX_M:
            self._steer_boxed = True
        if self._steer_boxed:
            gap = self._veto_sign(sign if sign != 0.0 else (1.0 if sl > sr else -1.0))
            if gap == 0.0:
                return 0.0, 0.0, 'boxed'
            return -0.08, gap * min(max_steer, 0.16), 'boxed'

        # Openness per side: nose half, tempered by that hip.
        open_l = min(lh, sl * 1.5)
        open_r = min(rh, sr * 1.5)
        want = 1.0 if open_l > open_r else -1.0
        held = self._steer_sign
        if held == 0.0:
            self._steer_sign = want
            self._steer_sign_since = now
        elif want != held and abs(open_l - open_r) > PICTURE_GAP_HYST_M \
                and (now - self._steer_sign_since) >= 0.8:
            self._steer_sign = want
            self._steer_sign_since = now
        gap = self._veto_sign(self._steer_sign)

        far = PICTURE_FRONT_FAR_M
        if closest < far:
            t = (far - closest) / max(0.05, far - PICTURE_BOX_M)
            t = max(0.0, min(1.0, t))
            lin = max(0.06, max(0.0, cruise) * (1.0 - 0.5 * t))
            ang = 0.0 if gap == 0.0 else gap * (0.04 + t * (min(max_steer, 0.18) - 0.04))
            return lin, ang, ('close' if t >= 0.5 else 'near')
        self._steer_sign = 0.0
        if side_rank >= 2 and side_sign != 0.0:
            # Hip close but nose clear: keep going, peel off the wall.
            s2 = self._veto_sign(side_sign)
            return max(0.06, max(0.0, cruise) * 0.7), s2 * min(max_steer, 0.10), 'near'
        lin = max(0.0, cruise)
        ang = self._commit_heading(self._far_steer(max_steer), now, 0.0)
        hs = self._veto_sign(1.0 if ang > 0.02 else (-1.0 if ang < -0.02 else 0.0))
        if hs == 0.0:
            ang = 0.0
        return lin, ang, 'far'

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

    def _l8_real_m(self, v: float) -> float:
        if math.isnan(v) or v <= 0.02 or v >= self._l8_open_m:
            return float('nan')
        return v

    def _note_l8_distance(self, d: float) -> None:
        if math.isnan(d) or d >= 0.55:
            self._l8_near_streak = max(0, self._l8_near_streak - 1)
        else:
            self._l8_near_streak = min(12, self._l8_near_streak + 1)

    def _push_l8_ema(self, v: float) -> None:
        v = self._l8_real_m(v)
        if math.isnan(v):
            return
        if math.isnan(self._l8_fwd_ema):
            self._l8_fwd_ema = v
        else:
            self._l8_fwd_ema = 0.55 * self._l8_fwd_ema + 0.45 * v
        self._note_l8_distance(self._l8_fwd_ema)

    def l8_sustained_close(self, frames: int = 3) -> bool:
        return self._l8_near_streak >= frames

    def update_front_tof(self, center_m: float, left_m: float = float('nan'),
                         right_m: float = float('nan')) -> None:
        now = time.monotonic()
        self._front_ms = now

        def _ok(v: float) -> float:
            return self._l8_real_m(v)

        center = _ok(center_m)
        self._front_m = center
        if not math.isnan(center):
            self._push_l8_ema(center)
        if not math.isnan(left_m):
            self._front_l = _ok(left_m)
        if not math.isnan(right_m):
            self._front_r = _ok(right_m)
        samples = [
            v for v in (self._front_m, self._front_l, self._front_r) if not math.isnan(v)
        ]
        if samples:
            self._note_floor(samples)

    def update_l8_cols(self, cols: list[float]) -> None:
        now = time.monotonic()
        out = [float('nan')] * 8
        for i, raw in enumerate(cols[:8]):
            try:
                v = float(raw)
            except (TypeError, ValueError):
                continue
            v = self._l8_real_m(v)
            if not math.isnan(v):
                out[i] = v
        self._l8_cols = out
        self._l8_cols_ms = now
        vals = [v for v in out if not math.isnan(v)]
        if vals:
            self._push_l8_ema(min(vals))
            self._note_floor(vals)

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
        """Inner nose range — EMA filtered; open/floor is NaN not 3.5 m."""
        if not self._front_fresh():
            return float('nan')
        if not math.isnan(self._l8_fwd_ema):
            return self._l8_fwd_ema
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
        """Both hips see walls. Enter fast, exit only after sustained open gap."""
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        if math.isnan(sl) or math.isnan(sr):
            return self._corridor_latched
        enter_m = self.cfg.side_corridor_max_m
        exit_m = enter_m + 0.22
        now = time.monotonic()
        both_close = sl < enter_m and sr < enter_m
        both_far = sl > exit_m and sr > exit_m
        if both_close:
            self._corridor_latched = True
            self._corridor_exit_since = 0.0
            self._corridor_seen = now
            return True
        if not self._corridor_latched:
            return False
        if both_far:
            if self._corridor_exit_since <= 0.0:
                self._corridor_exit_since = now
            elif (now - self._corridor_exit_since) >= 2.0:
                self._corridor_latched = False
                self._corridor_exit_since = 0.0
                self._corridor_ang_filt = 0.0
                return False
        else:
            self._corridor_exit_since = 0.0
        return True

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
            # lin=0 makes ESP ignore turn. Ease into reverse — recover owns hard peel.
            return -min(0.08, max(0.06, abs(lin) * 0.5 if lin > 0.02 else 0.06)), (
                self.turn_away_from_closest() * 0.05
            )
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
        if both and abs(sl - sr) < 0.08:
            return 0.0
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
        """Hallway: drive forward, nudge off a close hip only.

        Do not equalize left/right ToF — yaw makes the outside sensor read longer
        and that steers harder into the wall. Matches hallway_follow bench logic.
        """
        sl = self._side_clear('left')
        sr = self._side_clear('right')
        if math.isnan(sl) or math.isnan(sr):
            lin = max(0.12, min(0.14, cruise))
            return lin, 0.0

        pinch = min(sl, sr)
        deadband = 0.06 if pinch < 0.32 else 0.10
        comfort = 0.38
        max_ang = 0.042 if pinch < 0.32 else 0.035
        lead = deadband * 0.5
        ang = 0.0
        if abs(sl - sr) < deadband:
            ang = 0.0
        elif sl < comfort and sl < sr - lead:
            ang -= max_ang * min(1.0, (comfort - sl) / 0.22)
        elif sr < comfort and sr < sl - lead:
            ang += max_ang * min(1.0, (comfort - sr) / 0.22)
        ang = max(-max_ang, min(max_ang, ang))
        if abs(ang) < 0.018:
            ang = 0.0

        self._corridor_ang_filt = 0.72 * self._corridor_ang_filt + 0.28 * ang
        if abs(self._corridor_ang_filt) < 0.015:
            self._corridor_ang_filt = 0.0
        ang = self._corridor_ang_filt

        lin = max(0.12, min(0.14, cruise))
        if pinch < 0.28:
            lin = min(lin, 0.10)
        if pinch < 0.22:
            lin = min(lin, 0.09)

        fwd = self.front_center_m()
        if math.isnan(fwd):
            fwd = self.l8_col_min()
        if not math.isnan(fwd) and fwd < 0.50:
            lin = min(lin, max(0.08, 0.05 + fwd * 0.16))
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
        if not self.l8_sustained_close(3):
            return False
        fwd = self.front_center_m()
        if not math.isnan(fwd) and fwd < 0.26:
            return True
        col = self.l8_col_min()
        if not math.isnan(col) and col < 0.24:
            return True
        return False

    def wants_early_turn(self) -> bool:
        """Wall entering the nose FOV — peel before the bumper."""
        if not self.l8_sustained_close(2):
            return False
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

    def cap_forward_l8(self, lin: float, ang: float, *, max_steer: float) -> tuple[float, float]:
        """Gradual nose slow/steer — must run in hallways too (ESP brakes at 24 cm)."""
        if lin <= 0.02:
            return lin, ang
        fwd = self._obstacle_front_m()
        if math.isnan(fwd):
            return lin, ang
        cfg = self.cfg
        if fwd < cfg.l8_turn_m:
            gap = self.l8_gap_steer(max_steer)
            if abs(gap) >= 0.02:
                blend = 0.35 if self.in_corridor() else 0.55
                ang = max(-max_steer, min(max_steer, (1.0 - blend) * ang + blend * gap))
        if fwd >= cfg.l8_slow_m:
            return lin, ang
        if fwd < cfg.l8_stop_m:
            lin = min(lin, max(0.06, lin * 0.30))
        else:
            scale = (fwd - cfg.l8_stop_m) / max(0.08, cfg.l8_slow_m - cfg.l8_stop_m)
            lin = min(lin, lin * max(0.22, min(1.0, scale)))
        return lin, ang

    def cap_turn_l8(self, lin: float, ang: float, *, max_steer: float) -> tuple[float, float]:
        """While turning, ease off if L8 columns on the inside of the arc are close."""
        if abs(ang) < 0.025:
            return lin, ang
        if (time.monotonic() - self._l8_cols_ms) > 0.6:
            return lin, ang
        side = self._l8_cols[:3] if ang > 0.0 else self._l8_cols[5:8]
        vals = [v for v in side if not math.isnan(v)]
        if not vals:
            return lin, ang
        close = min(vals)
        if close >= 0.40:
            return lin, ang
        scale = max(0.20, (close - 0.14) / 0.26)
        ang = max(-max_steer, min(max_steer, ang * scale))
        if close < 0.26 and lin > 0.05:
            lin = min(lin, 0.09)
        return lin, ang

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
                # Coast down — a hard zero here reads as a sudden stop on carpet.
                lin = min(lin, max(0.06, lin * 0.45))
            else:
                return -min(0.08, max(0.06, abs(lin) * 0.45)), ang * 0.5

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
