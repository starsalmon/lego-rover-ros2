"""Bench-tested rover explore + escape — extracted from autonomous_explore.py.

Brain sends intent (cruise / wander / escape); ESP SmoothDrive + heading hold
shape the motors. Escape fires on bump or stall only — not sonar proximity.
"""
from __future__ import annotations

from collections import deque
import math
import os
import random
import time
from enum import Enum, auto
from typing import Callable

from rover_proximity import ProximityState

# ESP spin-in-place when |linear| < ~0.08 and |angular| > 0.02 — creep forward when steering.
# This must be slow (otherwise we "creep-turn" into skirting/legs).
MIN_LIN_FOR_STEER = 0.06

MIN_DRIVE_SEC = 3.0
BURST_PROB = 0.01
BURST_DURATION = (0.8, 1.2)

ESCAPE_REVERSE_SEC = 1.0
ESCAPE_ARC_LIN = 0.22
ESCAPE_ARC_ANG = 0.12
SPIN_DEG_MIN = 45
SPIN_DEG_MAX = 85

FRONT_AVOID_REVERSE_SEC = 1.0
FRONT_AVOID_STRAIGHT_SEC = 0.40
FRONT_AVOID_STEER = 0.10
FRONT_DRIVE_AWAY_SEC = 1.6
FRONT_DRIVE_AWAY_POWER = 0.16
FRONT_DRIVE_AWAY_STEER = 0.12
FRONT_CLEAR_HOLD_SEC = 0.40
FRONT_CLEAR_TIMEOUT_SEC = 2.5
FRONT_IR_COOLDOWN_SEC = 0.6
FRONT_STUCK_ESCALATE = 3

STARTUP_GRACE_SEC = 1.5
EVENT_COOLDOWN_SEC = 5.0
STRAIGHT_BLOCK_SEC = 10.0
MAX_ESCAPES_PER_MIN = 3
ESCAPE_FLOOD_PAUSE_SEC = 10.0
ARC_YAW_TOL_DEG = 8.0

# Brain-side output slew in open modes (ESP still does per-wheel ramping).
LIN_SLEW_PER_S = 0.40
ANG_SLEW_PER_S = 0.90


def _noop(*_a, **_k) -> None:
    pass


def _max_linear() -> float:
    return max(0.08, min(1.0, float(os.environ.get('ROVER_AUTO_MAX_LINEAR', '0.17'))))


def _burst_max() -> float:
    return max(_max_linear(), min(1.0, float(os.environ.get('ROVER_AUTO_BURST_MAX', '0.28'))))


def _reverse_power() -> float:
    raw = os.environ.get('ROVER_AUTO_REVERSE_POWER', '').strip()
    if raw:
        return max(0.06, min(1.0, float(raw)))
    frac = float(os.environ.get('ROVER_AUTO_REVERSE_FRAC', '0.65'))
    return _max_linear() * max(0.25, min(1.0, frac))


def _max_reverse() -> float:
    raw = os.environ.get('ROVER_AUTO_MAX_REVERSE', '').strip()
    if raw:
        return max(0.06, min(1.0, float(raw)))
    return _reverse_power()


def _burst_prob() -> float:
    return max(0.0, min(0.2, float(os.environ.get('ROVER_AUTO_BURST_PROB', str(BURST_PROB)))))


def _max_steer() -> float:
    raw = os.environ.get('ROVER_AUTO_MAX_ANGULAR', '').strip()
    if raw:
        return max(0.03, min(0.35, float(raw)))
    return max(0.05, _max_linear() * 0.55)


def _escape_spin_sign() -> float:
    return float(os.environ.get('ROVER_ESCAPE_SPIN_SIGN', '-1'))


def _imu_gyro_sign() -> float:
    return float(os.environ.get('ROVER_IMU_GYRO_SIGN', '1'))


def _front_ir_stop() -> bool:
    if os.environ.get('ROVER_SONAR', '1').strip().lower() in ('1', 'yes', 'true'):
        return False
    return os.environ.get('ROVER_IR_FRONT_STOP', '0').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _sonar_front() -> bool:
    return os.environ.get('ROVER_SONAR', '1').strip().lower() in ('1', 'yes', 'true')


def _cap_linear(lin: float, *, burst: bool = False) -> float:
    cap = _max_reverse() if lin < 0 else (_burst_max() if burst else _max_linear())
    return max(-cap, min(cap, lin))


def _cap_angular(ang: float) -> float:
    return max(-_max_steer(), min(_max_steer(), ang))


def _arc_not_spin(lin: float, ang: float) -> tuple[float, float]:
    if abs(ang) <= 0.02:
        return lin, ang
    if abs(lin) >= MIN_LIN_FOR_STEER:
        return lin, ang
    creep = MIN_LIN_FOR_STEER if lin >= 0 else -MIN_LIN_FOR_STEER
    if abs(lin) < 0.02:
        creep = MIN_LIN_FOR_STEER
    return creep, ang


class Mode(Enum):
    CRUISE = auto()
    BURST = auto()
    WANDER = auto()
    AVOID = auto()
    ESCAPE = auto()
    RECOVER = auto()


class RecoverPhase(Enum):
    REVERSE = auto()
    SCAN = auto()
    DRIVE_OUT = auto()


class EscapePhase(Enum):
    REVERSE = auto()
    SCAN = auto()
    ARC = auto()


class AvoidPhase(Enum):
    REVERSE = auto()
    DRIVE_AWAY = auto()


class ExploreController:
    """Session-scoped explore FSM — port of autonomous_explore.py (Aug 2026 captures)."""

    def __init__(self, *, log: Callable[[str], None] | None = None) -> None:
        self._log = log or _noop
        self._speaker = _noop
        self._ring = _noop
        self._bg_radar = None
        self._proximity = ProximityState()
        self._init_state()
        self._try_bg_radar()

    def _try_bg_radar(self) -> None:
        try:
            from rover_radar import BackgroundRadar, cruise_radar_enabled

            if cruise_radar_enabled():
                self._bg_radar = BackgroundRadar.start()
        except Exception:
            self._bg_radar = None

    def _init_state(self) -> None:
        self.mode = Mode.CRUISE
        self.mode_until = time.monotonic() + MIN_DRIVE_SEC
        self._mode_dur = MIN_DRIVE_SEC
        cap = _max_linear()
        steer = _max_steer()
        self._cruise_speed = cap * 0.85
        self._cruise_curve = 0.04
        self.wander_ang = 0.05
        self.wander_lin = cap * 0.80

        self.escape_phase = EscapePhase.REVERSE
        self.avoid_steer_dir = 1.0
        self.avoid_phase = AvoidPhase.REVERSE
        self.avoid_phase_until = 0.0
        self.avoid_start = 0.0

        self.escape_backoff = -_reverse_power()
        self.arc_steer = ESCAPE_ARC_ANG
        self.arc_until = 0.0
        self.reverse_until = 0.0
        self._arc_target_deg = SPIN_DEG_MIN
        self._arc_yaw_start = 0.0

        self._yaw_integrated = 0.0
        self._imu_last_mono: float | None = None
        self._imu_ok = False

        self.straight_block_until = 0.0
        self._driving_linear = 0.0
        self._stuck_linear = 0.0
        self._start_time = time.monotonic()
        self._event_ignore_until = self._start_time + STARTUP_GRACE_SEC
        self._bump_pending = False
        self._stall_pending = False
        self._scan_done = False
        self._ir_front_cooldown_until = 0.0
        self._ir_wait_clear = False
        self._front_clear_since: float | None = None
        self._ir_wait_clear_since: float | None = None
        self._front_stuck_retries = 0
        self._escape_times: list[float] = []
        self._imu_gyro_sign = _imu_gyro_sign()
        self._escape_spin_sign = _escape_spin_sign()
        self._last_driving = False
        self._out_lin = 0.0
        self._out_ang = 0.0
        self._scan_until = 0.0
        self._scan_best_deg = 90.0
        self._scan_best_m = -1.0
        self._scan_seen_left = False
        self._scan_seen_right = False
        self._scan_started = 0.0
        self._scan_cooldown_until = 0.0

        # Sonar-brake + oscillation recovery (stops the left-right rotate bounce).
        self._sonar_avoid_active = False
        self._sonar_avoid_since = 0.0
        self._imu_gz_hist: deque[tuple[float, float]] = deque()
        self._recover_phase = RecoverPhase.REVERSE
        self._recover_reverse_until = 0.0
        self._recover_driveout_until = 0.0
        self._recover_sign = 0.0
        self._recover_cooldown_until = 0.0
        self._recover_reason = ""
        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))

    def reset(self) -> None:
        if self._bg_radar is not None:
            try:
                self._bg_radar.resume()
            except Exception:
                pass
        self._init_state()
        self._log(
            f'explore reset — cruise {_max_linear():.0%}, steer max {_max_steer():.2f}, '
            f'escape bump/stall only'
        )

    def shutdown(self) -> None:
        if self._bg_radar is not None:
            try:
                from rover_radar import BackgroundRadar

                BackgroundRadar.stop()
            except Exception:
                pass

    def on_bump(self) -> None:
        self._bump_pending = True

    def on_stall(self) -> None:
        self._stall_pending = True

    def on_sonar_avoid(self, active: bool) -> None:
        """ESP sonar brake active (forward hard-brake)."""
        now = time.monotonic()
        active = bool(active)
        if active and not self._sonar_avoid_active:
            self._sonar_avoid_since = now
        if not active:
            self._sonar_avoid_since = 0.0
        self._sonar_avoid_active = active

    def on_sonar_range(self, range_m: float) -> None:
        self._proximity.update_range(range_m)
        # During a scan, treat incoming range as candidates for the best direction.
        if self._scan_until > time.monotonic() and not math.isnan(range_m) and range_m > 0.0:
            # Use the current pan position as the "bearing".
            try:
                pan = float(self._proximity._pan_deg)  # internal; ok for this heuristic
            except Exception:
                pan = 90.0
            if range_m > self._scan_best_m:
                self._scan_best_m = float(range_m)
                self._scan_best_deg = pan

    def on_pan(self, pan_deg: float) -> None:
        self._proximity.update_pan(pan_deg)
        if self._scan_until > time.monotonic():
            if pan_deg < 20.0:
                self._scan_seen_left = True
            elif pan_deg > 160.0:
                self._scan_seen_right = True

    def wants_cal_sweep(self, now: float) -> bool:
        return now < self._scan_until

    def _maybe_start_scan(self, now: float) -> None:
        # Only in open modes; escape remains bump/stall only.
        if now < self._scan_cooldown_until:
            return
        if self.mode in (Mode.ESCAPE, Mode.AVOID, Mode.RECOVER):
            return
        # If we're trying to drive forward but the forward cone stays blocked for a bit,
        # do a quick pan sweep and commit toward the best opening.
        if self._proximity.persistently_blocked(now, min_s=1.2):
            self._scan_started = now
            self._scan_until = now + 3.2  # covers current ESP cal sweep timings
            self._scan_best_deg = 90.0
            self._scan_best_m = -1.0
            self._scan_seen_left = False
            self._scan_seen_right = False
            self._scan_cooldown_until = now + 8.0
            self._log('stuck → quick scan (pan sweep)')

    def on_tof_left(self, range_m: float) -> None:
        self._proximity.update_tof_left(range_m)

    def on_tof_right(self, range_m: float) -> None:
        self._proximity.update_tof_right(range_m)

    def on_imu(self, gz_rad_s: float) -> None:
        now = time.monotonic()
        gz = self._imu_gyro_sign * float(gz_rad_s)
        if self._imu_last_mono is not None:
            dt = now - self._imu_last_mono
            if 0.0 < dt < 0.5:
                self._yaw_integrated += gz * dt
        self._imu_last_mono = now
        self._imu_ok = True
        self._imu_gz_hist.append((now, gz))
        # Keep a short window for oscillation detection.
        while self._imu_gz_hist and (now - self._imu_gz_hist[0][0]) > 2.6:
            self._imu_gz_hist.popleft()

    def _imu_oscillating(self, now: float) -> bool:
        """Detect back-and-forth rotation (sign-flipping yaw-rate) in a short window."""
        if now < self._start_time + STARTUP_GRACE_SEC:
            return False
        seq: list[int] = []
        for _t, gz in self._imu_gz_hist:
            if abs(gz) < 0.18:
                continue
            s = 1 if gz > 0.0 else -1
            if not seq or s != seq[-1]:
                seq.append(s)
        flips = max(0, len(seq) - 1)
        return flips >= 4

    def _close_for_recover(self) -> bool:
        cfg = self._proximity.cfg
        fwd = self._proximity.forward_stop_m()
        if not math.isnan(fwd) and fwd < cfg.sonar_slow_m():
            return True
        # Side ToF close.
        tl = self._proximity._side_clear("left")
        tr = self._proximity._side_clear("right")
        if (not math.isnan(tl) and tl < cfg.tof_slow_m()) or (not math.isnan(tr) and tr < cfg.tof_slow_m()):
            return True
        # ESP is actively braking forward.
        if self._sonar_avoid_active:
            return True
        return False

    def _start_recover(self, now: float, *, reason: str) -> None:
        if now < self._recover_cooldown_until:
            return
        self._recover_reason = reason
        self._recover_phase = RecoverPhase.REVERSE
        self._recover_reverse_until = now + 0.75
        self._recover_driveout_until = 0.0
        self._recover_sign = 0.0
        self.mode = Mode.RECOVER
        self.mode_until = self._recover_reverse_until
        self._mode_dur = 0.75
        self._scan_until = 0.0
        self._proximity.clear_blocked()
        self._recover_cooldown_until = now + 7.0
        self._log(f"recover ({reason}): reverse → scan → drive-out")

    def _tick_recover(self, now: float) -> tuple[float, float]:
        # Reverse first to give the tail clearance before we turn.
        if self._recover_phase == RecoverPhase.REVERSE:
            if now < self._recover_reverse_until:
                return -min(0.10, _reverse_power()), 0.0
            # Start scan.
            self._recover_phase = RecoverPhase.SCAN
            self._scan_started = now
            self._scan_until = now + 3.2
            self._scan_best_deg = 90.0
            self._scan_best_m = -1.0
            self._scan_seen_left = False
            self._scan_seen_right = False
            return 0.0, 0.0

        if self._recover_phase == RecoverPhase.SCAN:
            # Scan is handled by `_scan_until` early-return in tick().
            return 0.0, 0.0

        # DRIVE_OUT: hold a committed arc for a short time.
        if now < self._recover_driveout_until and self._recover_sign != 0.0:
            lin = max(0.10, _max_linear() * 0.70)
            ang = self._recover_sign * min(_max_steer(), 0.12)
            return lin, ang

        # Done → back to cruise.
        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
        self._recover_phase = RecoverPhase.REVERSE
        self._recover_driveout_until = 0.0
        self._recover_sign = 0.0
        return self._cruise_speed, self._cruise_curve

    def _arc_yaw_turned_deg(self) -> float:
        return abs(math.degrees(self._yaw_integrated - self._arc_yaw_start))

    def _front_blocked(self) -> bool:
        try:
            from rover_ir import read_front

            return bool(read_front())
        except Exception:
            return False

    def _reversing_from_front(self) -> bool:
        if self.mode == Mode.AVOID and self.avoid_phase == AvoidPhase.REVERSE:
            return True
        if self.mode == Mode.ESCAPE and self.escape_phase == EscapePhase.REVERSE:
            return True
        return False

    def _apply_front_stop(self, lin: float, ang: float, front_blocked: bool) -> tuple[float, float]:
        if not _front_ir_stop() or not front_blocked:
            return lin, ang
        if self._reversing_from_front():
            return (0.0, ang) if lin > 0.0 else (lin, ang)
        return 0.0, 0.0

    def _sync_bg_radar(self) -> None:
        if self._bg_radar is None:
            return
        if self.mode in (Mode.ESCAPE, Mode.AVOID):
            self._bg_radar.pause()
        else:
            self._bg_radar.resume()

    def _enter(self, mode: Mode, duration: float) -> None:
        duration = max(duration, MIN_DRIVE_SEC)
        self.mode = mode
        self.mode_until = time.monotonic() + duration
        self._mode_dur = duration
        cap = _max_linear()
        steer = _max_steer()
        if mode == Mode.CRUISE:
            # Wider spread makes it feel less "one speed".
            self._cruise_speed = random.uniform(cap * 0.55, cap * 0.92)
            if random.random() < 0.55:
                self._cruise_curve = 0.0
            else:
                self._cruise_curve = random.choice([-1, 1]) * random.uniform(0.01, steer * 0.28)
        elif mode == Mode.WANDER:
            if random.random() < 0.25:
                self.wander_ang = 0.0
            else:
                self.wander_ang = random.choice([-1, 1]) * random.uniform(0.01, steer * 0.40)
            self.wander_lin = random.uniform(cap * 0.40, cap * 0.82)
        elif mode == Mode.BURST:
            self._cruise_curve = random.choice([-1, 1]) * random.uniform(0.02, steer * 0.35)

    def _trigger_escape(self) -> None:
        now = time.monotonic()
        self._escape_times = [t for t in self._escape_times if now - t < 60.0]
        if len(self._escape_times) >= MAX_ESCAPES_PER_MIN:
            self._log('escape flood — pausing')
            self._bump_pending = False
            self._stall_pending = False
            self._event_ignore_until = now + ESCAPE_FLOOD_PAUSE_SEC
            return

        if not (self._bump_pending or self._stall_pending):
            return
        if self.mode == Mode.ESCAPE:
            return
        if now < self._event_ignore_until:
            self._bump_pending = False
            self._stall_pending = False
            return

        reason = 'bump+stall' if self._bump_pending and self._stall_pending else (
            'bump' if self._bump_pending else 'stall'
        )
        self._bump_pending = False
        self._stall_pending = False
        self._escape_times.append(now)
        self._stuck_linear = self._driving_linear
        self._scan_done = False
        self.escape_phase = EscapePhase.REVERSE
        self.escape_backoff = -_reverse_power()
        self.reverse_until = now + ESCAPE_REVERSE_SEC
        self.mode = Mode.ESCAPE
        self._ir_wait_clear = False
        self._log(f'escape ({reason}): reverse → arc (IMU-limited)')

    def _start_escape_arc(self, now: float, steer: float, duration: float, target_deg: float) -> None:
        self.escape_phase = EscapePhase.ARC
        self.arc_steer = steer
        self.arc_until = now + duration
        self._arc_target_deg = min(target_deg, SPIN_DEG_MAX)
        self._arc_yaw_start = self._yaw_integrated
        self.mode_until = self.arc_until
        self._mode_dur = duration
        self._event_ignore_until = self.arc_until + EVENT_COOLDOWN_SEC

    def _run_escape_scan(self) -> None:
        self._scan_done = True
        steer = max(0.08, min(ESCAPE_ARC_ANG, _max_steer()))
        now = time.monotonic()
        spin_deg = random.uniform(SPIN_DEG_MIN, SPIN_DEG_MAX)
        spin_dir = random.choice([-1.0, 1.0]) * self._escape_spin_sign
        bearing = -1.0

        if self._bg_radar is not None:
            hits, _ = self._bg_radar.get_hits()
            if hits and not all(hits):
                try:
                    from rover_radar import pick_escape_spin

                    spin_dir, spin_deg, bearing = pick_escape_spin(
                        hits,
                        spin_min=SPIN_DEG_MIN,
                        spin_max=SPIN_DEG_MAX,
                    )
                except Exception:
                    pass

        duration = max(1.2, min(2.8, spin_deg / 40.0))
        if bearing >= 0:
            self._log(
                f'escape arc {bearing:.0f}° gap → {spin_deg:.0f}° '
                f'({"right" if spin_dir > 0 else "left"})'
            )
        else:
            self._log(
                f'escape arc {spin_deg:.0f}° '
                f'({"right" if spin_dir > 0 else "left"})'
            )
        self._start_escape_arc(now, spin_dir * steer, duration, spin_deg)

    def _finish_escape_arc(self, now: float) -> None:
        self._event_ignore_until = now + EVENT_COOLDOWN_SEC
        self.straight_block_until = now + STRAIGHT_BLOCK_SEC
        if self.arc_steer != 0.0:
            self._proximity.commit_steer_away(1.0 if self.arc_steer > 0.0 else -1.0, 6.0)
        self._enter(Mode.CRUISE, random.uniform(5.0, 9.0))
        self._log('escape done → straight cruise')

    def _next_mode(self) -> None:
        now = time.monotonic()
        if now < self.straight_block_until:
            self._enter(Mode.CRUISE, random.uniform(5.0, 9.0))
            self._log('cruise (post-escape)')
            return
        roll = random.random()
        if roll < _burst_prob():
            dur = random.uniform(*BURST_DURATION)
            self._enter(Mode.BURST, dur)
            self._log(f'burst ({dur:.1f}s)')
        elif roll < 0.70:
            self._enter(Mode.CRUISE, random.uniform(4.0, 8.0))
        else:
            self._enter(Mode.WANDER, random.uniform(3.0, 5.0))

    def tick(self) -> tuple[float, float]:
        now = time.monotonic()

        # Note intent for stuck detection (before shaping).
        self._proximity.note_drive_intent(
            want_forward=self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST, Mode.RECOVER)
        )
        self._maybe_start_scan(now)

        if _front_ir_stop() and self.mode not in (Mode.ESCAPE, Mode.AVOID):
            if self._front_blocked() and now >= self._ir_front_cooldown_until:
                self._trigger_front_avoid(now)

        if self._bump_pending or self._stall_pending:
            self._trigger_escape()

        # If we're oscillating (back-and-forth rotation) while close to obstacles,
        # force a clean recover: reverse → scan → committed drive-out.
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST) and now >= self._event_ignore_until:
            if self._imu_oscillating(now) and self._close_for_recover():
                self._start_recover(now, reason="oscillation")

        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST) and now >= self.mode_until:
            self._next_mode()

        lin = 0.0
        ang = 0.0
        front_blocked = _front_ir_stop() and self._front_blocked()

        # During scan: stop and let the pan sweep do its thing.
        if now < self._scan_until:
            self._sync_bg_radar()
            return 0.0, 0.0

        if self.mode == Mode.CRUISE:
            lin = self._cruise_speed
            ang = self._cruise_curve
        elif self.mode == Mode.BURST:
            peak = _burst_max()
            base = _max_linear()
            t_left = max(0.0, self.mode_until - now)
            frac = 1.0 - (t_left / max(0.01, self._mode_dur))
            lin = base + (peak - base) * min(1.0, frac / 0.35) if frac < 0.35 else peak
            ang = self._cruise_curve * 0.5
        elif self.mode == Mode.WANDER:
            lin = self.wander_lin
            ang = self.wander_ang
        elif self.mode == Mode.AVOID:
            lin, ang = self._tick_avoid(now, front_blocked)
        elif self.mode == Mode.ESCAPE:
            lin, ang = self._tick_escape(now, front_blocked)
        elif self.mode == Mode.RECOVER:
            lin, ang = self._tick_recover(now)

        # Tier-1 shaping: slow + steer-away in open room. Never hard-stop here —
        # ESP owns the forward brake; brain just biases away early.
        if _sonar_front() and self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST, Mode.RECOVER):
            lin, ang = self._proximity.apply(lin, ang, max_steer=_max_steer())
            commit = self._proximity.steer_commit_active(now)
            if commit != 0.0:
                ang = max(-_max_steer(), min(_max_steer(), ang + commit * _max_steer() * 0.35))

        if self._bg_radar and self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST, Mode.RECOVER):
            ang += self._bg_radar.steer_bias()

        lin, ang = self._apply_front_stop(lin, ang, front_blocked)
        lin, ang = _arc_not_spin(lin, ang)

        # If scan just finished and we saw both sides, commit toward the best opening.
        if self._scan_until > 0.0 and now >= self._scan_until and (now - self._scan_started) < 6.0:
            if self._scan_seen_left and self._scan_seen_right:
                center = self._proximity.cfg.pan_center
                err = self._scan_best_deg - center
                sign = 1.0 if err > 6.0 else (-1.0 if err < -6.0 else random.choice([-1.0, 1.0]))
                self._proximity.commit_steer_away(sign, 3.0)
                if self.mode == Mode.RECOVER and self._recover_phase == RecoverPhase.SCAN:
                    self._recover_phase = RecoverPhase.DRIVE_OUT
                    self._recover_sign = sign
                    self._recover_driveout_until = now + 2.2
                    self.mode_until = self._recover_driveout_until
                    self._mode_dur = 2.2
                    self._log(
                        f'scan done → drive-out {"right" if sign > 0 else "left"} '
                        f'(best {self._scan_best_m:.2f}m @ {self._scan_best_deg:.0f}°)'
                    )
                else:
                    self._enter(Mode.CRUISE, random.uniform(4.0, 6.0))
                    self._log(
                        f'scan done → commit {"right" if sign > 0 else "left"} '
                        f'(best {self._scan_best_m:.2f}m @ {self._scan_best_deg:.0f}°)'
                    )
            self._scan_until = 0.0

        # Slew-limit in open modes (not during explicit avoid/escape).
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST, Mode.RECOVER):
            dt = 0.05
            dlin = LIN_SLEW_PER_S * dt
            dang = ANG_SLEW_PER_S * dt
            lin = max(self._out_lin - dlin, min(self._out_lin + dlin, lin))
            ang = max(self._out_ang - dang, min(self._out_ang + dang, ang))
            self._out_lin = lin
            self._out_ang = ang
        else:
            self._out_lin = lin
            self._out_ang = ang

        # Avoid pivot-style spins: as we slow down, clamp angular to stay arc-like.
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST, Mode.RECOVER):
            max_ang = min(_max_steer(), 0.06 + abs(lin) * 1.05)
            ang = max(-max_ang, min(max_ang, ang))

        if self.mode != Mode.ESCAPE:
            self._driving_linear = lin
        lin = _cap_linear(lin, burst=self.mode == Mode.BURST)
        ang = _cap_angular(ang)
        self._last_driving = lin > 0.05
        self._sync_bg_radar()
        return lin, ang

    def _tick_avoid(self, now: float, front_blocked: bool) -> tuple[float, float]:
        lin = 0.0
        ang = 0.0
        if self.avoid_phase == AvoidPhase.REVERSE:
            lin = -_reverse_power()
            ang = 0.0 if now < self.avoid_start + FRONT_AVOID_STRAIGHT_SEC else (
                self.avoid_steer_dir * FRONT_AVOID_STEER
            )
            if now >= self.avoid_phase_until:
                if front_blocked:
                    self.avoid_start = now
                    self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
                    self.avoid_steer_dir *= -1.0
                else:
                    self.avoid_phase = AvoidPhase.DRIVE_AWAY
                    self.avoid_phase_until = now + FRONT_DRIVE_AWAY_SEC
        elif front_blocked:
            self.avoid_phase = AvoidPhase.REVERSE
            self.avoid_start = now
            self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
            lin = -_reverse_power()
            ang = self.avoid_steer_dir * FRONT_AVOID_STEER
        else:
            lin = FRONT_DRIVE_AWAY_POWER
            ang = self.avoid_steer_dir * FRONT_DRIVE_AWAY_STEER
            if now >= self.avoid_phase_until:
                self._finish_front_avoid(now)
        return lin, ang

    def _tick_escape(self, now: float, front_blocked: bool) -> tuple[float, float]:
        lin = 0.0
        ang = 0.0
        if self.escape_phase == EscapePhase.REVERSE:
            if now < self.reverse_until:
                lin = self.escape_backoff
            else:
                self.escape_phase = EscapePhase.SCAN
        elif self.escape_phase == EscapePhase.SCAN:
            if not self._scan_done:
                self._run_escape_scan()
        elif self.escape_phase == EscapePhase.ARC:
            if front_blocked:
                self.escape_phase = EscapePhase.REVERSE
                self.reverse_until = now + ESCAPE_REVERSE_SEC * 0.7
                lin = self.escape_backoff
                self._log('escape arc blocked — reverse again')
            else:
                lin = ESCAPE_ARC_LIN
                ang = self.arc_steer
                turned = self._arc_yaw_turned_deg()
                yaw_done = self._imu_ok and turned >= (
                    self._arc_target_deg - ARC_YAW_TOL_DEG
                )
                if yaw_done or now >= self.arc_until:
                    if yaw_done:
                        self._log(f'escape arc IMU {turned:.0f}° → cruise')
                    self._finish_escape_arc(now)
        return lin, ang

    def _trigger_front_avoid(self, now: float) -> None:
        if self.mode in (Mode.ESCAPE, Mode.AVOID) or now < self._event_ignore_until:
            return
        self.avoid_steer_dir = random.choice([-1.0, 1.0])
        self.avoid_start = now
        self.avoid_phase = AvoidPhase.REVERSE
        self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
        self.mode = Mode.AVOID
        self._ir_front_cooldown_until = now + FRONT_IR_COOLDOWN_SEC
        self._log('front IR — reverse then arc')

    def _finish_front_avoid(self, now: float) -> None:
        self._event_ignore_until = now + 1.0
        self._proximity.commit_steer_away(self.avoid_steer_dir, 4.5)
        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
