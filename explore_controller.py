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
from rover_local_memory import LocalHeadingMemory
from rover_motion import ExploreMotionHints

# ESP spin-in-place when |linear| < ~0.08 and |angular| > 0.02 — creep forward when steering.
# This must be slow (otherwise we "creep-turn" into skirting/legs).
MIN_LIN_FOR_STEER = 0.06

MIN_DRIVE_SEC = 3.0
BURST_PROB = 0.01
BURST_DURATION = (0.8, 1.2)

ESCAPE_REVERSE_SEC = 1.0
ESCAPE_ARC_LIN = 0.10
ESCAPE_ARC_ANG = 0.06
SPIN_DEG_MIN = 20
SPIN_DEG_MAX = 35

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

# Brain-side output slew — always on. ESP still ramps wheels in time.
LIN_SLEW_PER_S = 0.32
LIN_BRAKE_SLEW_PER_S = 0.18
ANG_SLEW_PER_S = 0.22
ANG_BRAKE_SLEW_PER_S = 0.14

# ESP cal_sweep duration (rover_sonar.cpp). Keep in sync so scan windows don’t
# “do nothing” for multiple seconds.
CAL_SWEEP_SEC = float(os.environ.get('ROVER_CAL_SWEEP_SEC', '1.9'))


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
    return False


def _sonar_front() -> bool:
    # Nose occupancy from VL53L8CX (pan sonar is aft).
    return os.environ.get('ROVER_FRONT_TOF', '1').strip().lower() not in (
        '0',
        'no',
        'false',
    )


def _cap_linear(lin: float, *, burst: bool = False) -> float:
    cap = _max_reverse() if lin < 0 else (_burst_max() if burst else _max_linear())
    return max(-cap, min(cap, lin))


def _cap_angular(ang: float) -> float:
    return max(-_max_steer(), min(_max_steer(), ang))


def _arc_not_spin(lin: float, ang: float) -> tuple[float, float]:
    if abs(ang) <= 0.02:
        return lin, ang
    if abs(lin) < 0.02:
        return lin, ang
    if abs(lin) >= MIN_LIN_FOR_STEER:
        return lin, ang
    # Preserve sign: never turn a tiny reverse into forward creep.
    creep = MIN_LIN_FOR_STEER if lin >= 0 else -MIN_LIN_FOR_STEER
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
        self._memory = LocalHeadingMemory()
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
        self._cruise_curve = 0.0
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
        self._wheel_travel_m = 0.0
        self._wheels_moving = False
        self._motion_hints = ExploreMotionHints(
            allow_shaping=False,
            in_corridor=False,
            along_wall=False,
            sonar_front=_sonar_front(),
        )
        self._scan_done = False
        self._ir_front_cooldown_until = 0.0
        self._ir_fl = False
        self._ir_fr = False
        self._ir_rear = False
        self._ir_hits_at = 0.0
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
        self._last_pan_deg = 90.0

        # Sonar-brake + oscillation recovery (stops the left-right rotate bounce).
        self._sonar_avoid_active = False
        self._sonar_avoid_since = 0.0
        self._imu_gz_hist: deque[tuple[float, float]] = deque()
        self._recover_phase = RecoverPhase.REVERSE
        self._recover_reverse_until = 0.0
        self._recover_driveout_until = 0.0
        self._recover_sign = 0.0
        self._recover_need_scan = False
        self._recover_did_scan = False
        self._recover_cooldown_until = 0.0
        self._recover_reason = ""
        self._recover_yaw_start = 0.0
        self._recover_yaw_need = 60.0
        self._memory.reset()
        self._memory.start_leg()
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

    def set_wheels_moving(self, moving: bool) -> None:
        self._wheels_moving = bool(moving)

    def add_wheel_travel(self, delta_m: float) -> None:
        if delta_m > 0.0:
            self._wheel_travel_m += delta_m

    def on_ir_hits(self, bits: int) -> None:
        """ESP delta IR: bit0=front L, bit1=front R, bit2=rear."""
        bits = int(bits) & 0xFF
        self._ir_fl = bool(bits & 0x01)
        self._ir_fr = bool(bits & 0x02)
        self._ir_rear = bool(bits & 0x04)
        self._ir_hits_at = time.monotonic()

    def _ir_fresh(self) -> bool:
        return (time.monotonic() - self._ir_hits_at) < 0.30

    def _ir_away_sign(self) -> float:
        """+ang = left. Hit left only → peel right (−). Hit right only → peel left (+)."""
        if not self._ir_fresh():
            return 0.0
        if self._ir_fl and not self._ir_fr:
            return -1.0
        if self._ir_fr and not self._ir_fl:
            return 1.0
        return 0.0

    def _rear_ir_bumper(self) -> bool:
        return self._ir_fresh() and self._ir_rear

    def _rear_ir_hit(self) -> bool:
        if self._rear_ir_bumper():
            return True
        return self._proximity.is_rear_blocked()

    def on_sonar_avoid(self, active: bool) -> None:
        """ESP sonar brake active (aft reverse hard-brake)."""
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
        # Note: the ESP cal_sweep can start slightly after we request it; keep
        # accepting samples for a short grace window beyond `_scan_until`.
        if (
            self._scan_until > 0.0
            and (time.monotonic() - self._scan_started) < 6.0
            and not math.isnan(range_m)
            and range_m > 0.0
        ):
            pan = float(self._last_pan_deg)
            if range_m > self._scan_best_m:
                self._scan_best_m = float(range_m)
                self._scan_best_deg = pan

    def on_pan(self, pan_deg: float) -> None:
        self._last_pan_deg = float(pan_deg)
        self._proximity.update_pan(pan_deg)
        if self._scan_until > 0.0 and (time.monotonic() - self._scan_started) < 6.0:
            if pan_deg < 20.0:
                self._scan_seen_right = True
            elif pan_deg > 160.0:
                self._scan_seen_left = True

    def wants_cal_sweep(self, now: float) -> bool:
        return now < self._scan_until

    def _maybe_start_scan(self, now: float) -> None:
        # Default: do not scan on "stuck" (it feels like doing nothing).
        # Recovery + sonar-brake fallback can still request cal_sweep explicitly.
        if os.environ.get('ROVER_SCAN_ON_STUCK', '0').strip().lower() not in ('1', 'true', 'yes'):
            return
        # Only in open modes; escape remains bump/stall only.
        if now < self._scan_cooldown_until:
            return
        if self.mode in (Mode.ESCAPE, Mode.AVOID, Mode.RECOVER):
            return
        # If we're trying to drive forward but the forward cone stays blocked for a bit,
        # do a quick pan sweep and commit toward the best opening.
        if self._proximity.persistently_blocked(now, min_s=1.2):
            self._scan_started = now
            self._scan_until = now + CAL_SWEEP_SEC
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

    def on_tof_front(self, center_m: float, left_m: float = float('nan'),
                     right_m: float = float('nan')) -> None:
        self._proximity.update_front_tof(center_m, left_m, right_m)

    def on_l8_cols(self, cols: list[float]) -> None:
        self._proximity.update_l8_cols(cols)

    def react_corridor(self) -> bool:
        """Two walls visible — hallway follower owns the twist, whatever explore mode."""
        if time.monotonic() < self._scan_until:
            return False
        return self._proximity.in_corridor()

    def allows_nav_blend(self) -> bool:
        """Nav2 may nudge cruise — never during escape/recover/avoid."""
        return self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST)

    def allows_wheel_stall_check(self) -> bool:
        return self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST)

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
        return flips >= 6

    def _recover_steer(self) -> float:
        return min(0.26, max(_max_steer(), _max_steer() * 1.35))

    def _recover_turned_deg(self) -> float:
        return abs(math.degrees(self._yaw_integrated - self._recover_yaw_start))

    def _recover_peel(self, now: float, *, note: str) -> None:
        """Peel using nose L8 / side ToF. Do not sit and scan the tail."""
        sign, confident = self._proximity.live_opening_sign()
        if not confident:
            sign = self._proximity.turn_away_from_closest()
        if sign == 0.0:
            sign = self._proximity.l8_opening_sign()
        if sign == 0.0:
            sign = -1.0  # last resort: peel right, not a coin-flip each recover
        self._recover_need_scan = False
        self._recover_did_scan = True
        self._scan_until = 0.0
        self._recover_sign = sign
        self._proximity.commit_steer_away(sign, 6.0)
        self._recover_phase = RecoverPhase.DRIVE_OUT
        self._recover_driveout_until = now + 4.0
        self.mode_until = self._recover_driveout_until
        self._mode_dur = 4.0
        self._recover_yaw_start = self._yaw_integrated
        self._recover_yaw_need = 60.0
        side = 'left' if sign > 0 else 'right'
        self._log(f'recover: {note} — peel {side} (no rear scan)')

    def _begin_recover_scan(self, now: float) -> None:
        self._recover_peel(now, note='opening unclear')

    def _start_recover(self, now: float, *, reason: str) -> None:
        if now < self._recover_cooldown_until:
            if not self._memory.heading_blocked(self._yaw_integrated):
                return
        self._recover_reason = reason
        self._recover_driveout_until = 0.0
        self._recover_reverse_until = 0.0
        self._recover_did_scan = False
        self._scan_until = 0.0
        self.mode = Mode.RECOVER
        self._proximity.clear_blocked()
        self._recover_cooldown_until = now + 2.5
        self._recover_yaw_start = self._yaw_integrated
        self._recover_yaw_need = 60.0

        nose = self._proximity.is_nose_jammed()
        note = self._memory.mark_blocked(self._yaw_integrated, reason=reason)
        if note:
            self._log(note)
        hip = self._proximity.turn_away_from_closest()
        sign, confident = self._proximity.live_opening_sign()
        if hip != 0.0:
            sl = self._proximity._side_clear('left')
            sr = self._proximity._side_clear('right')
            pinched = (
                (not math.isnan(sl) and sl < 0.40)
                or (not math.isnan(sr) and sr < 0.40)
            )
            if pinched:
                sign = hip
                confident = True
        if not confident:
            ir_sign = self._ir_away_sign()
            if ir_sign != 0.0:
                sign = ir_sign
                confident = True
        if not confident:
            guessed, mem_msg = self._memory.suggest_sign(self._yaw_integrated)
            if guessed != 0.0:
                sign, confident = guessed, True
                if mem_msg:
                    self._log(mem_msg)
        self._recover_sign = sign if confident else hip if hip != 0.0 else 0.0
        if self._recover_sign != 0.0:
            confident = True
        self._recover_need_scan = not confident
        if confident:
            self._proximity.commit_steer_away(self._recover_sign, 6.0)

        if nose:
            # Nose in the wall — back off straight. Don't steer in reverse
            # (that swings the tail into the same wall). Abort if stall fires.
            self._recover_phase = RecoverPhase.REVERSE
            self._recover_reverse_until = now + 1.2
            self.mode_until = self._recover_reverse_until
            self._mode_dur = 1.2
            side = 'scan' if self._recover_sign == 0.0 else (
                'left' if self._recover_sign > 0 else 'right'
            )
            self._log(f'recover ({reason}): reverse then peel {side}')
            return

        if not confident:
            self._recover_peel(now, note=f'{reason} (nose clear, sides unclear)')
            return

        # Known opening — turn until the nose is actually open (~60°), not 18°.
        self._recover_phase = RecoverPhase.DRIVE_OUT
        self._recover_driveout_until = now + 4.0
        self.mode_until = self._recover_driveout_until
        self._mode_dur = 4.0
        side = 'left' if self._recover_sign > 0 else 'right'
        self._log(f'recover ({reason}): peel {side} until clear')

    def _tick_recover(self, now: float) -> tuple[float, float]:
        if self._recover_phase == RecoverPhase.REVERSE:
            # Stall or rear IR while reversing = we hit something behind.
            if self._stall_pending or self._rear_ir_hit():
                why = 'rear IR' if self._rear_ir_hit() else 'stall'
                self._log(f'recover: {why} in reverse — stop backing')
                self._recover_peel(now, note=f'{why} in reverse')
                return 0.0, 0.0
            if now < self._recover_reverse_until:
                turn = 0.0 if self._recover_sign == 0.0 else self._recover_sign * 0.05
                return -max(0.10, min(0.14, _reverse_power())), turn
            if self._recover_need_scan or self._recover_sign == 0.0:
                self._recover_peel(now, note='reverse done')
                return 0.0, 0.0
            self._recover_phase = RecoverPhase.DRIVE_OUT
            self._recover_driveout_until = now + 4.0
            self.mode_until = self._recover_driveout_until
            self._mode_dur = 4.0
            self._recover_yaw_start = self._yaw_integrated
            return 0.0, 0.0

        if self._recover_phase == RecoverPhase.SCAN:
            self._recover_peel(now, note='leave parked scan')
            return 0.0, 0.0

        # DRIVE_OUT: keep turning until yaw and L8 agree the couch is no longer ahead.
        if self._recover_sign != 0.0:
            jammed = self._proximity.is_nose_jammed()
            turned = self._recover_turned_deg()
            open_enough = self._proximity.path_clear_to_drive()
            hip = self._proximity.turn_away_from_closest()
            if hip != 0.0:
                sl = self._proximity._side_clear('left')
                sr = self._proximity._side_clear('right')
                if (not math.isnan(sl) and sl < 0.40) or (not math.isnan(sr) and sr < 0.40):
                    self._recover_sign = hip
            if jammed:
                # ESP zeros forward when the nose is < 24 cm, and then ignores
                # turn. Reverse is the only command that still moves.
                return -max(0.10, min(0.14, _reverse_power())), self._recover_sign * 0.05
            ang = self._recover_sign * min(0.05, self._recover_steer())
            lin = max(0.11, min(_max_linear() * 0.85, 0.12))
            done = (turned >= self._recover_yaw_need and open_enough) or (
                now >= self._recover_driveout_until and turned >= 40.0 and open_enough
            )
            if done:
                self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
                self._memory.start_leg()
                self._recover_phase = RecoverPhase.REVERSE
                self._recover_driveout_until = 0.0
                self._recover_sign = 0.0
                self._recover_need_scan = False
                self._log(f'recover: heading clear after {turned:.0f}°')
                return self._cruise_speed, self._cruise_curve
            if now >= self._recover_driveout_until and not open_enough:
                self._recover_driveout_until = now + 2.0
                self.mode_until = self._recover_driveout_until
                if turned >= 70.0:
                    # Keep peeling away from the closer hip. Do not flip into the wall.
                    if hip != 0.0:
                        self._recover_sign = hip
                    self._recover_yaw_start = self._yaw_integrated
                    self._proximity.commit_steer_away(self._recover_sign, 6.0)
                    self._log('recover: still closed — keep peeling away from the wall')
            return lin, ang

        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
        self._memory.start_leg()
        self._recover_phase = RecoverPhase.REVERSE
        self._recover_driveout_until = 0.0
        self._recover_sign = 0.0
        self._recover_need_scan = False
        return self._cruise_speed, self._cruise_curve

    def _arc_yaw_turned_deg(self) -> float:
        return abs(math.degrees(self._yaw_integrated - self._arc_yaw_start))

    def _front_blocked(self) -> bool:
        return self._ir_fresh() and (self._ir_fl or self._ir_fr)

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
            if self._proximity.in_corridor() or random.random() < 0.82:
                self._cruise_curve = 0.0
            else:
                self._cruise_curve = random.choice([-1, 1]) * random.uniform(0.01, steer * 0.18)
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
        self._escape_reverse_started = now
        self.mode = Mode.ESCAPE
        self._ir_wait_clear = False
        self._log(f'escape ({reason}): reverse → arc (IMU-limited)')
        note = self._memory.mark_blocked(self._yaw_integrated, reason=reason)
        if note:
            self._log(note)

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
        sign, confident = self._proximity.live_opening_sign()
        if confident and sign != 0.0:
            spin_dir = sign
        else:
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
                f'({"left" if spin_dir > 0 else "right"})'
            )
        else:
            self._log(
                f'escape arc {spin_deg:.0f}° '
                f'({"left" if spin_dir > 0 else "right"})'
            )
        self._start_escape_arc(now, spin_dir * steer, duration, spin_deg)

    def _finish_escape_arc(self, now: float) -> None:
        self._event_ignore_until = now + EVENT_COOLDOWN_SEC
        self.straight_block_until = now + STRAIGHT_BLOCK_SEC
        sign, confident = self._proximity.live_opening_sign()
        if confident and sign != 0.0:
            self._proximity.commit_steer_away(sign, 1.5)
        self._enter(Mode.CRUISE, random.uniform(5.0, 9.0))
        self._memory.start_leg()
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
            want_forward=self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST)
        )
        self._maybe_start_scan(now)

        if _front_ir_stop() and self.mode not in (Mode.ESCAPE, Mode.AVOID):
            if self._front_blocked() and now >= self._ir_front_cooldown_until:
                self._trigger_front_avoid(now)

        if self._bump_pending or self._stall_pending:
            stall_in_recover_reverse = (
                self.mode == Mode.RECOVER
                and self._recover_phase == RecoverPhase.REVERSE
                and self._stall_pending
                and not self._bump_pending
            )
            if not stall_in_recover_reverse:
                self._trigger_escape()

        # Recover only when nose is in the wall AND wheels are not making progress.
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST) and now >= self._event_ignore_until:
            if (
                not self._proximity.in_corridor()
                and self._proximity.is_nose_jammed()
                and not self._wheels_moving
                and self._wheel_travel_m < 0.06
            ):
                self._start_recover(now, reason="l8 close")

        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST) and now >= self.mode_until:
            if not self._proximity.in_corridor():
                self._next_mode()

        lin = 0.0
        ang = 0.0
        front_blocked = _front_ir_stop() and self._front_blocked()

        # During scan: coast to stop (slew below) and let the pan sweep.
        if now < self._scan_until:
            self._sync_bg_radar()
            lin, ang = 0.0, 0.0
        elif self.mode == Mode.CRUISE:
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

        scanning = now < self._scan_until
        in_corridor = self.react_corridor()
        cruise_modes = self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST)
        steer_commit = 0.0
        memory_ang = 0.0
        radar_ang = 0.0
        if (not scanning) and cruise_modes:
            if not self._proximity.along_wall():
                steer_commit = self._proximity.steer_commit_active(now)
                memory_ang = self._memory.cruise_bias(self._yaw_integrated, _max_steer())
            if (
                self._bg_radar
                and not self._proximity.along_wall()
                and not in_corridor
            ):
                radar_ang = self._bg_radar.steer_bias()
            if not in_corridor:
                lin, ang = self._apply_front_stop(lin, ang, front_blocked)

        self._motion_hints = ExploreMotionHints(
            allow_shaping=(not scanning) and cruise_modes,
            in_corridor=in_corridor,
            along_wall=self._proximity.along_wall(),
            sonar_front=_sonar_front(),
            steer_commit=steer_commit,
            memory_ang=memory_ang,
            radar_ang=radar_ang,
            front_blocked=front_blocked,
        )

        # Scan finished: commit toward a real opening, never a coin-flip.
        if self._scan_until > 0.0 and now >= self._scan_until and (now - self._scan_started) < 6.0:
            sign = self._proximity.opening_sign(self._scan_best_deg, self._scan_best_m)
            sign, mem_msg = self._memory.prefer_sign(self._yaw_integrated, sign)
            if mem_msg:
                self._log(mem_msg)
            self._proximity.commit_steer_away(sign, 4.0)
            side = 'left' if sign > 0 else 'right'
            if self.mode == Mode.RECOVER and self._recover_phase == RecoverPhase.SCAN:
                self._recover_phase = RecoverPhase.DRIVE_OUT
                self._recover_sign = sign
                self._recover_driveout_until = now + 2.8
                self.mode_until = self._recover_driveout_until
                self._mode_dur = 2.8
                self._log(
                    f'scan done → drive-out {side} '
                    f'(best {self._scan_best_m:.2f}m @ {self._scan_best_deg:.0f}° '
                    f'seen L={self._scan_seen_left} R={self._scan_seen_right})'
                )
            else:
                self._enter(Mode.CRUISE, random.uniform(4.0, 6.0))
                self._log(
                    f'scan done → commit {side} '
                    f'(best {self._scan_best_m:.2f}m @ {self._scan_best_deg:.0f}° '
                    f'seen L={self._scan_seen_left} R={self._scan_seen_right})'
                )
            self._scan_until = 0.0

        # Avoid pivot-style spins: as we slow down, clamp angular to stay arc-like.
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST):
            max_ang = min(_max_steer(), 0.08 + abs(lin) * 1.35)
            ang = max(-max_ang, min(max_ang, ang))
        elif self.mode == Mode.RECOVER:
            max_ang = self._recover_steer()
            ang = max(-max_ang, min(max_ang, ang))

        if self.mode != Mode.ESCAPE:
            self._driving_linear = lin
        lin = _cap_linear(lin, burst=self.mode == Mode.BURST)
        ang = _cap_angular(ang)
        self._out_lin = lin
        self._out_ang = ang
        self._last_driving = lin > 0.05
        dt = 0.05  # brain tick period (rover_brain.TICK)
        if self.mode in (Mode.CRUISE, Mode.WANDER, Mode.BURST) or (
            self.mode == Mode.RECOVER and self._recover_phase == RecoverPhase.DRIVE_OUT
        ):
            self._memory.note_progress(
                lin, dt, self._yaw_integrated, front_open=self._proximity.front_open(),
                travel_m=self._wheel_travel_m if self._wheel_travel_m > 0.0 else None,
            )
            self._wheel_travel_m = 0.0
        self._sync_bg_radar()
        return lin, ang

    def _tick_avoid(self, now: float, front_blocked: bool) -> tuple[float, float]:
        lin = 0.0
        ang = 0.0
        if self.avoid_phase == AvoidPhase.REVERSE:
            if self._rear_ir_hit():
                self._log('front IR avoid: rear IR — stop reverse, peel')
                self.avoid_phase = AvoidPhase.DRIVE_AWAY
                self.avoid_phase_until = now + FRONT_DRIVE_AWAY_SEC
                return 0.0, 0.0
            lin = -_reverse_power()
            ang = 0.0
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
            ang = 0.0
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
            elapsed = now - getattr(self, '_escape_reverse_started', now)
            rear = elapsed >= 0.45 and self._rear_ir_bumper()
            if rear or now >= self.reverse_until:
                if rear:
                    self._log('escape: rear bumper — peel')
                if not self._scan_done:
                    self._run_escape_scan()
            else:
                lin = self.escape_backoff
        elif self.escape_phase == EscapePhase.SCAN:
            if not self._scan_done:
                self._run_escape_scan()
        elif self.escape_phase == EscapePhase.ARC:
            if front_blocked:
                self.escape_phase = EscapePhase.REVERSE
                self.reverse_until = now + ESCAPE_REVERSE_SEC * 0.7
                self._escape_reverse_started = now
                self._scan_done = False
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
        ir_sign = self._ir_away_sign()
        if ir_sign != 0.0:
            self.avoid_steer_dir = ir_sign
        else:
            self.avoid_steer_dir = self._proximity.turn_away_from_closest()
        self.avoid_steer_dir, mem_msg = self._memory.prefer_sign(
            self._yaw_integrated, self.avoid_steer_dir
        )
        if mem_msg:
            self._log(mem_msg)
        note = self._memory.mark_blocked(self._yaw_integrated, reason='front ir')
        if note:
            self._log(note)
        self.avoid_start = now
        self.avoid_phase = AvoidPhase.REVERSE
        self.avoid_phase_until = now + FRONT_AVOID_REVERSE_SEC
        self.mode = Mode.AVOID
        self._ir_front_cooldown_until = now + FRONT_IR_COOLDOWN_SEC
        side = 'head-on' if ir_sign == 0.0 else ('left bumper' if ir_sign < 0 else 'right bumper')
        self._log(f'front IR — reverse then peel ({side})')

    def _finish_front_avoid(self, now: float) -> None:
        self._event_ignore_until = now + 1.0
        self._proximity.commit_steer_away(self.avoid_steer_dir, 4.5)
        self._enter(Mode.CRUISE, random.uniform(4.0, 7.0))
        self._memory.start_leg()
