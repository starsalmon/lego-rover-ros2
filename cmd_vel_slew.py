"""Rate-limit cmd_vel — smooths step changes without killing responsiveness.

Inspired by teleop_mac_ssh linear slew (bleed-before-reverse), tuned for wander
at 20 Hz. Large jumps ease over ~150–250 ms; stops brake faster than accel.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass


def _env_float(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


@dataclass
class CmdVelSlew:
    """Slew-filter for normalized linear.x / angular.z cmd_vel."""

    lin_accel: float = 0.28
    lin_brake: float = 0.22
    ang_accel: float = 0.08
    ang_brake: float = 0.14
    rest_eps: float = 0.012
    lin_out: float = 0.0
    ang_out: float = 0.0

    @classmethod
    def from_env(cls) -> CmdVelSlew:
        return cls(
            lin_accel=_env_float('CMD_SLEW_LIN_ACCEL', 0.28),
            lin_brake=_env_float('CMD_SLEW_LIN_BRAKE', 0.22),
            ang_accel=_env_float('CMD_SLEW_ANG_ACCEL', 0.08),
            ang_brake=_env_float('CMD_SLEW_ANG_BRAKE', 0.14),
        )

    def reset(self) -> None:
        self.lin_out = 0.0
        self.ang_out = 0.0

    def at_rest(self) -> bool:
        return abs(self.lin_out) < self.rest_eps and abs(self.ang_out) < self.rest_eps

    @staticmethod
    def _slew_scalar(current: float, target: float, dt: float, rate: float) -> float:
        if dt <= 0.0:
            return current
        delta = target - current
        step = rate * dt
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)

    def step(
        self,
        lin_tgt: float,
        ang_tgt: float,
        dt: float,
        *,
        hard_stop: bool = False,
    ) -> tuple[float, float]:
        if hard_stop:
            lin_tgt = 0.0
            ang_tgt = 0.0

        lin_opposite = (self.lin_out > 0.03 and lin_tgt < -0.03) or (
            self.lin_out < -0.03 and lin_tgt > 0.03
        )
        lin_eff = 0.0 if lin_opposite else lin_tgt
        lin_rate = self.lin_brake if (
            hard_stop or lin_opposite or abs(lin_eff) < abs(self.lin_out)
        ) else self.lin_accel
        self.lin_out = self._slew_scalar(self.lin_out, lin_eff, dt, lin_rate)

        ang_opposite = self.ang_out * ang_tgt < 0.0 and abs(self.ang_out) > 0.005
        # Always bleed through zero on a sign flip — never snap left↔right.
        ang_eff = 0.0 if ang_opposite else ang_tgt
        ang_rate = self.ang_brake if (
            hard_stop or ang_opposite or abs(ang_eff) < abs(self.ang_out)
        ) else self.ang_accel
        self.ang_out = self._slew_scalar(self.ang_out, ang_eff, dt, ang_rate)

        if hard_stop and self.at_rest():
            self.lin_out = 0.0
            self.ang_out = 0.0

        return self.lin_out, self.ang_out
