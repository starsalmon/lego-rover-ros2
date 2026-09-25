"""Single motion pipeline — one place shapes cruise cmd_vel from sensors.

Explore produces mode intent (cruise / escape / recover speeds).
Brain calls shape_explore_cmd() once per tick, then CmdVelSlew once.
"""
from __future__ import annotations

from dataclasses import dataclass

from rover_proximity import ProximityState

MIN_LIN_FOR_STEER = 0.06


@dataclass
class ExploreMotionHints:
    allow_shaping: bool
    in_corridor: bool
    along_wall: bool
    sonar_front: bool
    steer_commit: float = 0.0
    memory_ang: float = 0.0
    radar_ang: float = 0.0
    front_blocked: bool = False


def arc_not_spin(lin: float, ang: float) -> tuple[float, float]:
    if abs(ang) <= 0.02 or abs(lin) < 0.02 or abs(lin) >= MIN_LIN_FOR_STEER:
        return lin, ang
    creep = MIN_LIN_FOR_STEER if lin >= 0 else -MIN_LIN_FOR_STEER
    return creep, ang


def shape_explore_cmd(
    prox: ProximityState,
    lin: float,
    ang: float,
    *,
    hints: ExploreMotionHints,
    max_lin: float,
    max_steer: float,
) -> tuple[float, float]:
    """Apply sensor shaping in fixed priority order — no duplicate passes."""
    if not hints.allow_shaping:
        return prox.cap_forward_l8(lin, ang, max_steer=max_steer)

    if abs(hints.steer_commit) >= 0.02:
        ang = max(-max_steer, min(max_steer, ang + hints.steer_commit * max_steer * 0.22))
    if abs(hints.memory_ang) >= 0.02:
        ang = max(-max_steer, min(max_steer, ang + hints.memory_ang))
        lin = min(lin, max_lin * 0.60)
    if abs(hints.radar_ang) >= 0.001:
        ang = max(-max_steer, min(max_steer, ang + hints.radar_ang))

    if hints.in_corridor:
        lin, ang = prox.corridor_drive(max_lin)
        # Corridor: forward bias, tiny steer — do not layer L8 gap-steer or repel.
        return lin, ang
    else:
        lin, ang = prox.repel_overlay(lin, ang, max_steer=max_steer)
        if not prox.must_pivot():
            lin, ang = arc_not_spin(lin, ang)

    lin, ang = prox.cap_forward_l8(lin, ang, max_steer=max_steer)
    lin, ang = prox.cap_turn_l8(lin, ang, max_steer=max_steer)

    if hints.sonar_front and not hints.in_corridor:
        lin, ang = prox.apply(lin, ang, max_steer=max_steer)

    return lin, ang
