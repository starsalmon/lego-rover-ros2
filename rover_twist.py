"""Map cmd_vel linear/angular → ROS Twist (standard linear.x / angular.z)."""
from __future__ import annotations

import os

from geometry_msgs.msg import Twist

# Canonical ROS on Pi (linear.x=forward). ESP firmware uncrosses micro-ROS Twist fields.
# Leave ROVER_TWIST_SWAP=0 — do not re-enable unless debugging legacy setups.
_SWAP = os.environ.get('ROVER_TWIST_SWAP', '0').strip().lower() not in ('0', 'no', 'false')
_LINEAR_SIGN = float(os.environ.get('ROVER_LINEAR_SIGN', '1'))
_ANGULAR_SIGN = float(os.environ.get('ROVER_ANGULAR_SIGN', '1'))


def set_twist(msg: Twist, linear: float, angular: float) -> None:
    lin = float(linear) * _LINEAR_SIGN
    ang = float(angular) * _ANGULAR_SIGN
    if _SWAP:
        lin, ang = ang, lin
    msg.linear.x = lin
    msg.angular.z = ang


def wire_to_logical(lin_wire: float, ang_wire: float) -> tuple[float, float]:
    """Undo publish swap for analysis / logging."""
    if _SWAP:
        return ang_wire, lin_wire
    return lin_wire, ang_wire


def twist_swap_enabled() -> bool:
    return _SWAP


def logical_from_wire_angular(wire_ang: float) -> float:
    """Logical angular for set_twist() to produce wire_ang on /cmd_vel (sign is ±1)."""
    if _ANGULAR_SIGN == 0:
        return wire_ang
    return float(wire_ang) * _ANGULAR_SIGN
