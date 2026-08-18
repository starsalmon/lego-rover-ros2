"""ESP aux servo — angle requests via ROS bridge file handoff."""
from __future__ import annotations

import os
from pathlib import Path

SERVO_ANGLE_FILE = Path(os.environ.get('ROVER_SERVO_ANGLE_FILE', '/tmp/rover_servo_angle'))


def use_esp_servo() -> bool:
    src = os.environ.get('ROVER_SERVO_SOURCE', 'esp').strip().lower()
    return src in ('esp', 'esp32', '1', 'true', 'yes')


def publish_angle(deg: float) -> None:
    """Queue one servo angle for rover_esp_bridge to publish on /rover/servo/angle."""
    SERVO_ANGLE_FILE.write_text(f'{float(deg):.2f}')
