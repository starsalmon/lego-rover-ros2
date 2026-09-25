"""Persist wheel tick rate vs cmd_vel — used by stall detect."""
from __future__ import annotations

import os
from pathlib import Path


def cal_file_path() -> Path:
    raw = os.environ.get('ROVER_WHEEL_CAL_FILE', '').strip()
    if raw:
        return Path(raw)
    return Path('/opt/fleet/cal/wheel_stall.env')


def load_wheel_cal() -> tuple[float, int]:
    """Return (ticks_per_cmd, sample_count). 0,0 if missing."""
    path = cal_file_path()
    ticks = 0.0
    samples = 0
    if not path.is_file():
        return ticks, samples
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if '=' not in line:
            continue
        key, val = line.split('=', 1)
        key = key.strip()
        val = val.strip()
        try:
            if key == 'ROVER_WHEEL_TICKS_PER_CMD':
                ticks = float(val)
            elif key == 'ROVER_WHEEL_CAL_SAMPLES':
                samples = int(float(val))
        except ValueError:
            continue
    return ticks, samples


def save_wheel_cal(ticks_per_cmd: float, samples: int) -> Path:
    path = cal_file_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    body = (
        f'# Wheel tick stall cal — combined ticks/s per unit cmd magnitude\n'
        f'ROVER_WHEEL_TICKS_PER_CMD={ticks_per_cmd:.3f}\n'
        f'ROVER_WHEEL_CAL_SAMPLES={samples}\n'
    )
    path.write_text(body)
    return path
