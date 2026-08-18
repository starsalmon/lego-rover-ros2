"""Ring events from autonomous subprocess → parent rover_session."""
from __future__ import annotations

import os
import time

_PATH = os.environ.get('ROVER_RING_IPC', '/tmp/rover_ring_evt')


def emit(name: str) -> None:
    try:
        with open(_PATH, 'w', encoding='utf-8') as f:
            f.write(f'{name} {time.time():.3f}\n')
    except OSError:
        pass


def read_event() -> str | None:
    try:
        with open(_PATH, encoding='utf-8') as f:
            line = f.read().strip()
    except OSError:
        return None
    if not line:
        return None
    return line.split()[0]
