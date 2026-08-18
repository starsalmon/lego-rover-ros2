#!/usr/bin/env python3
"""Keep /rover/heartbeat + /rover/button bridge alive for rover-main.service."""
from __future__ import annotations

import sys
from pathlib import Path

DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(DIR))

from rover_esp_button import _run_bridge, set_session_active, _write_drive_mode, DRIVE_EXPLORE  # noqa: E402


def main() -> int:
    set_session_active(False)
    _write_drive_mode(DRIVE_EXPLORE)
    _run_bridge()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        set_session_active(False)
        raise SystemExit(0)
