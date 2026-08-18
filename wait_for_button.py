#!/usr/bin/env python3
"""Block until start — ESP GPIO14 (default) or Pi GPIO if ROVER_BUTTON_SOURCE=pi."""
import signal

from rover_esp_button import use_esp_button, wait_for_press as wait_esp


def _interrupted(_signum, _frame) -> None:
    raise SystemExit(128 + int(_signum))


def main() -> int:
    signal.signal(signal.SIGTERM, _interrupted)
    signal.signal(signal.SIGINT, _interrupted)
    if use_esp_button():
        wait_esp()
    else:
        from rover_button import wait_start

        wait_start()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except SystemExit as exc:
        raise
    except KeyboardInterrupt:
        raise SystemExit(130)
