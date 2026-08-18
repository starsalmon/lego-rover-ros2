#!/usr/bin/env python3
"""Play a short rover tune when the mini locks onto its IR beacon."""
from __future__ import annotations

import socket
import time

from rover_speaker import play

HOST = "0.0.0.0"
PORT = 4242
MIN_REPEAT_S = 2.0


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    last_play = 0.0
    print(f"rover beacon notifier listening on UDP {PORT}", flush=True)
    while True:
        data, _addr = sock.recvfrom(128)
        if data.strip() != b"beacon_lock":
            continue
        now = time.monotonic()
        if now - last_play >= MIN_REPEAT_S:
            last_play = now
            play("beacon_lock")


if __name__ == "__main__":
    main()
