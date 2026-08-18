#!/usr/bin/env bash
# Sonar radar demo — servo + aux IR + LED ring + pings.
set -eo pipefail
cd "$(dirname "$0")"
exec python3 demo_radar.py "$@"
