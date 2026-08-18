#!/usr/bin/env bash
# Aux servo sweep + optional pulsed IR ranging.
# Usage:  bash ~/lego-rover-ros2/test_servo_sweep.sh --center
#         bash ~/lego-rover-ros2/test_servo_sweep.sh --scan --delta
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

exec python3 "$DIR/test_servo_sweep.py" "$@"
