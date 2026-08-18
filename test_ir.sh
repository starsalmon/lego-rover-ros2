#!/usr/bin/env bash
# IR sensor bench test (front pulse GPIO 6, aux GPIO 26).
# Usage:  bash ~/lego-rover-ros2/test_ir.sh -w
#         ROVER_IR_FRONT_OFF_MS=50 ROVER_IR_FRONT_ON_MS=10 bash ~/lego-rover-ros2/test_ir.sh -w
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

exec python3 "$DIR/test_ir_sensors.py" "$@"
