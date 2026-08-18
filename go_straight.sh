#!/usr/bin/env bash
# Straight-line / IMU heading-hold tune on the Pi.
# Usage:  bash ~/lego-rover-ros2/go_straight.sh [seconds] [speed]
# From Mac: bash ~/lego-rover-ros2/tune_straight_mac.sh
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_TWIST_SWAP=0

exec bash "$DIR/tune_straight.sh" "$@"
