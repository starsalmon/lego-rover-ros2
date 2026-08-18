#!/usr/bin/env bash
# Mac: sync + run heading-hold straight test on the Pi.
# Usage: tune_straight_mac.sh [seconds] [speed]
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SECS="${1:-20}"
SPEED="${2:-0.40}"

bash "$DIR/sync_to_pi.sh"
exec ssh -t -i "$KEY" -o StrictHostKeyChecking=accept-new "$PI" \
  "bash ~/lego-rover-ros2/go_straight.sh ${SECS} ${SPEED}"
