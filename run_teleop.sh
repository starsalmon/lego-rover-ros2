#!/usr/bin/env bash
# Drive with a Bluetooth gamepad (PS4 / Xbox / Switch).
set -eo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/rover_common.sh"
rover_source_ros

if ! rover_preflight; then
  exit 1
fi

if ! test -e /dev/input/js0 && ! ls /dev/input/by-id/*Joystick* &>/dev/null; then
  echo "ERROR: no joystick — plug PS4 into Pi USB (or pair BT)"
  echo "  Check: ls /dev/input/js*"
  exit 1
fi

CONFIG="${TELEOP_CONFIG:-$ROVER_DIR/teleop_ps4.yaml}"
export TELEOP_CONFIG="$CONFIG"

rover_start_agent
AGENT_PID=$!

cleanup() {
  kill "$AGENT_PID" 2>/dev/null || true
  pkill -f teleop_ps4.py 2>/dev/null || true
  rover_stop_motors
}
trap cleanup EXIT INT TERM

echo "Starting teleop (hold L1 / enable button + left stick)..."
echo "Config: $CONFIG"
cd "$ROVER_DIR"
python3 "$ROVER_DIR/teleop_ps4.py"
