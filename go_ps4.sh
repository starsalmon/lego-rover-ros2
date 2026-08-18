#!/usr/bin/env bash
# One-shot PS4 teleop. Agent stays up between runs.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

echo "=== LEGO Rover PS4 teleop ==="

if ! python3 -c 'import os; assert os.path.exists("/dev/input/js0") or __import__("glob").glob("/dev/input/by-id/*Joystick*")' 2>/dev/null; then
  echo "ERROR: no joystick at /dev/input/js0 — plug in PS4 USB cable"
  exit 1
fi

if ! rover_preflight; then
  exit 1
fi

# Reconnect saved PS4 (BT sometimes drops on Pi Zero 2W)
MAC_FILE="$HOME/.config/lego-rover/ps4_mac"
if [[ -f "$MAC_FILE" ]]; then
  MAC=$(cat "$MAC_FILE")
  echo "Reconnecting PS4 at $MAC..."
  sudo bluetoothctl connect "$MAC" 2>/dev/null || true
  sleep 2
fi

echo "[1/2] Ensuring micro-ROS link..."
if ! rover_ensure_agent; then
  tail -10 /tmp/rover_agent.log 2>/dev/null || true
  exit 1
fi

cleanup() {
  pkill -f teleop_ps4.py 2>/dev/null || true
  rover_stop_motors
}
trap cleanup EXIT INT TERM

echo "[2/2] PS4 teleop — hold L1 + left stick (R1 = turbo)"
echo "      Pair first if needed: bash ~/lego-rover-ros2/pair_ps4.sh"
echo ""
cd "$DIR"
python3 "$DIR/teleop_ps4.py"
