#!/usr/bin/env bash
# Wall-follow along right (default) or left side wall.
# Usage:  bash ~/lego-rover-ros2/go_wall.sh
#         ROVER_WALL_SIDE=left bash ~/lego-rover-ros2/go_wall.sh
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

export ROVER_DRIVE_SCRIPT=wall_follow.py
export ROVER_WALL_SIDE="${ROVER_WALL_SIDE:-right}"
export ROVER_WALL_LINEAR="${ROVER_WALL_LINEAR:-0.28}"
export ROVER_WALL_STEER="${ROVER_WALL_STEER:-0.08}"
export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_TWIST_SWAP="${ROVER_TWIST_SWAP:-0}"

echo "=== LEGO Rover — wall follow (${ROVER_WALL_SIDE} wall) ==="
echo "Aux IR locked to side; front IR stops head-on."
echo "Press GPIO button again to stop."
echo ""

pkill -f 'cmd_vel_udp_relay.py' 2>/dev/null || true
pkill -f 'cmd_vel_ssh_relay.py' 2>/dev/null || true
sleep 0.3

if [[ "${ROVER_QUICK_START:-0}" == "1" ]] && rover_esp_linked; then
  echo "Quick start — ESP32 already linked."
elif ! rover_preflight; then
  exit 1
elif ! rover_ensure_agent; then
  exit 1
fi

pkill -f 'wait_for_button.py' 2>/dev/null || true
pkill -f 'rover_session.py' 2>/dev/null || true
if [[ "${ROVER_QUICK_START:-0}" == "1" ]]; then
  sleep 0.1
else
  sleep 0.4
fi

cleanup() {
  rover_stop_motors
  sleep 0.15
  pkill -f 'wall_follow.py|rover_session.py' 2>/dev/null || true
  rover_stop_motors
  rover_ir_close
}
trap cleanup EXIT INT TERM

cd "$DIR"
python3 "$DIR/rover_session.py"
