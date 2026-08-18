#!/usr/bin/env bash
# Drive with a PS4 DualShock 4. Pair first: bash ~/lego-rover-ros2/pair_ps4.sh
set -eo pipefail

export TELEOP_CONFIG="${TELEOP_CONFIG:-$(dirname "$0")/teleop_ps4.yaml}"

# Reconnect PS4 if we saved the MAC
MAC_FILE="$HOME/.config/lego-rover/ps4_mac"
if [[ -f "$MAC_FILE" ]] && [[ ! -e /dev/input/js0 ]]; then
  MAC=$(cat "$MAC_FILE")
  echo "Reconnecting PS4 at $MAC..."
  sudo bluetoothctl connect "$MAC" 2>/dev/null || true
  sleep 2
fi

exec bash "$(dirname "$0")/run_teleop.sh"
