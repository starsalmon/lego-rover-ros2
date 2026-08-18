#!/usr/bin/env bash
# Quick pre-flight before teleop / tune capture.
#   check_drive_ready.sh           — teleop (needs UDP relay)
#   check_drive_ready.sh --esp-only — straight tune / auto (relay optional)
set -uo pipefail

ESP_ONLY=0
if [[ "${1:-}" == "--esp-only" ]]; then
  ESP_ONLY=1
fi

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

# pgrep -c prints 0 and exits 1 when nothing matches — do not use `|| echo 0` (yields "0\n0").
RELAYS=$(pgrep -cf 'python3.*cmd_vel_udp_relay' 2>/dev/null || true)
RELAYS=${RELAYS:-0}
ESP=$(ros2 node list 2>/dev/null | grep -c lego_rover_esp32 || true)
ESP=${ESP:-0}
LAST=$(cat /tmp/cmd_vel_udp_last.txt 2>/dev/null || echo "none")

echo "--- drive ready ---"
echo "ESP32 node:     $([ "$ESP" -ge 1 ] && echo OK || echo MISSING — check agent / USB)"
if [[ "$ESP_ONLY" -eq 1 ]]; then
  echo "UDP relays:     $RELAYS (not required for straight tune)"
else
  echo "UDP relays:     $RELAYS $([ "$RELAYS" -eq 1 ] && echo OK || echo "(want 1 — run: bash ~/lego-rover-ros2/start_udp_relay.sh)")"
  echo "Last UDP cmd:   $LAST $([ "$LAST" != "none" ] && [ "$LAST" != "0.0000 0.0000" ] && echo '(teleop active)' || echo '(start ./go_mac.sh on Mac, hold R2)')"
fi

if [[ "$ESP" -lt 1 ]]; then
  exit 1
fi
if [[ "$ESP_ONLY" -eq 0 ]] && [[ "$RELAYS" -ne 1 ]]; then
  exit 1
fi
