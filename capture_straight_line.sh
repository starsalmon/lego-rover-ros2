#!/usr/bin/env bash
# Mac: sync + Pi straight-line IMU capture (heading-hold tune).
# Prereq: ./go_mac.sh running in another terminal.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SSH_OPTS=(-i "$KEY" -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8)
SECS="${1:-30}"
PREP="${2:-10}"

echo "=== Straight-line capture ==="
echo "1) Start ./go_mac.sh in another terminal (if not already)"
echo "2) Place rover on open floor, stick centred"
echo "3) When countdown ends, hold R2 and drive straight — no steering"
echo ""

bash "$DIR/sync_to_pi.sh"

echo ""
echo "--- Pi pre-flight ---"
if ! ssh "${SSH_OPTS[@]}" "$PI" 'bash ~/lego-rover-ros2/check_drive_ready.sh'; then
  echo ""
  echo "WARN: pre-flight failed — fix agent/relay, then re-run."
  echo "  Pi: bash ~/lego-rover-ros2/start_udp_relay.sh"
  echo "  Mac: ./go_mac.sh"
  exit 1
fi

echo ""
ssh -t "${SSH_OPTS[@]}" "$PI" "bash ~/lego-rover-ros2/record_tune_session.sh $SECS $PREP"
