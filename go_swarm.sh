#!/usr/bin/env bash
# Rover side of explore/chase swarm — autonomous + IR beacon on ESP.
# SSH: ssh lego-rover 'bash ~/lego-rover-ros2/go_swarm.sh'
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

echo "=== Swarm: rover explore (Pi + ESP IR beacon) ==="

if ! systemctl --user is-active --quiet rover-agent.service 2>/dev/null; then
  echo "WARN: rover-agent.service not running — starting..."
  systemctl --user start rover-agent.service 2>/dev/null || true
  sleep 2
fi

if rover_esp_linked; then
  echo "ESP32: linked (IR beacon should be on when firmware IR_TX_DEFAULT_ON=1)."
else
  echo "WARN: ESP32 not linked — check UART / rover-agent.service."
fi

echo ""
export ROVER_AUTO_MAX_LINEAR="${ROVER_AUTO_MAX_LINEAR:-0.20}"
export ROVER_AUTO_BURST_MAX="${ROVER_AUTO_BURST_MAX:-0.28}"
export ROVER_AUTO_BURST_PROB="${ROVER_AUTO_BURST_PROB:-0}"
echo "Speed cap: ${ROVER_AUTO_MAX_LINEAR} (burst ${ROVER_AUTO_BURST_MAX}, prob ${ROVER_AUTO_BURST_PROB})"
exec bash "$DIR/go_auto.sh"
