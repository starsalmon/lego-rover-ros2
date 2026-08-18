#!/usr/bin/env bash
# Autonomous explore — main rover mode.
# Requires: rover-agent.service + ESP32 firmware with heading/bump/stall.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

ROVER_SESSION_LOCK="${ROVER_SESSION_LOCK:-/tmp/rover_autonomous.lock}"
exec 9>"$ROVER_SESSION_LOCK"
if ! flock -n 9; then
  echo "Another autonomous session is already running (lock: $ROVER_SESSION_LOCK)."
  echo "If stuck: rm -f $ROVER_SESSION_LOCK and retry."
  exit 1
fi

echo "=== LEGO Rover — autonomous explore ==="

if [[ "${ROVER_QUICK_START:-0}" == "1" ]] && rover_esp_linked; then
  echo "Quick start — ESP32 already linked."
elif ! rover_preflight; then
  exit 1
elif ! rover_ensure_agent; then
  tail -10 /tmp/rover_agent.log 2>/dev/null || true
  exit 1
fi

# Release GPIO 17 if a crashed session still holds it.
# Do not kill wait_for_button when rover-main.service is managing standby — that
# makes rover_main think Go was pressed and starts a second go.sh (SSH session dies).
if [[ "${ROVER_SKIP_WAIT_KILL:-0}" != "1" ]]; then
  pkill -f 'wait_for_button.py' 2>/dev/null || true
fi
pkill -f 'rover_session.py' 2>/dev/null || true
if [[ "${ROVER_QUICK_START:-0}" == "1" ]]; then
  sleep 0.1
else
  sleep 0.25
fi

cleanup() {
  rover_stop_motors
  sleep 0.15
  pkill -f 'autonomous_explore.py|rover_session.py' 2>/dev/null || true
  pkill -f 'demo_showcase.py' 2>/dev/null || true
  pkill -f 'rover_button.py watch' 2>/dev/null || true
  rover_stop_motors
  rover_ir_close
}
trap cleanup EXIT INT TERM

echo "ESP32: heading hold, MPU bump + stall"
echo "Pi: front IR hard stop, MPU escape, radar, optional wheel odom log"
echo "Press GPIO button again to stop (or Ctrl+C over SSH)"
echo ""

cd "$DIR"
python3 "$DIR/rover_session.py"
