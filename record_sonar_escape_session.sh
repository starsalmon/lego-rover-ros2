#!/usr/bin/env bash
# Pi: autonomous drive + sonar escape recorder + analyze (one shot).
# Usage: record_sonar_escape_session.sh [record_seconds] [prep_seconds]
set -uo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
SECS="${1:-90}"
PREP="${2:-20}"
OUT="${ROVER_SONAR_RECORD_DIR:-$HOME/rover_sonar_escape_$(date +%Y%m%d_%H%M%S)}"
AUTO_LOG="/tmp/rover_sonar_auto_$$.log"

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

export ROVER_SONAR_RECORD_DIR="$OUT"
export ROVER_SONAR_REAR_SCAN="${ROVER_SONAR_REAR_SCAN:-1}"
export ROVER_SONAR="${ROVER_SONAR:-1}"
export ROVER_AUTO_MAX_LINEAR="${ROVER_AUTO_MAX_LINEAR:-0.23}"
export ROVER_CRUISE_RADAR="${ROVER_CRUISE_RADAR:-1}"
export ROVER_QUICK_START=1
export ROVER_SKIP_WAIT_KILL=1
# Capture runs without rover-main; start our own ESP bridge for aux IR requests.
unset ROVER_BRIDGE_EXTERNAL

_bridge_running() {
  pgrep -f 'rover_bridge_daemon.py' >/dev/null 2>&1
}

_dedupe_esp_bridge() {
  local count
  count=$(pgrep -fc 'rover_bridge_daemon.py' 2>/dev/null || echo 0)
  if [[ "$count" -gt 1 ]]; then
    echo "WARN: $count rover_bridge_daemon copies — resetting to one"
    pkill -9 -f 'rover_bridge_daemon.py' 2>/dev/null || true
    sleep 1
  fi
}

_start_esp_bridge() {
  _dedupe_esp_bridge
  if _bridge_running; then
    echo "ESP bridge: already running"
    BRIDGE_STARTED_BY_US=0
    return 0
  fi
  echo "Starting rover_esp_bridge (aux IR + servo file bridge)..."
  python3 "$DIR/rover_bridge_daemon.py" >>"$OUT/bridge.log" 2>&1 &
  BRIDGE_PID=$!
  BRIDGE_STARTED_BY_US=1
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    sleep 0.3
    if _bridge_running; then
      echo "ESP bridge: OK (pid $BRIDGE_PID)"
      return 0
    fi
  done
  echo "WARN: rover_esp_bridge did not appear — rear IR scans may fail"
  return 1
}

cleanup() {
  echo ""
  echo "Stopping autonomous + recorder..."
  pkill -f 'rover_sonar_escape_recorder.py' 2>/dev/null || true
  pkill -f 'autonomous_explore.py' 2>/dev/null || true
  pkill -f 'rover_sonar_ring.py' 2>/dev/null || true
  if [[ -n "${BRIDGE_PID:-}" ]] && [[ "${BRIDGE_STARTED_BY_US:-0}" == 1 ]]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
  fi
  timeout 3 bash "$DIR/rover_kill.sh" --keep-agent 2>/dev/null || true
  pkill -9 -f 'autonomous_explore.py|rover_sonar_escape_recorder.py|rover_sonar_ring.py' 2>/dev/null || true
  for _ in 1 2 3; do
    timeout 2 ros2 topic pub --once --no-wait-for-subscribers /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo ""
echo "=============================================="
echo "  SONAR ESCAPE SESSION"
echo "  Prep:     ${PREP}s  (place rover, clear path)"
echo "  Record:   ${SECS}s  (auto drives + logs escapes)"
echo "  Output:   $OUT"
echo "=============================================="
echo ""

bash "$DIR/check_drive_ready.sh" --esp-only || {
  echo "Pre-flight failed — is ESP linked and agent up?"
  exit 1
}

while (( PREP > 0 )); do
  echo "  *** GET READY — auto starts in ${PREP}s (clear floor, obstacles ahead) ***"
  sleep 1
  PREP=$((PREP - 1))
done

mkdir -p "$OUT"
rm -f /tmp/rover_autonomous.lock 2>/dev/null || true
pkill -f 'autonomous_explore.py|rover_session.py|rover_sonar_ring.py|rover_sonar_escape_recorder.py' 2>/dev/null || true
sleep 0.5

BRIDGE_PID=""
BRIDGE_STARTED_BY_US=0
_start_esp_bridge || true

echo "Starting recorder → $OUT"
python3 "$DIR/rover_sonar_escape_recorder.py" >"$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 1.5

echo "Starting autonomous explore (log: $AUTO_LOG)"
python3 "$DIR/autonomous_explore.py" >"$AUTO_LOG" 2>&1 &
AUTO_PID=$!
python3 "$DIR/rover_sonar_ring.py" >>"$AUTO_LOG" 2>&1 &
RING_PID=$!

echo ""
echo "  >>>>>>  DRIVING + RECORDING ${SECS}s — rover should wander into obstacles  <<<<<<"
echo ""

sleep "$SECS"

echo ""
echo "=============================================="
echo "  SESSION DONE — analyzing $OUT"
echo "=============================================="
python3 "$DIR/analyze_sonar_escape.py" "$OUT" || true
echo ""
echo "Full log: $OUT"
echo "Auto log: $AUTO_LOG"
