#!/usr/bin/env bash
# Mac: sync + Pi sonar escape capture (auto drive + record + analyze).
# Prereq: ESP OTA'd with escape_event topics; Pi on WiFi.
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PI="${ROVER_PI:-cain@lego-rover.local}"
KEY="${ROVER_SSH_KEY:-$HOME/.ssh/lego_rover_cursor}"
SSH_OPTS=(-i "$KEY" -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
SECS="${1:-90}"
PREP="${2:-25}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOCAL_OUT="$DIR/captures/rover_sonar_escape_$STAMP"

echo "=== Sonar escape capture (Mac → Pi) ==="
echo "Record: ${SECS}s  Prep: ${PREP}s"
echo "Pi: $PI"
echo ""
echo "Place rover on floor with obstacles/walls ahead so sonar escapes fire."
echo ""

bash "$DIR/sync_to_pi.sh"

echo ""
echo "--- Waiting for Pi / ESP (flash may still be rebooting) ---"
READY=0
for attempt in 1 2 3 4 5 6 7 8 9 10; do
  if ssh "${SSH_OPTS[@]}" "$PI" 'bash ~/lego-rover-ros2/check_drive_ready.sh --esp-only' 2>/dev/null; then
    READY=1
    break
  fi
  echo "  attempt $attempt/10 — not ready yet (agent/ESP?), retry in 8s..."
  sleep 8
done

if [[ "$READY" -ne 1 ]]; then
  echo ""
  echo "Pi/ESP not ready after waits. Check:"
  echo "  - ESP OTA finished and on WiFi"
  echo "  - ssh $PI 'systemctl status rover-agent.service'"
  exit 1
fi

REMOTE_OUT="rover_sonar_escape_$STAMP"
echo ""
ssh "${SSH_OPTS[@]}" "$PI" \
  "ROVER_SONAR_RECORD_DIR=\$HOME/$REMOTE_OUT bash ~/lego-rover-ros2/record_sonar_escape_session.sh $SECS $PREP" </dev/null

mkdir -p "$LOCAL_OUT"
echo ""
echo "--- Pulling capture to Mac: $LOCAL_OUT ---"
rsync -avz -e "ssh ${SSH_OPTS[*]}" \
  "$PI:~/$REMOTE_OUT/" "$LOCAL_OUT/"

echo ""
python3 "$DIR/analyze_sonar_escape.py" "$LOCAL_OUT" || true
echo ""
echo "Local copy: $LOCAL_OUT"
