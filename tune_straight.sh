#!/usr/bin/env bash
# Pi-only straight-line heading-hold test (no Mac teleop).
# Usage: tune_straight.sh [seconds] [speed]
#   ROVER_TUNE_PREP=1     countdown before drive (default 1, was 5)
#   ROVER_TUNE_SKIP_STOP=1  don't stop rover-main (faster repeat runs)
# From Mac: bash tune_straight_mac.sh
set -uo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

SECS="${1:-20}"
SPEED="${2:-0.30}"
PREP="${ROVER_TUNE_PREP:-1}"

export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_TWIST_SWAP=0

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: run this on the Pi, not your Mac." >&2
  exit 1
fi

if [[ "${ROVER_TUNE_SKIP_STOP:-0}" != "1" ]]; then
  systemctl --user stop rover-main.service 2>/dev/null || true
  pkill -f 'cmd_vel_udp_relay.py' 2>/dev/null || true
  pkill -f 'autonomous_explore.py|wall_follow.py|rover_session.py' 2>/dev/null || true
  sleep 0.15
  rover_stop_motors 2>/dev/null || true
  rover_ir_close
fi

OUT="$HOME/rover_tune_$(date +%Y%m%d_%H%M%S)"

echo ""
echo "=============================================="
echo "  HEADING-HOLD STRAIGHT TEST"
echo "  Speed: ${SPEED}  Duration: ${SECS}s  Prep: ${PREP}s"
echo "  ROVER_LINEAR_SIGN=${ROVER_LINEAR_SIGN}  ROVER_TWIST_SWAP=${ROVER_TWIST_SWAP}"
echo "=============================================="
echo ""

if rover_esp_linked; then
  echo "ESP32 already linked — skipping agent wait."
elif ! rover_ensure_agent; then
  echo "ERROR: micro-ROS agent not running." >&2
  exit 1
fi

while (( PREP > 0 )); do
  echo "  Place on open floor — starting in ${PREP}s"
  sleep 1
  PREP=$((PREP - 1))
done

echo ""
echo "  >>> DRIVING STRAIGHT NOW <<<"
echo ""

python3 "$DIR/record_tune_light.py" -o "$OUT" -d "$SECS" &
REC_PID=$!
sleep 0.15
python3 "$DIR/straight_drive_tune.py" -d "$SECS" -s "$SPEED"
wait "$REC_PID" 2>/dev/null || true
rover_stop_motors

echo ""
echo "=============================================="
echo "  STOP — saved: $OUT"
echo "=============================================="
python3 "$DIR/analyze_tune_bag.py" "$OUT"
