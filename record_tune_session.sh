#!/usr/bin/env bash
# Record /cmd_vel + /imu/data for heading-hold tuning (rosbag2).
# Usage: record_tune_session.sh [seconds] [prep_seconds]
set -uo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
SECS="${1:-25}"
PREP="${2:-15}"
# Pi Zero: small cache + -d duration (timeout kills recorder before flush).
CACHE="${ROVER_BAG_CACHE:-1048576}"

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

OUT="$HOME/rover_tune_$(date +%Y%m%d_%H%M%S)"

_wait_topics() {
  local i
  for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q '^/imu/data$' \
      && ros2 topic list 2>/dev/null | grep -q '^/cmd_vel$'; then
      return 0
    fi
    sleep 0.5
  done
  echo "WARN: /imu/data or /cmd_vel not visible on graph"
  return 1
}

_analyze() {
  if [[ -f "$OUT/imu.jsonl" ]] || [[ -f "$OUT/metadata.yaml" ]]; then
    python3 "$DIR/analyze_tune_bag.py" "$OUT"
  else
    echo "WARN: no capture written — check agent, ESP IMU, disk space."
    ls -la "$OUT" 2>/dev/null || true
    return 1
  fi
}

echo ""
echo "=============================================="
echo "  ROVER TUNE CAPTURE"
echo "  Mac: ./go_mac.sh running, R2 ready"
echo "  Record: ${SECS}s  |  Do NOT press Ctrl+C"
echo "=============================================="
echo ""
bash "$DIR/check_drive_ready.sh" || true
_wait_topics || true
echo ""

while (( PREP > 0 )); do
  echo "  *** GET READY — drive starts in ${PREP}s ***"
  sleep 1
  PREP=$((PREP - 1))
done

for i in 3 2 1; do
  echo ""
  echo "  >>> DRIVE IN ${i} <<<"
  sleep 1
done

echo ""
echo "=============================================="
echo "  >>>>>>  DRIVE STRAIGHT NOW — HOLD R2  <<<<<<"
echo "  >>>>>>       ${SECS} SECONDS           <<<<<<"
echo "=============================================="
echo ""

set +e
python3 "$DIR/record_tune_light.py" -o "$OUT" -d "$SECS"
rc=$?
set -e

echo ""
echo "=============================================="
echo "  STOP — capture finished (exit $rc)"
echo "  Saved: $OUT"
echo "=============================================="
_analyze
