#!/usr/bin/env bash
# Record sonar escape events while driving autonomous (JSONL, Pi-friendly).
# Usage: record_sonar_escape.sh [seconds]
set -uo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
SECS="${1:-120}"
OUT="${ROVER_SONAR_RECORD_DIR:-$HOME/rover_sonar_escape_$(date +%Y%m%d_%H%M%S)}"

set +u
source /opt/ros/jazzy/setup.bash
source ~/uros_agent_ws/install/local_setup.bash 2>/dev/null || true
set -u

export ROVER_SONAR_RECORD_DIR="$OUT"
export ROVER_SONAR_REAR_SCAN="${ROVER_SONAR_REAR_SCAN:-1}"

echo ""
echo "=============================================="
echo "  SONAR ESCAPE RECORDING"
echo "  Duration: ${SECS}s"
echo "  Output:   $OUT"
echo "  Rear IR scan during front sonar escape: $ROVER_SONAR_REAR_SCAN"
echo "=============================================="
echo ""
echo "Start autonomous driving (go_auto.sh) in another terminal if not already."
echo "Drive toward obstacles so the front sonar does escape scans."
echo ""

for i in 5 4 3 2 1; do
  echo "  >>> Recording starts in ${i}s"
  sleep 1
done

echo ""
echo "  >>>>>>  RECORDING ${SECS}s — trigger sonar escapes  <<<<<<"
echo ""

timeout "$SECS" python3 "$DIR/rover_sonar_escape_recorder.py" || true

echo ""
echo "=============================================="
echo "  STOP — saved under $OUT"
echo "=============================================="
python3 "$DIR/analyze_sonar_escape.py" "$OUT" || true
