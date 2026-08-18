#!/usr/bin/env bash
# Repeat straight-line tune runs with minimal delay between them.
# Usage: tune_loop.sh [runs] [seconds] [speed]
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
RUNS="${1:-5}"
SECS="${2:-10}"
SPEED="${3:-0.30}"

export ROVER_TUNE_PREP=1
export ROVER_TUNE_SKIP_STOP=1

systemctl --user stop rover-main.service 2>/dev/null || true
pkill -f 'autonomous_explore.py|rover_session.py' 2>/dev/null || true
sleep 0.2

for ((i = 1; i <= RUNS; i++)); do
  echo ""
  echo "========== tune run $i / $RUNS =========="
  bash "$DIR/go_straight.sh" "$SECS" "$SPEED" || true
  sleep 1
done
