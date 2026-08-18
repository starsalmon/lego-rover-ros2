#!/usr/bin/env bash
# Simple back-and-forth showcase (manual demo).
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros

echo "=== LEGO Rover showcase demo ==="
if ! rover_preflight; then exit 1; fi
if ! rover_ensure_agent; then exit 1; fi

cleanup() {
  pkill -f 'demo_showcase.py' 2>/dev/null || true
  rover_stop_motors
}
trap cleanup EXIT INT TERM

python3 "$DIR/demo_showcase.py"
