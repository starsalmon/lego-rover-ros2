#!/usr/bin/env bash
# Wheel tick monitor — needs micro-ROS agent + ESP linked.
set -eo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"
rover_source_ros
rover_ensure_agent || exit 1
export ROVER_BRIDGE_EXTERNAL=1
python3 "$DIR/rover_bridge_daemon.py" &
BRIDGE_PID=$!
trap 'kill "$BRIDGE_PID" 2>/dev/null || true' EXIT INT TERM
sleep 1
python3 "$DIR/test_wheel_sensors.py" "$@"
