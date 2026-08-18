#!/usr/bin/env bash
# Quick health check before a demo.
set -eo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/rover_common.sh"
rover_source_ros

echo "=== LEGO Rover preflight ==="
echo "User: $USER  Groups: $(groups)"
echo "UART: $SERIAL_DEV"
ls -l "$SERIAL_DEV" 2>/dev/null || echo "  (missing)"
echo ""

if rover_preflight; then
  echo "OK: ready to run demo or teleop."
  echo ""
  echo "  bash ~/lego-rover-ros2/run_demo.sh"
  echo "  bash ~/lego-rover-ros2/run_teleop.sh"
  exit 0
else
  echo ""
  echo "Fix the errors above, then re-run preflight."
  exit 1
fi
