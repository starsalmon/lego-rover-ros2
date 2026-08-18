#!/usr/bin/env bash
# Motor direction diagnostic — run on Pi with rover lifted or on open floor.
# Usage:  bash ~/lego-rover-ros2/go_test_dirs.sh
#         bash ~/lego-rover-ros2/go_test_dirs.sh --only logical
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

echo "=== Drive direction test ==="
echo "  Lift rover or clear floor. Each test: 2s countdown, then ~2.5s move."
echo ""

rover_source_ros

export ROVER_TWIST_SWAP=0
export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_ANGULAR_SIGN="${ROVER_ANGULAR_SIGN:-1}"

systemctl --user stop rover-main.service 2>/dev/null || true
pkill -f 'autonomous_explore.py|wall_follow.py|rover_session.py|straight_drive_tune|test_drive_dirs' 2>/dev/null || true
sleep 0.2
rover_stop_motors 2>/dev/null || true

if ! rover_esp_linked; then
  echo "Waiting for ESP / micro-ROS agent..."
  rover_ensure_agent || exit 1
fi

echo ""
echo "ROVER_LINEAR_SIGN=${ROVER_LINEAR_SIGN}  ROVER_ANGULAR_SIGN=${ROVER_ANGULAR_SIGN}"
echo ""

exec python3 "$DIR/test_drive_dirs.py" --logical "$@"
