#!/usr/bin/env bash
# Autonomous smooth-drive showcase. Run on the Pi with ESP32 on UART.
set -eo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/rover_common.sh"
rover_source_ros

if ! rover_preflight; then
  exit 1
fi

bash "$(dirname "$0")/rover_kill.sh" --keep-agent

DEMO_MODE="${DEMO_MODE:-showcase}"  # showcase | classic

if ! rover_esp_linked; then
  rover_start_agent
  AGENT_PID=$!
  if ! rover_wait_for_esp 45; then
    kill "$AGENT_PID" 2>/dev/null || true
    exit 1
  fi
else
  AGENT_PID=$(pgrep -f 'micro_ros_agent serial' | head -1 || true)
  echo "Using existing agent (pid ${AGENT_PID:-?}) + ESP32 link."
fi

echo "Test pulse to ESP (reliable QoS)..."
ros2 topic pub --once --qos-reliability reliable \
  /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}}" 2>/dev/null || true
sleep 1

case "$DEMO_MODE" in
  showcase)
    DEMO_SCRIPT="$ROVER_DIR/demo_showcase.py"
    ;;
  classic)
    DEMO_SCRIPT="$ROVER_DIR/demo_smooth_drive.py"
    ;;
  *)
    echo "Unknown DEMO_MODE=$DEMO_MODE (use showcase or classic)"
    exit 1
    ;;
esac

cleanup() {
  pkill -f 'demo_showcase.py|demo_smooth_drive.py' 2>/dev/null || true
  rover_stop_motors
}
trap cleanup EXIT INT TERM

echo ""
echo "=== DEMO RUNNING ==="
echo "  Terminal goes quiet — that is normal."
echo "  Rover loops: 5s forward, 5s back."
echo "  Ctrl+C to stop, or: bash ~/lego-rover-ros2/stop_rover.sh"
echo ""

python3 "$DEMO_SCRIPT"
