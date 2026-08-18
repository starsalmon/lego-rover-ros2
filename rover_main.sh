#!/usr/bin/env bash
# Boot standby loop: wait for button → run autonomous → repeat.
# Install: bash ~/lego-rover-ros2/install_rover_boot.sh
set -eo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$DIR/rover_common.sh"

export ROVER_BUTTON_SOURCE="${ROVER_BUTTON_SOURCE:-esp}"
export ROVER_SPEAKER_GPIO="${ROVER_SPEAKER_GPIO:-17}"
export ROVER_SPEAKER_VOL="${ROVER_SPEAKER_VOL:-0.38}"
export ROVER_RING_GPIO="${ROVER_RING_GPIO:-18}"
export ROVER_AUTO_MAX_LINEAR="${ROVER_AUTO_MAX_LINEAR:-0.22}"
export ROVER_AUTO_BURST_MAX="${ROVER_AUTO_BURST_MAX:-0.32}"
export ROVER_AUTO_BURST_PROB="${ROVER_AUTO_BURST_PROB:-0.005}"
export ROVER_LINEAR_SIGN="${ROVER_LINEAR_SIGN:-1}"
export ROVER_SERVO_SOURCE="${ROVER_SERVO_SOURCE:-esp}"
export ROVER_IR_SOURCE="${ROVER_IR_SOURCE:-esp}"
export ROVER_IR_FRONT_ESCAPE="${ROVER_IR_FRONT_ESCAPE:-0}"
export ROVER_IR_FRONT_STOP="${ROVER_IR_FRONT_STOP:-1}"
export ROVER_TWIST_SWAP="${ROVER_TWIST_SWAP:-0}"

echo "=== LEGO Rover standby ==="
echo "  Agent: rover-agent.service (micro-ROS, always on)"
echo "  Start/stop: ESP GPIO14 (top-right KEY) via /rover/button"
echo "  Speaker: GPIO ${ROVER_SPEAKER_GPIO} (physical pin 11)"
echo "  LED ring: GPIO ${ROVER_RING_GPIO} (physical pin 12, 8× WS2812)"
echo ""

rover_source_ros

BRIDGE_PID=""
start_bridge() {
  export ROVER_BRIDGE_EXTERNAL=1
  python3 "$DIR/rover_bridge_daemon.py" &
  BRIDGE_PID=$!
}
stop_bridge() {
  if [[ -n "${BRIDGE_PID:-}" ]]; then
    kill "$BRIDGE_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
    BRIDGE_PID=""
  fi
  python3 -c "import sys; sys.path.insert(0, '$DIR'); from rover_esp_button import set_session_active; set_session_active(False)" 2>/dev/null || true
}
trap stop_bridge EXIT INT TERM

start_bridge

ring_set() {
  python3 -c "import sys; sys.path.insert(0, '$DIR'); from rover_ring import set_mode; set_mode('$1')" 2>/dev/null || true
}

READY_FLAG=/tmp/rover_ready_notified
rm -f "$READY_FLAG"

_ready_notify() {
  if [[ -f "$READY_FLAG" ]]; then
    return 0
  fi
  touch "$READY_FLAG"
  if ! python3 "$DIR/rover_speaker.py" ready; then
    echo "WARN: ready chime failed — check GPIO 17 buzzer (python3 ~/lego-rover-ros2/rover_speaker.py ready)" >&2
  fi
  ring_set ready
  sleep 0.3
  ring_set standby
  echo "Ready — hold Go=mode, double Go=power, tap Go=start."
}

ring_set standby

_esp_wait_secs="${ROVER_ESP_READY_SECS:-180}"
if rover_wait_for_esp "$_esp_wait_secs"; then
  _ready_notify
else
  echo "ESP not linked after ${_esp_wait_secs}s — will chime when it connects..."
  (
    rover_source_ros
    while true; do
      if rover_esp_linked; then
        _ready_notify
        break
      fi
      sleep 3
    done
  ) &
fi
echo ""

while true; do
  if ! python3 "$DIR/wait_for_button.py"; then
    rc=$?
    echo "Standby wait ended (exit $rc) — not starting autonomous."
    sleep 0.5
    continue
  fi

  sleep 0.05

  MODE="$(python3 -c "import sys; sys.path.insert(0, '$DIR'); from rover_esp_button import read_drive_mode, drive_mode_name; print(drive_mode_name(read_drive_mode()))")"
  echo ""
  echo ">>> Starting autonomous ${MODE} <<<"
  echo ""

  export ROVER_QUICK_START=1
  case "$MODE" in
    wall)
      if bash "$DIR/go_wall.sh"; then
        echo ""
        echo "Rover session ended."
      else
        echo ""
        echo "Rover session exited with error (see above)."
      fi
      ;;
    *)
      if bash "$DIR/go.sh"; then
        echo ""
        echo "Rover session ended."
      else
        echo ""
        echo "Rover session exited with error (see above)."
      fi
      ;;
  esac

  rover_source_ros
  rover_stop_motors 2>/dev/null || true
  rover_ir_close 2>/dev/null || true

  echo ""
  echo "Standby — hold Go=mode, double Go=power, tap Go=start."
  echo ""
done
